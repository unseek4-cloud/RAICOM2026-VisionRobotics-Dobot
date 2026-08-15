# -*- coding: utf-8 -*-
"""在比赛电脑本地训练任务二/任务三 YOLO-OBB 旋转框模型。

脚本不会自动联网下载权重。比赛前请把可离线使用的基础权重放到本地，
现场完成拍照、标注和 data.yaml 后再运行本工具。训练成功时把 best.pt 复制到
主程序约定的 ``models/task2.pt`` 或 ``models/task3.pt``。
"""

from __future__ import annotations

import argparse
import errno
import math
import os
import shutil
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
# 训练 checkpoint 不再写进 VS Code/Git 正在监视的项目目录。按比赛电脑约定，
# 默认使用 D 盘独立目录；正式模型仍会复制回 PROJECT_ROOT/models。
DEFAULT_RUNS_DIR = Path("D:/RAICOM-YOLO-Runs")
_CHECKPOINT_RETRY_COUNT = 12
_TRANSIENT_ERRNOS = {errno.EACCES, errno.EBUSY, errno.EINVAL, errno.EPERM}
_TRANSIENT_WINERRORS = {5, 32, 33, 87}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="睿抗任务二/三 YOLO 离线训练工具")
    parser.add_argument("--task", choices=("task2", "task3"), required=True, help="模型用途")
    parser.add_argument("--data", type=Path, required=True, help="Ultralytics 数据集 data.yaml")
    parser.add_argument(
        "--base",
        type=Path,
        default=None,
        help="新训练所用本地基础权重；使用 --resume 时可省略",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        default=None,
        help="从 last.pt 继续训练，并把后续 checkpoint 迁移到新的 D 盘训练目录",
    )
    parser.add_argument("--epochs", type=int, default=80, help="训练轮数，默认 80")
    parser.add_argument("--imgsz", type=int, default=640, help="训练图像尺寸，默认 640")
    parser.add_argument("--batch", type=int, default=8, help="批大小；显存不足时减小")
    parser.add_argument("--device", default="0", help="GPU 编号，例如 0；CPU 填 cpu")
    parser.add_argument("--workers", type=int, default=4, help="数据加载线程数")
    parser.add_argument(
        "--project",
        type=Path,
        default=DEFAULT_RUNS_DIR,
        help=f"训练过程目录；默认 {DEFAULT_RUNS_DIR}",
    )
    parser.add_argument("--name", default=None, help="训练名称；默认 task2_field/task3_field")
    parser.add_argument("--patience", type=int, default=20, help="早停等待轮数")
    parser.add_argument(
        "--skip-test",
        action="store_true",
        help="训练后不使用 data.yaml 的 test 集评估（现场应尽量保留自动测试）",
    )
    return parser


def _existing_file(path: Path, label: str) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"{label}不存在：{resolved}")
    return resolved


def _is_transient_checkpoint_error(exc: OSError) -> bool:
    return exc.errno in _TRANSIENT_ERRNOS or getattr(exc, "winerror", None) in (
        _TRANSIENT_WINERRORS
    )


def _replace_checkpoint_with_retry(source: Path, destination: Path) -> None:
    """在 Windows 临时占用解除后原子替换 checkpoint。"""

    last_error: OSError | None = None
    for attempt in range(1, _CHECKPOINT_RETRY_COUNT + 1):
        try:
            source.replace(destination)
            return
        except OSError as exc:
            if not _is_transient_checkpoint_error(exc):
                raise
            last_error = exc
            delay = min(2.0, 0.25 * attempt)
            print(
                f"[权重保存重试] {destination.name} 暂时被占用或文件系统拒绝覆盖，"
                f"{delay:.2f} 秒后重试（{attempt}/{_CHECKPOINT_RETRY_COUNT}）：{exc}",
                file=sys.stderr,
                flush=True,
            )
            time.sleep(delay)
    assert last_error is not None
    raise last_error


def _build_reliable_detection_trainer(base_trainer: type[Any]) -> type[Any]:
    """为 Ultralytics Trainer 增加可靠 checkpoint 保存（名称保留以兼容旧调用）。"""

    class ReliableDetectionTrainer(base_trainer):
        def save_model(self) -> bool:
            token = f"{os.getpid()}-{uuid.uuid4().hex}"
            original_last, original_best = self.last, self.best
            temporary_last = self.wdir / f".last-{token}.tmp"
            temporary_best = self.wdir / f".best-{token}.tmp"
            try:
                self.last, self.best = temporary_last, temporary_best
                result = super().save_model()
                self.last, self.best = original_last, original_best
                if not temporary_last.is_file():
                    raise OSError(f"Ultralytics 未生成临时 checkpoint：{temporary_last}")
                _replace_checkpoint_with_retry(temporary_last, original_last)
                if temporary_best.is_file():
                    _replace_checkpoint_with_retry(temporary_best, original_best)
                return bool(result)
            finally:
                self.last, self.best = original_last, original_best
                # 替换成功后源文件已不存在；异常退出时清理残留临时文件。
                for temporary in (temporary_last, temporary_best):
                    try:
                        temporary.unlink(missing_ok=True)
                    except OSError:
                        pass

    return ReliableDetectionTrainer


def _validate_obb_labels(data_yaml: Path) -> None:
    """拒绝把旧 5 列水平框误用于 OBB 训练。"""

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("校验 OBB 数据集需要 PyYAML") from exc
    payload = yaml.safe_load(data_yaml.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"数据集配置不是 YAML 对象：{data_yaml}")
    root_value = payload.get("path", data_yaml.parent)
    dataset_root = Path(str(root_value)).expanduser()
    if not dataset_root.is_absolute():
        dataset_root = (data_yaml.parent / dataset_root).resolve()
    label_root = dataset_root / "labels"
    label_files = sorted(label_root.rglob("*.txt")) if label_root.is_dir() else []
    if not label_files:
        raise ValueError(f"没有找到 OBB 标签：{label_root}")
    for label_file in label_files:
        for line_number, raw_line in enumerate(
            label_file.read_text(encoding="utf-8").splitlines(), start=1
        ):
            fields = raw_line.split()
            if not fields:
                continue
            if len(fields) != 9:
                raise ValueError(
                    f"{label_file}:{line_number} 有 {len(fields)} 列；OBB 必须是 "
                    "class x1 y1 x2 y2 x3 y3 x4 y4 共 9 列"
                )
            try:
                class_id = int(fields[0])
                coordinates = [float(value) for value in fields[1:]]
            except ValueError as exc:
                raise ValueError(f"{label_file}:{line_number} 含非数字 OBB 字段") from exc
            if class_id < 0 or not all(
                math.isfinite(value) and 0.0 <= value <= 1.0 for value in coordinates
            ):
                raise ValueError(
                    f"{label_file}:{line_number} 类别必须非负，归一化四点坐标必须在 [0,1]"
                )


def _unique_resume_dir(project_dir: Path, task: str, requested_name: str | None) -> Path:
    base_name = requested_name or f"{task}_resume_{datetime.now():%Y%m%d_%H%M%S}"
    candidate = project_dir / base_name
    suffix = 2
    while candidate.exists():
        candidate = project_dir / f"{base_name}_{suffix}"
        suffix += 1
    return candidate


def main() -> int:
    args = build_parser().parse_args()
    if args.epochs < 1 or args.imgsz < 160 or args.batch < 1 or args.workers < 0:
        print("[参数错误] epochs/batch 必须为正数，imgsz>=160，workers>=0", file=sys.stderr)
        return 2

    try:
        data_yaml = _existing_file(args.data, "数据集配置")
        _validate_obb_labels(data_yaml)
        resume_weight = (
            _existing_file(args.resume, "恢复训练权重") if args.resume is not None else None
        )
        if resume_weight is None:
            if args.base is None:
                print("[参数错误] 新训练必须提供 --base；恢复训练请提供 --resume", file=sys.stderr)
                return 2
            base_weight = _existing_file(args.base, "本地基础权重")
        else:
            base_weight = None
        from ultralytics import YOLO
        from ultralytics.models.yolo.obb import OBBTrainer
    except (FileNotFoundError, ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"[准备失败] {exc}", file=sys.stderr)
        return 3

    run_name = args.name or f"{args.task}_field"
    project_dir = args.project.expanduser().resolve()
    try:
        project_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f"[准备失败] 无法创建 D 盘训练目录 {project_dir}：{exc}", file=sys.stderr)
        return 3

    reliable_trainer = _build_reliable_detection_trainer(OBBTrainer)
    if resume_weight is not None:
        resume_save_dir = _unique_resume_dir(project_dir, args.task, args.name)
        run_name = resume_save_dir.name
        print(f"继续训练 {args.task}：checkpoint={resume_weight}")
        print(f"后续 checkpoint 迁移到 D 盘独立目录：{resume_save_dir}")
        model = YOLO(str(resume_weight))
        if str(getattr(model, "task", "")).lower() != "obb":
            print("[准备失败] --resume 必须指向 OBB checkpoint", file=sys.stderr)
            return 3
        model.train(
            resume=str(resume_weight),
            save_dir=str(resume_save_dir),
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            workers=args.workers,
            patience=args.patience,
            verbose=True,
            trainer=reliable_trainer,
        )
    else:
        print(f"开始训练 {args.task}：data={data_yaml}")
        print(f"checkpoint 输出目录：{project_dir}")
        print("训练时保持离线；若程序尝试下载，说明 --base 指向的本地权重不完整。")
        assert base_weight is not None
        model = YOLO(str(base_weight))
        if str(getattr(model, "task", "")).lower() != "obb":
            print(
                "[准备失败] --base 必须使用本地 *-obb.pt 权重，普通 detect 权重没有角度头",
                file=sys.stderr,
            )
            return 3
        model.train(
            data=str(data_yaml),
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            workers=args.workers,
            project=str(project_dir),
            name=run_name,
            patience=args.patience,
            exist_ok=False,
            verbose=True,
            trainer=reliable_trainer,
        )

    # save_dir 由 Trainer 给出，避免猜测 Ultralytics 自动编号后的目录名。
    trainer = getattr(model, "trainer", None)
    raw_save_dir = getattr(trainer, "save_dir", None)
    if raw_save_dir is None:
        print("[训练异常] Ultralytics 未返回本次训练目录", file=sys.stderr)
        return 4
    save_dir = Path(str(raw_save_dir)).resolve()
    best_weight = save_dir / "weights" / "best.pt"
    if not best_weight.is_file():
        print(f"[训练异常] 未找到最佳权重：{best_weight}", file=sys.stderr)
        return 4

    destination = PROJECT_ROOT / "models" / f"{args.task}.pt"
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_weight, destination)
    print(f"训练完成，已复制正式模型：{destination}")
    if not args.skip_test:
        print("开始使用 data.yaml 中的 test 集评估最佳权重……")
        try:
            test_model = YOLO(str(destination))
            test_model.val(
                data=str(data_yaml),
                split="test",
                imgsz=args.imgsz,
                batch=args.batch,
                device=args.device,
                workers=args.workers,
                project=str(project_dir),
                name=f"{run_name}_test",
                exist_ok=False,
                verbose=True,
                plots=True,
            )
        except Exception as exc:
            print(
                f"[测试失败] 正式模型已经生成在 {destination}，但 test 集评估失败：{exc}",
                file=sys.stderr,
            )
            return 5
        print("test 集评估完成；指标和图表已保存到本次测试目录。")
    else:
        print("已按 --skip-test 跳过独立测试集评估。")
    print("请再运行模拟/离线图片验证，然后按 README 做真机低速验证。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
