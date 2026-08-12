# -*- coding: utf-8 -*-
"""在比赛电脑本地训练任务二/任务三 YOLO 模型。

脚本不会自动联网下载权重。比赛前请把可离线使用的基础权重放到本地，
现场完成拍照、标注和 data.yaml 后再运行本工具。训练成功时把 best.pt 复制到
主程序约定的 ``models/task2.pt`` 或 ``models/task3.pt``。
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="睿抗任务二/三 YOLO 离线训练工具")
    parser.add_argument("--task", choices=("task2", "task3"), required=True, help="模型用途")
    parser.add_argument("--data", type=Path, required=True, help="Ultralytics 数据集 data.yaml")
    parser.add_argument(
        "--base",
        type=Path,
        required=True,
        help="本地基础权重，例如 tools/offline_weights/yolo11n.pt；不会自动下载",
    )
    parser.add_argument("--epochs", type=int, default=80, help="训练轮数，默认 80")
    parser.add_argument("--imgsz", type=int, default=640, help="训练图像尺寸，默认 640")
    parser.add_argument("--batch", type=int, default=8, help="批大小；显存不足时减小")
    parser.add_argument("--device", default="0", help="GPU 编号，例如 0；CPU 填 cpu")
    parser.add_argument("--workers", type=int, default=4, help="数据加载线程数")
    parser.add_argument(
        "--project",
        type=Path,
        default=PROJECT_ROOT / "runs",
        help="训练过程目录",
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


def main() -> int:
    args = build_parser().parse_args()
    if args.epochs < 1 or args.imgsz < 160 or args.batch < 1 or args.workers < 0:
        print("[参数错误] epochs/batch 必须为正数，imgsz>=160，workers>=0", file=sys.stderr)
        return 2

    try:
        data_yaml = _existing_file(args.data, "数据集配置")
        base_weight = _existing_file(args.base, "本地基础权重")
        from ultralytics import YOLO
    except (FileNotFoundError, ImportError) as exc:
        print(f"[准备失败] {exc}", file=sys.stderr)
        return 3

    run_name = args.name or f"{args.task}_field"
    project_dir = args.project.expanduser().resolve()
    print(f"开始训练 {args.task}：data={data_yaml}")
    print("训练时保持离线；若程序尝试下载，说明 --base 指向的本地权重不完整。")

    model = YOLO(str(base_weight))
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
