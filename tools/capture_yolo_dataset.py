# -*- coding: utf-8 -*-
"""使用 Intel RealSense D435 采集 YOLO 图片并划分数据集。

界面固定采集 1280×720、30 FPS 的 RGB 彩色流。空格键或“拍照”按钮把 JPG
原图保存到当前任务的 ``datasets/<task>/photo``。完成 YOLO 标注后，把与图片
同名的 ``.txt`` 标签放在 ``photo`` 中，再用“一键划分”按 70%/20%/10% 复制到
train/val/test。原始照片和原始标签不会被移动或删除。
"""

from __future__ import annotations

import argparse
import json
import random
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATASETS_ROOT = PROJECT_ROOT / "datasets"
TASKS = ("task2", "task3")
SPLITS = ("train", "val", "test")
SPLIT_RATIOS = (0.70, 0.20, 0.10)
CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
CAPTURE_FPS = 30
MANIFEST_NAME = "split_manifest.json"


class DatasetSplitError(RuntimeError):
    """数据集目录、文件冲突或划分过程错误。"""


@dataclass(frozen=True, slots=True)
class PhotoInventory:
    images: tuple[Path, ...]
    missing_labels: tuple[Path, ...]


@dataclass(frozen=True, slots=True)
class SplitSummary:
    task: str
    total: int
    train: int
    val: int
    test: int
    labels: int
    missing_labels: int
    manifest: Path


def _validate_task_root(task_root: Path) -> tuple[Path, str]:
    root = task_root.expanduser().resolve()
    task = root.name.lower()
    if task not in TASKS:
        raise DatasetSplitError(f"任务目录必须以 task2 或 task3 命名：{root}")
    photo_dir = root / "photo"
    if not photo_dir.is_dir():
        raise DatasetSplitError(f"照片目录不存在：{photo_dir}")
    return root, task


def inspect_photos(task_root: Path) -> PhotoInventory:
    """读取 photo 下的 JPG，并检查旁边是否有同名 YOLO 标签。"""

    root, _ = _validate_task_root(task_root)
    images = tuple(
        sorted(
            (
                path
                for path in (root / "photo").rglob("*")
                if path.is_file() and path.suffix.lower() == ".jpg"
            ),
            key=lambda path: path.relative_to(root).as_posix().lower(),
        )
    )
    names: dict[str, Path] = {}
    for image in images:
        key = image.name.lower()
        if key in names:
            raise DatasetSplitError(
                "photo 的不同子目录中存在同名图片，划分后的平铺目录会冲突："
                f"{names[key]} 和 {image}"
            )
        names[key] = image
    missing = tuple(image for image in images if not image.with_suffix(".txt").is_file())
    return PhotoInventory(images=images, missing_labels=missing)


def _split_counts(total: int) -> dict[str, int]:
    """使用最大余数法得到总数严格相等的 70/20/10 整数划分。"""

    if total < 0:
        raise ValueError("total 不能为负数")
    raw = [total * ratio for ratio in SPLIT_RATIOS]
    counts = [int(value) for value in raw]
    remainder = total - sum(counts)
    order = sorted(
        range(len(SPLITS)),
        key=lambda index: (raw[index] - counts[index], -index),
        reverse=True,
    )
    for index in order[:remainder]:
        counts[index] += 1
    return dict(zip(SPLITS, counts))


def _load_previous_outputs(task_root: Path) -> set[str]:
    manifest = task_root / MANIFEST_NAME
    if not manifest.is_file():
        return set()
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetSplitError(f"无法读取上次划分清单 {manifest}：{exc}") from exc
    outputs = payload.get("outputs", [])
    if not isinstance(outputs, list) or not all(isinstance(item, str) for item in outputs):
        raise DatasetSplitError(f"上次划分清单格式错误：{manifest}")
    return set(outputs)


def _managed_output_path(task_root: Path, relative: str) -> Path:
    candidate = (task_root / relative).resolve()
    allowed_roots = tuple(
        (task_root / kind / split).resolve()
        for kind in ("images", "labels")
        for split in SPLITS
    )
    if not any(candidate.is_relative_to(root) for root in allowed_roots):
        raise DatasetSplitError(f"划分清单含越界路径，拒绝处理：{relative}")
    return candidate


def split_dataset(task_root: Path, *, seed: int = 2026) -> SplitSummary:
    """复制原始图片和同名标签，重新生成当前任务的 70/20/10 划分。"""

    root, task = _validate_task_root(task_root)
    inventory = inspect_photos(root)
    if not inventory.images:
        raise DatasetSplitError(f"没有可划分的 JPG 图片：{root / 'photo'}")

    images = list(inventory.images)
    random.Random(seed).shuffle(images)
    counts = _split_counts(len(images))
    assignments: list[tuple[str, Path]] = []
    offset = 0
    for split in SPLITS:
        end = offset + counts[split]
        assignments.extend((split, image) for image in images[offset:end])
        offset = end

    previous_outputs = _load_previous_outputs(root)
    unmanaged_outputs: list[str] = []
    for kind, suffixes in (
        ("images", {".jpg", ".jpeg", ".png", ".bmp", ".webp"}),
        ("labels", {".txt"}),
    ):
        for split in SPLITS:
            output_dir = root / kind / split
            if not output_dir.is_dir():
                continue
            for output in output_dir.rglob("*"):
                relative = output.relative_to(root).as_posix()
                if (
                    output.is_file()
                    and output.suffix.lower() in suffixes
                    and relative not in previous_outputs
                ):
                    unmanaged_outputs.append(relative)
    if unmanaged_outputs:
        preview = "\n".join(f"  - {item}" for item in sorted(unmanaged_outputs)[:10])
        raise DatasetSplitError(
            "train/val/test 中存在非本工具管理的数据，拒绝清空或混合划分：\n"
            f"{preview}\n请先备份并手工处理这些文件。"
        )

    next_outputs: set[str] = set()
    for split, image in assignments:
        image_relative = (Path("images") / split / image.name).as_posix()
        next_outputs.add(image_relative)
        label = image.with_suffix(".txt")
        if label.is_file():
            next_outputs.add((Path("labels") / split / label.name).as_posix())

    # 上次清单明确记录的文件是本工具的派生副本，可以安全重建。
    for relative in sorted(previous_outputs):
        destination = _managed_output_path(root, relative)
        if destination.is_file():
            destination.unlink()

    copied_labels = 0
    records: list[dict[str, str | None]] = []
    try:
        for kind in ("images", "labels"):
            for split in SPLITS:
                (root / kind / split).mkdir(parents=True, exist_ok=True)
        for split, image in assignments:
            image_destination = root / "images" / split / image.name
            image_destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(image, image_destination)

            label = image.with_suffix(".txt")
            label_relative: str | None = None
            if label.is_file():
                label_destination = root / "labels" / split / label.name
                label_destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(label, label_destination)
                label_relative = label_destination.relative_to(root).as_posix()
                copied_labels += 1
            records.append(
                {
                    "source": image.relative_to(root).as_posix(),
                    "split": split,
                    "image": image_destination.relative_to(root).as_posix(),
                    "label": label_relative,
                }
            )
    except OSError as exc:
        # 所有 next_outputs 在复制前均已确认属于上次受管文件或原本不存在，
        # 因此失败时可删除这些不完整派生副本；photo 原始文件始终不受影响。
        for relative in sorted(next_outputs):
            destination = _managed_output_path(root, relative)
            if destination.is_file():
                destination.unlink()
        raise DatasetSplitError(f"复制数据集文件失败：{exc}") from exc

    manifest = root / MANIFEST_NAME
    payload: dict[str, Any] = {
        "version": 1,
        "task": task,
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "seed": seed,
        "ratios": {"train": 0.70, "val": 0.20, "test": 0.10},
        "counts": counts,
        "outputs": sorted(next_outputs),
        "records": records,
    }
    temporary_manifest = manifest.with_suffix(".json.tmp")
    temporary_manifest.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary_manifest.replace(manifest)

    return SplitSummary(
        task=task,
        total=len(images),
        train=counts["train"],
        val=counts["val"],
        test=counts["test"],
        labels=copied_labels,
        missing_labels=len(inventory.missing_labels),
        manifest=manifest,
    )


_GUI_IMPORT_ERROR: ImportError | None = None
try:
    import cv2
    import numpy as np
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError as exc:  # 允许 --split-only 在没有 GUI 依赖时给出清晰提示。
    _GUI_IMPORT_ERROR = exc
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]
    QtCore = None  # type: ignore[assignment]
    QtGui = None  # type: ignore[assignment]
    QtWidgets = None  # type: ignore[assignment]


if QtCore is not None:

    class CameraWorker(QtCore.QThread):
        frame_ready = QtCore.pyqtSignal(object)
        camera_ready = QtCore.pyqtSignal(str, bool)
        camera_error = QtCore.pyqtSignal(str)

        def __init__(self, serial: str | None = None, parent: Any = None) -> None:
            super().__init__(parent)
            self.serial = serial

        def run(self) -> None:
            pipeline = None
            started = False
            try:
                import pyrealsense2 as rs

                pipeline = rs.pipeline()
                config = rs.config()
                if self.serial:
                    config.enable_device(self.serial)
                config.enable_stream(
                    rs.stream.color,
                    CAPTURE_WIDTH,
                    CAPTURE_HEIGHT,
                    rs.format.bgr8,
                    CAPTURE_FPS,
                )
                profile = pipeline.start(config)
                started = True
                device = profile.get_device()
                try:
                    device_name = str(device.get_info(rs.camera_info.name))
                except Exception:
                    device_name = "Intel RealSense"
                is_d435 = "D435" in device_name.upper()

                # 丢弃自动曝光尚未稳定的前若干帧。
                for _ in range(20):
                    if self.isInterruptionRequested():
                        return
                    pipeline.wait_for_frames(5000)
                self.camera_ready.emit(device_name, is_d435)

                while not self.isInterruptionRequested():
                    try:
                        frames = pipeline.wait_for_frames(1000)
                    except RuntimeError:
                        if self.isInterruptionRequested():
                            break
                        raise
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue
                    frame = np.asanyarray(color_frame.get_data()).copy()
                    if frame.shape[:2] != (CAPTURE_HEIGHT, CAPTURE_WIDTH):
                        raise RuntimeError(
                            f"彩色帧分辨率异常：{frame.shape[1]}×{frame.shape[0]}，"
                            f"要求 {CAPTURE_WIDTH}×{CAPTURE_HEIGHT}"
                        )
                    self.frame_ready.emit(frame)
            except Exception as exc:
                self.camera_error.emit(
                    "D435 RGB 相机启动或取帧失败。请检查 USB 3.x 连接、相机占用、"
                    f"pyrealsense2 和 1280×720@30 支持情况。\n\n原始错误：{exc}"
                )
            finally:
                if started and pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass


    class SplitWorker(QtCore.QThread):
        completed = QtCore.pyqtSignal(object)
        failed = QtCore.pyqtSignal(str)

        def __init__(self, task_root: Path, seed: int, parent: Any = None) -> None:
            super().__init__(parent)
            self.task_root = task_root
            self.seed = seed

        def run(self) -> None:
            try:
                self.completed.emit(split_dataset(self.task_root, seed=self.seed))
            except Exception as exc:
                self.failed.emit(str(exc))


    class CaptureWindow(QtWidgets.QMainWindow):
        def __init__(self, *, initial_task: str, serial: str | None, seed: int) -> None:
            super().__init__()
            self.serial = serial
            self.seed = seed
            self.camera_worker: CameraWorker | None = None
            self.split_worker: SplitWorker | None = None
            self.latest_frame: Any = None
            self._build_ui(initial_task)
            self._apply_style()
            self._wire_events()
            self._update_task_details()
            QtCore.QTimer.singleShot(0, self.start_camera)

        def _build_ui(self, initial_task: str) -> None:
            self.setWindowTitle("D435 · YOLO 数据集拍照工具")
            self.resize(1180, 820)
            self.setMinimumSize(900, 680)
            self.setFont(QtGui.QFont("Microsoft YaHei UI", 10))

            central = QtWidgets.QWidget(self)
            self.setCentralWidget(central)
            root = QtWidgets.QVBoxLayout(central)
            root.setContentsMargins(14, 12, 14, 12)
            root.setSpacing(10)

            title = QtWidgets.QLabel("RealSense D435 · YOLO 数据采集")
            title.setObjectName("title")
            subtitle = QtWidgets.QLabel(
                "RGB 1280×720 @ 30 FPS　|　空格键拍照　|　原图固定保存为 JPG"
            )
            subtitle.setObjectName("muted")
            root.addWidget(title)
            root.addWidget(subtitle)

            task_box = QtWidgets.QGroupBox("选择拍照任务")
            task_layout = QtWidgets.QHBoxLayout(task_box)
            self.task2_radio = QtWidgets.QRadioButton("Task 2（形状/颜色）")
            self.task3_radio = QtWidgets.QRadioButton("Task 3（顶部图案）")
            (self.task2_radio if initial_task == "task2" else self.task3_radio).setChecked(True)
            task_layout.addWidget(self.task2_radio)
            task_layout.addWidget(self.task3_radio)
            task_layout.addStretch(1)
            root.addWidget(task_box)

            self.video = QtWidgets.QLabel("正在连接 D435……")
            self.video.setObjectName("video")
            self.video.setAlignment(QtCore.Qt.AlignCenter)
            self.video.setMinimumSize(640, 360)
            root.addWidget(self.video, 1)

            controls = QtWidgets.QHBoxLayout()
            self.start_button = QtWidgets.QPushButton("启动相机")
            self.stop_button = QtWidgets.QPushButton("停止相机")
            self.capture_button = QtWidgets.QPushButton("拍照（空格）")
            self.capture_button.setObjectName("captureButton")
            self.split_button = QtWidgets.QPushButton("一键划分当前任务 70/20/10")
            self.split_button.setObjectName("splitButton")
            self.stop_button.setEnabled(False)
            self.capture_button.setEnabled(False)
            controls.addWidget(self.start_button)
            controls.addWidget(self.stop_button)
            controls.addWidget(self.capture_button)
            controls.addStretch(1)
            controls.addWidget(self.split_button)
            root.addLayout(controls)

            self.path_label = QtWidgets.QLabel()
            self.path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.count_label = QtWidgets.QLabel()
            self.status_label = QtWidgets.QLabel("准备就绪")
            self.status_label.setObjectName("status")
            root.addWidget(self.path_label)
            root.addWidget(self.count_label)
            root.addWidget(self.status_label)

            self.space_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self)
            self.space_shortcut.setContext(QtCore.Qt.WindowShortcut)

        def _apply_style(self) -> None:
            self.setStyleSheet(
                """
                QMainWindow, QWidget { background:#f3f6fa; color:#1c2733; }
                QLabel#title { font-size:24px; font-weight:700; color:#102a43; }
                QLabel#muted { color:#6b7c8f; }
                QLabel#video { background:#0b1118; color:#8fa3b7; border-radius:7px; }
                QLabel#status {
                    background:white; border:1px solid #d9e2ec;
                    border-radius:6px; padding:7px;
                }
                QGroupBox {
                    background:white; border:1px solid #d9e2ec; border-radius:7px;
                    margin-top:9px; padding-top:8px; font-weight:700;
                }
                QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }
                QPushButton {
                    background:white; border:1px solid #bcccdc; border-radius:6px;
                    padding:9px 13px; font-weight:600;
                }
                QPushButton:hover { background:#eaf2fb; border-color:#5b9bd5; }
                QPushButton:disabled { color:#9aa8b5; background:#eef2f5; }
                QPushButton#captureButton { background:#087f5b; color:white; border-color:#087f5b; }
                QPushButton#splitButton { background:#0b63ce; color:white; border-color:#0b63ce; }
                """
            )

        def _wire_events(self) -> None:
            self.start_button.clicked.connect(self.start_camera)
            self.stop_button.clicked.connect(self.stop_camera)
            self.capture_button.clicked.connect(self.capture_photo)
            self.split_button.clicked.connect(self.split_current_task)
            self.space_shortcut.activated.connect(self.capture_photo)
            self.task2_radio.toggled.connect(self._update_task_details)
            self.task3_radio.toggled.connect(self._update_task_details)

        def current_task(self) -> str:
            return "task2" if self.task2_radio.isChecked() else "task3"

        def current_root(self) -> Path:
            return DATASETS_ROOT / self.current_task()

        def _update_task_details(self) -> None:
            root = self.current_root()
            photo_dir = root / "photo"
            photo_dir.mkdir(parents=True, exist_ok=True)
            try:
                inventory = inspect_photos(root)
                photo_count = len(inventory.images)
                label_count = photo_count - len(inventory.missing_labels)
            except DatasetSplitError:
                photo_count = 0
                label_count = 0
            self.path_label.setText(f"当前保存路径：{photo_dir}")
            self.count_label.setText(
                f"当前任务原图：{photo_count} 张　|　同名标签：{label_count} 个"
            )

        @QtCore.pyqtSlot()
        def start_camera(self) -> None:
            if self.camera_worker is not None and self.camera_worker.isRunning():
                return
            self.latest_frame = None
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.capture_button.setEnabled(False)
            self.status_label.setText("正在启动 D435 RGB 1280×720@30……")
            worker = CameraWorker(self.serial, self)
            worker.frame_ready.connect(self._show_frame)
            worker.camera_ready.connect(self._camera_ready)
            worker.camera_error.connect(self._camera_error)
            worker.finished.connect(self._camera_finished)
            self.camera_worker = worker
            worker.start()

        @QtCore.pyqtSlot()
        def stop_camera(self) -> None:
            if self.camera_worker is not None and self.camera_worker.isRunning():
                self.status_label.setText("正在停止相机……")
                self.camera_worker.requestInterruption()

        @QtCore.pyqtSlot(str, bool)
        def _camera_ready(self, device_name: str, is_d435: bool) -> None:
            suffix = "" if is_d435 else "（警告：设备名称不是 D435，请核对硬件）"
            self.status_label.setText(
                f"相机已连接：{device_name}，RGB {CAPTURE_WIDTH}×{CAPTURE_HEIGHT}@{CAPTURE_FPS}{suffix}"
            )

        @QtCore.pyqtSlot(str)
        def _camera_error(self, message: str) -> None:
            QtWidgets.QMessageBox.critical(self, "相机错误", message)

        @QtCore.pyqtSlot()
        def _camera_finished(self) -> None:
            self.latest_frame = None
            self.camera_worker = None
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.capture_button.setEnabled(False)
            self.video.clear()
            self.video.setText("相机已停止")
            if not self.status_label.text().startswith("拍照完成"):
                self.status_label.setText("相机已停止")

        @QtCore.pyqtSlot(object)
        def _show_frame(self, bgr: object) -> None:
            if not isinstance(bgr, np.ndarray) or bgr.shape != (
                CAPTURE_HEIGHT,
                CAPTURE_WIDTH,
                3,
            ):
                return
            self.latest_frame = bgr.copy()
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            image = QtGui.QImage(
                rgb.data,
                rgb.shape[1],
                rgb.shape[0],
                rgb.strides[0],
                QtGui.QImage.Format_RGB888,
            ).copy()
            pixmap = QtGui.QPixmap.fromImage(image).scaled(
                self.video.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
            self.video.setPixmap(pixmap)
            self.capture_button.setEnabled(True)

        @QtCore.pyqtSlot()
        def capture_photo(self) -> None:
            if self.latest_frame is None or not self.capture_button.isEnabled():
                self.status_label.setText("尚未取得 D435 彩色帧，暂时不能拍照")
                return
            task = self.current_task()
            photo_dir = self.current_root() / "photo"
            photo_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            destination = photo_dir / f"{task}_{stamp}.jpg"
            sequence = 1
            while destination.exists():
                destination = photo_dir / f"{task}_{stamp}_{sequence:02d}.jpg"
                sequence += 1
            try:
                ok, encoded = cv2.imencode(
                    ".jpg", self.latest_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95]
                )
                if not ok:
                    raise OSError("OpenCV JPG 编码失败")
                encoded.tofile(str(destination))
            except Exception as exc:
                QtWidgets.QMessageBox.critical(self, "保存失败", str(exc))
                return
            self.status_label.setText(f"拍照完成：{destination.name}")
            self._update_task_details()

        @QtCore.pyqtSlot()
        def split_current_task(self) -> None:
            if self.split_worker is not None and self.split_worker.isRunning():
                QtWidgets.QMessageBox.information(self, "正在划分", "请等待当前划分完成。")
                return
            root = self.current_root()
            try:
                inventory = inspect_photos(root)
            except DatasetSplitError as exc:
                QtWidgets.QMessageBox.critical(self, "无法划分", str(exc))
                return
            if not inventory.images:
                QtWidgets.QMessageBox.warning(self, "没有照片", f"请先拍照：{root / 'photo'}")
                return

            warnings: list[str] = []
            if len(inventory.images) < 10:
                warnings.append(
                    f"当前只有 {len(inventory.images)} 张图片，整数划分可能无法接近 70/20/10。"
                )
            if inventory.missing_labels:
                warnings.append(
                    f"有 {len(inventory.missing_labels)} 张图片没有同名 .txt 标签；"
                    "YOLO 会把无标签图片视为背景图。"
                )
            if warnings:
                answer = QtWidgets.QMessageBox.question(
                    self,
                    "划分前确认",
                    "\n\n".join(warnings) + "\n\n仍要继续划分吗？",
                    QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                    QtWidgets.QMessageBox.No,
                )
                if answer != QtWidgets.QMessageBox.Yes:
                    return

            self.split_button.setEnabled(False)
            self.status_label.setText(f"正在划分 {self.current_task()}，请稍候……")
            worker = SplitWorker(root, self.seed, self)
            worker.completed.connect(self._split_completed)
            worker.failed.connect(self._split_failed)
            worker.finished.connect(self._split_finished)
            self.split_worker = worker
            worker.start()

        @QtCore.pyqtSlot(object)
        def _split_completed(self, summary: SplitSummary) -> None:
            text = (
                f"{summary.task} 划分完成：train={summary.train}，val={summary.val}，"
                f"test={summary.test}；标签 {summary.labels}/{summary.total}。"
            )
            self.status_label.setText(text)
            QtWidgets.QMessageBox.information(
                self,
                "划分完成",
                text
                + "\n\n原始 photo 文件已保留。"
                + f"\n划分清单：{summary.manifest}",
            )

        @QtCore.pyqtSlot(str)
        def _split_failed(self, message: str) -> None:
            self.status_label.setText("数据集划分失败")
            QtWidgets.QMessageBox.critical(self, "划分失败", message)

        @QtCore.pyqtSlot()
        def _split_finished(self) -> None:
            self.split_worker = None
            self.split_button.setEnabled(True)
            self._update_task_details()

        def closeEvent(self, event: Any) -> None:
            if self.camera_worker is not None and self.camera_worker.isRunning():
                self.camera_worker.requestInterruption()
                if not self.camera_worker.wait(4000):
                    QtWidgets.QMessageBox.warning(
                        self, "相机仍在停止", "请等待相机线程停止后再关闭窗口。"
                    )
                    event.ignore()
                    return
            if self.split_worker is not None and self.split_worker.isRunning():
                QtWidgets.QMessageBox.information(
                    self, "划分尚未完成", "请等待数据集划分完成后再关闭窗口。"
                )
                event.ignore()
                return
            event.accept()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="D435 RGB 拍照和 YOLO 数据集划分工具")
    parser.add_argument("--task", choices=TASKS, default="task2", help="界面默认选中的任务")
    parser.add_argument("--serial", default=None, help="多台 RealSense 时指定 D435 序列号")
    parser.add_argument("--seed", type=int, default=2026, help="可重复的数据集随机划分种子")
    parser.add_argument(
        "--split-only",
        choices=TASKS,
        default=None,
        help="不启动相机和 UI，直接划分指定任务",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.split_only:
        try:
            summary = split_dataset(DATASETS_ROOT / args.split_only, seed=args.seed)
        except DatasetSplitError as exc:
            print(f"[划分失败] {exc}", file=sys.stderr)
            return 2
        print(
            f"{summary.task} 划分完成：train={summary.train}, val={summary.val}, "
            f"test={summary.test}, labels={summary.labels}/{summary.total}"
        )
        return 0

    if _GUI_IMPORT_ERROR is not None or QtWidgets is None:
        print(
            "[依赖缺失] 无法启动拍照界面，请安装 requirements.txt 中的 "
            f"PyQt5、OpenCV 和 NumPy。\n原始错误：{_GUI_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 3
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setApplicationName("D435 YOLO 数据集拍照工具")
    window = CaptureWindow(initial_task=args.task, serial=args.serial, seed=args.seed)
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
