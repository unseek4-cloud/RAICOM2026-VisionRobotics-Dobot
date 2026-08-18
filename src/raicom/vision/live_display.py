# -*- coding: utf-8 -*-
"""主界面三路实时视觉画面的帧构建。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from raicom.types import Detection
from raicom.vision.realsense_camera import FrameBundle


@dataclass(slots=True)
class LiveVisionFrame:
    """同一采集时刻的 RGB、深度伪彩色和 YOLO 标注画面。"""

    color_bgr: np.ndarray
    depth_bgr: np.ndarray
    yolo_bgr: np.ndarray
    task: str
    detections: tuple[Detection, ...]
    timestamp_ms: float = 0.0
    frame_number: int = 0


def colorize_depth_mm(
    depth_mm: np.ndarray,
    *,
    depth_min_mm: float,
    depth_max_mm: float,
) -> np.ndarray:
    """把毫米深度转换为固定量程的 BGR Turbo 伪彩色图。

    固定量程可以避免画面随每帧最小/最大值闪烁；无效深度显示为黑色。
    """

    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - 项目运行依赖已包含 OpenCV
        raise RuntimeError("深度伪彩色显示需要 OpenCV（cv2）") from exc
    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("深度图必须是二维毫米数组")
    low = float(depth_min_mm)
    high = float(depth_max_mm)
    if not np.isfinite((low, high)).all() or low <= 0.0 or high <= low:
        raise ValueError("深度显示量程必须满足 0 < min < max")

    valid = np.isfinite(depth) & (depth > 0.0)
    normalized = np.zeros(depth.shape, dtype=np.uint8)
    if np.any(valid):
        clipped = np.clip(depth[valid].astype(np.float32), low, high)
        # Turbo 的高值偏红；反转后近处为暖色、远处为冷色，更符合深度图习惯。
        scaled = 255.0 - (clipped - low) * (255.0 / (high - low))
        normalized[valid] = np.rint(scaled).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored


def attach_object_heights(
    bundle: FrameBundle,
    detections: Sequence[Detection],
    *,
    camera: Any,
    calibration: Any,
) -> None:
    """按“空台深度 - 工件表面深度”为每个检测附加毫米高度。"""

    for detection in detections:
        try:
            surface_depth_mm = float(camera.measure_depth_mm(bundle, detection.bbox))
            detection.depth_mm = surface_depth_mm
            object_height_mm = float(calibration.object_height_mm(surface_depth_mm))
        except Exception as exc:
            detection.extra["height_error"] = str(exc)
            detection.extra.pop("object_height_mm", None)
            continue
        detection.extra["object_height_mm"] = object_height_mm
        detection.extra.pop("height_error", None)


def build_live_vision_frame(
    bundle: FrameBundle,
    *,
    task: str,
    detector: Any,
    camera: Any,
    calibration: Any,
    recognition_regions: Any,
    depth_min_mm: float,
    depth_max_mm: float,
) -> LiveVisionFrame:
    """在同一同步帧上完成区域过滤、高度计算和三路显示图构建。"""

    color = np.asarray(bundle.color_bgr)
    if color.ndim != 3 or color.shape[2] != 3:
        raise ValueError("RGB 显示源必须是 BGR 三通道图像")
    raw_detections = detector.detect(color)
    height, width = color.shape[:2]
    detections = recognition_regions.filter(
        task,
        raw_detections,
        width,
        height,
    )
    attach_object_heights(
        bundle,
        detections,
        camera=camera,
        calibration=calibration,
    )
    annotated = detector.annotate(color, detections)
    depth_bgr = colorize_depth_mm(
        bundle.depth_mm,
        depth_min_mm=depth_min_mm,
        depth_max_mm=depth_max_mm,
    )
    return LiveVisionFrame(
        color_bgr=np.ascontiguousarray(color),
        depth_bgr=np.ascontiguousarray(depth_bgr),
        yolo_bgr=np.ascontiguousarray(annotated),
        task=task,
        detections=tuple(detections),
        timestamp_ms=float(bundle.timestamp_ms),
        frame_number=int(bundle.frame_number),
    )


__all__ = [
    "LiveVisionFrame",
    "attach_object_heights",
    "build_live_vision_frame",
    "colorize_depth_mm",
]
