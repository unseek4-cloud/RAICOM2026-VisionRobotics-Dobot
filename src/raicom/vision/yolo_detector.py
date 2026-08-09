# -*- coding: utf-8 -*-
"""任务二/三 YOLO 检测、类别路由和中文可视化。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from raicom.config import Settings
from raicom.types import Detection


class DetectorError(RuntimeError):
    """模型、图像或推理结果不可用。"""


_COLOR_WORDS: dict[str, tuple[str, ...]] = {
    "red": ("red", "红"),
    "blue": ("blue", "蓝"),
    "green": ("green", "绿"),
    "yellow": ("yellow", "黄"),
}

_SHAPE_WORDS: dict[str, tuple[str, ...]] = {
    "cube": ("cube", "box", "square", "block", "立方", "方块", "正方"),
    "cylinder": ("cylinder", "cyl", "round", "circle", "圆柱", "圆形"),
}

_DISPLAY_COLOR = {
    "red": "红色",
    "blue": "蓝色",
    "green": "绿色",
    "yellow": "黄色",
    "unknown": "颜色未知",
    "未知": "颜色未知",
}
_DISPLAY_SHAPE = {
    "cube": "立方体",
    "cylinder": "圆柱体",
    "unknown": "形状未知",
    "未知": "形状未知",
}


def _log(logger: Any, level: str, message: str) -> None:
    if logger is None:
        return
    method = getattr(logger, level, None)
    if callable(method):
        method(message)
    elif callable(logger):
        logger(message)


def _contains_word(text: str, words: Iterable[str]) -> bool:
    folded = text.casefold()
    return any(str(word).casefold() in folded for word in words)


def infer_shape(class_name: str) -> str:
    for shape, words in _SHAPE_WORDS.items():
        if _contains_word(class_name, words):
            return shape
    return "unknown"


def infer_color_from_name(class_name: str) -> str:
    for color, words in _COLOR_WORDS.items():
        if _contains_word(class_name, words):
            return color
    return "unknown"


def classify_color_hsv(image_bgr: np.ndarray, bbox: Sequence[int]) -> str:
    """在检测框中心收缩区域内识别主色，避免背景主导结果。"""
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover
        raise DetectorError("HSV 颜色识别需要 OpenCV（cv2）") from exc
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        return "unknown"
    height, width = image_bgr.shape[:2]
    if len(bbox) != 4:
        return "unknown"
    x1, y1, x2, y2 = (int(v) for v in bbox)
    x1, x2 = max(0, min(x1, width)), max(0, min(x2, width))
    y1, y2 = max(0, min(y1, height)), max(0, min(y2, height))
    if x2 <= x1 or y2 <= y1:
        return "unknown"
    # 每侧收缩 15%，排除框边缘桌面；小框至少保留 2 像素。
    margin_x = min(max(2, int((x2 - x1) * 0.15)), max(0, (x2 - x1) // 3))
    margin_y = min(max(2, int((y2 - y1) * 0.15)), max(0, (y2 - y1) // 3))
    roi = image_bgr[y1 + margin_y : y2 - margin_y, x1 + margin_x : x2 - margin_x]
    if roi.size == 0:
        return "unknown"
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    ranges = {
        "red": (((0, 80, 55), (12, 255, 255)), ((165, 80, 55), (179, 255, 255))),
        "blue": (((95, 70, 45), (135, 255, 255)),),
        "green": (((36, 55, 40), (90, 255, 255)),),
        "yellow": (((16, 75, 70), (38, 255, 255)),),
    }
    total = float(roi.shape[0] * roi.shape[1])
    scores: dict[str, int] = {}
    for color, intervals in ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in intervals:
            mask |= cv2.inRange(
                hsv, np.asarray(low, dtype=np.uint8), np.asarray(high, dtype=np.uint8)
            )
        # 去掉孤立噪点，但不对很小的工件过度腐蚀。
        if min(mask.shape) >= 12:
            kernel = np.ones((3, 3), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        scores[color] = int(np.count_nonzero(mask))
    if not scores:
        return "unknown"
    color = max(scores, key=scores.get)
    best = scores[color]
    second = sorted(scores.values(), reverse=True)[1] if len(scores) > 1 else 0
    if best / total < 0.08 or (second > 0 and best < second * 1.15):
        return "unknown"
    return color


def _task_route(settings: Settings, task: str, detection: Detection) -> str:
    if task == "task2":
        route_by = str(settings.get("tasks.task2.route_by", "color")).lower()
        if route_by == "shape":
            return detection.shape if detection.shape != "unknown" else "default"
        if route_by == "class":
            return detection.class_name
        return detection.color if detection.color != "unknown" else "default"
    known_label = settings.get("yolo.task3.known_label", None)
    if known_label not in (None, ""):
        return (
            "match"
            if detection.class_name.casefold() == str(known_label).casefold()
            else "not_match"
        )
    return detection.class_name or "default"


def annotate_detections(
    image_bgr: np.ndarray,
    detections: Sequence[Detection],
    *,
    font_path: str | Path | None = None,
    logger: Any = None,
) -> np.ndarray:
    """绘制检测框和中文标签，返回独立图像。"""
    try:
        import cv2
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:  # pragma: no cover
        raise DetectorError("中文检测图绘制需要 OpenCV 与 Pillow") from exc
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise DetectorError("annotate 需要 BGR 三通道图像")
    output = image_bgr.copy()
    height, width = output.shape[:2]
    labels: list[tuple[int, int, str]] = []
    for detection in detections:
        x1, y1, x2, y2 = detection.bbox
        x1, x2 = max(0, min(x1, width - 1)), max(0, min(x2, width - 1))
        y1, y2 = max(0, min(y1, height - 1)), max(0, min(y2, height - 1))
        color = (30, 210, 50) if detection.task == "task2" else (230, 150, 30)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        cv2.drawMarker(
            output,
            detection.pixel_center,
            (0, 0, 255),
            cv2.MARKER_CROSS,
            12,
            2,
        )
        parts = [detection.class_name]
        if detection.task == "task2":
            parts.append(_DISPLAY_COLOR.get(detection.color, detection.color))
            parts.append(_DISPLAY_SHAPE.get(detection.shape, detection.shape))
        parts.append(f"{detection.confidence:.2f}")
        labels.append((x1, max(0, y1 - 24), " / ".join(parts)))

    # Pillow 使用系统中文字体；找不到字体时仍绘制 ASCII 类别并记录警告。
    rgb = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb)
    drawer = ImageDraw.Draw(pil_image)
    font = None
    if font_path:
        try:
            font = ImageFont.truetype(str(font_path), 18)
        except Exception as exc:
            _log(logger, "warning", f"中文字体加载失败 {font_path}：{exc}")
    if font is None:
        try:
            font = ImageFont.truetype("msyh.ttc", 18)
        except Exception:
            font = ImageFont.load_default()
    for x, y, label in labels:
        try:
            box = drawer.textbbox((x, y), label, font=font)
            drawer.rectangle(box, fill=(0, 0, 0))
            drawer.text((x, y), label, font=font, fill=(255, 255, 255))
        except UnicodeEncodeError:
            ascii_label = label.encode("ascii", errors="replace").decode("ascii")
            drawer.text((x, y), ascii_label, font=font, fill=(255, 255, 255))
    return cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)


class YoloDetector:
    """Ultralytics YOLO 真模型封装。"""

    def __init__(self, settings: Settings, task: str, logger: Any = None) -> None:
        if task not in {"task2", "task3"}:
            raise DetectorError("YOLO task 只能是 task2 或 task3")
        self.settings = settings
        self.task = task
        self.logger = logger
        self.confidence = float(settings.get("yolo.confidence", 0.70))
        self.iou = float(settings.get("yolo.iou", 0.45))
        self.image_size = int(settings.get("yolo.image_size", 640))
        self.max_detections = int(settings.get("yolo.max_detections", 20))
        self.use_hsv = bool(settings.get("yolo.use_hsv_color_fallback", True))
        self.font_path = settings.get("yolo.chinese_font", None)
        self.include_keywords = tuple(
            str(item).casefold()
            for item in settings.get(f"yolo.{task}.include_class_keywords", [])
        )
        self.exclude_keywords = tuple(
            str(item).casefold()
            for item in settings.get(f"yolo.{task}.exclude_class_keywords", [])
        )
        model_path = settings.resolve_path(f"yolo.{task}.model", required=True)
        if not model_path.is_file():
            raise DetectorError(f"{task} YOLO 模型不存在：{model_path}")
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise DetectorError("缺少 torch 或 ultralytics，无法加载 YOLO") from exc
        requested_device = str(settings.get("yolo.device", "cpu"))
        if requested_device.lower().startswith("cuda") and not torch.cuda.is_available():
            self.device = "cpu"
            _log(logger, "warning", "CUDA 当前不可用，YOLO 已安全回退 CPU")
        else:
            self.device = requested_device
        try:
            self.model = YOLO(str(model_path))
        except Exception as exc:
            raise DetectorError(f"YOLO 模型加载失败 {model_path}：{exc}") from exc
        self.names = getattr(self.model, "names", {})
        _log(logger, "info", f"已加载 {task} 模型：{model_path.name}，设备={self.device}")

    def _allowed_class(self, class_name: str) -> bool:
        folded = class_name.casefold()
        if self.include_keywords and not any(k in folded for k in self.include_keywords):
            return False
        if self.exclude_keywords and any(k in folded for k in self.exclude_keywords):
            return False
        return True

    def _class_name(self, class_id: int, result: Any) -> str:
        names = getattr(result, "names", None) or self.names
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            raise DetectorError("YOLO 输入必须是 BGR 三通道图像")
        try:
            results = self.model.predict(
                source=image_bgr,
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.image_size,
                device=self.device,
                max_det=self.max_detections,
                verbose=False,
            )
        except Exception as exc:
            raise DetectorError(f"YOLO 推理失败：{exc}") from exc
        if not results:
            return []
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        image_height, image_width = image_bgr.shape[:2]
        detections: list[Detection] = []
        for box in boxes:
            try:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                coords = box.xyxy[0].detach().cpu().tolist()
                x1, y1, x2, y2 = (int(round(float(value))) for value in coords)
            except Exception as exc:
                _log(self.logger, "warning", f"忽略无法解析的 YOLO 检测框：{exc}")
                continue
            class_name = self._class_name(class_id, result)
            if not self._allowed_class(class_name):
                continue
            x1 = max(0, min(x1, image_width - 1))
            x2 = max(0, min(x2, image_width - 1))
            y1 = max(0, min(y1, image_height - 1))
            y2 = max(0, min(y2, image_height - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            center = (int(round((x1 + x2) / 2)), int(round((y1 + y2) / 2)))
            color = infer_color_from_name(class_name)
            if self.task == "task2" and color == "unknown" and self.use_hsv:
                color = classify_color_hsv(image_bgr, (x1, y1, x2, y2))
            shape = infer_shape(class_name) if self.task == "task2" else "unknown"
            detection = Detection(
                task=self.task,
                object_id=f"{self.task}-{class_id}-{center[0]}-{center[1]}",
                class_name=class_name,
                confidence=confidence,
                bbox=(x1, y1, x2, y2),
                color=color,
                shape=shape,
                pixel_center=center,
                extra={"class_id": class_id},
            )
            detection.route_key = _task_route(self.settings, self.task, detection)
            detections.append(detection)
        detections.sort(key=lambda item: item.confidence, reverse=True)
        return detections

    def annotate(
        self, image_bgr: np.ndarray, detections: Sequence[Detection]
    ) -> np.ndarray:
        return annotate_detections(
            image_bgr,
            detections,
            font_path=self.font_path,
            logger=self.logger,
        )


__all__ = [
    "DetectorError",
    "YoloDetector",
    "annotate_detections",
    "classify_color_hsv",
    "infer_color_from_name",
    "infer_shape",
]
