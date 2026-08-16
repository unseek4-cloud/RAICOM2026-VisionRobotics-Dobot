# -*- coding: utf-8 -*-
"""Task2/Task3 识别区域的校验、线程安全存储与检测过滤。"""

from __future__ import annotations

import math
import threading
from collections.abc import Iterable, Sequence
from typing import Any, TypeVar


RecognitionRegion = tuple[float, float, float, float]
FULL_RECOGNITION_REGION: RecognitionRegion = (0.0, 0.0, 1.0, 1.0)
_SUPPORTED_TASKS = frozenset({"task2", "task3"})
_DetectionT = TypeVar("_DetectionT")


class RecognitionRegionError(ValueError):
    """识别区域格式或任务名无效。"""


def validate_recognition_region(value: Any) -> RecognitionRegion:
    """把 ``[x1, y1, x2, y2]`` 校验为 0~1 范围内的矩形。"""

    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or len(value) != 4
    ):
        raise RecognitionRegionError("识别区域必须是 [x1, y1, x2, y2] 四个归一化数值")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise RecognitionRegionError("识别区域坐标必须是有限数值")
    region = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in region):
        raise RecognitionRegionError("识别区域坐标必须是有限数值")
    x1, y1, x2, y2 = region
    if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
        raise RecognitionRegionError(
            "识别区域必须满足 0 <= x1 < x2 <= 1、0 <= y1 < y2 <= 1"
        )
    return x1, y1, x2, y2


def region_contains_pixel(
    region: RecognitionRegion,
    pixel_center: tuple[int, int],
    image_width: int,
    image_height: int,
) -> bool:
    """判断像素中心是否位于识别区域内。"""

    if image_width <= 0 or image_height <= 0:
        raise RecognitionRegionError("图像尺寸必须大于 0")
    x, y = pixel_center
    normalized_x = (float(x) + 0.5) / float(image_width)
    normalized_y = (float(y) + 0.5) / float(image_height)
    x1, y1, x2, y2 = region
    return x1 <= normalized_x <= x2 and y1 <= normalized_y <= y2


class RecognitionRegionStore:
    """保存两个任务的区域；UI 与检测工作线程可安全地同时访问。"""

    def __init__(self, settings: Any) -> None:
        self._lock = threading.RLock()
        self._regions: dict[str, RecognitionRegion] = {
            task: validate_recognition_region(
                settings.get(f"tasks.{task}.recognition_region", FULL_RECOGNITION_REGION)
            )
            for task in _SUPPORTED_TASKS
        }

    @staticmethod
    def _validate_task(task: str) -> None:
        if task not in _SUPPORTED_TASKS:
            raise RecognitionRegionError(f"不支持设置识别区域的任务：{task}")

    def get(self, task: str) -> RecognitionRegion:
        self._validate_task(task)
        with self._lock:
            return self._regions[task]

    def set(self, task: str, region: Sequence[float]) -> RecognitionRegion:
        self._validate_task(task)
        validated = validate_recognition_region(region)
        with self._lock:
            self._regions[task] = validated
        return validated

    def filter(
        self,
        task: str,
        detections: Iterable[_DetectionT],
        image_width: int,
        image_height: int,
    ) -> list[_DetectionT]:
        region = self.get(task)
        return [
            detection
            for detection in detections
            if region_contains_pixel(
                region,
                detection.pixel_center,
                image_width,
                image_height,
            )
        ]


__all__ = [
    "FULL_RECOGNITION_REGION",
    "RecognitionRegion",
    "RecognitionRegionError",
    "RecognitionRegionStore",
    "region_contains_pixel",
    "validate_recognition_region",
]
