# -*- coding: utf-8 -*-
"""完整视觉模拟世界。

默认同时摆放 3D 识别抓取的七类工件。机器人模拟完成后调用
``SimulationWorld.remove(object_id)``，下一帧图像、深度图和检测结果会同步
移除该工件，可验证“抓一个、回拍照位、重新识别”的正式流程。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from raicom.config import Settings
from raicom.types import CameraIntrinsics, Detection
from raicom.vision.realsense_camera import (
    CameraError,
    FrameBundle,
    robust_depth_from_bundle,
)
from raicom.vision.yolo_detector import (
    DetectorError,
    annotate_detections,
    oriented_box_axis,
)


def _log(logger: Any, level: str, message: str) -> None:
    if logger is None:
        return
    method = getattr(logger, level, None)
    if callable(method):
        method(message)
    elif callable(logger):
        logger(message)


@dataclass(frozen=True, slots=True)
class SimulationObject:
    object_id: str
    task: str
    class_name: str
    bbox: tuple[int, int, int, int]
    height_mm: float
    color: str
    shape: str
    route_key: str
    confidence: float = 0.96
    angle_deg: float = 0.0

    def oriented_bbox(self) -> tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ]:
        """生成位于轴对齐包围框内的模拟旋转矩形。"""

        x1, y1, x2, y2 = self.bbox
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        width = max(2.0, (x2 - x1) * 0.72)
        height = max(2.0, (y2 - y1) * 0.48)
        angle = math.radians(self.angle_deg)
        ux = (math.cos(angle), math.sin(angle))
        uy = (-math.sin(angle), math.cos(angle))
        result: list[tuple[float, float]] = []
        for along, across in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
            result.append(
                (
                    cx + along * width * ux[0] / 2.0 + across * height * uy[0] / 2.0,
                    cy + along * width * ux[1] / 2.0 + across * height * uy[1] / 2.0,
                )
            )
        return result[0], result[1], result[2], result[3]


class SimulationWorld:
    """线程安全的模拟桌面、工件和深度图状态。"""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.width = int(settings.get("simulation.frame_width", 640))
        self.height = int(settings.get("simulation.frame_height", 480))
        self.table_depth_mm = float(settings.get("simulation.table_depth_mm", 700.0))
        if self.width < 320 or self.height < 240:
            raise CameraError("模拟画面至少需要 320×240")
        if self.table_depth_mm <= 0:
            raise CameraError("simulation.table_depth_mm 必须大于 0")
        self.intrinsics = CameraIntrinsics(
            width=self.width,
            height=self.height,
            fx=610.0 * self.width / 640.0,
            fy=610.0 * self.height / 480.0,
            ppx=(self.width - 1) / 2.0,
            ppy=(self.height - 1) / 2.0,
        )
        self._lock = threading.RLock()
        self._frame_number = 0
        self._placement_stacks_mm: dict[tuple[float, float], float] = {}
        self._objects: dict[str, SimulationObject] = {
            item.object_id: item for item in self._default_objects()
        }

    def _scale_bbox(self, box: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
        sx, sy = self.width / 640.0, self.height / 480.0
        x1, y1, x2, y2 = box
        return (
            int(round(x1 * sx)),
            int(round(y1 * sy)),
            int(round(x2 * sx)),
            int(round(y2 * sy)),
        )

    def _default_objects(self) -> tuple[SimulationObject, ...]:
        return (
            SimulationObject(
                "T3-LARGE-CYLINDER",
                "task3",
                "大圆柱",
                self._scale_bbox((25, 65, 105, 155)),
                54.0,
                "unknown",
                "cylinder",
                "大圆柱",
                0.98,
                0.0,
            ),
            SimulationObject(
                "T3-CUBE",
                "task3",
                "正方体",
                self._scale_bbox((125, 65, 205, 155)),
                40.0,
                "unknown",
                "cube",
                "正方体",
                0.97,
                22.0,
            ),
            SimulationObject(
                "T3-TRAPEZOID",
                "task3",
                "梯形",
                self._scale_bbox((225, 65, 305, 155)),
                32.0,
                "unknown",
                "cube",
                "梯形",
                0.96,
                -35.0,
            ),
            SimulationObject(
                "T3-CUBOID",
                "task3",
                "长方体",
                self._scale_bbox((325, 65, 425, 155)),
                46.0,
                "unknown",
                "cube",
                "长方体",
                0.95,
                42.0,
            ),
            SimulationObject(
                "T3-CYLINDER",
                "task3",
                "圆柱",
                self._scale_bbox((75, 245, 155, 335)),
                36.0,
                "unknown",
                "cylinder",
                "圆柱",
                0.97,
                0.0,
            ),
            SimulationObject(
                "T3-HEXAGONAL-PRISM",
                "task3",
                "六棱柱",
                self._scale_bbox((200, 245, 290, 335)),
                48.0,
                "unknown",
                "cube",
                "六棱柱",
                0.96,
                -18.0,
            ),
            SimulationObject(
                "T3-PARALLELOGRAM",
                "task3",
                "平行四边形",
                self._scale_bbox((335, 245, 445, 335)),
                28.0,
                "unknown",
                "cube",
                "平行四边形",
                0.95,
                33.0,
            ),
        )

    def reset(self) -> None:
        with self._lock:
            self._objects = {item.object_id: item for item in self._default_objects()}
            self._frame_number = 0
            self._placement_stacks_mm.clear()

    def objects(self, task: str | None = None) -> tuple[SimulationObject, ...]:
        with self._lock:
            values = tuple(self._objects.values())
        if task is not None:
            values = tuple(item for item in values if item.task == task)
        return tuple(sorted(values, key=lambda item: item.bbox[0]))

    def remove(self, object_id: str) -> bool:
        """移除已抓取工件；不存在时返回 False，便于发现重复命令。"""
        with self._lock:
            return self._objects.pop(str(object_id), None) is not None

    @staticmethod
    def _place_key(place_x_mm: float, place_y_mm: float) -> tuple[float, float]:
        return round(float(place_x_mm), 3), round(float(place_y_mm), 3)

    def complete_direct_placement(
        self, place_x_mm: float, place_y_mm: float, object_height_mm: float
    ) -> None:
        """记录一次不经过放置观察位的 P1→P2 同 Z 模拟放置。"""

        key = self._place_key(place_x_mm, place_y_mm)
        with self._lock:
            self._placement_stacks_mm[key] = self._placement_stacks_mm.get(key, 0.0) + float(
                object_height_mm
            )

    def placement_stack_height_mm(self, place_x_mm: float, place_y_mm: float) -> float:
        key = self._place_key(place_x_mm, place_y_mm)
        with self._lock:
            return float(self._placement_stacks_mm.get(key, 0.0))

    @staticmethod
    def _object_mask(
        obj: SimulationObject, width: int, height: int
    ) -> tuple[slice, slice, np.ndarray]:
        x1, y1, x2, y2 = obj.bbox
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        local_h, local_w = max(0, y2 - y1), max(0, x2 - x1)
        mask = np.zeros((local_h, local_w), dtype=bool)
        if local_h == 0 or local_w == 0:
            return slice(y1, y2), slice(x1, x2), mask
        if obj.shape == "cylinder":
            yy, xx = np.ogrid[:local_h, :local_w]
            rx, ry = max(1.0, local_w / 2.0), max(1.0, local_h / 2.0)
            mask = ((xx - (local_w - 1) / 2.0) / rx) ** 2 + (
                (yy - (local_h - 1) / 2.0) / ry
            ) ** 2 <= 1.0
        else:
            try:
                import cv2
            except ImportError as exc:  # pragma: no cover
                raise CameraError("模拟旋转工件生成需要 OpenCV（cv2）") from exc
            polygon = np.rint(np.asarray(obj.oriented_bbox())).astype(np.int32)
            polygon[:, 0] -= x1
            polygon[:, 1] -= y1
            cv2.fillConvexPoly(mask.view(np.uint8), polygon, 1)
        return slice(y1, y2), slice(x1, x2), mask

    def render(self) -> FrameBundle:
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover
            raise CameraError("模拟画面生成需要 OpenCV（cv2）") from exc
        with self._lock:
            objects = tuple(self._objects.values())
            self._frame_number += 1
            frame_number = self._frame_number
        # 带细网格的浅灰桌面，便于肉眼确认画面在持续刷新。
        color = np.full((self.height, self.width, 3), (205, 205, 200), dtype=np.uint8)
        spacing = max(30, int(self.width / 16))
        for x in range(0, self.width, spacing):
            cv2.line(color, (x, 0), (x, self.height - 1), (190, 190, 185), 1)
        for y in range(0, self.height, spacing):
            cv2.line(color, (0, y), (self.width - 1, y), (190, 190, 185), 1)
        depth = np.full(
            (self.height, self.width), self.table_depth_mm, dtype=np.float32
        )

        for obj in objects:
            ys, xs, mask = self._object_mask(obj, self.width, self.height)
            if mask.size == 0:
                continue
            local_color = color[ys, xs]
            palette = {
                "大圆柱": (65, 115, 220),
                "正方体": (210, 105, 55),
                "梯形": (70, 170, 75),
                "长方体": (180, 90, 180),
                "圆柱": (50, 185, 210),
                "六棱柱": (200, 155, 60),
                "平行四边形": (115, 115, 220),
            }
            bgr = palette.get(obj.class_name, (235, 235, 235))
            local_color[mask] = bgr
            local_depth = depth[ys, xs]
            local_depth[mask] = self.table_depth_mm - obj.height_mm
            x1, y1, x2, y2 = obj.bbox
            if obj.shape == "cylinder":
                cv2.ellipse(
                    color,
                    ((x1 + x2) // 2, (y1 + y2) // 2),
                    (max(1, (x2 - x1) // 2), max(1, (y2 - y1) // 2)),
                    0,
                    0,
                    360,
                    (30, 30, 30),
                    2,
                )
            else:
                polygon = np.rint(np.asarray(obj.oriented_bbox())).astype(np.int32)
                cv2.polylines(color, [polygon.reshape((-1, 1, 2))], True, (30, 30, 30), 2)

        return FrameBundle(
            color_bgr=color,
            depth_mm=depth,
            intrinsics=self.intrinsics,
            timestamp_ms=time.time() * 1000.0,
            frame_number=frame_number,
        )


class MockCamera:
    """与 ``RealSenseCamera`` 同接口的模拟相机。"""

    def __init__(
        self,
        settings: Settings,
        world: SimulationWorld | Any | None = None,
        logger: Any = None,
    ) -> None:
        # 兼容 MockCamera(settings, logger) 的装配方式。
        if world is not None and not isinstance(world, SimulationWorld):
            if logger is None:
                logger = world
                world = None
            else:
                raise TypeError("world 必须是 SimulationWorld")
        self.settings = settings
        self.world = world or SimulationWorld(settings)
        self.logger = logger
        self.patch_px = int(settings.get("camera.depth_patch_px", 9))
        self.depth_min_mm = float(settings.get("camera.depth_min_mm", 300.0))
        self.depth_max_mm = float(settings.get("camera.depth_max_mm", 1500.0))
        self._running = False

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        self._running = True
        _log(self.logger, "info", "模拟 RealSense 已启动")

    def stop(self) -> None:
        self._running = False

    def get_frame(self, timeout_ms: int = 5000) -> FrameBundle:
        del timeout_ms
        if not self._running:
            raise CameraError("模拟相机尚未启动")
        return self.world.render()

    def flush(self, count: int | None = None) -> FrameBundle:
        number = int(
            self.settings.get("camera.flush_frames_after_motion", 8)
            if count is None
            else count
        )
        latest: FrameBundle | None = None
        for _ in range(max(1, number)):
            latest = self.get_frame()
        assert latest is not None
        return latest

    def reset_depth_history(self) -> None:
        return None

    def measure_depth_mm(
        self, bundle: FrameBundle, bbox: Sequence[int | float]
    ) -> float:
        return robust_depth_from_bundle(
            bundle,
            bbox,
            patch_px=self.patch_px,
            depth_min_mm=self.depth_min_mm,
            depth_max_mm=self.depth_max_mm,
        )


class MockDetector:
    """读取模拟世界真值，但输出正式 ``Detection`` 数据结构。"""

    def __init__(
        self,
        settings: Settings,
        task: str,
        world: SimulationWorld | Any | None = None,
        logger: Any = None,
    ) -> None:
        if task != "task3":
            raise DetectorError("模拟检测只支持 3D识别抓取（task3）")
        # 兼容 MockDetector(settings, task, logger)；共享世界时请显式传 world。
        if world is not None and not isinstance(world, SimulationWorld):
            if logger is None:
                logger = world
                world = None
            else:
                raise TypeError("world 必须是 SimulationWorld")
        self.settings = settings
        self.task = task
        self.world = world or SimulationWorld(settings)
        self.logger = logger
        self.font_path = settings.get("yolo.chinese_font", None)

    def detect(self, image_bgr: np.ndarray) -> list[Detection]:
        if image_bgr is None or image_bgr.ndim != 3:
            raise DetectorError("模拟检测器输入图像无效")
        detections: list[Detection] = []
        for obj in self.world.objects(self.task):
            x1, y1, x2, y2 = obj.bbox
            oriented_bbox = obj.oriented_bbox()
            _, _, image_angle = oriented_box_axis(oriented_bbox)
            detection = Detection(
                task=obj.task,
                object_id=obj.object_id,
                class_name=obj.class_name,
                confidence=obj.confidence,
                bbox=obj.bbox,
                color=obj.color,
                shape=obj.shape,
                pixel_center=((x1 + x2) // 2, (y1 + y2) // 2),
                oriented_bbox=oriented_bbox,
                image_angle_deg=image_angle,
                route_key=obj.route_key,
                extra={"simulation": True, "height_mm": obj.height_mm},
            )
            detections.append(detection)
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
    "MockCamera",
    "MockDetector",
    "SimulationObject",
    "SimulationWorld",
]
