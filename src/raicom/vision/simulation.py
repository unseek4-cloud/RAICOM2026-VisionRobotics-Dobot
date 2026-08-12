# -*- coding: utf-8 -*-
"""完整视觉模拟世界。

默认同时摆放四件工件：任务二两件、任务三两件。机器人模拟完成后调用
``SimulationWorld.remove(object_id)``，下一帧图像、深度图和检测结果会同步
移除该工件，可验证“抓一个、回拍照位、重新识别”的正式流程。
"""

from __future__ import annotations

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
from raicom.vision.yolo_detector import DetectorError, annotate_detections


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


class SimulationWorld:
    """线程安全的模拟桌面、工件和深度图状态。"""

    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.width = int(settings.get("camera.width", 640))
        self.height = int(settings.get("camera.height", 480))
        self.table_depth_mm = float(settings.get("simulation.table_depth_mm", 700.0))
        self.place_base_robot_z_mm = float(
            settings.get("simulation.place_base_robot_z_mm", 90.0)
        )
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
        self._placement_view: dict[str, Any] | None = None
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
                "T2-RED-CUBE",
                "task2",
                "red_cube",
                self._scale_bbox((70, 105, 165, 200)),
                38.0,
                "red",
                "cube",
                "red",
                0.98,
            ),
            SimulationObject(
                "T2-BLUE-CYLINDER",
                "task2",
                "blue_cylinder",
                self._scale_bbox((210, 245, 305, 340)),
                52.0,
                "blue",
                "cylinder",
                "blue",
                0.97,
            ),
            SimulationObject(
                "T3-MATCH",
                "task3",
                "match",
                self._scale_bbox((365, 90, 465, 190)),
                30.0,
                "unknown",
                "cube",
                "match",
                0.96,
            ),
            SimulationObject(
                "T3-NOT-MATCH",
                "task3",
                "not_match",
                self._scale_bbox((485, 270, 590, 375)),
                44.0,
                "unknown",
                "cube",
                "not_match",
                0.95,
            ),
        )

    def reset(self) -> None:
        with self._lock:
            self._objects = {item.object_id: item for item in self._default_objects()}
            self._frame_number = 0
            self._placement_stacks_mm.clear()
            self._placement_view = None

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

    def begin_placement_inspection(
        self,
        place_x_mm: float,
        place_y_mm: float,
        inspection_z_mm: float,
        object_height_mm: float,
    ) -> None:
        """把模拟相机切换到目标放置点上方，吸盘继续保持工件。"""

        key = self._place_key(place_x_mm, place_y_mm)
        with self._lock:
            stack_height = self._placement_stacks_mm.get(key, 0.0)
            surface_z = self.place_base_robot_z_mm + stack_height
            depth_mm = float(inspection_z_mm) - surface_z
            if depth_mm <= 0:
                raise CameraError("模拟放置观察位低于当前堆叠顶面")
            self._placement_view = {
                "key": key,
                "surface_z_mm": surface_z,
                "depth_mm": depth_mm,
                "object_height_mm": float(object_height_mm),
            }

    def complete_placement(self) -> None:
        """完成当前模拟放置，使下一次深度帧能看到新的堆叠顶面。"""

        with self._lock:
            if self._placement_view is None:
                raise CameraError("模拟环境当前没有待完成的动态放置")
            key = self._placement_view["key"]
            self._placement_stacks_mm[key] = self._placement_stacks_mm.get(key, 0.0) + float(
                self._placement_view["object_height_mm"]
            )
            self._placement_view = None

    def cancel_placement_view(self) -> None:
        with self._lock:
            self._placement_view = None

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
            mask[:, :] = True
        return slice(y1, y2), slice(x1, x2), mask

    def render(self) -> FrameBundle:
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover
            raise CameraError("模拟画面生成需要 OpenCV（cv2）") from exc
        with self._lock:
            objects = tuple(self._objects.values())
            placement_view = (
                None if self._placement_view is None else dict(self._placement_view)
            )
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

        if placement_view is not None:
            # 观察阶段相机已移动到放置点上方。画面只表示目标顶面，深度由与
            # 抓取台面完全独立的机器人 Z 基准和当前堆叠高度生成。
            depth[:, :] = float(placement_view["depth_mm"])
            color[:, :] = (185, 205, 215)
            return FrameBundle(
                color_bgr=color,
                depth_mm=depth,
                intrinsics=self.intrinsics,
                timestamp_ms=time.time() * 1000.0,
                frame_number=frame_number,
            )

        for obj in objects:
            ys, xs, mask = self._object_mask(obj, self.width, self.height)
            if mask.size == 0:
                continue
            local_color = color[ys, xs]
            if obj.task == "task2":
                bgr = (25, 35, 225) if obj.color == "red" else (220, 70, 25)
            else:
                bgr = (235, 235, 235)
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
                cv2.rectangle(color, (x1, y1), (x2, y2), (30, 30, 30), 2)
            if obj.task == "task3":
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                radius = max(8, min(x2 - x1, y2 - y1) // 4)
                if obj.route_key == "match":
                    cv2.circle(color, (cx, cy), radius, (30, 170, 30), 4)
                    cv2.line(
                        color,
                        (cx - radius // 2, cy),
                        (cx - radius // 8, cy + radius // 2),
                        (30, 170, 30),
                        4,
                    )
                    cv2.line(
                        color,
                        (cx - radius // 8, cy + radius // 2),
                        (cx + radius // 2, cy - radius // 2),
                        (30, 170, 30),
                        4,
                    )
                else:
                    cv2.line(
                        color,
                        (cx - radius, cy - radius),
                        (cx + radius, cy + radius),
                        (30, 30, 210),
                        5,
                    )
                    cv2.line(
                        color,
                        (cx + radius, cy - radius),
                        (cx - radius, cy + radius),
                        (30, 30, 210),
                        5,
                    )

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
        if task not in {"task2", "task3"}:
            raise DetectorError("模拟检测 task 只能是 task2 或 task3")
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
            detection = Detection(
                task=obj.task,
                object_id=obj.object_id,
                class_name=obj.class_name,
                confidence=obj.confidence,
                bbox=obj.bbox,
                color=obj.color,
                shape=obj.shape,
                pixel_center=((x1 + x2) // 2, (y1 + y2) // 2),
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
