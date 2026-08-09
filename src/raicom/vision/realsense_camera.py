# -*- coding: utf-8 -*-
"""Intel RealSense D435 彩色/深度同步采集。

深度图在本模块出口统一转换为毫米浮点数组，并与彩色图对齐。调用者不得把
0、NaN、越量程像素当作抓取高度；``measure_depth_mm`` 对单帧中心邻域做中值
和 MAD 离群剔除，跨帧中值及波动检查由任务状态机统一完成。
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass
from math import isfinite
from typing import Any, Sequence

import numpy as np

from raicom.config import Settings
from raicom.types import CameraIntrinsics


class CameraError(RuntimeError):
    """相机启动、取帧或配置失败。"""


class DepthMeasurementError(CameraError):
    """目标区域没有足够可信的深度。"""


@dataclass(slots=True)
class FrameBundle:
    """同一时刻的 BGR 彩色图、对齐深度图和所用内参。"""

    color_bgr: np.ndarray
    depth_mm: np.ndarray
    intrinsics: CameraIntrinsics
    timestamp_ms: float = 0.0
    frame_number: int = 0


def _emit_log(logger: Any, level: str, message: str) -> None:
    if logger is None:
        return
    method = getattr(logger, level, None)
    if callable(method):
        method(message)
    elif callable(logger):
        logger(message)


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise CameraError(f"{name} 必须是数字")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CameraError(f"{name} 必须是数字，当前为 {value!r}") from exc
    if not isfinite(result):
        raise CameraError(f"{name} 不能是 NaN 或无穷大")
    return result


def _normalize_bbox(
    bbox: Sequence[int | float], width: int, height: int
) -> tuple[int, int, int, int]:
    if len(bbox) != 4:
        raise DepthMeasurementError("检测框必须是 (x1,y1,x2,y2)")
    try:
        x1, y1, x2, y2 = (int(round(float(v))) for v in bbox)
    except (TypeError, ValueError) as exc:
        raise DepthMeasurementError(f"检测框含非数字：{bbox!r}") from exc
    x1, x2 = sorted((max(0, min(width, x1)), max(0, min(width, x2))))
    y1, y2 = sorted((max(0, min(height, y1)), max(0, min(height, y2))))
    if x2 - x1 < 2 or y2 - y1 < 2:
        raise DepthMeasurementError(f"检测框无有效面积：{bbox!r}")
    return x1, y1, x2, y2


def robust_depth_from_bundle(
    bundle: FrameBundle,
    bbox: Sequence[int | float],
    *,
    patch_px: int,
    depth_min_mm: float,
    depth_max_mm: float,
) -> float:
    """从检测框中心邻域提取单帧可信深度，单位 mm。"""
    depth = np.asarray(bundle.depth_mm)
    if depth.ndim != 2:
        raise DepthMeasurementError("深度图必须是二维数组")
    image_height, image_width = depth.shape
    if bundle.color_bgr.shape[:2] != (image_height, image_width):
        raise DepthMeasurementError("深度图未与彩色图对齐，禁止用于抓取")
    x1, y1, x2, y2 = _normalize_bbox(bbox, image_width, image_height)
    if patch_px < 3 or patch_px % 2 == 0:
        raise CameraError("camera.depth_patch_px 必须是大于等于 3 的奇数")

    cx = int(round((x1 + x2 - 1) / 2.0))
    cy = int(round((y1 + y2 - 1) / 2.0))
    half = patch_px // 2
    # 中心邻域不能越过检测框，尽量避免背景深度污染。
    px1, px2 = max(x1, cx - half), min(x2, cx + half + 1)
    py1, py2 = max(y1, cy - half), min(y2, cy + half + 1)
    patch = depth[py1:py2, px1:px2].astype(np.float64, copy=False)
    valid_mask = (
        np.isfinite(patch)
        & (patch >= depth_min_mm)
        & (patch <= depth_max_mm)
        & (patch > 0.0)
    )
    values = patch[valid_mask]
    required = max(3, int(np.ceil(patch.size * 0.35)))
    if values.size < required:
        raise DepthMeasurementError(
            f"目标中心有效深度不足：{values.size}/{patch.size}；"
            "可能位于盲区、反光、遮挡或框中心不在工件顶面"
        )

    median = float(np.median(values))
    absolute_deviation = np.abs(values - median)
    mad = float(np.median(absolute_deviation))
    if mad > 0.0:
        # 1.4826 把 MAD 换算成近似标准差；至少保留 2 mm 容差，避免量化噪声。
        tolerance = max(2.0, 3.5 * 1.4826 * mad)
        filtered = values[absolute_deviation <= tolerance]
        if filtered.size >= required:
            median = float(np.median(filtered))
    if not depth_min_mm <= median <= depth_max_mm:
        raise DepthMeasurementError(f"深度 {median:.2f} mm 超出相机安全量程")
    return median


class RealSenseCamera:
    """真实 D435 相机。一个实例应只由一个采集线程调用。"""

    def __init__(self, settings: Settings, logger: Any = None) -> None:
        self.settings = settings
        self.logger = logger
        self.width = int(settings.get("camera.width", 640))
        self.height = int(settings.get("camera.height", 480))
        self.fps = int(settings.get("camera.fps", 30))
        self.align_depth_to_color = bool(
            settings.get("camera.align_depth_to_color", True)
        )
        self.warmup_frames = int(settings.get("camera.warmup_frames", 30))
        self.flush_frames_after_motion = int(
            settings.get("camera.flush_frames_after_motion", 8)
        )
        self.depth_min_mm = _finite_number(
            settings.get("camera.depth_min_mm", 300.0), "camera.depth_min_mm"
        )
        self.depth_max_mm = _finite_number(
            settings.get("camera.depth_max_mm", 1500.0), "camera.depth_max_mm"
        )
        self.patch_px = int(settings.get("camera.depth_patch_px", 9))
        self.temporal_samples = max(
            1, int(settings.get("camera.temporal_depth_samples", 5))
        )
        if self.width <= 0 or self.height <= 0 or self.fps <= 0:
            raise CameraError("相机宽、高、FPS 必须大于 0")
        if self.depth_min_mm <= 0 or self.depth_max_mm <= self.depth_min_mm:
            raise CameraError("相机深度量程配置错误")
        if self.patch_px < 3 or self.patch_px % 2 == 0:
            raise CameraError("camera.depth_patch_px 必须是大于等于 3 的奇数")

        self._rs: Any = None
        self._pipeline: Any = None
        self._align: Any = None
        self._depth_scale_mm = 1.0
        self._running = False
        self._lock = threading.RLock()
        self._history: deque[tuple[tuple[int, int], float]] = deque(
            maxlen=self.temporal_samples
        )

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            if not self.align_depth_to_color:
                raise CameraError(
                    "真实抓取必须启用 camera.align_depth_to_color，"
                    "否则 YOLO 彩色像素不能直接索引深度图"
                )
            try:
                import pyrealsense2 as rs
            except ImportError as exc:
                raise CameraError(
                    "未安装 pyrealsense2，请在 HKtest 环境安装后重试"
                ) from exc

            pipeline = rs.pipeline()
            config = rs.config()
            serial = self.settings.get("camera.serial", None)
            if serial not in (None, ""):
                config.enable_device(str(serial))
            config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.bgr8,
                self.fps,
            )
            config.enable_stream(
                rs.stream.depth,
                self.width,
                self.height,
                rs.format.z16,
                self.fps,
            )
            try:
                profile = pipeline.start(config)
            except Exception as exc:
                raise CameraError(
                    "RealSense 启动失败；请检查 D435 连接、序列号、USB3 带宽和占用程序："
                    f"{exc}"
                ) from exc

            try:
                device = profile.get_device()
                depth_sensor = device.first_depth_sensor()
                self._depth_scale_mm = float(depth_sensor.get_depth_scale()) * 1000.0
                self._apply_exposure(device, rs)
                self._rs = rs
                self._pipeline = pipeline
                self._align = rs.align(rs.stream.color)
                self._running = True
                self._history.clear()
                for _ in range(max(0, self.warmup_frames)):
                    pipeline.wait_for_frames(5000)
            except Exception:
                pipeline.stop()
                self._pipeline = None
                self._running = False
                raise
            _emit_log(
                self.logger,
                "info",
                f"RealSense 已启动：{self.width}×{self.height}@{self.fps}，"
                f"depth_scale={self._depth_scale_mm:.6f} mm",
            )

    def _apply_exposure(self, device: Any, rs: Any) -> None:
        color_value = self.settings.get("camera.color_exposure", None)
        depth_value = self.settings.get("camera.depth_exposure", None)
        if color_value is None and depth_value is None:
            return
        for sensor in device.query_sensors():
            try:
                name = sensor.get_info(rs.camera_info.name).lower()
            except Exception:
                name = ""
            requested = depth_value if "stereo" in name or "depth" in name else color_value
            if requested is None:
                continue
            try:
                if sensor.supports(rs.option.enable_auto_exposure):
                    sensor.set_option(rs.option.enable_auto_exposure, 0.0)
                if sensor.supports(rs.option.exposure):
                    sensor.set_option(rs.option.exposure, float(requested))
                else:
                    _emit_log(self.logger, "warning", f"传感器 {name} 不支持曝光设置")
            except Exception as exc:
                raise CameraError(f"设置 {name or '相机'} 曝光失败：{exc}") from exc

    def stop(self) -> None:
        with self._lock:
            pipeline, self._pipeline = self._pipeline, None
            self._running = False
            self._align = None
            self._history.clear()
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
        _emit_log(self.logger, "info", "RealSense 已停止")

    def get_frame(self, timeout_ms: int = 5000) -> FrameBundle:
        with self._lock:
            if not self._running or self._pipeline is None:
                raise CameraError("RealSense 尚未启动")
            try:
                frames = self._pipeline.wait_for_frames(int(timeout_ms))
                frames = self._align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    raise CameraError("RealSense 同步帧缺少彩色或深度数据")
                color = np.asanyarray(color_frame.get_data()).copy()
                raw_depth = np.asanyarray(depth_frame.get_data())
                depth_mm = raw_depth.astype(np.float32) * self._depth_scale_mm
                # 对齐后的深度帧坐标正是 YOLO 彩色像素坐标。保存该 profile 的
                # 原生 intrinsics，后续交给 RealSense SDK 按实际畸变模型反投影。
                video_profile = depth_frame.profile.as_video_stream_profile()
                intr = video_profile.get_intrinsics()
                intrinsics = CameraIntrinsics(
                    width=int(intr.width),
                    height=int(intr.height),
                    fx=float(intr.fx),
                    fy=float(intr.fy),
                    ppx=float(intr.ppx),
                    ppy=float(intr.ppy),
                    native=intr,
                )
                if color.shape[:2] != depth_mm.shape:
                    raise CameraError("RealSense 对齐后彩色图与深度图尺寸仍不一致")
                return FrameBundle(
                    color_bgr=color,
                    depth_mm=depth_mm,
                    intrinsics=intrinsics,
                    timestamp_ms=float(color_frame.get_timestamp()),
                    frame_number=int(color_frame.get_frame_number()),
                )
            except CameraError:
                raise
            except Exception as exc:
                raise CameraError(f"RealSense 取帧失败：{exc}") from exc

    def flush(self, count: int | None = None) -> FrameBundle:
        """机械臂运动停止后丢弃若干旧帧，返回最新稳定帧。"""
        number = self.flush_frames_after_motion if count is None else int(count)
        if number < 1:
            number = 1
        self.reset_depth_history()
        latest: FrameBundle | None = None
        for _ in range(number):
            latest = self.get_frame()
        assert latest is not None
        return latest

    def reset_depth_history(self) -> None:
        self._history.clear()

    def measure_depth_mm(
        self, bundle: FrameBundle, bbox: Sequence[int | float]
    ) -> float:
        # 必须返回本帧的独立样本。若在这里先做累计中值，状态机看到的将是
        # “中值的中值”，会低估真实跨帧极差，削弱振动/旧帧安全检查。
        return robust_depth_from_bundle(
            bundle,
            bbox,
            patch_px=self.patch_px,
            depth_min_mm=self.depth_min_mm,
            depth_max_mm=self.depth_max_mm,
        )


__all__ = [
    "CameraError",
    "DepthMeasurementError",
    "FrameBundle",
    "RealSenseCamera",
    "robust_depth_from_bundle",
]
