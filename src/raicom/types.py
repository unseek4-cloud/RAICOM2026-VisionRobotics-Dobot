# -*- coding: utf-8 -*-
"""跨模块共享的数据结构。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class TaskState(str, Enum):
    IDLE = "待机"
    INITIALIZING = "初始化"
    TASK1 = "任务一"
    TASK3 = "3D识别抓取"
    STOPPING = "停止中"
    COMPLETED = "已完成"
    FAILED = "失败"


@dataclass(slots=True)
class Detection:
    task: str
    object_id: str
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    color: str = "未知"
    shape: str = "未知"
    pixel_center: tuple[int, int] = (0, 0)
    oriented_bbox: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ] | None = None
    image_angle_deg: float | None = None
    pick_rz_deg: float | None = None
    depth_mm: float | None = None
    camera_xyz_mm: tuple[float, float, float] | None = None
    robot_xyz_mm: tuple[float, float, float] | None = None
    route_key: str = "default"
    status: str = "已识别"
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    # 真机由 pyrealsense2 提供的原生 intrinsics。保留它才能让 SDK 按实际
    # 畸变模型反投影；模拟和纯数学测试保持 None，使用针孔公式。
    native: Any | None = None


@dataclass(slots=True)
class DirectPlaceTarget:
    """P1 抓取后以相同 Z 直接移动到 P2 的一次抓放参数。"""

    task: str
    object_id: str
    pick_x_mm: float
    pick_y_mm: float
    pick_z_mm: float
    pick_rz_deg: float
    place_x_mm: float
    place_y_mm: float
    place_rx_deg: float
    place_ry_deg: float
    place_rz_deg: float
    route_key: str


@dataclass(slots=True)
class RobotReply:
    command_id: str
    status: str
    message: str = ""
    raw: dict[str, Any] = field(default_factory=dict)
