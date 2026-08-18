# -*- coding: utf-8 -*-
"""集中配置读取、路径解析和真机安全校验。"""

from __future__ import annotations

import math
import ipaddress
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from .recognition_region import RecognitionRegionError, validate_recognition_region

try:
    import yaml
except ImportError as exc:  # pragma: no cover - 由入口给出更友好的环境报告
    raise ImportError("缺少 PyYAML，请执行 python -m pip install PyYAML") from exc


_MISSING = object()
PLACE_POSE_CLASSES = (
    "大圆柱",
    "正方体",
    "梯形",
    "长方体",
    "圆柱",
    "六棱柱",
    "平行四边形",
)


class SettingsError(RuntimeError):
    """配置文件格式或字段错误。"""


class Settings:
    """YAML 配置的只读访问器。

    使用 ``get('robot.photo_pose_mm_deg')`` 读取嵌套字段；所有相对路径都以
    项目根目录（配置文件的上一级目录）为基准，避免从不同工作目录启动时失效。
    """

    def __init__(self, data: Mapping[str, Any], config_path: Path):
        self._data = dict(data)
        self.config_path = config_path.resolve()
        self.project_root = self.config_path.parent.parent.resolve()

    @classmethod
    def load(cls, path: Path) -> "Settings":
        path = path.expanduser().resolve()
        if not path.exists():
            raise SettingsError(f"配置文件不存在：{path}")
        try:
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
        except UnicodeDecodeError as exc:
            raise SettingsError(f"配置文件必须保存为 UTF-8：{path}") from exc
        except yaml.YAMLError as exc:
            raise SettingsError(f"YAML 语法错误：{exc}") from exc
        if not isinstance(data, Mapping):
            raise SettingsError("配置文件顶层必须是键值结构")
        settings = cls(data, path)
        settings._validate_structure()
        return settings

    def get(self, dotted_key: str, default: Any = _MISSING) -> Any:
        node: Any = self._data
        for part in dotted_key.split("."):
            if not isinstance(node, Mapping) or part not in node:
                if default is _MISSING:
                    raise SettingsError(f"缺少配置项：{dotted_key}")
                return default
            node = node[part]
        return node

    def section(self, dotted_key: str) -> dict[str, Any]:
        value = self.get(dotted_key)
        if not isinstance(value, Mapping):
            raise SettingsError(f"配置项应为对象：{dotted_key}")
        return dict(value)

    def resolve_path(self, dotted_key: str, *, required: bool = False) -> Path:
        raw = self.get(dotted_key, "")
        if raw in (None, ""):
            if required:
                raise SettingsError(f"路径配置不能为空：{dotted_key}")
            return self.project_root
        path = Path(str(raw)).expanduser()
        if not path.is_absolute():
            path = self.project_root / path
        return path.resolve()

    def as_dict(self) -> dict[str, Any]:
        return dict(self._data)

    def task_max_objects(self, task_name: str) -> int:
        """返回 3D 识别抓取单次运行允许分拣的最大工件数。"""

        if task_name != "task3":
            raise SettingsError(f"不支持配置分拣数量的任务：{task_name}")
        key = f"tasks.{task_name}.max_objects"
        value = self.get(key, None)
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise SettingsError(f"{key} 必须是不小于 1 的整数")
        return value

    def task_recognition_region(self, task_name: str) -> tuple[float, float, float, float]:
        """返回 3D 识别抓取的归一化识别区域。"""

        if task_name != "task3":
            raise SettingsError(f"不支持配置识别区域的任务：{task_name}")
        key = f"tasks.{task_name}.recognition_region"
        try:
            return validate_recognition_region(self.get(key, [0.0, 0.0, 1.0, 1.0]))
        except RecognitionRegionError as exc:
            raise SettingsError(f"{key} 无效：{exc}") from exc

    def _validate_structure(self) -> None:
        required_sections = (
            "application",
            "network",
            "dvs",
            "camera",
            "yolo",
            "calibration",
            "robot",
            "tasks",
        )
        for key in required_sections:
            if not isinstance(self.get(key, None), Mapping):
                raise SettingsError(f"缺少配置节或类型错误：{key}")

        self.task_max_objects("task3")
        self.task_recognition_region("task3")

        try:
            timeout = float(self.get("application.competition_timeout_s", 600))
        except (TypeError, ValueError) as exc:
            raise SettingsError("application.competition_timeout_s 必须是数字") from exc
        if not 1 <= timeout <= 600:
            raise SettingsError("application.competition_timeout_s 必须在 1~600 秒")

        for port_key in ("network.dvs.port", "network.robot_bridge.port"):
            try:
                raw_port = self.get(port_key)
                if isinstance(raw_port, bool):
                    raise TypeError
                port = int(raw_port)
            except (TypeError, ValueError) as exc:
                raise SettingsError(f"{port_key} 必须是整数") from exc
            if not 1024 <= port <= 65535:
                raise SettingsError(f"{port_key} 必须在 1024~65535")
            if port_key == "network.robot_bridge.port":
                if port > 59999:
                    raise SettingsError(
                        "network.robot_bridge.port 为避免控制器保留高位端口，必须在 1024~59999"
                    )
                forbidden = {
                    1501, 1502, 1503, 4840, 8172, 9527, 11740,
                    22000, 22001, 29999, 30004, 30005, 30006,
                    65506, 65521, 65522,
                }
                if port in forbidden:
                    raise SettingsError(
                        f"{port_key}={port} 与 DobotStudio Pro/控制器保留端口冲突"
                    )

    @staticmethod
    def _is_number(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )

    def validate_real_run(self) -> list[str]:
        """返回真机运行前仍未解决的必需配置问题。"""
        issues: list[str] = []

        calibration_path = self.resolve_path("calibration.eih_yaml")
        if not calibration_path.is_file():
            issues.append(
                "缺少现场 EIH 标定文件："
                f"{calibration_path}（请把现场生成的 YAML 放到这里）"
            )

        model_path = self.resolve_path("yolo.task3.model")
        if not model_path.is_file():
            issues.append(f"缺少 3D识别抓取 YOLO 模型：{model_path}")

        table_depth = self.get("calibration.table_depth_mm", None)
        table_robot_z = self.get("calibration.robot_table_touch_z_mm", None)
        if not self._is_number(table_depth) or float(table_depth) <= 0:
            issues.append("calibration.table_depth_mm 未填写（检测/放置共用空台面深度，mm）")
        if not self._is_number(table_robot_z):
            issues.append(
                "calibration.robot_table_touch_z_mm 未填写（吸盘刚贴共用台面时的机器人 Z，mm）"
            )

        photo_pose = self.get("robot.photo_pose_mm_deg", None)
        if (
            not isinstance(photo_pose, list)
            or len(photo_pose) != 6
            or not all(self._is_number(v) for v in photo_pose)
        ):
            issues.append("robot.photo_pose_mm_deg 必须填写 6 个实测值 [X,Y,Z,Rx,Ry,Rz]")

        orientation = self.get("robot.motion.orientation_mm_deg", None)
        if (
            not isinstance(orientation, list)
            or len(orientation) != 3
            or not all(self._is_number(v) for v in orientation)
        ):
            issues.append("robot.motion.orientation_mm_deg 必须填写吸盘朝下姿态 [Rx,Ry,Rz]")
        elif not math.isclose(float(orientation[2]), 0.0, abs_tol=1e-6):
            issues.append("robot.motion.orientation_mm_deg[2] 必须为 0（抓取动态 RZ 的基准）")

        for key in ("robot.user_coordinate_index", "robot.tool_coordinate_index"):
            value = self.get(key, None)
            if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 9:
                issues.append(f"{key} 必须填写 0~9 的现场坐标系编号")

        for key in ("robot.photo_pose_tolerance_mm", "robot.photo_pose_tolerance_deg"):
            value = self.get(key, None)
            if not self._is_number(value) or float(value) <= 0:
                issues.append(f"{key} 必须填写大于 0 的有限数值")

        poses = self.get("robot.place_poses_mm_deg", {})
        if not isinstance(poses, Mapping):
            issues.append("robot.place_poses_mm_deg 必须列出七类工件放置位姿")
        else:
            for class_name in PLACE_POSE_CLASSES:
                pose = poses.get(class_name)
                key = f"robot.place_poses_mm_deg.{class_name}"
                if not isinstance(pose, list) or len(pose) != 6:
                    issues.append(f"{key} 必须为 [X,Y,Z,Rx,Ry,Rz]")
                    continue
                if pose[2] is not None:
                    issues.append(f"{key}[2] 必须为 null（P2.Z 直接复用 P1.Z）")
                for index, field in ((0, "X"), (1, "Y"), (3, "Rx"), (4, "Ry"), (5, "Rz")):
                    if not self._is_number(pose[index]):
                        issues.append(f"{key} 的 {field} 未填写有效数值")

        pc_server_ip = str(self.get("robot.lua.pc_server_ip", "")).strip()
        if not pc_server_ip:
            issues.append("robot.lua.pc_server_ip 未填写（运行 Python 的电脑有线网卡 IP）")
        else:
            try:
                parsed_ip = ipaddress.ip_address(pc_server_ip)
                if parsed_ip.version != 4 or parsed_ip.is_loopback or parsed_ip.is_unspecified:
                    raise ValueError
            except ValueError:
                issues.append("robot.lua.pc_server_ip 必须是电脑网卡的非回环 IPv4 地址")

        lua_port = self.get("robot.lua.pc_server_port", None)
        bridge_port = self.get("network.robot_bridge.port", None)
        if lua_port != bridge_port:
            issues.append("robot.lua.pc_server_port 必须与 network.robot_bridge.port 完全一致")

        for key in (
            "robot.vacuum.suction_wait_ms",
            "robot.vacuum.release_wait_ms",
        ):
            value = self.get(key, None)
            if not self._is_number(value) or float(value) <= 0:
                issues.append(f"{key} 必须填写大于 0 的有限数值")

        angle_tolerance = self.get("tasks.task3.stable_angle_tolerance_deg", None)
        if (
            not self._is_number(angle_tolerance)
            or not 0 < float(angle_tolerance) <= 45
        ):
            issues.append("tasks.task3.stable_angle_tolerance_deg 必须在 (0,45]")

        for key in (
            "robot.motion.travel_speed_percent",
            "robot.motion.pick_speed_percent",
            "robot.motion.acceleration_percent",
        ):
            value = self.get(key, None)
            if not self._is_number(value) or not 0 < float(value) <= 100:
                issues.append(f"{key} 必须在 (0,100]")

        io_index = self.get("robot.vacuum.io_index", None)
        if not isinstance(io_index, int) or isinstance(io_index, bool) or io_index < 1:
            issues.append("robot.vacuum.io_index 未确认")
        vacuum_api = self.get("robot.vacuum.api", None)
        if vacuum_api not in ("ToolDO", "DO"):
            issues.append('robot.vacuum.api 必须按接线填写 "ToolDO" 或 "DO"')
        elif isinstance(io_index, int) and not isinstance(io_index, bool):
            maximum = 2 if vacuum_api == "ToolDO" else 16
            if not 1 <= io_index <= maximum:
                issues.append(f"robot.vacuum.io_index 对 {vacuum_api} 必须在 1~{maximum}")

        on_value = self.get("robot.vacuum.on_value", None)
        off_value = self.get("robot.vacuum.off_value", None)
        if on_value not in (0, 1) or off_value not in (0, 1) or on_value == off_value:
            issues.append("robot.vacuum.on_value/off_value 必须是互不相同的 0/1")

        return issues
