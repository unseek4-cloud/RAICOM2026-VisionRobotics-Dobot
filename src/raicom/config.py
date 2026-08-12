# -*- coding: utf-8 -*-
"""集中配置读取、路径解析和真机安全校验。"""

from __future__ import annotations

import math
import ipaddress
from collections.abc import Mapping
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover - 由入口给出更友好的环境报告
    raise ImportError("缺少 PyYAML，请执行 python -m pip install PyYAML") from exc


_MISSING = object()


class SettingsError(RuntimeError):
    """配置文件格式或字段错误。"""


class Settings:
    """YAML 配置的只读访问器。

    使用 ``get('robot.motion.pick_lift_mm')`` 读取嵌套字段；所有相对路径都以
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
        """返回真机运行前仍未解决的参数问题；空列表表示结构上可运行。

        这只是软件级防呆，不替代人工确认工作空间、实体急停和低速试运行。
        """
        issues: list[str] = []

        calibration_path = self.resolve_path("calibration.eih_yaml")
        if not calibration_path.is_file():
            issues.append(
                "缺少现场 EIH 标定文件："
                f"{calibration_path}（请把现场生成的 YAML 放到这里）"
            )

        for task_name in ("task2", "task3"):
            model_path = self.resolve_path(f"yolo.{task_name}.model")
            if not model_path.is_file():
                issues.append(f"缺少 {task_name} YOLO 模型：{model_path}")

        table_depth = self.get("calibration.table_depth_mm", None)
        table_robot_z = self.get("calibration.robot_table_touch_z_mm", None)
        if not self._is_number(table_depth) or float(table_depth) <= 0:
            issues.append("calibration.table_depth_mm 未填写（抓取区空台面 D435 深度，mm）")
        if not self._is_number(table_robot_z):
            issues.append(
                "calibration.robot_table_touch_z_mm 未填写（吸盘刚贴抓取台面时的机器人 Z，mm）"
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

        for key in ("robot.user_coordinate_index", "robot.tool_coordinate_index"):
            value = self.get(key, None)
            if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 9:
                issues.append(f"{key} 必须填写 0~9 的现场坐标系编号")

        workspace = self.get("robot.workspace_mm", {})
        for axis in ("x", "y", "z"):
            bounds = workspace.get(axis) if isinstance(workspace, Mapping) else None
            if (
                not isinstance(bounds, list)
                or len(bounds) != 2
                or not all(self._is_number(v) for v in bounds)
                or float(bounds[0]) >= float(bounds[1])
            ):
                issues.append(f"robot.workspace_mm.{axis} 必须填写有效的 [最小值, 最大值]")

        if (
            isinstance(photo_pose, list)
            and len(photo_pose) == 6
            and all(self._is_number(value) for value in photo_pose)
            and isinstance(workspace, Mapping)
        ):
            for index, axis in enumerate(("x", "y", "z")):
                bounds = workspace.get(axis)
                if (
                    isinstance(bounds, list)
                    and len(bounds) == 2
                    and all(self._is_number(value) for value in bounds)
                    and not float(bounds[0]) <= float(photo_pose[index]) <= float(bounds[1])
                ):
                    issues.append(
                        f"robot.photo_pose_mm_deg.{axis}={float(photo_pose[index]):g} "
                        f"超出 robot.workspace_mm.{axis}={bounds}"
                    )

        for task_name in ("task2", "task3"):
            points = self.get(f"robot.place_points.{task_name}", {})
            if not isinstance(points, Mapping) or not points:
                issues.append(f"robot.place_points.{task_name} 至少需要一个放置点")
                continue
            for name, point in points.items():
                if not isinstance(point, Mapping):
                    issues.append(f"放置点 {task_name}.{name} 格式错误")
                    continue
                fields = (
                    ("x_mm", "y_mm")
                    if task_name == "task3"
                    else ("x_mm", "y_mm", "down_mm")
                )
                values = [point.get(field) for field in fields]
                # default 是必须的安全兜底；其他预置颜色点可整组留空，现场用到再填写。
                if name != "default" and all(value is None for value in values):
                    continue
                for field, value in zip(fields, values):
                    if not self._is_number(value):
                        issues.append(f"放置点 {task_name}.{name}.{field} 未填写")

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

        z_up_sign = self.get("robot.motion.z_up_sign", 1)
        if z_up_sign not in (-1, 1):
            issues.append("robot.motion.z_up_sign 只能是 1 或 -1")

        for key in (
            "robot.motion.approach_mm",
            "robot.motion.pick_lift_mm",
            "robot.motion.release_retract_mm",
            "robot.vacuum.suction_wait_ms",
            "robot.vacuum.release_wait_ms",
        ):
            value = self.get(key, None)
            if not self._is_number(value) or float(value) <= 0:
                issues.append(f"{key} 必须填写大于 0 的有限数值")

        inspection_z = self.get("robot.motion.place_inspection_z_mm", None)
        if not self._is_number(inspection_z):
            issues.append("robot.motion.place_inspection_z_mm 必须填写任务三视觉观察位 Z")
        else:
            z_bounds = workspace.get("z") if isinstance(workspace, Mapping) else None
            if (
                isinstance(z_bounds, list)
                and len(z_bounds) == 2
                and all(self._is_number(value) for value in z_bounds)
                and not float(z_bounds[0]) <= float(inspection_z) <= float(z_bounds[1])
            ):
                issues.append("robot.motion.place_inspection_z_mm 超出 robot.workspace_mm.z")

        placement_numbers = {
            "tasks.task3.placement_vision.depth_min_mm": (0.0, False),
            "tasks.task3.placement_vision.place_table_depth_mm": (0.0, False),
            "tasks.task3.placement_vision.press_down_mm": (0.0, True),
            "tasks.task3.placement_vision.sample_radius_mm": (0.0, False),
            "tasks.task3.placement_vision.max_surface_spread_mm": (0.0, False),
            "tasks.task3.placement_vision.min_descent_clearance_mm": (0.0, False),
        }
        for key, (minimum, allow_equal) in placement_numbers.items():
            value = self.get(key, None)
            invalid_bound = self._is_number(value) and (
                float(value) < minimum if allow_equal else float(value) <= minimum
            )
            if not self._is_number(value) or invalid_bound:
                relation = "不小于" if allow_equal else "大于"
                issues.append(f"{key} 必须填写{relation} {minimum:g} 的有限数值")
        place_table_touch_z = self.get(
            "tasks.task3.placement_vision.place_table_touch_z_mm", None
        )
        if not self._is_number(place_table_touch_z):
            issues.append(
                "tasks.task3.placement_vision.place_table_touch_z_mm "
                "必须填写有限实测值"
            )
        for key, minimum in (
            ("tasks.task3.placement_vision.temporal_samples", 1),
            ("tasks.task3.placement_vision.min_valid_points", 3),
        ):
            value = self.get(key, None)
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                issues.append(f"{key} 必须填写不小于 {minimum} 的整数")

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
