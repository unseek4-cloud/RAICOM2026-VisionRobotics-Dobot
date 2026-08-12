# -*- coding: utf-8 -*-
"""不连接任何硬件的机械臂模拟实现。"""

from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from typing import Any

from ..config import Settings
from ..events import EventBus
from ..types import PickTarget, RobotReply, StackPlaceTarget


class MockRobot:
    """与 :class:`LuaBridgeServer` 相同接口的安全模拟机械臂。"""

    def __init__(
        self,
        settings: Settings,
        bus: EventBus,
        logger: logging.Logger,
        simulation_world: Any | None = None,
    ) -> None:
        self.settings = settings
        self.bus = bus
        self.log = logger.getChild("robot.mock")
        self.delay_s = max(0.0, float(settings.get("simulation.command_delay_s", 0.15)))
        self._connected = threading.Event()
        self._shutdown = threading.Event()
        self._command_lock = threading.Lock()
        self.simulation_world = simulation_world
        self._holding: tuple[str, StackPlaceTarget] | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    def start(self) -> None:
        self._shutdown.clear()
        self._holding = None
        if self.simulation_world is not None:
            self.simulation_world.cancel_placement_view()
        self._connected.set()
        self.bus.emit("robot_connection", True)
        self.log.info("模拟机械臂已启动")

    def stop(self) -> None:
        self._shutdown.set()
        self._connected.clear()
        self._holding = None
        if self.simulation_world is not None:
            self.simulation_world.cancel_placement_view()
        self.bus.emit("robot_connection", False)
        self.log.info("模拟机械臂已停止")

    def wait_connected(self, timeout_s: float) -> bool:
        return self._connected.wait(max(0.0, float(timeout_s)))

    def go_photo(self) -> RobotReply:
        if self._holding is not None:
            return RobotReply("", "error", "仍吸附任务三工件，禁止直接回拍照位", {"local": True})
        return self._run("go_photo", raw={"phase": "at_photo"})

    def pick_and_place(self, target: PickTarget) -> RobotReply:
        values: dict[str, Any] = {
            "pick_x_mm": target.pick_x_mm,
            "pick_y_mm": target.pick_y_mm,
            "pick_z_mm": target.pick_z_mm,
            "place_x_mm": target.place_x_mm,
            "place_y_mm": target.place_y_mm,
            "place_down_mm": target.place_down_mm,
        }
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in values.values()
        ):
            return RobotReply(
                command_id="",
                status="error",
                message="模拟抓取坐标包含非有限数字",
                raw={"local": True},
            )
        return self._run(
            "pick_place",
            raw={
                "phase": "at_photo",
                "task": target.task,
                "object_id": target.object_id,
                "route_key": target.route_key,
                "pick": [target.pick_x_mm, target.pick_y_mm, target.pick_z_mm],
                "place": [target.place_x_mm, target.place_y_mm, target.place_down_mm],
            },
        )

    def pick_to_inspection(self, target: StackPlaceTarget) -> RobotReply:
        if self._holding is not None:
            return RobotReply("", "error", "模拟吸盘已持有工件", {"local": True})
        values = (
            target.pick_x_mm,
            target.pick_y_mm,
            target.pick_z_mm,
            target.object_height_mm,
            target.place_x_mm,
            target.place_y_mm,
            target.inspection_x_mm,
            target.inspection_y_mm,
            target.inspection_z_mm,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in values
        ):
            return RobotReply("", "error", "模拟动态抓取坐标包含非有限数字", {"local": True})
        reply = self._run(
            "pick_to_inspection",
            raw={
                "phase": "at_place_inspection",
                "holding_part": True,
                "task": target.task,
                "object_id": target.object_id,
                "route_key": target.route_key,
            },
        )
        if reply.status == "done":
            self._holding = (reply.command_id, target)
            if self.simulation_world is not None:
                self.simulation_world.begin_placement_inspection(
                    target.place_x_mm,
                    target.place_y_mm,
                    target.inspection_z_mm,
                    target.object_height_mm,
                )
        return reply

    def place_from_inspection(
        self, target: StackPlaceTarget, hold_id: str, place_z_mm: float
    ) -> RobotReply:
        if self._holding is None or self._holding[0] != str(hold_id):
            return RobotReply("", "error", "模拟动态放置 hold_id 不匹配", {"local": True})
        if (
            isinstance(place_z_mm, bool)
            or not isinstance(place_z_mm, (int, float))
            or not math.isfinite(float(place_z_mm))
        ):
            return RobotReply("", "error", "模拟动态放置 Z 无效", {"local": True})
        reply = self._run(
            "place_from_inspection",
            raw={
                "phase": "at_photo",
                "holding_part": False,
                "hold_id": str(hold_id),
                "place_z": float(place_z_mm),
                "route_key": target.route_key,
            },
        )
        if reply.status == "done":
            if self.simulation_world is not None:
                self.simulation_world.complete_placement()
            self._holding = None
        return reply

    def request_stop(self) -> None:
        """记录停止后续任务请求；当前模拟动作仍按真实 Lua 语义执行完。"""

        self.log.info("模拟机械臂收到停止后续任务请求（不抢断当前动作）")

    def _run(self, command: str, *, raw: dict[str, Any]) -> RobotReply:
        command_id = f"MOCK-{command.upper()}-{uuid.uuid4().hex}"
        if not self._connected.is_set():
            return RobotReply(command_id, "error", "模拟机械臂尚未启动", {"local": True})
        if not self._command_lock.acquire(blocking=False):
            return RobotReply(command_id, "busy", "已有模拟命令正在执行", {"local": True})
        try:
            self.bus.emit(
                "robot_status",
                {"v": 1, "id": command_id, "status": "accepted", "cmd": command},
            )
            deadline = time.monotonic() + self.delay_s
            while time.monotonic() < deadline:
                if self._shutdown.wait(timeout=min(0.02, deadline - time.monotonic())):
                    return RobotReply(
                        command_id,
                        "stopped",
                        "模拟机械臂服务已关闭",
                        {"phase": "stopped", "local": True},
                    )
            result = {"v": 1, "id": command_id, "status": "done", **raw}
            self.bus.emit("robot_status", dict(result))
            return RobotReply(command_id, "done", "", result)
        finally:
            self._command_lock.release()
