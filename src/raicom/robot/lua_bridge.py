# -*- coding: utf-8 -*-
"""Python 与 DobotStudio Pro Lua 工程之间的 TCP 桥接。

本模块只负责应用层通信，不直接连接越疆控制器的 29999 端口。Python 是
TCP 服务端，运行在 E6 控制器中的 Lua 工程是 TCP 客户端。协议采用 UTF-8
NDJSON（一行一个 JSON 对象），因此接收端必须同时处理 TCP 半包和粘包。

安全约束：

* 同一时刻只允许一个运动命令在途；
* 每个命令必须先收到 ``accepted``，再收到最终 ``done``/``error``；
* 抓放和回拍照位只有在 Lua 明确返回 ``done`` 且 ``phase=at_photo`` 时成功；
* 连接中断后使用同一个命令 ID 重发。Lua 会缓存最近完成 ID，避免重复抓取；
* ``request_stop`` 只是停止后续任务的普通请求，不能替代实体急停。
"""

from __future__ import annotations

import json
import logging
import math
import socket
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any

from ..config import Settings
from ..events import EventBus
from ..types import DirectPlaceTarget, RobotReply


PROTOCOL_VERSION = 1
_TERMINAL_STATUSES = frozenset({"done", "error", "rejected", "busy", "stopped"})


@dataclass(slots=True)
class _PendingCommand:
    """一个等待 Lua 确认的命令。所有字段都在 ``_condition`` 下访问。"""

    command_id: str
    payload: dict[str, Any]
    accepted: bool = False
    terminal: dict[str, Any] | None = None
    last_generation_sent: int = -1
    phases: list[str] = field(default_factory=list)


class LuaBridgeServer:
    """监听 DobotStudio Lua 客户端并实现 :class:`RobotLike` 接口。"""

    def __init__(self, settings: Settings, bus: EventBus, logger: logging.Logger) -> None:
        self.settings = settings
        self.bus = bus
        self.log = logger.getChild("robot.lua_bridge")

        self.host = str(settings.get("network.robot_bridge.listen_host", "0.0.0.0"))
        self.port = int(settings.get("network.robot_bridge.port", 2006))
        self.accept_timeout_s = max(
            0.1, float(settings.get("network.robot_bridge.accept_timeout_s", 1.0))
        )
        self.receive_timeout_s = max(
            0.1, float(settings.get("network.robot_bridge.receive_timeout_s", 1.0))
        )
        self.command_timeout_s = max(
            0.1, float(settings.get("network.robot_bridge.command_timeout_s", 120.0))
        )
        self.heartbeat_s = max(
            0.0, float(settings.get("network.robot_bridge.heartbeat_s", 5.0))
        )
        self.max_line_bytes = int(
            settings.get("network.robot_bridge.max_line_bytes", 65536)
        )
        if self.max_line_bytes < 256:
            raise ValueError("network.robot_bridge.max_line_bytes 不能小于 256")

        self._condition = threading.Condition(threading.RLock())
        self._send_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._connected = threading.Event()
        self._server_thread: threading.Thread | None = None
        self._listener: socket.socket | None = None
        self._client: socket.socket | None = None
        self._client_address: tuple[str, int] | None = None
        self._generation = 0
        self._pending: dict[str, _PendingCommand] = {}

    @property
    def is_connected(self) -> bool:
        """Lua TCP 客户端当前是否已连接。"""

        return self._connected.is_set()

    def start(self) -> None:
        """后台启动 TCP 服务；重复调用不会创建第二个监听线程。"""

        with self._condition:
            if self._server_thread is not None and self._server_thread.is_alive():
                return
            self._shutdown.clear()
            self._server_thread = threading.Thread(
                target=self._serve,
                name="raicom-lua-bridge",
                daemon=True,
            )
            self._server_thread.start()

    def stop(self) -> None:
        """停止应用层桥接并关闭套接字，不向机器人追加运动命令。"""

        self._shutdown.set()
        with self._condition:
            listener = self._listener
            client = self._client
            self._condition.notify_all()
        self._close_socket(client)
        self._close_socket(listener)

        thread = self._server_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(2.0, self.accept_timeout_s + self.receive_timeout_s + 0.5))

        with self._condition:
            self._listener = None
            self._client = None
            self._client_address = None
            self._server_thread = None
            self._connected.clear()
            self._condition.notify_all()
        self.bus.emit("robot_connection", False)

    def wait_connected(self, timeout_s: float) -> bool:
        """等待 Lua 连接；``timeout_s<=0`` 时只读取当前状态。"""

        timeout_s = max(0.0, float(timeout_s))
        return self._connected.wait(timeout_s)

    def go_photo(self) -> RobotReply:
        """请求机械臂回到 Lua 现场配置的固定拍照位。"""

        fields, error = self._base_command_fields()
        if error:
            return self._local_error(self._new_command_id("CONFIG"), error)
        return self._execute_command(
            "go_photo", fields, required_phase="at_photo", required_holding=None
        )

    def is_at_photo(self) -> RobotReply:
        """读取真实 TCP 位姿并查询是否位于固定拍照位，不产生运动。"""

        fields, error = self._base_command_fields()
        if error:
            return self._local_error(self._new_command_id("CONFIG"), error)
        return self._execute_command(
            "check_photo", fields, required_phase=None, required_holding=None
        )

    def pick_and_place_direct(self, target: DirectPlaceTarget) -> RobotReply:
        """以 DobotStudio Pro MovJ 从识别抓取点 P1 直接到同 Z 放置点 P2。"""

        numeric = {
            "pick_x": target.pick_x_mm,
            "pick_y": target.pick_y_mm,
            "pick_z": target.pick_z_mm,
            "pick_rz": target.pick_rz_deg,
            "place_x": target.place_x_mm,
            "place_y": target.place_y_mm,
            "place_z": target.pick_z_mm,
            "place_rx": target.place_rx_deg,
            "place_ry": target.place_ry_deg,
            "place_rz": target.place_rz_deg,
        }
        error = self._validate_numeric_fields(numeric)
        if error:
            return self._local_error("", error)
        if not -90.0 <= float(target.pick_rz_deg) < 90.0:
            return self._local_error("", "pick_rz 必须是 [-90,90) 内的最短旋转角")

        context, error = self._base_command_fields()
        if error:
            return self._local_error(self._new_command_id("CONFIG"), error)
        runtime_context, error = self._runtime_context()
        if error:
            return self._local_error(self._new_command_id("CONFIG"), error)
        orientation = runtime_context["orientation"]
        payload: dict[str, Any] = {
            **{key: float(value) for key, value in numeric.items()},
            **context,
            "task": str(target.task),
            "object_id": str(target.object_id),
            "route_key": str(target.route_key),
            "pick_rx": orientation[0],
            "pick_ry": orientation[1],
            "pick_rz": float(target.pick_rz_deg),
        }
        return self._execute_command(
            "pick_place_direct",
            payload,
            required_phase="at_photo",
            required_holding=False,
        )

    def _base_command_fields(self) -> tuple[dict[str, Any], str]:
        context, error = self._runtime_context()
        if error:
            return {}, error
        photo = context.pop("photo_pose")
        context.pop("orientation")
        return {
            **context,
            "photo_x": photo[0],
            "photo_y": photo[1],
            "photo_z": photo[2],
            "photo_rx": photo[3],
            "photo_ry": photo[4],
            "photo_rz": photo[5],
        }, ""

    @staticmethod
    def _validate_numeric_fields(fields: dict[str, Any]) -> str:
        for name, value in fields.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return f"坐标字段 {name} 不是数字"
            if not math.isfinite(float(value)):
                return f"坐标字段 {name} 为 NaN/Inf"
        return ""

    def request_stop(self) -> None:
        """尽力通知 Lua 停止后续任务。

        该请求不会抢断正在执行的运动，也不是物理急停。Lua 主线程若正在抓放，
        会在当前动作完成并回拍照位后才读到它。
        """

        command_id = self._new_command_id("STOP")
        payload = {
            "v": PROTOCOL_VERSION,
            "id": command_id,
            "cmd": "stop_after_current",
        }
        with self._condition:
            client = self._client
        if client is None:
            self.log.info("已记录停止后续任务请求；Lua 当前未连接")
            return
        try:
            self._send_payload(client, payload)
            self.log.info("已向 Lua 发送停止后续任务请求（不是物理急停）")
        except (OSError, ValueError) as exc:
            self.log.warning("停止后续任务请求发送失败：%s", exc)
            self._disconnect_client(client)

    def _runtime_context(self) -> tuple[dict[str, Any], str]:
        """读取并校验真机命令需要的现场参数。

        ``Settings.validate_real_run`` 会在主入口做第一层拦截；这里再次检查，保证
        单独实例化桥接类或测试代码也不可能把 ``null`` 变成运动参数发送给 Lua。
        """

        def finite_number(key: str, *, positive: bool = False) -> tuple[float | None, str]:
            value = self.settings.get(key, None)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                return None, f"现场配置 {key} 未填写有效数字"
            result = float(value)
            if positive and result <= 0:
                return None, f"现场配置 {key} 必须大于 0"
            return result, ""

        user = self.settings.get("robot.user_coordinate_index", None)
        tool = self.settings.get("robot.tool_coordinate_index", None)
        if isinstance(user, bool) or not isinstance(user, int) or not 0 <= user <= 9:
            return {}, "robot.user_coordinate_index 必须填写 0~9 的现场坐标系编号"
        if isinstance(tool, bool) or not isinstance(tool, int) or not 0 <= tool <= 9:
            return {}, "robot.tool_coordinate_index 必须填写 0~9 的现场坐标系编号"

        photo = self.settings.get("robot.photo_pose_mm_deg", None)
        if not self._finite_sequence(photo, 6):
            return {}, "robot.photo_pose_mm_deg 必须填写 6 个有限实测值"
        orientation = self.settings.get("robot.motion.orientation_mm_deg", None)
        if not self._finite_sequence(orientation, 3):
            return {}, "robot.motion.orientation_mm_deg 必须填写 3 个有限姿态值"
        if not math.isclose(float(orientation[2]), 0.0, abs_tol=1e-6):
            return {}, "robot.motion.orientation_mm_deg 的 RZ 必须为 0（抓取动态 RZ 基准）"

        photo_tolerance_mm, error = finite_number(
            "robot.photo_pose_tolerance_mm", positive=True
        )
        if error:
            return {}, error
        photo_tolerance_deg, error = finite_number(
            "robot.photo_pose_tolerance_deg", positive=True
        )
        if error:
            return {}, error
        travel_v, error = finite_number("robot.motion.travel_speed_percent", positive=True)
        if error:
            return {}, error
        pick_v, error = finite_number("robot.motion.pick_speed_percent", positive=True)
        if error:
            return {}, error
        acceleration, error = finite_number(
            "robot.motion.acceleration_percent", positive=True
        )
        if error:
            return {}, error
        settle_ms, error = finite_number("robot.motion.settle_ms")
        if error:
            return {}, error
        if settle_ms is None or settle_ms < 0:
            return {}, "现场配置 robot.motion.settle_ms 必须大于或等于 0"
        for key, value in (
            ("robot.motion.travel_speed_percent", travel_v),
            ("robot.motion.pick_speed_percent", pick_v),
            ("robot.motion.acceleration_percent", acceleration),
        ):
            if value is None or value > 100:
                return {}, f"现场配置 {key} 必须在 (0,100]"

        vacuum_api = self.settings.get("robot.vacuum.api", None)
        if vacuum_api not in ("ToolDO", "DO"):
            return {}, 'robot.vacuum.api 必须按接线填写 "ToolDO" 或 "DO"'
        vacuum_io = self.settings.get("robot.vacuum.io_index", None)
        if isinstance(vacuum_io, bool) or not isinstance(vacuum_io, int):
            return {}, "robot.vacuum.io_index 尚未填写"
        maximum_io = 2 if vacuum_api == "ToolDO" else 16
        if not 1 <= vacuum_io <= maximum_io:
            return {}, f"{vacuum_api} 输出索引必须在 1~{maximum_io}"

        on_level = self.settings.get("robot.vacuum.on_value", None)
        off_level = self.settings.get("robot.vacuum.off_value", None)
        if on_level not in (0, 1) or off_level not in (0, 1) or on_level == off_level:
            return {}, "robot.vacuum.on_value/off_value 必须是互不相同的 0/1"
        suction_wait, error = finite_number("robot.vacuum.suction_wait_ms", positive=True)
        if error:
            return {}, error
        release_wait, error = finite_number("robot.vacuum.release_wait_ms", positive=True)
        if error:
            return {}, error

        feedback_index = self.settings.get("robot.vacuum.feedback_di_index", None)
        feedback_enabled = feedback_index is not None
        if feedback_enabled:
            if isinstance(feedback_index, bool) or not isinstance(feedback_index, int):
                return {}, "robot.vacuum.feedback_di_index 必须是整数或 null"
            if not 1 <= feedback_index <= maximum_io:
                return {}, f"真空反馈 DI 索引必须在 1~{maximum_io}"
        feedback_level = self.settings.get("robot.vacuum.feedback_ok_level", 1)
        if feedback_level not in (0, 1):
            return {}, "robot.vacuum.feedback_ok_level 必须为 0 或 1"
        feedback_timeout, error = finite_number(
            "robot.vacuum.feedback_timeout_ms", positive=True
        )
        if error:
            return {}, error

        return {
            "user": user,
            "tool": tool,
            "photo_pose": tuple(float(value) for value in photo),
            "orientation": tuple(float(value) for value in orientation),
            "photo_tolerance_mm": float(photo_tolerance_mm),
            "photo_tolerance_deg": float(photo_tolerance_deg),
            "travel_v": float(travel_v),
            "pick_v": float(pick_v),
            "accel": float(acceleration),
            "settle_ms": float(settle_ms),
            "vacuum_api": vacuum_api,
            "vacuum_io": vacuum_io,
            "vacuum_on_level": on_level,
            "vacuum_off_level": off_level,
            "vacuum_suction_wait_ms": float(suction_wait),
            "vacuum_release_wait_ms": float(release_wait),
            "vacuum_feedback_enabled": feedback_enabled,
            "vacuum_feedback_di": feedback_index,
            "vacuum_feedback_level": feedback_level,
            "vacuum_feedback_timeout_ms": float(feedback_timeout),
        }, ""

    @staticmethod
    def _finite_sequence(value: Any, length: int) -> bool:
        return (
            isinstance(value, (list, tuple))
            and len(value) == length
            and all(
                not isinstance(item, bool)
                and isinstance(item, (int, float))
                and math.isfinite(float(item))
                for item in value
            )
        )

    def _execute_command(
        self,
        command: str,
        fields: dict[str, Any],
        *,
        required_phase: str | None,
        required_holding: bool | None,
    ) -> RobotReply:
        if self._shutdown.is_set():
            return self._local_error("", "Lua 桥接服务未运行", status="stopped")

        # 非阻塞加锁：调用方若错误地并发控制机器人，第二条命令立即被拒绝，
        # 不允许它静默排队并在很久以后突然运动。
        if not self._command_lock.acquire(blocking=False):
            return self._local_error("", "已有机械臂命令正在执行", status="busy")

        command_id = self._new_command_id(command.upper())
        payload = {"v": PROTOCOL_VERSION, "id": command_id, "cmd": command, **fields}
        pending = _PendingCommand(command_id=command_id, payload=payload)
        deadline = time.monotonic() + self.command_timeout_s

        try:
            # 在进入等待前验证 JSON 可编码，禁止 NaN/Inf 或不可序列化对象进入协议。
            self._encode_payload(payload)
            with self._condition:
                self._pending[command_id] = pending
                self._condition.notify_all()

            while True:
                timeout_expired = False
                timed_out_client: socket.socket | None = None
                with self._condition:
                    if pending.terminal is not None:
                        terminal = dict(pending.terminal)
                        if "phase" not in terminal and pending.phases:
                            terminal["phase"] = pending.phases[-1]
                        accepted = pending.accepted
                        break
                    if self._shutdown.is_set():
                        return self._local_error(
                            command_id, "Lua 桥接服务已停止", status="stopped"
                        )

                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        # 超时后机械臂真实状态未知，必须先隔离当前连接，不能让下一
                        # 条命令排在可能仍在执行的动作后面。Lua 会在当前动作结束、
                        # 写终态失败后退出会话并重新连接。
                        timeout_expired = True
                        timed_out_client = self._client
                    else:
                        client = self._client
                        generation = self._generation
                        should_send = (
                            client is not None
                            and pending.last_generation_sent != generation
                        )
                        if should_send:
                            # 先记录代次，避免两个唤醒周期在同一连接重复发送。若发送
                            # 失败，断线重连后 generation 会变化并使用同一个 ID 重发。
                            pending.last_generation_sent = generation
                        else:
                            self._condition.wait(timeout=min(0.25, remaining))
                            continue

                if timeout_expired:
                    self._disconnect_client(timed_out_client)
                    return self._local_error(
                        command_id,
                        f"等待 Lua 执行结果超过 {self.command_timeout_s:.1f} 秒；"
                        "已隔离连接，须等待执行脚本完成当前动作并重新连接",
                        status="timeout",
                    )

                try:
                    self._send_payload(client, payload)
                    self.log.info("已发送 Lua 命令 %s（id=%s）", command, command_id)
                except (OSError, ValueError) as exc:
                    self.log.warning("Lua 命令发送失败，等待重连后以同一 ID 重试：%s", exc)
                    self._disconnect_client(client)

            if not accepted:
                return self._local_error(
                    command_id,
                    "Lua 未先返回 accepted 就发送了终态，协议顺序无效",
                    raw=terminal,
                )

            status = str(terminal.get("status", "error"))
            if status == "done" and required_phase is not None:
                if terminal.get("phase") != required_phase:
                    return self._local_error(
                        command_id,
                        f"Lua 返回 done，但未确认阶段 {required_phase}",
                        raw=terminal,
                    )
            if status == "done" and required_holding is not None:
                if terminal.get("holding_part") is not required_holding:
                    return self._local_error(
                        command_id,
                        "Lua 返回 done，但持件状态与命令目标不一致",
                        raw=terminal,
                    )
            message = str(terminal.get("message", terminal.get("code", "")))
            return RobotReply(
                command_id=command_id,
                status=status,
                message=message,
                raw=terminal,
            )
        except (TypeError, ValueError) as exc:
            return self._local_error(command_id, f"命令序列化失败：{exc}")
        finally:
            with self._condition:
                self._pending.pop(command_id, None)
                self._condition.notify_all()
            self._command_lock.release()

    def _serve(self) -> None:
        listener: socket.socket | None = None
        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self.host, self.port))
            listener.listen(1)
            listener.settimeout(self.accept_timeout_s)
            with self._condition:
                self._listener = listener
                self._condition.notify_all()
            self.log.info("Lua 桥接已监听 %s:%d", self.host, self.port)

            while not self._shutdown.is_set():
                try:
                    client, address = listener.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._shutdown.is_set():
                        break
                    raise

                try:
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    client.settimeout(self.receive_timeout_s)
                    self._install_client(client, address)
                    self._read_client(client)
                except OSError as exc:
                    if not self._shutdown.is_set():
                        self.log.warning("Lua 连接异常：%s", exc)
                finally:
                    self._disconnect_client(client)
        except Exception as exc:
            if not self._shutdown.is_set():
                self.log.exception("Lua 桥接服务异常退出：%s", exc)
                self.bus.emit("alarm", f"Lua 桥接服务异常：{exc}")
        finally:
            self._close_socket(listener)
            with self._condition:
                if self._listener is listener:
                    self._listener = None
                self._condition.notify_all()

    def _install_client(self, client: socket.socket, address: tuple[str, int]) -> None:
        with self._condition:
            old_client = self._client
            self._client = client
            self._client_address = address
            self._generation += 1
            generation = self._generation
            self._connected.set()
            self._condition.notify_all()
        if old_client is not None and old_client is not client:
            self._close_socket(old_client)
        self.log.info(
            "DobotStudio Lua 已连接：%s:%d（连接代次 %d）",
            address[0],
            address[1],
            generation,
        )
        self.bus.emit("robot_connection", True)

    def _read_client(self, client: socket.socket) -> None:
        buffer = bytearray()
        last_activity = time.monotonic()
        ping_sent_at: float | None = None
        while not self._shutdown.is_set():
            try:
                chunk = client.recv(8192)
            except socket.timeout:
                now = time.monotonic()
                if self.heartbeat_s > 0:
                    with self._condition:
                        idle = not self._pending and self._client is client
                    if idle and ping_sent_at is not None:
                        heartbeat_timeout = max(
                            self.heartbeat_s * 2.0,
                            self.receive_timeout_s * 3.0,
                        )
                        if now - ping_sent_at >= heartbeat_timeout:
                            raise OSError("Lua 空闲心跳超时")
                    elif idle and now - last_activity >= self.heartbeat_s:
                        ping_id = self._new_command_id("PING")
                        self._send_payload(
                            client,
                            {
                                "v": PROTOCOL_VERSION,
                                "id": ping_id,
                                "cmd": "ping",
                            },
                        )
                        ping_sent_at = now
                continue
            if not chunk:
                return
            last_activity = time.monotonic()
            ping_sent_at = None
            buffer.extend(chunk)

            while True:
                newline = buffer.find(b"\n")
                if newline < 0:
                    break
                raw_line = bytes(buffer[:newline])
                del buffer[: newline + 1]
                if raw_line.endswith(b"\r"):
                    raw_line = raw_line[:-1]
                if not raw_line:
                    continue
                if len(raw_line) > self.max_line_bytes:
                    raise OSError(
                        f"Lua 协议行超过限制：{len(raw_line)} > {self.max_line_bytes} 字节"
                    )
                self._handle_line(raw_line)

            # 没有换行且缓存持续增长，说明对端未遵守 NDJSON 边界。关闭连接以
            # 免后续数据永远无法重新同步。
            if len(buffer) > self.max_line_bytes:
                raise OSError(
                    f"Lua 未完成协议行超过限制：{len(buffer)} > {self.max_line_bytes} 字节"
                )

    def _handle_line(self, raw_line: bytes) -> None:
        try:
            text = raw_line.decode("utf-8", errors="strict")
            message = json.loads(text)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.log.warning("忽略无效 Lua NDJSON：%s", exc)
            return
        if not isinstance(message, dict):
            self.log.warning("忽略非对象 Lua 消息：%r", message)
            return
        version = message.get("v")
        if isinstance(version, bool) or version != PROTOCOL_VERSION:
            self.log.warning("忽略协议版本不匹配的 Lua 消息：%r", message)
            return

        status = message.get("status")
        command_id = message.get("id")
        if not isinstance(status, str) or not isinstance(command_id, str):
            self.log.warning("Lua 消息缺少字符串 id/status：%r", message)
            return

        self.bus.emit("robot_status", dict(message))
        with self._condition:
            pending = self._pending.get(command_id)
            if pending is None:
                # hello、request_stop 的异步回执及已超时命令都可能没有等待者。
                self._condition.notify_all()
                if status not in {"ready", "pong", "accepted", "done"}:
                    self.log.info("收到无等待者的 Lua 状态：%s", message)
                return

            if status == "accepted":
                pending.accepted = True
            elif status == "running":
                phase = message.get("phase")
                if isinstance(phase, str):
                    pending.phases.append(phase)
            elif status in _TERMINAL_STATUSES:
                pending.terminal = dict(message)
            else:
                self.log.warning("Lua 返回未知状态 %r（id=%s）", status, command_id)
            self._condition.notify_all()

    def _send_payload(self, client: socket.socket | None, payload: dict[str, Any]) -> None:
        if client is None:
            raise OSError("Lua 未连接")
        encoded = self._encode_payload(payload)
        with self._send_lock:
            client.sendall(encoded)

    @staticmethod
    def _encode_payload(payload: dict[str, Any]) -> bytes:
        text = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )
        return (text + "\n").encode("utf-8")

    def _disconnect_client(self, client: socket.socket | None) -> None:
        if client is None:
            return
        should_emit = False
        with self._condition:
            if self._client is client:
                self._client = None
                self._client_address = None
                self._connected.clear()
                should_emit = True
            self._condition.notify_all()
        self._close_socket(client)
        if should_emit:
            self.log.warning("DobotStudio Lua 已断开，等待自动重连")
            self.bus.emit("robot_connection", False)

    @staticmethod
    def _close_socket(sock: socket.socket | None) -> None:
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    @staticmethod
    def _new_command_id(prefix: str) -> str:
        safe_prefix = "".join(ch for ch in prefix if ch.isalnum())[:12] or "CMD"
        return f"{safe_prefix}-{uuid.uuid4().hex}"

    @staticmethod
    def _local_error(
        command_id: str,
        message: str,
        *,
        status: str = "error",
        raw: dict[str, Any] | None = None,
    ) -> RobotReply:
        data = dict(raw or {})
        data.setdefault("local", True)
        return RobotReply(
            command_id=command_id,
            status=status,
            message=message,
            raw=data,
        )
