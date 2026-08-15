# -*- coding: utf-8 -*-
"""LuaBridgeServer 本机回环测试；不连接相机或机械臂。"""

from __future__ import annotations

import copy
import json
import logging
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from raicom.config import Settings  # noqa: E402
from raicom.events import EventBus  # noqa: E402
from raicom.robot import LuaBridgeServer, MockRobot  # noqa: E402
from raicom.types import PickTarget  # noqa: E402


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def _settings(port: int) -> Settings:
    original = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
    data = copy.deepcopy(original.as_dict())
    bridge = data["network"]["robot_bridge"]
    bridge.update(
        {
            "listen_host": "127.0.0.1",
            "port": port,
            "accept_timeout_s": 0.1,
            "receive_timeout_s": 0.1,
            "command_timeout_s": 5.0,
            "heartbeat_s": 0.25,
            "max_line_bytes": 4096,
        }
    )
    robot = data["robot"]
    robot["user_coordinate_index"] = 0
    robot["tool_coordinate_index"] = 0
    robot["photo_pose_mm_deg"] = [200.0, 0.0, 250.0, 180.0, 0.0, 0.0]
    robot["motion"].update(
        {
            "orientation_mm_deg": [180.0, 0.0, 0.0],
            "z_up_sign": 1,
            "approach_mm": 40.0,
            "pick_lift_mm": 80.0,
            "release_retract_mm": 60.0,
            "travel_speed_percent": 10,
            "pick_speed_percent": 5,
            "acceleration_percent": 20,
        }
    )
    robot["vacuum"].update(
        {
            "api": "ToolDO",
            "io_index": 1,
            "on_value": 1,
            "off_value": 0,
            "suction_wait_ms": 100,
            "release_wait_ms": 100,
            "feedback_di_index": None,
            "feedback_ok_level": 1,
            "feedback_timeout_ms": 500,
        }
    )
    return Settings(data, original.config_path)


def _encode(message: dict[str, Any]) -> bytes:
    return (
        json.dumps(message, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")


class _OneShotLua(threading.Thread):
    def __init__(self, port: int) -> None:
        super().__init__(name="test-mock-lua", daemon=True)
        self.port = port
        self.error: BaseException | None = None
        self.stop_event = threading.Event()
        self.pick_id: str | None = None
        self.did_drop = False
        self.completed: dict[str, dict[str, Any]] = {}
        self.received: list[dict[str, Any]] = []
        self.motion_accepted = threading.Event()

    def run(self) -> None:
        try:
            while not self.stop_event.is_set():
                sock = self._connect()
                if sock is None:
                    return
                try:
                    with sock:
                        if not self._session(sock):
                            continue
                except OSError:
                    # Python 主动隔离超时连接时，模拟端写旧连接失败属于预期路径。
                    continue
        except BaseException as exc:  # 测试线程必须把异常传回主线程
            self.error = exc

    def _connect(self) -> socket.socket | None:
        while not self.stop_event.is_set():
            try:
                sock = socket.create_connection(("127.0.0.1", self.port), timeout=0.5)
                sock.settimeout(0.2)
                sock.sendall(
                    _encode({"v": 1, "id": "HELLO", "status": "ready", "phase": "idle"})
                )
                return sock
            except OSError:
                time.sleep(0.02)
        return None

    def _session(self, sock: socket.socket) -> bool:
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                chunk = sock.recv(8192)
            except socket.timeout:
                continue
            if not chunk:
                return False
            buffer.extend(chunk)
            while True:
                newline = buffer.find(b"\n")
                if newline < 0:
                    break
                request = json.loads(bytes(buffer[:newline]).decode("utf-8"))
                del buffer[: newline + 1]
                if not self._handle(sock, request):
                    return False
        return True

    def _handle(self, sock: socket.socket, request: dict[str, Any]) -> bool:
        self.received.append(dict(request))
        command_id = request["id"]
        command = request["cmd"]
        if command == "ping":
            sock.sendall(_encode({"v": 1, "id": command_id, "status": "pong"}))
            return True
        if command == "stop_after_current":
            sock.sendall(
                _encode({"v": 1, "id": command_id, "status": "accepted"})
                + _encode({"v": 1, "id": command_id, "status": "done", "phase": "idle"})
            )
            return True

        if command_id in self.completed:
            assert command_id == self.pick_id
            sock.sendall(
                _encode(
                    {"v": 1, "id": command_id, "status": "accepted", "duplicate": True}
                )
                + _encode(self.completed[command_id])
            )
            return True

        accepted = _encode({"v": 1, "id": command_id, "status": "accepted"})
        split = len(accepted) // 2
        sock.sendall(accepted[:split])
        time.sleep(0.01)
        sock.sendall(accepted[split:])
        self.motion_accepted.set()
        # 为并发互斥测试留出稳定窗口；实际 Lua 执行动作远慢于此。
        time.sleep(0.12)
        self.motion_accepted.clear()
        terminal = {"v": 1, "id": command_id, "status": "done", "phase": "at_photo"}

        if command == "pick_place" and not self.did_drop:
            self.did_drop = True
            self.pick_id = command_id
            self.completed[command_id] = terminal
            # 模拟动作已经完成并缓存，但 done 发送前网络断开。
            return False

        sock.sendall(
            _encode({"v": 1, "id": command_id, "status": "running", "phase": "return_photo"})
            + _encode(terminal)
        )
        return True


def main() -> int:
    port = _free_port()
    settings = _settings(port)
    bus = EventBus()
    logger = logging.getLogger("bridge-loopback-test")
    logger.handlers[:] = [logging.NullHandler()]
    logger.setLevel(logging.DEBUG)

    bridge = LuaBridgeServer(settings, bus, logger)
    lua = _OneShotLua(port)
    bridge.start()
    lua.start()
    try:
        assert bridge.wait_connected(3.0), "模拟 Lua 未连接"
        photo_holder: dict[str, Any] = {}
        photo_thread = threading.Thread(
            target=lambda: photo_holder.setdefault("reply", bridge.go_photo()),
            daemon=True,
        )
        photo_thread.start()
        assert lua.motion_accepted.wait(2.0), "模拟 Lua 未确认首条运动命令"
        busy = bridge.go_photo()
        assert busy.status == "busy", "并发运动命令未被立即拒绝"
        photo_thread.join(timeout=3.0)
        assert not photo_thread.is_alive(), "回拍照位命令未结束"
        photo = photo_holder["reply"]
        assert photo.status == "done" and photo.raw.get("phase") == "at_photo", photo

        target = PickTarget(
            task="task2",
            object_id="obj-1",
            pick_x_mm=100.0,
            pick_y_mm=50.0,
            pick_z_mm=20.0,
            pick_rz_deg=28.0,
            place_x_mm=250.0,
            place_y_mm=120.0,
            place_down_mm=50.0,
            route_key="red",
        )
        picked = bridge.pick_and_place(target)
        assert picked.status == "done" and picked.command_id == lua.pick_id, picked
        assert lua.did_drop, "未执行断线重连路径"

        photo_request = next(item for item in lua.received if item.get("cmd") == "go_photo")
        assert photo_request["settle_ms"] == 300.0
        assert "release_retract_mm" not in photo_request
        pick_requests = [item for item in lua.received if item.get("cmd") == "pick_place"]
        assert len(pick_requests) == 2, "掉线后未恰好以同一命令重发一次"
        assert pick_requests[0] == pick_requests[1], "重连后命令 ID 或载荷发生变化"
        pick_request = pick_requests[0]
        assert pick_request["approach_z"] == 60.0
        assert pick_request["transfer_z"] == 100.0
        assert pick_request["pick_rz"] == 28.0
        assert pick_request["place_rz"] == 0.0
        assert pick_request["place_z"] == 50.0
        assert pick_request["retract_z"] == 110.0
        assert pick_request["settle_ms"] == 300.0
        assert "release_retract_mm" not in pick_request

        # 空闲时间超过 heartbeat_s；pong 后连接应继续有效。
        time.sleep(0.8)
        assert bridge.is_connected, "空闲心跳后连接意外断开"
        assert any(item.get("cmd") == "ping" for item in lua.received), "未发送空闲心跳"
        bridge.request_stop()
        stop_deadline = time.monotonic() + 1.0
        while time.monotonic() < stop_deadline and not any(
            item.get("cmd") == "stop_after_current" for item in lua.received
        ):
            time.sleep(0.01)
        assert any(
            item.get("cmd") == "stop_after_current" for item in lua.received
        ), "普通停止请求未送达模拟 Lua"

        mock = MockRobot(settings, bus, logger)
        mock.start()
        assert mock.wait_connected(0.1)
        assert mock.go_photo().status == "done"
        mock_holder: dict[str, Any] = {}
        mock_thread = threading.Thread(
            target=lambda: mock_holder.setdefault("reply", mock.pick_and_place(target)),
            daemon=True,
        )
        mock_thread.start()
        time.sleep(0.03)
        mock.request_stop()
        mock_thread.join(timeout=2.0)
        assert mock_holder["reply"].status == "done", "普通停止错误地抢断了当前模拟动作"
        mock.stop()

        # 运动结果超时后必须立即隔离旧连接；模拟 Lua 完成旧动作后才能重新连入。
        generation_before_timeout = bridge._generation
        bridge.command_timeout_s = 0.04
        timed_out = bridge.go_photo()
        assert timed_out.status == "timeout", "慢响应未触发命令超时"
        assert not bridge.is_connected, "命令超时后仍错误地保留未知状态连接"
        assert bridge.wait_connected(2.0), "模拟 Lua 完成旧动作后未重新连接"
        assert bridge._generation > generation_before_timeout, "超时后未建立新连接代次"

        print(
            "LuaBridgeServer 回环测试：PASS（互斥、拆包、粘包、同ID重连、"
            "派生Z、心跳、普通停止、超时隔离、模拟器）"
        )
        return 0
    finally:
        lua.stop_event.set()
        bridge.stop()
        lua.join(timeout=2.0)
        if lua.error is not None:
            raise lua.error


if __name__ == "__main__":
    raise SystemExit(main())
