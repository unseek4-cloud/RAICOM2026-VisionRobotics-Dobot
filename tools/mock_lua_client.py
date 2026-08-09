# -*- coding: utf-8 -*-
"""在没有 E6 时模拟 DobotStudio Lua 客户端。

默认会故意把 ``accepted`` 拆成两个 TCP 包，并把 ``running`` 与 ``done``
合并发送，用于验证 Python 桥接的半包/粘包处理。加 ``--drop-once`` 可模拟
动作完成时恰好断线，重连后依靠相同命令 ID 返回缓存结果。
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from typing import Any


def encode(message: dict[str, Any]) -> bytes:
    return (
        json.dumps(message, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")


class MockLuaClient:
    def __init__(self, host: str, port: int, delay_s: float, drop_once: bool) -> None:
        self.host = host
        self.port = port
        self.delay_s = max(0.0, delay_s)
        self.drop_once = drop_once
        self.dropped = False
        self.completed: dict[str, tuple[dict[str, Any], dict[str, Any]]] = {}

    def run(self) -> None:
        print(f"[模拟Lua] 将连接 {self.host}:{self.port}，按 Ctrl+C 退出")
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=5.0) as sock:
                    sock.settimeout(1.0)
                    self._session(sock)
            except (ConnectionError, OSError) as exc:
                print(f"[模拟Lua] 连接中断：{exc}；1 秒后重连")
                time.sleep(1.0)

    def _session(self, sock: socket.socket) -> None:
        sock.sendall(
            encode(
                {
                    "v": 1,
                    "id": "HELLO",
                    "status": "ready",
                    "phase": "idle",
                    "model": "Mock-E6",
                }
            )
        )
        buffer = bytearray()
        while True:
            try:
                chunk = sock.recv(8192)
            except socket.timeout:
                continue
            if not chunk:
                return
            buffer.extend(chunk)
            while True:
                newline = buffer.find(b"\n")
                if newline < 0:
                    break
                raw = bytes(buffer[:newline])
                del buffer[: newline + 1]
                if not raw:
                    continue
                request = json.loads(raw.decode("utf-8"))
                if not self._handle(sock, request):
                    return

    def _handle(self, sock: socket.socket, request: dict[str, Any]) -> bool:
        command_id = str(request.get("id", ""))
        command = str(request.get("cmd", ""))
        print(f"[模拟Lua] 收到 {command} id={command_id}")

        cached = self.completed.get(command_id)
        if cached is not None:
            original, terminal = cached
            if original != request:
                sock.sendall(
                    encode(
                        {
                            "v": 1,
                            "id": command_id,
                            "status": "error",
                            "code": "DUPLICATE_ID_CONFLICT",
                            "recoverable": False,
                        }
                    )
                )
                return True
            sock.sendall(
                encode(
                    {
                        "v": 1,
                        "id": command_id,
                        "status": "accepted",
                        "duplicate": True,
                    }
                )
                + encode(terminal)
            )
            return True

        if request.get("v") != 1 or not command_id:
            return True
        if command == "ping":
            sock.sendall(
                encode({"v": 1, "id": command_id, "status": "pong", "state": "idle"})
            )
            return True
        if command not in {"go_photo", "pick_place", "stop_after_current"}:
            sock.sendall(
                encode(
                    {
                        "v": 1,
                        "id": command_id,
                        "status": "error",
                        "code": "UNKNOWN_COMMAND",
                        "recoverable": True,
                    }
                )
            )
            return True

        accepted = encode(
            {"v": 1, "id": command_id, "status": "accepted", "cmd": command}
        )
        midpoint = max(1, len(accepted) // 2)
        sock.sendall(accepted[:midpoint])
        time.sleep(0.01)
        sock.sendall(accepted[midpoint:])
        time.sleep(self.delay_s)

        phase = "idle" if command == "stop_after_current" else "at_photo"
        terminal = {"v": 1, "id": command_id, "status": "done", "phase": phase}
        self.completed[command_id] = (dict(request), terminal)

        if self.drop_once and not self.dropped and command in {"go_photo", "pick_place"}:
            self.dropped = True
            print("[模拟Lua] 已缓存终态，故意在发送 done 前断线")
            return False

        # 两个完整 JSON 行一次 sendall，故意制造 TCP 粘包场景。
        sock.sendall(
            encode(
                {
                    "v": 1,
                    "id": command_id,
                    "status": "running",
                    "phase": "return_photo",
                }
            )
            + encode(terminal)
        )
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="模拟 DobotStudio Pro Lua TCP 客户端")
    parser.add_argument("--host", default="127.0.0.1", help="Python 桥接监听地址")
    parser.add_argument("--port", type=int, default=2006, help="Python 桥接端口")
    parser.add_argument("--delay", type=float, default=0.1, help="模拟动作耗时（秒）")
    parser.add_argument(
        "--drop-once",
        action="store_true",
        help="第一次动作完成后、发送 done 前断线，验证同 ID 幂等重连",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        MockLuaClient(args.host, args.port, args.delay, args.drop_once).run()
    except KeyboardInterrupt:
        print("\n[模拟Lua] 已退出")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

