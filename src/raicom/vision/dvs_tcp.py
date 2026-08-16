# -*- coding: utf-8 -*-
"""Dobot Vision Studio 4.1.2 TCP 结果接收。

TCP 是字节流，不保留消息边界。本模块先按终止符完成粘包/拆包，再对完整消息
尝试 UTF-8 和 GB18030 解码，支持 JSON、键值和普通 CSV 三种现场格式。
"""

from __future__ import annotations

import csv
import json
import socket
import threading
import time
from collections import deque
from collections.abc import Mapping
from typing import Any

from raicom.config import Settings
from raicom.events import EventBus


class DVSError(RuntimeError):
    """DVS 网络或协议错误。"""


class DVSParseError(DVSError):
    """收到完整消息但内容无法解析。"""


def _log(logger: Any, level: str, message: str) -> None:
    if logger is None:
        return
    method = getattr(logger, level, None)
    if callable(method):
        method(message)
    elif callable(logger):
        logger(message)


def _scalar(value: str) -> Any:
    text = value.strip()
    if text == "":
        return ""
    lowered = text.casefold()
    if lowered in {"true", "yes"}:
        return True
    if lowered in {"false", "no"}:
        return False
    try:
        # 保留二维码等带前导 0 的文本，避免 00123 被改成 123。
        if len(text) > 1 and text[0] == "0" and text[1].isdigit() and "." not in text:
            return text
        return float(text)
    except ValueError:
        return text


def decode_dvs_bytes(
    payload: bytes, primary: str = "utf-8", fallback: str = "gb18030"
) -> str:
    """严格尝试两种编码，最后才替换坏字符以保留诊断原文。"""
    errors: list[str] = []
    for encoding in dict.fromkeys((primary, fallback)):
        try:
            return payload.decode(encoding, errors="strict")
        except (LookupError, UnicodeDecodeError) as exc:
            errors.append(f"{encoding}: {exc}")
    try:
        return payload.decode(primary, errors="replace")
    except LookupError as exc:
        raise DVSParseError(f"未知 DVS 编码 {primary!r}：{exc}") from exc


def parse_dvs_text(text: str, delimiter: str = ",") -> dict[str, Any]:
    """解析 JSON、``key=value``/``key:value`` 或 CSV。"""
    line = text.strip().lstrip("\ufeff")
    if not line:
        raise DVSParseError("DVS 消息为空")

    if line[0] in "[{":
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise DVSParseError(f"DVS JSON 语法错误：{exc.msg}") from exc
        if isinstance(value, Mapping):
            result = dict(value)
        elif isinstance(value, list):
            result = {f"val{index}": item for index, item in enumerate(value)}
        else:
            result = {"value": value}
        result.setdefault("raw", line)
        return result

    if not delimiter:
        delimiter = ","
    try:
        parts = next(csv.reader([line], delimiter=delimiter[0], skipinitialspace=True))
    except (csv.Error, StopIteration) as exc:
        raise DVSParseError(f"DVS CSV 解析失败：{exc}") from exc
    parts = [part.strip() for part in parts if part.strip()]
    if not parts:
        raise DVSParseError("DVS 消息没有有效字段")

    def looks_like_colon_pair(part: str) -> bool:
        if ":" not in part:
            return False
        key = part.split(":", 1)[0].strip()
        return bool(key) and key.casefold() not in {"http", "https"}

    has_key_value = any("=" in part for part in parts) or any(
        looks_like_colon_pair(part) for part in parts
    )
    result: dict[str, Any] = {}
    if has_key_value:
        flag_index = 0
        for part in parts:
            separator = (
                "="
                if "=" in part
                else (":" if looks_like_colon_pair(part) else None)
            )
            if separator is None:
                if "status" not in result:
                    result["status"] = _scalar(part)
                else:
                    result[f"flag{flag_index}"] = _scalar(part)
                    flag_index += 1
                continue
            key, value = part.split(separator, 1)
            key = key.strip()
            if not key:
                raise DVSParseError(f"DVS 键值字段缺少键名：{part!r}")
            result[key] = _scalar(value)
    else:
        for index, part in enumerate(parts):
            result[f"val{index}"] = _scalar(part)
    result["raw"] = line
    return result


def parse_dvs_line(
    line: str | bytes,
    *,
    delimiter: str = ",",
    encoding: str = "utf-8",
    fallback_encoding: str = "gb18030",
) -> dict[str, Any]:
    """兼容旧调用习惯的单消息解析入口。"""
    if isinstance(line, bytes):
        text = decode_dvs_bytes(line, encoding, fallback_encoding)
    else:
        text = line
    return parse_dvs_text(text, delimiter)


class DVSReceiver:
    """本机 TCP 服务端；DVS 作为客户端连接并发送任务一结果。"""

    def __init__(
        self,
        settings: Settings,
        event_bus: EventBus | None = None,
        logger: Any = None,
    ) -> None:
        self.settings = settings
        self.event_bus = event_bus
        self.logger = logger
        self.host = str(settings.get("network.dvs.listen_host", "0.0.0.0"))
        self.port = int(settings.get("network.dvs.port", 6001))
        self.accept_timeout = float(
            settings.get("network.dvs.accept_timeout_s", 1.0)
        )
        self.receive_timeout = float(
            settings.get("network.dvs.receive_timeout_s", 1.0)
        )
        self.max_line_bytes = int(settings.get("network.dvs.max_line_bytes", 65536))
        self.encoding = str(settings.get("dvs.encoding", "utf-8"))
        self.fallback_encoding = str(
            settings.get("dvs.fallback_encoding", "gb18030")
        )
        self.terminator = str(settings.get("dvs.terminator", "\n")).encode("ascii")
        self.delimiter = str(settings.get("dvs.delimiter", ","))
        self.default_timeout = float(settings.get("dvs.task1_timeout_s", 30.0))
        self.trigger_text = str(settings.get("dvs.trigger_text", "ok"))
        if not self.terminator:
            raise DVSError("dvs.terminator 不能为空")
        if not 1024 <= self.port <= 65535:
            raise DVSError("DVS 监听端口必须在 1024~65535")
        if self.max_line_bytes < 128:
            raise DVSError("network.dvs.max_line_bytes 设置过小")

        self._server: socket.socket | None = None
        self._connection: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._connected = False
        self._socket_lock = threading.RLock()
        self._condition = threading.Condition(threading.RLock())
        self._results: deque[tuple[float, int, dict[str, Any]]] = deque(maxlen=100)
        self._sequence = 0

    def _emit(self, event: str, *args: Any) -> None:
        if self.event_bus is not None:
            self.event_bus.emit(event, *args)

    def start(self) -> None:
        with self._socket_lock:
            if self._running:
                return
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                server.bind((self.host, self.port))
                server.listen(1)
                server.settimeout(self.accept_timeout)
            except Exception as exc:
                server.close()
                raise DVSError(f"DVS 监听 {self.host}:{self.port} 失败：{exc}") from exc
            self._server = server
            self._running = True
            self._thread = threading.Thread(
                target=self._server_loop, name="DVSReceiver", daemon=True
            )
            self._thread.start()
        _log(self.logger, "info", f"DVS TCP 正在监听 {self.host}:{self.port}")

    def stop(self) -> None:
        with self._socket_lock:
            self._running = False
            connection, self._connection = self._connection, None
            server, self._server = self._server, None
            self._connected = False
            for sock in (connection, server):
                if sock is not None:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                    try:
                        sock.close()
                    except OSError:
                        pass
        with self._condition:
            self._condition.notify_all()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.accept_timeout + self.receive_timeout + 0.5))
        self._emit("dvs_connection", False)
        _log(self.logger, "info", "DVS TCP 已停止")

    def is_connected(self) -> bool:
        with self._socket_lock:
            return self._connected

    def _server_loop(self) -> None:
        while self._running:
            server = self._server
            if server is None:
                break
            try:
                connection, address = server.accept()
                connection.settimeout(self.receive_timeout)
            except socket.timeout:
                continue
            except OSError as exc:
                if self._running:
                    _log(self.logger, "error", f"DVS accept 失败：{exc}")
                break
            with self._socket_lock:
                old = self._connection
                self._connection = connection
                self._connected = True
            if old is not None and old is not connection:
                try:
                    old.close()
                except OSError:
                    pass
            _log(self.logger, "info", f"DVS 已连接：{address[0]}:{address[1]}")
            self._emit("dvs_connection", True)
            try:
                self._receive_connection(connection)
            finally:
                with self._socket_lock:
                    if self._connection is connection:
                        self._connection = None
                        self._connected = False
                try:
                    connection.close()
                except OSError:
                    pass
                self._emit("dvs_connection", False)
                if self._running:
                    _log(self.logger, "warning", "DVS 已断开，继续等待重连")

    def _receive_connection(self, connection: socket.socket) -> None:
        # 缓冲区属于单次连接；断线时丢弃半包，避免与新连接的数据拼接。
        buffer = bytearray()
        while self._running:
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buffer.extend(chunk)
            if len(buffer) > self.max_line_bytes and self.terminator not in buffer:
                _log(
                    self.logger,
                    "error",
                    f"DVS 单条消息超过 {self.max_line_bytes} 字节，主动断开",
                )
                break
            while True:
                index = buffer.find(self.terminator)
                if index < 0:
                    break
                payload = bytes(buffer[:index]).rstrip(b"\r")
                del buffer[: index + len(self.terminator)]
                if not payload.strip():
                    continue
                if len(payload) > self.max_line_bytes:
                    _log(self.logger, "error", "忽略超过最大长度的 DVS 消息")
                    continue
                try:
                    result = parse_dvs_line(
                        payload,
                        delimiter=self.delimiter,
                        encoding=self.encoding,
                        fallback_encoding=self.fallback_encoding,
                    )
                except DVSParseError as exc:
                    _log(self.logger, "warning", f"忽略无法解析的 DVS 消息：{exc}")
                    self._emit("dvs_parse_error", str(exc))
                    continue
                self._publish_result(result)

    def _publish_result(self, result: dict[str, Any]) -> None:
        now = time.time()
        with self._condition:
            self._sequence += 1
            sequence = self._sequence
            enriched = dict(result)
            # 下划线字段供状态机去重，界面和结果文件会在展示前过滤。
            enriched["_seq"] = sequence
            enriched["_received_at"] = now
            self._results.append((now, sequence, enriched))
            self._condition.notify_all()
        _log(self.logger, "info", f"收到 DVS 结果 #{sequence}：{result}")
        self._emit("dvs_result", dict(enriched))

    def clear_results(self) -> None:
        with self._condition:
            self._results.clear()

    def wait_for_result(
        self,
        timeout: float | None = None,
        *,
        since: float | None = None,
        after_sequence: int | None = None,
    ) -> dict[str, Any] | None:
        """等待并消费一条结果；超时或服务停止返回 ``None``。"""
        timeout = self.default_timeout if timeout is None else max(0.0, float(timeout))
        deadline = time.monotonic() + timeout
        since_value = float("-inf") if since is None else float(since)
        sequence_value = -1 if after_sequence is None else int(after_sequence)
        with self._condition:
            while self._running:
                while self._results:
                    received_at, sequence, result = self._results.popleft()
                    if received_at > since_value and sequence > sequence_value:
                        return dict(result)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)
        return None

    def trigger(self, text: str | None = None) -> bool:
        payload_text = self.trigger_text if text is None else str(text)
        try:
            payload = payload_text.encode(self.encoding)
        except (LookupError, UnicodeEncodeError) as exc:
            raise DVSError(f"DVS 触发文本编码失败：{exc}") from exc
        with self._socket_lock:
            connection = self._connection
            if connection is None or not self._connected:
                _log(self.logger, "warning", "DVS 未连接，无法发送软触发")
                return False
            try:
                connection.sendall(payload)
                return True
            except OSError as exc:
                _log(self.logger, "error", f"DVS 软触发发送失败：{exc}")
                return False


class MockDVSReceiver:
    """与真实接收器同接口的任务一模拟源。"""

    def __init__(
        self,
        settings: Settings | None = None,
        event_bus: EventBus | None = None,
        logger: Any = None,
        result: Mapping[str, Any] | None = None,
    ) -> None:
        self.settings = settings
        self.event_bus = event_bus
        self.logger = logger
        self.result = dict(
            result
            or {
                "status": "OK",
                "a": 72.3,
                "b": 145.1,
                "c": 8.5,
                "qr": "RAICOM-2026",
                "text": "模拟工件",
                "raw": "模拟 DVS 结果",
            }
        )
        self._running = False
        self._condition = threading.Condition()
        self._queue: deque[dict[str, Any]] = deque()
        self._sequence = 0

    def start(self) -> None:
        with self._condition:
            self._running = True
            self._condition.notify_all()
        if self.event_bus is not None:
            self.event_bus.emit("dvs_connection", True)
        _log(self.logger, "info", "模拟 DVS 已启动")

    def stop(self) -> None:
        with self._condition:
            self._running = False
            self._queue.clear()
            self._condition.notify_all()
        if self.event_bus is not None:
            self.event_bus.emit("dvs_connection", False)

    def is_connected(self) -> bool:
        return self._running

    def clear_results(self) -> None:
        with self._condition:
            self._queue.clear()

    def trigger(self, text: str | None = None) -> bool:
        del text
        with self._condition:
            if not self._running:
                return False
            self._sequence += 1
            enriched = dict(self.result)
            enriched["_seq"] = self._sequence
            enriched["_received_at"] = time.time()
            self._queue.append(enriched)
            self._condition.notify_all()
        if self.event_bus is not None:
            self.event_bus.emit("dvs_result", dict(enriched))
        return True

    def wait_for_result(
        self,
        timeout: float | None = None,
        *,
        since: float | None = None,
        after_sequence: int | None = None,
    ) -> dict[str, Any] | None:
        del since, after_sequence
        timeout = 1.0 if timeout is None else max(0.0, float(timeout))
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._running:
                if self._queue:
                    return self._queue.popleft()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
        return None


# 简短别名便于装配层按 “Mock” 命名导入。
MockDVS = MockDVSReceiver


__all__ = [
    "DVSError",
    "DVSParseError",
    "DVSReceiver",
    "MockDVS",
    "MockDVSReceiver",
    "decode_dvs_bytes",
    "parse_dvs_line",
    "parse_dvs_text",
]
