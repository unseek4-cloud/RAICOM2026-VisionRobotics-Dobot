# -*- coding: utf-8 -*-
"""Dobot Vision Studio TCP 协议测试。"""

from __future__ import annotations

import copy
import json
import socket
import sys
import time
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from raicom.config import Settings  # noqa: E402
from raicom.vision.dvs_tcp import (  # noqa: E402
    DVSReceiver,
    parse_dvs_line,
)


def _unused_local_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])
    finally:
        probe.close()


def _receiver_settings(port: int) -> Settings:
    base = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
    data = copy.deepcopy(base.as_dict())
    data["network"]["dvs"]["listen_host"] = "127.0.0.1"
    data["network"]["dvs"]["port"] = port
    data["network"]["dvs"]["accept_timeout_s"] = 0.1
    data["network"]["dvs"]["receive_timeout_s"] = 0.1
    return Settings(data, PROJECT_ROOT / "config" / "settings.yaml")


class DVSMessageParsingTests(unittest.TestCase):
    def test_json_key_value_and_csv(self) -> None:
        json_result = parse_dvs_line('{"status":"OK","a":72.3,"qr":"00123"}')
        self.assertEqual(json_result["status"], "OK")
        self.assertAlmostEqual(json_result["a"], 72.3)
        self.assertEqual(json_result["qr"], "00123")

        kv_result = parse_dvs_line("OK,a=72.3,b:145.1,字符=睿抗")
        self.assertEqual(kv_result["status"], "OK")
        self.assertAlmostEqual(kv_result["a"], 72.3)
        self.assertAlmostEqual(kv_result["b"], 145.1)
        self.assertEqual(kv_result["字符"], "睿抗")

        csv_result = parse_dvs_line("72.3,145.1,ABC")
        self.assertAlmostEqual(csv_result["val0"], 72.3)
        self.assertAlmostEqual(csv_result["val1"], 145.1)
        self.assertEqual(csv_result["val2"], "ABC")

    def test_gb18030_fallback(self) -> None:
        payload = "状态=成功,字符=睿抗".encode("gb18030")
        result = parse_dvs_line(
            payload, encoding="utf-8", fallback_encoding="gb18030"
        )
        self.assertEqual(result["状态"], "成功")
        self.assertEqual(result["字符"], "睿抗")


class DVSTCPStreamTests(unittest.TestCase):
    def test_fragmentation_and_sticky_packets(self) -> None:
        """一条消息拆开发送，后续两条合并发送，且在中文多字节中间切包。"""
        receiver = DVSReceiver(_receiver_settings(_unused_local_port()))
        client: socket.socket | None = None
        try:
            receiver.start()
            client = socket.create_connection(("127.0.0.1", receiver.port), timeout=2)

            json_line = json.dumps(
                {"status": "OK", "a": 1.2}, ensure_ascii=False
            ).encode("utf-8")
            kv_line = "状态=成功,字符=睿抗".encode("gb18030")
            csv_line = b"3.1,4.2,ABC"

            # 第一条 JSON 被拆包；第二条 GB18030 消息又在中文编码中间拆包；
            # 最后一个 sendall 同时粘入第二条尾部和完整第三条。
            client.sendall(json_line[:7])
            client.sendall(json_line[7:] + b"\n" + kv_line[:3])
            client.sendall(kv_line[3:] + b"\n" + csv_line + b"\n")

            first = receiver.wait_for_result(timeout=2)
            second = receiver.wait_for_result(timeout=2)
            third = receiver.wait_for_result(timeout=2)
            self.assertIsNotNone(first)
            self.assertIsNotNone(second)
            self.assertIsNotNone(third)
            assert first is not None and second is not None and third is not None

            self.assertEqual(first["status"], "OK")
            self.assertAlmostEqual(first["a"], 1.2)
            self.assertEqual(second["字符"], "睿抗")
            self.assertAlmostEqual(third["val1"], 4.2)
            self.assertEqual(
                [first["_seq"], second["_seq"], third["_seq"]], [1, 2, 3]
            )

            deadline = time.monotonic() + 2.0
            while not receiver.is_connected() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(receiver.is_connected())
        finally:
            if client is not None:
                client.close()
            receiver.stop()


if __name__ == "__main__":
    unittest.main()
