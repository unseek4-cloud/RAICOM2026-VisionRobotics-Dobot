# -*- coding: utf-8 -*-
"""模拟 Dobot Vision Studio 向主程序发送任务一结果。

先启动 ``python main.py --demo`` 不需要本工具；要单独验证真实 TCP 接收器时，
用 ``python main.py --real`` 完成初始化后再运行本脚本（勿启动机械臂任务）。
"""

from __future__ import annotations

import argparse
import json
import socket


def main() -> int:
    parser = argparse.ArgumentParser(description="发送一条模拟 DVS UTF-8 JSON 行")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6001)
    parser.add_argument("--format", choices=("json", "kv", "csv"), default="json")
    args = parser.parse_args()

    if args.format == "json":
        payload = json.dumps(
            {
                "version": 1,
                "seq": 1,
                "task": "task1",
                "status": "OK",
                "object": "模拟工件",
                "a": 72.30,
                "b": 145.10,
                "c": 8.50,
                "qr": "DOBOT-DEMO",
                "text": "RAICOM2026",
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
    elif args.format == "kv":
        payload = "TASK1,status=OK,object=模拟工件,a=72.30,b=145.10,c=8.50,qr=DOBOT-DEMO"
    else:
        payload = "72.30,145.10,8.50"

    data = (payload + "\n").encode("utf-8")
    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        # 故意拆成两次发送，验证服务端能处理 TCP 半包。
        split = max(1, len(data) // 2)
        sock.sendall(data[:split])
        sock.sendall(data[split:])
    print(f"已发送到 {args.host}:{args.port}：{payload}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

