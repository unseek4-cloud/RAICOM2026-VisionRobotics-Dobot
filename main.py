# -*- coding: utf-8 -*-
"""2026 睿抗机器视觉赛项主程序入口。

默认进入模拟模式；只有显式传入 ``--real`` 且现场必要参数完整时，
程序才会连接真实相机、YOLO 模型和 DobotStudio Pro 执行脚本。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from raicom.config import Settings, SettingsError  # noqa: E402


def _configure_utf8_console() -> None:
    """尽量让 Windows 控制台以 UTF-8 输出中文，失败时不影响程序运行。"""
    os.environ.setdefault("PYTHONUTF8", "1")
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is not None and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="3D识别抓取：YOLO-OBB + RealSense + 越疆 E6"
    )
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "settings.yaml"),
        help="配置文件路径",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--demo",
        action="store_true",
        help="强制模拟模式，不连接任何真实硬件（默认）",
    )
    mode.add_argument(
        "--real",
        action="store_true",
        help="连接真实硬件运行；需填写相机、模型、标定和机器人现场参数",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="无 PyQt5 界面运行；适合自动测试或命令行联调",
    )
    parser.add_argument(
        "--task",
        choices=("all", "task1", "task3"),
        default="all",
        help="无界面模式要运行的任务",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="只检查配置与依赖，不启动服务或硬件",
    )
    return parser


def main() -> int:
    _configure_utf8_console()
    args = build_parser().parse_args()

    try:
        settings = Settings.load(Path(args.config))
    except SettingsError as exc:
        print(f"[配置错误] {exc}", file=sys.stderr)
        return 2

    # 配置文件不能单独开启真机，必须同时显式使用 --real。
    real_mode = bool(args.real)
    if args.demo:
        real_mode = False

    if args.check:
        from raicom.environment import print_environment_report

        ok = print_environment_report(settings, real_mode=real_mode)
        return 0 if ok else 3

    if real_mode:
        issues = settings.validate_real_run()
        if issues:
            print("[配置错误] 真机必要参数尚未填写完整：", file=sys.stderr)
            for issue in issues:
                print(f"  - {issue}", file=sys.stderr)
            print("请按 README 的『比赛前必须填写』章节逐项设置。", file=sys.stderr)
            return 4

    if args.headless:
        from raicom.runtime import run_headless

        return run_headless(settings, real_mode=real_mode, task=args.task)

    try:
        from raicom.ui.main_window import run_gui
    except ImportError as exc:
        print(
            "[依赖缺失] 无法加载 PyQt5 界面。请先执行：\n"
            "  python -m pip install PyQt5==5.15.11\n"
            "或使用 --headless 运行。\n"
            f"原始错误：{exc}",
            file=sys.stderr,
        )
        return 5
    return run_gui(settings, real_mode=real_mode)


if __name__ == "__main__":
    raise SystemExit(main())
