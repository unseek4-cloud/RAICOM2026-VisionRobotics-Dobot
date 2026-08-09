# -*- coding: utf-8 -*-
"""运行环境与现场文件自检。"""

from __future__ import annotations

import importlib.util
import platform
import socket
import sys
from dataclasses import dataclass
from pathlib import Path

from .config import Settings


@dataclass(frozen=True)
class Dependency:
    import_name: str
    display_name: str
    required_demo: bool
    required_real: bool
    install_hint: str


DEPENDENCIES = (
    Dependency("numpy", "NumPy", True, True, "numpy>=1.26,<2.0"),
    Dependency("yaml", "PyYAML", True, True, "PyYAML>=6.0"),
    Dependency("cv2", "OpenCV", True, True, "opencv-contrib-python>=4.9,<5"),
    Dependency("PIL", "Pillow", True, True, "Pillow>=10"),
    Dependency("PyQt5", "PyQt5", False, True, "PyQt5==5.15.11"),
    Dependency("ultralytics", "Ultralytics", False, True, "ultralytics>=8.3"),
    Dependency("torch", "PyTorch", False, True, "按显卡/CUDA版本安装 torch"),
    Dependency("pyrealsense2", "RealSense SDK Python", False, True, "pyrealsense2>=2.55"),
)


def module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def _can_bind(host: str, port: int) -> tuple[bool, str]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        return True, "可用"
    except OSError as exc:
        return False, f"不可用：{exc}"
    finally:
        sock.close()


def collect_environment_report(settings: Settings, real_mode: bool) -> tuple[bool, list[str]]:
    lines = [
        "=" * 72,
        "睿抗视觉系统环境自检",
        f"Python：{sys.version.splitlines()[0]}",
        f"解释器：{Path(sys.executable).resolve()}",
        f"系统：{platform.platform()}",
        f"模式：{'真机候选（仍需人工安全确认）' if real_mode else '模拟'}",
        "-" * 72,
        "依赖：",
    ]
    ok = True
    for dep in DEPENDENCIES:
        exists = module_available(dep.import_name)
        required = dep.required_real if real_mode else dep.required_demo
        marker = "正常" if exists else ("缺失" if required else "可选缺失")
        lines.append(f"  [{marker}] {dep.display_name}")
        if required and not exists:
            lines.append(f"           安装提示：python -m pip install {dep.install_hint}")
            ok = False

    lines.extend(["-" * 72, "端口："])
    for name, key in (
        ("Dobot Vision Studio 接收", "network.dvs"),
        ("DobotStudio Pro Lua 桥接", "network.robot_bridge"),
    ):
        cfg = settings.section(key)
        bind_ok, reason = _can_bind(str(cfg["listen_host"]), int(cfg["port"]))
        lines.append(f"  [{'正常' if bind_ok else '冲突'}] {name} {cfg['listen_host']}:{cfg['port']} - {reason}")
        if not bind_ok:
            ok = False

    if real_mode:
        lines.extend(["-" * 72, "真机参数："])
        issues = settings.validate_real_run()
        if issues:
            ok = False
            lines.append("  [未通过] 仍有以下现场项未设置：")
            lines.extend(f"    - {issue}" for issue in issues)
        else:
            lines.append("  [结构校验通过] 仍须人工确认实体急停、工作空间与低速轨迹。")
    lines.append("=" * 72)
    return ok, lines


def print_environment_report(settings: Settings, real_mode: bool = False) -> bool:
    ok, lines = collect_environment_report(settings, real_mode)
    print("\n".join(lines))
    return ok

