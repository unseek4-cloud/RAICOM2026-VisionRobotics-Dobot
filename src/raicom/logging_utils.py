# -*- coding: utf-8 -*-
"""中文日志配置。"""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

from .events import EventBus


class EventLogHandler(logging.Handler):
    def __init__(self, bus: EventBus):
        super().__init__()
        self.bus = bus

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.bus.emit("log", self.format(record))
        except Exception:
            self.handleError(record)


def configure_logging(project_root: Path, bus: EventBus, level: str = "INFO") -> logging.Logger:
    logs_dir = project_root / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("raicom")
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    logger.propagate = False
    # 测试或界面重新创建运行时会再次配置同名 logger。先关闭旧文件句柄，
    # 避免 Windows 下日志文件被长期占用，也避免 ResourceWarning。
    for handler in tuple(logger.handlers):
        try:
            handler.close()
        except Exception:
            pass
    logger.handlers.clear()

    formatter = logging.Formatter(
        "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%H:%M:%S",
    )
    file_handler = RotatingFileHandler(
        logs_dir / "raicom.log",
        maxBytes=5 * 1024 * 1024,
        backupCount=5,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    event_handler = EventLogHandler(bus)
    event_handler.setFormatter(formatter)
    logger.addHandler(event_handler)
    return logger
