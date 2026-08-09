# -*- coding: utf-8 -*-
"""线程安全的轻量事件总线，用于业务线程向 PyQt5 界面推送状态。"""

from __future__ import annotations

import threading
from collections import defaultdict
from collections.abc import Callable
from typing import Any


class EventBus:
    def __init__(self) -> None:
        self._callbacks: dict[str, list[Callable[..., None]]] = defaultdict(list)
        self._lock = threading.RLock()

    def subscribe(self, event: str, callback: Callable[..., None]) -> None:
        with self._lock:
            self._callbacks[event].append(callback)

    def emit(self, event: str, *args: Any, **kwargs: Any) -> None:
        with self._lock:
            callbacks = tuple(self._callbacks.get(event, ()))
        for callback in callbacks:
            try:
                callback(*args, **kwargs)
            except Exception:
                # 界面回调不能反向打断实时任务；真正异常由调用模块单独记录。
                continue

