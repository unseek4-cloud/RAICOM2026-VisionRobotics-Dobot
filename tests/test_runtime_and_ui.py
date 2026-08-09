# -*- coding: utf-8 -*-
"""主流程与中文 PyQt5 面板的离线冒烟测试。"""

from __future__ import annotations

import math
import os
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from raicom.config import Settings
from raicom.runtime import SystemRuntime
from raicom.task.orchestrator import TaskError, TaskOrchestrator


class RuntimeFlowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")

    def test_full_demo_removes_all_four_objects(self) -> None:
        """模拟模式也必须按任务一→二→三完成并重新确认桌面为空。"""

        runtime = SystemRuntime(self.settings, real_mode=False)
        try:
            self.assertTrue(runtime.start())
            world = runtime.simulation_world
            self.assertIsNotNone(world)
            self.assertEqual(len(world.objects()), 4)
            self.assertIsNotNone(runtime.orchestrator)
            self.assertTrue(runtime.orchestrator.run("all"))
            self.assertEqual(len(world.objects()), 0)
        finally:
            runtime.stop()

    def test_dvs_quality_guards(self) -> None:
        """失败标记、错任务、错误版本和非有限值不得完成任务一。"""

        valid = {"version": 1, "task": "task1", "ok": True, "a": 12.3}
        TaskOrchestrator._validate_dvs_result(valid)
        invalid_items = (
            {"ok": False},
            {"task": "task2"},
            {"version": 2},
            {"a": math.nan},
        )
        for item in invalid_items:
            with self.subTest(item=item), self.assertRaises(TaskError):
                TaskOrchestrator._validate_dvs_result(item)


class ChineseGuiSmokeTests(unittest.TestCase):
    def test_window_constructs_offscreen_without_garbled_text(self) -> None:
        """离屏创建窗口并核对关键中文控件，避免编码或 UI 导入回归。"""

        try:
            from PyQt5 import QtWidgets
            from raicom.ui.main_window import MainWindow
        except ImportError as exc:  # pragma: no cover - HKtest 中已安装
            self.skipTest(f"PyQt5 不可用：{exc}")

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
        window = MainWindow(settings, real_mode=False)
        try:
            self.assertIn("睿抗", window.windowTitle())
            self.assertEqual(window.btn_all.text(), "全流程自动运行")
            self.assertIn("非物理急停", window.btn_stop.text())
            app.processEvents()
        finally:
            window.runtime.stop()
            window.deleteLater()
            app.processEvents()


if __name__ == "__main__":
    unittest.main()
