# -*- coding: utf-8 -*-
"""主流程与中文 PyQt5 面板的离线冒烟测试。"""

from __future__ import annotations

import logging
import math
import os
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from raicom.config import Settings
from raicom.events import EventBus
from raicom.robot.lua_bridge import LuaBridgeServer
from raicom.runtime import SystemRuntime
from raicom.task.orchestrator import TaskError, TaskOrchestrator
from raicom.types import RobotReply, StackPlaceTarget


class _CapturingLuaBridge(LuaBridgeServer):
    def __init__(self, settings: Settings) -> None:
        super().__init__(settings, EventBus(), logging.getLogger("test.lua_bridge"))
        self.calls: list[tuple[str, dict[str, object], str | None, bool | None]] = []

    def _execute_command(
        self,
        command: str,
        fields: dict[str, object],
        *,
        required_phase: str | None,
        required_holding: bool | None,
    ) -> RobotReply:
        self.calls.append((command, fields, required_phase, required_holding))
        return RobotReply(
            command_id="PICKTOINSPECTION-test-hold",
            status="done",
            raw={"phase": required_phase, "holding_part": required_holding},
        )


class RuntimeFlowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")

    def test_full_demo_removes_all_four_objects(self) -> None:
        """模拟模式也必须按任务一→二→三完成并重新确认桌面为空。"""

        runtime = SystemRuntime(self.settings, real_mode=False)
        robot_statuses: list[dict[str, object]] = []
        runtime.bus.subscribe("robot_status", lambda item: robot_statuses.append(dict(item)))
        try:
            self.assertTrue(runtime.start())
            world = runtime.simulation_world
            self.assertIsNotNone(world)
            self.assertEqual(len(world.objects()), 4)
            self.assertIsNotNone(runtime.orchestrator)
            self.assertTrue(runtime.orchestrator.run("all"))
            self.assertEqual(len(world.objects()), 0)
            # 两件任务三工件放在同一 XY：首件看到独立的放置台面 Z=90，
            # 第二件必须看到首件 30 mm 高的顶面，而不是复用抓取台面 Z=100。
            self.assertAlmostEqual(world.placement_stack_height_mm(250.0, 120.0), 74.0)
            visual_releases = [
                item
                for item in robot_statuses
                if item.get("status") == "done" and "hold_id" in item
            ]
            self.assertEqual(len(visual_releases), 2)
            self.assertAlmostEqual(float(visual_releases[0]["place_z"]), 119.5)
            self.assertAlmostEqual(float(visual_releases[1]["place_z"]), 163.5)
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

    def test_lua_bridge_builds_two_phase_task3_commands(self) -> None:
        bridge = _CapturingLuaBridge(self.settings)
        target = StackPlaceTarget(
            task="task3",
            object_id="T3-1",
            pick_x_mm=200.0,
            pick_y_mm=0.0,
            pick_z_mm=120.0,
            object_height_mm=30.0,
            place_x_mm=250.0,
            place_y_mm=120.0,
            inspection_x_mm=276.0,
            inspection_y_mm=-3.0,
            inspection_z_mm=430.0,
            route_key="match",
        )

        first = bridge.pick_to_inspection(target)
        self.assertEqual(first.status, "done")
        command, fields, phase, holding = bridge.calls[-1]
        self.assertEqual(command, "pick_to_inspection")
        self.assertEqual(phase, "at_place_inspection")
        self.assertIs(holding, True)
        self.assertAlmostEqual(float(fields["approach_z"]), 160.0)
        self.assertAlmostEqual(float(fields["lift_z"]), 200.0)

        second = bridge.place_from_inspection(target, first.command_id, 149.5)
        self.assertEqual(second.status, "done")
        command, fields, phase, holding = bridge.calls[-1]
        self.assertEqual(command, "place_from_inspection")
        self.assertEqual(fields["hold_id"], first.command_id)
        self.assertEqual(phase, "at_photo")
        self.assertIs(holding, False)
        self.assertAlmostEqual(float(fields["place_z"]), 149.5)
        self.assertAlmostEqual(float(fields["retract_z"]), 229.5)


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
