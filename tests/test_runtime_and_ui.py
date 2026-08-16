# -*- coding: utf-8 -*-
"""主流程与中文 PyQt5 面板的离线冒烟测试。"""

from __future__ import annotations

import copy
import logging
import math
import os
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from raicom.config import Settings, SettingsError
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

    def test_task3_respects_configurable_sorting_limit(self) -> None:
        """任务三达到 settings 上限后应正常结束并保留其余同任务目标。"""

        data = copy.deepcopy(self.settings.as_dict())
        data["tasks"]["task3"]["max_objects"] = 1
        settings = Settings(data, self.settings.config_path)
        runtime = SystemRuntime(settings, real_mode=False)
        try:
            self.assertTrue(runtime.start())
            world = runtime.simulation_world
            self.assertIsNotNone(world)
            self.assertTrue(runtime.orchestrator.run("task3"))
            self.assertEqual(len(world.objects("task2")), 2)
            self.assertEqual(len(world.objects("task3")), 1)
            self.assertAlmostEqual(world.placement_stack_height_mm(250.0, 120.0), 30.0)
        finally:
            runtime.stop()

    def test_task2_can_finish_below_configured_limit(self) -> None:
        """目标少于上限时，连续空帧确认后按实际完成数结束。"""

        data = copy.deepcopy(self.settings.as_dict())
        data["tasks"]["task2"]["max_objects"] = 3
        settings = Settings(data, self.settings.config_path)
        runtime = SystemRuntime(settings, real_mode=False)
        try:
            self.assertTrue(runtime.start())
            world = runtime.simulation_world
            self.assertIsNotNone(world)
            self.assertTrue(runtime.orchestrator.run("task2"))
            self.assertEqual(len(world.objects("task2")), 0)
            self.assertEqual(len(world.objects("task3")), 2)
        finally:
            runtime.stop()

    def test_sorting_limit_must_be_positive_integer(self) -> None:
        for invalid in (0, -1, 1.5, True, "2"):
            with self.subTest(invalid=invalid):
                data = copy.deepcopy(self.settings.as_dict())
                data["tasks"]["task3"]["max_objects"] = invalid
                settings = Settings(data, self.settings.config_path)
                with self.assertRaisesRegex(SettingsError, "max_objects"):
                    settings.task_max_objects("task3")

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

    def test_task3_uses_independent_place_table_reference(self) -> None:
        """空放置台面和后续堆顶必须使用任务三自己的深度/TCP Z 参考。"""

        convert = TaskOrchestrator._place_surface_z_from_reference
        self.assertAlmostEqual(convert(388.0, 90.0, 388.0, 1), 90.0)
        self.assertAlmostEqual(convert(388.0, 90.0, 363.0, 1), 115.0)

    def test_lua_bridge_builds_two_phase_task3_commands(self) -> None:
        bridge = _CapturingLuaBridge(self.settings)
        target = StackPlaceTarget(
            task="task3",
            object_id="T3-1",
            pick_x_mm=200.0,
            pick_y_mm=0.0,
            pick_z_mm=120.0,
            pick_rz_deg=-37.0,
            object_height_mm=30.0,
            place_x_mm=250.0,
            place_y_mm=120.0,
            inspection_x_mm=276.0,
            inspection_y_mm=-3.0,
            inspection_z_mm=410.0,
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
        self.assertAlmostEqual(float(fields["inspection_z"]), 410.0)
        self.assertAlmostEqual(float(fields["pick_rz"]), -37.0)
        self.assertAlmostEqual(float(fields["inspection_rz"]), 0.0)

        second = bridge.place_from_inspection(target, first.command_id, 149.5)
        self.assertEqual(second.status, "done")
        command, fields, phase, holding = bridge.calls[-1]
        self.assertEqual(command, "place_from_inspection")
        self.assertEqual(fields["hold_id"], first.command_id)
        self.assertEqual(phase, "at_photo")
        self.assertIs(holding, False)
        self.assertAlmostEqual(float(fields["place_z"]), 149.5)
        self.assertAlmostEqual(float(fields["retract_z"]), 229.5)
        self.assertAlmostEqual(float(fields["place_rz"]), 0.0)
        # Lua 同时把 retract_z 用作低位水平转运高度，避免前往无逆解的
        # (place_x, place_y, inspection_z) 高位终点。
        self.assertLess(float(fields["retract_z"]), float(fields["inspection_z"]))

    def test_task3_lua_avoids_high_z_at_arbitrary_pick_xy(self) -> None:
        """第一阶段必须先低位到观察 XY，不能在任意抓取 XY 直接升到 410。"""

        source = (PROJECT_ROOT / "dobotstudio" / "raicom_e6_executor.lua").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "x = job.inspection_x, y = job.inspection_y, z = job.lift_z",
            source,
        )
        self.assertNotIn(
            "x = job.pick_x, y = job.pick_y, z = job.inspection_z",
            source,
        )
        self.assertIn("phase = \"straighten_rz\"", source)
        self.assertIn("rz = job.inspection_rz", source)

    def test_go_photo_carries_workspace_contract(self) -> None:
        """拍照命令也必须让 Lua 核对 Python 当前使用的工作空间。"""

        bridge = _CapturingLuaBridge(self.settings)
        reply = bridge.go_photo()

        self.assertEqual(reply.status, "done")
        command, fields, phase, holding = bridge.calls[-1]
        self.assertEqual(command, "go_photo")
        self.assertEqual(phase, "at_photo")
        self.assertIsNone(holding)
        self.assertAlmostEqual(float(fields["photo_x"]), 160.0)
        for axis in ("x", "y", "z"):
            bounds = self.settings.get(f"robot.workspace_mm.{axis}")
            self.assertAlmostEqual(float(fields[f"workspace_{axis}_min"]), float(bounds[0]))
            self.assertAlmostEqual(float(fields[f"workspace_{axis}_max"]), float(bounds[1]))


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
            self.assertEqual(window.region_task.count(), 2)
            self.assertEqual(window.btn_region_full.text(), "恢复全画面")
            window._region_drawn((0.1, 0.2, 0.8, 0.9))
            self.assertEqual(
                window.runtime.recognition_regions.get("task2"),
                (0.1, 0.2, 0.8, 0.9),
            )
            app.processEvents()
        finally:
            window.runtime.stop()
            window.deleteLater()
            app.processEvents()


if __name__ == "__main__":
    unittest.main()
