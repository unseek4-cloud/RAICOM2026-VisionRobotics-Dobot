# -*- coding: utf-8 -*-
"""主流程与中文 PyQt5 面板的离线冒烟测试。"""

from __future__ import annotations

import copy
import logging
import math
import os
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from raicom.config import PLACE_POSE_CLASSES, Settings, SettingsError
from raicom.events import EventBus
from raicom.robot.lua_bridge import LuaBridgeServer
from raicom.runtime import SystemRuntime
from raicom.task.orchestrator import TaskError, TaskOrchestrator
from raicom.types import DirectPlaceTarget, RobotReply


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
            command_id="DIRECT-test",
            status="done",
            raw={
                "phase": required_phase,
                "holding_part": required_holding,
                "at_photo": command == "check_photo",
                "current_pose": "160,-60,430,180,0,0",
            },
        )


class RuntimeFlowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")

    def test_settings_remove_task2_and_list_all_seven_place_poses(self) -> None:
        self.assertNotIn("task2", self.settings.get("tasks", {}))
        self.assertNotIn("task2", self.settings.get("yolo", {}))
        poses = self.settings.get("robot.place_poses_mm_deg")
        self.assertEqual(set(poses), set(PLACE_POSE_CLASSES))
        for class_name in PLACE_POSE_CLASSES:
            with self.subTest(class_name=class_name):
                self.assertEqual(len(poses[class_name]), 6)
                self.assertIsNone(poses[class_name][2])

    def test_full_demo_removes_all_seven_objects(self) -> None:
        """模拟模式必须按任务一→3D识别抓取完成并重新确认桌面为空。"""

        runtime = SystemRuntime(self.settings, real_mode=False)
        robot_statuses: list[dict[str, object]] = []
        runtime.bus.subscribe("robot_status", lambda item: robot_statuses.append(dict(item)))
        try:
            self.assertTrue(runtime.start())
            world = runtime.simulation_world
            self.assertIsNotNone(world)
            self.assertEqual(len(world.objects()), 7)
            self.assertIsNotNone(runtime.orchestrator)
            with patch.object(
                runtime.dvs, "trigger", wraps=runtime.dvs.trigger
            ) as trigger:
                self.assertTrue(runtime.orchestrator.run("all"))
            trigger.assert_called_once_with()
            self.assertEqual(len(world.objects()), 0)
            direct_places = [
                item
                for item in robot_statuses
                if item.get("status") == "done" and "route_key" in item
            ]
            self.assertEqual(len(direct_places), 7)
            heights = {
                "大圆柱": 54.0,
                "正方体": 40.0,
                "梯形": 32.0,
                "长方体": 46.0,
                "圆柱": 36.0,
                "六棱柱": 48.0,
                "平行四边形": 28.0,
            }
            configured = self.settings.get("simulation.place_poses_mm_deg")
            for item in direct_places:
                class_name = str(item["route_key"])
                pose = configured[class_name]
                self.assertAlmostEqual(float(item["place_z"]), float(item["pick_z"]))
                self.assertAlmostEqual(float(item["pick_z"]), 100.0 + heights[class_name] - 1.0)
                self.assertAlmostEqual(float(item["place_rz"]), float(pose[5]))
                self.assertAlmostEqual(
                    world.placement_stack_height_mm(float(pose[0]), float(pose[1])),
                    heights[class_name],
                )
        finally:
            runtime.stop()

    def test_standalone_task1_sends_trigger(self) -> None:
        """单独运行任务一也必须先触发 DVS，再接收任务结果。"""
        runtime = SystemRuntime(self.settings, real_mode=False)
        try:
            self.assertTrue(runtime.start())
            self.assertIsNotNone(runtime.orchestrator)
            with patch.object(
                runtime.dvs, "trigger", wraps=runtime.dvs.trigger
            ) as trigger:
                self.assertTrue(runtime.orchestrator.run("task1"))
            trigger.assert_called_once_with()
        finally:
            runtime.stop()

    def test_task3_respects_configurable_sorting_limit(self) -> None:
        """3D识别抓取达到 settings 上限后应正常结束并保留其余目标。"""

        data = copy.deepcopy(self.settings.as_dict())
        data["tasks"]["task3"]["max_objects"] = 1
        settings = Settings(data, self.settings.config_path)
        runtime = SystemRuntime(settings, real_mode=False)
        try:
            self.assertTrue(runtime.start())
            world = runtime.simulation_world
            self.assertIsNotNone(world)
            self.assertTrue(runtime.orchestrator.run("task3"))
            self.assertEqual(len(world.objects("task3")), 6)
            self.assertAlmostEqual(world.placement_stack_height_mm(175.0, 120.0), 54.0)
        finally:
            runtime.stop()

    def test_task3_refuses_to_start_away_from_photo_and_return_button_recovers(self) -> None:
        runtime = SystemRuntime(self.settings, real_mode=False)
        try:
            self.assertTrue(runtime.start())
            runtime.robot._at_photo = False
            with patch.object(runtime.camera, "flush") as flush:
                self.assertFalse(runtime.orchestrator.run("task3"))
            flush.assert_not_called()
            self.assertEqual(len(runtime.simulation_world.objects("task3")), 7)

            self.assertTrue(runtime.return_to_photo())
            checked = runtime.robot.is_at_photo()
            self.assertEqual(checked.status, "done")
            self.assertIs(checked.raw.get("at_photo"), True)
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
            {"task": "other"},
            {"version": 2},
            {"a": math.nan},
        )
        for item in invalid_items:
            with self.subTest(item=item), self.assertRaises(TaskError):
                TaskOrchestrator._validate_dvs_result(item)

    def test_detection_and_placement_share_one_table_reference(self) -> None:
        self.assertIsNone(self.settings.get("tasks.task3.placement_vision", None))
        self.assertNotIn("workspace_mm", self.settings.get("robot"))
        self.assertEqual(
            self.settings.get("simulation.robot_table_touch_z_mm"),
            self.settings.get("calibration.robot_table_touch_z_mm"),
        )

    def test_lua_bridge_builds_one_direct_same_z_task3_command(self) -> None:
        bridge = _CapturingLuaBridge(self.settings)
        target = DirectPlaceTarget(
            task="task3",
            object_id="T3-1",
            pick_x_mm=200.0,
            pick_y_mm=0.0,
            pick_z_mm=120.0,
            pick_rz_deg=-37.0,
            place_x_mm=250.0,
            place_y_mm=120.0,
            place_rx_deg=180.0,
            place_ry_deg=0.0,
            place_rz_deg=35.0,
            route_key="match",
        )

        reply = bridge.pick_and_place_direct(target)
        self.assertEqual(reply.status, "done")
        command, fields, phase, holding = bridge.calls[-1]
        self.assertEqual(command, "pick_place_direct")
        self.assertEqual(phase, "at_photo")
        self.assertIs(holding, False)
        self.assertAlmostEqual(float(fields["pick_z"]), 120.0)
        self.assertAlmostEqual(float(fields["place_z"]), 120.0)
        self.assertAlmostEqual(float(fields["pick_rz"]), -37.0)
        self.assertAlmostEqual(float(fields["place_rz"]), 35.0)
        self.assertAlmostEqual(float(fields["place_rx"]), 180.0)
        self.assertAlmostEqual(float(fields["place_ry"]), 0.0)

    def test_task3_lua_uses_dobot_movj_directly_from_p1_to_same_z_p2(self) -> None:

        source = (PROJECT_ROOT / "dobotstudio" / "raicom_e6_executor.lua").read_text(
            encoding="utf-8"
        )
        self.assertIn("checked_movj(poses.p1", source)
        self.assertIn("checked_movj(poses.p2", source)
        self.assertIn("phase = \"move_p1_to_p2_same_z\"", source)
        self.assertIn("P1_P2_Z_MISMATCH", source)
        self.assertNotIn("workspace", source.lower())

    def test_photo_commands_carry_pose_tolerance_without_workspace(self) -> None:

        bridge = _CapturingLuaBridge(self.settings)
        reply = bridge.go_photo()

        self.assertEqual(reply.status, "done")
        command, fields, phase, holding = bridge.calls[-1]
        self.assertEqual(command, "go_photo")
        self.assertEqual(phase, "at_photo")
        self.assertIsNone(holding)
        self.assertAlmostEqual(float(fields["photo_x"]), 160.0)
        self.assertAlmostEqual(float(fields["photo_tolerance_mm"]), 2.0)
        self.assertAlmostEqual(float(fields["photo_tolerance_deg"]), 2.0)
        self.assertFalse(any(key.startswith("workspace_") for key in fields))

        reply = bridge.is_at_photo()
        self.assertEqual(reply.status, "done")
        command, fields, phase, holding = bridge.calls[-1]
        self.assertEqual(command, "check_photo")
        self.assertIsNone(phase)
        self.assertIsNone(holding)


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
            self.assertEqual(window.windowTitle(), "3D识别抓取")
            self.assertEqual(window.btn_all.text(), "任务一 + 3D识别抓取")
            self.assertEqual(window.btn_task3.text(), "运行3D识别抓取")
            self.assertEqual(window.btn_go_photo.text(), "自动回到拍照位")
            self.assertTrue(window.btn_go_photo.isHidden())
            self.assertFalse(hasattr(window, "btn_task2"))
            self.assertIn("非物理急停", window.btn_stop.text())
            self.assertEqual(window.region_task.count(), 1)
            self.assertEqual(window.btn_region_full.text(), "恢复全画面")
            self.assertIs(window.video, window.yolo_video)
            self.assertEqual(window.rgb_video.objectName(), "rgbVideo")
            self.assertEqual(window.depth_video.objectName(), "depthVideo")
            self.assertEqual(window.yolo_video.objectName(), "yoloVideo")
            self.assertEqual(window.result_table.columnCount(), 10)
            self.assertEqual(window.result_table.horizontalHeaderItem(6).text(), "高度(mm)")
            window._region_drawn((0.1, 0.2, 0.8, 0.9))
            self.assertEqual(
                window.runtime.recognition_regions.get("task3"),
                (0.1, 0.2, 0.8, 0.9),
            )
            app.processEvents()
        finally:
            window.runtime.stop()
            window.deleteLater()
            app.processEvents()

        real_window = MainWindow(settings, real_mode=True)
        try:
            self.assertFalse(real_window.btn_go_photo.isHidden())
        finally:
            real_window.runtime.stop()
            real_window.deleteLater()
            app.processEvents()


if __name__ == "__main__":
    unittest.main()
