# -*- coding: utf-8 -*-
"""Task2/Task3 可调识别区域的离线测试。"""

from __future__ import annotations

import copy
import unittest
from pathlib import Path

from raicom.config import Settings, SettingsError
from raicom.recognition_region import RecognitionRegionStore
from raicom.runtime import SystemRuntime
from raicom.types import Detection


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class RecognitionRegionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")

    def test_regions_are_independent_and_filter_by_detection_center(self) -> None:
        store = RecognitionRegionStore(self.settings)
        store.set("task2", (0.0, 0.0, 0.5, 1.0))
        detections = [
            Detection("task2", "inside", "part", 0.9, (40, 40, 60, 60), pixel_center=(49, 50)),
            Detection("task2", "outside", "part", 0.9, (40, 40, 60, 60), pixel_center=(50, 50)),
        ]

        filtered = store.filter("task2", detections, 100, 100)

        self.assertEqual([item.object_id for item in filtered], ["inside"])
        self.assertEqual(store.get("task3"), (0.0, 0.0, 1.0, 1.0))

    def test_invalid_configured_region_is_rejected(self) -> None:
        for invalid in (
            [0.5, 0.0, 0.5, 1.0],
            [-0.1, 0.0, 1.0, 1.0],
            [0.0, 0.0, 1.1, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, True, 1.0],
        ):
            with self.subTest(invalid=invalid):
                data = copy.deepcopy(self.settings.as_dict())
                data["tasks"]["task2"]["recognition_region"] = invalid
                settings = Settings(data, self.settings.config_path)
                with self.assertRaisesRegex(SettingsError, "recognition_region"):
                    settings.task_recognition_region("task2")

    def test_demo_task_only_picks_objects_inside_live_region(self) -> None:
        runtime = SystemRuntime(self.settings, real_mode=False)
        try:
            self.assertTrue(runtime.start())
            runtime.recognition_regions.set("task2", (0.0, 0.0, 0.3, 1.0))

            self.assertTrue(runtime.orchestrator.run("task2"))

            remaining = runtime.simulation_world.objects("task2")
            self.assertEqual([item.object_id for item in remaining], ["T2-BLUE-CYLINDER"])
        finally:
            runtime.stop()


if __name__ == "__main__":
    unittest.main()
