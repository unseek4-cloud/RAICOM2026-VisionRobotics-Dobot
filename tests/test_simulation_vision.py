# -*- coding: utf-8 -*-
"""四工件同时出现的模拟相机和模拟检测器测试。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from raicom.config import Settings  # noqa: E402
from raicom.vision.calibration import CalibrationModel  # noqa: E402
from raicom.vision.simulation import (  # noqa: E402
    MockCamera,
    MockDetector,
    SimulationWorld,
)


class SimulationVisionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
        self.world = SimulationWorld(self.settings)
        self.camera = MockCamera(self.settings, self.world)
        self.task2 = MockDetector(self.settings, "task2", self.world)
        self.task3 = MockDetector(self.settings, "task3", self.world)
        self.calibration = CalibrationModel.from_settings(
            self.settings, simulation=True
        )
        self.camera.start()

    def tearDown(self) -> None:
        self.camera.stop()

    def test_four_objects_are_visible_with_consistent_depth(self) -> None:
        bundle = self.camera.get_frame()
        task2_detections = self.task2.detect(bundle.color_bgr)
        task3_detections = self.task3.detect(bundle.color_bgr)
        detections = task2_detections + task3_detections

        self.assertEqual(len(task2_detections), 2)
        self.assertEqual(len(task3_detections), 2)
        self.assertEqual(
            {item.object_id for item in detections},
            {
                "T2-RED-CUBE",
                "T2-BLUE-CYLINDER",
                "T3-MATCH",
                "T3-NOT-MATCH",
            },
        )
        self.assertEqual(bundle.color_bgr.shape, (480, 640, 3))
        self.assertEqual(bundle.depth_mm.shape, (480, 640))

        for detection in detections:
            measured = self.camera.measure_depth_mm(bundle, detection.bbox)
            expected_height = float(detection.extra["height_mm"])
            self.assertAlmostEqual(
                measured,
                self.world.table_depth_mm - expected_height,
                places=6,
            )
            camera_xyz, robot_xyz = self.calibration.locate(
                detection.pixel_center, measured, bundle.intrinsics
            )
            self.assertAlmostEqual(camera_xyz[2], measured, places=6)
            self.assertAlmostEqual(
                robot_xyz[2],
                self.calibration.robot_table_touch_z_mm + expected_height,
                places=6,
            )

        annotated = self.task2.annotate(bundle.color_bgr, task2_detections)
        self.assertEqual(annotated.shape, bundle.color_bgr.shape)
        self.assertFalse(np.array_equal(annotated, bundle.color_bgr))

    def test_remove_updates_color_depth_and_detection_together(self) -> None:
        before = self.camera.get_frame()
        red = next(
            item
            for item in self.task2.detect(before.color_bgr)
            if item.object_id == "T2-RED-CUBE"
        )
        self.assertTrue(self.world.remove(red.object_id))
        self.assertFalse(self.world.remove(red.object_id), "重复移除必须返回 False")

        after = self.camera.get_frame()
        remaining_task2 = self.task2.detect(after.color_bgr)
        remaining_task3 = self.task3.detect(after.color_bgr)
        self.assertEqual([item.object_id for item in remaining_task2], ["T2-BLUE-CYLINDER"])
        self.assertEqual(len(remaining_task3), 2)

        # 原红色工件中心恢复为空台面深度。
        center_depth = self.camera.measure_depth_mm(after, red.bbox)
        self.assertAlmostEqual(center_depth, self.world.table_depth_mm, places=6)


if __name__ == "__main__":
    unittest.main()
