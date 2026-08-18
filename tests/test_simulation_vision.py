# -*- coding: utf-8 -*-
"""七类工件同时出现的模拟相机和模拟检测器测试。"""

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
from raicom.recognition_region import RecognitionRegionStore  # noqa: E402
from raicom.vision.calibration import CalibrationModel  # noqa: E402
from raicom.vision.live_display import (  # noqa: E402
    build_live_vision_frame,
    colorize_depth_mm,
)
from raicom.vision.realsense_camera import RealSenseCamera  # noqa: E402
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
        self.task3 = MockDetector(self.settings, "task3", self.world)
        self.calibration = CalibrationModel.from_settings(
            self.settings, simulation=True
        )
        self.camera.start()

    def tearDown(self) -> None:
        self.camera.stop()

    def test_seven_objects_are_visible_with_consistent_depth(self) -> None:
        bundle = self.camera.get_frame()
        detections = self.task3.detect(bundle.color_bgr)

        self.assertEqual(len(detections), 7)
        self.assertEqual(
            {item.class_name for item in detections},
            {
                "大圆柱",
                "正方体",
                "梯形",
                "长方体",
                "圆柱",
                "六棱柱",
                "平行四边形",
            },
        )
        self.assertEqual(bundle.color_bgr.shape, (480, 640, 3))
        self.assertEqual(bundle.depth_mm.shape, (480, 640))

        for detection in detections:
            self.assertIsNotNone(detection.oriented_bbox)
            self.assertIsNotNone(detection.image_angle_deg)
            measured = self.camera.measure_depth_mm(bundle, detection.bbox)
            expected_height = float(detection.extra["height_mm"])
            self.assertAlmostEqual(
                measured,
                self.world.table_depth_mm - expected_height,
                places=6,
            )
            if detection.shape != "cylinder":
                from raicom.vision.yolo_detector import oriented_box_axis

                axis_center, axis_endpoint, _ = oriented_box_axis(
                    detection.oriented_bbox
                )
                rz = self.calibration.image_axis_to_robot_rz_deg(
                    axis_center, axis_endpoint, measured, bundle.intrinsics
                )
                expected = next(
                    item.angle_deg
                    for item in self.world.objects()
                    if item.object_id == detection.object_id
                )
                expected = (expected + 90.0) % 180.0 - 90.0
                self.assertAlmostEqual(rz, expected, places=6)
            camera_xyz, robot_xyz = self.calibration.locate(
                detection.pixel_center, measured, bundle.intrinsics
            )
            self.assertAlmostEqual(camera_xyz[2], measured, places=6)
            self.assertAlmostEqual(
                robot_xyz[2],
                self.calibration.robot_table_touch_z_mm
                + expected_height
                - self.calibration.press_down_mm,
                places=6,
            )

        annotated = self.task3.annotate(bundle.color_bgr, detections)
        self.assertEqual(annotated.shape, bundle.color_bgr.shape)
        self.assertFalse(np.array_equal(annotated, bundle.color_bgr))

    def test_remove_updates_color_depth_and_detection_together(self) -> None:
        before = self.camera.get_frame()
        first = next(
            item
            for item in self.task3.detect(before.color_bgr)
            if item.object_id == "T3-LARGE-CYLINDER"
        )
        self.assertTrue(self.world.remove(first.object_id))
        self.assertFalse(self.world.remove(first.object_id), "重复移除必须返回 False")

        after = self.camera.get_frame()
        remaining = self.task3.detect(after.color_bgr)
        self.assertEqual(len(remaining), 6)
        self.assertNotIn(first.object_id, {item.object_id for item in remaining})

        # 原工件中心恢复为空台面深度。
        center_depth = self.camera.measure_depth_mm(after, first.bbox)
        self.assertAlmostEqual(center_depth, self.world.table_depth_mm, places=6)

    def test_live_frame_contains_three_synchronized_images_and_heights(self) -> None:
        bundle = self.camera.get_frame()
        live = build_live_vision_frame(
            bundle,
            task="task3",
            detector=self.task3,
            camera=self.camera,
            calibration=self.calibration,
            recognition_regions=RecognitionRegionStore(self.settings),
            depth_min_mm=300.0,
            depth_max_mm=1500.0,
        )

        self.assertEqual(live.color_bgr.shape, (480, 640, 3))
        self.assertEqual(live.depth_bgr.shape, (480, 640, 3))
        self.assertEqual(live.yolo_bgr.shape, (480, 640, 3))
        self.assertEqual(len(live.detections), 7)
        for detection in live.detections:
            self.assertIsNotNone(detection.depth_mm)
            expected = self.world.table_depth_mm - float(detection.depth_mm)
            self.assertAlmostEqual(
                float(detection.extra["object_height_mm"]),
                expected,
                places=6,
            )

    def test_depth_colormap_marks_invalid_pixels_black(self) -> None:
        depth = np.asarray([[0.0, 300.0], [900.0, np.nan]], dtype=np.float32)
        colored = colorize_depth_mm(
            depth,
            depth_min_mm=300.0,
            depth_max_mm=1500.0,
        )
        self.assertEqual(colored.shape, (2, 2, 3))
        np.testing.assert_array_equal(colored[0, 0], (0, 0, 0))
        np.testing.assert_array_equal(colored[1, 1], (0, 0, 0))
        self.assertTrue(np.any(colored[0, 1] != 0))

    def test_real_camera_uses_independent_max_rgb_and_depth_profiles(self) -> None:
        camera = RealSenseCamera(self.settings)
        self.assertEqual((camera.color_width, camera.color_height, camera.color_fps), (1920, 1080, 30))
        self.assertEqual((camera.depth_width, camera.depth_height, camera.depth_fps), (1280, 720, 30))


if __name__ == "__main__":
    unittest.main()
