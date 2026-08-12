# -*- coding: utf-8 -*-
"""示例 EIH 标定矩阵和抓取 Z 公式测试。"""

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from raicom.config import Settings  # noqa: E402
from raicom.types import CameraIntrinsics  # noqa: E402
from raicom.vision.calibration import (  # noqa: E402
    CalibrationError,
    CalibrationModel,
    _pose_matrix,
)
from raicom.vision.realsense_camera import FrameBundle  # noqa: E402


def _real_calibration_settings(*, press_down_mm: float = 2.0) -> Settings:
    base = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
    data = copy.deepcopy(base.as_dict())
    data["calibration"].update(
        {
            "eih_yaml": "config/calibration/CaliMatrixData.example.yaml",
            "table_depth_mm": 700.0,
            "robot_table_touch_z_mm": 100.0,
            "press_down_mm": press_down_mm,
            "matrix_translation_unit": "m",
            "invert_cam_to_tip": False,
        }
    )
    data["robot"]["photo_pose_mm_deg"] = [160.0, -95.0, 400.0, 180.0, 0.0, 0.0]
    data["robot"]["workspace_mm"] = {
        "x": [-1000.0, 1000.0],
        "y": [-1000.0, 1000.0],
        "z": [-1000.0, 1000.0],
    }
    return Settings(data, PROJECT_ROOT / "config" / "settings.yaml")


class CalibrationModelTests(unittest.TestCase):
    def setUp(self) -> None:
        self.intrinsics = CameraIntrinsics(
            width=640,
            height=480,
            fx=610.4421094830959,
            fy=611.8248983649639,
            ppx=314.2365610998556,
            ppy=235.3488553673523,
        )

    def test_example_eih_matrix_is_loaded_in_millimetres(self) -> None:
        model = CalibrationModel.from_settings(_real_calibration_settings())
        self.assertIsNotNone(model.camera_to_tip_mm)
        assert model.camera_to_tip_mm is not None
        self.assertEqual(model.camera_to_tip_mm.shape, (4, 4))
        np.testing.assert_allclose(
            model.camera_to_tip_mm[:3, 3],
            [-26.73430450436789, -123.03086474782367, 32.631065673169965],
            rtol=0.0,
            atol=1e-9,
        )
        rotation = model.camera_to_tip_mm[:3, :3]
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=9)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)

    def test_locate_uses_strict_table_depth_z_formula(self) -> None:
        model = CalibrationModel.from_settings(
            _real_calibration_settings(press_down_mm=2.0)
        )
        pixel = (self.intrinsics.ppx, self.intrinsics.ppy)
        camera_xyz, robot_xyz = model.locate(pixel, 650.0, self.intrinsics)

        # 相机主点反投影后 X/Y 应接近 0，Z 就是输入深度。
        np.testing.assert_allclose(camera_xyz, (0.0, 0.0, 650.0), atol=1e-9)
        # Z = 台面接触机器人Z + (空台深度 - 工件深度) - 下压量
        #   = 100 + (700 - 650) - 2 = 148 mm。
        self.assertAlmostEqual(robot_xyz[2], 148.0, places=9)

    def test_invalid_height_is_rejected_instead_of_guessed(self) -> None:
        model = CalibrationModel.from_settings(_real_calibration_settings())
        with self.assertRaises(CalibrationError):
            model.locate((320, 240), 705.0, self.intrinsics)

    def test_dynamic_place_surface_uses_current_tip_pose_not_pick_table(self) -> None:
        model = CalibrationModel.from_settings(_real_calibration_settings())
        inspection_pose = model.placement_inspection_pose(
            (250.0, 120.0), 430.0, (180.0, 0.0, 0.0)
        )
        assert model.camera_to_tip_mm is not None
        base_from_camera = (
            _pose_matrix(inspection_pose, model.pose_rotation_order)
            @ model.camera_to_tip_mm
        )
        rotation = base_from_camera[:3, :3]
        translation = base_from_camera[:3, 3]

        rows, cols = np.indices((self.intrinsics.height, self.intrinsics.width))
        rays = np.stack(
            (
                (cols - self.intrinsics.ppx) / self.intrinsics.fx,
                (rows - self.intrinsics.ppy) / self.intrinsics.fy,
                np.ones_like(rows, dtype=np.float64),
            ),
            axis=-1,
        )
        target_surface_z = 80.0  # 与抓取台面的 100 mm 故意不同。
        denominator = rays @ rotation[2, :]
        depth = (target_surface_z - translation[2]) / denominator
        bundle = FrameBundle(
            color_bgr=np.zeros((480, 640, 3), dtype=np.uint8),
            depth_mm=depth.astype(np.float32),
            intrinsics=self.intrinsics,
        )

        surface, measured_depth, valid_points = model.locate_surface_at_robot_xy(
            bundle,
            (250.0, 120.0),
            inspection_pose,
            depth_min_mm=50.0,
            depth_max_mm=1500.0,
            radius_mm=8.0,
            min_points=20,
        )
        self.assertAlmostEqual(surface[0], 250.0, delta=0.5)
        self.assertAlmostEqual(surface[1], 120.0, delta=0.5)
        self.assertAlmostEqual(surface[2], target_surface_z, places=4)
        self.assertGreater(measured_depth, 0.0)
        self.assertGreaterEqual(valid_points, 20)

    def test_configured_task3_inspection_pose_is_inside_real_workspace(self) -> None:
        settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
        model = CalibrationModel.from_settings(settings)
        place = settings.get("robot.place_points.task3.default")
        orientation = tuple(settings.get("robot.motion.orientation_mm_deg"))
        pose = model.placement_inspection_pose(
            (float(place["x_mm"]), float(place["y_mm"])),
            float(settings.get("robot.motion.place_inspection_z_mm")),
            orientation,
        )
        model.validate_workspace(pose[:3])

    def test_real_run_rejects_photo_pose_outside_workspace(self) -> None:
        base = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
        data = copy.deepcopy(base.as_dict())
        data["robot"]["photo_pose_mm_deg"][0] = 154.0
        settings = Settings(data, PROJECT_ROOT / "config" / "settings.yaml")

        issues = settings.validate_real_run()

        self.assertTrue(
            any("robot.photo_pose_mm_deg.x=154" in issue for issue in issues),
            issues,
        )


if __name__ == "__main__":
    unittest.main()
