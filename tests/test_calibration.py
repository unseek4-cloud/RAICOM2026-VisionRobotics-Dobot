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
)


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


if __name__ == "__main__":
    unittest.main()
