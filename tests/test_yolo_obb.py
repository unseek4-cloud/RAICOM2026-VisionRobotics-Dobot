# -*- coding: utf-8 -*-
"""YOLO-OBB 旋转框、角度稳定和最短 RZ 的离线测试。"""

from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from raicom.config import Settings
from raicom.task.orchestrator import TaskOrchestrator
from raicom.types import Detection
from raicom.vision.yolo_detector import (
    DetectorError,
    YoloDetector,
    infer_shape,
    normalize_axis_angle_deg,
    oriented_box_axis,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class _Scalar:
    def __init__(self, value: float) -> None:
        self.value = value

    def item(self) -> float:
        return self.value


class _Points:
    def __init__(self, values: list[list[float]]) -> None:
        self.values = values

    def __getitem__(self, _index: int) -> "_Points":
        return self

    def detach(self) -> "_Points":
        return self

    def cpu(self) -> "_Points":
        return self

    def tolist(self) -> list[list[float]]:
        return self.values


class _Obb:
    def __init__(self, points: list[list[float]]) -> None:
        self.cls = [_Scalar(0)]
        self.conf = [_Scalar(0.97)]
        self.xyxyxyxy = _Points(points)


class _Result:
    names = {0: "part"}

    def __init__(self, obb: object) -> None:
        self.obb = obb


class _Model:
    def __init__(self, result: _Result) -> None:
        self.result = result

    def predict(self, **_kwargs: object) -> list[_Result]:
        return [self.result]


def _detector(result: _Result) -> YoloDetector:
    detector = YoloDetector.__new__(YoloDetector)
    detector.settings = Settings.load(PROJECT_ROOT / "config" / "settings.yaml")
    detector.task = "task3"
    detector.logger = None
    detector.confidence = 0.5
    detector.iou = 0.45
    detector.image_size = 640
    detector.max_detections = 20
    detector.use_hsv = False
    detector.font_path = None
    detector.include_keywords = ()
    detector.exclude_keywords = ()
    detector.device = "cpu"
    detector.model = _Model(result)
    detector.names = result.names
    return detector


class YoloObbTests(unittest.TestCase):
    def test_task3_circular_class_is_treated_as_rotation_symmetric(self) -> None:
        self.assertEqual(infer_shape("circula"), "cylinder")

    def test_axis_angle_is_undirected_and_uses_shortest_rotation(self) -> None:
        points = ((40.0, 30.0), (80.0, 50.0), (70.0, 70.0), (30.0, 50.0))
        center, endpoint, angle = oriented_box_axis(points)

        self.assertEqual(center, (55.0, 50.0))
        self.assertGreater(endpoint[0], center[0])
        self.assertAlmostEqual(angle, -26.565051, places=5)
        self.assertAlmostEqual(normalize_axis_angle_deg(100.0), -80.0)
        self.assertAlmostEqual(normalize_axis_angle_deg(-100.0), 80.0)

    def test_detector_reads_four_point_obb_not_horizontal_boxes(self) -> None:
        points = [[40.0, 30.0], [80.0, 50.0], [70.0, 70.0], [30.0, 50.0]]
        detector = _detector(_Result([_Obb(points)]))

        detections = detector.detect(np.zeros((100, 120, 3), dtype=np.uint8))

        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertEqual(detection.bbox, (30, 30, 80, 70))
        self.assertEqual(detection.pixel_center, (55, 50))
        self.assertEqual(len(detection.oriented_bbox or ()), 4)
        self.assertAlmostEqual(float(detection.image_angle_deg), -26.565051, places=5)

    def test_detector_rejects_result_without_obb(self) -> None:
        detector = _detector(_Result(None))
        with self.assertRaisesRegex(DetectorError, "不含 OBB"):
            detector.detect(np.zeros((100, 120, 3), dtype=np.uint8))

    def test_stability_uses_180_degree_wraparound(self) -> None:
        first = Detection("task3", "a", "part", 0.9, (0, 0, 10, 10), image_angle_deg=89.0)
        second = Detection("task3", "b", "part", 0.9, (0, 0, 10, 10), image_angle_deg=-89.0)
        first.pixel_center = second.pixel_center = (5, 5)

        self.assertTrue(TaskOrchestrator._same_target(first, second, 2.0, 3.0))
        self.assertFalse(TaskOrchestrator._same_target(first, second, 2.0, 1.0))


if __name__ == "__main__":
    unittest.main()
