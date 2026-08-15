# -*- coding: utf-8 -*-
"""YOLO 训练入口的 Windows checkpoint 保存保护测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from tools.train_yolo import _build_reliable_detection_trainer, _validate_obb_labels


class _FakeDetectionTrainer:
    def save_model(self) -> bool:
        self.last.write_bytes(b"new-last")
        if self.write_best:
            self.best.write_bytes(b"new-best")
        return True


class ReliableCheckpointTests(unittest.TestCase):
    def test_checkpoint_is_replaced_and_old_best_is_preserved(self) -> None:
        trainer_type = _build_reliable_detection_trainer(_FakeDetectionTrainer)
        with tempfile.TemporaryDirectory() as temporary:
            weights = Path(temporary)
            trainer = trainer_type.__new__(trainer_type)
            trainer.wdir = weights
            trainer.last = weights / "last.pt"
            trainer.best = weights / "best.pt"
            trainer.last.write_bytes(b"old-last")
            trainer.best.write_bytes(b"old-best")

            trainer.write_best = False
            self.assertTrue(trainer.save_model())
            self.assertEqual(trainer.last.read_bytes(), b"new-last")
            self.assertEqual(trainer.best.read_bytes(), b"old-best")

            trainer.write_best = True
            self.assertTrue(trainer.save_model())
            self.assertEqual(trainer.last.read_bytes(), b"new-last")
            self.assertEqual(trainer.best.read_bytes(), b"new-best")
            self.assertEqual(list(weights.glob(".*.tmp")), [])

    def test_obb_label_validation_rejects_legacy_horizontal_box(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "labels" / "train").mkdir(parents=True)
            data_yaml = root / "data.yaml"
            data_yaml.write_text("path: .\ntrain: images/train\n", encoding="utf-8")
            label = root / "labels" / "train" / "sample.txt"
            label.write_text("0 0.5 0.5 0.2 0.2\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "OBB 必须"):
                _validate_obb_labels(data_yaml)

    def test_obb_label_validation_accepts_four_points(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "labels" / "train").mkdir(parents=True)
            data_yaml = root / "data.yaml"
            data_yaml.write_text("path: .\ntrain: images/train\n", encoding="utf-8")
            label = root / "labels" / "train" / "sample.txt"
            label.write_text(
                "0 0.2 0.2 0.8 0.2 0.8 0.7 0.2 0.7\n", encoding="utf-8"
            )

            _validate_obb_labels(data_yaml)


if __name__ == "__main__":
    unittest.main()
