# -*- coding: utf-8 -*-
"""YOLO 训练入口的 Windows checkpoint 保存保护测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from tools.train_yolo import _build_reliable_detection_trainer


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


if __name__ == "__main__":
    unittest.main()
