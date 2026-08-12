# -*- coding: utf-8 -*-
"""D435 拍照工具的数据集划分逻辑测试（不连接真实相机）。"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from tools.capture_yolo_dataset import (
    DatasetSplitError,
    _split_counts,
    inspect_photos,
    split_dataset,
)


class YoloDatasetSplitTests(unittest.TestCase):
    def _make_task(self, parent: Path, total: int, *, labels: int | None = None) -> Path:
        task_root = parent / "task2"
        photo_dir = task_root / "photo"
        photo_dir.mkdir(parents=True, exist_ok=True)
        label_total = total if labels is None else labels
        for index in range(total):
            image = photo_dir / f"task2_{index:03d}.jpg"
            image.write_bytes(b"not-a-real-jpg-needed-for-copy-test")
            if index < label_total:
                image.with_suffix(".txt").write_text(
                    "0 0.5 0.5 0.2 0.2\n", encoding="utf-8"
                )
        return task_root

    def test_exact_ten_image_ratio_and_labels(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            task_root = self._make_task(Path(temporary), 10)
            summary = split_dataset(task_root, seed=2026)

            self.assertEqual((summary.train, summary.val, summary.test), (7, 2, 1))
            self.assertEqual(summary.labels, 10)
            self.assertEqual(summary.missing_labels, 0)
            self.assertEqual(len(list((task_root / "photo").glob("*.jpg"))), 10)
            for split, expected in (("train", 7), ("val", 2), ("test", 1)):
                self.assertEqual(len(list((task_root / "images" / split).glob("*.jpg"))), expected)
                self.assertEqual(len(list((task_root / "labels" / split).glob("*.txt"))), expected)

            manifest = json.loads(summary.manifest.read_text(encoding="utf-8"))
            self.assertEqual(manifest["ratios"], {"train": 0.7, "val": 0.2, "test": 0.1})
            self.assertEqual(len(manifest["records"]), 10)

    def test_rebuild_removes_only_previous_generated_split(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            task_root = self._make_task(Path(temporary), 10)
            split_dataset(task_root, seed=2026)
            self._make_task(Path(temporary), 20)

            summary = split_dataset(task_root, seed=2026)

            self.assertEqual((summary.train, summary.val, summary.test), (14, 4, 2))
            split_images = [
                image
                for split in ("train", "val", "test")
                for image in (task_root / "images" / split).glob("*.jpg")
            ]
            self.assertEqual(len(split_images), 20)
            self.assertEqual(len({image.name for image in split_images}), 20)

    def test_missing_labels_are_reported_not_fabricated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            task_root = self._make_task(Path(temporary), 10, labels=8)
            inventory = inspect_photos(task_root)
            summary = split_dataset(task_root, seed=7)

            self.assertEqual(len(inventory.missing_labels), 2)
            self.assertEqual(summary.labels, 8)
            self.assertEqual(summary.missing_labels, 2)
            labels = [
                label
                for split in ("train", "val", "test")
                for label in (task_root / "labels" / split).glob("*.txt")
            ]
            self.assertEqual(len(labels), 8)

    def test_largest_remainder_counts_keep_every_image(self) -> None:
        self.assertEqual(_split_counts(10), {"train": 7, "val": 2, "test": 1})
        self.assertEqual(sum(_split_counts(17).values()), 17)

    def test_manual_split_data_is_never_deleted_or_mixed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            task_root = self._make_task(Path(temporary), 10)
            manual = task_root / "images" / "train" / "manual.jpg"
            manual.parent.mkdir(parents=True, exist_ok=True)
            manual.write_bytes(b"manual-data")

            with self.assertRaises(DatasetSplitError):
                split_dataset(task_root, seed=2026)
            self.assertEqual(manual.read_bytes(), b"manual-data")


if __name__ == "__main__":
    unittest.main()
