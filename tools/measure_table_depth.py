# -*- coding: utf-8 -*-
"""在固定拍照位测量空台面 D435 深度 z-table（mm）。

运行前必须把机械臂示教到最终拍照位，清空桌面目标区域并保持相机不动。
本工具只读取相机，不连接或移动机械臂。
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from raicom.config import Settings  # noqa: E402
from raicom.vision.realsense_camera import RealSenseCamera  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="测量固定拍照位的空台面深度")
    parser.add_argument("--config", default=str(ROOT / "config" / "settings.yaml"))
    parser.add_argument("--frames", type=int, default=60, help="采样帧数")
    parser.add_argument("--roi", nargs=4, type=int, metavar=("X", "Y", "W", "H"),
                        help="台面空白ROI；默认取图像中央 80×80")
    args = parser.parse_args()
    settings = Settings.load(Path(args.config))
    camera = RealSenseCamera(settings)
    values: list[float] = []
    try:
        camera.start()
        camera.flush()
        for _ in range(max(5, args.frames)):
            bundle = camera.get_frame()
            height, width = bundle.depth_mm.shape
            if args.roi:
                x, y, w, h = args.roi
            else:
                w = h = 80
                x, y = width // 2 - w // 2, height // 2 - h // 2
            x1, y1 = max(0, x), max(0, y)
            x2, y2 = min(width, x + w), min(height, y + h)
            patch = bundle.depth_mm[y1:y2, x1:x2].astype(np.float64)
            valid = patch[
                np.isfinite(patch)
                & (patch >= float(settings.get("camera.depth_min_mm")))
                & (patch <= float(settings.get("camera.depth_max_mm")))
                & (patch > 0)
            ]
            if valid.size >= max(20, patch.size // 3):
                values.append(float(np.median(valid)))
    finally:
        camera.stop()

    if len(values) < 5:
        print("有效深度帧不足，请检查 ROI、反光、USB3 连接和相机量程。", file=sys.stderr)
        return 1
    median = statistics.median(values)
    spread = max(values) - min(values)
    print(f"有效帧：{len(values)}")
    print(f"z-table 中值：{median:.3f} mm")
    print(f"帧间极差：{spread:.3f} mm")
    print("请填写到 config/settings.yaml：")
    print(f"  calibration.table_depth_mm: {median:.3f}")
    if spread > float(settings.get("camera.max_temporal_depth_spread_mm", 8.0)):
        print("警告：深度波动偏大，不建议用于抓取；请改善曝光、角度或表面纹理。")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

