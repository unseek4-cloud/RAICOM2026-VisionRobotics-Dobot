# -*- coding: utf-8 -*-
"""用多个已知桌面点验证 EIH 矩阵方向、单位和姿态约定。

CSV 每行：u,v,depth_mm,expected_robot_x_mm,expected_robot_y_mm
至少取工作区左上、右上、左下、右下、中心等分散点。工具只计算，不移动机械臂。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from raicom.config import Settings  # noqa: E402
from raicom.vision.calibration import CalibrationModel  # noqa: E402
from raicom.vision.realsense_camera import RealSenseCamera  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="多点验证 Eye-In-Hand 坐标转换")
    parser.add_argument("points", type=Path, help="验证点 CSV")
    parser.add_argument("--config", default=str(ROOT / "config" / "settings.yaml"))
    parser.add_argument("--max-error-mm", type=float, default=5.0)
    args = parser.parse_args()
    settings = Settings.load(Path(args.config))
    calibration = CalibrationModel.from_settings(settings, simulation=False)

    camera = RealSenseCamera(settings)
    try:
        camera.start()
        intrinsics = camera.get_frame().intrinsics
    finally:
        camera.stop()

    samples = []
    with args.points.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        for row_number, row in enumerate(reader, start=1):
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                u, v, depth, x, y = (float(value) for value in row[:5])
            except (ValueError, IndexError):
                print(f"第 {row_number} 行格式错误：{row}", file=sys.stderr)
                return 2
            samples.append(((u, v), depth, intrinsics, (x, y)))

    try:
        errors = calibration.validate_reference_points(
            samples, max_xy_error_mm=args.max_error_mm, minimum_points=3
        )
    except Exception as exc:
        print(f"验证失败：{exc}", file=sys.stderr)
        return 1
    for index, error in enumerate(errors, start=1):
        print(f"点 {index}: XY误差 {error:.3f} mm")
    print(f"验证通过：最大误差 {max(errors):.3f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

