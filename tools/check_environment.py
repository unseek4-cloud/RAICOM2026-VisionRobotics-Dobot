# -*- coding: utf-8 -*-
"""独立环境自检入口，可在 HKtest 环境中直接运行。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from raicom.config import Settings  # noqa: E402
from raicom.environment import print_environment_report  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="检查比赛电脑依赖、端口和配置")
    parser.add_argument("--real", action="store_true", help="同时检查真机必填参数")
    parser.add_argument(
        "--config", default=str(ROOT / "config" / "settings.yaml"), help="配置文件"
    )
    args = parser.parse_args()
    settings = Settings.load(Path(args.config))
    return 0 if print_environment_report(settings, real_mode=args.real) else 1


if __name__ == "__main__":
    raise SystemExit(main())

