"""Smoke: day8_two_phase.launch.py DeclareLaunchArgument min_detection_confidence default."""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
LAUNCH = (
    REPO
    / "src"
    / "go2_bringup_sim"
    / "launch"
    / "day8_two_phase.launch.py"
).read_text(encoding="utf-8")


def main() -> int:
    m = re.search(
        r'min_detection_confidence"\s*,\s*default_value="(0\.\d+)"',
        LAUNCH,
    )
    if not m:
        print("could not find min_detection_confidence default", file=sys.stderr)
        return 1
    val = float(m.group(1))
    if val not in (0.45, 0.40):
        print(f"unexpected min_detection_confidence default: {val}", file=sys.stderr)
        return 1
    print(f"[PASS] min_detection_confidence default is {val}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
