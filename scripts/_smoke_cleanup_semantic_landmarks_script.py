"""Smoke: cleanup_semantic_landmarks.sh exists, bash-syntax OK, expected strings."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "cleanup_semantic_landmarks.sh"


def main() -> int:
    if not SCRIPT.is_file():
        print(f"missing {SCRIPT}", file=sys.stderr)
        return 1
    text = SCRIPT.read_text(encoding="utf-8")
    for needle in (
        "keep_best_class person",
        "keep_best_class table",
        "clear_candidates",
        'timeout 15s ros2 topic pub --once',
        "/semantic_map/control",
    ):
        if needle not in text:
            print(f"missing substring {needle!r}", file=sys.stderr)
            return 1
    r = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    if r.returncode != 0:
        print(r.stderr or r.stdout, file=sys.stderr)
        return r.returncode
    print("[PASS] cleanup_semantic_landmarks.sh structure + bash -n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
