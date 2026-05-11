"""Smoke tests for NL command chain helpers (no running ROS graph)."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "go2_nl_parser"))
sys.path.insert(0, str(REPO / "src" / "go2_task_coordinator"))

from go2_nl_parser.nl_parser_node import NlParserNode  # noqa: E402
from go2_task_coordinator.task_coordinator_node import (  # noqa: E402
    coordinator_fallback_target_class,
)


def _test_coordinator_fallback() -> None:
    assert coordinator_fallback_target_class("go to person") == "person"
    assert coordinator_fallback_target_class("navigate to table") == "table"
    assert coordinator_fallback_target_class("go to desk") == "table"
    assert coordinator_fallback_target_class("find worker") == "person"
    assert coordinator_fallback_target_class("open the pod bay door") is None


def _test_nl_canonical() -> None:
    assert NlParserNode._canonical_navigation_class("desk") == "table"
    assert NlParserNode._canonical_navigation_class("table") == "table"
    assert NlParserNode._canonical_navigation_class("person") == "person"


def _test_test_script_pub_line() -> None:
    txt = (REPO / "scripts" / "test_day8_nl_to_goal.sh").read_text(
        encoding="utf-8"
    )
    assert "ros2 topic pub --once /user_command std_msgs/msg/String" in txt
    assert "{data: '${USER_CMD}'}" in txt


def main() -> int:
    _test_coordinator_fallback()
    _test_nl_canonical()
    _test_test_script_pub_line()
    r = subprocess.run(
        ["bash", "-n", str(REPO / "scripts" / "test_day8_nl_to_goal.sh")],
        capture_output=True,
        text=True,
        check=False,
    )
    if r.returncode != 0:
        print(r.stderr or r.stdout, file=sys.stderr)
        return r.returncode
    print("[PASS] nl command chain smoke (fallback + canonical + bash -n)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
