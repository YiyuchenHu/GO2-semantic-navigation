#!/usr/bin/env python3
"""Day 9+ Phase-A smoke — verify frontier_explorer responds to
``/mapping/status`` transitions by emitting DELETEALL on every
marker topic, and that the throttle/lifetime knobs apply.

We deliberately skip /map / /global_costmap and the GetFrontiers
service — those are out of scope. The test focuses on the
mapping_status callback path and the marker-publish guards.

Run with:

    python3 scripts/_smoke_frontier_marker_clear.py
"""
from __future__ import annotations

import os
import sys
import time
from collections import defaultdict
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = os.environ.get("SMOKE_ROS_DOMAIN_ID", "212")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "go2_navigation"))

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from go2_navigation.frontier_explorer_node import (  # noqa: E402
    FrontierExplorerNode,
)


def _count_actions(msg) -> dict:
    """Histogram of Marker.action values inside a Marker / MarkerArray."""
    out: dict = defaultdict(int)
    if isinstance(msg, Marker):
        out[int(msg.action)] += 1
    else:
        for m in msg.markers:
            out[int(m.action)] += 1
    return dict(out)


def _drain_for(executor, sec: float) -> None:
    end = time.time() + sec
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = FrontierExplorerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Subscribers for every marker topic the node owns.
    cells_msgs: list = []
    accepted_msgs: list = []
    rejected_msgs: list = []
    legacy_msgs: list = []
    node.create_subscription(
        Marker, "/frontier/debug/frontier_cells",
        lambda m: cells_msgs.append(m), 10,
    )
    node.create_subscription(
        MarkerArray, "/frontier/debug/accepted_centroids",
        lambda m: accepted_msgs.append(m), 10,
    )
    node.create_subscription(
        MarkerArray, "/frontier/debug/rejected_centroids",
        lambda m: rejected_msgs.append(m), 10,
    )
    node.create_subscription(
        MarkerArray, "/frontier_markers",
        lambda m: legacy_msgs.append(m), 10,
    )

    # Status publisher — match the TRANSIENT_LOCAL QoS the explorer
    # subscribes with so the latched value is delivered immediately.
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
    )
    status_pub = node.create_publisher(
        String, "/mapping/status",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        ),
    )

    # Let discovery + the boot-time legacy DELETEALL run first.
    _drain_for(executor, 1.5)

    # ---- Test 1 — DONE clears every marker topic ---------------------------
    print("Test 1 — mapping_status=DONE clears all marker topics")
    cells_msgs.clear()
    accepted_msgs.clear()
    rejected_msgs.clear()
    legacy_msgs.clear()
    msg = String()
    msg.data = "DONE"
    status_pub.publish(msg)
    _drain_for(executor, 1.5)

    if not cells_msgs:
        print("  [FAIL] no Marker on /frontier/debug/frontier_cells")
        sys.exit(1)
    if not accepted_msgs:
        print("  [FAIL] no MarkerArray on accepted_centroids")
        sys.exit(1)
    if not rejected_msgs:
        print("  [FAIL] no MarkerArray on rejected_centroids")
        sys.exit(1)
    if not legacy_msgs:
        print("  [FAIL] no MarkerArray on /frontier_markers")
        sys.exit(1)
    # Marker.DELETEALL == 3
    if _count_actions(cells_msgs[-1]).get(3, 0) < 1:
        print("  [FAIL] frontier_cells last msg lacks DELETEALL")
        sys.exit(1)
    if _count_actions(accepted_msgs[-1]).get(3, 0) < 1:
        print("  [FAIL] accepted_centroids last msg lacks DELETEALL")
        sys.exit(1)
    if _count_actions(rejected_msgs[-1]).get(3, 0) < 1:
        print("  [FAIL] rejected_centroids last msg lacks DELETEALL")
        sys.exit(1)
    if _count_actions(legacy_msgs[-1]).get(3, 0) < 1:
        print("  [FAIL] legacy /frontier_markers lacks DELETEALL")
        sys.exit(1)
    print("  [OK] all four marker topics received DELETEALL")

    # ---- Test 2 — repeated DONE doesn't re-flood (idempotent guard) -------
    print("Test 2 — repeating DONE is a no-op")
    cells_msgs.clear()
    accepted_msgs.clear()
    rejected_msgs.clear()
    legacy_msgs.clear()
    status_pub.publish(msg)  # same DONE again
    _drain_for(executor, 1.5)
    leak = sum(map(len, [cells_msgs, accepted_msgs, rejected_msgs, legacy_msgs]))
    if leak > 0:
        print(f"  [FAIL] expected no new markers on repeated DONE, got "
              f"{leak} ({len(cells_msgs)},{len(accepted_msgs)},"
              f"{len(rejected_msgs)},{len(legacy_msgs)})")
        sys.exit(1)
    print("  [OK] repeated DONE produced no new marker traffic")

    # ---- Test 3 — transition to NAVIGATING does NOT trigger DELETEALL -----
    print("Test 3 — NAVIGATING is not in clear_marker_states")
    cells_msgs.clear()
    accepted_msgs.clear()
    rejected_msgs.clear()
    legacy_msgs.clear()
    msg2 = String()
    msg2.data = "NAVIGATING"
    status_pub.publish(msg2)
    _drain_for(executor, 1.0)
    leak = sum(map(len, [cells_msgs, accepted_msgs, rejected_msgs, legacy_msgs]))
    if leak > 0:
        print(f"  [FAIL] NAVIGATING transition unexpectedly published "
              f"{leak} marker msgs")
        sys.exit(1)
    print("  [OK] NAVIGATING transition was a no-op for marker topics")

    # ---- Test 4 — FAILED:<reason> still triggers clear via verb match -----
    print("Test 4 — FAILED:<reason> matches the FAILED clear-set")
    cells_msgs.clear()
    accepted_msgs.clear()
    rejected_msgs.clear()
    legacy_msgs.clear()
    msg3 = String()
    msg3.data = "FAILED:no_TF_yet"
    status_pub.publish(msg3)
    _drain_for(executor, 1.5)
    if not cells_msgs or _count_actions(cells_msgs[-1]).get(3, 0) < 1:
        print("  [FAIL] FAILED:<reason> did not clear frontier_cells")
        sys.exit(1)
    print("  [OK] FAILED:<reason> matched FAILED → DELETEALL fired")

    # ---- Test 5 — knobs reflected in instance attributes -------------------
    print("Test 5 — Day 9+ marker hygiene knobs read correctly")
    if not node._publish_debug_markers:
        print("  [FAIL] expected publish_debug_markers default True")
        sys.exit(1)
    if abs(node._debug_publish_period_sec - 1.0) > 1e-6:
        print("  [FAIL] expected debug_publish_period_sec default 1.0")
        sys.exit(1)
    if node._max_debug_cells != 2000:
        print(f"  [FAIL] expected max_debug_cells default 2000, got "
              f"{node._max_debug_cells}")
        sys.exit(1)
    if node._publish_rejected_text:
        print("  [FAIL] expected publish_rejected_centroid_text default False")
        sys.exit(1)
    if abs(node._marker_lifetime_sec - 2.0) > 1e-6:
        print("  [FAIL] expected marker_lifetime_sec default 2.0")
        sys.exit(1)
    if "DONE" not in node._clear_marker_states:
        print("  [FAIL] DONE missing from clear_marker_states")
        sys.exit(1)
    print(f"  [OK] knobs: debug={node._publish_debug_markers}, "
          f"period={node._debug_publish_period_sec}, "
          f"max_cells={node._max_debug_cells}, "
          f"reject_text={node._publish_rejected_text}, "
          f"lifetime={node._marker_lifetime_sec}, "
          f"clear_states={sorted(node._clear_marker_states)}")

    node.destroy_node()
    rclpy.shutdown()
    print("\n[ALL_OK] Day 9+ frontier_marker_clear smoke passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
