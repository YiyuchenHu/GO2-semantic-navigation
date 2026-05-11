#!/usr/bin/env python3
"""Day 9+ Phase-A smoke — verify /mapping/debug/status emits the rich
operator-friendly status line, and that the visited-frontier blacklist
reject path tags the selection_reason correctly.

Boots ``mapping_explorer_node`` in isolation under a private
ROS_DOMAIN_ID so a live day8_two_phase stack on the same machine
can't interfere. We don't actually fire NavigateToPose / GetFrontiers
service calls — instead we manipulate the node's internal state
directly (``_visited_frontiers``, ``_inflight_goal_xy``, etc.) and
trigger the publisher manually, then assert the published string
contains the expected ``key=value`` tokens.

Run with:

    python3 scripts/_smoke_mapping_debug_status.py

Returns exit code 0 on success, 1 on assertion failure.
"""
from __future__ import annotations

import os
import sys
import time
from collections import deque
from pathlib import Path

# Hermetic isolation — pick a domain ID outside the day8_two_phase
# default range. Override via env if it conflicts with another smoke.
os.environ["ROS_DOMAIN_ID"] = os.environ.get("SMOKE_ROS_DOMAIN_ID", "211")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "go2_navigation"))

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from go2_navigation.mapping_explorer_node import (  # noqa: E402
    MappingExplorerNode,
    _State,
)


def _assert_in(needle: str, haystack: str, label: str) -> None:
    if needle in haystack:
        print(f"  [OK] {label}: found {needle!r}")
    else:
        print(f"  [FAIL] {label}: missing {needle!r} in:\n    {haystack}")
        sys.exit(1)


def main() -> int:
    rclpy.init()
    node = MappingExplorerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Listener to capture debug status messages.
    inbox: deque = deque(maxlen=20)

    def _on_dbg(msg: String) -> None:
        inbox.append(msg.data)

    node.create_subscription(
        String, "/mapping/debug/status", _on_dbg, 10
    )

    # Spin a bit so the publisher and our listener finalise discovery.
    end = time.time() + 1.5
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)

    # Trigger an explicit publish and capture.
    node._publish_debug_status()
    end = time.time() + 1.0
    while time.time() < end and not inbox:
        executor.spin_once(timeout_sec=0.05)

    if not inbox:
        print("[FAIL] no /mapping/debug/status message received")
        sys.exit(1)
    msg0 = inbox[-1]
    print(f"\nTest 1 — boot publish on /mapping/debug/status\n  raw: {msg0}")
    _assert_in("state=", msg0, "state field")
    _assert_in("goal=none", msg0, "no in-flight goal at boot")
    _assert_in("dist=nan", msg0, "no distance at boot")
    _assert_in("nav2=-", msg0, "no nav2 status yet")
    _assert_in("visited=0", msg0, "visited counter starts at 0")
    _assert_in("get_frontiers=", msg0, "get_frontiers counter present")
    _assert_in("selected=-", msg0, "no selection yet")

    # ---- Test 2 — visited-frontier marking ---------------------------------
    print("\nTest 2 — visited frontier blacklist")
    node._mark_visited_frontier(3.5, -1.2)
    if node._visited_frontier_count != 1:
        print(f"  [FAIL] visited_frontier_count expected 1, got "
              f"{node._visited_frontier_count}")
        sys.exit(1)
    print("  [OK] first mark increments counter")
    # Re-mark within the same reject radius — should aggregate, not append.
    node._mark_visited_frontier(3.6, -1.1)
    if len(node._visited_frontiers) != 1:
        print(f"  [FAIL] expected aggregation into 1 entry, got "
              f"{len(node._visited_frontiers)}")
        sys.exit(1)
    print("  [OK] nearby second mark aggregates into the same entry")
    # Far away — should be a separate entry.
    node._mark_visited_frontier(6.0, 4.0)
    if len(node._visited_frontiers) != 2:
        print(f"  [FAIL] expected 2 entries after far-away mark, got "
              f"{len(node._visited_frontiers)}")
        sys.exit(1)
    print("  [OK] far-away mark created second entry")

    # ---- Test 3 — recent-visit rejection -----------------------------------
    print("\nTest 3 — _is_recently_visited rejects close points")
    if not node._is_recently_visited(3.55, -1.15):
        print("  [FAIL] expected (3.55,-1.15) to be marked recently visited")
        sys.exit(1)
    print("  [OK] close point matches visited entry")
    if node._is_recently_visited(0.0, 0.0):
        print("  [FAIL] expected (0,0) not visited")
        sys.exit(1)
    print("  [OK] far point not flagged")

    # ---- Test 4 — memory aging ---------------------------------------------
    # Force the entry to look ancient by rewriting ts_ns. 200 sec >
    # default visited_frontier_memory_sec=120.
    print("\nTest 4 — memory aging prunes ancient entries")
    now_ns = node.get_clock().now().nanoseconds
    very_old_ns = now_ns - int(200.0 * 1e9)
    node._visited_frontiers = [
        (3.5, -1.2, 1, very_old_ns),
        (6.0, 4.0, 1, now_ns),
    ]
    # Trigger the prune via the `recent` check.
    _ = node._is_recently_visited(0.0, 0.0)
    if len(node._visited_frontiers) != 1:
        print(f"  [FAIL] expected aging to keep 1 entry, got "
              f"{len(node._visited_frontiers)}: {node._visited_frontiers}")
        sys.exit(1)
    print("  [OK] aged entry pruned, fresh entry kept")

    # ---- Test 5 — debug status surfaces visited counter & goal -------------
    print("\nTest 5 — debug status surfaces visited counter, in-flight goal")
    # Drain any messages still buffered in the subscription's middleware
    # queue (rclpy can hold the boot-time + 1Hz timer publishes from
    # before we modify state). Without this, inbox[-1] often returns
    # a stale "state=IDLE visited=0" snapshot.
    drain_end = time.time() + 0.5
    while time.time() < drain_end:
        executor.spin_once(timeout_sec=0.05)
    inbox.clear()
    node._inflight_goal_xy = (1.5, 2.5)
    node._dbg_last_frontier_goal_xy = (1.5, 2.5)
    node._dbg_last_nav2_status = "ACCEPTED"
    node._dbg_n_nav2_accepted = 7
    node._dbg_last_n_frontiers = 5
    node._dbg_last_selected_idx = 1
    node._dbg_last_selected_score = 42.0
    node._dbg_last_selection_reason = "picked#1 after_skipping_already_visited_frontier"
    node._set_state(_State.NAVIGATING)
    node._publish_debug_status()
    # Spin until we see the NAVIGATING message specifically (the 1Hz
    # timer will deliver it eventually if we miss the manual one).
    msg5 = ""
    end = time.time() + 2.0
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)
        for m in reversed(inbox):
            if "state=NAVIGATING" in m and "visited=" in m:
                msg5 = m
                break
        if msg5:
            break
    if not msg5:
        print("  [FAIL] no NAVIGATING-state debug status received")
        for i, m in enumerate(inbox):
            print(f"    inbox[{i}]: {m}")
        sys.exit(1)
    print(f"  raw: {msg5}")
    _assert_in("state=NAVIGATING", msg5, "state=NAVIGATING")
    _assert_in("goal=(1.50,2.50)", msg5, "in-flight goal echoed")
    _assert_in("nav2=ACCEPTED", msg5, "nav2=ACCEPTED echoed")
    _assert_in("goals_accepted=7", msg5, "goals_accepted counter")
    _assert_in("frontiers=5", msg5, "frontiers count")
    _assert_in("selected=1@42.0", msg5, "selected idx@score")
    # 3 calls to _mark_visited_frontier: (3.5,-1.2), (3.6,-1.1), (6.0,4.0).
    # Counter increments per call (not per unique entry), so we expect 3.
    _assert_in("visited=3", msg5, "visited counter (per-call increment)")
    # active entries went 0 -> 1 -> 1 (aggregated) -> 2 -> aged to 1 (Test 4).
    _assert_in("(1_active)", msg5, "active entries after aging")
    _assert_in("reason=picked#1 after_skipping_already_visited_frontier",
               msg5, "selection reason includes already_visited tag")

    # ---- Test 6 — operator restart wipes the visited blacklist -------------
    print("\nTest 6 — restart clears visited blacklist")
    inbox.clear()
    ctrl = String()
    ctrl.data = "restart"
    node._on_control(ctrl)
    if node._visited_frontiers:
        print(f"  [FAIL] expected empty visited list, got "
              f"{node._visited_frontiers}")
        sys.exit(1)
    if node._visited_frontier_count != 0:
        print(f"  [FAIL] expected counter reset, got "
              f"{node._visited_frontier_count}")
        sys.exit(1)
    print("  [OK] restart wiped visited list and counter")

    # ---- Test 7 — fast DONE transition when frontiers exhaust --------------
    # Day 9+ Phase B (May-9 mapping run). Symptom: a 120 s capture
    # showed mapping_status=NAVIGATING for the entire run even though
    # the last frontier had SUCCEEDED at ~t=116 s and every subsequent
    # /get_frontiers returned 0. The old code waited done_confirm_sec
    # (5 s default) before flipping DONE; the new code locks DONE
    # immediately when (n_succeeded > 0) AND last_nav2 == "SUCCEEDED"
    # AND nothing is in flight.
    print("\nTest 7 — fast DONE when frontiers exhaust after a SUCCEEDED goal")
    inbox.clear()
    # Seed the post-arrival state we'd see on a real run.
    node._set_state(_State.NAVIGATING)
    node._n_goals_succeeded = 2
    node._n_goals_sent = 2
    node._dbg_n_nav2_accepted = 2
    node._dbg_last_nav2_status = "SUCCEEDED"
    node._dbg_last_frontier_goal_xy = (7.87, 4.20)
    node._inflight_goal_xy = None
    node._nav_handle = None
    node._nav_goal_send_future = None
    node._mark_visited_frontier(7.87, 4.20)  # bump visited counter
    visited_before = node._visited_frontier_count
    # Call the empty-frontiers path directly; in production this is
    # invoked by _on_frontier_response when resp.frontier_goals is [].
    node._handle_empty_frontiers("test: no_frontiers_returned")
    if node._state != _State.DONE:
        print(f"  [FAIL] expected state=DONE, got {node._state.value}")
        sys.exit(1)
    print("  [OK] state transitioned to DONE on first empty response")
    if node._dbg_last_selection_reason != "no_frontiers_returned":
        print(
            "  [FAIL] expected selection_reason=no_frontiers_returned, "
            f"got {node._dbg_last_selection_reason!r}"
        )
        sys.exit(1)
    print("  [OK] selection_reason locked to no_frontiers_returned")

    # Drain stale messages (incl. the one we triggered inside
    # _handle_empty_frontiers via _publish_debug_status), then look
    # for the DONE-state line.
    drain_end = time.time() + 0.5
    while time.time() < drain_end:
        executor.spin_once(timeout_sec=0.05)
    msg7 = ""
    for m in reversed(inbox):
        if "state=DONE" in m:
            msg7 = m
            break
    if not msg7:
        # Force a fresh publish in case the inbox missed it.
        node._publish_debug_status()
        end = time.time() + 1.0
        while time.time() < end and not msg7:
            executor.spin_once(timeout_sec=0.05)
            for m in reversed(inbox):
                if "state=DONE" in m:
                    msg7 = m
                    break
    if not msg7:
        print("  [FAIL] no /mapping/debug/status with state=DONE received")
        for i, m in enumerate(inbox):
            print(f"    inbox[{i}]: {m}")
        sys.exit(1)
    print(f"  raw: {msg7}")
    _assert_in("state=DONE", msg7, "state=DONE in debug status")
    _assert_in("reason=no_frontiers_returned", msg7,
               "reason field carries no_frontiers_returned")
    _assert_in("goals_succeeded=2", msg7, "goals_succeeded surfaced")
    _assert_in(f"visited={visited_before}", msg7,
               "visited counter present in DONE line")

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    print("\n[ALL_OK] Day 9+ mapping_debug_status smoke passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
