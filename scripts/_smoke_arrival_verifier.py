"""Day 9 hot-fix smoke — distance-based arrival verifier + IN_FLIGHT.

Boots ``ApproachGoalPlannerNode`` and exercises:

  1. Far from goal_pose → no arrival yet.
  2. Within distance threshold but cmd_vel still moving → no arrival.
  3. Within distance threshold + cmd_vel near zero, held for hold_sec
     → /navigation/status "SUCCEEDED" + /arrival/status "ARRIVED_CONFIRMED:<class>"
     + action_debug "NavigateToPose RESULT: SUCCEEDED reason=distance_based_arrival_verifier".
  4. Same entity reaches arrival again → must NOT re-fire (latched).
  5. New entity_id resets the latch.
  6. enable_distance_arrival_verifier=False → no arrival regardless.

Also confirms IN_FLIGHT emission gates on _current_goal_handle being set.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional

# Hermetic isolation: pick an unusual ROS_DOMAIN_ID so a live
# day8_two_phase stack on the same host can't leak /target/selected
# or /cmd_vel into our smoke. Must be set BEFORE rclpy.init().
os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "SMOKE_ROS_DOMAIN_ID", "207"
)
# Also make sure no localhost-only override is lying around.
os.environ.pop("ROS_LOCALHOST_ONLY", None)

import rclpy
from geometry_msgs.msg import (
    Pose, PoseStamped, Quaternion, Twist,
)
from go2_msgs.msg import SelectedTarget
from std_msgs.msg import String

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(
    0, str(REPO / "src" / "go2_semantic_perception")
)

from go2_semantic_perception.approach_goal_planner_node import (  # noqa: E402
    ApproachGoalPlannerNode,
)


def _make_target(eid: str, cls: str = "person") -> SelectedTarget:
    s = SelectedTarget()
    s.entity_id = eid
    s.class_label = cls
    s.target_pose_map = Pose()
    s.target_pose_map.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    s.score = 1.0
    s.reachable = True
    s.estimated_distance = 1.0
    return s


def _set_goal_pose(node, x: float, y: float) -> None:
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.w = 1.0
    node._latest_goal_pose = ps


def _set_robot(node, x: float, y: float) -> None:
    node._lookup_robot_pose = lambda: (x, y, 0.0)  # type: ignore


def _set_cmd_vel(node, lin: float, ang: float) -> None:
    msg = Twist()
    msg.linear.x = lin
    msg.angular.z = ang
    node._latest_cmd_vel = msg
    node._latest_cmd_vel_time = node.get_clock().now()


class _Sink:
    def __init__(self, node, topic: str) -> None:
        self.msgs: List[str] = []
        node.create_subscription(String, topic, self._cb, 10)

    def _cb(self, msg: String) -> None:
        self.msgs.append(msg.data)

    def find(self, needle: str) -> Optional[str]:
        for m in reversed(self.msgs):
            if needle in m:
                return m
        return None

    def reset(self) -> None:
        self.msgs.clear()


def spin_for(executor, dt_s: float) -> None:
    end = time.time() + dt_s
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)


def _expect(sink: _Sink, executor, needle: str, dt_s: float = 1.5) -> None:
    end = time.time() + dt_s
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)
        if sink.find(needle):
            return
    raise AssertionError(
        f"expected '{needle}' within {dt_s}s, got={sink.msgs}"
    )


def _expect_absent(
    sink: _Sink, executor, needle: str, dt_s: float = 1.0,
) -> None:
    end = time.time() + dt_s
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)
    if sink.find(needle) is not None:
        raise AssertionError(
            f"expected '{needle}' to be ABSENT for {dt_s}s, "
            f"got={sink.msgs}"
        )


def main() -> int:
    rclpy.init()
    node = ApproachGoalPlannerNode()
    # Tighten timing so the smoke is fast.
    node._arrival_hold_sec = 0.3
    node._action_debug_throttle = 0.0
    node._inflight_period_sec = 0.2

    nav_sink = _Sink(node, "/navigation/status")
    arr_sink = _Sink(node, "/arrival/status")
    ad_sink = _Sink(node, "/semantic_goal/action_debug")

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_for(executor, 0.3)

    # ----- (1) Far away from goal_pose -------------------------------------
    sel = _make_target("person_777")
    node._latest_selected = sel
    _set_goal_pose(node, 5.0, 0.0)
    _set_robot(node, 0.0, 0.0)
    _set_cmd_vel(node, 0.0, 0.0)
    nav_sink.reset(); arr_sink.reset(); ad_sink.reset()
    node._on_arrival_tick()
    spin_for(executor, 0.5)
    _expect_absent(nav_sink, executor, "SUCCEEDED", dt_s=0.4)
    print("[PASS] far_from_goal_no_arrival")

    # ----- (2) Within dist but cmd_vel moving ------------------------------
    _set_goal_pose(node, 0.2, 0.0)
    _set_cmd_vel(node, 0.30, 0.0)  # moving fast
    nav_sink.reset(); arr_sink.reset(); ad_sink.reset()
    # Trigger several ticks to accumulate hold time.
    for _ in range(8):
        node._on_arrival_tick()
        spin_for(executor, 0.05)
    _expect_absent(nav_sink, executor, "SUCCEEDED", dt_s=0.4)
    print("[PASS] within_dist_but_moving_no_arrival")

    # ----- (3) Within dist + stopped + held → arrival ----------------------
    _set_cmd_vel(node, 0.0, 0.0)
    node._arrival_close_since = None  # restart hold timer cleanly
    nav_sink.reset(); arr_sink.reset(); ad_sink.reset()
    end = time.time() + 0.6
    while time.time() < end:
        node._on_arrival_tick()
        executor.spin_once(timeout_sec=0.05)
        if nav_sink.find("SUCCEEDED"):
            break
    _expect(nav_sink, executor, "SUCCEEDED", dt_s=0.5)
    _expect(arr_sink, executor, "ARRIVED_CONFIRMED:person", dt_s=0.5)
    _expect(
        ad_sink, executor,
        "NavigateToPose RESULT: SUCCEEDED "
        "reason=distance_based_arrival_verifier",
        dt_s=0.5,
    )
    print("[PASS] within_dist_and_stopped_arrives")

    # ----- (4) Same entity → must NOT re-fire ------------------------------
    nav_sink.reset(); arr_sink.reset(); ad_sink.reset()
    end = time.time() + 0.5
    while time.time() < end:
        node._on_arrival_tick()
        executor.spin_once(timeout_sec=0.05)
    _expect_absent(nav_sink, executor, "SUCCEEDED", dt_s=0.3)
    _expect_absent(arr_sink, executor, "ARRIVED_CONFIRMED", dt_s=0.3)
    print("[PASS] same_entity_no_re_arrival")

    # ----- (5) New entity_id resets latch ----------------------------------
    sel2 = _make_target("table_001", cls="table")
    nav_sink.reset(); arr_sink.reset(); ad_sink.reset()
    node._on_selected(sel2)   # exercises the latch reset
    assert node._arrival_published_for_entity is None, \
        "latch should clear on entity change"
    node._arrival_close_since = None
    _set_goal_pose(node, 0.2, 0.0)
    end = time.time() + 0.6
    while time.time() < end:
        node._on_arrival_tick()
        executor.spin_once(timeout_sec=0.05)
        if nav_sink.find("SUCCEEDED"):
            break
    _expect(arr_sink, executor, "ARRIVED_CONFIRMED:table", dt_s=0.5)
    print("[PASS] new_entity_relatches_arrival")

    # ----- (6) Disabled → no arrival ---------------------------------------
    node._distance_arrival_enabled = False
    sel3 = _make_target("person_disabled")
    node._on_selected(sel3)
    node._arrival_close_since = None
    nav_sink.reset(); arr_sink.reset(); ad_sink.reset()
    for _ in range(15):
        node._on_arrival_tick()
        executor.spin_once(timeout_sec=0.05)
    _expect_absent(nav_sink, executor, "SUCCEEDED", dt_s=0.5)
    print("[PASS] disabled_no_arrival")

    # ----- (7) IN_FLIGHT emission gating -----------------------------------
    # No goal handle set → must NOT emit IN_FLIGHT.
    node._distance_arrival_enabled = True
    node._current_goal_handle = None
    node._goal_accepted_time = None
    node._last_inflight_emit_time = None
    ad_sink.reset()
    node._maybe_emit_inflight(_make_target("person_777"))
    spin_for(executor, 0.1)
    if ad_sink.find("NavigateToPose IN_FLIGHT") is not None:
        raise AssertionError(
            "IN_FLIGHT must not emit when no goal handle is set"
        )
    # With a handle set → IN_FLIGHT must emit at least once.
    node._current_goal_handle = SimpleNamespace()
    node._goal_accepted_time = node.get_clock().now()
    node._last_inflight_emit_time = None
    ad_sink.reset()
    node._maybe_emit_inflight(_make_target("person_777"))
    spin_for(executor, 0.2)
    _expect(
        ad_sink, executor,
        "NavigateToPose IN_FLIGHT target='person_777'",
        dt_s=0.4,
    )
    print("[PASS] inflight_emits_only_when_handle_set")

    print("---\n7/7 arrival-verifier smoke tests passed.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
