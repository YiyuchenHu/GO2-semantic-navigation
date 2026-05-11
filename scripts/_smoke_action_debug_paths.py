"""Day 9 hot-fix smoke — every approach_goal_planner code path that
could have starved /semantic_goal/action_debug must now emit a
String message on it.

Boots an instance of ``ApproachGoalPlannerNode``, drains its
publisher into a private subscription, and exercises:

  1. _on_replan_tick with no target           → NOSEND no_target_selected
  2. _on_replan_tick with target, no costmap  → NOSEND no_costmap
  3. _on_replan_tick with target+costmap, no TF
                                              → NOSEND no_tf_to_base_link
  4. _on_replan_tick with target+costmap+TF,
     same target as last_sent (throttled)     → NOSEND throttled_*
  5. _send_goal with action server unavailable
                                              → SEND_FAILED action_server_unavailable
  6. auto_send_goal=False                     → NOSEND auto_send_goal_disabled

Each step asserts that exactly the expected ``NavigateToPose``
substring appears on /semantic_goal/action_debug within a short
timeout. Does NOT need Nav2, Isaac, or any external publisher up.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import List, Optional

# Hermetic isolation — see _smoke_arrival_verifier.py for context.
os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "SMOKE_ROS_DOMAIN_ID", "207"
)
os.environ.pop("ROS_LOCALHOST_ONLY", None)

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from go2_msgs.msg import SelectedTarget
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(
    0, str(REPO / "src" / "go2_semantic_perception")
)

from go2_semantic_perception.approach_goal_planner_node import (  # noqa: E402
    ApproachGoalPlannerNode,
)


def _make_costmap() -> OccupancyGrid:
    cm = OccupancyGrid()
    cm.header.frame_id = "map"
    cm.info.resolution = 0.05
    cm.info.width = 100
    cm.info.height = 100
    cm.info.origin.position.x = -2.5
    cm.info.origin.position.y = -2.5
    cm.info.origin.orientation.w = 1.0
    cm.data = [0] * (cm.info.width * cm.info.height)
    return cm


def _make_target(eid: str = "t1", x: float = 1.0, y: float = 0.0) -> SelectedTarget:
    s = SelectedTarget()
    s.entity_id = eid
    s.class_label = "table"
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    s.target_pose_map = p
    s.score = 1.0
    s.reachable = True
    s.estimated_distance = 1.0
    return s


class _DebugSink:
    """Captures everything written to /semantic_goal/action_debug."""

    def __init__(self, node) -> None:
        self.msgs: List[str] = []
        node.create_subscription(
            String, "/semantic_goal/action_debug",
            self._cb, 10,
        )

    def _cb(self, msg: String) -> None:
        self.msgs.append(msg.data)

    def latest(self) -> Optional[str]:
        return self.msgs[-1] if self.msgs else None

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


def _assert_emits(
    sink: _DebugSink, executor, needle: str, dt_s: float = 1.5,
) -> None:
    end = time.time() + dt_s
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)
        if sink.find(needle) is not None:
            return
    raise AssertionError(
        f"expected '{needle}' on /semantic_goal/action_debug within "
        f"{dt_s}s, captured={sink.msgs}"
    )


def main() -> int:
    rclpy.init()
    # Force the throttle to ~0 so each tick can emit immediately
    # in this smoke test (the node default is 1 s).
    node = ApproachGoalPlannerNode()
    node.set_parameters(
        [
            rclpy.parameter.Parameter(
                "action_debug_throttle_period_sec",
                rclpy.parameter.Parameter.Type.DOUBLE, 0.0,
            ),
        ]
    )
    node._action_debug_throttle = 0.0

    sink = _DebugSink(node)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    # Allow pub/sub discovery.
    spin_for(executor, 0.3)

    # ----- (1) no target ---------------------------------------------------
    sink.reset()
    node._on_replan_tick()
    _assert_emits(sink, executor, "NOSEND reason=no_target_selected")
    print("[PASS] no_target_selected")

    # ----- (2) target without costmap --------------------------------------
    sink.reset()
    node._latest_selected = _make_target("t_nocostmap")
    node._on_replan_tick()
    _assert_emits(sink, executor, "NOSEND reason=no_costmap")
    print("[PASS] no_costmap")

    # ----- (3) target + costmap, but no TF ---------------------------------
    sink.reset()
    node._costmap = _make_costmap()
    # _lookup_robot_pose returns None because the buffer has nothing.
    node._on_replan_tick()
    _assert_emits(sink, executor, "NOSEND reason=no_tf_to_base_link")
    print("[PASS] no_tf_to_base_link")

    # ----- (4) target + costmap + TF, throttled (same entity) --------------
    # Force the planner past the TF gate by stubbing _lookup_robot_pose.
    sink.reset()
    node._lookup_robot_pose = lambda: (0.0, 0.0, 0.0)  # type: ignore
    sel = _make_target("t_throttle", 0.5, 0.0)
    node._latest_selected = sel
    # Pretend we already sent on this entity at the same xy.
    node._last_sent_entity_id = "t_throttle"
    node._last_sent_target_xy = (0.5, 0.0)
    node._on_replan_tick()
    _assert_emits(
        sink, executor, "NOSEND reason=throttled_target_unchanged",
    )
    print("[PASS] throttled_target_unchanged")

    # ----- (5) action server unavailable -> SEND_FAILED --------------------
    # Reset throttle / committed-target so the next tick takes the
    # send-fresh-goal branch instead of throttle.
    sink.reset()
    node._last_sent_entity_id = None
    node._last_sent_target_xy = None
    sel2 = _make_target("t_send", 0.6, 0.0)
    node._latest_selected = sel2
    # Stub wait_for_server False to trigger SEND_FAILED.
    node._nav_client.wait_for_server = lambda timeout_sec=1.0: False  # type: ignore
    node._on_replan_tick()
    _assert_emits(
        sink, executor,
        "SEND_FAILED action_server_unavailable",
    )
    print("[PASS] send_failed_action_server_unavailable")

    # ----- (6) auto_send_goal=False -> NOSEND auto_send_goal_disabled -----
    sink.reset()
    node._auto_send_goal = False
    node._last_sent_entity_id = None
    node._last_sent_target_xy = None
    node._latest_selected = _make_target("t_off", 0.7, 0.0)
    node._on_replan_tick()
    _assert_emits(
        sink, executor, "NOSEND reason=auto_send_goal_disabled",
    )
    print("[PASS] auto_send_goal_disabled")

    print("---\n6/6 action_debug-path smoke tests passed.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
