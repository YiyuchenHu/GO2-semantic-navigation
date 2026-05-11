"""Smoke: NavigateToPose terminal result clears replan throttle + logs RESET_THROTTLE.

After ABORT, ``_last_sent_entity_id`` / ``_last_sent_target_xy`` must be None so a
second identical ``/target/selected`` tick can call ``_send_goal`` again instead of
emitting only ``throttled_target_unchanged``.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import List

os.environ["ROS_DOMAIN_ID"] = os.environ.get("SMOKE_ROS_DOMAIN_ID", "207")
os.environ.pop("ROS_LOCALHOST_ONLY", None)

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from go2_msgs.msg import SelectedTarget
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "go2_semantic_perception"))

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


def _make_target(eid: str, x: float, y: float) -> SelectedTarget:
    s = SelectedTarget()
    s.entity_id = eid
    s.class_label = "person"
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
    def __init__(self, node) -> None:
        self.msgs: List[str] = []
        node.create_subscription(
            String, "/semantic_goal/action_debug", self._cb, 10,
        )

    def _cb(self, msg: String) -> None:
        self.msgs.append(msg.data)

    def find(self, needle: str) -> bool:
        return any(needle in m for m in self.msgs)


def spin_for(executor, dt_s: float) -> None:
    end = time.time() + dt_s
    while time.time() < end:
        executor.spin_once(timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = ApproachGoalPlannerNode()
    node.set_parameters(
        [
            rclpy.parameter.Parameter(
                "action_debug_throttle_period_sec",
                rclpy.parameter.Parameter.Type.DOUBLE,
                0.0,
            ),
        ]
    )
    node._action_debug_throttle = 0.0
    node._lookup_robot_pose = lambda: (0.0, 0.0, 0.0)  # type: ignore
    node._costmap = _make_costmap()
    node._current_class_label = "person"
    node._last_sent_entity_id = "person_001"
    node._last_sent_target_xy = (0.5, 0.0)

    sink = _DebugSink(node)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_for(executor, 0.25)

    res = NavigateToPose.Result()
    res.error_code = 1
    res.error_msg = "smoke_abort"
    wrap = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=res,
    )
    fut = SimpleNamespace(result=lambda: wrap)
    node._on_result(fut)
    spin_for(executor, 0.2)

    assert node._last_sent_entity_id is None, node._last_sent_entity_id
    assert node._last_sent_target_xy is None, node._last_sent_target_xy
    assert sink.find("RESET_THROTTLE reason=result_ABORTED")

    sends: List[bool] = []

    def _stub_send_goal(*args, **kwargs) -> bool:  # type: ignore[no-untyped-def]
        sends.append(True)
        return True

    node._send_goal = _stub_send_goal  # type: ignore[method-assign]

    sink.msgs.clear()
    node._latest_selected = _make_target("person_001", 0.5, 0.0)
    node._on_replan_tick()
    spin_for(executor, 0.2)

    assert sends, "_send_goal did not run after ABORT throttle reset"
    assert not sink.find("throttled_target_unchanged"), sink.msgs

    print("[PASS] approach_goal_planner throttle reset after ABORT + replay SEND.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
