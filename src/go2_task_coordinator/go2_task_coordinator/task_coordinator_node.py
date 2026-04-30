from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from go2_msgs.msg import SelectedTarget, SemanticTask
from rclpy.node import Node
from std_msgs.msg import Bool, String


class FsmState(str, Enum):
    IDLE = "IDLE"
    PARSE_COMMAND = "PARSE_COMMAND"
    CHECK_MEMORY = "CHECK_MEMORY"
    TARGET_FOUND = "TARGET_FOUND"
    TARGET_NOT_FOUND = "TARGET_NOT_FOUND"
    SEARCH = "SEARCH"
    PLAN_APPROACH_GOAL = "PLAN_APPROACH_GOAL"
    NAVIGATE_TO_GOAL = "NAVIGATE_TO_GOAL"
    VERIFY_TARGET = "VERIFY_TARGET"
    ARRIVED = "ARRIVED"
    FAILED = "FAILED"
    SAFETY_STOP = "SAFETY_STOP"


class TaskCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("task_coordinator_node")
        self._state = FsmState.IDLE
        self._current_task: Optional[SemanticTask] = None
        self._selected_target: Optional[SelectedTarget] = None
        self._goal: Optional[PoseStamped] = None
        self._navigation_status = "IDLE"
        self._arrival_status = "UNKNOWN"
        self._safety_status = "OK"
        self._state_enter_ns = self.get_clock().now().nanoseconds

        self.create_subscription(String, "/user_command", self._on_user_command, 10)
        self.create_subscription(SemanticTask, "/semantic_task/request", self._on_semantic_task, 10)
        self.create_subscription(SelectedTarget, "/semantic_query/selected_target", self._on_selected_target, 10)
        self.create_subscription(PoseStamped, "/semantic_goal/goal_pose", self._on_goal, 10)
        self.create_subscription(String, "/navigation/status", self._on_nav_status, 10)
        self.create_subscription(String, "/arrival/status", self._on_arrival_status, 10)
        self.create_subscription(String, "/safety/status", self._on_safety_status, 10)

        self._task_status_pub = self.create_publisher(String, "/task/status", 10)
        self._task_current_pub = self.create_publisher(SemanticTask, "/semantic_task/current", 10)
        self._explore_pub = self.create_publisher(Bool, "/exploration/enabled", 10)
        self._cancel_pub = self.create_publisher(Bool, "/navigation/cancel", 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("Task coordinator ready.")

    def _on_user_command(self, _: String) -> None:
        self._set_state(FsmState.PARSE_COMMAND)

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        self._current_task = msg
        self._task_current_pub.publish(msg)
        self._set_state(FsmState.CHECK_MEMORY)

    def _on_selected_target(self, msg: SelectedTarget) -> None:
        if self._current_task is None or msg.task_id != self._current_task.task_id:
            return
        self._selected_target = msg
        self._set_state(FsmState.TARGET_FOUND)

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        if self._state in (FsmState.PLAN_APPROACH_GOAL, FsmState.TARGET_FOUND):
            self._set_state(FsmState.NAVIGATE_TO_GOAL)

    def _on_nav_status(self, msg: String) -> None:
        self._navigation_status = msg.data

    def _on_arrival_status(self, msg: String) -> None:
        self._arrival_status = msg.data

    def _on_safety_status(self, msg: String) -> None:
        self._safety_status = msg.data
        if "STOP" in msg.data.upper():
            self._set_state(FsmState.SAFETY_STOP)

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        state_age_sec = (now_ns - self._state_enter_ns) / 1e9

        if self._state == FsmState.CHECK_MEMORY and state_age_sec > 2.0 and self._selected_target is None:
            self._set_state(FsmState.TARGET_NOT_FOUND)
        if self._state == FsmState.TARGET_FOUND:
            self._set_state(FsmState.PLAN_APPROACH_GOAL)
        if self._state == FsmState.TARGET_NOT_FOUND:
            self._set_state(FsmState.SEARCH)
        if self._state == FsmState.NAVIGATE_TO_GOAL:
            if self._navigation_status in ("SUCCEEDED", "RESULT_4"):
                self._set_state(FsmState.VERIFY_TARGET)
            elif "ABORT" in self._navigation_status:
                self._set_state(FsmState.FAILED)
        if self._state == FsmState.VERIFY_TARGET:
            if self._arrival_status.startswith("ARRIVED_CONFIRMED"):
                self._set_state(FsmState.ARRIVED)
            elif state_age_sec > 8.0:
                self._set_state(FsmState.SEARCH)
        if self._state == FsmState.SAFETY_STOP:
            self._publish_cancel(True)

        self._publish_controls()

    def _publish_controls(self) -> None:
        status = String()
        status.data = self._state.value
        self._task_status_pub.publish(status)

        exploration_on = self._state in (FsmState.SEARCH, FsmState.TARGET_NOT_FOUND)
        explore = Bool()
        explore.data = exploration_on
        self._explore_pub.publish(explore)

        if self._state not in (FsmState.SAFETY_STOP, FsmState.FAILED):
            self._publish_cancel(False)

    def _publish_cancel(self, flag: bool) -> None:
        c = Bool()
        c.data = flag
        self._cancel_pub.publish(c)

    def _set_state(self, new_state: FsmState) -> None:
        if self._state == new_state:
            return
        self._state = new_state
        self._state_enter_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f"FSM -> {new_state.value}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
