from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from .base_backend import NavigationBackend


class Nav2Backend(NavigationBackend):
    def __init__(self, node: Node) -> None:
        self._node = node
        self._status = "IDLE"
        self._goal_handle = None
        self._result_future = None
        self._available = False
        self._client: Optional[ActionClient] = None
        try:
            from nav2_msgs.action import NavigateToPose

            self._NavigateToPose = NavigateToPose
            self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")
            self._available = True
        except Exception:
            self._available = False

    @property
    def available(self) -> bool:
        return self._available

    def send_goal(self, goal: PoseStamped) -> bool:
        if not self._available or self._client is None:
            self._status = "BACKEND_UNAVAILABLE"
            return False
        if not self._client.wait_for_server(timeout_sec=0.5):
            self._status = "NAV2_SERVER_MISSING"
            return False

        goal_msg = self._NavigateToPose.Goal()
        goal_msg.pose = goal
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=0.3)
        if not future.done():
            self._status = "GOAL_SEND_TIMEOUT"
            return False
        self._goal_handle = future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self._status = "GOAL_REJECTED"
            return False
        self._result_future = self._goal_handle.get_result_async()
        self._status = "NAVIGATING"
        return True

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._status = "CANCELED"

    def status(self) -> str:
        if self._result_future is not None and self._result_future.done():
            result = self._result_future.result()
            if result is not None and result.status == 4:
                self._status = "SUCCEEDED"
            elif result is not None and result.status == 6:
                self._status = "ABORTED"
            elif result is not None and result.status == 5:
                self._status = "CANCELED"
            else:
                self._status = f"RESULT_{result.status if result else 'UNKNOWN'}"
        return self._status
