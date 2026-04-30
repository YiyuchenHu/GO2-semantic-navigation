from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .base_backend import NavigationBackend


class Go2VelocityBackend(NavigationBackend):
    """
    Stub integration point for real Go2 locomotion controller.
    Replace this class with actual high-level velocity command bridge.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._status = "IDLE"

    def send_goal(self, goal: PoseStamped) -> bool:
        self._node.get_logger().warning(
            "go2_velocity_backend is a stub. Integrate real Go2 controller here. "
            f"Received goal in map: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
        )
        self._status = "STUB_BACKEND_NO_CONTROLLER"
        return False

    def cancel(self) -> None:
        self._status = "CANCELED"

    def status(self) -> str:
        return self._status
