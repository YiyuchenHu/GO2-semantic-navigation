"""Phase 3B nav executor: own the closed loop from /semantic_goal/goal_pose
to /cmd_vel, via a pluggable backend.

Default backend is `simple_p_controller` — a small proportional
controller (see backends/simple_p_controller_backend.py). `nav2` and
the legacy `go2_velocity` stub are still selectable through the
`backend` parameter for future swap-in.
"""

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .backends.go2_velocity_backend import Go2VelocityBackend
from .backends.nav2_backend import Nav2Backend
from .backends.simple_p_controller_backend import SimplePControllerBackend


class NavExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_executor_node")
        self.declare_parameter("backend", "simple_p_controller")
        self.declare_parameter("log_period_sec", 2.0)
        backend_name = str(self.get_parameter("backend").value)
        self._log_period_ns = int(float(self.get_parameter("log_period_sec").value) * 1e9)

        self._backend = self._create_backend(backend_name)
        self._last_goal_key: Optional[str] = None
        self._goals_received = 0
        self._goals_accepted = 0
        self._last_heartbeat_ns = 0

        self.create_subscription(PoseStamped, "/semantic_goal/goal_pose", self._on_goal, 10)
        self.create_subscription(Bool, "/navigation/cancel", self._on_cancel, 10)
        self._status_pub = self.create_publisher(String, "/navigation/status", 10)
        self.create_timer(0.2, self._publish_status)
        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info(f"Nav executor ready with backend={backend_name}")

    def _create_backend(self, backend_name: str):
        if backend_name == "simple_p_controller":
            return SimplePControllerBackend(self)
        if backend_name == "go2_velocity":
            self.get_logger().warning(
                "Using legacy go2_velocity stub backend. No /cmd_vel will be published."
            )
            return Go2VelocityBackend(self)
        if backend_name == "nav2":
            nav2 = Nav2Backend(self)
            if nav2.available:
                return nav2
            self.get_logger().warning(
                "Nav2 backend unavailable, falling back to simple_p_controller."
            )
            return SimplePControllerBackend(self)
        self.get_logger().warning(
            f"Unknown backend '{backend_name}', falling back to simple_p_controller."
        )
        return SimplePControllerBackend(self)

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goals_received += 1
        # Deduplicate repeated goal poses (Phase 3A publishes at ~2 Hz; the
        # pose itself only changes when the robot moves). Rounding to
        # millimetres is enough to collapse stationary re-publishes.
        key = f"{msg.pose.position.x:.3f}-{msg.pose.position.y:.3f}-{msg.pose.orientation.z:.3f}"
        if key == self._last_goal_key:
            return
        self._last_goal_key = key
        ok = self._backend.send_goal(msg)
        if ok:
            self._goals_accepted += 1
        else:
            self._publish_status("GOAL_REJECTED_OR_BACKEND_UNAVAILABLE")

    def _on_cancel(self, msg: Bool) -> None:
        if msg.data:
            self._backend.cancel()
            self._publish_status("CANCELED")

    def _publish_status(self, forced: Optional[str] = None) -> None:
        s = String()
        s.data = forced if forced is not None else self._backend.status()
        self._status_pub.publish(s)

    def _heartbeat(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_heartbeat_ns < self._log_period_ns:
            return
        self._last_heartbeat_ns = now_ns
        self.get_logger().info(
            f"[nav-exec/hb] status={self._backend.status()} "
            f"goals_received={self._goals_received} goals_accepted={self._goals_accepted}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
