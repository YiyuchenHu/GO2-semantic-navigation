import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntegrationCommandPublisher(Node):
    """Publish test command a few times to ensure delivery."""

    def __init__(self) -> None:
        super().__init__("integration_command_publisher")
        self.declare_parameter("command_text", "go to the chair")
        self.declare_parameter("repeat_count", 5)
        self.declare_parameter("repeat_interval_sec", 0.5)
        self.declare_parameter("startup_delay_sec", 1.5)
        self.declare_parameter("min_subscribers", 2)
        self.declare_parameter("max_wait_for_subscribers_sec", 5.0)

        self._command = str(self.get_parameter("command_text").value)
        self._repeat_count = int(self.get_parameter("repeat_count").value)
        interval = float(self.get_parameter("repeat_interval_sec").value)
        self._startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self._min_subscribers = int(self.get_parameter("min_subscribers").value)
        self._max_wait_sub_sec = float(self.get_parameter("max_wait_for_subscribers_sec").value)
        self._sent = 0
        self._t0_ns = self.get_clock().now().nanoseconds

        self._pub = self.create_publisher(String, "/user_command", 10)
        self._timer = self.create_timer(interval, self._on_timer)
        self.get_logger().info(f"Ready to publish integration command: '{self._command}'")

    def _on_timer(self) -> None:
        elapsed_sec = (self.get_clock().now().nanoseconds - self._t0_ns) / 1e9
        if elapsed_sec < self._startup_delay_sec:
            return
        subs = self._pub.get_subscription_count()
        if subs < self._min_subscribers and elapsed_sec < self._startup_delay_sec + self._max_wait_sub_sec:
            self.get_logger().info(
                f"Waiting subscribers for /user_command ({subs}/{self._min_subscribers})..."
            )
            return

        if self._sent >= self._repeat_count:
            self.get_logger().info("Integration command publishing complete.")
            self.destroy_timer(self._timer)
            rclpy.shutdown()
            return

        msg = String()
        msg.data = self._command
        self._pub.publish(msg)
        self._sent += 1
        self.get_logger().info(f"Published /user_command [{self._sent}/{self._repeat_count}]: {msg.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegrationCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
