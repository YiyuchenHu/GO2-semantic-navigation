"""
Temporary validation aid:
Inject synthetic chair ObjectObservationArray when perception/localization is unavailable.

Do not use this in production runs.
"""

import uuid

import rclpy
from go2_msgs.msg import ObjectObservation, ObjectObservationArray
from rclpy.node import Node


class SyntheticChairObservationPublisher(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_chair_observation_publisher")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 2.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("z", 0.5)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("confidence", 0.9)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._x = float(self.get_parameter("x").value)
        self._y = float(self.get_parameter("y").value)
        self._z = float(self.get_parameter("z").value)
        self._conf = float(self.get_parameter("confidence").value)
        hz = float(self.get_parameter("publish_hz").value)
        period = 1.0 / max(0.1, hz)

        self._pub = self.create_publisher(ObjectObservationArray, "/perception/objects_3d", 10)
        self.create_timer(period, self._on_timer)
        self.get_logger().warning(
            "Synthetic chair injector enabled. This is ONLY for temporary integration validation."
        )

    def _on_timer(self) -> None:
        msg = ObjectObservationArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        obs = ObjectObservation()
        obs.header = msg.header
        obs.observation_id = str(uuid.uuid4())
        obs.source_detection_id = "synthetic"
        obs.class_label = "chair"
        obs.confidence = self._conf
        obs.centroid_map.x = self._x
        obs.centroid_map.y = self._y
        obs.centroid_map.z = self._z
        obs.centroid_base_link.x = self._x
        obs.centroid_base_link.y = self._y
        obs.centroid_base_link.z = self._z
        obs.size_xyz.x = 0.5
        obs.size_xyz.y = 0.5
        obs.size_xyz.z = 0.8
        obs.depth_median = 2.0
        obs.depth_p10 = 1.8
        obs.depth_p90 = 2.2
        obs.depth_valid_ratio = 1.0
        obs.uncertainty = 0.05
        obs.currently_visible = True
        msg.observations.append(obs)

        self._pub.publish(msg)
        self.get_logger().info(
            f"[TEMP] Published synthetic chair observation at map ({self._x:.2f}, {self._y:.2f}, {self._z:.2f})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticChairObservationPublisher()
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
