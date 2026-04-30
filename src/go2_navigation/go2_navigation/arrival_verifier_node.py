import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from go2_msgs.msg import ObjectObservationArray, SelectedTarget, SemanticEntityArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .utils import angle_diff, heading_to, odom_pose_xyz, yaw_from_quaternion


class ArrivalVerifierNode(Node):
    def __init__(self) -> None:
        super().__init__("arrival_verifier_node")
        self.declare_parameter("heading_tol_deg", 40.0)
        self.declare_parameter("recent_visible_sec", 2.0)
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("log_period_sec", 2.0)
        self._heading_tol_rad = math.radians(float(self.get_parameter("heading_tol_deg").value))
        self._recent_visible_ns = int(float(self.get_parameter("recent_visible_sec").value) * 1e9)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._log_period_ns = int(float(self.get_parameter("log_period_sec").value) * 1e9)
        # Per-class stop radius. These are "arrival" radii and should be
        # looser than the controller's stop_radius so that transient
        # overshoot doesn't flip arrival off/on/off.
        self._reach_dist = {"chair": 1.0, "table": 1.1, "person": 1.3, "door": 1.2, "cap": 0.8}

        self._goal: Optional[PoseStamped] = None
        self._selected: Optional[SelectedTarget] = None
        self._entities: Optional[SemanticEntityArray] = None
        self._objects_3d: Optional[ObjectObservationArray] = None
        self._odom: Optional[Odometry] = None
        self._last_seen_ns: dict[str, int] = {}
        self._last_status = ""
        self._last_heartbeat_ns = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(PoseStamped, "/semantic_goal/goal_pose", self._on_goal, 10)
        self.create_subscription(SelectedTarget, "/semantic_query/selected_target", self._on_selected, 10)
        self.create_subscription(SemanticEntityArray, "/semantic_map/entities", self._on_entities, 10)
        self.create_subscription(ObjectObservationArray, "/perception/objects_3d", self._on_objects, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self._status_pub = self.create_publisher(String, "/arrival/status", 10)
        self._msg_pub = self.create_publisher(String, "/user_guidance/message", 10)
        self.create_timer(0.5, self._verify)
        self.get_logger().info(
            f"Arrival verifier ready. global_frame={self._global_frame} "
            f"base_frame={self._base_frame} heading_tol={self._heading_tol_rad:.2f}rad"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg

    def _on_selected(self, msg: SelectedTarget) -> None:
        self._selected = msg

    def _on_entities(self, msg: SemanticEntityArray) -> None:
        self._entities = msg

    def _on_objects(self, msg: ObjectObservationArray) -> None:
        self._objects_3d = msg
        now_ns = self.get_clock().now().nanoseconds
        for obs in msg.observations:
            self._last_seen_ns[obs.class_label.lower()] = now_ns

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _verify(self) -> None:
        if self._selected is None:
            # Still publish so downstream consumers and `ros2 topic hz`
            # see a live stream instead of "topic not published yet".
            self._publish("WAITING_FOR_TARGET", "Searching for a target…")
            self._maybe_heartbeat("no selected target yet")
            return
        target_cls = self._selected.class_label.lower()
        target_pos = np.array(
            [
                self._selected.target_pose_map.position.x,
                self._selected.target_pose_map.position.y,
            ],
            dtype=np.float32,
        )
        robot_pose = self._robot_pose_map()
        if robot_pose is None:
            if self._odom is None:
                return
            robot_xyz = odom_pose_xyz(self._odom)
            robot_xy = robot_xyz[:2]
            robot_yaw = yaw_from_quaternion(self._odom.pose.pose.orientation)
        else:
            robot_xy = np.array([robot_pose[0], robot_pose[1]], dtype=np.float32)
            robot_yaw = float(robot_pose[3])
        dist = float(np.linalg.norm(robot_xy - target_pos))
        desired_yaw = heading_to((float(robot_xy[0]), float(robot_xy[1])), (float(target_pos[0]), float(target_pos[1])))
        heading_ok = abs(angle_diff(desired_yaw, robot_yaw)) < self._heading_tol_rad
        dist_ok = dist <= self._reach_dist.get(target_cls, 1.0)
        visible_ok = self._is_visible_recent(target_cls)

        if dist_ok and heading_ok and visible_ok:
            status = "ARRIVED_CONFIRMED"
            guidance = f"Arrived near the {target_cls}. It should be about {dist:.1f} m in front of you."
        else:
            reasons = []
            if not dist_ok:
                reasons.append(f"distance={dist:.2f}m")
            if not heading_ok:
                reasons.append("heading_not_aligned")
            if not visible_ok:
                reasons.append("target_not_recently_visible")
            status = "NOT_CONFIRMED:" + ",".join(reasons)
            guidance = f"Approaching {target_cls}: dist={dist:.2f}m."

        self._publish(status, guidance)

        if status != self._last_status:
            self.get_logger().info(
                f"[arrival] {status} target={target_cls} dist={dist:.2f}m "
                f"heading_err={abs(angle_diff(desired_yaw, robot_yaw)):.2f}rad "
                f"visible={visible_ok}"
            )
            self._last_status = status
        self._maybe_heartbeat(
            f"status={status} target={target_cls} dist={dist:.2f}m "
            f"heading_ok={heading_ok} visible={visible_ok}"
        )

    def _is_visible_recent(self, cls: str) -> bool:
        if self._entities is not None:
            for e in self._entities.entities:
                if e.class_label.lower() == cls and e.currently_visible:
                    return True
        now_ns = self.get_clock().now().nanoseconds
        if cls in self._last_seen_ns and (now_ns - self._last_seen_ns[cls]) < self._recent_visible_ns:
            return True
        return False

    def _publish(self, status: str, message: str) -> None:
        s = String()
        s.data = status
        self._status_pub.publish(s)
        g = String()
        g.data = message
        self._msg_pub.publish(g)

    def _maybe_heartbeat(self, note: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_heartbeat_ns < self._log_period_ns:
            return
        self._last_heartbeat_ns = now_ns
        self.get_logger().info(f"[arrival/hb] {note}")

    def _robot_pose_map(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            yaw = yaw_from_quaternion(tf.transform.rotation)
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
                yaw,
            )
        except TransformException:
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArrivalVerifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
