import math
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import Bool, String


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor_node")
        self.declare_parameter("max_speed_mps", 1.2)
        self.declare_parameter("max_pitch_roll_deg", 25.0)
        self.declare_parameter("min_obstacle_distance_m", 0.35)
        self._max_speed = float(self.get_parameter("max_speed_mps").value)
        self._max_pr = math.radians(float(self.get_parameter("max_pitch_roll_deg").value))
        self._min_obs = float(self.get_parameter("min_obstacle_distance_m").value)

        self._odom: Optional[Odometry] = None
        self._imu: Optional[Imu] = None
        self._points: Optional[PointCloud2] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._joint_states: Optional[JointState] = None
        self._emergency = False

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
        self.create_subscription(PointCloud2, "/camera/depth/points", self._on_points, 10)
        self.create_subscription(OccupancyGrid, "/costmap/local", self._on_costmap, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint, 10)
        self.create_subscription(Bool, "/emergency_stop", self._on_emergency, 10)

        self._status_pub = self.create_publisher(String, "/safety/status", 10)
        self._cancel_pub = self.create_publisher(Bool, "/navigation/cancel", 10)
        self.create_timer(0.1, self._check)
        self.get_logger().info("Safety monitor ready.")

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_imu(self, msg: Imu) -> None:
        self._imu = msg

    def _on_points(self, msg: PointCloud2) -> None:
        self._points = msg

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _on_joint(self, msg: JointState) -> None:
        self._joint_states = msg

    def _on_emergency(self, msg: Bool) -> None:
        self._emergency = bool(msg.data)

    def _check(self) -> None:
        reasons = []
        if self._emergency:
            reasons.append("EMERGENCY_STOP")

        if self._odom is not None:
            v = self._odom.twist.twist.linear
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            if speed > self._max_speed:
                reasons.append(f"OVERSPEED:{speed:.2f}")

        if self._imu is not None:
            roll, pitch = self._roll_pitch(self._imu)
            if abs(roll) > self._max_pr or abs(pitch) > self._max_pr:
                reasons.append("EXCESSIVE_TILT")

        nearest = self._nearest_pointcloud_distance(self._points)
        if nearest is not None and nearest < self._min_obs:
            reasons.append(f"OBSTACLE_TOO_CLOSE:{nearest:.2f}")

        if self._costmap is not None and self._local_cost_bad(self._costmap):
            reasons.append("LOCAL_COSTMAP_BLOCKED")

        if reasons:
            self._publish_status("STOP:" + ",".join(reasons), cancel=True)
        else:
            self._publish_status("OK", cancel=False)

    def _publish_status(self, status: str, cancel: bool) -> None:
        s = String()
        s.data = status
        self._status_pub.publish(s)
        c = Bool()
        c.data = cancel
        self._cancel_pub.publish(c)

    @staticmethod
    def _roll_pitch(imu: Imu) -> tuple[float, float]:
        q = imu.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        return roll, pitch

    @staticmethod
    def _nearest_pointcloud_distance(points: Optional[PointCloud2]) -> Optional[float]:
        if points is None:
            return None
        try:
            from sensor_msgs_py import point_cloud2

            min_d = None
            count = 0
            for p in point_cloud2.read_points(points, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = p
                d = math.sqrt(x * x + y * y + z * z)
                if min_d is None or d < min_d:
                    min_d = d
                count += 1
                if count > 1500:
                    break
            return min_d
        except Exception:
            return None

    @staticmethod
    def _local_cost_bad(costmap: OccupancyGrid) -> bool:
        w, h = costmap.info.width, costmap.info.height
        if w == 0 or h == 0:
            return False
        cx, cy = w // 2, h // 2
        idx = cy * w + cx
        if idx < 0 or idx >= len(costmap.data):
            return False
        center = int(costmap.data[idx])
        return center >= 90


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
