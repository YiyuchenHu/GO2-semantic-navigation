import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from go2_msgs.msg import SelectedTarget
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .utils import heading_to, occupancy_at_xy, odom_pose_xyz, quaternion_from_yaw, safe_cost


class GoalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_planner_node")
        self.declare_parameter("num_angle_samples", 16)
        self.declare_parameter("cost_threshold", 60)
        # Phase 3A: the Phase 2 semantic map publishes entities in 'odom'
        # because there is no SLAM map yet. Make it a param so Phase 3B+
        # just flips it to 'map' when a real map frame comes online.
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("log_period_sec", 1.0)

        self._num_samples = int(self.get_parameter("num_angle_samples").value)
        self._cost_threshold = int(self.get_parameter("cost_threshold").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)

        self._selected: Optional[SelectedTarget] = None
        self._odom: Optional[Odometry] = None
        self._global: Optional[OccupancyGrid] = None
        self._local: Optional[OccupancyGrid] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Per-class approach stand-off distance (robot stops this far from
        # the target centroid). Easy to extend with more classes later.
        self._approach_dist: Dict[str, float] = {
            "chair": 0.9,
            "table": 1.0,
            "person": 1.2,
            "door": 1.1,
            "cup": 0.7,
        }

        # Diagnostic counters for the heartbeat.
        self._goals_published_total = 0
        self._last_published_selected_id: Optional[str] = None
        self._last_log_time = self.get_clock().now()

        self.create_subscription(SelectedTarget, "/semantic_query/selected_target", self._on_target, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(OccupancyGrid, "/costmap/global", self._on_global, 10)
        self.create_subscription(OccupancyGrid, "/costmap/local", self._on_local, 10)

        self._goal_pub = self.create_publisher(PoseStamped, "/semantic_goal/goal_pose", 10)
        self._cand_pub = self.create_publisher(MarkerArray, "/semantic_goal/goal_candidates", 10)
        self.create_timer(0.5, self._plan)

        self.get_logger().info(
            f"Goal planner ready. global_frame='{self._global_frame}' "
            f"base_frame='{self._base_frame}' "
            f"num_angle_samples={self._num_samples} "
            f"cost_threshold={self._cost_threshold}"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_target(self, msg: SelectedTarget) -> None:
        self._selected = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_global(self, msg: OccupancyGrid) -> None:
        self._global = msg

    def _on_local(self, msg: OccupancyGrid) -> None:
        self._local = msg

    # ------------------------------------------------------------------
    # Main planning pass
    # ------------------------------------------------------------------
    def _plan(self) -> None:
        if self._selected is None:
            self._maybe_heartbeat(
                valid=0,
                published=False,
                reason="waiting for /semantic_query/selected_target",
            )
            return

        tx = float(self._selected.target_pose_map.position.x)
        ty = float(self._selected.target_pose_map.position.y)
        target_cls = self._selected.class_label.lower()
        app_dist = self._approach_dist.get(target_cls, 0.9)

        robot = self._robot_pose_global()
        if robot is None:
            if self._odom is None:
                self._maybe_heartbeat(
                    valid=0,
                    published=False,
                    reason=(
                        "waiting for robot pose (no TF "
                        f"{self._global_frame}->{self._base_frame}, no /odom)"
                    ),
                )
                return
            robot = odom_pose_xyz(self._odom)

        candidates = self._sample_candidates(tx, ty, app_dist)
        valid: List[Tuple[np.ndarray, float]] = []
        marker_arr = MarkerArray()
        marker_arr.markers = []
        num_unsafe = 0

        for idx, c in enumerate(candidates):
            gx, gy = float(c[0]), float(c[1])
            cost_g = occupancy_at_xy(self._global, gx, gy)
            cost_l = occupancy_at_xy(self._local, gx, gy)
            is_safe = safe_cost(cost_g, self._cost_threshold) and safe_cost(
                cost_l, self._cost_threshold
            )
            marker_arr.markers.append(self._candidate_marker(idx, gx, gy, is_safe))
            if not is_safe:
                num_unsafe += 1
                continue
            dist_robot = float(
                np.linalg.norm(np.array([gx, gy], dtype=np.float32) - robot[:2])
            )
            # Score = prefer candidates closer to the robot. This keeps
            # the goal on the robot's side of the chair, which is both
            # intuitively "approach" and shortest drive distance.
            score = -dist_robot
            valid.append((c, score))

        # Publish candidate markers even when we can't find a valid one —
        # RViz then shows red spheres everywhere, which makes it obvious
        # that the costmap rejected the whole ring.
        if not valid:
            self._cand_pub.publish(marker_arr)
            self._maybe_heartbeat(
                valid=0,
                published=False,
                reason=(
                    f"0/{self._num_samples} candidates safe (target='{target_cls}' "
                    f"approach_dist={app_dist:.2f}m; {num_unsafe} unsafe)"
                ),
            )
            return

        valid.sort(key=lambda x: x[1], reverse=True)
        best = valid[0][0]
        gx, gy = float(best[0]), float(best[1])
        yaw = heading_to((gx, gy), (tx, ty))

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._global_frame
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation = quaternion_from_yaw(yaw)
        self._goal_pub.publish(goal)
        self._goals_published_total += 1

        # Extra arrow marker for the chosen goal. Same MarkerArray topic
        # as the candidates so one RViz display shows both.
        marker_arr.markers.append(self._final_goal_marker(gx, gy, yaw))
        # Thin line from robot to goal for intuitive "go there" arrow.
        marker_arr.markers.append(
            self._approach_line_marker(
                start=(float(robot[0]), float(robot[1])),
                end=(gx, gy),
            )
        )
        self._cand_pub.publish(marker_arr)

        # Log on every new selected target, not every goal — otherwise
        # 2 Hz of "PUBLISHED" would drown out everything else.
        if self._selected.entity_id != self._last_published_selected_id:
            self.get_logger().info(
                f"[goal-planner] PUBLISHED goal for entity="
                f"{self._selected.entity_id[:8]} class='{target_cls}' "
                f"target=({tx:.2f}, {ty:.2f}) "
                f"goal=({gx:.2f}, {gy:.2f}) yaw={yaw:.2f}rad "
                f"approach_dist={app_dist:.2f}m "
                f"valid_candidates={len(valid)}/{self._num_samples}"
            )
            self._last_published_selected_id = self._selected.entity_id

        self._maybe_heartbeat(
            valid=len(valid),
            published=True,
            reason=(
                f"goal=({gx:.2f}, {gy:.2f}) yaw={yaw:.2f} target='{target_cls}'"
            ),
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _robot_pose_global(self) -> Optional[np.ndarray]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            return np.array(
                [
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                ],
                dtype=np.float32,
            )
        except TransformException:
            return None

    def _sample_candidates(self, tx: float, ty: float, radius: float) -> List[np.ndarray]:
        out = []
        for i in range(self._num_samples):
            ang = (2.0 * math.pi * i) / self._num_samples
            out.append(
                np.array(
                    [tx + radius * math.cos(ang), ty + radius * math.sin(ang)],
                    dtype=np.float32,
                )
            )
        return out

    def _candidate_marker(self, idx: int, x: float, y: float, valid: bool) -> Marker:
        m = Marker()
        m.header.frame_id = self._global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "approach_candidates"
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15
        m.color.r = 0.9 if not valid else 0.1
        m.color.g = 0.9 if valid else 0.1
        m.color.b = 0.1
        m.color.a = 0.8
        return m

    def _final_goal_marker(self, x: float, y: float, yaw: float) -> Marker:
        m = Marker()
        m.header.frame_id = self._global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "approach_goal"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation = quaternion_from_yaw(yaw)
        m.scale.x = 0.6   # shaft length
        m.scale.y = 0.1   # shaft diameter
        m.scale.z = 0.1   # head diameter
        m.color.r = 0.1
        m.color.g = 0.4
        m.color.b = 1.0
        m.color.a = 0.95
        return m

    def _approach_line_marker(
        self, start: Tuple[float, float], end: Tuple[float, float]
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self._global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "approach_line"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.04   # line width
        m.color.r = 0.1
        m.color.g = 0.6
        m.color.b = 1.0
        m.color.a = 0.9
        from geometry_msgs.msg import Point

        p0 = Point(); p0.x = start[0]; p0.y = start[1]; p0.z = 0.1
        p1 = Point(); p1.x = end[0]; p1.y = end[1]; p1.z = 0.1
        m.points = [p0, p1]
        return m

    def _maybe_heartbeat(self, valid: int, published: bool, reason: str) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds / 1e9 < self._log_period:
            return
        self._last_log_time = now
        sel = (
            self._selected.entity_id[:8]
            if self._selected is not None and self._selected.entity_id
            else "None"
        )
        self.get_logger().info(
            f"[goal-planner] selected={sel} valid_candidates={valid} "
            f"goals_total={self._goals_published_total} "
            f"published_this_tick={published} {reason}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
