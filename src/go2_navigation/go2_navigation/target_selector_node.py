from typing import Dict, List, Optional

import numpy as np
import rclpy
from go2_msgs.msg import SelectedTarget, SemanticEntityArray, SemanticTask
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .utils import distance_xy, entity_pose_xyz, occupancy_at_xy, odom_pose_xyz, safe_cost


class TargetSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("target_selector_node")
        # Phase 3A: Phase 2 publishes entities in 'odom' because no SLAM
        # map frame exists yet. Accept the frame as a launch parameter so
        # this node works the same once Phase 3B+ introduces a 'map'
        # frame — just pass `global_frame:=map` at launch time.
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # Phase 3A default target. Semantic memory already normalizes the
        # chair's class_label to "chair" (see Phase 1 label alias table),
        # so this is the right default for the chair-only MVP. A real
        # /semantic_task/request still overrides this when it arrives.
        self.declare_parameter("default_target_class", "chair")
        self.declare_parameter("log_period_sec", 1.0)
        # How often to run the selection pass (selection is also data-
        # driven via the subscriptions, but this timer guarantees we
        # heartbeat even when the entity topic goes silent).
        self.declare_parameter("select_period_sec", 0.5)

        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._default_target_class = (
            str(self.get_parameter("default_target_class").value).lower().strip()
        )
        self._log_period = float(self.get_parameter("log_period_sec").value)
        self._select_period = float(self.get_parameter("select_period_sec").value)

        self._task: Optional[SemanticTask] = None
        self._entities: Optional[SemanticEntityArray] = None
        self._odom: Optional[Odometry] = None
        self._grid: Optional[OccupancyGrid] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Diagnostic counters for the heartbeat.
        self._selections_total = 0
        self._last_selected_id: Optional[str] = None
        self._last_log_time = self.get_clock().now()

        self.create_subscription(SemanticTask, "/semantic_task/request", self._on_task, 10)
        self.create_subscription(SemanticEntityArray, "/semantic_map/entities", self._on_entities, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_grid, 10)
        self.create_subscription(OccupancyGrid, "/costmap/global", self._on_grid, 10)

        self._pub = self.create_publisher(SelectedTarget, "/semantic_query/selected_target", 10)
        # Phase 3A: RViz-friendly marker on the selected entity. Separate
        # topic from /semantic_map/markers so operators can toggle them
        # independently.
        self._marker_pub = self.create_publisher(
            MarkerArray, "/semantic_query/selected_target_marker", 10
        )
        self.create_timer(self._select_period, self._run_selection)

        self.get_logger().info(
            f"Target selector ready. global_frame='{self._global_frame}' "
            f"base_frame='{self._base_frame}' "
            f"default_target_class='{self._default_target_class}' "
            f"select_period={self._select_period:.2f}s"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_task(self, msg: SemanticTask) -> None:
        self._task = msg
        self.get_logger().info(
            f"[target-selector] SemanticTask received: task_id={msg.task_id!r} "
            f"target_class={msg.target_class!r}"
        )

    def _on_entities(self, msg: SemanticEntityArray) -> None:
        self._entities = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _on_grid(self, msg: OccupancyGrid) -> None:
        self._grid = msg

    # ------------------------------------------------------------------
    # Main selection pass
    # ------------------------------------------------------------------
    def _run_selection(self) -> None:
        # Phase 3A: we do NOT gate on SemanticTask presence any more. If
        # no task has arrived, fall back to `default_target_class` so
        # that the chair-only MVP runs standalone without a task
        # coordinator.
        if self._task is not None and self._task.target_class:
            target_cls = self._task.target_class.lower().strip()
            task_id = self._task.task_id
        else:
            target_cls = self._default_target_class
            task_id = "mvp-default"

        if self._entities is None:
            self._maybe_heartbeat(
                candidates=0,
                reason=f"waiting for /semantic_map/entities, target_class='{target_cls}'",
                selected=None,
            )
            return

        robot = self._robot_pose_global()
        if robot is None:
            if self._odom is None:
                self._maybe_heartbeat(
                    candidates=0,
                    reason=(
                        "waiting for robot pose (no TF "
                        f"{self._global_frame}->{self._base_frame}, no /odom)"
                    ),
                    selected=None,
                )
                return
            robot = odom_pose_xyz(self._odom)

        best_score = -1e9
        best = None
        best_reasons: List[str] = []
        candidate_count = 0

        for entity in self._entities.entities:
            if entity.class_label.lower() != target_cls:
                continue
            candidate_count += 1
            epos = entity_pose_xyz(entity)
            dist = distance_xy(robot, epos)
            recency_sec = max(
                0.0,
                (
                    self.get_clock().now().nanoseconds
                    - (entity.last_seen.sec * int(1e9) + entity.last_seen.nanosec)
                )
                / 1e9,
            )
            occ = occupancy_at_xy(self._grid, float(epos[0]), float(epos[1]))
            reachable = safe_cost(occ, threshold=70)
            # Simple weighted score. Kept identical to the pre-existing
            # heuristic so Phase 3A is a config/wiring change, not a
            # ranking redesign.
            vis_bonus = 0.25 if entity.currently_visible else 0.0
            recency_bonus = max(0.0, 2.0 - 0.05 * recency_sec)
            dist_penalty = min(3.0, dist / 4.0)
            reach_bonus = 1.0 if reachable else -1.5
            conf_bonus = 2.0 * float(entity.confidence)
            uncertainty_penalty = min(1.0, float(entity.uncertainty))
            score = (
                conf_bonus
                + recency_bonus
                + vis_bonus
                + reach_bonus
                - dist_penalty
                - uncertainty_penalty
            )

            if score > best_score:
                best_score = score
                best = (entity, dist, reachable)
                best_reasons = [
                    f"class_match={entity.class_label}",
                    f"confidence={entity.confidence:.2f}",
                    f"recency_sec={recency_sec:.1f}",
                    f"distance_m={dist:.2f}",
                    f"reachable={reachable}",
                    f"visible={entity.currently_visible}",
                    f"score={score:.2f}",
                ]

        if best is None:
            self._maybe_heartbeat(
                candidates=0,
                reason=(
                    f"no entity with class_label='{target_cls}' among "
                    f"{len(self._entities.entities)} total entities"
                ),
                selected=None,
            )
            return

        entity, dist, reachable = best
        out = SelectedTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._global_frame
        out.task_id = task_id
        out.entity_id = entity.entity_id
        out.class_label = entity.class_label
        out.target_pose_map = entity.pose_map
        out.score = float(best_score)
        out.reachable = bool(reachable)
        out.estimated_distance = float(dist)
        out.ranking_reasons = best_reasons
        self._pub.publish(out)

        self._selections_total += 1
        if entity.entity_id != self._last_selected_id:
            # Log on every change of selected entity so operators can see
            # when the target switched.
            self.get_logger().info(
                f"[target-selector] SELECTED entity={entity.entity_id[:8]} "
                f"class='{entity.class_label}' "
                f"pose=({entity.pose_map.position.x:.2f}, "
                f"{entity.pose_map.position.y:.2f}, "
                f"{entity.pose_map.position.z:.2f}) "
                f"dist={dist:.2f}m conf={entity.confidence:.2f} "
                f"reachable={reachable} score={best_score:.2f}"
            )
            self._last_selected_id = entity.entity_id

        self._publish_selected_marker(entity)

        self._maybe_heartbeat(
            candidates=candidate_count,
            reason=(
                f"selected entity={entity.entity_id[:8]} "
                f"dist={dist:.2f}m score={best_score:.2f}"
            ),
            selected=entity.entity_id,
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

    def _publish_selected_marker(self, entity) -> None:
        arr = MarkerArray()
        # Big translucent highlight sphere directly on the selected entity.
        m = Marker()
        m.header.frame_id = self._global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "selected_target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = entity.pose_map
        m.scale.x = 0.6
        m.scale.y = 0.6
        m.scale.z = 0.6
        m.color.r = 1.0
        m.color.g = 0.9
        m.color.b = 0.1
        m.color.a = 0.55
        arr.markers.append(m)

        label = Marker()
        label.header = m.header
        label.ns = "selected_target_label"
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = entity.pose_map
        label.pose.position.z = entity.pose_map.position.z + 0.8
        label.scale.z = 0.25
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 0.95
        label.text = f"TARGET: {entity.class_label}"
        arr.markers.append(label)

        self._marker_pub.publish(arr)

    def _maybe_heartbeat(
        self,
        candidates: int,
        reason: str,
        selected: Optional[str],
    ) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds / 1e9 < self._log_period:
            return
        self._last_log_time = now
        self.get_logger().info(
            f"[target-selector] candidates={candidates} "
            f"selections_total={self._selections_total} "
            f"selected={selected[:8] if selected else 'None'} "
            f"{reason}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
