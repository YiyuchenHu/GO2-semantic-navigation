from dataclasses import dataclass
from typing import Dict

import rclpy
from geometry_msgs.msg import PoseStamped
from go2_msgs.msg import (
    Detection2DArray,
    InstanceMaskArray,
    ObjectObservationArray,
    SelectedTarget,
    SemanticEntityArray,
    SemanticTask,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


@dataclass
class Checkpoint:
    name: str
    passed: bool = False
    detail: str = ""


class IntegrationTraceWatcher(Node):
    """
    Compact end-to-end trace watcher for single scenario:
    "go to the chair".
    """

    def __init__(self) -> None:
        super().__init__("integration_trace_watcher")
        self.declare_parameter("expected_target", "chair")
        self.declare_parameter("timeout_sec", 120.0)
        self.declare_parameter("require_arrival", False)

        self._expected_target = str(self.get_parameter("expected_target").value).lower()
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._require_arrival = bool(self.get_parameter("require_arrival").value)
        self._start_ns = self.get_clock().now().nanoseconds

        self._cp: Dict[str, Checkpoint] = {
            "cmd": Checkpoint("command_parsed"),
            "det": Checkpoint("chair_detected"),
            "mask": Checkpoint("mask_stream_seen"),
            "loc": Checkpoint("chair_localized_3d"),
            "mem": Checkpoint("semantic_entity_stable"),
            "sel": Checkpoint("target_selected"),
            "goal": Checkpoint("goal_generated"),
            "nav": Checkpoint("navigation_status_active"),
            "arr": Checkpoint("arrival_verified"),
            "guide": Checkpoint("user_guidance_published"),
            "tf": Checkpoint("tf_seen"),
            "odom": Checkpoint("odom_seen"),
        }
        self._last_task_id = ""
        tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(String, "/user_command", self._on_user_command, 10)
        self.create_subscription(SemanticTask, "/semantic_task/request", self._on_task, 10)
        self.create_subscription(Detection2DArray, "/perception/detections_2d", self._on_det, 10)
        self.create_subscription(InstanceMaskArray, "/perception/masks", self._on_masks, 10)
        self.create_subscription(ObjectObservationArray, "/perception/objects_3d", self._on_loc, 10)
        self.create_subscription(SemanticEntityArray, "/semantic_map/entities", self._on_entities, 10)
        self.create_subscription(SelectedTarget, "/semantic_query/selected_target", self._on_selected, 10)
        self.create_subscription(PoseStamped, "/semantic_goal/goal_pose", self._on_goal, 10)
        self.create_subscription(String, "/navigation/status", self._on_nav, 10)
        self.create_subscription(String, "/arrival/status", self._on_arrival, 10)
        self.create_subscription(String, "/user_guidance/message", self._on_guidance, 10)
        self.create_subscription(String, "/task/status", self._on_task_status, 10)
        self.create_subscription(String, "/safety/status", self._on_safety, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, tf_qos)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf, tf_static_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info("Integration trace watcher started.")

    def _mark(self, key: str, detail: str) -> None:
        cp = self._cp[key]
        if cp.passed:
            return
        cp.passed = True
        cp.detail = detail
        self.get_logger().info(f"[PASS] {cp.name}: {detail}")

    def _on_user_command(self, msg: String) -> None:
        if self._expected_target in msg.data.lower():
            self.get_logger().info(f"[TRACE] /user_command='{msg.data}'")

    def _on_task(self, msg: SemanticTask) -> None:
        self._last_task_id = msg.task_id
        if msg.target_class.lower() == self._expected_target:
            self._mark("cmd", f"task_id={msg.task_id} target_class={msg.target_class}")

    def _on_det(self, msg: Detection2DArray) -> None:
        for d in msg.detections:
            if d.class_label.lower() == self._expected_target:
                self._mark("det", f"score={d.score:.2f} bbox=({d.xmin:.1f},{d.ymin:.1f},{d.xmax:.1f},{d.ymax:.1f})")
                break

    def _on_masks(self, msg: InstanceMaskArray) -> None:
        if msg.masks:
            self._mark("mask", f"num_masks={len(msg.masks)} backend={msg.backend_name}")

    def _on_loc(self, msg: ObjectObservationArray) -> None:
        for o in msg.observations:
            if o.class_label.lower() != self._expected_target:
                continue
            if abs(o.centroid_map.x) > 100.0 or abs(o.centroid_map.y) > 100.0:
                continue
            self._mark(
                "loc",
                f"map=({o.centroid_map.x:.2f},{o.centroid_map.y:.2f},{o.centroid_map.z:.2f}) valid_ratio={o.depth_valid_ratio:.2f}",
            )
            break

    def _on_entities(self, msg: SemanticEntityArray) -> None:
        for e in msg.entities:
            if e.class_label.lower() != self._expected_target:
                continue
            if e.observations_count >= 3 and e.confidence >= 0.30:
                self._mark(
                    "mem",
                    f"entity_id={e.entity_id[:8]} conf={e.confidence:.2f} obs={e.observations_count}",
                )
                break

    def _on_selected(self, msg: SelectedTarget) -> None:
        if msg.class_label.lower() != self._expected_target:
            return
        if self._last_task_id and msg.task_id != self._last_task_id:
            return
        if msg.entity_id:
            self._mark("sel", f"entity_id={msg.entity_id[:8]} score={msg.score:.2f} reachable={msg.reachable}")

    def _on_goal(self, msg: PoseStamped) -> None:
        self._mark("goal", f"goal_map=({msg.pose.position.x:.2f},{msg.pose.position.y:.2f}) frame={msg.header.frame_id}")

    def _on_nav(self, msg: String) -> None:
        if msg.data not in ("IDLE", "BACKEND_UNAVAILABLE"):
            self._mark("nav", f"status={msg.data}")
        self.get_logger().info(f"[TRACE] /navigation/status={msg.data}")

    def _on_arrival(self, msg: String) -> None:
        self.get_logger().info(f"[TRACE] /arrival/status={msg.data}")
        if msg.data.startswith("ARRIVED_CONFIRMED"):
            self._mark("arr", msg.data)

    def _on_guidance(self, msg: String) -> None:
        if msg.data.strip():
            self._mark("guide", msg.data.strip())
        self.get_logger().info(f"[TRACE] /user_guidance/message={msg.data}")

    def _on_task_status(self, msg: String) -> None:
        self.get_logger().info(f"[TRACE] /task/status={msg.data}")

    def _on_safety(self, msg: String) -> None:
        self.get_logger().info(f"[TRACE] /safety/status={msg.data}")

    def _on_tf(self, msg: TFMessage) -> None:
        if msg.transforms:
            self._mark("tf", f"{len(msg.transforms)} transforms")

    def _on_odom(self, msg: Odometry) -> None:
        self._mark("odom", f"pose=({msg.pose.pose.position.x:.2f},{msg.pose.pose.position.y:.2f})")

    def _heartbeat(self) -> None:
        elapsed = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
        needed = ["tf", "odom", "cmd", "det", "loc", "mem", "sel", "goal", "nav"]
        if self._require_arrival:
            needed.extend(["arr", "guide"])
        all_pass = all(self._cp[k].passed for k in needed)
        summary = " ".join([f"{k}={'Y' if self._cp[k].passed else 'N'}" for k in needed])
        self.get_logger().info(f"[HEARTBEAT {elapsed:.1f}s] {summary}")

        if all_pass:
            self.get_logger().info("[RESULT] INTEGRATION TRACE PASS")
            rclpy.shutdown()
            return

        if elapsed > self._timeout_sec:
            missing = [self._cp[k].name for k in needed if not self._cp[k].passed]
            self.get_logger().error(f"[RESULT] INTEGRATION TRACE TIMEOUT. Missing: {missing}")
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntegrationTraceWatcher()
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
