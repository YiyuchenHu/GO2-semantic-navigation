"""Day 7 — semantic target selector.

Subscribes
----------
/semantic_map/objects (go2_msgs/SemanticEntityArray)
    Persistent object registry from Day 6's
    semantic_memory_aggregator.

Publishes
---------
/target/selected (go2_msgs/SelectedTarget)
    The current best entity matching the configured ``target_class``
    parameter. Always published — when no entity qualifies, the
    message is emitted with empty ``entity_id`` and ``score=0`` so
    downstream consumers can distinguish "no target" from "topic
    silent". The downstream goal planner uses the empty form as a
    cancel signal.

Selection logic
---------------
On each housekeeping tick (``select_period_sec``, default 0.5 s)
AND on every input message:

  1. Filter ``/semantic_map/objects`` to entities whose
     ``class_label`` matches ``target_class`` (case-insensitive,
     space → underscore normalised so 'office_chair' and
     'office chair' both match).
  2. Drop entities below ``min_confidence``.
  3. Score the remaining entities. Default scoring rewards
     currently-visible entities first (they are the most reliable
     to actually drive to), then high confidence, then proximity
     to the robot's base_link.
  4. Publish the top-scored entity as a SelectedTarget.

The Day 6 aggregator does NOT compute reachability (no costmap
look-up); ``SelectedTarget.reachable`` is left True here as an
optimistic default. Day 7's approach_goal_planner_node performs
the actual costmap-aware feasibility check on the approach pose
and either succeeds (planner publishes a goal) or fails (planner
gives up after retries) — the failure surfaces in NavigateToPose
action result, not in this selector's output.

Why a separate selector node (instead of folding into the planner)
------------------------------------------------------------------
Selection and goal generation are conceptually different:

  * Selection: scoring entities → "which object?"
  * Planning:  approach-pose ring sampling + costmap filtering →
    "where to stop relative to that object?"

Two nodes lets us debug them independently:

  * `ros2 topic echo /target/selected` shows what the selector
    picks every 0.5 s, with score breakdown in `ranking_reasons`.
  * If the selector picks a chair the planner can't reach, the
    planner aborts but the selector keeps re-publishing the same
    target — operator can see selection succeeded but planning
    failed without log archeology.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from go2_msgs.msg import (
    SelectedTarget,
    SemanticEntity,
    SemanticEntityArray,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    LookupException,
    TransformException,
    TransformListener,
)


def _normalise_class(s: str) -> str:
    """Lower-case + space→underscore so 'office chair' == 'office_chair'."""
    return s.strip().lower().replace(" ", "_")


class TargetSelectorNode(Node):
    """Pick the best semantic entity matching the requested class."""

    def __init__(self) -> None:
        super().__init__("target_selector")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter("entities_topic", "/semantic_map/objects")
        self.declare_parameter("selected_topic", "/target/selected")
        # The class to look for. Day 7 MVP exposes a single class —
        # Day 10's command interface will rewrite this parameter via
        # `ros2 param set` on receipt of "go to chair" / "find the
        # table" / etc. Synonym matching happens upstream in YOLOE
        # via `set_classes()`, not here.
        self.declare_parameter("target_class", "chair")
        # Minimum confidence on a SemanticEntity for it to be
        # considered a candidate. Day 6's aggregator already filters
        # raw detections at min_detection_confidence; this is a
        # second-line gate on the *aggregated* entity to skip very-
        # decayed ghosts.
        self.declare_parameter("min_confidence", 0.30)
        # Frame to use as the robot's reference for distance scoring.
        # base_link is the canonical Go2 body frame; its pose in the
        # global frame is looked up via tf2.
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")
        # Re-evaluate selection at this rate even when no fresh
        # /semantic_map/objects message arrived (useful when the
        # operator changes target_class via ros2 param set and the
        # entity stream is steady-state).
        self.declare_parameter("select_period_sec", 0.5)
        # Per-tick log heartbeat. Set <=0 to disable.
        self.declare_parameter("log_period_sec", 5.0)
        # Scoring weights. Final score = w_visible * (visible ? 1 : 0) +
        # w_confidence * confidence + w_proximity * (1 / (1 + dist)).
        # Defaults bias toward currently-visible entities heavily so
        # the Go2 doesn't repeatedly try to drive to a memory-only
        # ghost when a fresh chair is right in front of it.
        self.declare_parameter("score_weight_visible", 1.0)
        self.declare_parameter("score_weight_confidence", 0.5)
        self.declare_parameter("score_weight_proximity", 0.3)

        entities_topic = str(self.get_parameter("entities_topic").value)
        selected_topic = str(self.get_parameter("selected_topic").value)
        # Stash class as a normalised string AND keep the raw form
        # for re-publishing. Param callback below keeps both in sync.
        self._target_class_raw = str(
            self.get_parameter("target_class").value
        )
        self._target_class_norm = _normalise_class(self._target_class_raw)
        self._min_confidence = float(
            self.get_parameter("min_confidence").value
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        select_period = float(self.get_parameter("select_period_sec").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)
        self._w_visible = float(
            self.get_parameter("score_weight_visible").value
        )
        self._w_confidence = float(
            self.get_parameter("score_weight_confidence").value
        )
        self._w_proximity = float(
            self.get_parameter("score_weight_proximity").value
        )

        # --------------------------------------------------------------
        # State + ROS infra
        # --------------------------------------------------------------
        self._latest: Optional[SemanticEntityArray] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # /semantic_map/objects is RELIABLE (Day 6 aggregator
        # default); match it.
        in_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            SemanticEntityArray, entities_topic, self._on_entities, in_qos
        )
        self._pub = self.create_publisher(
            SelectedTarget, selected_topic, 10
        )
        self.create_timer(select_period, self._select_and_publish)

        # Param-change callback so `ros2 param set /target_selector
        # target_class table` retunes the selector live (Day 10
        # command interface uses this hook).
        self.add_on_set_parameters_callback(self._on_param_change)

        # Heartbeat / metrics
        self._n_selections = 0
        self._n_published_with_target = 0
        self._n_published_empty = 0
        self._last_log_time = self.get_clock().now()

        self.get_logger().info(
            f"target_selector ready. "
            f"in={entities_topic} out={selected_topic} "
            f"target_class={self._target_class_raw!r} "
            f"min_confidence={self._min_confidence} "
            f"base_frame={self._base_frame} "
            f"global_frame={self._global_frame}"
        )

    # ------------------------------------------------------------------
    # Param hot-swap
    # ------------------------------------------------------------------
    def _on_param_change(self, params):
        """Apply runtime updates to target_class / min_confidence /
        scoring weights. Returns SetParametersResult required by rclpy.
        """
        from rcl_interfaces.msg import SetParametersResult

        for p in params:
            if p.name == "target_class":
                old = self._target_class_raw
                self._target_class_raw = str(p.value)
                self._target_class_norm = _normalise_class(
                    self._target_class_raw
                )
                self.get_logger().info(
                    f"target_class changed: {old!r} -> {self._target_class_raw!r}"
                )
            elif p.name == "min_confidence":
                self._min_confidence = float(p.value)
            elif p.name == "score_weight_visible":
                self._w_visible = float(p.value)
            elif p.name == "score_weight_confidence":
                self._w_confidence = float(p.value)
            elif p.name == "score_weight_proximity":
                self._w_proximity = float(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------
    def _on_entities(self, msg: SemanticEntityArray) -> None:
        self._latest = msg

    # ------------------------------------------------------------------
    # Selection — the hot path (timer + on-message)
    # ------------------------------------------------------------------
    def _select_and_publish(self) -> None:
        self._n_selections += 1
        out = SelectedTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._global_frame
        out.task_id = ""        # Day 10 will fill this on a fresh task
        out.class_label = self._target_class_raw
        # Defaults for the empty / no-target case.
        out.entity_id = ""
        out.score = 0.0
        out.reachable = False
        out.estimated_distance = 0.0
        out.ranking_reasons = []

        if self._latest is None:
            out.ranking_reasons = ["no /semantic_map/objects received yet"]
            self._publish_with_log(out)
            return

        # Filter by class + confidence.
        candidates: List[SemanticEntity] = []
        for e in self._latest.entities:
            if _normalise_class(e.class_label) != self._target_class_norm:
                continue
            if float(e.confidence) < self._min_confidence:
                continue
            candidates.append(e)

        if not candidates:
            out.ranking_reasons = [
                f"no entities with class={self._target_class_raw!r} "
                f"and confidence>={self._min_confidence}"
            ]
            self._publish_with_log(out)
            return

        # Score each candidate. We need the robot's pose to compute
        # proximity; if tf2 isn't ready yet we score with proximity=0
        # so visibility + confidence still order the candidates.
        robot_xy = self._lookup_robot_xy()

        scored: List[Tuple[float, SemanticEntity, dict]] = []
        for e in candidates:
            entity_xy = (e.pose_map.position.x, e.pose_map.position.y)
            dist = (
                math.hypot(robot_xy[0] - entity_xy[0],
                           robot_xy[1] - entity_xy[1])
                if robot_xy is not None else 0.0
            )
            visibility_score = 1.0 if e.currently_visible else 0.0
            proximity_score = 1.0 / (1.0 + max(0.0, dist))
            score = (
                self._w_visible * visibility_score
                + self._w_confidence * float(e.confidence)
                + self._w_proximity * proximity_score
            )
            breakdown = {
                "visible": visibility_score,
                "confidence": float(e.confidence),
                "proximity": proximity_score,
                "dist_m": dist,
            }
            scored.append((score, e, breakdown))

        # Pick the top-scored entity.
        scored.sort(key=lambda t: t[0], reverse=True)
        top_score, top_e, top_bd = scored[0]

        out.entity_id = top_e.entity_id
        out.target_pose_map = top_e.pose_map
        out.score = float(top_score)
        out.reachable = True   # optimistic; planner does the real check
        out.estimated_distance = float(top_bd["dist_m"])
        out.ranking_reasons = [
            f"chose {top_e.entity_id!r} from {len(candidates)} candidates",
            (
                f"score={top_score:.3f} = "
                f"{self._w_visible:.2f}*visible({top_bd['visible']:.0f}) "
                f"+ {self._w_confidence:.2f}*conf({top_bd['confidence']:.2f}) "
                f"+ {self._w_proximity:.2f}*prox({top_bd['proximity']:.3f})"
            ),
            f"dist_to_base_link={top_bd['dist_m']:.2f}m",
        ]
        self._publish_with_log(out)

    def _publish_with_log(self, out: SelectedTarget) -> None:
        if out.entity_id:
            self._n_published_with_target += 1
        else:
            self._n_published_empty += 1
        self._pub.publish(out)
        self._tick_log()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _lookup_robot_xy(self) -> Optional[Tuple[float, float]]:
        """Return base_link's (x, y) in global_frame via tf2, or None.

        Uses ``Time()`` (latest available) because target selection
        doesn't need stamp-aligned proximity — being off by a frame
        on a slow-moving Go2 changes proximity by < 1 cm.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, TransformException):
            return None
        return (
            float(t.transform.translation.x),
            float(t.transform.translation.y),
        )

    def _tick_log(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_log_time).nanoseconds
        if elapsed_ns < int(self._log_period * 1e9):
            return
        elapsed_s = elapsed_ns / 1e9
        sel_hz = self._n_selections / elapsed_s if elapsed_s > 0 else 0.0
        self.get_logger().info(
            f"target_selector @ {sel_hz:.1f} Hz; "
            f"with_target={self._n_published_with_target} "
            f"empty={self._n_published_empty} "
            f"target_class={self._target_class_raw!r}"
        )
        self._n_selections = 0
        self._n_published_with_target = 0
        self._n_published_empty = 0
        self._last_log_time = now


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
