"""Day 7 — approach goal planner with NavigateToPose action client.

Subscribes
----------
/target/selected (go2_msgs/SelectedTarget)
    Output of target_selector_node. Carries the chosen entity's
    class and map-frame pose, or an empty-id "no target" signal.

/global_costmap/costmap (nav_msgs/OccupancyGrid)
    Nav2's costmap from Day 4. Used to filter approach-pose
    candidates that fall in obstacle / inflation cells.

Publishes
---------
/semantic_goal/goal_pose (geometry_msgs/PoseStamped)
    The currently committed approach pose (debug/visual). Mirrors
    the Pose sent through the action; lets RViz show "where the
    Go2 is heading" without subscribing to action topics.

/semantic_goal/goal_candidates (visualization_msgs/MarkerArray)
    All ring-sample candidates, colour-coded by feasibility:
      * green = costmap-clear and not too close to a wall
      * red   = blocked by costmap (lethal cell or inflation)
    Excellent for tuning ``approach_distance`` and
    ``cost_threshold`` against a specific scene.

Action client
-------------
/navigate_to_pose (nav2_msgs/action/NavigateToPose, Day 4)
    Goal sent every time a NEW target is selected (different
    entity_id) OR the existing committed approach pose has gone
    stale (target moved more than ``replan_distance_m`` since
    the last goal was sent). The previous in-flight goal is
    preempted (cancel + new goal); Nav2's bt_navigator handles
    the transition.

Algorithm
---------
For each target update (with throttling at ``replan_period_sec``):

  1. Sample N=``num_angle_samples`` poses on a ring of radius
     ``approach_distance[class]`` around the target's pose, each
     facing the target.
  2. Look up the global_costmap cost at each sample's (x, y).
     Drop any sample whose cost > ``cost_threshold`` (Nav2
     convention: 0=free, 99=inflation peak, 100=lethal,
     >100=unknown). 60 by default = "no inflated obstacles".
  3. Score remaining samples by (a) closeness to the robot's
     current pose and (b) angular alignment with the robot's
     current heading toward the target. Closest + most-aligned
     wins; the heuristic prefers approaches that don't require
     a 180° spin in place.
  4. If at least one sample survives, send NavigateToPose action
     with the winning sample as the goal. Republish on
     /semantic_goal/goal_pose for RViz.
  5. If no sample survives, log + republish empty (no goal sent).
     Operator should either move the robot, lower
     ``cost_threshold``, or shrink ``approach_distance``.

Why this is a separate node from target_selector
------------------------------------------------
Selection and goal generation have different update rates,
different failure modes, and benefit from independent debugging.
target_selector ticks at 2 Hz on entity-stream; planner ticks
when a fresh /target/selected arrives OR every
``replan_period_sec``, runs costmap math, and talks to Nav2's
action server. Folding them into one node would make goal-
generation failures (no costmap, no costmap-clear pose) bleed
into selection-state diagnostics.

Why NavigateToPose action vs /goal_pose topic
---------------------------------------------
The action gives us:
  * Feedback (current_pose, distance_remaining,
    number_of_recoveries) we can stream into a status topic for
    Day 8+.
  * Cancel for re-planning when the target changes mid-traverse.
  * Result codes (NavigateToPose error codes) to distinguish
    "Nav2 reached the goal" from "Nav2 gave up on the recovery
    behaviour tree".

The /goal_pose topic path also works (Day 4 verified) but is
fire-and-forget; downstream state machines need either feedback
or explicit poll loops, which Day 8 will want.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from go2_msgs.msg import SelectedTarget
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from tf2_ros import (
    Buffer,
    LookupException,
    TransformException,
    TransformListener,
)
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_to_quat(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(
        x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)
    )


class ApproachGoalPlannerNode(Node):
    """Ring-sample approach poses around a target, send to Nav2."""

    def __init__(self) -> None:
        super().__init__("approach_goal_planner")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter("selected_topic", "/target/selected")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("goal_pose_topic", "/semantic_goal/goal_pose")
        self.declare_parameter(
            "candidates_topic", "/semantic_goal/goal_candidates"
        )
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")
        # Number of equally-spaced angles around the target's centre
        # to sample. 16 → every 22.5°. Lower for speed, higher for
        # narrow corridors where most directions are blocked.
        self.declare_parameter("num_angle_samples", 16)
        # Per-class approach stand-off distance. The robot stops
        # this far from the target centroid. Falls back to
        # ``approach_distance_default`` for any class not listed.
        # Day 7 ships sensible defaults for the Day 6 prompt set;
        # Day 10 may override per-task.
        self.declare_parameter("approach_distance_default", 0.9)
        self.declare_parameter("approach_distance_chair", 0.9)
        self.declare_parameter("approach_distance_table", 1.0)
        self.declare_parameter("approach_distance_desk", 1.0)
        self.declare_parameter("approach_distance_box", 0.7)
        self.declare_parameter("approach_distance_person", 1.2)
        # Costmap cell value above which a sample is rejected.
        # Nav2 convention: 0 free, 99 inflation peak, 100 lethal,
        # >100 unknown. 60 = "stay clear of obstacles + their
        # inflation halo".
        self.declare_parameter("cost_threshold", 60)
        # Re-evaluate / re-send a goal at most this often. Lower
        # values make Nav2 chase a moving target faster but also
        # spam the action server with cancel-and-resend cycles.
        self.declare_parameter("replan_period_sec", 1.0)
        # If the target's pose has moved less than this since the
        # last goal send, do NOT re-send. Avoids action churn on
        # tiny EMA jitter from semantic_memory.
        self.declare_parameter("replan_distance_m", 0.10)
        # Heartbeat log period.
        self.declare_parameter("log_period_sec", 5.0)
        # Scoring weights for choosing among the cost-clear
        # candidates. Final score = -dist_to_robot - w_align *
        # heading_misalignment_radians.
        self.declare_parameter("score_alignment_weight", 0.5)

        self._sel_topic = str(self.get_parameter("selected_topic").value)
        self._costmap_topic = str(self.get_parameter("costmap_topic").value)
        self._goal_topic = str(self.get_parameter("goal_pose_topic").value)
        self._cand_topic = str(self.get_parameter("candidates_topic").value)
        self._nav_action_name = str(
            self.get_parameter("nav_action_name").value
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._num_samples = int(
            self.get_parameter("num_angle_samples").value
        )
        # Pre-compute the per-class approach distance dict so
        # planning doesn't allocate per-tick.
        self._approach: Dict[str, float] = {
            "chair": float(
                self.get_parameter("approach_distance_chair").value
            ),
            "table": float(
                self.get_parameter("approach_distance_table").value
            ),
            "desk": float(
                self.get_parameter("approach_distance_desk").value
            ),
            "box": float(
                self.get_parameter("approach_distance_box").value
            ),
            "person": float(
                self.get_parameter("approach_distance_person").value
            ),
        }
        self._approach_default = float(
            self.get_parameter("approach_distance_default").value
        )
        self._cost_threshold = int(
            self.get_parameter("cost_threshold").value
        )
        replan_period = float(
            self.get_parameter("replan_period_sec").value
        )
        self._replan_dist_m = float(
            self.get_parameter("replan_distance_m").value
        )
        self._log_period = float(self.get_parameter("log_period_sec").value)
        self._w_align = float(
            self.get_parameter("score_alignment_weight").value
        )

        # --------------------------------------------------------------
        # State
        # --------------------------------------------------------------
        self._latest_selected: Optional[SelectedTarget] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # In-flight action handle. None when no goal is active.
        self._current_goal_handle = None
        # Last committed (entity_id, target_xy) so we can decide
        # whether to re-send.
        self._last_sent_entity_id: Optional[str] = None
        self._last_sent_target_xy: Optional[Tuple[float, float]] = None

        # Heartbeat counters
        self._n_replans = 0
        self._n_goals_sent = 0
        self._n_no_feasible = 0
        self._n_action_succeeded = 0
        self._n_action_aborted = 0
        self._n_action_canceled = 0
        self._last_log_time = self.get_clock().now()

        # --------------------------------------------------------------
        # ROS infra
        # --------------------------------------------------------------
        in_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # /global_costmap/costmap is TRANSIENT_LOCAL latched by
        # Nav2 — match it so we get the latest version on subscribe.
        cm_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            SelectedTarget, self._sel_topic, self._on_selected, in_qos
        )
        self.create_subscription(
            OccupancyGrid, self._costmap_topic, self._on_costmap, cm_qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, self._goal_topic, 10
        )
        self._cand_pub = self.create_publisher(
            MarkerArray, self._cand_topic, 10
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, self._nav_action_name
        )

        self.create_timer(replan_period, self._on_replan_tick)

        self.get_logger().info(
            f"approach_goal_planner ready. "
            f"in={self._sel_topic} costmap={self._costmap_topic} "
            f"action={self._nav_action_name} "
            f"num_samples={self._num_samples} "
            f"cost_thresh={self._cost_threshold} "
            f"replan_period={replan_period:.2f}s "
            f"replan_dist={self._replan_dist_m:.2f}m"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_selected(self, msg: SelectedTarget) -> None:
        self._latest_selected = msg

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    # ------------------------------------------------------------------
    # Replan tick — the hot path
    # ------------------------------------------------------------------
    def _on_replan_tick(self) -> None:
        self._n_replans += 1
        sel = self._latest_selected

        # -- No selected target: cancel any in-flight goal.
        if sel is None or not sel.entity_id:
            if self._current_goal_handle is not None:
                self.get_logger().info(
                    "selected target cleared; canceling in-flight goal"
                )
                self._cancel_current_goal()
                self._last_sent_entity_id = None
                self._last_sent_target_xy = None
            self._publish_empty_candidates()
            self._tick_log()
            return

        # -- Need costmap before we can filter samples.
        if self._costmap is None:
            self.get_logger().warn(
                "no /global_costmap/costmap yet; skipping plan",
                throttle_duration_sec=2.0,
            )
            self._tick_log()
            return

        # -- Need robot pose before we can score by proximity.
        robot = self._lookup_robot_pose()
        if robot is None:
            self.get_logger().warn(
                "no base_link -> map TF yet; skipping plan",
                throttle_duration_sec=2.0,
            )
            self._tick_log()
            return
        rx, ry, ryaw = robot

        # -- Throttle: skip if same target & target hasn't moved much.
        target_xy = (
            float(sel.target_pose_map.position.x),
            float(sel.target_pose_map.position.y),
        )
        if (
            self._last_sent_entity_id == sel.entity_id
            and self._last_sent_target_xy is not None
            and self._dist(self._last_sent_target_xy, target_xy)
                < self._replan_dist_m
        ):
            # Republish current goal pose for RViz freshness, but
            # don't re-send the action — Nav2 is still working on
            # the previous goal.
            self._tick_log()
            return

        # -- Generate ring of candidates + filter by costmap.
        approach_d = self._approach.get(
            sel.class_label.lower(), self._approach_default
        )
        candidates = self._sample_ring(target_xy, approach_d)
        scored, rejected = self._filter_by_costmap(candidates)
        # Publish all candidates as markers (green=ok, red=rejected)
        # so the operator can eyeball *why* a target failed.
        self._publish_candidates(target_xy, scored, rejected)

        if not scored:
            self._n_no_feasible += 1
            self.get_logger().warn(
                f"no costmap-clear approach pose around "
                f"({target_xy[0]:.2f},{target_xy[1]:.2f}) at d={approach_d:.2f}m. "
                f"({len(rejected)} candidates rejected — see "
                f"/semantic_goal/goal_candidates in RViz)",
                throttle_duration_sec=2.0,
            )
            self._tick_log()
            return

        # -- Score each viable candidate. Pick the best.
        best = self._pick_best(scored, target_xy, rx, ry, ryaw)
        self._send_goal(best, sel)
        self._last_sent_entity_id = sel.entity_id
        self._last_sent_target_xy = target_xy
        self._tick_log()

    # ------------------------------------------------------------------
    # Sampling / filtering / scoring
    # ------------------------------------------------------------------
    def _sample_ring(
        self, target_xy: Tuple[float, float], radius: float
    ) -> List[Tuple[float, float, float]]:
        """N equally-spaced (x, y, yaw) poses on a circle around target.

        Yaw points back at the target so an approach pose places the
        Go2 facing the target — the natural way to "stop in front of
        the chair".
        """
        out: List[Tuple[float, float, float]] = []
        tx, ty = target_xy
        for i in range(self._num_samples):
            theta = 2.0 * math.pi * i / self._num_samples
            sx = tx + radius * math.cos(theta)
            sy = ty + radius * math.sin(theta)
            # Yaw = direction from sample TO target.
            yaw = math.atan2(ty - sy, tx - sx)
            out.append((sx, sy, yaw))
        return out

    def _filter_by_costmap(
        self, candidates: List[Tuple[float, float, float]]
    ) -> Tuple[
        List[Tuple[float, float, float]],
        List[Tuple[float, float, float]],
    ]:
        """Split candidates into (cost-clear, rejected) by costmap value."""
        cm = self._costmap
        if cm is None:
            return candidates, []
        info = cm.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution
        w = info.width
        h = info.height
        if w == 0 or h == 0 or res <= 0.0:
            return candidates, []
        data = cm.data  # row-major, len == w*h
        scored: List[Tuple[float, float, float]] = []
        rejected: List[Tuple[float, float, float]] = []
        for sx, sy, yaw in candidates:
            cx = int(math.floor((sx - ox) / res))
            cy = int(math.floor((sy - oy) / res))
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                # Outside the costmap — treat as rejected; we don't
                # know the cost. (Day 6+ map could still be growing.)
                rejected.append((sx, sy, yaw))
                continue
            idx = cy * w + cx
            cost = int(data[idx])
            # Nav2 uses signed int8; -1 = unknown, 0..99 inflation
            # gradient, 100 lethal. Treat unknown as rejected to
            # be conservative. cost_threshold typically excludes
            # everything > 60.
            if cost < 0 or cost > self._cost_threshold:
                rejected.append((sx, sy, yaw))
                continue
            scored.append((sx, sy, yaw))
        return scored, rejected

    def _pick_best(
        self,
        viable: List[Tuple[float, float, float]],
        target_xy: Tuple[float, float],
        rx: float, ry: float, ryaw: float,
    ) -> Tuple[float, float, float]:
        """Closest-with-good-heading wins.

        We prefer candidates that:
          (a) are close to the robot's current position (less travel)
          (b) lie roughly along the direction the robot already faces
              (less rotation upfront)

        Score = -distance(robot, candidate) - w_align *
                |heading_to(candidate) - ryaw|

        Higher score = better (closer + more aligned).
        """
        best = None
        best_score = -float("inf")
        for sx, sy, yaw in viable:
            dist = math.hypot(rx - sx, ry - sy)
            heading_to_cand = math.atan2(sy - ry, sx - rx)
            misalign = abs(self._wrap_pi(heading_to_cand - ryaw))
            score = -dist - self._w_align * misalign
            if score > best_score:
                best_score = score
                best = (sx, sy, yaw)
        assert best is not None  # viable list was non-empty
        return best

    @staticmethod
    def _wrap_pi(a: float) -> float:
        """Wrap ``a`` to the [-pi, pi] interval."""
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # ------------------------------------------------------------------
    # Action client helpers
    # ------------------------------------------------------------------
    def _send_goal(
        self, pose_xy_yaw: Tuple[float, float, float], sel: SelectedTarget
    ) -> None:
        """Send a NavigateToPose goal, canceling any predecessor."""
        sx, sy, yaw = pose_xy_yaw
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self._global_frame
        ps.pose.position.x = float(sx)
        ps.pose.position.y = float(sy)
        ps.pose.position.z = 0.0
        ps.pose.orientation = _yaw_to_quat(yaw)

        # Republish on the debug topic so RViz shows where we're
        # going regardless of whether the action is accepted.
        self._goal_pub.publish(ps)

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"action server {self._nav_action_name!r} not "
                f"available; not sending goal",
                throttle_duration_sec=2.0,
            )
            return

        # Cancel the previous goal first, otherwise Nav2 sees a
        # second NavigateToPose and queues it. The bt_navigator
        # technically handles preemption itself but explicit
        # cancellation makes the node's intent visible in logs
        # and reduces edge cases on action_msgs version mismatch.
        self._cancel_current_goal()

        goal = NavigateToPose.Goal()
        goal.pose = ps
        goal.behavior_tree = ""

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)
        self._n_goals_sent += 1
        self.get_logger().info(
            f"NavigateToPose goal sent: target={sel.entity_id!r} "
            f"approach=({sx:.2f},{sy:.2f}) yaw={math.degrees(yaw):.1f}°"
        )

    def _cancel_current_goal(self) -> None:
        if self._current_goal_handle is None:
            return
        try:
            self._current_goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(
                f"cancel failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=5.0,
            )
        self._current_goal_handle = None

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"goal_response future raised: {exc}"
            )
            return
        if not handle.accepted:
            self.get_logger().warn(
                "Nav2 rejected the NavigateToPose goal — check that "
                "lifecycle_manager_navigation reported 'Managed nodes "
                "are active'."
            )
            return
        self._current_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        # We don't republish feedback yet (Day 8 may add a
        # /navigation/status topic). Logging at info-throttle so
        # the operator sees progress without drowning the console.
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Nav2 feedback: dist_remaining={fb.distance_remaining:.2f}m "
            f"recoveries={fb.number_of_recoveries} "
            f"navtime={fb.navigation_time.sec}.{fb.navigation_time.nanosec // 1000000:03d}s",
            throttle_duration_sec=2.0,
        )

    def _on_result(self, future) -> None:
        try:
            result_wrapper = future.result()
        except Exception as exc:
            self.get_logger().warn(f"result future raised: {exc}")
            return
        status = result_wrapper.status
        result = result_wrapper.result
        # The handle is "done" — drop the cached reference so the
        # next replan tick can fire a fresh goal.
        self._current_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._n_action_succeeded += 1
            self.get_logger().info(
                f"NavigateToPose SUCCEEDED (error_code={result.error_code})"
            )
        elif status == GoalStatus.STATUS_ABORTED:
            self._n_action_aborted += 1
            self.get_logger().warn(
                f"NavigateToPose ABORTED (error_code={result.error_code} "
                f"msg={result.error_msg!r})"
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self._n_action_canceled += 1
            self.get_logger().info("NavigateToPose CANCELED")
        else:
            self.get_logger().info(
                f"NavigateToPose finished with status={status}"
            )

    # ------------------------------------------------------------------
    # TF lookup
    # ------------------------------------------------------------------
    def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return (x, y, yaw) of base_link in global_frame, or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, TransformException):
            return None
        x = float(t.transform.translation.x)
        y = float(t.transform.translation.y)
        # Extract yaw from quaternion. Robot's roll/pitch are zero
        # for our planar Go2 (sim is kinematic), so the standard
        # quaternion → yaw formula is exact.
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (x, y, yaw)

    # ------------------------------------------------------------------
    # Marker publishing for RViz
    # ------------------------------------------------------------------
    def _publish_candidates(
        self,
        target_xy: Tuple[float, float],
        scored: List[Tuple[float, float, float]],
        rejected: List[Tuple[float, float, float]],
    ) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = self._global_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        ma.markers.append(clear)

        # Target centre.
        m = Marker()
        m.header = clear.header
        m.ns = "target"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(target_xy[0])
        m.pose.position.y = float(target_xy[1])
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)  # yellow
        ma.markers.append(m)

        for i, (sx, sy, _yaw) in enumerate(scored):
            ma.markers.append(self._candidate_marker(
                "viable", i + 1, sx, sy, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7),
            ))
        for i, (sx, sy, _yaw) in enumerate(rejected):
            ma.markers.append(self._candidate_marker(
                "rejected", 1000 + i, sx, sy,
                ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.5),
            ))
        self._cand_pub.publish(ma)

    def _publish_empty_candidates(self) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = self._global_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        ma.markers.append(clear)
        self._cand_pub.publish(ma)

    def _candidate_marker(
        self,
        ns: str,
        idx: int,
        x: float,
        y: float,
        color: ColorRGBA,
    ) -> Marker:
        m = Marker()
        m.header.frame_id = self._global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = int(idx)
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.02
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.15
        m.scale.z = 0.04
        m.color = color
        return m

    # ------------------------------------------------------------------
    # Heartbeat
    # ------------------------------------------------------------------
    def _tick_log(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_log_time).nanoseconds
        if elapsed_ns < int(self._log_period * 1e9):
            return
        elapsed_s = elapsed_ns / 1e9
        replan_hz = self._n_replans / elapsed_s if elapsed_s > 0 else 0.0
        self.get_logger().info(
            f"approach_planner @ {replan_hz:.1f} Hz replan; "
            f"sent={self._n_goals_sent} no_feasible={self._n_no_feasible} "
            f"succeeded={self._n_action_succeeded} "
            f"aborted={self._n_action_aborted} "
            f"canceled={self._n_action_canceled}"
        )
        self._n_replans = 0
        self._n_goals_sent = 0
        self._n_no_feasible = 0
        self._n_action_succeeded = 0
        self._n_action_aborted = 0
        self._n_action_canceled = 0
        self._last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApproachGoalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
