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
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
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
from std_msgs.msg import ColorRGBA, String
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
        # Day 8 wiring fix: in day8_two_phase.launch.py we don't run
        # the heavyweight nav_executor / arrival_verifier nodes, so
        # task_coordinator's NAVIGATE_TO_GOAL → ARRIVED loop has no
        # status feed. Closing that loop here means: when our Nav2
        # action terminates, mirror the result onto /navigation/status
        # and /arrival/status with the strings task_coordinator
        # already understands.
        self.declare_parameter("navigation_status_topic", "/navigation/status")
        self.declare_parameter("arrival_status_topic", "/arrival/status")
        # Day 9 (action-debug): mirror the NavigateToPose lifecycle on
        # a dedicated string topic so the operator can tell whether
        # the planner actually sent the action, whether Nav2 accepted
        # it, and how it terminated — without grep-ing the planner
        # logs. The exact string vocabulary is committed to (see
        # ``_publish_action_debug``):
        #   "NavigateToPose SEND ..."        -- goal_async called
        #   "NavigateToPose ACCEPTED ..."    -- action server accepted
        #   "NavigateToPose REJECTED ..."    -- action server rejected
        #   "NavigateToPose RESULT: SUCCEEDED|ABORTED|CANCELED ..."
        self.declare_parameter(
            "action_debug_topic", "/semantic_goal/action_debug"
        )
        # Day 9 hot-fix: explicit kill switch for the action send.
        # When False, the planner still computes ring samples,
        # republishes /semantic_goal/goal_pose, and emits the
        # /semantic_goal/action_debug "NOSEND reason=auto_send_goal_disabled"
        # message — it just does NOT call NavigateToPose.send_goal_async.
        # Useful for "RViz dry-run" testing without driving the robot.
        # Default TRUE so the MVP demo works out of the box.
        self.declare_parameter("auto_send_goal", True)
        # Day 9 hot-fix: when target_selector commits the SAME entity
        # for many ticks (e.g. a stale_but_confirmed table), we used
        # to silently re-publish goal_pose only — leaving
        # /semantic_goal/action_debug bone-dry and the operator
        # blind. Set this to True (default) so every replan tick
        # drops a "NOSEND reason=throttled_target_unchanged" line on
        # the debug topic; the operator's `ros2 topic echo --once`
        # always returns immediately. Throttle the message rate via
        # ``action_debug_throttle_period_sec`` if it ever feels
        # noisy.
        self.declare_parameter("action_debug_emit_throttled", True)
        self.declare_parameter(
            "action_debug_throttle_period_sec", 1.0
        )
        # Day 9 hot-fix Task 3 — periodic IN_FLIGHT emission while a
        # NavigateToPose goal is accepted. Lets the operator
        # distinguish "Nav2 still grinding through the BT" from
        # "result callback was lost" — the throttled NOSEND already
        # includes the cached state/last_result fields, so a real
        # IN_FLIGHT line every ~2 s is the missing positive signal.
        self.declare_parameter("inflight_debug_period_sec", 2.0)
        # Day 9 hot-fix Task 2 — distance-based arrival verifier.
        # Bridges the case where Nav2's action result callback is
        # silently dropped (DDS hiccup, behaviour-tree quirk) but
        # the robot has *physically* reached the semantic goal.
        # When robot↔goal_pose distance drops below threshold AND
        # /cmd_vel stays quiet for ``arrival_hold_sec`` seconds, we
        # publish SUCCEEDED + ARRIVED_CONFIRMED:<class> ourselves,
        # so task_coordinator's FSM can advance.
        # Defaults are relaxed because Go2's locomotion policy
        # tends to add ~10–20 cm overshoot when stopping.
        self.declare_parameter(
            "arrival_distance_threshold_m", 0.5
        )
        self.declare_parameter("arrival_hold_sec", 1.0)
        self.declare_parameter(
            "require_low_cmd_vel_for_arrival", True
        )
        self.declare_parameter("cmd_vel_linear_threshold", 0.05)
        self.declare_parameter("cmd_vel_angular_threshold", 0.1)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # Master switch: lets the operator force-disable the
        # distance-based arrival verifier (e.g. on a real robot
        # where the Nav2 result is the only trustworthy signal).
        self.declare_parameter("enable_distance_arrival_verifier", True)

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
        self._nav_status_topic = str(
            self.get_parameter("navigation_status_topic").value
        )
        self._arrival_status_topic = str(
            self.get_parameter("arrival_status_topic").value
        )
        self._action_debug_topic = str(
            self.get_parameter("action_debug_topic").value
        )
        self._auto_send_goal = bool(
            self.get_parameter("auto_send_goal").value
        )
        self._emit_throttled_debug = bool(
            self.get_parameter("action_debug_emit_throttled").value
        )
        self._action_debug_throttle = float(
            self.get_parameter("action_debug_throttle_period_sec").value
        )
        self._last_throttled_debug_time = self.get_clock().now()
        self._inflight_period_sec = float(
            self.get_parameter("inflight_debug_period_sec").value
        )
        self._arrival_dist_thresh = float(
            self.get_parameter("arrival_distance_threshold_m").value
        )
        self._arrival_hold_sec = float(
            self.get_parameter("arrival_hold_sec").value
        )
        self._require_low_cmd_vel = bool(
            self.get_parameter("require_low_cmd_vel_for_arrival").value
        )
        self._cmd_vel_lin_thresh = float(
            self.get_parameter("cmd_vel_linear_threshold").value
        )
        self._cmd_vel_ang_thresh = float(
            self.get_parameter("cmd_vel_angular_threshold").value
        )
        self._cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )
        self._distance_arrival_enabled = bool(
            self.get_parameter("enable_distance_arrival_verifier").value
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
        # Class label associated with the in-flight goal. Captured at
        # send-time (not at result-time) so a SUCCEEDED result still
        # has the correct label even if /target/selected has already
        # rolled over to a different class while Nav2 was running.
        self._current_class_label: str = ""

        # Heartbeat counters
        self._n_replans = 0
        self._n_goals_sent = 0
        self._n_no_feasible = 0
        self._n_action_succeeded = 0
        self._n_action_aborted = 0
        self._n_action_canceled = 0
        self._last_log_time = self.get_clock().now()

        # Day 9 hot-fix Task 3 — last action-result snapshot so the
        # throttled NOSEND on the next replan tick can carry it.
        # Possible values: "", "ACCEPTED", "SUCCEEDED", "ABORTED",
        # "CANCELED", "REJECTED", "SEND_FAILED".
        self._last_goal_status: str = ""
        self._last_result_msg: str = ""
        # Latched start time of the current accepted goal so the
        # IN_FLIGHT line can show "elapsed=...".
        self._goal_accepted_time: Optional[Time] = None
        self._last_inflight_emit_time: Optional[Time] = None

        # Day 9 hot-fix Task 2 — distance-based arrival verifier
        # state.
        self._latest_cmd_vel: Optional[Twist] = None
        self._latest_cmd_vel_time: Optional[Time] = None
        self._arrival_close_since: Optional[Time] = None
        self._arrival_published_for_entity: Optional[str] = None
        # Cache of the latest /semantic_goal/goal_pose (we publish it
        # ourselves but read it back for arrival distance maths).
        self._latest_goal_pose: Optional[PoseStamped] = None

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
        # Day 8 wiring fix: bridge the Nav2 action result back onto the
        # status topics task_coordinator's FSM listens to. Without these
        # two publishers the FSM stalls in NAVIGATE_TO_GOAL forever in
        # day8_two_phase (which doesn't launch nav_executor /
        # arrival_verifier).
        self._navigation_status_pub = self.create_publisher(
            String, self._nav_status_topic, 10
        )
        self._arrival_status_pub = self.create_publisher(
            String, self._arrival_status_topic, 10
        )
        # Day 9: NavigateToPose lifecycle on a single string topic for
        # quick `ros2 topic echo /semantic_goal/action_debug` triage.
        self._action_debug_pub = self.create_publisher(
            String, self._action_debug_topic, 10
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, self._nav_action_name
        )

        # Day 9 hot-fix Task 2 — /cmd_vel subscription so the
        # distance-based arrival verifier can require "robot stopped".
        # BestEffort QoS matches Nav2's velocity_smoother default.
        cmd_vel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Twist, self._cmd_vel_topic, self._on_cmd_vel, cmd_vel_qos,
        )

        self.create_timer(replan_period, self._on_replan_tick)
        # Arrival verifier ticks at 5 Hz so the "stopped for 1 s"
        # check has reasonable resolution without flooding logs.
        self.create_timer(0.2, self._on_arrival_tick)

        self.get_logger().info(
            f"approach_goal_planner ready. "
            f"in={self._sel_topic} costmap={self._costmap_topic} "
            f"action={self._nav_action_name} "
            f"num_samples={self._num_samples} "
            f"cost_thresh={self._cost_threshold} "
            f"replan_period={replan_period:.2f}s "
            f"replan_dist={self._replan_dist_m:.2f}m "
            f"goal_pose_topic={self._goal_topic} "
            f"action_debug_topic={self._action_debug_topic} "
            f"auto_send_goal={self._auto_send_goal} "
            f"action_debug_emit_throttled={self._emit_throttled_debug} "
            f"action_debug_throttle="
            f"{self._action_debug_throttle:.2f}s "
            f"inflight_period={self._inflight_period_sec:.2f}s "
            f"distance_arrival="
            f"{'on' if self._distance_arrival_enabled else 'off'} "
            f"arrival_dist={self._arrival_dist_thresh:.2f}m "
            f"arrival_hold={self._arrival_hold_sec:.2f}s "
            f"require_low_cmd_vel={self._require_low_cmd_vel} "
            f"cmd_vel_topic={self._cmd_vel_topic}"
        )
        if not self._auto_send_goal:
            self.get_logger().warn(
                "auto_send_goal=False — NavigateToPose action will "
                "NOT be sent. /semantic_goal/goal_pose still updates "
                "for RViz, /semantic_goal/action_debug emits "
                "'NOSEND reason=auto_send_goal_disabled' every tick."
            )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_selected(self, msg: SelectedTarget) -> None:
        # Day 9 hot-fix Task 2 — when target_selector commits a NEW
        # entity, drop the arrival latch so the verifier can fire
        # again on the fresh goal.
        prev = self._latest_selected
        self._latest_selected = msg
        new_eid = msg.entity_id if msg is not None else ""
        prev_eid = prev.entity_id if prev is not None else ""
        if new_eid != prev_eid:
            self._arrival_close_since = None
            self._arrival_published_for_entity = None

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Day 9 hot-fix Task 2 — feed the arrival verifier so it can
        # require the robot to actually be stopped before declaring
        # arrival. We snapshot the latest twist + arrival timestamp.
        self._latest_cmd_vel = msg
        self._latest_cmd_vel_time = self.get_clock().now()

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
            # Day 9 hot-fix: keep /semantic_goal/action_debug warm
            # even when we have nothing to send. This is the
            # single most useful state for debugging "the user
            # said go-to-table but nothing happens" — the silent
            # variant used to look identical to "approach_planner
            # crashed".
            self._maybe_publish_throttled_debug(
                "NavigateToPose NOSEND reason=no_target_selected"
            )
            self._tick_log()
            return

        # -- Need costmap before we can filter samples.
        if self._costmap is None:
            self.get_logger().warn(
                "no /global_costmap/costmap yet; skipping plan",
                throttle_duration_sec=2.0,
            )
            self._maybe_publish_throttled_debug(
                f"NavigateToPose NOSEND reason=no_costmap "
                f"target={sel.entity_id!r} class={sel.class_label!r} "
                f"costmap_topic={self._costmap_topic}"
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
            self._maybe_publish_throttled_debug(
                f"NavigateToPose NOSEND reason=no_tf_to_base_link "
                f"target={sel.entity_id!r} class={sel.class_label!r} "
                f"global_frame={self._global_frame!r} "
                f"base_frame={self._base_frame!r}"
            )
            self._tick_log()
            return
        rx, ry, ryaw = robot

        target_xy = (
            float(sel.target_pose_map.position.x),
            float(sel.target_pose_map.position.y),
        )

        # -- Generate ring of candidates + filter by costmap.
        # ALWAYS run sampling + costmap filtering + marker publishing
        # every tick (even when we're throttling the action send).
        # That way `/semantic_goal/goal_candidates` and `/goal_pose`
        # never go silent — RViz keeps animating, `ros2 topic echo`
        # always sees fresh data, and the operator can see in real
        # time when a previously-rejected ring direction opens up
        # (e.g. costmap inflation just got re-published, or target
        # drifted enough to expose new clear cells). The throttle
        # below only short-circuits the *action send*, never the
        # diagnostic publishing.
        approach_d = self._approach.get(
            sel.class_label.lower(), self._approach_default
        )
        candidates = self._sample_ring(target_xy, approach_d)
        scored, rejected = self._filter_by_costmap(candidates)
        self._publish_candidates(target_xy, scored, rejected)

        if not scored:
            self._n_no_feasible += 1
            # Day 8+: tally rejection reasons so the WARN line tells
            # the operator exactly WHY every candidate failed (instead
            # of just "all rejected"). This is the single most useful
            # log when Nav2 ABORTs on a goal that visually looks fine
            # in the occupancy map.
            reason_counts: Dict[str, int] = {}
            for _sx, _sy, _yaw, reason in rejected:
                # Normalise inflated_cell:NN → inflated_cell so the
                # tally is small enough to fit on a single line.
                key = reason.split(":", 1)[0]
                reason_counts[key] = reason_counts.get(key, 0) + 1
            reasons_str = ", ".join(
                f"{k}={v}" for k, v in sorted(reason_counts.items())
            ) or "none"
            self.get_logger().warn(
                f"no costmap-clear approach pose around "
                f"({target_xy[0]:.2f},{target_xy[1]:.2f}) at "
                f"d={approach_d:.2f}m. "
                f"{len(rejected)} candidates rejected ({reasons_str}). "
                f"See /semantic_goal/goal_candidates in RViz for the "
                f"per-pose reasons.",
                throttle_duration_sec=2.0,
            )
            self._publish_action_debug(
                f"NavigateToPose NOSEND reason=no_feasible_approach "
                f"target={sel.entity_id!r} class={sel.class_label!r} "
                f"target_xy=({target_xy[0]:.3f},{target_xy[1]:.3f}) "
                f"approach_d={approach_d:.2f} "
                f"rejected={reasons_str}"
            )
            self._tick_log()
            return

        # -- Pick best candidate every tick (pose is republished on
        # /semantic_goal/goal_pose for RViz freshness).
        best = self._pick_best(scored, target_xy, rx, ry, ryaw)

        # -- Day 9 hot-fix: explicit kill switch for the action send.
        # Republishes goal_pose for RViz, emits NOSEND on the debug
        # topic, but never calls send_goal_async. Use this for dry
        # runs where the operator wants to inspect the approach pose
        # without driving the robot.
        if not self._auto_send_goal:
            self._republish_goal_pose(best)
            self._maybe_publish_throttled_debug(
                f"NavigateToPose NOSEND reason=auto_send_goal_disabled "
                f"target={sel.entity_id!r} class={sel.class_label!r} "
                f"approach=({best[0]:.3f},{best[1]:.3f})"
            )
            self._tick_log()
            return

        # -- Throttle: only re-send the NavigateToPose action if the
        # target has moved more than `replan_distance_m` since the
        # last send, OR the target identity has changed entirely.
        # The goal_pose marker is republished by `_send_goal()` on
        # every actual send; for throttled ticks we still republish
        # `best` directly so RViz `/semantic_goal/goal_pose` stays
        # fresh AND emit a throttled NOSEND on action_debug — that's
        # what guarantees `ros2 topic echo /semantic_goal/action_debug
        # --once` always returns within ~replan_period_sec, instead
        # of looking dead the moment target_selector commits a
        # stable confirmed entity.
        if (
            self._last_sent_entity_id == sel.entity_id
            and self._last_sent_target_xy is not None
            and self._dist(self._last_sent_target_xy, target_xy)
                < self._replan_dist_m
        ):
            self._republish_goal_pose(best)
            in_flight = (
                "in_flight"
                if self._current_goal_handle is not None
                else "no_in_flight_handle"
            )
            elapsed = self._inflight_elapsed_str()
            # Day 9 hot-fix Task 3 — throttled NOSEND now carries
            # last_goal_status / last_result_msg / elapsed so the
            # operator can answer "did we ever hear back from Nav2?"
            # from a single `topic echo --once`.
            self._maybe_publish_throttled_debug(
                f"NavigateToPose NOSEND reason=throttled_target_unchanged "
                f"target={sel.entity_id!r} class={sel.class_label!r} "
                f"target_xy=({target_xy[0]:.3f},{target_xy[1]:.3f}) "
                f"approach=({best[0]:.3f},{best[1]:.3f}) "
                f"replan_dist_m={self._replan_dist_m:.2f} "
                f"state={in_flight} "
                f"sent_total={self._n_goals_sent} "
                f"last_goal_status={self._last_goal_status or 'none'!r} "
                f"last_result={self._last_result_msg or 'none'!r} "
                f"elapsed={elapsed}"
            )
            # Also emit a positive IN_FLIGHT heartbeat while a goal
            # is accepted, on its own timer so the operator's eye
            # sees "yes Nav2 still grinding" rather than only the
            # stale-looking NOSEND.
            self._maybe_emit_inflight(sel)
            self._tick_log()
            return

        # New target / target moved enough → send a fresh goal.
        queued = bool(self._send_goal(best, sel))
        if queued:
            self._last_sent_entity_id = sel.entity_id
            self._last_sent_target_xy = target_xy
        self._tick_log()

    def _republish_goal_pose(
        self, pose_xy_yaw: Tuple[float, float, float]
    ) -> None:
        """Publish best candidate as PoseStamped without sending action.

        Used during throttled ticks (target hasn't moved enough to
        warrant a re-send to Nav2) so /semantic_goal/goal_pose stays
        warm. Mirrors the publish in `_send_goal()` minus the action
        client call.
        """
        sx, sy, yaw = pose_xy_yaw
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self._global_frame
        ps.pose.position.x = float(sx)
        ps.pose.position.y = float(sy)
        ps.pose.position.z = 0.0
        ps.pose.orientation = _yaw_to_quat(yaw)
        self._goal_pub.publish(ps)
        self._latest_goal_pose = ps

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
        List[Tuple[float, float, float, str]],
    ]:
        """Split candidates into (cost-clear, rejected_with_reason).

        Day 8+: the rejected list now carries a per-candidate reason
        string drawn from {"outside_costmap", "unknown_cell",
        "lethal_cell", "inflated_cell"} so the rejected RViz markers
        can render a hover label. cost_threshold from the parameter
        set is used as the inflated/unknown gate; the existing
        success path is unchanged.
        """
        cm = self._costmap
        if cm is None:
            # No costmap means we can't classify anything — return
            # candidates as cost-clear and let the action server's
            # planner be the gate. Empty rejected list keeps the
            # marker layer quiet.
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
        rejected: List[Tuple[float, float, float, str]] = []
        for sx, sy, yaw in candidates:
            cx = int(math.floor((sx - ox) / res))
            cy = int(math.floor((sy - oy) / res))
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                # Outside the costmap — treat as rejected; we don't
                # know the cost. (Day 6+ map could still be growing.)
                rejected.append((sx, sy, yaw, "outside_costmap"))
                continue
            idx = cy * w + cx
            cost = int(data[idx])
            # Nav2 uses signed int8; -1 = unknown, 0..99 inflation
            # gradient, 100 lethal. Treat unknown as rejected to
            # be conservative. cost_threshold typically excludes
            # everything > 60.
            if cost < 0:
                rejected.append((sx, sy, yaw, "unknown_cell"))
                continue
            if cost >= 100:
                rejected.append((sx, sy, yaw, "lethal_cell"))
                continue
            if cost > self._cost_threshold:
                rejected.append(
                    (sx, sy, yaw, f"inflated_cell:{cost}")
                )
                continue
            scored.append((sx, sy, yaw))
        return scored, rejected

    def _diagnose_goal_costmap(
        self,
        goal_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
    ) -> Dict[str, object]:
        """Classify a (goal_xy) against the latest /global_costmap.

        Returns a dict with keys::

            origin_xy, resolution, width, height       -- costmap meta
            bounds_xy_min, bounds_xy_max               -- world-frame AABB
            in_bounds   : bool                         -- goal inside grid
            cell_xy     : (mx, my) | None              -- cell index
            cost        : int | None                   -- cost@cell
            cost_min_r0_2, cost_max_r0_2 : int | None  -- 0.2 m radius
            cost_min_r0_4, cost_max_r0_4 : int | None  -- 0.4 m radius
            target_cost : int | None                   -- cost@target
            category    : str
                one of {"no_costmap", "outside_costmap", "unknown_cell",
                "lethal_cell", "inflated_cell", "near_obstacle",
                "free"}
            summary     : str   -- single-line human-readable summary
                                   suitable for paste into Nav2 ABORT
                                   incident reports.

        ``goal_xy`` is the approach pose actually being sent to Nav2.
        ``target_xy`` is the underlying semantic-target centroid; we
        sample a 0.2 / 0.4 m ring around the goal to check for
        inflation halo, and an additional point at the target to
        report whether the *target itself* is on top of an obstacle
        (useful for diagnosing a bad semantic landmark vs a bad
        approach offset).
        """
        out: Dict[str, object] = {
            "origin_xy": None,
            "resolution": None,
            "width": None,
            "height": None,
            "bounds_xy_min": None,
            "bounds_xy_max": None,
            "in_bounds": False,
            "cell_xy": None,
            "cost": None,
            "cost_min_r0_2": None,
            "cost_max_r0_2": None,
            "cost_min_r0_4": None,
            "cost_max_r0_4": None,
            "target_cost": None,
            "category": "no_costmap",
            "summary": "no /global_costmap/costmap available",
        }
        cm = self._costmap
        if cm is None:
            return out
        info = cm.info
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
        res = float(info.resolution)
        w = int(info.width)
        h = int(info.height)
        out["origin_xy"] = (ox, oy)
        out["resolution"] = res
        out["width"] = w
        out["height"] = h
        if w == 0 or h == 0 or res <= 0.0:
            out["category"] = "no_costmap"
            out["summary"] = (
                f"degenerate costmap (w={w} h={h} res={res})"
            )
            return out
        bmin = (ox, oy)
        bmax = (ox + w * res, oy + h * res)
        out["bounds_xy_min"] = bmin
        out["bounds_xy_max"] = bmax

        gx, gy = goal_xy
        mx = int(math.floor((gx - ox) / res))
        my = int(math.floor((gy - oy) / res))
        in_bounds = (0 <= mx < w and 0 <= my < h)
        out["in_bounds"] = in_bounds
        out["cell_xy"] = (mx, my) if in_bounds else None

        if not in_bounds:
            out["category"] = "outside_costmap"
            out["summary"] = (
                f"goal=({gx:.3f},{gy:.3f}) outside costmap "
                f"bounds [({bmin[0]:.2f},{bmin[1]:.2f})..."
                f"({bmax[0]:.2f},{bmax[1]:.2f})] "
                f"(width={w} height={h} res={res:.3f})"
            )
            return out

        data = cm.data
        cost = int(data[my * w + mx])
        out["cost"] = cost

        # Sample a small disk around the goal. Even when the centre
        # cell is free, a 0.2 m halo of >50 cost is a near-certain
        # planner reject because RPP/DWB mark the inflation gradient
        # as "expensive" and Nav2's bt_navigator gives up after a
        # couple of replan attempts.
        for radius_m, lo_key, hi_key in (
            (0.20, "cost_min_r0_2", "cost_max_r0_2"),
            (0.40, "cost_min_r0_4", "cost_max_r0_4"),
        ):
            r_cells = max(1, int(math.ceil(radius_m / res)))
            lo, hi = 255, -1
            for dy in range(-r_cells, r_cells + 1):
                for dx in range(-r_cells, r_cells + 1):
                    if dx * dx + dy * dy > r_cells * r_cells:
                        continue
                    cx2 = mx + dx
                    cy2 = my + dy
                    if not (0 <= cx2 < w and 0 <= cy2 < h):
                        continue
                    c = int(data[cy2 * w + cx2])
                    if c < lo:
                        lo = c
                    if c > hi:
                        hi = c
            out[lo_key] = None if lo == 255 else lo
            out[hi_key] = None if hi == -1 else hi

        # Target-cell cost — useful to diagnose "bad semantic landmark"
        # (target sitting on a wall) vs "bad approach pose" (target OK
        # but our ring sample landed on inflation).
        tmx = int(math.floor((target_xy[0] - ox) / res))
        tmy = int(math.floor((target_xy[1] - oy) / res))
        if 0 <= tmx < w and 0 <= tmy < h:
            out["target_cost"] = int(data[tmy * w + tmx])

        # Categorise. Nav2 OccupancyGrid signed-int8 convention:
        #   -1   = unknown
        #   0    = free
        #   1-99 = inflated/costed
        #  100   = lethal/inscribed obstacle
        if cost < 0:
            cat = "unknown_cell"
        elif cost >= 100:
            cat = "lethal_cell"
        elif cost > self._cost_threshold:
            cat = "inflated_cell"
        else:
            # Centre is OK — but check the halo. If anything in the
            # 0.2 m ring is lethal or above threshold the planner
            # will likely reject the goal anyway.
            hi_ring = out.get("cost_max_r0_2") or 0
            if hi_ring >= 100:
                cat = "near_obstacle"
            elif hi_ring > self._cost_threshold:
                cat = "near_obstacle"
            else:
                cat = "free"

        out["category"] = cat
        # Build a human summary in nav2-ABORT-incident style.
        tcost_str = (
            f"target_cost={out['target_cost']}"
            if out["target_cost"] is not None
            else "target_cost=outside_costmap"
        )
        ring02 = (
            f"r0.2[{out['cost_min_r0_2']}..{out['cost_max_r0_2']}]"
            if out["cost_min_r0_2"] is not None
            else "r0.2=N/A"
        )
        ring04 = (
            f"r0.4[{out['cost_min_r0_4']}..{out['cost_max_r0_4']}]"
            if out["cost_min_r0_4"] is not None
            else "r0.4=N/A"
        )
        out["summary"] = (
            f"goal=({gx:.3f},{gy:.3f}) cell=({mx},{my}) "
            f"cost={cost} {ring02} {ring04} {tcost_str} "
            f"bounds=[({bmin[0]:.2f},{bmin[1]:.2f})..."
            f"({bmax[0]:.2f},{bmax[1]:.2f})] "
            f"thresh={self._cost_threshold}"
        )
        return out

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
    ) -> bool:
        """Send a NavigateToPose goal, canceling any predecessor.

        Returns ``True`` if ``send_goal_async`` queued cleanly (Nav2 goal
        may still reject); ``False`` on server-timeout / client exception —
        callers must skip latching `_last_sent_*` so the throttle cannot
        freeze replans.
        """
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
        self._latest_goal_pose = ps

        # Day 8+: log everything the operator needs to debug a Nav2
        # ABORT after the fact. The most common cause is "the goal
        # cell looked free here but Nav2's planner saw an inflated
        # neighbour"; printing the cell value and a small radius
        # min/max gives the operator that context without re-running
        # with --debug. _diagnose_goal_costmap also classifies the
        # result (free / inflated / lethal / unknown / outside) so
        # the logs are scannable.
        target_xy = (
            float(sel.target_pose_map.position.x),
            float(sel.target_pose_map.position.y),
        )
        diag = self._diagnose_goal_costmap((sx, sy), target_xy)
        self.get_logger().info(
            f"semantic goal diag: target entity={sel.entity_id!r} "
            f"class={sel.class_label!r} target=({target_xy[0]:.3f},"
            f"{target_xy[1]:.3f}) "
            f"approach=({sx:.3f},{sy:.3f}) yaw={math.degrees(yaw):.1f}° "
            f"category={diag['category']} {diag['summary']}"
        )

        server_ready = self._nav_client.wait_for_server(timeout_sec=1.0)
        if not server_ready:
            self.get_logger().warn(
                f"action server {self._nav_action_name!r} not "
                f"available; not sending goal",
                throttle_duration_sec=2.0,
            )
            self._last_goal_status = "SEND_FAILED"
            self._last_result_msg = "action_server_unavailable"
            self._publish_action_debug(
                f"NavigateToPose SEND_FAILED action_server_unavailable "
                f"action={self._nav_action_name} "
                f"target={sel.entity_id!r} class={sel.class_label!r}"
            )
            self._emit_reset_throttle(
                debug_reason="send_failed",
                target_entity=sel.entity_id,
                extra=(
                    " detail=action_server_unavailable "
                    f"action={self._nav_action_name!r}"
                ),
            )
            return False

        # Cancel the previous goal first, otherwise Nav2 sees a
        # second NavigateToPose and queues it. The bt_navigator
        # technically handles preemption itself but explicit
        # cancellation makes the node's intent visible in logs
        # and reduces edge cases on action_msgs version mismatch.
        self._cancel_current_goal()

        # Day 9 hot-fix: build SEND message FIRST, then publish it
        # together with the actual action send. Earlier the SEND
        # broadcast happened AFTER ``send_goal_async`` returned, so
        # any exception inside the action client (rare but possible
        # on rclpy shutdown / DDS hiccups) silently swallowed the
        # debug topic. Now the contract is: every successful
        # ``self._goal_pub.publish(ps)`` above is followed by
        # exactly one /semantic_goal/action_debug entry.
        # Capture the class label NOW, not at result time. By the
        # time _on_result fires, /target/selected may have rolled
        # over to a different class.
        self._current_class_label = (
            sel.class_label or ""
        ).lower().strip()
        send_msg = (
            f"NavigateToPose SEND target={sel.entity_id!r} "
            f"class={sel.class_label!r} "
            f"target_xy=({target_xy[0]:.3f},{target_xy[1]:.3f}) "
            f"approach=({sx:.3f},{sy:.3f}) "
            f"yaw_deg={math.degrees(yaw):.1f} "
            f"action={self._nav_action_name} "
            f"server_ready={server_ready}"
        )
        self.get_logger().info(send_msg)
        self._publish_action_debug(send_msg)

        goal = NavigateToPose.Goal()
        goal.pose = ps
        goal.behavior_tree = ""
        try:
            future = self._nav_client.send_goal_async(
                goal, feedback_callback=self._on_feedback
            )
        except Exception as exc:
            err = (
                f"NavigateToPose SEND_FAILED "
                f"send_goal_async_raised={type(exc).__name__}:{exc} "
                f"target={sel.entity_id!r} class={sel.class_label!r}"
            )
            self.get_logger().warn(err)
            self._last_goal_status = "SEND_FAILED"
            self._last_result_msg = (
                f"send_goal_async_raised={type(exc).__name__}"
            )
            self._publish_action_debug(err)
            self._emit_reset_throttle(
                debug_reason="send_failed",
                target_entity=sel.entity_id,
                extra=(
                    f"detail=send_goal_async_raised="
                    f"{type(exc).__name__}"
                ),
            )
            return False
        future.add_done_callback(self._on_goal_response)
        self._n_goals_sent += 1
        self._last_goal_status = "SENT"
        self._last_result_msg = ""
        return True

    def _emit_reset_throttle(
        self,
        *,
        debug_reason: str,
        target_entity: Optional[str] = None,
        extra: str = "",
    ) -> None:
        """Clear in-flight refs + `_last_sent_*` replay latch; log RESET_THROTTLE."""
        tid_raw = (
            target_entity
            if target_entity is not None
            else self._last_sent_entity_id
        )
        tid = tid_raw if tid_raw is not None else ""
        self._current_goal_handle = None
        self._goal_accepted_time = None
        self._last_sent_entity_id = None
        self._last_sent_target_xy = None
        suf = extra.strip()
        suf_part = f" {suf}" if suf else ""
        self._publish_action_debug(
            f"NavigateToPose RESET_THROTTLE reason={debug_reason} "
            f"target={tid!r}{suf_part}"
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
            self._last_goal_status = "REJECTED"
            self._last_result_msg = (
                f"future_exception={type(exc).__name__}"
            )
            self._publish_action_debug(
                f"NavigateToPose REJECTED future_exception={type(exc).__name__}"
            )
            self._emit_reset_throttle(
                debug_reason="goal_rejected",
                target_entity=self._last_sent_entity_id,
                extra=(
                    f"detail=goal_response_future_exception="
                    f"{type(exc).__name__}"
                ),
            )
            return
        if not handle.accepted:
            reject_msg = (
                "NavigateToPose REJECTED reason=action_server_rejected_goal "
                "(check lifecycle_manager_navigation reports "
                "'Managed nodes are active')"
            )
            self.get_logger().warn(reject_msg)
            self._last_goal_status = "REJECTED"
            self._last_result_msg = "action_server_rejected_goal"
            self._publish_action_debug(reject_msg)
            self._emit_reset_throttle(
                debug_reason="goal_rejected",
                target_entity=self._last_sent_entity_id,
                extra="detail=action_server_rejected_goal",
            )
            return
        accept_msg = (
            f"NavigateToPose ACCEPTED target={self._last_sent_entity_id!r} "
            f"class={self._current_class_label!r}"
        )
        self.get_logger().info(accept_msg)
        self._last_goal_status = "ACCEPTED"
        self._last_result_msg = ""
        self._goal_accepted_time = self.get_clock().now()
        self._last_inflight_emit_time = self._goal_accepted_time
        self._publish_action_debug(accept_msg)
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
            cls = self._current_class_label
            self._current_class_label = ""
            tgt = self._last_sent_entity_id or ""
            self._last_goal_status = "UNKNOWN"
            self._last_result_msg = (
                f"result_future_exception={type(exc).__name__}"
            )
            self._publish_action_debug(
                f"NavigateToPose RESULT: UNKNOWN "
                f"reason=result_future_raised class={cls!r} "
                f"exc={type(exc).__name__}"
            )
            self._emit_reset_throttle(
                debug_reason="result_future_exception",
                target_entity=tgt,
            )
            return
        status = result_wrapper.status
        result = result_wrapper.result
        # The handle is "done" — drop the cached reference so the
        # next replan tick can fire a fresh goal.
        self._current_goal_handle = None
        self._goal_accepted_time = None
        # Snapshot + clear the class label up-front so any subsequent
        # send overwrites it cleanly. Empty string means we somehow
        # got a result without a send (shouldn't happen but stays safe).
        cls = self._current_class_label
        self._current_class_label = ""
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._n_action_succeeded += 1
            result_msg = (
                f"NavigateToPose RESULT: SUCCEEDED "
                f"class={cls!r} error_code={result.error_code}"
            )
            self.get_logger().info(result_msg)
            self._last_goal_status = "SUCCEEDED"
            self._last_result_msg = (
                f"error_code={result.error_code}"
            )
            self._publish_action_debug(result_msg)
            self._publish_nav_status("SUCCEEDED")
            # task_coordinator's VERIFY_TARGET state matches on the
            # exact prefix "ARRIVED_CONFIRMED" — keep the format
            # stable. Class is best-effort context for the operator.
            self._publish_arrival_status(
                f"ARRIVED_CONFIRMED:{cls}" if cls else "ARRIVED_CONFIRMED"
            )
            # Latch before clearing throttle fields.
            latch_eid = self._last_sent_entity_id or ""
            self._arrival_published_for_entity = latch_eid
            self._emit_reset_throttle(
                debug_reason="result_SUCCEEDED",
                target_entity=latch_eid or None,
            )
        elif status == GoalStatus.STATUS_ABORTED:
            self._n_action_aborted += 1
            abort_eid = self._last_sent_entity_id or ""
            result_msg = (
                f"NavigateToPose RESULT: ABORTED "
                f"class={cls!r} error_code={result.error_code} "
                f"msg={result.error_msg!r}"
            )
            self.get_logger().warn(result_msg)
            self._last_goal_status = "ABORTED"
            self._last_result_msg = (
                f"error_code={result.error_code} "
                f"msg={result.error_msg!r}"
            )
            self._publish_action_debug(result_msg)
            self._publish_nav_status("ABORTED")
            self._publish_arrival_status(
                f"ARRIVAL_FAILED:{cls}" if cls else "ARRIVAL_FAILED"
            )
            self._emit_reset_throttle(
                debug_reason="result_ABORTED",
                target_entity=abort_eid or None,
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self._n_action_canceled += 1
            cancel_eid = self._last_sent_entity_id or ""
            result_msg = f"NavigateToPose RESULT: CANCELED class={cls!r}"
            self.get_logger().info(result_msg)
            self._last_goal_status = "CANCELED"
            self._last_result_msg = "self_preempt_or_user_cancel"
            self._publish_action_debug(result_msg)
            # Note: CANCELED is intentionally NOT mirrored onto
            # /arrival/status — it usually means "we preempted
            # ourselves to send a fresher goal", not a real failure
            # the FSM should react to. task_coordinator's _on_nav_status
            # ignores CANCELED for the same reason.
            self._publish_nav_status("CANCELED")
            self._emit_reset_throttle(
                debug_reason="result_CANCELED",
                target_entity=cancel_eid or None,
            )
        else:
            unk_eid = self._last_sent_entity_id or ""
            result_msg = (
                f"NavigateToPose RESULT: UNKNOWN status={status} "
                f"class={cls!r}"
            )
            self.get_logger().info(result_msg)
            self._last_goal_status = "UNKNOWN"
            self._last_result_msg = f"raw_status={status}"
            self._publish_action_debug(result_msg)
            self._emit_reset_throttle(
                debug_reason="result_UNKNOWN",
                target_entity=unk_eid or None,
            )

    def _publish_nav_status(self, value: str) -> None:
        self._navigation_status_pub.publish(String(data=value))

    def _publish_arrival_status(self, value: str) -> None:
        self._arrival_status_pub.publish(String(data=value))

    # ------------------------------------------------------------------
    # Day 9 hot-fix Task 3 — IN_FLIGHT heartbeat
    # ------------------------------------------------------------------
    def _inflight_elapsed_str(self) -> str:
        if self._goal_accepted_time is None:
            return "n/a"
        elapsed_ns = (
            self.get_clock().now() - self._goal_accepted_time
        ).nanoseconds
        return f"{elapsed_ns / 1e9:.2f}s"

    def _maybe_emit_inflight(self, sel: SelectedTarget) -> None:
        """Emit a periodic IN_FLIGHT line while a goal is accepted.

        The throttled NOSEND covers "I'm alive but not sending"; this
        helper covers the complementary "Nav2 has accepted my goal
        and is still grinding on it". Both lines together let the
        operator answer "should I be patient or kill the BT?" in
        seconds.
        """
        if self._current_goal_handle is None:
            return
        if self._inflight_period_sec <= 0.0:
            return
        now = self.get_clock().now()
        last = self._last_inflight_emit_time
        if last is not None:
            elapsed_ns = (now - last).nanoseconds
            if elapsed_ns < int(self._inflight_period_sec * 1e9):
                return
        self._last_inflight_emit_time = now
        elapsed = self._inflight_elapsed_str()
        msg = (
            f"NavigateToPose IN_FLIGHT target={sel.entity_id!r} "
            f"class={sel.class_label!r} elapsed={elapsed} "
            f"sent_total={self._n_goals_sent}"
        )
        # IN_FLIGHT is a *positive* signal — log at info level + go
        # straight onto /semantic_goal/action_debug bypassing the
        # NOSEND throttle clock.
        self.get_logger().info(msg, throttle_duration_sec=2.0)
        self._publish_action_debug(msg)

    # ------------------------------------------------------------------
    # Day 9 hot-fix Task 2 — distance-based arrival verifier
    # ------------------------------------------------------------------
    def _on_arrival_tick(self) -> None:
        """Publish SUCCEEDED + ARRIVED_CONFIRMED when robot reaches goal.

        Bridges the case where Nav2's action result callback never
        fires (DDS hiccup, BT quirk) but the robot has already
        physically arrived. Logic:
          1. We need a /target/selected entity, a /semantic_goal/goal_pose,
             and a TF map → base_link.
          2. dist(robot, goal_pose.position) < arrival_distance_threshold_m
             must hold continuously for >= arrival_hold_sec.
          3. If require_low_cmd_vel_for_arrival, /cmd_vel must also be
             below the linear/angular thresholds when the latch fires.
          4. We only fire ONCE per entity_id — _arrival_published_for_entity
             is the latch.
        """
        if not self._distance_arrival_enabled:
            return
        sel = self._latest_selected
        if sel is None or not sel.entity_id:
            self._arrival_close_since = None
            return
        # Already declared arrival for this entity (either via Nav2
        # result callback or a previous arrival tick) — don't re-fire.
        if self._arrival_published_for_entity == sel.entity_id:
            return
        gp = self._latest_goal_pose
        if gp is None:
            return
        robot = self._lookup_robot_pose()
        if robot is None:
            return
        rx, ry, _ = robot
        gx = float(gp.pose.position.x)
        gy = float(gp.pose.position.y)
        dist = math.hypot(rx - gx, ry - gy)

        now = self.get_clock().now()
        within = dist <= self._arrival_dist_thresh
        if not within:
            self._arrival_close_since = None
            return
        if self._arrival_close_since is None:
            self._arrival_close_since = now
        # Need to have been close for at least arrival_hold_sec.
        held_ns = (now - self._arrival_close_since).nanoseconds
        if held_ns < int(self._arrival_hold_sec * 1e9):
            return

        # Optionally require the robot to actually be stopped.
        if self._require_low_cmd_vel:
            if self._latest_cmd_vel is None:
                return
            lin_mag = math.hypot(
                float(self._latest_cmd_vel.linear.x),
                float(self._latest_cmd_vel.linear.y),
            )
            ang_mag = abs(float(self._latest_cmd_vel.angular.z))
            if (lin_mag > self._cmd_vel_lin_thresh
                    or ang_mag > self._cmd_vel_ang_thresh):
                return

        cls = (sel.class_label or "").lower().strip()
        info_msg = (
            f"distance arrival verifier latched: "
            f"target={sel.entity_id!r} class={cls!r} "
            f"dist={dist:.3f}m thresh={self._arrival_dist_thresh:.2f}m "
            f"hold={held_ns/1e9:.2f}s "
            f"goal_pose=({gx:.3f},{gy:.3f}) "
            f"robot=({rx:.3f},{ry:.3f}) "
            f"nav2_status={self._last_goal_status or 'none'!r}"
        )
        self.get_logger().info(info_msg)
        self._publish_action_debug(
            f"NavigateToPose RESULT: SUCCEEDED "
            f"reason=distance_based_arrival_verifier "
            f"target={sel.entity_id!r} class={cls!r} "
            f"dist={dist:.3f}m hold={held_ns/1e9:.2f}s "
            f"nav2_status={self._last_goal_status or 'none'!r}"
        )
        self._publish_nav_status("SUCCEEDED")
        self._publish_arrival_status(
            f"ARRIVED_CONFIRMED:{cls}" if cls else "ARRIVED_CONFIRMED"
        )
        self._arrival_published_for_entity = sel.entity_id
        self._emit_reset_throttle(
            debug_reason="distance_based_arrival",
            target_entity=sel.entity_id,
        )
        # Keep a marker on the action lifecycle so subsequent
        # NOSEND lines tell the operator we already declared arrival.
        self._last_goal_status = "ARRIVED_DISTANCE"
        self._last_result_msg = (
            f"distance_arrival dist={dist:.3f}m hold={held_ns/1e9:.2f}s"
        )

    def _publish_action_debug(self, value: str) -> None:
        """Mirror NavigateToPose lifecycle onto a single string topic.

        Always also stamps the message into the planner's logger at
        the call-site, but routing through this helper guarantees the
        exact same string appears on /semantic_goal/action_debug —
        which is what scripts/debug_nav2_action_chain.sh greps.
        """
        try:
            self._action_debug_pub.publish(String(data=value))
        except Exception as exc:
            self.get_logger().warn(
                f"action_debug publish raised: "
                f"{type(exc).__name__}: {exc}",
                throttle_duration_sec=5.0,
            )

    def _maybe_publish_throttled_debug(self, value: str) -> None:
        """Rate-limit version of _publish_action_debug.

        Used for the high-frequency NOSEND categories (no_target,
        no_costmap, no_tf, throttled_target_unchanged) so that
        /semantic_goal/action_debug always has a fresh message
        within ``action_debug_throttle_period_sec`` (default 1.0 s)
        without overwhelming `ros2 topic echo` consumers. When
        ``action_debug_emit_throttled=False`` the call is a no-op
        for those high-frequency states (the lifecycle SEND /
        ACCEPTED / RESULT events still go through
        `_publish_action_debug` directly).
        """
        if not self._emit_throttled_debug:
            return
        now = self.get_clock().now()
        if self._action_debug_throttle > 0.0:
            elapsed_ns = (now - self._last_throttled_debug_time).nanoseconds
            if elapsed_ns < int(self._action_debug_throttle * 1e9):
                return
        self._last_throttled_debug_time = now
        self._publish_action_debug(value)

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
        rejected: List[Tuple[float, float, float, str]],
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
        for i, item in enumerate(rejected):
            sx, sy, _yaw, reason = item
            ma.markers.append(self._candidate_marker(
                "rejected", 1000 + i, sx, sy,
                ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.5),
            ))
            # Hovering RViz label so the operator can read WHY a
            # specific red sphere was dropped without diving into
            # the planner logs.
            txt = Marker()
            txt.header = clear.header
            txt.ns = "rejected_reason"
            txt.id = 2000 + i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(sx)
            txt.pose.position.y = float(sy)
            txt.pose.position.z = 0.20
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.10
            txt.color = ColorRGBA(r=1.0, g=0.7, b=0.7, a=0.95)
            txt.text = reason
            ma.markers.append(txt)
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
