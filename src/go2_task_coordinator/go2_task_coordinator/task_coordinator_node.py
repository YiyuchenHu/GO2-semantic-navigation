"""Task coordinator FSM for Go2 semantic navigation.

Day 8 changes (vs. the original Phase-3-era coordinator)
--------------------------------------------------------
1. Switched the topic surface from the legacy go2_navigation stack to
   the Day 6/7 go2_semantic_perception stack:
       /semantic_query/selected_target  →  /target/selected
       /semantic_map/entities           →  /semantic_map/objects
   (See docs/day7_completion.md for stack history.)

2. Added a new state EXPLORE which replaces SEARCH on every transition
   that previously meant "we don't know where the target is":
       TARGET_NOT_FOUND  →  EXPLORE         (was → SEARCH)
       VERIFY_TARGET timeout → EXPLORE      (was → SEARCH)
   SEARCH is kept as an enum value with a deprecation comment so legacy
   downstream consumers of /exploration/enabled (search_manager_node)
   keep importing cleanly. Nothing in this file enters SEARCH any more.

3. EXPLORE owns its own NavigateToPose action client. It calls
   /get_frontiers (go2_msgs/srv/GetFrontiers) on entry, sends the
   highest-scored frontier as a Nav2 goal, and on SUCCEEDED re-queries
   for the next frontier. Three consecutive ABORTED → FAILED. An empty
   frontier list with success=True → FAILED("environment fully
   explored"). approach_goal_planner stays untouched and continues to
   own NavigateToPose during APPROACH; this coexistence is on purpose
   (see Day 8 design decision C).

4. EXPLORE is preempted by /semantic_map/objects: if any entity in the
   incoming SemanticEntityArray matches the effective target_class, we
   cancel the in-flight Nav2 goal and transition back to CHECK_MEMORY
   so target_selector + approach_goal_planner can take over.

5. target_class resolution: prefer SemanticTask.target_class from
   /semantic_task/request; fall back to the launch parameter
   default_target_class. If neither is set when EXPLORE is entered we
   FAIL with a clear message rather than wandering aimlessly.
"""

from __future__ import annotations

import re
from enum import Enum
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from go2_msgs.msg import (
    SelectedTarget,
    SemanticEntityArray,
    SemanticTask,
)
from go2_msgs.srv import GetFrontiers
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

# -----------------------------------------------------------------------------
# Coordinator-local NL fallback (PARSE_COMMAND when nl_parser is deaf / DDS miss)
# -----------------------------------------------------------------------------
_FALLBACK_PUNCT_RE = re.compile(r"[^\w\s]")
_FALLBACK_STOP = frozenset(
    {
        "a", "an", "the", "to", "please", "could", "would", "you", "i", "me",
        "go", "goto", "head", "move", "drive", "walk", "find", "fetch",
        "navigate", "navigation", "over", "for", "of", "at", "on", "in",
        "robot", "go2", "dog", "hey",
    }
)


def coordinator_fallback_target_class(command: str) -> Optional[str]:
    """Keyword map for PARSE_COMMAND if /semantic_task/request never arrives."""
    raw = (command or "").strip()
    if not raw:
        return None
    norm = _FALLBACK_PUNCT_RE.sub(" ", raw.lower())
    if "dining table" in norm:
        return "table"
    tokens = [t for t in norm.split() if t and t not in _FALLBACK_STOP]
    person_kw = {"person", "human", "man", "worker", "people"}
    table_kw = {"table", "desk", "workbench"}
    for t in tokens:
        if t in person_kw:
            return "person"
    for t in tokens:
        if t in table_kw:
            return "table"
    if "workbench" in norm:
        return "table"
    return None


class FsmState(str, Enum):
    IDLE = "IDLE"
    PARSE_COMMAND = "PARSE_COMMAND"
    CHECK_MEMORY = "CHECK_MEMORY"
    TARGET_FOUND = "TARGET_FOUND"
    TARGET_NOT_FOUND = "TARGET_NOT_FOUND"
    # Day 8: EXPLORE replaces SEARCH on every active transition.
    EXPLORE = "EXPLORE"
    # DEPRECATED — kept only so legacy code that still subscribes to
    # /exploration/enabled keeps building. No transition leads here in
    # Day 8+. Safe to delete once search_manager_node is retired.
    SEARCH = "SEARCH"
    PLAN_APPROACH_GOAL = "PLAN_APPROACH_GOAL"
    NAVIGATE_TO_GOAL = "NAVIGATE_TO_GOAL"
    VERIFY_TARGET = "VERIFY_TARGET"
    ARRIVED = "ARRIVED"
    FAILED = "FAILED"
    SAFETY_STOP = "SAFETY_STOP"


# Acts as a bounded retry counter for transient pre-conditions during
# EXPLORE entry: GetFrontiers returning success=False ("no map yet"),
# or robot-pose TF lookup not ready. Each tick we re-attempt; we give
# up once we've burnt this many attempts and call it FAILED("no map").
_EXPLORE_PRECONDITION_RETRIES = 6  # 6 ticks * 0.2 s = ~1.2 s; doubles
                                   # as "GetFrontiers returned !success
                                   # this many times" once we're past
                                   # the pose-lookup phase.

# Three consecutive frontier nav failures → declare exploration broken.
# Single ABORT is common (Nav2 abandons a recovery tree on a tricky
# corner) and shouldn't kill the whole search.
_EXPLORE_MAX_CONSECUTIVE_ABORTS = 3


class TaskCoordinatorNode(Node):
    def __init__(self) -> None:
        super().__init__("task_coordinator_node")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        # Frame names for TF lookup of the robot pose passed to
        # /get_frontiers. Defaults match day7.launch.py
        # (target_frame:=map, base_frame:=base_link).
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        # Fallback target_class when no SemanticTask has arrived. Lets
        # `ros2 launch ... target_class:=chair` do an end-to-end EXPLORE
        # → CHECK_MEMORY demo without an LLM/parser in the loop.
        self.declare_parameter("default_target_class", "")
        # Service / action names. Match the rest of the Day 7 launch.
        self.declare_parameter("get_frontiers_service", "/get_frontiers")
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("tick_period_sec", 0.2)
        self.declare_parameter("log_period_sec", 2.0)
        # PARSE_COMMAND fallback. When nl_parser never delivers
        # /semantic_task/request (volatile QoS, DDS miss, or parser wedged),
        # the coordinator can synthesize a SemanticTask from the raw
        # /user_command string after ``parse_command_fallback_sec``.
        # ``<= 0`` is treated as "almost immediate" (one scheduler cycle).
        self.declare_parameter("parse_command_fallback_sec", 0.5)
        # Day 8 wiring fix: target_selector (go2_semantic_perception) is
        # parameter-driven — it filters /semantic_map/objects against its
        # own `target_class` parameter. day8_two_phase.launch.py boots it
        # with target_class:"", so without an explicit dynamic update,
        # /target/selected stays empty after a SemanticTask arrives and
        # approach_goal_planner never sends a Nav2 goal. We close that
        # loop here: every /semantic_task/request → push the requested
        # target_class onto /target_selector via AsyncParameterClient.
        self.declare_parameter("target_selector_node_name", "/target_selector")
        # ---- Day 8+: TF readiness tolerance ---------------------------
        # `robot_base_frame` is the canonical name for the base link
        # used in TF readiness checks. It defaults to the legacy
        # `base_frame` value so existing launch files (which set
        # `base_frame:=base_link`) keep working. Override only if you
        # need the readiness check to follow a different frame than
        # the one used by /lookup_robot_pose.
        self.declare_parameter("robot_base_frame", "")
        # Odometry frame for fine-grained diagnostics when the
        # readiness check fails: we report which leg of map→odom→
        # base_link is missing so the operator can tell SLAM stalls
        # from URDF / static_transform_publisher mistakes.
        self.declare_parameter("odom_frame", "odom")
        # In SLAM mode, slam_toolbox's first map→odom transform may
        # take 5-10 s to appear (it fires once the first scan match
        # converges). Failing the EXPLORE branch on the 6th tick
        # (~1.2 s) used to lock the FSM into FAILED before SLAM had
        # a chance to come up. The new behaviour: keep retrying for
        # this many wall-clock seconds, publishing
        #   WAITING_FOR_TF: <global_frame>->...
        # on /task/status until either TF arrives or the grace
        # expires. Once it's been seen at least once, mid-exploration
        # drop-outs also fall under the same grace window.
        self.declare_parameter("tf_startup_grace_sec", 15.0)
        # Throttle for the WAITING_FOR_TF log line. The /task/status
        # publish itself fires every tick; this only governs the
        # human-readable WARN log, so 2 s strikes a balance between
        # "operator sees something is happening" and "console is not
        # spammed for 15 s straight".
        self.declare_parameter("tf_retry_status_period_sec", 2.0)

        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._default_target_class = (
            str(self.get_parameter("default_target_class").value)
            .lower()
            .strip()
        )
        self._get_frontiers_service = str(
            self.get_parameter("get_frontiers_service").value
        )
        self._nav_action_name = str(
            self.get_parameter("nav_action_name").value
        )
        tick_period = float(self.get_parameter("tick_period_sec").value)
        self._tick_period_sec = tick_period
        self._log_period_ns = int(
            float(self.get_parameter("log_period_sec").value) * 1e9
        )
        self._parse_fallback_sec = float(
            self.get_parameter("parse_command_fallback_sec").value
        )
        self._target_selector_node_name = str(
            self.get_parameter("target_selector_node_name").value
        ).strip()
        # Sentinel "" → fall back to base_frame so legacy launches
        # (only setting base_frame) get identical behaviour.
        _rbf = str(self.get_parameter("robot_base_frame").value).strip()
        self._robot_base_frame = _rbf if _rbf else self._base_frame
        self._odom_frame = str(self.get_parameter("odom_frame").value).strip()
        self._tf_startup_grace_sec = float(
            self.get_parameter("tf_startup_grace_sec").value
        )
        self._tf_retry_status_period_ns = int(
            float(self.get_parameter("tf_retry_status_period_sec").value)
            * 1e9
        )
        # Counter for synthesized fallback task_ids — keeps them unique
        # so target_selector / approach_planner can tell repeated
        # /user_command pulses apart in their logs.
        self._fallback_task_seq = 0
        # Latest /user_command text while waiting on nl_parser (cleared when
        # /semantic_task/request arrives or fallback runs).
        self._pending_user_command: str = ""
        # Throttle TARGET_SELECTOR_PENDING lines.
        self._sel_pending_log_ns: int = 0
        # Pending target_class → target_selector dispatch. Set by
        # _on_semantic_task; flushed by _tick once the parameter
        # service is reachable. Decoupling the dispatch from the
        # SemanticTask callback lets us survive the (common) startup
        # race where /semantic_task/request fires before
        # /target_selector has finished registering its services.
        self._pending_target_selector_class: str = ""
        # ---- TF-readiness state ---------------------------------------
        # When non-None, holds the wall-clock ns at which the most
        # recent run of consecutive missing-TF lookups began. Reset to
        # None as soon as a lookup succeeds. (now - this) compared
        # against tf_startup_grace_sec drives the WAITING_FOR_TF →
        # FAILED transition.
        self._tf_first_missing_ns: Optional[int] = None
        # Throttle for the WAITING_FOR_TF log line.
        self._tf_last_status_log_ns: int = 0
        # When set, _publish_controls overrides /task/status with
        # "WAITING_FOR_TF:<message>". Cleared as soon as TF is healthy.
        self._tf_wait_message: Optional[str] = None
        # Latest tf2 exception text, kept for the FAILED diagnostics
        # message so the operator sees the same string nav2 sees.
        self._tf_last_exception: str = ""

        # --------------------------------------------------------------
        # State
        # --------------------------------------------------------------
        self._state = FsmState.IDLE
        self._current_task: Optional[SemanticTask] = None
        self._selected_target: Optional[SelectedTarget] = None
        self._latest_objects: Optional[SemanticEntityArray] = None
        self._goal: Optional[PoseStamped] = None
        self._navigation_status = "IDLE"
        self._arrival_status = "UNKNOWN"
        self._safety_status = "OK"
        self._state_enter_ns = self.get_clock().now().nanoseconds
        self._last_log_ns = 0

        # EXPLORE bookkeeping --------------------------------------------
        # In-flight async GetFrontiers future.
        self._frontier_future = None
        # NavigateToPose goal handle for the EXPLORE-owned goal.
        self._explore_nav_handle = None
        # In-flight send_goal_async future (kept so we can ignore stale
        # callbacks after a state transition).
        self._explore_goal_send_future = None
        self._explore_consecutive_aborts = 0
        # Bounded retry counters for the pre-conditions of an EXPLORE
        # tick: TF lookup not ready yet, /get_frontiers returning
        # success=False (no map yet).
        self._explore_pose_retries = 0
        self._explore_no_map_retries = 0
        # Once set, contains the most recent failure message; surfaced
        # on /task/status so the user sees WHY exploration ended.
        self._failure_reason: Optional[str] = None

        # --------------------------------------------------------------
        # TF
        # --------------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --------------------------------------------------------------
        # ROS infra
        # --------------------------------------------------------------
        self.create_subscription(
            String, "/user_command", self._on_user_command, 10
        )
        self.create_subscription(
            SemanticTask, "/semantic_task/request", self._on_semantic_task, 10
        )
        # Day 8: switched from /semantic_query/selected_target to the
        # Day 7 stack's /target/selected.
        self.create_subscription(
            SelectedTarget, "/target/selected", self._on_selected_target, 10
        )
        # Day 8: subscribe to the live entity stream for EXPLORE
        # preemption AND to give CHECK_MEMORY an immediate shortcut on
        # warm cache (target already known when the task arrives).
        self.create_subscription(
            SemanticEntityArray,
            "/semantic_map/objects",
            self._on_objects,
            10,
        )
        # Approach planner still publishes the chosen approach pose on
        # this topic for RViz; we use the arrival as a "approach is
        # being driven" signal to keep the existing FSM transitions.
        self.create_subscription(
            PoseStamped,
            "/semantic_goal/goal_pose",
            self._on_goal,
            10,
        )
        self.create_subscription(
            String, "/navigation/status", self._on_nav_status, 10
        )
        self.create_subscription(
            String, "/arrival/status", self._on_arrival_status, 10
        )
        self.create_subscription(
            String, "/safety/status", self._on_safety_status, 10
        )

        self._task_status_pub = self.create_publisher(
            String, "/task/status", 10
        )
        dbg_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._task_debug_pub = self.create_publisher(
            String, "/task/status/debug", dbg_qos
        )
        self._task_current_pub = self.create_publisher(
            SemanticTask, "/semantic_task/current", 10
        )
        # Legacy: stays at 0 (False) in Day 8 — search_manager is no
        # longer used. Kept to avoid breaking any external subscriber.
        self._explore_pub = self.create_publisher(
            Bool, "/exploration/enabled", 10
        )
        self._cancel_pub = self.create_publisher(
            Bool, "/navigation/cancel", 10
        )

        # --------------------------------------------------------------
        # GetFrontiers + NavigateToPose clients (Day 8)
        # --------------------------------------------------------------
        self._frontier_client = self.create_client(
            GetFrontiers, self._get_frontiers_service
        )
        self._nav_client = ActionClient(
            self, NavigateToPose, self._nav_action_name
        )

        # --------------------------------------------------------------
        # Parameter client for /target_selector target_class sync
        # --------------------------------------------------------------
        # Strip a leading '/' for AsyncParameterClient — the rclpy
        # implementation prepends it onto every service path
        # (`<remote>/set_parameters`), so a leading slash works but
        # produces '//target_selector/set_parameters' which some DDS
        # implementations log as a warning. Bare 'target_selector'
        # is friendlier.
        remote_node = self._target_selector_node_name.lstrip("/")
        try:
            self._target_selector_param_client: Optional[
                AsyncParameterClient
            ] = AsyncParameterClient(self, remote_node)
            self.get_logger().info(
                f"target_selector parameter client created for "
                f"remote_node={remote_node!r}"
            )
        except Exception as exc:
            # Should not happen unless rclpy itself is broken; degrade
            # gracefully by logging and disabling the sync.
            self._target_selector_param_client = None
            self.get_logger().error(
                f"failed to create AsyncParameterClient for "
                f"{remote_node!r}: {type(exc).__name__}: {exc}; "
                f"target_class sync disabled"
            )

        self.create_timer(tick_period, self._tick)
        self.get_logger().info(
            f"Task coordinator ready. global_frame={self._global_frame!r} "
            f"base_frame={self._base_frame!r} "
            f"default_target_class={self._default_target_class!r} "
            f"frontier_service={self._get_frontiers_service!r} "
            f"nav_action={self._nav_action_name!r}"
        )

    def _selector_diag(self, line: str) -> None:
        self.get_logger().info(line)
        try:
            self._task_debug_pub.publish(String(data=line))
        except Exception:
            pass

    # ==================================================================
    # Subscription callbacks
    # ==================================================================
    def _on_user_command(self, msg: String) -> None:
        self._pending_user_command = (msg.data or "").strip()
        self._set_state(FsmState.PARSE_COMMAND)

    def _on_semantic_task(self, msg: SemanticTask) -> None:
        self._pending_user_command = ""
        # Day 8 bugfix: a new task must invalidate residue from the
        # previous task. Without this the FSM happily reuses the old
        # _selected_target (e.g. last task's chair entity) when a new
        # task asks for a different class (microwave) and target_selector
        # — which is parameter-driven, not task-driven — keeps publishing
        # the old class. Clearing here is cheap and self-contained.
        self._current_task = msg
        self._selected_target = None
        self._navigation_status = "IDLE"
        self._arrival_status = "UNKNOWN"
        self._explore_no_map_retries = 0
        self._explore_pose_retries = 0
        self._explore_consecutive_aborts = 0
        # New task → fresh TF grace window. If TF was healthy this is
        # a no-op; if a previous task expired the grace, this gives
        # SLAM another chance instead of permanently locking out.
        self._tf_first_missing_ns = None
        self._tf_wait_message = None
        self._task_current_pub.publish(msg)
        # Day 8 wiring fix: push msg.target_class onto /target_selector
        # so the rest of the pipeline (target_selector → approach_planner
        # → Nav2) receives the correct semantic class. Empty target_class
        # is logged as a warning and skipped — that almost certainly
        # means the upstream NL parser couldn't extract a class, so
        # blindly clobbering /target_selector to "" would break any
        # in-flight task.
        self._set_target_selector_class(msg.target_class)
        self._set_state(FsmState.CHECK_MEMORY)

    def _on_selected_target(self, msg: SelectedTarget) -> None:
        # Day 8: relax the strict task_id match — the new
        # go2_semantic_perception target_selector publishes task_id=""
        # by design (see its source). We accept any non-empty entity_id
        # as a valid target whenever we're waiting for one.
        if not msg.entity_id:
            return
        if (
            self._current_task is not None
            and self._current_task.task_id
            and msg.task_id
            and msg.task_id != self._current_task.task_id
        ):
            return
        # Day 8 bugfix: even with task_id="" coming from the parameter-
        # driven target_selector, the FSM must reject candidates whose
        # class_label disagrees with the active task's target_class.
        # Otherwise a microwave task accepts a chair SelectedTarget and
        # navigates to the chair (real bug seen in Gate 4 of check_day8).
        target_cls = self._effective_target_class()
        if target_cls:
            cand_cls = (msg.class_label or "").lower().strip()
            if cand_cls and cand_cls != target_cls:
                # Don't spam: only log when class actually changes the
                # decision (not every 0.5 s of selector heartbeat).
                if cand_cls != getattr(self, "_last_rejected_cand_cls", ""):
                    self.get_logger().info(
                        f"discarding /target/selected: class_label="
                        f"{cand_cls!r} != target_class={target_cls!r}; "
                        f"check that target_selector.target_class param "
                        f"is in sync with the active task"
                    )
                    self._last_rejected_cand_cls = cand_cls
                return
        self._selected_target = msg
        # If we were exploring, the /semantic_map/objects callback
        # likely already preempted; this branch covers the case where
        # /target/selected arrives first.
        if self._state in (
            FsmState.EXPLORE,
            FsmState.CHECK_MEMORY,
            FsmState.IDLE,
            FsmState.TARGET_NOT_FOUND,
        ):
            if self._state == FsmState.EXPLORE:
                self._cancel_explore_goal("target acquired (via /target/selected)")
            self._set_state(FsmState.TARGET_FOUND)

    def _on_objects(self, msg: SemanticEntityArray) -> None:
        self._latest_objects = msg
        # EXPLORE preemption: if any entity matches the effective
        # target_class, cancel the frontier nav goal and bounce back
        # to CHECK_MEMORY so target_selector + approach_goal_planner
        # take over. Confidence/multi-frame gating is intentionally
        # left out for Day 8 MVP (see docs/day8_status.md known issue).
        if self._state != FsmState.EXPLORE:
            return
        target_cls = self._effective_target_class()
        if not target_cls:
            return
        for e in msg.entities:
            if (e.class_label or "").lower().strip() == target_cls:
                self.get_logger().info(
                    f"EXPLORE preempted: target_class={target_cls!r} "
                    f"entity_id={e.entity_id[:8] if e.entity_id else '?'} "
                    f"conf={e.confidence:.2f}"
                )
                self._cancel_explore_goal("target_class spotted by perception")
                self._set_state(FsmState.CHECK_MEMORY)
                return

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        if self._state in (FsmState.PLAN_APPROACH_GOAL, FsmState.TARGET_FOUND):
            self._set_state(FsmState.NAVIGATE_TO_GOAL)

    def _on_nav_status(self, msg: String) -> None:
        self._navigation_status = msg.data

    def _on_arrival_status(self, msg: String) -> None:
        self._arrival_status = msg.data

    def _on_safety_status(self, msg: String) -> None:
        self._safety_status = msg.data
        if "STOP" in msg.data.upper():
            self._set_state(FsmState.SAFETY_STOP)

    # ==================================================================
    # Main tick
    # ==================================================================
    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        state_age_sec = (now_ns - self._state_enter_ns) / 1e9

        # Day 8: PARSE_COMMAND normally exits when nl_parser publishes
        # /semantic_task/request. If that message is lost (volatile QoS)
        # or the parser stalls, fall back to a tiny regex parser of the
        # captured /user_command text, then optionally to
        # default_target_class (legacy launch demos).
        if self._state == FsmState.PARSE_COMMAND:
            eff_fb = float(self._parse_fallback_sec)
            if eff_fb <= 0.0:
                eff_fb = min(
                    0.05, max(self._tick_period_sec * 0.5, 0.02)
                )
            if state_age_sec >= eff_fb:
                raw_cmd = self._pending_user_command
                parsed = coordinator_fallback_target_class(raw_cmd)
                if parsed:
                    self._emit_coordinator_fallback_task(parsed, raw_cmd)
                elif self._default_target_class:
                    self._synthesize_fallback_task()
                    self._pending_user_command = ""

        state_age_sec = (now_ns - self._state_enter_ns) / 1e9

        # CHECK_MEMORY → TARGET_NOT_FOUND if we've waited long enough
        # for /target/selected without one arriving.
        if (
            self._state == FsmState.CHECK_MEMORY
            and state_age_sec > 2.0
            and self._selected_target is None
        ):
            # Day 8: also short-circuit straight to TARGET_FOUND if the
            # entity stream already carries our target. Saves a 2 s
            # CHECK_MEMORY → EXPLORE → preempt round-trip on warm cache.
            if self._memory_has_target():
                self._set_state(FsmState.TARGET_FOUND)
            else:
                self._set_state(FsmState.TARGET_NOT_FOUND)

        if self._state == FsmState.TARGET_FOUND:
            self._set_state(FsmState.PLAN_APPROACH_GOAL)

        if self._state == FsmState.TARGET_NOT_FOUND:
            # Day 8: was → SEARCH, now → EXPLORE.
            self._set_state(FsmState.EXPLORE)

        if self._state == FsmState.NAVIGATE_TO_GOAL:
            if self._navigation_status in ("SUCCEEDED", "RESULT_4"):
                self._set_state(FsmState.VERIFY_TARGET)
            elif "ABORT" in self._navigation_status:
                self._fail("NAVIGATE_TO_GOAL: nav_status reported ABORT")

        if self._state == FsmState.VERIFY_TARGET:
            if self._arrival_status.startswith("ARRIVED_CONFIRMED"):
                self._set_state(FsmState.ARRIVED)
            elif state_age_sec > 8.0:
                # Day 8: was → SEARCH, now → EXPLORE.
                self._set_state(FsmState.EXPLORE)

        if self._state == FsmState.SAFETY_STOP:
            self._publish_cancel(True)

        # Day 8: EXPLORE driver. Idempotent — only kicks off a fresh
        # frontier query when there is no goal in flight and no
        # outstanding service future. Pre-conditions (TF, frontier
        # service ready) are re-checked each tick with bounded retries.
        if self._state == FsmState.EXPLORE:
            self._explore_drive_step()

        # Day 8 wiring fix: flush a pending /target_selector target_class
        # update if it could not be sent immediately (e.g. /target_selector
        # not yet up when the SemanticTask arrived). Cheap when nothing is
        # pending — short-circuits on the first guard.
        self._maybe_flush_target_selector_class()

        self._publish_controls()
        self._maybe_heartbeat(now_ns)

    # ==================================================================
    # EXPLORE state machinery
    # ==================================================================
    def _explore_drive_step(self) -> None:
        """Re-attempt frontier query if EXPLORE has nothing in flight."""
        # We have an active Nav2 goal — wait for its result.
        if self._explore_nav_handle is not None:
            return
        # We have an outstanding service future — wait for the response
        # callback to fire. The future's done callback will move the
        # state forward.
        if self._frontier_future is not None and not self._frontier_future.done():
            return

        target_cls = self._effective_target_class()
        if not target_cls:
            self._fail(
                "EXPLORE: no target_class set (no SemanticTask AND "
                "no `default_target_class` parameter)"
            )
            return

        if not self._frontier_client.service_is_ready():
            # Bounded wait for the service to come up.
            self._explore_no_map_retries += 1
            if self._explore_no_map_retries > _EXPLORE_PRECONDITION_RETRIES:
                self._fail(
                    f"EXPLORE: {self._get_frontiers_service!r} not "
                    f"available after "
                    f"{_EXPLORE_PRECONDITION_RETRIES} ticks"
                )
            return

        pose = self._lookup_robot_pose()
        if pose is None:
            # Wall-clock grace replaces the previous tick-counter gate.
            # Reasoning: in SLAM mode slam_toolbox's first map→odom
            # transform can take 5-10 s after a /scan starts flowing;
            # the old "6 ticks @ 0.2 s = 1.2 s" budget was guaranteed
            # to fail before SLAM published anything. The new behaviour
            # keeps the FSM in EXPLORE but flips the published status
            # to WAITING_FOR_TF until either TF appears or the grace
            # window (default 15 s) expires.
            now_ns = self.get_clock().now().nanoseconds
            if self._tf_first_missing_ns is None:
                self._tf_first_missing_ns = now_ns
            elapsed_sec = (now_ns - self._tf_first_missing_ns) / 1e9
            self._explore_pose_retries += 1  # kept for heartbeat info
            self._tf_wait_message = (
                f"{self._global_frame}->{self._robot_base_frame} not "
                f"ready (elapsed={elapsed_sec:.1f}s/"
                f"{self._tf_startup_grace_sec:.1f}s)"
            )
            # Throttle WARN logs so the console isn't spammed for 15 s.
            if (
                now_ns - self._tf_last_status_log_ns
                >= self._tf_retry_status_period_ns
            ):
                self.get_logger().warn(
                    f"WAITING_FOR_TF: {self._tf_wait_message}",
                )
                self._tf_last_status_log_ns = now_ns
            if elapsed_sec > self._tf_startup_grace_sec:
                # Grace expired — collect granular diagnostics so the
                # operator sees which leg of the TF chain is missing.
                diag = self._diagnose_tf_chain()
                self._fail(
                    f"EXPLORE: TF lookup {self._global_frame}->"
                    f"{self._robot_base_frame} not ready after "
                    f"{self._tf_startup_grace_sec:.1f}s grace "
                    f"({diag})"
                )
                # Clear so the next task isn't poisoned by stale state.
                self._tf_first_missing_ns = None
                self._tf_wait_message = None
            return

        # TF healthy — clear the grace state so any future drop-out
        # gets a fresh window AND the WAITING_FOR_TF status disappears
        # from /task/status on the very next tick.
        self._tf_first_missing_ns = None
        self._tf_wait_message = None
        # Reset pose retries — once we got one, we'll keep getting them.
        self._explore_pose_retries = 0
        req = GetFrontiers.Request()
        req.robot_pose = pose
        self._frontier_future = self._frontier_client.call_async(req)
        self._frontier_future.add_done_callback(self._on_frontier_response)

    def _on_frontier_response(self, future) -> None:
        # The future may complete after we've left EXPLORE (e.g. user
        # issued a new task). Drop the response in that case.
        if self._state != FsmState.EXPLORE:
            self._frontier_future = None
            return
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"GetFrontiers call raised: {type(exc).__name__}: {exc}"
            )
            self._frontier_future = None
            self._explore_consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        self._frontier_future = None

        if not resp.success:
            self._explore_no_map_retries += 1
            self.get_logger().warn(
                f"GetFrontiers !success ({resp.message}); retry "
                f"{self._explore_no_map_retries}/"
                f"{_EXPLORE_PRECONDITION_RETRIES}"
            )
            if self._explore_no_map_retries > _EXPLORE_PRECONDITION_RETRIES:
                self._fail(
                    f"EXPLORE: GetFrontiers !success "
                    f"{_EXPLORE_PRECONDITION_RETRIES + 1}x — {resp.message}"
                )
            # Else: the next _tick will re-attempt.
            return

        # Reset map-retry counter on a clean response.
        self._explore_no_map_retries = 0

        if not resp.frontier_goals:
            target_cls = self._effective_target_class() or "<unknown>"
            self._fail(
                f"EXPLORE: environment fully explored, target "
                f"{target_cls!r} not found"
            )
            return

        self.get_logger().info(
            f"EXPLORE: GetFrontiers returned {len(resp.frontier_goals)} "
            f"frontier(s); driving to best (score="
            f"{resp.scores[0] if resp.scores else float('nan'):.1f}, "
            f"info_gain={resp.info_gains[0] if resp.info_gains else 0}, "
            f"dist={resp.distances[0] if resp.distances else 0.0:.2f}m)"
        )
        self._send_explore_goal(resp.frontier_goals[0])

    def _send_explore_goal(self, goal_pose: PoseStamped) -> None:
        if self._state != FsmState.EXPLORE:
            return
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Nav2 action server {self._nav_action_name!r} not "
                f"available; will retry on next tick"
            )
            return
        # Force the goal stamp to "now" so Nav2 doesn't reject it for
        # being older than its TF buffer (the explorer stamps with the
        # service-call time, which can drift in sim).
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        if not goal_pose.header.frame_id:
            goal_pose.header.frame_id = self._global_frame

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.behavior_tree = ""

        self._explore_goal_send_future = self._nav_client.send_goal_async(goal)
        self._explore_goal_send_future.add_done_callback(
            self._on_explore_goal_response
        )

    def _on_explore_goal_response(self, future) -> None:
        if self._state != FsmState.EXPLORE:
            self._explore_goal_send_future = None
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"EXPLORE goal_response raised: {exc}"
            )
            self._explore_goal_send_future = None
            self._explore_consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        self._explore_goal_send_future = None
        if handle is None or not handle.accepted:
            self.get_logger().warn(
                "EXPLORE: Nav2 rejected the frontier goal"
            )
            self._explore_consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        self._explore_nav_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_explore_result)

    def _on_explore_result(self, future) -> None:
        # Always clear the handle; whether we recover or fail, this goal
        # is done.
        self._explore_nav_handle = None
        if self._state != FsmState.EXPLORE:
            # Likely we already preempted ourselves. Don't recurse.
            return
        try:
            result_wrapper = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"EXPLORE result future raised: {exc}"
            )
            self._explore_consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        status = result_wrapper.status if result_wrapper else 0
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._explore_consecutive_aborts = 0
            self.get_logger().info(
                "EXPLORE: frontier goal SUCCEEDED; querying next frontier"
            )
            # The next tick will re-call /get_frontiers because both
            # _explore_nav_handle and _frontier_future are None.
            return
        if status == GoalStatus.STATUS_CANCELED:
            # Cancel was driven by us (preemption). State already moved.
            return
        # ABORTED or unknown — treat as soft failure of THIS frontier.
        self._explore_consecutive_aborts += 1
        self.get_logger().warn(
            f"EXPLORE: frontier goal ended status={status}; "
            f"consecutive_aborts={self._explore_consecutive_aborts}/"
            f"{_EXPLORE_MAX_CONSECUTIVE_ABORTS}"
        )
        self._maybe_fail_after_aborts()

    def _maybe_fail_after_aborts(self) -> None:
        if self._explore_consecutive_aborts >= _EXPLORE_MAX_CONSECUTIVE_ABORTS:
            self._fail(
                f"EXPLORE: {_EXPLORE_MAX_CONSECUTIVE_ABORTS} consecutive "
                f"frontier-nav failures"
            )

    def _cancel_explore_goal(self, reason: str) -> None:
        if self._explore_nav_handle is not None:
            self.get_logger().info(
                f"EXPLORE: canceling Nav2 goal — {reason}"
            )
            try:
                self._explore_nav_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"EXPLORE cancel failed: {type(exc).__name__}: {exc}"
                )
            self._explore_nav_handle = None
        # Also publish the legacy /navigation/cancel signal so anything
        # still hooked to the old stack stops.
        self._publish_cancel(True)

    # ==================================================================
    # FSM helpers
    # ==================================================================
    def _set_state(self, new_state: FsmState) -> None:
        if self._state == new_state:
            return
        old = self._state
        self._state = new_state
        self._state_enter_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f"FSM {old.value} -> {new_state.value}")

        # Reset EXPLORE bookkeeping on entry — last run's state is stale.
        if new_state == FsmState.EXPLORE:
            self._explore_consecutive_aborts = 0
            self._explore_pose_retries = 0
            self._explore_no_map_retries = 0
            self._failure_reason = None

        # Leaving EXPLORE without a cancel (e.g. natural transition)
        # should still drop any cached handle so the next entry starts
        # clean. Cancel was already issued by the caller if needed.
        if old == FsmState.EXPLORE and new_state != FsmState.EXPLORE:
            self._explore_nav_handle = None
            self._frontier_future = None

    def _fail(self, reason: str) -> None:
        if self._state == FsmState.FAILED:
            return
        self._failure_reason = reason
        # Cancel anything we still own.
        if self._explore_nav_handle is not None:
            self._cancel_explore_goal(f"FAILED: {reason}")
        self.get_logger().error(f"FSM FAILED: {reason}")
        self._set_state(FsmState.FAILED)

    def _effective_target_class(self) -> str:
        if (
            self._current_task is not None
            and self._current_task.target_class
        ):
            return self._current_task.target_class.lower().strip()
        return self._default_target_class

    # ==================================================================
    # /target_selector parameter sync (Day 8 wiring fix)
    # ==================================================================
    def _set_target_selector_class(self, target_class: str) -> None:
        """Schedule an async ``target_class`` push to /target_selector.

        Empty class is a no-op (with a warning) — see _on_semantic_task
        for the rationale. Otherwise we either dispatch immediately
        (services ready) or stash the value for _tick to flush.
        Never blocks; never raises.
        """
        cls = (target_class or "").lower().strip()
        if not cls:
            self.get_logger().warn(
                "/target_selector target_class sync skipped: empty "
                "target_class on incoming SemanticTask. Upstream NL "
                "parser likely failed to map the user's command to a "
                "known class."
            )
            return
        if self._target_selector_param_client is None:
            # Constructor logged a clear error; don't spam.
            return
        # Stash, then attempt immediate flush. _maybe_flush will
        # idempotently retry on subsequent ticks if services aren't
        # discoverable yet.
        self._pending_target_selector_class = cls
        self._selector_diag(
            f"TARGET_SELECTOR_SET requested target_class={cls!r}"
        )
        self._maybe_flush_target_selector_class()

    def _maybe_flush_target_selector_class(self) -> None:
        """Try to dispatch the latest pending target_class update."""
        cls = self._pending_target_selector_class
        if not cls:
            return
        client = self._target_selector_param_client
        if client is None:
            self._selector_diag(
                f"TARGET_SELECTOR_SET_FAILED target_class={cls!r} "
                f"reason=no_parameter_client"
            )
            self._pending_target_selector_class = ""
            return
        if not client.services_are_ready():
            now = self.get_clock().now().nanoseconds
            if now - self._sel_pending_log_ns >= int(2e9):
                self._sel_pending_log_ns = now
                self._selector_diag(
                    f"TARGET_SELECTOR_PENDING target_class={cls!r} "
                    f"service_not_ready node={self._target_selector_node_name!r}"
                )
            return
        param = Parameter("target_class", Parameter.Type.STRING, cls)
        try:
            future = client.set_parameters([param])
        except Exception as exc:
            self._selector_diag(
                f"TARGET_SELECTOR_SET_FAILED target_class={cls!r} "
                f"reason=dispatch_exception:{exc!r}"
            )
            return
        self._pending_target_selector_class = ""
        future.add_done_callback(
            lambda f, c=cls: self._on_target_selector_param_done(f, c)
        )

    def _on_target_selector_param_done(
        self, future, requested_cls: str
    ) -> None:
        """Log set_parameters outcome; re-queue on failure."""
        try:
            response = future.result()
        except Exception as exc:
            self._selector_diag(
                f"TARGET_SELECTOR_SET_FAILED target_class={requested_cls!r} "
                f"reason=future_exception:{exc!r}"
            )
            self._pending_target_selector_class = requested_cls
            return
        if response is None:
            self._selector_diag(
                f"TARGET_SELECTOR_SET_FAILED target_class={requested_cls!r} "
                f"reason=null_response"
            )
            self._pending_target_selector_class = requested_cls
            return
        results = list(getattr(response, "results", []) or [])
        if not results:
            self._selector_diag(
                f"TARGET_SELECTOR_SET_OK target_class={requested_cls!r} "
                f"detail=empty_results_list"
            )
            return
        r0 = results[0]
        if getattr(r0, "successful", False):
            self._selector_diag(
                f"TARGET_SELECTOR_SET_OK target_class={requested_cls!r}"
            )
        else:
            reason = getattr(r0, "reason", "")
            self._selector_diag(
                f"TARGET_SELECTOR_SET_FAILED target_class={requested_cls!r} "
                f"reason={reason!r}"
            )
            self._pending_target_selector_class = requested_cls

    def _emit_coordinator_fallback_task(
        self, nav_class: str, raw_command: str
    ) -> None:
        """Synthesize SemanticTask from /user_command when nl_parser is silent."""
        self._fallback_task_seq += 1
        task = SemanticTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.header.frame_id = self._global_frame
        task.task_id = f"tc-fallback-{self._fallback_task_seq:04d}"
        task.raw_command = raw_command or "(user_command empty)"
        task.intent = "find"
        task.target_class = nav_class.lower().strip()
        task.target_label = task.target_class
        task.target_aliases = []
        task.frame_id = self._global_frame
        task.requires_search = True
        task.timeout_sec = 0.0
        self.get_logger().info(
            f"PARSE_COMMAND coordinator fallback: task_id={task.task_id!r} "
            f"target_class={task.target_class!r} raw={raw_command!r} "
            f"(no timely /semantic_task/request)"
        )
        self._on_semantic_task(task)

    def _synthesize_fallback_task(self) -> None:
        """Create a minimal SemanticTask from default_target_class.

        Drives the same code path as a "real" /semantic_task/request:
        store self._current_task, publish on /semantic_task/current,
        transition to CHECK_MEMORY. Lets PARSE_COMMAND make progress
        when there is no upstream NL parser in the loop yet.
        """
        if not self._default_target_class:
            return
        self._fallback_task_seq += 1
        task = SemanticTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.header.frame_id = self._global_frame
        task.task_id = f"fallback-{self._fallback_task_seq:04d}"
        task.raw_command = "(synthesized from default_target_class)"
        task.intent = "find"
        task.target_class = self._default_target_class
        task.target_label = self._default_target_class
        task.target_aliases = []
        task.frame_id = self._global_frame
        task.requires_search = True
        task.timeout_sec = 0.0
        self.get_logger().info(
            f"PARSE_COMMAND fallback: synthesized SemanticTask "
            f"task_id={task.task_id!r} target_class="
            f"{self._default_target_class!r} (no external parser publishing "
            f"/semantic_task/request)"
        )
        # Route through the regular task ingress so residual state from
        # any previous task is cleared in one place.
        self._on_semantic_task(task)

    def _memory_has_target(self) -> bool:
        if self._latest_objects is None:
            return False
        target_cls = self._effective_target_class()
        if not target_cls:
            return False
        for e in self._latest_objects.entities:
            if (e.class_label or "").lower().strip() == target_cls:
                return True
        return False

    def _lookup_robot_pose(self) -> Optional[PoseStamped]:
        # Time() = "latest available". This deliberately avoids passing
        # `now` because it would ask tf2 for a sample that hasn't been
        # published yet ("extrapolation into the future") on slow
        # map→odom links — the exact failure mode we used to misdiagnose
        # as "TF missing".
        try:
            t = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            # Cache the message so _diagnose_tf_chain can surface it
            # on the eventual FAILED status without re-querying tf2.
            self._tf_last_exception = (
                f"{type(exc).__name__}: {exc}"
            )
            return None
        ps = PoseStamped()
        ps.header.frame_id = self._global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(t.transform.translation.x)
        ps.pose.position.y = float(t.transform.translation.y)
        ps.pose.position.z = float(t.transform.translation.z)
        ps.pose.orientation = t.transform.rotation
        return ps

    def _diagnose_tf_chain(self) -> str:
        """Per-leg readiness summary for the FAILED diagnostic.

        Splits the canonical SLAM TF chain (map → odom → base_link)
        and reports which legs are present. Almost always at least one
        leg is missing on a real failure: SLAM not converged →
        map→odom missing; URDF / static_transform_publisher not loaded
        → odom→base_link missing; both missing usually means TF wasn't
        published at all (a launch wiring bug).
        """
        def _has(parent: str, child: str) -> bool:
            try:
                return bool(
                    self._tf_buffer.can_transform(
                        parent, child, Time(),
                        timeout=Duration(seconds=0.05),
                    )
                )
            except TransformException:
                return False
        map_to_odom = _has(self._global_frame, self._odom_frame)
        odom_to_base = _has(self._odom_frame, self._robot_base_frame)
        return (
            f"{self._global_frame}->{self._odom_frame}="
            f"{'OK' if map_to_odom else 'MISSING'}, "
            f"{self._odom_frame}->{self._robot_base_frame}="
            f"{'OK' if odom_to_base else 'MISSING'}, "
            f"global_frame={self._global_frame!r} "
            f"robot_base_frame={self._robot_base_frame!r} "
            f"odom_frame={self._odom_frame!r} "
            f"last_exception={self._tf_last_exception!r}"
        )

    # ==================================================================
    # Periodic publishing
    # ==================================================================
    def _publish_controls(self) -> None:
        status = String()
        if self._state == FsmState.FAILED and self._failure_reason:
            status.data = f"{self._state.value}:{self._failure_reason}"
        elif (
            self._state == FsmState.EXPLORE
            and self._tf_wait_message is not None
        ):
            # While EXPLORE is parked waiting for SLAM TF, publish a
            # bespoke transitional status so the operator (and any
            # downstream UI) can distinguish "FSM is patiently waiting"
            # from a hard FAILED. The state itself stays EXPLORE.
            status.data = f"WAITING_FOR_TF:{self._tf_wait_message}"
        else:
            status.data = self._state.value
        self._task_status_pub.publish(status)

        # Day 8: legacy /exploration/enabled stays at False. EXPLORE no
        # longer wakes search_manager_node.
        explore = Bool()
        explore.data = False
        self._explore_pub.publish(explore)

        if self._state not in (
            FsmState.SAFETY_STOP,
            FsmState.FAILED,
            FsmState.EXPLORE,
        ):
            # Don't toggle cancel during EXPLORE — we manage that
            # explicitly via _cancel_explore_goal.
            self._publish_cancel(False)

    def _publish_cancel(self, flag: bool) -> None:
        c = Bool()
        c.data = flag
        self._cancel_pub.publish(c)

    def _maybe_heartbeat(self, now_ns: int) -> None:
        if self._log_period_ns <= 0:
            return
        if now_ns - self._last_log_ns < self._log_period_ns:
            return
        self._last_log_ns = now_ns
        target_cls = self._effective_target_class() or "<unset>"
        self.get_logger().info(
            f"[coord/hb] state={self._state.value} "
            f"target_class={target_cls!r} "
            f"nav_status={self._navigation_status} "
            f"arrival={self._arrival_status} "
            f"explore_aborts={self._explore_consecutive_aborts} "
            f"explore_no_map={self._explore_no_map_retries} "
            f"in_flight=goal:{self._explore_nav_handle is not None} "
            f"req:{self._frontier_future is not None}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
