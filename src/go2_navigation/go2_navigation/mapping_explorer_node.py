"""Day 8 (two-phase variant) — autonomous map-completion driver.

Single responsibility: drive Go2 around the environment until the SLAM
``/map`` has no remaining frontier clusters, then idle. NO knowledge of
semantic targets, NO hand-shake with task_coordinator, NO NLP. The
semantic perception stack runs in parallel and quietly populates
``/semantic_map/objects`` with chair / table / etc. as Go2 passes them;
those entities persist on the SLAM map (semantic_memory's
``permanent_after_observations`` ensures that).

When mapping is done, an external **Phase B** stack (NLP parser + Day 7
target_selector / approach_planner / task_coordinator) drives Go2 to a
human-issued goal like ``go to table``. By then the entity is already
in ``/semantic_map/objects`` so the FSM never has to call
``/get_frontiers`` and never enters EXPLORE.

State machine (intentionally small)
-----------------------------------

    IDLE                          (boot — wait for service + first /map)
      └─ pre-conditions met ──►  NAVIGATING (request frontier, send Nav2 goal)
    NAVIGATING
      ├─ Nav2 SUCCEEDED      ──►  re-query /get_frontiers
      ├─ Nav2 ABORTED        ──►  count failure, drop this frontier, re-query
      ├─ frontier list empty ──►  DONE
      └─ N consecutive abts  ──►  FAILED
    DONE / FAILED
      └─ /mapping/control == "restart" ──►  IDLE

Topics published
----------------
* ``/mapping/status`` (std_msgs/String) — one of
  ``IDLE`` / ``NAVIGATING`` / ``DONE`` / ``FAILED:<reason>``.
  TRANSIENT_LOCAL so a late subscriber (Phase B operator console)
  immediately sees the latest state.

Topics consumed
---------------
* ``/mapping/control`` (std_msgs/String) — operator hooks.
  ``restart`` resets the FSM to IDLE and starts a fresh sweep.
  ``abort`` cancels any in-flight Nav2 goal and goes to DONE.

Service / action clients
------------------------
* ``/get_frontiers`` (go2_msgs/srv/GetFrontiers) — frontier_explorer.
* ``/navigate_to_pose`` (nav2_msgs/action/NavigateToPose) — Nav2.

This node is the Day 8 phase-A counterpart to ``task_coordinator`` in
the legacy single-launch design, with all the target-driven complexity
stripped out. It's deliberately ~one-third the size.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from go2_msgs.srv import GetFrontiers
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
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class _State(str, Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    DONE = "DONE"
    FAILED = "FAILED"


# Bounded retries while waiting for the frontier service / first robot
# pose / first /map. 120 ticks at 0.5 s tick = 60 s grace period — slam_toolbox
# needs many seconds to publish its first map->odom TF after the sim
# restarts (lidar must rotate, scan must match, lifecycle must activate).
# Be generous; the real cost of waiting is just a heartbeat log every
# few seconds.
_BOOT_PRECONDITION_RETRIES = 120

# When boot fails (TF / service still missing after _BOOT_PRECONDITION_RETRIES),
# mapping_explorer enters FAILED. Instead of staying there forever, we
# AUTO-RESET back to IDLE every _AUTO_RESET_AFTER_FAIL_SEC seconds and
# try again. Phase A is supposed to be self-healing — the operator
# should never need to manually publish 'restart'. Only stay FAILED
# for good if the failure happened mid-mission (after at least one
# Nav2 goal succeeded).
_AUTO_RESET_AFTER_FAIL_SEC = 30.0

# Frontier-attempt accounting. Two layers of avoidance:
#
#  1. SOFT blacklist (timed) — every ABORT marks a frontier as
#     "skip me for the next abort_cooldown_sec seconds". Lets us
#     try other frontiers while Nav2 / costmap settle, and gives the
#     bad frontier a chance to vanish when SLAM grows the map.
#
#  2. HARD blacklist (permanent) — once a frontier has been ABORTed
#     ``_MAX_ATTEMPTS_PER_FRONTIER`` times, mark it permanently dead.
#     No amount of waiting will help: it's likely a goal cell wedged
#     against a wall / inflation ring / out-of-bounds.
#
# This is the operator's request from the day8_two_phase RViz session
# (Wed May 6): "if a candidate centroid won't accept Nav2 goals 3 times
# in a row, just refuse it instead of keeping it in PLAN_APPROACH_GOAL".
#
# _DEFAULT_ABORT_COOLDOWN_SEC is just the default for the
# ``abort_cooldown_sec`` ROS parameter — runtime override at launch time
# (or via `ros2 param set /mapping_explorer abort_cooldown_sec X`) is
# what you want for tuning. Lower it (5–10 s) for quick demos where
# you'd rather Go2 retry fast than spend half a minute idle waiting for
# a flaky frontier to "rest".
_DEFAULT_ABORT_COOLDOWN_SEC = 15.0   # soft skip window (seconds)
_MAX_ATTEMPTS_PER_FRONTIER = 3       # hard-blacklist after this many fails

# Two frontier centroids within this distance (metres) are considered
# "the same frontier" for accounting. Should be larger than the typical
# SLAM-grow per-tick centroid jitter (~0.1 m) but small enough that a
# 5 m apart, equally-good frontier still gets selected.
_ABORT_BLACKLIST_RADIUS_M = 1.0

# How many consecutive Nav2 ABORTs we tolerate before giving up. A
# single ABORT is common (corridor too tight, timed-out recovery), the
# coordinator-style behaviour skips that frontier and tries the next.
# 8 is generous on purpose — the Isaac Sim slam_toolbox map->odom TF
# lags sim time by ~1 s under GPU contention, so Nav2's first goal
# attempt almost always ABORTs with "Lookup would require extrapolation
# into the future". Combined with the recent-abort blacklist we still
# rotate frontiers; raising the budget just delays "give up entirely".
_DEFAULT_MAX_CONSECUTIVE_ABORTS = 8


class MappingExplorerNode(Node):
    """Frontier-driven autonomous mapping. See module docstring."""

    def __init__(self) -> None:
        super().__init__("mapping_explorer_node")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("get_frontiers_service", "/get_frontiers")
        self.declare_parameter("nav_action_name", "/navigate_to_pose")
        self.declare_parameter("status_topic", "/mapping/status")
        self.declare_parameter("control_topic", "/mapping/control")
        # tick is the FSM heartbeat; every tick we either send a new
        # frontier query, send a Nav2 goal, or wait. 0.5 s is enough —
        # Phase A is a "minutes-long" task, not a 10 Hz controller.
        self.declare_parameter("tick_period_sec", 0.5)
        self.declare_parameter("log_period_sec", 5.0)
        # How many consecutive Nav2 failures we tolerate before
        # declaring exploration broken. Larger than task_coordinator's
        # default (3) because Phase A is supposed to be tolerant —
        # we'd rather skip a couple bad frontiers than abort the sweep.
        self.declare_parameter(
            "max_consecutive_aborts", _DEFAULT_MAX_CONSECUTIVE_ABORTS
        )
        # Once frontier_explorer reports zero clusters, we wait this
        # many seconds before locking in DONE. The map can briefly
        # show no frontiers between two SLAM scans, especially right
        # after a goal-arrival rotation. Holding for 5 s avoids
        # flapping DONE / NAVIGATING.
        self.declare_parameter("done_confirm_sec", 5.0)
        # When True, after the very first Nav2 goal completes, future
        # /get_frontiers responses with empty list bypass the
        # confirmation timer and lock DONE immediately. Defaults to
        # False; useful for tiny scenes where the wait is just dead
        # time.
        self.declare_parameter("done_fast", False)
        # How long (seconds) a frontier centroid is "soft-skipped" after
        # a Nav2 ABORT, before it becomes eligible for retry. The lower
        # this is the snappier Go2 looks (less idle time between failed
        # attempts), but too low means we'll spam Nav2 with a frontier
        # that genuinely can't be reached and burn the consecutive-abort
        # budget. Default 15 s is a balance for the warehouse demo;
        # bump back to 30 if your scene has many transient costmap-
        # inflation false-failures and you'd rather wait than thrash.
        self.declare_parameter(
            "abort_cooldown_sec", _DEFAULT_ABORT_COOLDOWN_SEC
        )
        # ---------------- Day 9+ — visited-frontier blacklist (Task 2) ----
        # Distinct from the abort-blacklist above (which counts FAILURE).
        # This one tracks SUCCESS / arrival: once we've actually been at a
        # frontier centroid, don't go to it again — even if frontier_explorer
        # keeps returning a slightly-jittered version of the same cluster,
        # because (a) RViz markers were observed flickering between two
        # adjacent centroids while Go2 stood still, and (b) returning to a
        # visited centroid wastes time without adding any new SLAM info.
        # The aging memory window (default 120 s) prevents the blacklist
        # from growing unboundedly during very long mapping runs and lets
        # an intentional restart re-explore an area if SLAM has changed.
        self.declare_parameter("frontier_arrival_radius_m", 0.5)
        self.declare_parameter("visited_frontier_reject_radius_m", 0.75)
        self.declare_parameter("visited_frontier_memory_sec", 120.0)
        # After Nav2 reports SUCCEEDED, wait this long before calling
        # /get_frontiers again. Gives slam_toolbox a chance to fold the
        # arrival-pose scan into the map so the next frontier query
        # reflects the new known area instead of replaying the old one.
        self.declare_parameter("map_update_settle_sec", 0.8)
        # ---------------- /mapping/debug/status — operator triage feed ---
        # Rich human-readable line at 1 Hz with everything we know about
        # the current FSM tick. Lets `debug_mapping_explorer.sh` answer
        # the question "is the explorer stuck, recomputing, or done?"
        # without running ros2 node info / parameters. Topic is plain
        # std_msgs/String so it shows up in any `ros2 topic echo`
        # without extra deps.
        self.declare_parameter("debug_status_topic", "/mapping/debug/status")
        self.declare_parameter("debug_status_period_sec", 1.0)

        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._frontier_service = str(
            self.get_parameter("get_frontiers_service").value
        )
        self._nav_action = str(self.get_parameter("nav_action_name").value)
        self._status_topic = str(self.get_parameter("status_topic").value)
        self._control_topic = str(self.get_parameter("control_topic").value)
        self._tick_period = float(
            self.get_parameter("tick_period_sec").value
        )
        self._log_period_ns = int(
            float(self.get_parameter("log_period_sec").value) * 1e9
        )
        self._max_aborts = int(
            self.get_parameter("max_consecutive_aborts").value
        )
        self._done_confirm_sec = float(
            self.get_parameter("done_confirm_sec").value
        )
        self._done_fast = bool(self.get_parameter("done_fast").value)
        self._abort_cooldown_sec = float(
            self.get_parameter("abort_cooldown_sec").value
        )
        self._arrival_radius_m = float(
            self.get_parameter("frontier_arrival_radius_m").value
        )
        self._visited_reject_radius_m = float(
            self.get_parameter("visited_frontier_reject_radius_m").value
        )
        self._visited_memory_sec = float(
            self.get_parameter("visited_frontier_memory_sec").value
        )
        self._map_update_settle_sec = float(
            self.get_parameter("map_update_settle_sec").value
        )
        self._debug_status_topic = str(
            self.get_parameter("debug_status_topic").value
        )
        self._debug_status_period_sec = float(
            self.get_parameter("debug_status_period_sec").value
        )

        # Visited-frontier list. Each entry is (wx, wy, n_visits, ts_ns).
        # See parameter docstrings above for semantics. Linear scan —
        # the list is kept tiny via the memory-window prune in
        # _is_recently_visited.
        self._visited_frontiers: List[Tuple[float, float, int, int]] = []
        # Counter surfaced in /mapping/debug/status.
        self._visited_frontier_count = 0
        # When the most recent Nav2 goal SUCCEEDED. Used to enforce the
        # post-arrival settle so the next /get_frontiers call sees the
        # freshly-folded scan. None whenever no settle is pending.
        self._post_arrival_settle_until_ns: Optional[int] = None

        # Frontier-attempt accounting (see _MAX_ATTEMPTS_PER_FRONTIER
        # docstring above). Two parallel containers:
        #
        #   _attempt_log : list[(wx, wy, n_attempts, last_abort_ns)]
        #       Per-frontier counter. n_attempts increments on every
        #       ABORT for a goal whose (wx, wy) is within
        #       _ABORT_BLACKLIST_RADIUS_M of an existing entry; new
        #       entry is appended otherwise. Never expires — used for
        #       the hard "n >= _MAX_ATTEMPTS_PER_FRONTIER" check.
        #
        #   The "soft blacklist" is implicit: an entry whose
        #   ``last_abort_ns`` is younger than abort_cooldown_sec is
        #   skipped even if its count is below max.
        self._attempt_log: list = []
        # The (wx, wy) of the goal currently in flight — recorded so
        # _on_nav_result can attribute the ABORT to the right frontier.
        self._inflight_goal_xy: Optional[tuple] = None

        # FSM state
        self._state = _State.IDLE
        self._state_enter_ns = self.get_clock().now().nanoseconds
        self._frontier_future = None
        self._nav_goal_send_future = None
        self._nav_handle = None
        self._consecutive_aborts = 0
        self._boot_retries = 0
        # Number of successful Nav2 goals so far. Used by done_fast.
        self._n_goals_succeeded = 0
        self._failure_reason: Optional[str] = None
        # Timestamp at which the most recent /get_frontiers returned
        # an empty list. None whenever we have at least one frontier.
        self._empty_since_ns: Optional[int] = None
        # Last log time so the heartbeat doesn't spam.
        self._last_log_ns = 0
        # Total /get_frontiers + Nav2 goal counters surfaced in the
        # heartbeat for operator situational awareness.
        self._n_frontier_calls = 0
        self._n_goals_sent = 0

        # ----- Day 9+ /mapping/debug/status fields -------------------
        # Snapshot of the most recent /get_frontiers response for the
        # debug-status publisher. None means "no call has completed yet".
        self._dbg_last_get_frontiers_success: Optional[bool] = None
        self._dbg_last_get_frontiers_msg: str = ""
        self._dbg_last_n_frontiers: int = 0
        self._dbg_last_selected_idx: Optional[int] = None
        self._dbg_last_selected_score: Optional[float] = None
        self._dbg_last_selection_reason: str = ""
        # Last frontier centroid we successfully sent as a goal —
        # surfaces "are we re-using the same goal as last cycle?".
        self._dbg_last_frontier_goal_xy: Optional[Tuple[float, float]] = None
        # Cumulative counters for the debug status line.
        self._dbg_n_get_frontiers_calls = 0
        self._dbg_n_get_frontiers_success = 0
        self._dbg_n_get_frontiers_failure = 0
        self._dbg_n_nav2_accepted = 0
        # nav2 result lifecycle bookkeeping. ``last_nav2_result_status``
        # is one of "SENT" / "ACCEPTED" / "REJECTED" / "SUCCEEDED" /
        # "ABORTED" / "CANCELED" / "UNKNOWN" / "" (no goal yet); kept
        # as a plain string so the status topic is grep-friendly.
        self._dbg_last_nav2_status: str = ""

        # TF for the robot pose passed to /get_frontiers.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Status pub uses TRANSIENT_LOCAL so the Phase B operator
        # console attaching after Phase A finishes immediately reads
        # "DONE" instead of waiting for the next periodic publish.
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._status_pub = self.create_publisher(
            String, self._status_topic, status_qos
        )
        # /mapping/debug/status — same TRANSIENT_LOCAL/depth=1 QoS so a
        # late-attached operator script (debug_mapping_explorer.sh)
        # always sees the most recent status without missing a beat.
        self._debug_status_pub = self.create_publisher(
            String, self._debug_status_topic, status_qos
        )
        self.create_subscription(
            String, self._control_topic, self._on_control, 10
        )

        self._frontier_client = self.create_client(
            GetFrontiers, self._frontier_service
        )
        self._nav_client = ActionClient(
            self, NavigateToPose, self._nav_action
        )

        self.create_timer(self._tick_period, self._tick)
        # Independent 1 Hz debug-status timer (separate from FSM tick so
        # debug stays fresh even if the tick path early-returns waiting
        # for a future).
        self.create_timer(
            self._debug_status_period_sec, self._publish_debug_status
        )
        self.get_logger().info(
            f"mapping_explorer ready. global_frame={self._global_frame!r} "
            f"base_frame={self._base_frame!r} "
            f"frontier_service={self._frontier_service!r} "
            f"nav_action={self._nav_action!r} "
            f"max_consecutive_aborts={self._max_aborts} "
            f"done_confirm_sec={self._done_confirm_sec} "
            f"done_fast={self._done_fast} "
            f"abort_cooldown_sec={self._abort_cooldown_sec} "
            f"arrival_radius_m={self._arrival_radius_m:.2f} "
            f"visited_reject_radius_m={self._visited_reject_radius_m:.2f} "
            f"visited_memory_sec={self._visited_memory_sec:.0f} "
            f"map_settle_sec={self._map_update_settle_sec:.2f} "
            f"debug_status_topic={self._debug_status_topic!r}"
        )
        self._publish_status()
        self._publish_debug_status()

    # ------------------------------------------------------------------
    # Operator hook
    # ------------------------------------------------------------------
    def _on_control(self, msg: String) -> None:
        cmd = (msg.data or "").strip().lower()
        if cmd == "restart":
            self.get_logger().info(
                "mapping_explorer: control=restart — cancelling any "
                "in-flight goal and resetting to IDLE"
            )
            self._cancel_nav("operator restart")
            self._consecutive_aborts = 0
            self._boot_retries = 0
            self._empty_since_ns = None
            self._failure_reason = None
            self._inflight_goal_xy = None
            self._attempt_log = []
            # Day 9+ — restart wipes the visited blacklist too: operator
            # explicitly asked us to re-scan everything.
            self._visited_frontiers = []
            self._visited_frontier_count = 0
            self._post_arrival_settle_until_ns = None
            self._dbg_last_selection_reason = "operator_restart"
            self._set_state(_State.IDLE)
        elif cmd == "abort":
            self.get_logger().info(
                "mapping_explorer: control=abort — locking in DONE"
            )
            self._cancel_nav("operator abort")
            self._set_state(_State.DONE)
        else:
            self.get_logger().warn(
                f"mapping_explorer: unknown /mapping/control={cmd!r}; "
                f"expected 'restart' or 'abort'"
            )

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        # Re-publish status periodically; cheap, gives operator a
        # liveness signal even when we're "just navigating".
        self._publish_status()
        self._maybe_log()

        # Day 9+ Task 2 — opportunistically mark the in-flight goal
        # visited as soon as the robot is physically close to it. The
        # Nav2 SUCCEEDED callback is the *authoritative* marker, but
        # this side-channel guards against the symptom "Go2 reaches the
        # frontier but no result fires, then the next cycle picks the
        # same centroid". Cheap (one TF lookup, one hypot).
        if self._inflight_goal_xy is not None:
            self._maybe_mark_arrived_by_distance()

        if self._state == _State.IDLE:
            self._step_idle()
        elif self._state == _State.NAVIGATING:
            self._step_navigating()
        elif self._state == _State.FAILED:
            self._maybe_auto_reset_after_fail()
        # DONE: terminal until /mapping/control restarts us.

    def _maybe_auto_reset_after_fail(self) -> None:
        """Self-heal Phase A boot failures.

        If we entered FAILED before ever reaching NAVIGATING — i.e.
        before any Nav2 goal succeeded — it almost always means the
        startup race lost (slam_toolbox not active yet when the boot
        retry budget ran out). In that case it's safe and right to
        flip back to IDLE after a cooldown and try again. Once we've
        actually moved (``_n_goals_succeeded > 0``) FAILED becomes
        sticky again — the operator should investigate.
        """
        if self._n_goals_succeeded > 0:
            return  # mid-mission failure; stay FAILED, surface the issue
        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (now_ns - self._state_enter_ns) / 1e9
        if elapsed_sec < _AUTO_RESET_AFTER_FAIL_SEC:
            return
        self.get_logger().info(
            f"mapping_explorer: AUTO-RESET after {elapsed_sec:.0f}s in "
            f"FAILED (no Nav2 goal had succeeded; assuming boot race). "
            f"Returning to IDLE for another attempt."
        )
        self._consecutive_aborts = 0
        self._boot_retries = 0
        self._empty_since_ns = None
        self._failure_reason = None
        self._set_state(_State.IDLE)

    # ------------------------------------------------------------------
    # IDLE: wait for service + TF + first /map, then ask for a frontier
    # ------------------------------------------------------------------
    def _step_idle(self) -> None:
        if self._frontier_future is not None:
            # Boot-time query already in flight; wait for the callback.
            return

        if not self._frontier_client.service_is_ready():
            self._boot_retries += 1
            if self._boot_retries > _BOOT_PRECONDITION_RETRIES:
                self._fail(
                    f"frontier service {self._frontier_service!r} not "
                    f"available after {_BOOT_PRECONDITION_RETRIES} ticks"
                )
            return

        pose = self._lookup_robot_pose()
        if pose is None:
            self._boot_retries += 1
            if self._boot_retries > _BOOT_PRECONDITION_RETRIES:
                self._fail(
                    f"TF {self._global_frame}->{self._base_frame} not "
                    f"available after {_BOOT_PRECONDITION_RETRIES} ticks"
                )
            return

        # Boot pre-conditions OK — request first batch of frontiers.
        self._boot_retries = 0
        self._send_frontier_request(pose)

    # ------------------------------------------------------------------
    # NAVIGATING: wait for active Nav2 goal OR send the next one
    # ------------------------------------------------------------------
    def _step_navigating(self) -> None:
        # Active Nav2 goal in flight — its done callback will move
        # us forward.
        if self._nav_handle is not None:
            return
        # send_goal_async hasn't completed yet — wait for it.
        if (
            self._nav_goal_send_future is not None
            and not self._nav_goal_send_future.done()
        ):
            return
        # Outstanding /get_frontiers query — wait for its callback.
        if (
            self._frontier_future is not None
            and not self._frontier_future.done()
        ):
            return

        # Day 9+ Task 3 — after a frontier arrival we deliberately
        # withhold the next /get_frontiers call for ``map_update_settle_sec``.
        # SLAM needs a beat to fold the arrival-pose scan into the map;
        # without this gate we sometimes re-query the *old* map, get
        # the same centroid we just visited, and chase our tail.
        if self._post_arrival_settle_until_ns is not None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns < self._post_arrival_settle_until_ns:
                return
            # Settle expired — proceed with the fresh query and clear
            # the gate so we don't re-arm it on every tick.
            self._post_arrival_settle_until_ns = None

        # Nothing in flight — fire a fresh frontier request.
        pose = self._lookup_robot_pose()
        if pose is None:
            # TF lookup transiently failed mid-mission; just wait. We
            # already proved at boot that TF works.
            return
        self._send_frontier_request(pose)

    # ------------------------------------------------------------------
    # /get_frontiers handling
    # ------------------------------------------------------------------
    def _send_frontier_request(self, pose: PoseStamped) -> None:
        req = GetFrontiers.Request()
        req.robot_pose = pose
        self._n_frontier_calls += 1
        self._frontier_future = self._frontier_client.call_async(req)
        self._frontier_future.add_done_callback(self._on_frontier_response)

    def _on_frontier_response(self, future) -> None:
        self._dbg_n_get_frontiers_calls += 1
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"GetFrontiers call raised: {type(exc).__name__}: {exc}"
            )
            self._frontier_future = None
            self._dbg_last_get_frontiers_success = False
            self._dbg_last_get_frontiers_msg = (
                f"exception:{type(exc).__name__}"
            )
            self._dbg_n_get_frontiers_failure += 1
            self._consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        self._frontier_future = None

        # Snapshot for /mapping/debug/status.
        self._dbg_last_get_frontiers_success = bool(resp.success)
        self._dbg_last_get_frontiers_msg = str(resp.message or "")
        self._dbg_last_n_frontiers = (
            len(resp.frontier_goals) if resp.success else 0
        )
        self._dbg_last_selected_idx = None
        self._dbg_last_selected_score = None

        if not resp.success:
            # Map missing / metadata invalid — keep IDLE and wait.
            self._dbg_n_get_frontiers_failure += 1
            self._dbg_last_selection_reason = "get_frontiers_failed"
            self.get_logger().warn(
                f"GetFrontiers !success: {resp.message}; will retry "
                f"(state={self._state.value})"
            )
            return

        self._dbg_n_get_frontiers_success += 1

        if not resp.frontier_goals:
            # Possibly final answer; confirm with hold timer.
            self._dbg_last_selection_reason = "no_frontiers_returned"
            self._handle_empty_frontiers(resp.message)
            return

        # We have at least one frontier — clear the empty timer.
        self._empty_since_ns = None
        # Move into NAVIGATING (idempotent if already there).
        self._set_state(_State.NAVIGATING)

        # Pick the first frontier that is neither (a) in the recent-abort
        # blacklist nor (b) a frontier we have already visited recently
        # (Day 9+ Task 2). frontier_explorer sorts by score descending,
        # so this preserves the "always go to the best" intent while
        # rotating off frontiers we've just failed to reach OR already
        # reached. Per-candidate rejection reasons are kept so the
        # operator can see in /mapping/debug/status why we skipped over
        # the top-scoring frontier.
        chosen_idx: Optional[int] = None
        n_skipped_abort = 0
        n_skipped_visited = 0
        first_rejection_reason = ""
        for i, ps in enumerate(resp.frontier_goals):
            wx = float(ps.pose.position.x)
            wy = float(ps.pose.position.y)
            if self._is_blacklisted(wx, wy):
                n_skipped_abort += 1
                if not first_rejection_reason:
                    first_rejection_reason = (
                        "abort_blacklisted_frontier"
                    )
                continue
            if self._is_recently_visited(wx, wy):
                n_skipped_visited += 1
                if not first_rejection_reason:
                    first_rejection_reason = (
                        "already_visited_frontier"
                    )
                continue
            chosen_idx = i
            break

        if chosen_idx is None:
            # Every returned frontier is blacklisted. Decide which kind:
            #
            #   ALL HARD-blacklisted ──► Phase A is structurally done.
            #     Either the remaining frontiers are physically
            #     unreachable (wedged against walls, inflated out, sim
            #     glitch), OR SLAM has exhausted the actual environment
            #     and the only "unknown" left is outside the warehouse.
            #     Either way, no amount of waiting will help — promote
            #     to the empty-frontiers DONE-confirm path so we lock
            #     in DONE after done_confirm_sec instead of looping
            #     "NAVIGATING ↔ FAILED" forever.
            #
            #   At least one SOFT-blacklisted ──► temporary; one or
            #     more frontiers are still in their post-ABORT
            #     cooldown. Keep waiting; SLAM might also expand and
            #     surface a fresh frontier in the next call.
            n_hard = sum(
                1 for e in self._attempt_log
                if e[2] >= _MAX_ATTEMPTS_PER_FRONTIER
            )
            all_hard = True
            for ps in resp.frontier_goals:
                wx = float(ps.pose.position.x)
                wy = float(ps.pose.position.y)
                _, entry = self._find_attempt_entry(wx, wy)
                if entry is None or entry[2] < _MAX_ATTEMPTS_PER_FRONTIER:
                    all_hard = False
                    break

            if all_hard:
                self._dbg_last_selection_reason = (
                    "all_frontiers_hard_blacklisted"
                )
                self.get_logger().info(
                    f"mapping_explorer: all "
                    f"{len(resp.frontier_goals)} returned frontiers "
                    f"are HARD-blacklisted (log size="
                    f"{len(self._attempt_log)}); treating as "
                    f"'no reachable frontier' — entering DONE confirm."
                )
                self._handle_empty_frontiers(
                    f"all {len(resp.frontier_goals)} frontiers "
                    f"hard-blacklisted (unreachable)"
                )
            else:
                # Tag the dominant rejection reason for the operator —
                # if we skipped more visited than aborted, the node is
                # "circling old ground" not "pounding a bad frontier".
                if n_skipped_visited > n_skipped_abort:
                    self._dbg_last_selection_reason = (
                        f"all_frontiers_already_visited "
                        f"(visited={n_skipped_visited} "
                        f"abort={n_skipped_abort})"
                    )
                else:
                    self._dbg_last_selection_reason = (
                        f"all_frontiers_blacklisted "
                        f"(abort={n_skipped_abort} "
                        f"visited={n_skipped_visited})"
                    )
                self.get_logger().warn(
                    f"mapping_explorer: all "
                    f"{len(resp.frontier_goals)} returned frontiers "
                    f"are blacklisted (abort_log="
                    f"{len(self._attempt_log)}, {n_hard} hard, "
                    f"visited={len(self._visited_frontiers)}); "
                    f"waiting for SLAM to expand or soft cooldowns "
                    f"to lapse."
                )
            return

        chosen = resp.frontier_goals[chosen_idx]
        chosen_xy = (
            float(chosen.pose.position.x),
            float(chosen.pose.position.y),
        )
        chosen_score = (
            float(resp.scores[chosen_idx])
            if len(resp.scores) > chosen_idx else float("nan")
        )
        chosen_ig = (
            int(resp.info_gains[chosen_idx])
            if len(resp.info_gains) > chosen_idx else 0
        )
        chosen_dist = (
            float(resp.distances[chosen_idx])
            if len(resp.distances) > chosen_idx else 0.0
        )
        skipped = chosen_idx
        skip_note = (
            f" (skipped {skipped} higher-scored frontier(s) — "
            f"abort_skip={n_skipped_abort} "
            f"visited_skip={n_skipped_visited})"
            if skipped > 0 else ""
        )
        self.get_logger().info(
            f"mapping_explorer: frontier picked #{chosen_idx} — "
            f"xy=({chosen_xy[0]:.2f},{chosen_xy[1]:.2f}) "
            f"score={chosen_score:.1f} info_gain={chosen_ig} "
            f"dist={chosen_dist:.2f}m "
            f"({len(resp.frontier_goals)} returned, msg={resp.message!r})"
            f"{skip_note}"
        )
        self._inflight_goal_xy = chosen_xy
        self._dbg_last_selected_idx = chosen_idx
        self._dbg_last_selected_score = chosen_score
        self._dbg_last_frontier_goal_xy = chosen_xy
        if skipped > 0 and first_rejection_reason:
            # Surface "skipped N due to already_visited_frontier" as
            # the selection reason so debug script users can
            # immediately see the visited-blacklist working.
            self._dbg_last_selection_reason = (
                f"picked#{chosen_idx} after_skipping_"
                f"{first_rejection_reason}"
            )
        else:
            self._dbg_last_selection_reason = f"picked#{chosen_idx}"
        self._send_nav_goal(chosen)

    def _handle_empty_frontiers(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds

        # Day 9+ Phase B (May-9 mapping run) — fast-DONE for the
        # "exploration just finished cleanly" pattern. Symptom we hit:
        # mapping_status stayed NAVIGATING for the whole 120 s capture
        # even though the second frontier had already SUCCEEDED at
        # ~t=116 s and every subsequent /get_frontiers returned 0.
        # done_confirm_sec=5.0 would have eventually fired DONE at
        # ~t=121 s, but that's the wrong default: once we've genuinely
        # arrived at a frontier (Nav2 reported SUCCEEDED, no goal in
        # flight, no send_goal_async pending) and the next service
        # call returns 0 frontiers, there's nothing to wait for.
        # Holding NAVIGATING just makes the FSM look stuck and keeps
        # frontier markers alive in RViz.
        #
        # We deliberately keep this stricter than ``done_fast`` (which
        # locks DONE on *any* empty response after the first success).
        # Required predicates:
        #   * at least one frontier goal SUCCEEDED already,
        #   * the most recent Nav2 result was SUCCEEDED (we're at
        #     rest at the last frontier; not aborted, not mid-flight),
        #   * no goal is currently in flight or being sent.
        # If these all hold and frontiers=0, exploration is over.
        nothing_in_flight = (
            self._inflight_goal_xy is None
            and self._nav_handle is None
            and (
                self._nav_goal_send_future is None
                or self._nav_goal_send_future.done()
            )
        )
        if (
            self._n_goals_succeeded > 0
            and self._dbg_last_nav2_status == "SUCCEEDED"
            and nothing_in_flight
        ):
            # Make the debug status line carry the reason so an
            # operator reading /mapping/debug/status after the fact
            # immediately sees *why* we transitioned (and so the
            # recorder summary can scrape `reason=no_frontiers_returned`
            # from the line).
            self._dbg_last_selection_reason = "no_frontiers_returned"
            self.get_logger().info(
                f"mapping_explorer: 0 frontiers, nav2=SUCCEEDED, "
                f"in_flight=0, goals_succeeded={self._n_goals_succeeded}, "
                f"visited={self._visited_frontier_count} — locking DONE "
                f"immediately. message={message!r}"
            )
            self._set_state(_State.DONE)
            # Refresh the debug-status line right away so subscribers
            # see DONE+reason without waiting for the 1 Hz timer.
            self._publish_debug_status()
            return

        # done_fast: skip the confirm hold once Go2 has actually moved.
        if self._done_fast and self._n_goals_succeeded > 0:
            self.get_logger().info(
                f"mapping_explorer: 0 frontiers (done_fast active); "
                f"locking DONE — {message}"
            )
            self._set_state(_State.DONE)
            self._publish_debug_status()
            return

        if self._empty_since_ns is None:
            self._empty_since_ns = now_ns
            self.get_logger().info(
                f"mapping_explorer: 0 frontiers; holding for "
                f"{self._done_confirm_sec:.1f}s before locking DONE — "
                f"{message}"
            )
            return

        elapsed = (now_ns - self._empty_since_ns) / 1e9
        if elapsed >= self._done_confirm_sec:
            self.get_logger().info(
                f"mapping_explorer: 0 frontiers held for {elapsed:.1f}s "
                f"— DONE. message={message!r} "
                f"goals_succeeded={self._n_goals_succeeded} "
                f"frontier_calls={self._n_frontier_calls}"
            )
            self._set_state(_State.DONE)
            self._publish_debug_status()
        # Otherwise just wait; the next tick will re-query.

    # ------------------------------------------------------------------
    # NavigateToPose handling
    # ------------------------------------------------------------------
    def _send_nav_goal(self, goal_pose: PoseStamped) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Nav2 action server {self._nav_action!r} not "
                f"available; will retry on next tick"
            )
            return
        # Re-stamp NOW so Nav2 doesn't reject us for being older than
        # its TF buffer (frontier_explorer stamps with the service-call
        # time, which can lag in sim).
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        if not goal_pose.header.frame_id:
            goal_pose.header.frame_id = self._global_frame

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.behavior_tree = ""

        self._n_goals_sent += 1
        self._dbg_last_nav2_status = "SENT"
        self._nav_goal_send_future = self._nav_client.send_goal_async(goal)
        self._nav_goal_send_future.add_done_callback(
            self._on_nav_goal_response
        )

    def _on_nav_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"Nav2 send_goal_async raised: {exc}"
            )
            self._nav_goal_send_future = None
            self._dbg_last_nav2_status = f"SEND_FAILED:{type(exc).__name__}"
            self._consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        self._nav_goal_send_future = None
        if handle is None or not handle.accepted:
            self.get_logger().warn(
                "mapping_explorer: Nav2 rejected the frontier goal"
            )
            self._dbg_last_nav2_status = "REJECTED"
            self._consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        self._nav_handle = handle
        self._dbg_last_nav2_status = "ACCEPTED"
        self._dbg_n_nav2_accepted += 1
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        self._nav_handle = None
        try:
            wrapper = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"Nav2 result future raised: {exc}"
            )
            self._dbg_last_nav2_status = (
                f"RESULT_EXCEPTION:{type(exc).__name__}"
            )
            self._consecutive_aborts += 1
            self._maybe_fail_after_aborts()
            return
        status = wrapper.status if wrapper else 0
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._consecutive_aborts = 0
            self._n_goals_succeeded += 1
            # Day 9+ Task 2 + 3 — record arrival, arm post-arrival
            # settle, and clear the in-flight goal so the next tick
            # forces a fresh /get_frontiers query against the
            # newly-folded map.
            if self._inflight_goal_xy is not None:
                self._mark_visited_frontier(*self._inflight_goal_xy)
            self._inflight_goal_xy = None
            self._dbg_last_nav2_status = "SUCCEEDED"
            now_ns = self.get_clock().now().nanoseconds
            settle_ns = int(self._map_update_settle_sec * 1e9)
            self._post_arrival_settle_until_ns = now_ns + settle_ns
            self.get_logger().info(
                f"mapping_explorer: frontier goal SUCCEEDED "
                f"(goals_succeeded={self._n_goals_succeeded}, "
                f"visited={self._visited_frontier_count}); "
                f"settling {self._map_update_settle_sec:.2f}s before "
                f"querying next frontier"
            )
            return
        if status == GoalStatus.STATUS_CANCELED:
            # We canceled (operator restart/abort) — the caller already
            # transitioned the FSM. Don't recurse.
            self._inflight_goal_xy = None
            self._dbg_last_nav2_status = "CANCELED"
            return
        # ABORTED / unknown — count this attempt against the frontier
        # so frontier_explorer doesn't keep handing it to us, and so
        # frontiers that are structurally unreachable (wedged against
        # a wall, inflated out of existence) get permanently rejected
        # after _MAX_ATTEMPTS_PER_FRONTIER tries.
        if status == GoalStatus.STATUS_ABORTED:
            self._dbg_last_nav2_status = "ABORTED"
        else:
            self._dbg_last_nav2_status = f"UNKNOWN:{status}"
        if self._inflight_goal_xy is not None:
            self._record_abort(*self._inflight_goal_xy)
        self._inflight_goal_xy = None
        self._consecutive_aborts += 1
        self.get_logger().warn(
            f"mapping_explorer: frontier goal ended status={status}; "
            f"consecutive_aborts={self._consecutive_aborts}/"
            f"{self._max_aborts}. Skipping this frontier."
        )
        self._maybe_fail_after_aborts()

    # ------------------------------------------------------------------
    # Frontier-attempt accounting helpers
    # ------------------------------------------------------------------
    def _find_attempt_entry(self, wx: float, wy: float):
        """Return ``(idx, entry)`` for the existing attempt-log entry
        within ``_ABORT_BLACKLIST_RADIUS_M`` of (wx, wy), else
        ``(None, None)``. Linear scan — log size stays tiny in
        practice (one entry per "interesting" failed frontier)."""
        r2 = _ABORT_BLACKLIST_RADIUS_M ** 2
        for i, (bx, by, _, _) in enumerate(self._attempt_log):
            if (bx - wx) ** 2 + (by - wy) ** 2 < r2:
                return i, self._attempt_log[i]
        return None, None

    def _record_abort(self, wx: float, wy: float) -> None:
        """Increment the abort counter for the frontier nearest
        (wx, wy), or create a fresh entry. Logs HARD blacklist as soon
        as the count crosses _MAX_ATTEMPTS_PER_FRONTIER."""
        now_ns = self.get_clock().now().nanoseconds
        idx, entry = self._find_attempt_entry(wx, wy)
        if entry is None:
            self._attempt_log.append((wx, wy, 1, now_ns))
            self.get_logger().warn(
                f"mapping_explorer: frontier xy=({wx:.2f},{wy:.2f}) "
                f"ABORTed (attempt 1/{_MAX_ATTEMPTS_PER_FRONTIER}); "
                f"soft-skipping for {self._abort_cooldown_sec:.0f}s"
            )
            return
        bx, by, n, _ = entry
        new_n = n + 1
        # Update averaged centroid so jittered re-detections aggregate
        # toward the same logical frontier.
        avg_x = (bx * n + wx) / new_n
        avg_y = (by * n + wy) / new_n
        self._attempt_log[idx] = (avg_x, avg_y, new_n, now_ns)
        if new_n >= _MAX_ATTEMPTS_PER_FRONTIER:
            self.get_logger().warn(
                f"mapping_explorer: frontier xy=({avg_x:.2f},"
                f"{avg_y:.2f}) PERMANENTLY blacklisted after "
                f"{new_n} ABORTs — likely unreachable (wedged against "
                f"wall / costmap inflation / out-of-bounds)."
            )
        else:
            self.get_logger().warn(
                f"mapping_explorer: frontier xy=({avg_x:.2f},"
                f"{avg_y:.2f}) ABORTed (attempt {new_n}/"
                f"{_MAX_ATTEMPTS_PER_FRONTIER}); soft-skipping for "
                f"{self._abort_cooldown_sec:.0f}s"
            )

    def _is_blacklisted(self, wx: float, wy: float) -> bool:
        """Return True iff (wx, wy) is within
        ``_ABORT_BLACKLIST_RADIUS_M`` of an attempt-log entry that is
        either (a) past the hard-attempt limit, or (b) inside the soft
        cooldown window since its last ABORT."""
        idx, entry = self._find_attempt_entry(wx, wy)
        if entry is None:
            return False
        _, _, n_attempts, last_ns = entry
        if n_attempts >= _MAX_ATTEMPTS_PER_FRONTIER:
            return True  # hard-blacklist; never reconsider
        # Soft cooldown — let other frontiers run while this one rests.
        elapsed_sec = (
            self.get_clock().now().nanoseconds - last_ns
        ) / 1e9
        return elapsed_sec < self._abort_cooldown_sec

    def _maybe_fail_after_aborts(self) -> None:
        if self._consecutive_aborts >= self._max_aborts:
            self._fail(
                f"{self._max_aborts} consecutive frontier-nav failures "
                f"({self._n_goals_sent} goals sent total)"
            )

    def _cancel_nav(self, reason: str) -> None:
        if self._nav_handle is not None:
            self.get_logger().info(
                f"mapping_explorer: canceling Nav2 goal — {reason}"
            )
            try:
                self._nav_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"mapping_explorer: cancel failed: "
                    f"{type(exc).__name__}: {exc}"
                )
            self._nav_handle = None
            self._dbg_last_nav2_status = f"CANCELED:{reason}"

    # ------------------------------------------------------------------
    # Day 9+ Task 2 — visited-frontier blacklist helpers
    # ------------------------------------------------------------------
    def _mark_visited_frontier(self, wx: float, wy: float) -> None:
        """Record (wx, wy) as a visited frontier centroid.

        If a previous entry within ``visited_frontier_reject_radius_m``
        already exists, increment its visit count and refresh its
        timestamp (so the rejection window slides forward — visiting
        the same place twice doesn't *reduce* the cooldown). Otherwise
        append a new entry.
        """
        now_ns = self.get_clock().now().nanoseconds
        r2 = self._visited_reject_radius_m ** 2
        for i, (vx, vy, n, _ts) in enumerate(self._visited_frontiers):
            if (vx - wx) ** 2 + (vy - wy) ** 2 < r2:
                # Average centroid so jittered re-visits aggregate.
                avg_x = (vx * n + wx) / (n + 1)
                avg_y = (vy * n + wy) / (n + 1)
                self._visited_frontiers[i] = (
                    avg_x, avg_y, n + 1, now_ns
                )
                self._visited_frontier_count += 1
                self.get_logger().info(
                    f"mapping_explorer: VISITED frontier "
                    f"xy=({avg_x:.2f},{avg_y:.2f}) "
                    f"(visits={n + 1}, log_size="
                    f"{len(self._visited_frontiers)})"
                )
                return
        self._visited_frontiers.append((wx, wy, 1, now_ns))
        self._visited_frontier_count += 1
        self.get_logger().info(
            f"mapping_explorer: VISITED frontier "
            f"xy=({wx:.2f},{wy:.2f}) "
            f"(first visit, log_size={len(self._visited_frontiers)})"
        )

    def _is_recently_visited(self, wx: float, wy: float) -> bool:
        """True iff (wx, wy) is within ``visited_frontier_reject_radius_m``
        of an entry whose age is below ``visited_frontier_memory_sec``.

        Side-effect: prunes expired entries so the list stays bounded
        on long-running mapping sessions. Linear scan; the list rarely
        exceeds ~20 entries in practice.
        """
        if (
            self._visited_reject_radius_m <= 0.0
            or not self._visited_frontiers
        ):
            return False
        now_ns = self.get_clock().now().nanoseconds
        memory_ns = int(self._visited_memory_sec * 1e9)
        # Prune expired in-place (build a new list, cheaper than del-ing).
        if memory_ns > 0:
            fresh: List[Tuple[float, float, int, int]] = []
            for (vx, vy, n, ts) in self._visited_frontiers:
                if (now_ns - ts) <= memory_ns:
                    fresh.append((vx, vy, n, ts))
            self._visited_frontiers = fresh
        r2 = self._visited_reject_radius_m ** 2
        for (vx, vy, _n, _ts) in self._visited_frontiers:
            if (vx - wx) ** 2 + (vy - wy) ** 2 < r2:
                return True
        return False

    def _maybe_mark_arrived_by_distance(self) -> None:
        """When the robot is already within ``frontier_arrival_radius_m``
        of the in-flight goal, treat that as arrival even before Nav2
        publishes its result. Avoids the case where Nav2 stops short
        and then the FSM stalls because no SUCCEEDED arrives. The hard
        ABORT/SUCCESS callbacks remain authoritative for the FSM
        transitions; this helper only adds the centroid to the
        visited blacklist so the next /get_frontiers cycle won't
        re-pick the same point. Once a visited entry exists for the
        in-flight goal, _is_recently_visited would also reject the
        next replay — guarding against the "same goal repeats"
        symptom even when Nav2 never fires SUCCEEDED.
        """
        if self._inflight_goal_xy is None:
            return
        if self._arrival_radius_m <= 0.0:
            return
        gx, gy = self._inflight_goal_xy
        # Already on the visited list — nothing to do.
        if self._is_recently_visited(gx, gy):
            return
        pose = self._lookup_robot_pose()
        if pose is None:
            return
        rx = float(pose.pose.position.x)
        ry = float(pose.pose.position.y)
        d = math.hypot(gx - rx, gy - ry)
        if d <= self._arrival_radius_m:
            self.get_logger().info(
                f"mapping_explorer: in-flight goal "
                f"xy=({gx:.2f},{gy:.2f}) reached by distance "
                f"({d:.2f}m <= {self._arrival_radius_m:.2f}m); "
                f"adding to visited blacklist (Nav2 result still "
                f"pending — keeping goal in flight)"
            )
            self._mark_visited_frontier(gx, gy)

    # ------------------------------------------------------------------
    # FSM helpers
    # ------------------------------------------------------------------
    def _set_state(self, new_state: _State) -> None:
        if self._state == new_state:
            return
        old = self._state
        self._state = new_state
        self._state_enter_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f"mapping_explorer FSM {old.value} -> {new_state.value}"
        )
        # Reset cluster-empty timer when leaving NAVIGATING (we'll
        # re-establish it on the next empty response).
        if new_state != _State.NAVIGATING:
            self._empty_since_ns = None
        self._publish_status()

    def _fail(self, reason: str) -> None:
        if self._state == _State.FAILED:
            return
        self._failure_reason = reason
        self.get_logger().error(f"mapping_explorer FAILED: {reason}")
        self._cancel_nav("entering FAILED")
        self._set_state(_State.FAILED)

    def _publish_status(self) -> None:
        msg = String()
        if self._state == _State.FAILED and self._failure_reason:
            msg.data = f"FAILED:{self._failure_reason}"
        else:
            msg.data = self._state.value
        self._status_pub.publish(msg)

    def _publish_debug_status(self) -> None:
        """Operator-grade triage line on /mapping/debug/status (~1 Hz).

        Format is a single space-separated ``key=value`` line so it's
        easy to grep / parse from a shell. Order is deliberate:
        FSM state first, then current goal & distance (the "what is
        the robot doing right now" question), then per-cycle counters
        (the "is it making progress" question).
        """
        # ---- live distance to current goal --------------------------
        cur_goal_xy: Optional[Tuple[float, float]] = self._inflight_goal_xy
        dist_to_goal_str = "nan"
        if cur_goal_xy is not None:
            pose = self._lookup_robot_pose()
            if pose is not None:
                rx = float(pose.pose.position.x)
                ry = float(pose.pose.position.y)
                d = math.hypot(cur_goal_xy[0] - rx, cur_goal_xy[1] - ry)
                dist_to_goal_str = f"{d:.2f}"

        cur_goal_str = (
            f"({cur_goal_xy[0]:.2f},{cur_goal_xy[1]:.2f})"
            if cur_goal_xy is not None else "none"
        )
        last_goal_str = (
            f"({self._dbg_last_frontier_goal_xy[0]:.2f},"
            f"{self._dbg_last_frontier_goal_xy[1]:.2f})"
            if self._dbg_last_frontier_goal_xy is not None else "none"
        )
        sel_idx = (
            str(self._dbg_last_selected_idx)
            if self._dbg_last_selected_idx is not None else "-"
        )
        sel_score = (
            f"{self._dbg_last_selected_score:.1f}"
            if self._dbg_last_selected_score is not None else "-"
        )
        n_hard = sum(
            1 for e in self._attempt_log
            if e[2] >= _MAX_ATTEMPTS_PER_FRONTIER
        )
        in_flight = (
            "1" if (
                self._nav_handle is not None
                or (
                    self._nav_goal_send_future is not None
                    and not self._nav_goal_send_future.done()
                )
            ) else "0"
        )
        # Truncate selection_reason to keep line length sane.
        reason = (self._dbg_last_selection_reason or "-")[:60]
        msg = String()
        msg.data = (
            f"state={self._state.value} "
            f"goal={cur_goal_str} dist={dist_to_goal_str} "
            f"nav2={self._dbg_last_nav2_status or '-'} "
            f"in_flight={in_flight} "
            f"last_goal={last_goal_str} "
            f"goals_sent={self._n_goals_sent} "
            f"goals_accepted={self._dbg_n_nav2_accepted} "
            f"goals_succeeded={self._n_goals_succeeded} "
            f"aborts={self._consecutive_aborts}/{self._max_aborts} "
            f"abort_blacklist={len(self._attempt_log)}({n_hard}_hard) "
            f"visited={self._visited_frontier_count}"
            f"({len(self._visited_frontiers)}_active) "
            f"get_frontiers="
            f"{self._dbg_n_get_frontiers_success}/"
            f"{self._dbg_n_get_frontiers_failure} "
            f"frontiers={self._dbg_last_n_frontiers} "
            f"selected={sel_idx}@{sel_score} "
            f"reason={reason}"
        )
        self._debug_status_pub.publish(msg)

    def _maybe_log(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_log_ns) < self._log_period_ns:
            return
        self._last_log_ns = now_ns
        n_hard = sum(
            1 for e in self._attempt_log
            if e[2] >= _MAX_ATTEMPTS_PER_FRONTIER
        )
        self.get_logger().info(
            f"[mapping/hb] state={self._state.value} "
            f"frontier_calls={self._n_frontier_calls} "
            f"goals_sent={self._n_goals_sent} "
            f"goals_succeeded={self._n_goals_succeeded} "
            f"aborts={self._consecutive_aborts}/{self._max_aborts} "
            f"blacklisted={len(self._attempt_log)}({n_hard}_hard) "
            f"in_flight=goal:{self._nav_handle is not None} "
            f"req:{self._frontier_future is not None}"
        )

    # ------------------------------------------------------------------
    # TF — shared helper
    # ------------------------------------------------------------------
    def _lookup_robot_pose(self) -> Optional[PoseStamped]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        ps = PoseStamped()
        ps.header.stamp = tf.header.stamp
        ps.header.frame_id = self._global_frame
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        return ps


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
