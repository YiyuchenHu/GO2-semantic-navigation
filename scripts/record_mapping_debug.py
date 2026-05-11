#!/usr/bin/env python3
"""Day 9+ Phase-A — dual-rate mapping / control debug recorder.

Why two rates?
--------------
*Mapping* status (FSM state, goal, distance, navigation/arrival mirror,
TF map->base_link) changes on the order of seconds. 1 Hz is plenty for
post-mortem progress analysis ("did Go2 stay in NAVIGATING for 90 s
without ever decreasing dist_to_goal?").

*Control* topics (``/cmd_vel_nav``, ``/cmd_vel_smoothed``, ``/cmd_vel``,
``/odom``) move at 10-20 Hz, and the questions you ask of them — wave-
like driving, angular.z sign flips, zero-command stalls — *only*
make sense at >= the publish rate. 1 Hz strobing aliases sign flips and
hides oscillation. We sample these at 10 Hz by default.

What you get
------------
Two CSV files dropped into ``logs/`` (override with ``--output-dir``):

  mapping_debug_status_YYYYMMDD_HHMMSS.csv     (1 Hz)
    timestamp_iso, t_rel_sec,
    mapping_status, mapping_debug_status, task_status,
    navigation_status, arrival_status,
    target_entity_id, target_class, target_reachable,
    goal_x, goal_y,
    robot_x, robot_y, dist_to_goal,
    notes

  mapping_debug_control_YYYYMMDD_HHMMSS.csv    (10 Hz)
    timestamp_iso, t_rel_sec, mapping_state,
    nav_lin_x, nav_lin_y, nav_ang_z,
    smoothed_lin_x, smoothed_lin_y, smoothed_ang_z,
    cmd_lin_x, cmd_lin_y, cmd_ang_z,
    odom_x, odom_y, odom_lin_x, odom_ang_z,
    nav_msg_age_sec, smoothed_msg_age_sec, cmd_msg_age_sec, odom_msg_age_sec

Missing topics (e.g. /cmd_vel_nav not present because Nav2 hasn't been
brought up) are *not* a hard error: the corresponding columns are
left blank for that row and the recorder keeps running. The summary
flags any topic that produced zero messages.

Summary at end of run
---------------------
Status summary:
  * state_changes — count + ordered list (first 20)
  * goal_changes — count of unique goals seen on /semantic_goal/goal_pose
  * first/last/min dist_to_goal during NAVIGATING
  * whether dist_to_goal monotonically decreased while NAVIGATING
    (Spearman-style sign of the linear regression slope; we don't
    pull scipy in just for this — we use a tiny least-squares fit)
  * whether DONE / IDLE was reached

Control summary:
  * incoming-message rate estimate per topic (NOT the sampling rate)
  * max abs angular.z over the run
  * count of angular.z sign flips (zero-crossings outside dead-band)
  * longest zero-cmd window (sec, all 6 components below 0.001)
  * average linear.x
  * % of NAVIGATING time during which /cmd_vel was non-zero
  * heuristic flag: angular.z alternates sign quickly
    (>= 4 flips/sec in any 1 s window — wave-like driving)

Usage
-----
    bash scripts/run_mapping_debug_record.sh        # convenience wrapper, optional
    python3 scripts/record_mapping_debug.py
    python3 scripts/record_mapping_debug.py --duration-sec 300 \\
        --status-rate-hz 1 --control-rate-hz 20 --output-dir my_logs

Press Ctrl-C any time to stop early — the CSVs are flushed line-by-line
so partial captures are valid.
"""
from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
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

# Optional message types — we want the recorder to start even if the
# user hasn't built/installed go2_msgs / nav_msgs. Each import is
# guarded so a missing type just disables the column, not the script.
try:
    from go2_msgs.msg import SelectedTarget  # type: ignore
    _HAS_SELECTED_TARGET = True
except Exception as _exc:  # pragma: no cover - install-dependent
    SelectedTarget = None  # type: ignore
    _HAS_SELECTED_TARGET = False
    _SELECTED_TARGET_IMPORT_ERROR = repr(_exc)
else:
    _SELECTED_TARGET_IMPORT_ERROR = ""

try:
    from nav_msgs.msg import Odometry  # type: ignore
    _HAS_ODOMETRY = True
except Exception as _exc:  # pragma: no cover - install-dependent
    Odometry = None  # type: ignore
    _HAS_ODOMETRY = False
    _ODOMETRY_IMPORT_ERROR = repr(_exc)
else:
    _ODOMETRY_IMPORT_ERROR = ""


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------
def _status_qos() -> QoSProfile:
    """Match the TRANSIENT_LOCAL/depth=1 publishers used by
    mapping_explorer / approach_goal_planner so a recorder attached
    mid-run immediately gets the latched latest value."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


# ---------------------------------------------------------------------------
# /mapping/debug/status mini-parser
# ---------------------------------------------------------------------------
# mapping_explorer publishes a single space-separated line of
# ``key=value`` pairs, e.g.
#
#   state=NAVIGATING goal=(7.87,4.20) dist=0.43 nav2=ACCEPTED
#   in_flight=1 last_goal=(7.87,4.20) goals_sent=2 ...
#
# We only need a handful of fields downstream (in_flight, dist, goal,
# state, reason). Splitting on the first ``=`` per token is enough —
# values never contain a literal ``=``. Values that contain spaces
# (e.g. ``reason=all_frontiers_blacklisted (abort=4 visited=0)``) get
# split across multiple tokens; for our use case we only read the
# first token after the key, which is the verb / number we care
# about. Operators reading the full reason should look at the raw
# CSV cell, not this parsed view.
def _parse_mapping_debug_kv(s: Optional[str]) -> Dict[str, str]:
    """Return ``{key: value}`` for every ``k=v`` token in ``s``.

    Empty / None input yields an empty dict. Unrecognised tokens are
    silently ignored — the parser is best-effort and never raises.
    """
    out: Dict[str, str] = {}
    if not s:
        return out
    for tok in s.split():
        if "=" not in tok:
            continue
        k, _, v = tok.partition("=")
        if k and k not in out:
            out[k] = v
    return out


def _kv_int(d: Dict[str, str], key: str) -> Optional[int]:
    v = d.get(key)
    if v is None:
        return None
    try:
        return int(v)
    except ValueError:
        return None


def _kv_float(d: Dict[str, str], key: str) -> Optional[float]:
    v = d.get(key)
    if v is None or v == "nan":
        return None
    try:
        return float(v)
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# Per-topic message rate estimator
# ---------------------------------------------------------------------------
class _RateEstimator:
    """Sliding-window receive-time tracker.

    Keeps the last ``window`` epoch-seconds of arrival timestamps.
    ``rate_hz`` returns ``len(buf) / (newest - oldest)``; matches what
    ``ros2 topic hz`` reports without spawning a subprocess.
    """

    def __init__(self, window_sec: float = 5.0, max_samples: int = 256) -> None:
        self._window = float(window_sec)
        self._times: Deque[float] = deque(maxlen=max_samples)

    def stamp(self, t: float) -> None:
        self._times.append(t)
        # Drop anything older than the window.
        cutoff = t - self._window
        while self._times and self._times[0] < cutoff:
            self._times.popleft()

    @property
    def n_samples(self) -> int:
        return len(self._times)

    def rate_hz(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0.0:
            return 0.0
        return float(len(self._times) - 1) / span


# ---------------------------------------------------------------------------
# Recorder node
# ---------------------------------------------------------------------------
@dataclass
class _ControlState:
    """Latest per-topic Twist plus its arrival epoch-second time."""

    last_msg: Optional[Twist] = None
    last_t: Optional[float] = None
    rate: _RateEstimator = field(default_factory=_RateEstimator)
    n_received: int = 0


@dataclass
class _OdomState:
    last_msg: Any = None  # nav_msgs/Odometry, or None
    last_t: Optional[float] = None
    rate: _RateEstimator = field(default_factory=_RateEstimator)
    n_received: int = 0


class _MappingDebugRecorder(Node):
    """rclpy node owning all subscriptions + two sampling timers.

    The two timers are deliberately separate so missing one tick of a
    fast control sample doesn't push the slow status sample out of
    band, and so we can change ``--control-rate-hz`` without rewriting
    every column.
    """

    def __init__(
        self,
        *,
        status_rate_hz: float,
        control_rate_hz: float,
        duration_sec: float,
        output_dir: Path,
    ) -> None:
        super().__init__("mapping_debug_recorder")
        self._status_rate_hz = max(0.05, float(status_rate_hz))
        self._control_rate_hz = max(0.05, float(control_rate_hz))
        self._duration_sec = float(duration_sec)
        self._t0 = time.time()
        self._stop_at = self._t0 + self._duration_sec
        self._stopped = False

        # ---- Status caches ------------------------------------------
        self._last_mapping_status: Optional[str] = None
        self._last_mapping_debug_status: Optional[str] = None
        self._last_task_status: Optional[str] = None
        self._last_navigation_status: Optional[str] = None
        self._last_arrival_status: Optional[str] = None
        self._last_goal_pose: Optional[PoseStamped] = None
        self._last_selected: Any = None  # SelectedTarget or None

        # ---- Control caches -----------------------------------------
        self._cmd_nav = _ControlState()
        self._cmd_smoothed = _ControlState()
        self._cmd_vel = _ControlState()
        self._odom = _OdomState()

        # ---- TF -----------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._global_frame = "map"
        self._base_frame = "base_link"

        # ---- CSV files ----------------------------------------------
        output_dir.mkdir(parents=True, exist_ok=True)
        ts_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._status_csv_path = (
            output_dir / f"mapping_debug_status_{ts_stamp}.csv"
        )
        self._control_csv_path = (
            output_dir / f"mapping_debug_control_{ts_stamp}.csv"
        )
        # Open in line-buffered mode so a Ctrl-C gives a usable file
        # without flushing manually.
        self._status_fh = open(
            self._status_csv_path, "w", newline="", buffering=1
        )
        self._control_fh = open(
            self._control_csv_path, "w", newline="", buffering=1
        )
        self._status_writer = csv.writer(self._status_fh)
        self._control_writer = csv.writer(self._control_fh)
        self._status_writer.writerow([
            "timestamp_iso", "t_rel_sec",
            "mapping_status", "mapping_debug_status", "task_status",
            "navigation_status", "arrival_status",
            "target_entity_id", "target_class", "target_reachable",
            "goal_x", "goal_y",
            "robot_x", "robot_y", "dist_to_goal",
            "notes",
        ])
        self._control_writer.writerow([
            "timestamp_iso", "t_rel_sec", "mapping_state",
            "nav_lin_x", "nav_lin_y", "nav_ang_z",
            "smoothed_lin_x", "smoothed_lin_y", "smoothed_ang_z",
            "cmd_lin_x", "cmd_lin_y", "cmd_ang_z",
            "odom_x", "odom_y", "odom_lin_x", "odom_ang_z",
            "nav_msg_age_sec", "smoothed_msg_age_sec",
            "cmd_msg_age_sec", "odom_msg_age_sec",
        ])

        # ---- Live summary state -------------------------------------
        # Status side
        self._status_rows_written = 0
        self._state_changes: List[Tuple[float, str, str]] = []  # (t_rel, prev, new)
        self._goal_changes: List[Tuple[float, float, float]] = []  # (t_rel, x, y)
        self._last_state_for_change: Optional[str] = None
        self._last_goal_xy_for_change: Optional[Tuple[float, float]] = None
        self._first_dist: Optional[float] = None
        self._last_dist: Optional[float] = None
        self._min_dist: Optional[float] = None
        # (t_rel, dist) samples taken while in NAVIGATING — fed to a
        # least-squares slope estimator so the summary can answer
        # "did dist decrease?".
        self._navigating_dist_samples: List[Tuple[float, float]] = []
        self._reached_done = False
        self._reached_idle = False

        # Control side
        self._control_rows_written = 0
        self._max_abs_angular_z = 0.0
        self._sum_linear_x = 0.0
        self._n_linear_x = 0
        self._sign_flip_count = 0
        self._last_nonzero_angular_sign = 0  # +1, -1, or 0
        # Window of (t_rel, ang_sign) pairs for the "wave-like driving"
        # heuristic — we look for >= 4 flips inside any 1 s window.
        self._sign_window: Deque[Tuple[float, int]] = deque()
        self._wave_like_detected = False
        self._zero_run_start: Optional[float] = None
        self._longest_zero_run_sec = 0.0
        # Coverage during NAVIGATING: count rows where mapping_state
        # was NAVIGATING, and the subset where /cmd_vel was non-zero.
        self._navigating_rows = 0
        self._navigating_cmd_active_rows = 0

        # Day 9+ Phase B (May-9 mapping run) — second coverage view
        # gated on mapping_debug_status's ``in_flight=1`` field.
        # Why: if mapping_explorer is between goals (in_flight=0) the
        # locomotion stack legitimately holds zero cmd_vel, and the
        # plain "NAVIGATING & cmd_vel=0" coverage paints that as
        # stalling. In_flight=1 captures the strict "Nav2 has an
        # active goal RIGHT NOW" subset where cmd_vel really should
        # be non-zero.
        self._inflight_rows = 0
        self._inflight_cmd_active_rows = 0
        # Distinct zero-run tracker that only counts contiguous
        # zero-cmd samples while in_flight=1. Resets on any non-zero
        # cmd OR on in_flight transitioning to 0.
        self._zero_run_inflight_start: Optional[float] = None
        self._longest_zero_run_inflight_sec = 0.0
        # Whether we ever saw in_flight=1 / =0 / goal=none — used by
        # the summary to suppress the "stalling" verdict when the
        # whole NAVIGATING window was actually "exploration settled,
        # waiting for next frontier" rather than "stuck mid-goal".
        self._saw_inflight_1 = False
        self._navigating_inflight0_or_goal_none_rows = 0

        # Day 9+ Phase B Task 4 — distance fallback parsed from
        # mapping_debug_status's ``dist=`` field. The recorder's TF-
        # derived dist_to_goal can be NaN whenever map->base_link is
        # transiently unavailable or goal_x/goal_y haven't been
        # cached yet (status side runs at 1 Hz and TF is bursty).
        # mapping_explorer already computes the live distance for its
        # debug line, so we mirror it as a "true" distance signal and
        # report both first/last/min in the summary.
        self._dbg_dist_first: Optional[float] = None
        self._dbg_dist_last: Optional[float] = None
        self._dbg_dist_min: Optional[float] = None

        # ---- Subscriptions ------------------------------------------
        # Status topics (TRANSIENT_LOCAL where appropriate, plain
        # depth=10 elsewhere). Wrap each callback in a tiny lambda to
        # keep the wiring obvious.
        self.create_subscription(
            String, "/mapping/status",
            lambda m: self._set_str("mapping_status", m), _status_qos(),
        )
        self.create_subscription(
            String, "/mapping/debug/status",
            lambda m: self._set_str("mapping_debug_status", m),
            _status_qos(),
        )
        self.create_subscription(
            String, "/task/status",
            lambda m: self._set_str("task_status", m), 10,
        )
        self.create_subscription(
            String, "/navigation/status",
            lambda m: self._set_str("navigation_status", m), 10,
        )
        self.create_subscription(
            String, "/arrival/status",
            lambda m: self._set_str("arrival_status", m), 10,
        )
        self.create_subscription(
            PoseStamped, "/semantic_goal/goal_pose",
            self._on_goal_pose, 10,
        )
        if _HAS_SELECTED_TARGET:
            self.create_subscription(
                SelectedTarget, "/target/selected",
                self._on_selected_target, 10,
            )
        else:
            self.get_logger().warn(
                "SelectedTarget message type not importable "
                f"({_SELECTED_TARGET_IMPORT_ERROR}); "
                "/target/selected columns will be blank. "
                "Run `source install/setup.bash` after a colcon build."
            )

        # Control topics — best-effort QoS to match Nav2 publishers.
        self.create_subscription(
            Twist, "/cmd_vel_nav",
            lambda m: self._on_twist(self._cmd_nav, m),
            _best_effort_qos(),
        )
        self.create_subscription(
            Twist, "/cmd_vel_smoothed",
            lambda m: self._on_twist(self._cmd_smoothed, m),
            _best_effort_qos(),
        )
        self.create_subscription(
            Twist, "/cmd_vel",
            lambda m: self._on_twist(self._cmd_vel, m),
            _best_effort_qos(),
        )
        if _HAS_ODOMETRY:
            self.create_subscription(
                Odometry, "/odom",
                self._on_odom, _best_effort_qos(),
            )
        else:
            self.get_logger().warn(
                "nav_msgs/Odometry not importable "
                f"({_ODOMETRY_IMPORT_ERROR}); /odom columns blank."
            )

        # ---- Timers --------------------------------------------------
        self.create_timer(
            1.0 / self._status_rate_hz, self._sample_status
        )
        self.create_timer(
            1.0 / self._control_rate_hz, self._sample_control
        )

        self.get_logger().info(
            f"recorder ready. status_rate={self._status_rate_hz:.2f}Hz "
            f"control_rate={self._control_rate_hz:.2f}Hz "
            f"duration={self._duration_sec:.0f}s "
            f"status_csv={self._status_csv_path} "
            f"control_csv={self._control_csv_path}"
        )

    # ------------------------------------------------------------------
    # Per-topic callbacks
    # ------------------------------------------------------------------
    def _set_str(self, key: str, msg: String) -> None:
        data = msg.data if msg is not None else ""
        if key == "mapping_status":
            self._last_mapping_status = data
        elif key == "mapping_debug_status":
            self._last_mapping_debug_status = data
        elif key == "task_status":
            self._last_task_status = data
        elif key == "navigation_status":
            self._last_navigation_status = data
        elif key == "arrival_status":
            self._last_arrival_status = data

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self._last_goal_pose = msg

    def _on_selected_target(self, msg) -> None:
        self._last_selected = msg

    def _on_twist(self, slot: _ControlState, msg: Twist) -> None:
        now = time.time()
        slot.last_msg = msg
        slot.last_t = now
        slot.n_received += 1
        slot.rate.stamp(now)

    def _on_odom(self, msg) -> None:
        now = time.time()
        self._odom.last_msg = msg
        self._odom.last_t = now
        self._odom.n_received += 1
        self._odom.rate.stamp(now)

    # ------------------------------------------------------------------
    # Sampling timers
    # ------------------------------------------------------------------
    def _sample_status(self) -> None:
        if self._stopped:
            return
        now_wall = time.time()
        if now_wall >= self._stop_at:
            self._stopped = True
            return
        t_rel = now_wall - self._t0
        iso = datetime.now(timezone.utc).isoformat(timespec="milliseconds")

        # ---- TF lookup -----------------------------------------------
        robot_x = robot_y = ""
        notes: List[str] = []
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            robot_x = f"{float(tf.transform.translation.x):.4f}"
            robot_y = f"{float(tf.transform.translation.y):.4f}"
        except TransformException as exc:
            notes.append(f"tf_unavailable:{type(exc).__name__}")

        # ---- Goal extraction ----------------------------------------
        goal_x = goal_y = ""
        if self._last_goal_pose is not None:
            goal_x = f"{float(self._last_goal_pose.pose.position.x):.4f}"
            goal_y = f"{float(self._last_goal_pose.pose.position.y):.4f}"

        # ---- distance ------------------------------------------------
        dist_str = ""
        dist_val: Optional[float] = None
        if (
            goal_x != "" and goal_y != ""
            and robot_x != "" and robot_y != ""
        ):
            dx = float(goal_x) - float(robot_x)
            dy = float(goal_y) - float(robot_y)
            dist_val = math.hypot(dx, dy)
            dist_str = f"{dist_val:.4f}"

        # ---- /target/selected fields --------------------------------
        target_id = target_cls = target_reach = ""
        if self._last_selected is not None:
            target_id = str(getattr(self._last_selected, "entity_id", ""))
            target_cls = str(getattr(self._last_selected, "class_label", ""))
            r = getattr(self._last_selected, "reachable", None)
            target_reach = "" if r is None else ("1" if bool(r) else "0")

        mapping_status = self._last_mapping_status or ""
        mapping_dbg = self._last_mapping_debug_status or ""
        task_st = self._last_task_status or ""
        nav_st = self._last_navigation_status or ""
        arr_st = self._last_arrival_status or ""

        self._status_writer.writerow([
            iso, f"{t_rel:.3f}",
            mapping_status, mapping_dbg, task_st,
            nav_st, arr_st,
            target_id, target_cls, target_reach,
            goal_x, goal_y,
            robot_x, robot_y, dist_str,
            "|".join(notes),
        ])
        self._status_rows_written += 1

        # ---- Live summary updates -----------------------------------
        # Mapping_status changes (split FAILED:<reason> on the verb).
        verb = mapping_status.split(":", 1)[0].strip().upper()
        if verb:
            if (
                self._last_state_for_change is not None
                and verb != self._last_state_for_change
            ):
                self._state_changes.append(
                    (t_rel, self._last_state_for_change, verb)
                )
            self._last_state_for_change = verb
            if verb == "DONE":
                self._reached_done = True
            if verb == "IDLE" and self._status_rows_written > 1:
                # Boot value is IDLE; only count if we observe a
                # transition INTO it from elsewhere — captured by
                # _state_changes already, but this flag is friendlier
                # for the summary header.
                self._reached_idle = True

        # Goal changes — quantise to cm so float jitter doesn't count.
        if goal_x != "" and goal_y != "":
            qxy = (round(float(goal_x), 2), round(float(goal_y), 2))
            if (
                self._last_goal_xy_for_change is None
                or qxy != self._last_goal_xy_for_change
            ):
                self._goal_changes.append((t_rel, qxy[0], qxy[1]))
                self._last_goal_xy_for_change = qxy

        # Distance tracking. We track TF-derived dist and the
        # upstream dist (parsed from mapping_debug_status's ``dist=``)
        # SEPARATELY in the summary, but the slope-fit prefers the
        # upstream version when present (more reliable — uses the
        # FSM's own snapshot of the in-flight goal centroid, no
        # TF/goal_pose cache races on the recorder side). TF dist
        # remains the value written into the CSV's ``dist_to_goal``
        # column for backward compatibility.
        if dist_val is not None:
            if self._first_dist is None:
                self._first_dist = dist_val
            self._last_dist = dist_val
            if self._min_dist is None or dist_val < self._min_dist:
                self._min_dist = dist_val

        # Day 9+ Phase B Task 4 — fallback dist parsed from the
        # mapping_explorer-computed ``dist=`` field. Independent of
        # TF/goal_pose freshness on the recorder side.
        kv = _parse_mapping_debug_kv(mapping_dbg)
        dbg_dist = _kv_float(kv, "dist")
        if dbg_dist is not None and not math.isnan(dbg_dist):
            if self._dbg_dist_first is None:
                self._dbg_dist_first = dbg_dist
            self._dbg_dist_last = dbg_dist
            if (
                self._dbg_dist_min is None
                or dbg_dist < self._dbg_dist_min
            ):
                self._dbg_dist_min = dbg_dist

        # Slope-fit uses upstream dist when available, TF dist as
        # fallback. Never both — that'd double-count and bias the
        # least-squares.
        slope_dist: Optional[float] = (
            dbg_dist if (dbg_dist is not None and not math.isnan(dbg_dist))
            else dist_val
        )
        if (
            slope_dist is not None
            and verb == "NAVIGATING"
        ):
            self._navigating_dist_samples.append((t_rel, slope_dist))

    def _sample_control(self) -> None:
        if self._stopped:
            return
        now_wall = time.time()
        if now_wall >= self._stop_at:
            self._stopped = True
            return
        t_rel = now_wall - self._t0
        iso = datetime.now(timezone.utc).isoformat(timespec="milliseconds")

        def _twist_cells(slot: _ControlState) -> Tuple[str, str, str, str]:
            if slot.last_msg is None or slot.last_t is None:
                return ("", "", "", "")
            t = slot.last_msg
            age = now_wall - slot.last_t
            return (
                f"{float(t.linear.x):.4f}",
                f"{float(t.linear.y):.4f}",
                f"{float(t.angular.z):.4f}",
                f"{age:.3f}",
            )

        nav_lx, nav_ly, nav_az, nav_age = _twist_cells(self._cmd_nav)
        smo_lx, smo_ly, smo_az, smo_age = _twist_cells(self._cmd_smoothed)
        cmd_lx, cmd_ly, cmd_az, cmd_age = _twist_cells(self._cmd_vel)

        odom_x = odom_y = odom_lx = odom_az = odom_age = ""
        if (
            _HAS_ODOMETRY
            and self._odom.last_msg is not None
            and self._odom.last_t is not None
        ):
            odo = self._odom.last_msg
            odom_x = f"{float(odo.pose.pose.position.x):.4f}"
            odom_y = f"{float(odo.pose.pose.position.y):.4f}"
            odom_lx = f"{float(odo.twist.twist.linear.x):.4f}"
            odom_az = f"{float(odo.twist.twist.angular.z):.4f}"
            odom_age = f"{(now_wall - self._odom.last_t):.3f}"

        mapping_state_verb = (
            (self._last_mapping_status or "").split(":", 1)[0].strip().upper()
        )

        # Day 9+ Phase B Task 3 — derive in_flight from the latest
        # /mapping/debug/status. Two reasons we read it on the
        # control sampler too (10 Hz) instead of caching at the
        # status sampler (1 Hz):
        #
        #   * /mapping/debug/status is published at 1 Hz, but the
        #     recorder caches the latest message — so reading it on
        #     the 10 Hz tick still gives second-fresh values. Good
        #     enough for "is Nav2 currently driving" classification.
        #   * Keeping the control sampler self-contained means we can
        #     lower --status-rate-hz to 0.2 (one sample every 5 s)
        #     without losing the in_flight gating.
        kv_ctrl = _parse_mapping_debug_kv(self._last_mapping_debug_status)
        in_flight_str = kv_ctrl.get("in_flight", "")
        goal_str = kv_ctrl.get("goal", "")
        in_flight_now = (in_flight_str == "1")
        if in_flight_now:
            self._saw_inflight_1 = True

        self._control_writer.writerow([
            iso, f"{t_rel:.3f}", mapping_state_verb,
            nav_lx, nav_ly, nav_az,
            smo_lx, smo_ly, smo_az,
            cmd_lx, cmd_ly, cmd_az,
            odom_x, odom_y, odom_lx, odom_az,
            nav_age, smo_age, cmd_age, odom_age,
        ])
        self._control_rows_written += 1

        # ---- Live summary updates -----------------------------------
        # All summary stats below operate on /cmd_vel (the value the
        # locomotion stack actually sees). /cmd_vel_nav and
        # /cmd_vel_smoothed get their own rate counters but no
        # smoothness summary — operator can compute that from the CSV.
        cmd_msg = self._cmd_vel.last_msg
        if cmd_msg is not None:
            lx = float(cmd_msg.linear.x)
            ly = float(cmd_msg.linear.y)
            lz = float(cmd_msg.linear.z)
            ax = float(cmd_msg.angular.x)
            ay = float(cmd_msg.angular.y)
            az = float(cmd_msg.angular.z)

            self._max_abs_angular_z = max(
                self._max_abs_angular_z, abs(az)
            )
            self._sum_linear_x += lx
            self._n_linear_x += 1

            # Sign flip detection (dead-band 0.01 rad/s — ignore noise
            # near zero, only care about real direction reversals).
            sign = 0
            if az > 0.01:
                sign = +1
            elif az < -0.01:
                sign = -1
            if (
                sign != 0
                and self._last_nonzero_angular_sign != 0
                and sign != self._last_nonzero_angular_sign
            ):
                self._sign_flip_count += 1
                # Wave-like detector: keep last 1 s of sign events,
                # flag if >= 4 flips inside.
                self._sign_window.append((t_rel, sign))
                while (
                    self._sign_window
                    and (t_rel - self._sign_window[0][0]) > 1.0
                ):
                    self._sign_window.popleft()
                # Count flips inside the window
                flips_in_window = 0
                for i in range(1, len(self._sign_window)):
                    if (
                        self._sign_window[i][1]
                        != self._sign_window[i - 1][1]
                    ):
                        flips_in_window += 1
                if flips_in_window >= 4:
                    self._wave_like_detected = True
            if sign != 0:
                self._last_nonzero_angular_sign = sign

            # Zero-cmd window tracker. Two flavours:
            #   * overall — runs across the whole capture, useful for
            #     spotting stalls regardless of FSM state.
            #   * in_flight=1 only — the strict "Nav2 has an active
            #     goal *right now*" subset where cmd_vel really
            #     should be non-zero.
            zero = (
                abs(lx) < 0.001 and abs(ly) < 0.001 and abs(lz) < 0.001
                and abs(ax) < 0.001 and abs(ay) < 0.001 and abs(az) < 0.001
            )
            if zero:
                if self._zero_run_start is None:
                    self._zero_run_start = t_rel
                run_len = t_rel - self._zero_run_start
                if run_len > self._longest_zero_run_sec:
                    self._longest_zero_run_sec = run_len
            else:
                self._zero_run_start = None

            if in_flight_now and zero:
                if self._zero_run_inflight_start is None:
                    self._zero_run_inflight_start = t_rel
                run_len = t_rel - self._zero_run_inflight_start
                if run_len > self._longest_zero_run_inflight_sec:
                    self._longest_zero_run_inflight_sec = run_len
            else:
                # Either non-zero cmd OR in_flight=0 breaks the run.
                self._zero_run_inflight_start = None

            if mapping_state_verb == "NAVIGATING":
                self._navigating_rows += 1
                if not zero:
                    self._navigating_cmd_active_rows += 1
                # Track NAVIGATING rows where in_flight=0 OR goal=none
                # — these are "exploration settled, waiting on next
                # frontier" rows and SHOULD have zero cmd_vel.
                if (not in_flight_now) or (goal_str == "none"):
                    self._navigating_inflight0_or_goal_none_rows += 1

            if in_flight_now:
                self._inflight_rows += 1
                if not zero:
                    self._inflight_cmd_active_rows += 1
        else:
            # No /cmd_vel message yet; still account for NAVIGATING /
            # in_flight rows (so the active ratios are honest even
            # when /cmd_vel never published).
            if mapping_state_verb == "NAVIGATING":
                self._navigating_rows += 1
                if (not in_flight_now) or (goal_str == "none"):
                    self._navigating_inflight0_or_goal_none_rows += 1
            if in_flight_now:
                self._inflight_rows += 1

    # ------------------------------------------------------------------
    # Summary printer
    # ------------------------------------------------------------------
    def _print_summary(self) -> None:
        bar = "=" * 68

        def navigating_slope() -> Optional[float]:
            """Tiny least-squares slope of dist_to_goal vs t over
            samples taken in NAVIGATING. None if too few samples
            (need at least 3 for a meaningful direction)."""
            samples = self._navigating_dist_samples
            n = len(samples)
            if n < 3:
                return None
            sx = sum(t for t, _ in samples)
            sy = sum(d for _, d in samples)
            sxx = sum(t * t for t, _ in samples)
            sxy = sum(t * d for t, d in samples)
            denom = n * sxx - sx * sx
            if abs(denom) < 1e-9:
                return None
            return (n * sxy - sx * sy) / denom

        slope = navigating_slope()

        # cmd_vel rate during NAVIGATING (from active rows):
        cmd_active_pct: Optional[float] = None
        if self._navigating_rows > 0:
            cmd_active_pct = (
                100.0
                * self._navigating_cmd_active_rows
                / self._navigating_rows
            )

        avg_lx: Optional[float] = None
        if self._n_linear_x > 0:
            avg_lx = self._sum_linear_x / self._n_linear_x

        print()
        print(bar)
        print("Recorder summary")
        print(bar)
        print(f"  status_csv : {self._status_csv_path}")
        print(f"  control_csv: {self._control_csv_path}")
        print(
            f"  rows: status={self._status_rows_written} "
            f"control={self._control_rows_written}"
        )
        print(
            f"  duration  : {time.time() - self._t0:.1f}s "
            f"(target {self._duration_sec:.0f}s)"
        )

        # ---- Status summary -----------------------------------------
        print()
        print("Status summary")
        print(f"  state_changes      : {len(self._state_changes)}")
        for i, (t, prev, new) in enumerate(self._state_changes[:20]):
            print(f"      [{i:02d}] t={t:6.1f}s  {prev} -> {new}")
        if len(self._state_changes) > 20:
            print(f"      ... ({len(self._state_changes) - 20} more)")
        print(f"  goal_changes       : {len(self._goal_changes)}")
        for i, (t, gx, gy) in enumerate(self._goal_changes[:8]):
            print(f"      [{i:02d}] t={t:6.1f}s  goal=({gx:.2f},{gy:.2f})")
        if len(self._goal_changes) > 8:
            print(f"      ... ({len(self._goal_changes) - 8} more)")
        # Day 9+ Phase B Task 4 — report both distance signals.
        #   mapping_debug_dist : taken from mapping_explorer's own
        #       /mapping/debug/status (more reliable; no TF cache
        #       races on the recorder side).
        #   recorder_tf_dist   : computed by the recorder from TF
        #       (map -> base_link) and the cached /semantic_goal/
        #       goal_pose. Useful for cross-checking, NaN-prone
        #       early in a run while TF warms up.
        def _fmt(v: Optional[float]) -> str:
            return "n/a" if v is None else f"{v:.2f}m"

        print(
            f"  mapping_debug_dist : first={_fmt(self._dbg_dist_first)} "
            f"last={_fmt(self._dbg_dist_last)} "
            f"min={_fmt(self._dbg_dist_min)}"
        )
        print(
            f"  recorder_tf_dist   : first={_fmt(self._first_dist)} "
            f"last={_fmt(self._last_dist)} "
            f"min={_fmt(self._min_dist)}"
        )
        if slope is None:
            print(
                "  dist_decreasing    : insufficient NAVIGATING samples"
            )
        else:
            verdict = "yes" if slope < -0.01 else (
                "no (rising)" if slope > 0.01 else "flat"
            )
            print(
                f"  dist_decreasing    : {verdict} "
                f"(slope={slope:.4f} m/s over "
                f"{len(self._navigating_dist_samples)} NAVIGATING samples)"
            )
        print(
            f"  reached_DONE       : {'yes' if self._reached_done else 'no'}"
        )
        print(
            f"  reached_IDLE       : "
            f"{'yes' if self._reached_idle else 'no'}"
        )

        # ---- Control summary ----------------------------------------
        print()
        print("Control summary")

        def rate_line(label: str, slot: _ControlState) -> str:
            if slot.n_received == 0:
                return f"{label:<20}: silent (0 msgs received)"
            return (
                f"{label:<20}: ~{slot.rate.rate_hz():.2f} Hz "
                f"(n={slot.n_received})"
            )

        print(f"  {rate_line('cmd_vel_nav', self._cmd_nav)}")
        print(f"  {rate_line('cmd_vel_smoothed', self._cmd_smoothed)}")
        print(f"  {rate_line('cmd_vel', self._cmd_vel)}")
        if _HAS_ODOMETRY:
            if self._odom.n_received == 0:
                print("  odom                : silent (0 msgs received)")
            else:
                print(
                    f"  odom                : ~{self._odom.rate.rate_hz():.2f} Hz "
                    f"(n={self._odom.n_received})"
                )
        else:
            print("  odom                : nav_msgs not importable")
        print(
            f"  max |angular.z|     : {self._max_abs_angular_z:.3f} rad/s"
        )
        print(
            f"  angular.z sign flips: {self._sign_flip_count}"
        )
        print(
            f"  longest zero-cmd run: {self._longest_zero_run_sec:.2f} s"
        )
        print(
            f"  longest zero (in_flight=1): "
            f"{self._longest_zero_run_inflight_sec:.2f} s"
        )
        if avg_lx is None:
            print("  avg linear.x        : n/a (no /cmd_vel samples)")
        else:
            print(f"  avg linear.x        : {avg_lx:.3f} m/s")

        # Day 9+ Phase B Task 3 — two coverage views.
        #
        # First, the LOOSE view: cmd_vel non-zero ratio across all
        # NAVIGATING rows. This is what the previous version already
        # reported. Now we explicitly *don't* call this "stalling" if
        # most of the NAVIGATING rows actually had in_flight=0 or
        # goal=none — that's "between goals, exploration just
        # finished" and zero cmd_vel is correct.
        if cmd_active_pct is None:
            print("  cmd_vel during NAV  : no NAVIGATING rows seen")
        else:
            settled_rows = self._navigating_inflight0_or_goal_none_rows
            settled_frac = (
                settled_rows / self._navigating_rows
                if self._navigating_rows > 0 else 0.0
            )
            if cmd_active_pct >= 50.0:
                verdict = "OK"
            elif settled_frac >= 0.5:
                # Most of NAVIGATING was "in_flight=0 / goal=none"
                # — exploration was settling between goals, low
                # cmd_vel is correct, NOT stalling.
                verdict = (
                    "OK (exploration settled between goals — "
                    f"{settled_frac * 100:.0f}% of NAVIGATING rows had "
                    "in_flight=0 or goal=none)"
                )
            else:
                verdict = (
                    "stalling — cmd_vel is zero >50% of NAVIGATING time"
                )
            print(
                f"  cmd_vel during NAV  : {cmd_active_pct:.1f}% active "
                f"({self._navigating_cmd_active_rows}/"
                f"{self._navigating_rows} rows) — {verdict}"
            )

        # STRICT view: only count rows where mapping_explorer says
        # in_flight=1. This is the right denominator for "is the
        # locomotion stack ignoring active goals?".
        if not self._saw_inflight_1:
            print(
                "  cmd_vel during in_flight=1: never observed "
                "in_flight=1 (Nav2 had no active goals during run)"
            )
        elif self._inflight_rows == 0:
            print("  cmd_vel during in_flight=1: 0 rows captured")
        else:
            inflight_pct = (
                100.0
                * self._inflight_cmd_active_rows
                / self._inflight_rows
            )
            verdict = (
                "OK" if inflight_pct >= 50.0 else
                "stalling — cmd_vel is zero >50% of in_flight=1 time"
            )
            print(
                f"  cmd_vel during in_flight=1: {inflight_pct:.1f}% "
                f"active ({self._inflight_cmd_active_rows}/"
                f"{self._inflight_rows} rows) — {verdict}"
            )

        print(
            "  wave-like driving   : "
            f"{'YES' if self._wave_like_detected else 'no'} "
            "(>=4 angular.z sign flips inside any 1 s window)"
        )

        print()
        print(bar)
        print("Done. Inspect the CSVs with e.g. pandas:")
        print(
            f"    df = pandas.read_csv({str(self._status_csv_path)!r})"
        )
        print(
            f"    df = pandas.read_csv({str(self._control_csv_path)!r})"
        )
        print(bar)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def expired(self) -> bool:
        return self._stopped or time.time() >= self._stop_at

    def shutdown(self) -> None:
        try:
            self._status_fh.close()
        except Exception:
            pass
        try:
            self._control_fh.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="record_mapping_debug.py",
        description=(
            "Dual-rate recorder for the GO2 mapping/control debug feed. "
            "Status topics sample at --status-rate-hz (default 1 Hz); "
            "control topics sample at --control-rate-hz (default 10 Hz)."
        ),
    )
    p.add_argument(
        "--duration-sec", type=float, default=120.0,
        help="Total recording duration in seconds (default: 120).",
    )
    p.add_argument(
        "--status-rate-hz", type=float, default=1.0,
        help="Sampling rate for the status CSV (default: 1.0).",
    )
    p.add_argument(
        "--control-rate-hz", type=float, default=10.0,
        help="Sampling rate for the control CSV (default: 10.0).",
    )
    p.add_argument(
        "--output-dir", type=Path, default=Path("logs"),
        help="Directory to write the two CSV files into (default: logs).",
    )
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = _MappingDebugRecorder(
        status_rate_hz=args.status_rate_hz,
        control_rate_hz=args.control_rate_hz,
        duration_sec=args.duration_sec,
        output_dir=args.output_dir,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Cooperative shutdown on Ctrl-C — give the timers a chance to
    # flush their last row before we close the file handles.
    interrupted = {"flag": False}

    def _on_sigint(_sig, _frm) -> None:
        interrupted["flag"] = True

    signal.signal(signal.SIGINT, _on_sigint)
    signal.signal(signal.SIGTERM, _on_sigint)

    try:
        while not interrupted["flag"] and not node.expired():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node._print_summary()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
