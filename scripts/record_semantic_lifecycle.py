#!/usr/bin/env python3
"""Day 9+ Phase-D — semantic marker lifecycle recorder.

Why this exists
---------------
``record_mapping_debug.py`` covers the *navigation* side (mapping
state, frontier exploration, cmd_vel smoothness). The other half of
the project — "did Go2 actually *remember* the table / person?" —
needs its own continuous trace because the answer to "why is the
marker slow to appear?" depends on **multiple** layers all running
at once:

  1. YOLOE 2D detection rate / score (`/detections`,
     `/detections/masks`)
  2. Depth projector mask + bbox path success
     (`/detections_3d`, `/depth_projector/debug_stats`)
  3. semantic_memory_aggregator candidate vs confirmed transition
     (`/semantic_map/objects`,
     `/semantic_map/anchor_debug_stats`)
  4. The mode the operator was in at that moment — autonomous
     mapping, autonomous semantic-goal navigation, or manual teleop.

A single ``ros2 topic echo`` only gives you (1) snapshot per layer.
This recorder samples them all on a configurable schedule, writes
three CSVs, and produces a post-run summary that answers the
specific operator questions ("was the table only confirmed when I
manually pointed at it?" / "did the table get stuck at 2D → 3D?" /
"how many TF failures masked the table?").

Three CSV outputs
-----------------
``logs/semantic_lifecycle_perception_YYYYMMDD_HHMMSS.csv`` (5 Hz)
    YOLOE / depth_projector layer. One row per perception tick.

``logs/semantic_lifecycle_entities_YYYYMMDD_HHMMSS.csv`` (1 Hz)
    Per-entity snapshot of the semantic memory state. **Multiple
    rows per tick** — one per current entity in
    ``/semantic_map/objects``. Empty ticks (no entities yet) emit
    one synthetic ``entity_id=`` row so the timeline is still
    contiguous.

``logs/semantic_lifecycle_status_YYYYMMDD_HHMMSS.csv`` (1 Hz)
    Overall context: mapping state, navigation state, manual
    teleop / cmd_vel activity, current operator mode. Used to
    correlate "this entity was first observed during
    AUTO_MAPPING" vs "this one only during MANUAL_TELEOP".

Plus ``logs/semantic_lifecycle_YYYYMMDD_HHMMSS.summary.txt`` with
the post-run analysis (first-time milestones, failure counts,
mode-aware warnings).

Important
---------
This is a **pure data collector**. It does NOT:
  * publish to any topic except ``/semantic_recording/mode``
    (read-only by default — we only subscribe).
  * change perception, semantic-memory, mapping, or Nav2 behaviour.
  * decide whether a marker is good or bad — that's a downstream
    analysis decision based on the CSVs.

Topics that aren't published yet (e.g. ``/semantic_map/anchor_debug_stats``
on a stack predating Phase B) leave the corresponding columns blank
and the recorder keeps running. The summary lists every silent
topic so the operator notices.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection2DArray, Detection3DArray

# Optional message-type imports — keep the recorder usable even if
# go2_msgs hasn't been built into the current overlay yet. Each
# missing type just disables the corresponding columns / rows.
try:
    from go2_msgs.msg import (  # type: ignore
        InstanceMaskArray,
        SemanticEntity,
        SemanticEntityArray,
    )
    _HAS_GO2_MSGS = True
    _GO2_MSGS_IMPORT_ERROR = ""
except Exception as _exc:  # pragma: no cover - install-dependent
    InstanceMaskArray = None  # type: ignore
    SemanticEntity = None  # type: ignore
    SemanticEntityArray = None  # type: ignore
    _HAS_GO2_MSGS = False
    _GO2_MSGS_IMPORT_ERROR = repr(_exc)


# ---------------------------------------------------------------------------
# Constants & helpers
# ---------------------------------------------------------------------------
TABLE_LIKE_LABELS = {"table", "desk", "dining table", "dining_table",
                     "workbench", "office desk", "office_desk"}
PERSON_LIKE_LABELS = {"person", "human", "man", "woman", "people",
                      "pedestrian", "worker", "construction_worker"}


def _is_table_like(label: str) -> bool:
    norm = (label or "").strip().lower()
    if not norm:
        return False
    if norm in TABLE_LIKE_LABELS:
        return True
    norm_us = norm.replace(" ", "_")
    norm_sp = norm.replace("_", " ")
    return norm_us in TABLE_LIKE_LABELS or norm_sp in TABLE_LIKE_LABELS


def _is_person_like(label: str) -> bool:
    norm = (label or "").strip().lower()
    return bool(norm) and norm in PERSON_LIKE_LABELS


def _status_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _parse_kv_string(s: Optional[str]) -> Dict[str, str]:
    """``key=value`` parser — same shape as
    ``record_mapping_debug._parse_mapping_debug_kv``. Multi-word
    values are dropped after the first whitespace; the operator can
    consult the raw cell for the full text.
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


# Pattern matching the display_name format from
# semantic_memory_aggregator_node._render_display_name. The
# canonical layout is "<raw_label>|<status>|<anchor_id>" — e.g.
# "person|confirmed|pc_+0123_-0042" or "desk|candidate|-".
# Older builds may emit a 2-segment "<raw_label>|<status>" with no
# anchor field; the regex below tolerates that by treating anchor
# as optional.
_DISPLAY_RE = re.compile(
    r"^(?P<raw>[^|]*)\|(?P<status>[^|]*)(?:\|(?P<anchor>[^|]*))?$"
)


def _parse_display_name(display: str) -> Tuple[str, str, str]:
    """Return ``(raw_label, status, anchor_id)``. Missing fields
    surface as the empty string so CSV cells are uniform.
    """
    if not display:
        return "", "", ""
    m = _DISPLAY_RE.match(display.strip())
    if m is None:
        return display.strip(), "", ""
    return (m.group("raw") or "").strip(), \
           (m.group("status") or "").strip(), \
           (m.group("anchor") or "").strip()


# ---------------------------------------------------------------------------
# Mode inference
# ---------------------------------------------------------------------------
# AUTO_MAPPING / AUTO_SEMANTIC_NAV / MANUAL_TELEOP / IDLE / UNKNOWN.
# A simple precedence chain — semantic-nav wins over mapping wins over
# teleop, because the semantic-goal flow OVERRIDES the mapping state
# (the action-debug topic only fires while a semantic goal is in
# flight). Manual teleop is the catch-all for "operator pushed a
# stick / pressed a key but no autonomous mode is running."
def _infer_mode(
    *,
    operator_mode_raw: str,
    mapping_status: str,
    mapping_debug_kv: Dict[str, str],
    action_debug: str,
    cmd_vel_active: bool,
) -> str:
    op = (operator_mode_raw or "").strip().lower()
    # Operator-supplied label always wins — this is the manual
    # override the user emits via ``ros2 topic pub --once
    # /semantic_recording/mode std_msgs/String "data: 'manual_table_scan'"``.
    if "manual" in op:
        return "MANUAL_TELEOP"
    if op.startswith("auto_mapping"):
        return "AUTO_MAPPING"
    if op.startswith("auto_semantic_nav"):
        return "AUTO_SEMANTIC_NAV"

    ad = (action_debug or "").strip().upper()
    if any(tok in ad for tok in (" SEND", "ACCEPTED", "IN_FLIGHT")):
        return "AUTO_SEMANTIC_NAV"

    in_flight = mapping_debug_kv.get("in_flight", "")
    ms = (mapping_status or "").strip().upper()
    if ms == "NAVIGATING" or in_flight == "1":
        return "AUTO_MAPPING"

    if cmd_vel_active:
        return "MANUAL_TELEOP"

    return "IDLE"


# ---------------------------------------------------------------------------
# Per-topic latest-state caches
# ---------------------------------------------------------------------------
@dataclass
class _StringCache:
    """Cache the latest String message + arrival epoch-second."""
    last: Optional[str] = None
    last_t: Optional[float] = None
    n_received: int = 0


@dataclass
class _TwistCache:
    last: Optional[Twist] = None
    last_t: Optional[float] = None
    n_received: int = 0


@dataclass
class _PoseCache:
    last: Optional[PoseStamped] = None
    last_t: Optional[float] = None
    n_received: int = 0


@dataclass
class _Detection2DCache:
    """Latest /detections snapshot. We don't keep the whole buffer
    because only the most recent frame matters for the CSV row."""
    last: Optional[Detection2DArray] = None
    last_t: Optional[float] = None
    n_received: int = 0
    # First-time milestones — set once and never cleared.
    first_table_like_t: Optional[float] = None
    first_person_t: Optional[float] = None
    best_table_score_seen: float = 0.0
    best_table_label_seen: str = ""
    best_person_score_seen: float = 0.0


@dataclass
class _MaskCache:
    last: Any = None  # InstanceMaskArray or None
    last_t: Optional[float] = None
    n_received: int = 0
    first_table_like_t: Optional[float] = None
    first_person_t: Optional[float] = None
    best_table_score_seen: float = 0.0
    best_table_label_seen: str = ""
    best_person_score_seen: float = 0.0


@dataclass
class _Detection3DCache:
    last: Optional[Detection3DArray] = None
    last_t: Optional[float] = None
    n_received: int = 0
    first_table_t: Optional[float] = None
    first_person_t: Optional[float] = None
    best_table_score_seen: float = 0.0
    best_person_score_seen: float = 0.0


@dataclass
class _SemanticEntitiesCache:
    last: Any = None  # SemanticEntityArray or None
    last_t: Optional[float] = None
    n_received: int = 0
    # First-time milestones, indexed by canonical class.
    first_seen_t_by_class: Dict[str, float] = field(default_factory=dict)
    first_confirmed_t_by_class: Dict[str, float] = field(
        default_factory=dict
    )
    # Has the entity ever carried a pc_ anchor by class?
    pc_anchor_seen_by_class: Dict[str, bool] = field(default_factory=dict)
    # Mode at which the entity was first confirmed — used by the
    # summary to flag "person confirmed only during MANUAL_TELEOP".
    first_confirmed_mode_by_class: Dict[str, str] = field(
        default_factory=dict
    )


# ---------------------------------------------------------------------------
# Recorder node
# ---------------------------------------------------------------------------
class _SemanticLifecycleRecorder(Node):
    """Multi-rate data collector for the semantic perception →
    memory pipeline. See module docstring for design rationale.
    """

    def __init__(
        self,
        *,
        duration_sec: float,
        status_rate_hz: float,
        perception_rate_hz: float,
        output_dir: Path,
        target_class: str,
        print_live: bool,
    ) -> None:
        super().__init__("semantic_lifecycle_recorder")

        self._duration_sec = float(duration_sec)
        self._status_period = (
            1.0 / max(status_rate_hz, 0.01) if status_rate_hz > 0 else 1.0
        )
        self._perception_period = (
            1.0 / max(perception_rate_hz, 0.01)
            if perception_rate_hz > 0 else 0.2
        )
        self._target_class = (target_class or "").strip().lower()
        self._print_live = bool(print_live)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir.mkdir(parents=True, exist_ok=True)
        self._status_csv_path = output_dir / (
            f"semantic_lifecycle_status_{ts}.csv"
        )
        self._perception_csv_path = output_dir / (
            f"semantic_lifecycle_perception_{ts}.csv"
        )
        self._entities_csv_path = output_dir / (
            f"semantic_lifecycle_entities_{ts}.csv"
        )
        self._summary_path = output_dir / (
            f"semantic_lifecycle_{ts}.summary.txt"
        )

        # ---- per-topic state ------------------------------------
        self._mode_cache = _StringCache()
        self._mapping_status = _StringCache()
        self._mapping_debug = _StringCache()
        self._task_status = _StringCache()
        self._navigation_status = _StringCache()
        self._arrival_status = _StringCache()
        self._action_debug = _StringCache()
        self._depth_projector_stats = _StringCache()
        self._anchor_debug_stats = _StringCache()
        self._island_markers = _StringCache()  # we only count rx; no body

        self._cmd_vel_nav = _TwistCache()
        self._cmd_vel_smoothed = _TwistCache()
        self._cmd_vel = _TwistCache()

        self._detections = _Detection2DCache()
        self._masks = _MaskCache()
        self._detections_3d = _Detection3DCache()
        self._entities = _SemanticEntitiesCache()

        # ---- ROS infra ------------------------------------------
        # Status topics (latched). Use TRANSIENT_LOCAL so a recorder
        # joining mid-run still sees the last published value.
        self.create_subscription(
            String, "/mapping/status",
            lambda m: self._on_string(self._mapping_status, m),
            _status_qos(),
        )
        self.create_subscription(
            String, "/mapping/debug/status",
            lambda m: self._on_string(self._mapping_debug, m),
            _status_qos(),
        )
        self.create_subscription(
            String, "/task/status",
            lambda m: self._on_string(self._task_status, m),
            _status_qos(),
        )
        self.create_subscription(
            String, "/navigation/status",
            lambda m: self._on_string(self._navigation_status, m),
            _status_qos(),
        )
        self.create_subscription(
            String, "/arrival/status",
            lambda m: self._on_string(self._arrival_status, m),
            _status_qos(),
        )
        self.create_subscription(
            String, "/semantic_goal/action_debug",
            lambda m: self._on_string(self._action_debug, m),
            _reliable_qos(),
        )
        self.create_subscription(
            String, "/depth_projector/debug_stats",
            lambda m: self._on_string(self._depth_projector_stats, m),
            _reliable_qos(),
        )
        self.create_subscription(
            String, "/semantic_map/anchor_debug_stats",
            lambda m: self._on_string(self._anchor_debug_stats, m),
            _reliable_qos(),
        )
        self.create_subscription(
            String, "/semantic_recording/mode",
            lambda m: self._on_string(self._mode_cache, m),
            _reliable_qos(),
        )

        # Markers — we only count receipt (the body isn't useful in
        # CSV form). Keep a String-style cache for n_received.
        self.create_subscription(
            MarkerArray, "/semantic_map/island_debug_markers",
            lambda m: self._on_marker_array(m),
            _best_effort_qos(),
        )

        # cmd_vel triplet — best-effort to avoid backpressure.
        self.create_subscription(
            Twist, "/cmd_vel_nav",
            lambda m: self._on_twist(self._cmd_vel_nav, m),
            _best_effort_qos(),
        )
        self.create_subscription(
            Twist, "/cmd_vel_smoothed",
            lambda m: self._on_twist(self._cmd_vel_smoothed, m),
            _best_effort_qos(),
        )
        self.create_subscription(
            Twist, "/cmd_vel",
            lambda m: self._on_twist(self._cmd_vel, m),
            _best_effort_qos(),
        )

        # Perception streams.
        self.create_subscription(
            Detection2DArray, "/detections",
            self._on_detections, _reliable_qos(),
        )
        self.create_subscription(
            Detection3DArray, "/detections_3d",
            self._on_detections_3d, _reliable_qos(),
        )
        if _HAS_GO2_MSGS:
            self.create_subscription(
                InstanceMaskArray, "/detections/masks",
                self._on_masks, _reliable_qos(),
            )
            self.create_subscription(
                SemanticEntityArray, "/semantic_map/objects",
                self._on_entities, _reliable_qos(),
            )
        else:
            self.get_logger().warn(
                "go2_msgs import failed (%s); /detections/masks and "
                "/semantic_map/objects columns will be blank."
                % _GO2_MSGS_IMPORT_ERROR
            )

        # ---- CSV files ------------------------------------------
        self._status_csv_f = self._status_csv_path.open("w", newline="")
        self._perception_csv_f = self._perception_csv_path.open(
            "w", newline=""
        )
        self._entities_csv_f = self._entities_csv_path.open(
            "w", newline=""
        )
        self._status_writer = csv.writer(self._status_csv_f)
        self._perception_writer = csv.writer(self._perception_csv_f)
        self._entities_writer = csv.writer(self._entities_csv_f)

        self._status_writer.writerow(self._status_header())
        self._perception_writer.writerow(self._perception_header())
        self._entities_writer.writerow(self._entities_header())

        # ---- timers ---------------------------------------------
        self._t_start = time.time()
        self._n_status_rows = 0
        self._n_perception_rows = 0
        self._n_entity_rows = 0
        # Mode duration accounting — we increment whichever bin the
        # current sample lands in by the status period. Cheap; gives
        # the summary the AUTO_MAPPING/AUTO_SEMANTIC_NAV/etc.
        # breakdown without re-reading the CSV.
        self._mode_seconds: Dict[str, float] = {
            "AUTO_MAPPING": 0.0,
            "AUTO_SEMANTIC_NAV": 0.0,
            "MANUAL_TELEOP": 0.0,
            "IDLE": 0.0,
            "UNKNOWN": 0.0,
        }
        self._first_seen_anchor_by_eid: Dict[str, str] = {}

        self._status_timer = self.create_timer(
            self._status_period, self._sample_status,
        )
        self._perception_timer = self.create_timer(
            self._perception_period, self._sample_perception,
        )
        self._entities_timer = self.create_timer(
            self._status_period, self._sample_entities,
        )
        # Hard stop timer so this script terminates even if the
        # operator backgrounds it. ``_finish`` is also called from
        # ``main`` when the spin loop returns due to KeyboardInterrupt.
        self._shutdown_timer = self.create_timer(
            self._duration_sec, self._on_duration_elapsed,
        )

        self.get_logger().info(
            f"semantic_lifecycle_recorder started. duration={self._duration_sec:.0f}s "
            f"status_period={self._status_period:.2f}s "
            f"perception_period={self._perception_period:.2f}s "
            f"target_class={self._target_class!r} go2_msgs={_HAS_GO2_MSGS}"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_string(self, cache: _StringCache, msg: String) -> None:
        cache.last = str(msg.data)
        cache.last_t = time.time()
        cache.n_received += 1

    def _on_marker_array(self, msg: MarkerArray) -> None:
        self._island_markers.last = f"n_markers={len(msg.markers)}"
        self._island_markers.last_t = time.time()
        self._island_markers.n_received += 1

    def _on_twist(self, cache: _TwistCache, msg: Twist) -> None:
        cache.last = msg
        cache.last_t = time.time()
        cache.n_received += 1

    def _on_detections(self, msg: Detection2DArray) -> None:
        now = time.time()
        self._detections.last = msg
        self._detections.last_t = now
        self._detections.n_received += 1
        # First-time milestones — track the earliest time we ever
        # see a person- or table-like class with score > 0.
        for d in msg.detections:
            if not d.results:
                continue
            label = str(d.results[0].hypothesis.class_id)
            score = float(d.results[0].hypothesis.score)
            if score <= 0.0:
                continue
            if _is_table_like(label):
                if self._detections.first_table_like_t is None:
                    self._detections.first_table_like_t = now
                if score > self._detections.best_table_score_seen:
                    self._detections.best_table_score_seen = score
                    self._detections.best_table_label_seen = label
            elif _is_person_like(label):
                if self._detections.first_person_t is None:
                    self._detections.first_person_t = now
                if score > self._detections.best_person_score_seen:
                    self._detections.best_person_score_seen = score

    def _on_masks(self, msg: Any) -> None:  # InstanceMaskArray
        now = time.time()
        self._masks.last = msg
        self._masks.last_t = now
        self._masks.n_received += 1
        for m in getattr(msg, "masks", []) or []:
            label = str(getattr(m, "class_label", ""))
            score = float(getattr(m, "score", 0.0) or 0.0)
            if score <= 0.0:
                continue
            if _is_table_like(label):
                if self._masks.first_table_like_t is None:
                    self._masks.first_table_like_t = now
                if score > self._masks.best_table_score_seen:
                    self._masks.best_table_score_seen = score
                    self._masks.best_table_label_seen = label
            elif _is_person_like(label):
                if self._masks.first_person_t is None:
                    self._masks.first_person_t = now
                if score > self._masks.best_person_score_seen:
                    self._masks.best_person_score_seen = score

    def _on_detections_3d(self, msg: Detection3DArray) -> None:
        now = time.time()
        self._detections_3d.last = msg
        self._detections_3d.last_t = now
        self._detections_3d.n_received += 1
        for d in msg.detections:
            if not d.results:
                continue
            label = str(d.results[0].hypothesis.class_id)
            score = float(d.results[0].hypothesis.score)
            if score <= 0.0:
                continue
            if _is_table_like(label):
                if self._detections_3d.first_table_t is None:
                    self._detections_3d.first_table_t = now
                if score > self._detections_3d.best_table_score_seen:
                    self._detections_3d.best_table_score_seen = score
            elif _is_person_like(label):
                if self._detections_3d.first_person_t is None:
                    self._detections_3d.first_person_t = now
                if score > self._detections_3d.best_person_score_seen:
                    self._detections_3d.best_person_score_seen = score

    def _on_entities(self, msg: Any) -> None:  # SemanticEntityArray
        now = time.time()
        self._entities.last = msg
        self._entities.last_t = now
        self._entities.n_received += 1
        # Compute mode now so the milestone bin matches the rest of
        # the row's interpretation. We deliberately re-derive rather
        # than read the cached _last_status_mode to avoid skew.
        cur_mode = self._current_mode()
        for ent in getattr(msg, "entities", []) or []:
            cls = str(getattr(ent, "class_label", "")).strip().lower()
            display = str(getattr(ent, "display_name", ""))
            _, status, anchor = _parse_display_name(display)
            status_norm = status.strip().lower()
            if cls and cls not in self._entities.first_seen_t_by_class:
                self._entities.first_seen_t_by_class[cls] = now
            if cls and status_norm == "confirmed" and \
                    cls not in self._entities.first_confirmed_t_by_class:
                self._entities.first_confirmed_t_by_class[cls] = now
                self._entities.first_confirmed_mode_by_class[cls] = cur_mode
            if cls and anchor.startswith("pc_"):
                self._entities.pc_anchor_seen_by_class[cls] = True

    # ------------------------------------------------------------------
    # Mode helper — used by entity callback AND by the row writers
    # ------------------------------------------------------------------
    def _current_mode(self) -> str:
        cmd = self._cmd_vel.last
        cmd_active = self._twist_active(cmd)
        return _infer_mode(
            operator_mode_raw=self._mode_cache.last or "",
            mapping_status=self._mapping_status.last or "",
            mapping_debug_kv=_parse_kv_string(self._mapping_debug.last),
            action_debug=self._action_debug.last or "",
            cmd_vel_active=cmd_active,
        )

    @staticmethod
    def _twist_active(t: Optional[Twist]) -> bool:
        if t is None:
            return False
        eps = 1e-3
        return (
            abs(t.linear.x) > eps
            or abs(t.linear.y) > eps
            or abs(t.angular.z) > eps
        )

    # ------------------------------------------------------------------
    # CSV column definitions
    # ------------------------------------------------------------------
    @staticmethod
    def _status_header() -> List[str]:
        return [
            "wall_time", "ros_time_sec",
            "inferred_mode", "operator_mode_raw",
            "mapping_status", "mapping_debug_raw",
            "mapping_state", "mapping_goal_x", "mapping_goal_y",
            "mapping_dist", "mapping_nav2_state", "mapping_in_flight",
            "task_status", "navigation_status", "arrival_status",
            "action_debug_last",
            "cmd_vel_nav_x", "cmd_vel_smoothed_x",
            "cmd_vel_x", "cmd_vel_angular_z", "cmd_vel_active",
        ]

    @staticmethod
    def _perception_header() -> List[str]:
        return [
            "wall_time", "ros_time_sec",
            "inferred_mode", "operator_mode_raw",
            # YOLOE 2D
            "detection_count_total", "detection_classes",
            "person_2d_count", "table_like_2d_count",
            "best_person_2d_score", "best_table_like_2d_score",
            "best_table_like_raw_label",
            # Masks
            "mask_count_total", "mask_classes",
            "person_mask_count", "table_like_mask_count",
            "best_person_mask_score", "best_table_like_mask_score",
            "best_table_like_mask_label",
            # 3D
            "detection3d_count_total", "detection3d_classes",
            "person_3d_count", "table_3d_count",
            "best_person_3d_score", "best_table_3d_score",
            # depth_projector counters
            "depth_projector_build_tag",
            "table_detection_seen", "table_mask_seen",
            "table_detection_driven_attempted",
            "table_detection_driven_published",
            "table_detection_driven_failed_no_depth",
            "table_detection_driven_failed_bad_depth",
            "table_detection_driven_failed_tf",
            "table_mask_only_attempted",
            "table_mask_only_published",
            "table_mask_only_failed_no_depth",
            "table_mask_only_failed_bad_depth",
            "table_mask_only_failed_tf",
            "table_3d_published",
            "force_table_mask_only_projection",
            "detections_received", "masks_received", "published_3d",
        ]

    @staticmethod
    def _entities_header() -> List[str]:
        return [
            "wall_time", "ros_time_sec",
            "inferred_mode", "operator_mode_raw",
            "entity_id", "class_label", "raw_label", "status",
            "anchor_id", "confidence", "observations_count",
            "currently_visible", "x", "y", "z",
            "first_seen_sec", "last_seen_sec", "age_since_last_seen",
            "is_dynamic", "uncertainty",
            # Boolean conveniences for downstream pandas filtering.
            "is_person", "is_table",
            "is_confirmed", "is_remembered", "is_candidate",
            "is_invalid", "has_pointcloud_anchor",
            "has_island_anchor", "has_no_anchor",
        ]

    # ------------------------------------------------------------------
    # Periodic samplers
    # ------------------------------------------------------------------
    def _sample_status(self) -> None:
        wall = datetime.now().isoformat(timespec="milliseconds")
        t_rel = time.time() - self._t_start

        mode = self._current_mode()
        kv = _parse_kv_string(self._mapping_debug.last)
        cmd_active = self._twist_active(self._cmd_vel.last)

        # Pull goal_x/y from "goal=(x,y)". The mapping_explorer prints
        # ``goal=(7.87,4.20)`` or ``goal=none``; both parse cleanly.
        goal_x: Optional[float] = None
        goal_y: Optional[float] = None
        goal_raw = kv.get("goal", "")
        m = re.match(r"\(([-+0-9eE.]+),([-+0-9eE.]+)\)", goal_raw)
        if m:
            try:
                goal_x = float(m.group(1))
                goal_y = float(m.group(2))
            except ValueError:
                pass

        row = [
            wall, f"{t_rel:.3f}",
            mode, self._mode_cache.last or "",
            self._mapping_status.last or "",
            self._mapping_debug.last or "",
            kv.get("state", ""),
            "" if goal_x is None else f"{goal_x:.3f}",
            "" if goal_y is None else f"{goal_y:.3f}",
            kv.get("dist", ""),
            kv.get("nav2", ""),
            kv.get("in_flight", ""),
            self._task_status.last or "",
            self._navigation_status.last or "",
            self._arrival_status.last or "",
            self._action_debug.last or "",
            self._twist_x_str(self._cmd_vel_nav.last),
            self._twist_x_str(self._cmd_vel_smoothed.last),
            self._twist_x_str(self._cmd_vel.last),
            self._twist_angz_str(self._cmd_vel.last),
            "1" if cmd_active else "0",
        ]
        self._status_writer.writerow(row)
        self._n_status_rows += 1
        # Mode bookkeeping for summary.
        self._mode_seconds[mode] = self._mode_seconds.get(mode, 0.0) + \
            self._status_period
        if self._print_live:
            self.get_logger().info(
                f"[status t={t_rel:6.1f}s] mode={mode} "
                f"map={self._mapping_status.last!r} "
                f"in_flight={kv.get('in_flight', '?')} "
                f"cmd_vel_active={int(cmd_active)}"
            )

    def _sample_perception(self) -> None:
        wall = datetime.now().isoformat(timespec="milliseconds")
        t_rel = time.time() - self._t_start
        mode = self._current_mode()

        # Detection2D summary.
        d2 = self._detections.last
        det_count = 0
        det_classes: List[str] = []
        person_2d = 0
        table_2d = 0
        best_person_2d = 0.0
        best_table_2d = 0.0
        best_table_lbl = ""
        if d2 is not None:
            det_count = len(d2.detections)
            for d in d2.detections:
                if not d.results:
                    continue
                lbl = str(d.results[0].hypothesis.class_id)
                sc = float(d.results[0].hypothesis.score)
                det_classes.append(lbl)
                if _is_person_like(lbl):
                    person_2d += 1
                    if sc > best_person_2d:
                        best_person_2d = sc
                elif _is_table_like(lbl):
                    table_2d += 1
                    if sc > best_table_2d:
                        best_table_2d = sc
                        best_table_lbl = lbl

        # Mask summary.
        mk = self._masks.last
        mask_count = 0
        mask_classes: List[str] = []
        person_mask = 0
        table_mask = 0
        best_person_mk = 0.0
        best_table_mk = 0.0
        best_table_mk_lbl = ""
        if mk is not None:
            mask_count = len(mk.masks)
            for m in mk.masks:
                lbl = str(getattr(m, "class_label", ""))
                sc = float(getattr(m, "score", 0.0) or 0.0)
                mask_classes.append(lbl)
                if _is_person_like(lbl):
                    person_mask += 1
                    if sc > best_person_mk:
                        best_person_mk = sc
                elif _is_table_like(lbl):
                    table_mask += 1
                    if sc > best_table_mk:
                        best_table_mk = sc
                        best_table_mk_lbl = lbl

        # Detection3D summary.
        d3 = self._detections_3d.last
        d3_count = 0
        d3_classes: List[str] = []
        person_3d = 0
        table_3d = 0
        best_person_3d = 0.0
        best_table_3d = 0.0
        if d3 is not None:
            d3_count = len(d3.detections)
            for d in d3.detections:
                if not d.results:
                    continue
                lbl = str(d.results[0].hypothesis.class_id)
                sc = float(d.results[0].hypothesis.score)
                d3_classes.append(lbl)
                if _is_person_like(lbl):
                    person_3d += 1
                    if sc > best_person_3d:
                        best_person_3d = sc
                elif _is_table_like(lbl):
                    table_3d += 1
                    if sc > best_table_3d:
                        best_table_3d = sc

        # depth_projector debug counters.
        kv = _parse_kv_string(self._depth_projector_stats.last)

        def g(key: str) -> str:
            return kv.get(key, "")

        row = [
            wall, f"{t_rel:.3f}",
            mode, self._mode_cache.last or "",
            det_count, ";".join(det_classes),
            person_2d, table_2d,
            f"{best_person_2d:.3f}",
            f"{best_table_2d:.3f}",
            best_table_lbl,
            mask_count, ";".join(mask_classes),
            person_mask, table_mask,
            f"{best_person_mk:.3f}",
            f"{best_table_mk:.3f}",
            best_table_mk_lbl,
            d3_count, ";".join(d3_classes),
            person_3d, table_3d,
            f"{best_person_3d:.3f}",
            f"{best_table_3d:.3f}",
            g("depth_projector_build_tag"),
            g("table_detection_seen"),
            g("table_mask_seen"),
            g("table_detection_driven_attempted"),
            g("table_detection_driven_published"),
            g("table_detection_driven_failed_no_depth"),
            g("table_detection_driven_failed_bad_depth"),
            g("table_detection_driven_failed_tf"),
            g("table_mask_only_attempted"),
            g("table_mask_only_published"),
            g("table_mask_only_failed_no_depth"),
            g("table_mask_only_failed_bad_depth"),
            g("table_mask_only_failed_tf"),
            g("table_3d_published"),
            g("force_table_mask_only_projection"),
            g("detections_received"),
            g("masks_received"),
            g("published_3d"),
        ]
        self._perception_writer.writerow(row)
        self._n_perception_rows += 1
        if self._print_live:
            self.get_logger().info(
                f"[perception t={t_rel:6.1f}s] mode={mode} "
                f"2d_table={table_2d}@{best_table_2d:.2f} "
                f"mask_table={table_mask}@{best_table_mk:.2f} "
                f"3d_table={table_3d} 3d_person={person_3d} "
                f"build={g('depth_projector_build_tag') or '?'}"
            )

    def _sample_entities(self) -> None:
        wall = datetime.now().isoformat(timespec="milliseconds")
        t_rel = time.time() - self._t_start
        mode = self._current_mode()
        op = self._mode_cache.last or ""

        ent_msg = self._entities.last
        if ent_msg is None or not getattr(ent_msg, "entities", []):
            # Synthetic empty row keeps the timeline contiguous and
            # makes it obvious downstream that semantic memory was
            # silent at this tick. We still record the inferred mode
            # so the operator can see "during the first 30s of
            # AUTO_MAPPING semantic memory had nothing".
            row = [
                wall, f"{t_rel:.3f}", mode, op,
                "", "", "", "", "",
                "", "", "", "", "", "",
                "", "", "", "", "",
                "0", "0", "0", "0", "0",
                "0", "0", "0", "0",
            ]
            self._entities_writer.writerow(row)
            self._n_entity_rows += 1
            return

        for ent in ent_msg.entities:
            eid = str(getattr(ent, "entity_id", ""))
            cls = str(getattr(ent, "class_label", "")).strip().lower()
            display = str(getattr(ent, "display_name", ""))
            raw, status, anchor = _parse_display_name(display)
            status_norm = status.strip().lower()
            pose = getattr(ent, "pose_map", None)
            x = y = z = ""
            if pose is not None:
                p = pose.position
                x = f"{float(p.x):.4f}"
                y = f"{float(p.y):.4f}"
                z = f"{float(p.z):.4f}"
            conf = float(getattr(ent, "confidence", 0.0))
            obs = int(getattr(ent, "observations_count", 0))
            visible = bool(getattr(ent, "currently_visible", False))
            uncertainty = float(getattr(ent, "uncertainty", 0.0))
            is_dynamic = bool(getattr(ent, "is_dynamic", False))

            first_seen = getattr(ent, "first_seen", None)
            last_seen = getattr(ent, "last_seen", None)
            first_seen_sec = ""
            last_seen_sec = ""
            age_str = ""
            if first_seen is not None:
                fs = float(first_seen.sec) + float(first_seen.nanosec) * 1e-9
                first_seen_sec = f"{fs:.3f}"
            if last_seen is not None:
                ls = float(last_seen.sec) + float(last_seen.nanosec) * 1e-9
                last_seen_sec = f"{ls:.3f}"
                # ROS-time age. We can't directly compare to wall
                # time here, so use the message header stamp if
                # available; otherwise leave blank.
                hdr = getattr(ent, "header", None)
                if hdr is not None:
                    msg_t = float(hdr.stamp.sec) + \
                        float(hdr.stamp.nanosec) * 1e-9
                    age = msg_t - ls
                    age_str = f"{max(0.0, age):.3f}"

            is_person = _is_person_like(cls)
            is_table = cls == "table"
            is_confirmed = status_norm == "confirmed"
            # "remembered" = a confirmed entity that's no longer
            # currently_visible — survived past the active sighting.
            is_remembered = is_confirmed and not visible
            is_candidate = status_norm == "candidate"
            is_invalid = status_norm == "invalid"
            has_pc = anchor.startswith("pc_")
            has_isl = anchor.startswith("isl_")
            has_no_anchor = anchor in ("", "-")

            row = [
                wall, f"{t_rel:.3f}", mode, op,
                eid, cls, raw, status_norm, anchor,
                f"{conf:.3f}", obs, "1" if visible else "0",
                x, y, z,
                first_seen_sec, last_seen_sec, age_str,
                "1" if is_dynamic else "0", f"{uncertainty:.3f}",
                "1" if is_person else "0",
                "1" if is_table else "0",
                "1" if is_confirmed else "0",
                "1" if is_remembered else "0",
                "1" if is_candidate else "0",
                "1" if is_invalid else "0",
                "1" if has_pc else "0",
                "1" if has_isl else "0",
                "1" if has_no_anchor else "0",
            ]
            self._entities_writer.writerow(row)
            self._n_entity_rows += 1

            # Stash the FIRST anchor we ever saw for an entity_id
            # so the summary can flag "person_001 went confirmed
            # without ever having a pc_ anchor, only isl_".
            if eid and eid not in self._first_seen_anchor_by_eid:
                self._first_seen_anchor_by_eid[eid] = anchor

    @staticmethod
    def _twist_x_str(t: Optional[Twist]) -> str:
        if t is None:
            return ""
        return f"{float(t.linear.x):.3f}"

    @staticmethod
    def _twist_angz_str(t: Optional[Twist]) -> str:
        if t is None:
            return ""
        return f"{float(t.angular.z):.3f}"

    # ------------------------------------------------------------------
    # Termination
    # ------------------------------------------------------------------
    def _on_duration_elapsed(self) -> None:
        self._shutdown_timer.cancel()
        self.get_logger().info(
            f"semantic_lifecycle_recorder: duration {self._duration_sec:.0f}s "
            f"reached. Stopping spin loop."
        )
        # We can't call rclpy.shutdown() from inside a timer cleanly
        # on every distro; raising KeyboardInterrupt unblocks
        # rclpy.spin in main.
        raise KeyboardInterrupt()

    def close_files(self) -> None:
        for f in (self._status_csv_f, self._perception_csv_f,
                  self._entities_csv_f):
            try:
                f.close()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    def write_summary(self) -> None:
        t_end = time.time()
        duration = t_end - self._t_start
        kv_dp = _parse_kv_string(self._depth_projector_stats.last)
        kv_anchor = _parse_kv_string(self._anchor_debug_stats.last)

        def t_offset(t_abs: Optional[float]) -> Optional[float]:
            if t_abs is None:
                return None
            return t_abs - self._t_start

        def fmt(t_abs: Optional[float]) -> str:
            v = t_offset(t_abs)
            return f"{v:.1f}s" if v is not None else "(never)"

        person_2d_t = self._detections.first_person_t
        person_3d_t = self._detections_3d.first_person_t
        person_seen_t = self._entities.first_seen_t_by_class.get("person")
        person_conf_t = self._entities.first_confirmed_t_by_class.get(
            "person"
        )
        person_conf_mode = self._entities.first_confirmed_mode_by_class.get(
            "person", ""
        )

        table_2d_t = self._detections.first_table_like_t
        table_mask_t = self._masks.first_table_like_t
        table_3d_t = self._detections_3d.first_table_t
        table_seen_t = self._entities.first_seen_t_by_class.get("table")
        table_conf_t = self._entities.first_confirmed_t_by_class.get(
            "table"
        )
        table_conf_mode = self._entities.first_confirmed_mode_by_class.get(
            "table", ""
        )

        def diff(a: Optional[float], b: Optional[float]) -> str:
            if a is None or b is None:
                return "(n/a)"
            return f"{(a - b):.1f}s"

        # -------- write text summary --------
        lines: List[str] = []
        lines.append("Semantic Lifecycle Recording Summary")
        lines.append("=" * 60)
        lines.append("")
        lines.append(f"Duration                : {duration:.1f}s")
        lines.append(f"Status rows             : {self._n_status_rows}")
        lines.append(f"Perception rows         : {self._n_perception_rows}")
        lines.append(f"Entity rows             : {self._n_entity_rows}")
        lines.append(f"Target class filter     : {self._target_class!r}")
        lines.append(f"go2_msgs available      : {_HAS_GO2_MSGS}")
        if not _HAS_GO2_MSGS:
            lines.append(
                f"  (import error: {_GO2_MSGS_IMPORT_ERROR})"
            )
        lines.append("")
        lines.append("CSV output files:")
        lines.append(f"  status     : {self._status_csv_path}")
        lines.append(f"  perception : {self._perception_csv_path}")
        lines.append(f"  entities   : {self._entities_csv_path}")
        lines.append("")
        lines.append("Mode time distribution (s):")
        for k, v in sorted(
            self._mode_seconds.items(), key=lambda kv: -kv[1],
        ):
            pct = (100.0 * v / duration) if duration > 0 else 0.0
            lines.append(f"  {k:<20s} {v:7.1f}s ({pct:5.1f}%)")
        lines.append("")

        # -------- person --------
        lines.append("Person")
        lines.append("-" * 60)
        lines.append(f"  first /detections (person)         : {fmt(person_2d_t)}")
        lines.append(f"  first /detections_3d (person)      : {fmt(person_3d_t)}")
        lines.append(f"  first /semantic_map/objects person : {fmt(person_seen_t)}")
        lines.append(f"  first confirmed person             : {fmt(person_conf_t)}")
        lines.append(
            f"  time from 2D to confirmed          : "
            f"{diff(person_conf_t, person_2d_t)}"
        )
        lines.append(f"  confirmed during mode              : {person_conf_mode or '(never)'}")
        max_obs_p, final_conf_p, has_pc_p = self._scan_entity(
            class_label="person",
        )
        lines.append(f"  max observations_count             : {max_obs_p}")
        lines.append(f"  final confidence                   : {final_conf_p:.2f}")
        lines.append(
            f"  ever had pc_ anchor                : "
            f"{'yes' if has_pc_p else 'no'}"
        )
        lines.append(f"  best 2D person score               : "
                     f"{self._detections.best_person_score_seen:.3f}")
        lines.append(f"  best mask person score             : "
                     f"{self._masks.best_person_score_seen:.3f}")
        lines.append(f"  best 3D person score               : "
                     f"{self._detections_3d.best_person_score_seen:.3f}")
        lines.append("")

        # -------- table --------
        lines.append("Table (incl. desk / dining table / workbench)")
        lines.append("-" * 60)
        lines.append(f"  first /detections table-like        : {fmt(table_2d_t)}")
        lines.append(f"  first /detections/masks table-like  : {fmt(table_mask_t)}")
        lines.append(f"  first /detections_3d table          : {fmt(table_3d_t)}")
        lines.append(f"  first /semantic_map/objects table   : {fmt(table_seen_t)}")
        lines.append(f"  first confirmed table               : {fmt(table_conf_t)}")
        lines.append(
            f"  time from first 2D to confirmed     : "
            f"{diff(table_conf_t, table_2d_t)}"
        )
        lines.append(
            f"  confirmed during mode               : "
            f"{table_conf_mode or '(never)'}"
        )
        lines.append(
            f"  best 2D raw label                   : "
            f"{self._detections.best_table_label_seen!r}"
        )
        lines.append(
            f"  best mask raw label                 : "
            f"{self._masks.best_table_label_seen!r}"
        )
        max_obs_t, final_conf_t, has_pc_t = self._scan_entity(
            class_label="table",
        )
        lines.append(f"  max observations_count              : {max_obs_t}")
        lines.append(f"  final confidence                    : {final_conf_t:.2f}")
        lines.append(
            f"  ever had pc_ anchor                 : "
            f"{'yes' if has_pc_t else 'no'}"
        )
        lines.append(
            f"  best 2D table score                 : "
            f"{self._detections.best_table_score_seen:.3f}"
        )
        lines.append(
            f"  best mask table score               : "
            f"{self._masks.best_table_score_seen:.3f}"
        )
        lines.append(
            f"  best 3D table score                 : "
            f"{self._detections_3d.best_table_score_seen:.3f}"
        )
        lines.append("")

        # -------- failures / bottlenecks --------
        lines.append("Failures / bottlenecks")
        lines.append("-" * 60)
        lines.append(
            f"  depth_projector_build_tag                     : "
            f"{kv_dp.get('depth_projector_build_tag', '(missing)')}"
        )
        for k in (
            "table_detection_seen",
            "table_mask_seen",
            "table_detection_driven_attempted",
            "table_detection_driven_published",
            "table_detection_driven_failed_no_mask",
            "table_detection_driven_failed_no_depth",
            "table_detection_driven_failed_bad_depth",
            "table_detection_driven_failed_tf",
            "table_mask_only_attempted",
            "table_mask_only_published",
            "table_mask_only_skipped_used_mask",
            "table_mask_only_failed_low_score",
            "table_mask_only_failed_no_depth",
            "table_mask_only_failed_bad_depth",
            "table_mask_only_failed_tf",
            "table_3d_published",
            "force_table_mask_only_projection",
        ):
            lines.append(f"  {k:<46s}: {kv_dp.get(k, '(missing)')}")
        lines.append("")
        lines.append("Anchor stats (/semantic_map/anchor_debug_stats):")
        for k in (
            "observations_total",
            "pointcloud_anchor_success",
            "occupancy_island_anchor_success",
            "candidate_no_anchor",
            "rejected_no_pointcloud",
            "pc_map_disagreement",
        ):
            lines.append(f"  {k:<46s}: {kv_anchor.get(k, '(missing)')}")
        lines.append("")

        # -------- warnings --------
        warnings: List[str] = []
        if (
            table_2d_t is not None
            and (table_3d_t is None
                 or (table_3d_t - table_2d_t) > 5.0)
        ):
            gap = (
                (table_3d_t - table_2d_t)
                if table_3d_t is not None else duration
            )
            warnings.append(
                f"table-like 2D appeared at "
                f"{(table_2d_t - self._t_start):.1f}s but "
                f"/detections_3d table did not appear within 5s "
                f"(gap={gap:.1f}s). Suspect depth_projector "
                f"detection-driven failure — check "
                f"table_detection_driven_failed_* counts above."
            )
        if (
            self._detections.first_person_t is not None
            and (
                person_conf_t is None
                or (person_conf_t - self._detections.first_person_t) > 10.0
            )
        ):
            gap = (
                (person_conf_t - self._detections.first_person_t)
                if person_conf_t is not None else duration
            )
            warnings.append(
                f"person 2D detected at "
                f"{(self._detections.first_person_t - self._t_start):.1f}s "
                f"but no confirmed person within 10s (gap={gap:.1f}s). "
                f"Either observations_count never crossed the "
                f"semantic_memory promotion threshold or the "
                f"island/pc anchor never validated."
            )
        if table_conf_t is not None and table_conf_mode == "MANUAL_TELEOP":
            saw_during_auto = any(
                row_mode in ("AUTO_MAPPING", "AUTO_SEMANTIC_NAV")
                and t_offset(table_conf_t) is not None
                for row_mode in self._mode_seconds
                if self._mode_seconds.get(row_mode, 0.0) > 0.0
            )
            warnings.append(
                "table was confirmed during MANUAL_TELEOP. "
                "Check that the autonomous mapping pass has enough "
                "dwell-time on the table — or operator may need to "
                "manually point at it for confirmation."
            )
            _ = saw_during_auto
        if person_conf_t is not None and person_conf_mode == "MANUAL_TELEOP":
            warnings.append(
                "person was confirmed during MANUAL_TELEOP. "
                "Autonomous mapping may not be passing close enough "
                "to the person to accumulate enough observations."
            )
        # "many candidates but no confirmed" check.
        n_candidates = sum(
            1 for s in self._entities.first_seen_t_by_class
            if s not in self._entities.first_confirmed_t_by_class
        )
        if (
            n_candidates >= 3
            and not self._entities.first_confirmed_t_by_class
        ):
            warnings.append(
                f"{n_candidates} candidate classes seen but none "
                f"reached confirmed. Check semantic_memory anchor "
                f"stats (rejected_no_pointcloud above)."
            )

        # Topic-silent warnings.
        for label, cache in (
            ("/depth_projector/debug_stats", self._depth_projector_stats),
            ("/semantic_map/anchor_debug_stats", self._anchor_debug_stats),
            ("/mapping/debug/status", self._mapping_debug),
            ("/cmd_vel", self._cmd_vel),
            ("/detections", self._detections),
            ("/detections/masks", self._masks),
            ("/detections_3d", self._detections_3d),
            ("/semantic_map/objects", self._entities),
        ):
            n = getattr(cache, "n_received", 0)
            if n == 0:
                warnings.append(
                    f"topic {label} received 0 messages during the "
                    f"recording — verify it is being published."
                )

        if warnings:
            lines.append("Warnings")
            lines.append("-" * 60)
            for w in warnings:
                lines.append(f"  ! {w}")
        else:
            lines.append("Warnings: none")
        lines.append("")

        text = "\n".join(lines) + "\n"
        self._summary_path.write_text(text)
        self.get_logger().info(
            f"summary written: {self._summary_path} "
            f"({self._n_status_rows} status / "
            f"{self._n_perception_rows} perception / "
            f"{self._n_entity_rows} entity rows)"
        )

    def _scan_entity(self, *, class_label: str) -> Tuple[int, float, bool]:
        """Walk the latest /semantic_map/objects snapshot for the
        target ``class_label`` and return ``(max_obs, max_conf,
        ever_had_pc_anchor)``. ``ever_had_pc_anchor`` reads from
        ``self._entities.pc_anchor_seen_by_class`` so we count the
        whole recording, not just the last snapshot.
        """
        ent_msg = self._entities.last
        max_obs = 0
        max_conf = 0.0
        if ent_msg is not None:
            for ent in getattr(ent_msg, "entities", []) or []:
                cls = str(getattr(ent, "class_label", "")).strip().lower()
                if cls != class_label:
                    continue
                obs = int(getattr(ent, "observations_count", 0))
                conf = float(getattr(ent, "confidence", 0.0))
                if obs > max_obs:
                    max_obs = obs
                if conf > max_conf:
                    max_conf = conf
        ever_pc = bool(
            self._entities.pc_anchor_seen_by_class.get(class_label, False)
        )
        return max_obs, max_conf, ever_pc


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="record_semantic_lifecycle",
        description=(
            "Semantic marker lifecycle recorder. Writes 3 CSV files "
            "+ a summary describing how YOLOE detections become "
            "confirmed semantic_memory landmarks, with mode "
            "(AUTO_MAPPING / AUTO_SEMANTIC_NAV / MANUAL_TELEOP / "
            "IDLE) tagged on every row."
        ),
    )
    p.add_argument("--duration-sec", type=float, default=120.0)
    p.add_argument("--status-rate-hz", type=float, default=1.0)
    p.add_argument("--perception-rate-hz", type=float, default=5.0)
    p.add_argument("--output-dir", type=Path, default=Path("logs"))
    p.add_argument("--target-class", type=str, default="")
    p.add_argument(
        "--print-live", dest="print_live",
        action="store_true", default=True,
    )
    p.add_argument(
        "--no-print-live", dest="print_live",
        action="store_false",
    )
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    args = _parse_args(argv)

    rclpy.init()
    node = _SemanticLifecycleRecorder(
        duration_sec=args.duration_sec,
        status_rate_hz=args.status_rate_hz,
        perception_rate_hz=args.perception_rate_hz,
        output_dir=args.output_dir,
        target_class=args.target_class,
        print_live=args.print_live,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt — finalising.")
    finally:
        try:
            node.write_summary()
        except Exception as exc:  # pragma: no cover - best-effort
            node.get_logger().warn(f"summary failed: {exc!r}")
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
