#!/usr/bin/env python3
"""Day 9+ Phase D — RTX LiDAR + SLAM TF health recorder.

Why a separate tool from record_mapping_debug / record_semantic_lifecycle?
-------------------------------------------------------------------------
The other two recorders sample mapping FSM state (1 Hz) and the
perception → semantic_memory pipeline (5 Hz). They explicitly do
NOT track *gap* statistics on raw sensor topics, because in a
healthy run those topics are at 4–10 Hz and the recorders don't
care about jitter.

The Isaac Sim 5.1 RTX-LiDAR / Motion-BVH bug presents very
differently:

  * /lidar/points and /scan still publish on average around the
    expected rate, but they have **second-scale gaps** (we have
    field reports of ~14 s stalls).
  * map->odom TF stops being broadcast for the duration of a stall
    because slam_toolbox got starved of input.
  * /clock keeps ticking (Isaac's physics loop is independent), so
    the canonical "is the sim alive?" probe doesn't catch this.

A lifecycle recorder that samples at 1 Hz aliases those bursts.
We need a tool that records *every arrival* on each topic and
reports max inter-arrival gap, longest stall window, success/fail
counts on the TF lookup. That tool is this script.

Topics & lookups
----------------
* /lidar/points        sensor_msgs/PointCloud2          (BEST_EFFORT)
* /scan                sensor_msgs/LaserScan            (BEST_EFFORT)
* /map                 nav_msgs/OccupancyGrid           (RELIABLE,
                                                         TRANSIENT_LOCAL)
* /clock               rosgraph_msgs/Clock              (BEST_EFFORT)
* TF lookup            map -> odom (5 Hz polling)

CSV output (one row per snapshot tick, default 2 Hz):

  logs/lidar_health_YYYYMMDD_HHMMSS.csv
    wall_time, t_rel_sec,
    lidar_points_n_received, lidar_points_hz, lidar_points_gap_max_sec,
    scan_n_received, scan_hz, scan_gap_max_sec,
    map_n_received, map_hz,
    clock_n_received, clock_hz,
    tf_map_odom_ok, tf_map_odom_age_sec, tf_map_odom_n_fail

Plus a summary text file with per-topic verdicts. The summary's
verdict bar is the user's stated bug-presence threshold:

  GREEN   : max gap <  5.0 s
  YELLOW  : max gap <  9.5 s
  RED     : max gap >= 10.0 s   <- "stall" verdict; user wants this gone.

Output written to logs/lidar_health_*.summary.txt.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, List, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------
def _sensor_qos() -> QoSProfile:
    """Best-effort + KEEP_LAST. Matches ROS sensor-data convention.
    Crucial for /lidar/points and /scan: many publishers (Isaac Sim's
    RTX bridge included) emit BEST_EFFORT and a RELIABLE subscriber
    silently gets zero messages."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _map_qos() -> QoSProfile:
    """slam_toolbox latches /map TRANSIENT_LOCAL. A subscriber that
    joins after the first map publish needs this to receive the
    last-published map at all."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


# ---------------------------------------------------------------------------
# Per-topic gap tracker
# ---------------------------------------------------------------------------
@dataclass
class _GapStats:
    """Tracks every inter-arrival interval on a topic.

    We deliberately do NOT use a sliding window — for stall
    detection we need the GLOBAL max gap across the whole
    recording, not a windowed estimate that would wash out a
    14-second stall as soon as messages resume.
    """

    n_received: int = 0
    first_t: Optional[float] = None
    last_t: Optional[float] = None
    max_gap: float = 0.0
    max_gap_at: Optional[float] = None
    # 95th-percentile-ish: reservoir of all gaps. Capped at 4096 to
    # bound memory; for a 600s run at 10 Hz that's well within
    # capacity (6000 gaps).
    _gaps: Deque[float] = field(default_factory=lambda: deque(maxlen=4096))

    def stamp(self, t: float) -> None:
        if self.first_t is None:
            self.first_t = t
        elif self.last_t is not None:
            gap = t - self.last_t
            self._gaps.append(gap)
            if gap > self.max_gap:
                self.max_gap = gap
                self.max_gap_at = t
        self.last_t = t
        self.n_received += 1

    def hz(self, *, now: Optional[float] = None) -> float:
        """Average rate over the entire recording window."""
        if self.first_t is None or self.last_t is None:
            return 0.0
        end = now if now is not None else self.last_t
        span = end - self.first_t
        if span <= 0.0 or self.n_received < 2:
            return 0.0
        return float(self.n_received - 1) / span

    def gap_p95(self) -> float:
        if not self._gaps:
            return 0.0
        s = sorted(self._gaps)
        idx = max(0, int(0.95 * (len(s) - 1)))
        return s[idx]


@dataclass
class _TfLookupStats:
    """Tracks success/fail of map->odom lookups. Unlike topic
    gap stats, we have the option to do a polled lookup at fixed
    rate, so the gap is the time between two SUCCESSFUL lookups."""

    n_ok: int = 0
    n_fail: int = 0
    last_ok_t: Optional[float] = None
    max_age: float = 0.0
    max_age_at: Optional[float] = None
    last_fail_reason: str = ""

    def record_ok(self, t: float) -> None:
        if self.last_ok_t is not None:
            age = t - self.last_ok_t
            if age > self.max_age:
                self.max_age = age
                self.max_age_at = t
        self.last_ok_t = t
        self.n_ok += 1

    def record_fail(self, t: float, reason: str) -> None:
        self.n_fail += 1
        self.last_fail_reason = reason
        # Even on failure, update max_age so a long stretch of
        # failures shows up as a large age value.
        if self.last_ok_t is not None:
            age = t - self.last_ok_t
            if age > self.max_age:
                self.max_age = age
                self.max_age_at = t


# ---------------------------------------------------------------------------
# Recorder node
# ---------------------------------------------------------------------------
class _LidarHealthRecorder(Node):
    def __init__(
        self,
        *,
        duration_sec: float,
        snapshot_rate_hz: float,
        tf_poll_rate_hz: float,
        output_dir: Path,
        print_live: bool,
    ) -> None:
        super().__init__("lidar_health_recorder")
        self._duration = float(duration_sec)
        self._snapshot_period = (
            1.0 / max(snapshot_rate_hz, 0.01)
            if snapshot_rate_hz > 0 else 0.5
        )
        self._tf_poll_period = (
            1.0 / max(tf_poll_rate_hz, 0.01)
            if tf_poll_rate_hz > 0 else 0.2
        )
        self._print_live = bool(print_live)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = output_dir / f"lidar_health_{ts}.csv"
        self._summary_path = output_dir / (
            f"lidar_health_{ts}.summary.txt"
        )

        self._lidar = _GapStats()
        self._scan = _GapStats()
        self._map = _GapStats()
        # NB: NOT ``self._clock`` — rclpy.Node owns ``_clock`` and Timer
        # creation walks through it. Shadowing breaks every timer.
        self._clock_topic = _GapStats()
        self._tf_stats = _TfLookupStats()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Subscriptions. Sensor topics use BEST_EFFORT; /map uses
        # TRANSIENT_LOCAL. We do not store the message body — only
        # arrival timestamps, which keeps the recorder light.
        self.create_subscription(
            PointCloud2, "/lidar/points",
            lambda msg: self._lidar.stamp(time.time()),
            _sensor_qos(),
        )
        self.create_subscription(
            LaserScan, "/scan",
            lambda msg: self._scan.stamp(time.time()),
            _sensor_qos(),
        )
        self.create_subscription(
            OccupancyGrid, "/map",
            lambda msg: self._map.stamp(time.time()),
            _map_qos(),
        )
        self.create_subscription(
            Clock, "/clock",
            lambda msg: self._clock_topic.stamp(time.time()),
            _sensor_qos(),
        )

        self._csv_f = self._csv_path.open("w", newline="")
        self._csv = csv.writer(self._csv_f)
        self._csv.writerow(self._header())

        self._t_start = time.time()
        self._snapshot_timer = self.create_timer(
            self._snapshot_period, self._snapshot,
        )
        self._tf_timer = self.create_timer(
            self._tf_poll_period, self._poll_tf,
        )
        self._shutdown_timer = self.create_timer(
            self._duration, self._on_duration_elapsed,
        )

        self.get_logger().info(
            f"lidar_health_recorder started. duration={self._duration:.0f}s "
            f"snapshot_period={self._snapshot_period:.2f}s "
            f"tf_poll_period={self._tf_poll_period:.2f}s "
            f"output={self._csv_path}"
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _header() -> List[str]:
        return [
            "wall_time", "t_rel_sec",
            "lidar_points_n_received", "lidar_points_hz",
            "lidar_points_gap_max_sec",
            "scan_n_received", "scan_hz", "scan_gap_max_sec",
            "map_n_received", "map_hz",
            "clock_n_received", "clock_hz",
            "tf_map_odom_ok", "tf_map_odom_age_sec",
            "tf_map_odom_n_fail",
        ]

    def _poll_tf(self) -> None:
        now_w = time.time()
        try:
            self._tf_buffer.lookup_transform(
                "map", "odom", rclpy.time.Time(),
            )
            self._tf_stats.record_ok(now_w)
            return
        except TransformException as exc:
            self._tf_stats.record_fail(now_w, repr(exc))

    def _snapshot(self) -> None:
        wall = datetime.now().isoformat(timespec="milliseconds")
        now = time.time()
        t_rel = now - self._t_start

        lidar_age = (
            (now - self._tf_stats.last_ok_t)
            if self._tf_stats.last_ok_t is not None else float("inf")
        )
        # CSV cells must not be ``inf`` — write a sentinel large
        # number so downstream pandas / spreadsheet imports don't
        # choke. 1e9 is unmistakably "never seen".
        if lidar_age == float("inf"):
            lidar_age_str = "1e9"
        else:
            lidar_age_str = f"{lidar_age:.3f}"

        row = [
            wall, f"{t_rel:.3f}",
            self._lidar.n_received, f"{self._lidar.hz(now=now):.3f}",
            f"{self._lidar.max_gap:.3f}",
            self._scan.n_received, f"{self._scan.hz(now=now):.3f}",
            f"{self._scan.max_gap:.3f}",
            self._map.n_received, f"{self._map.hz(now=now):.3f}",
            self._clock_topic.n_received,
            f"{self._clock_topic.hz(now=now):.3f}",
            "1" if self._tf_stats.last_ok_t is not None else "0",
            lidar_age_str,
            self._tf_stats.n_fail,
        ]
        self._csv.writerow(row)
        if self._print_live:
            self.get_logger().info(
                f"[t={t_rel:6.1f}s] "
                f"lidar={self._lidar.n_received}@{self._lidar.hz(now=now):.1f}Hz "
                f"max_gap={self._lidar.max_gap:.2f}s | "
                f"scan max_gap={self._scan.max_gap:.2f}s | "
                f"map_n={self._map.n_received} "
                f"clock_n={self._clock_topic.n_received} | "
                f"tf_ok={self._tf_stats.n_ok} fail={self._tf_stats.n_fail} "
                f"max_age={self._tf_stats.max_age:.2f}s"
            )

    # ------------------------------------------------------------------
    def _on_duration_elapsed(self) -> None:
        self._shutdown_timer.cancel()
        self.get_logger().info(
            f"lidar_health_recorder: duration {self._duration:.0f}s "
            f"reached. Stopping spin loop."
        )
        raise KeyboardInterrupt()

    def close_files(self) -> None:
        try:
            self._csv_f.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    def write_summary(self) -> None:
        now = time.time()
        duration = now - self._t_start

        def verdict(max_gap: float, *, label: str) -> str:
            # Thresholds match the user's stated bug-presence definition:
            # the original report had ~14s stalls; >=10s is "RED",
            # 5s..10s is "YELLOW", <5s is "GREEN".
            if max_gap >= 10.0:
                return f"RED    ({label} max gap {max_gap:.2f}s >= 10s — stall)"
            if max_gap >= 5.0:
                return f"YELLOW ({label} max gap {max_gap:.2f}s, watch)"
            return f"GREEN  ({label} max gap {max_gap:.2f}s)"

        lines: List[str] = []
        lines.append("LiDAR / SLAM-TF Health Summary")
        lines.append("=" * 60)
        lines.append("")
        lines.append(f"Duration            : {duration:.1f}s")
        lines.append(f"CSV                 : {self._csv_path}")
        lines.append("")

        for label, stats in (
            ("/lidar/points", self._lidar),
            ("/scan", self._scan),
            ("/map", self._map),
            ("/clock", self._clock_topic),
        ):
            n = stats.n_received
            hz = stats.hz(now=now)
            lines.append(f"{label}")
            lines.append("-" * 60)
            lines.append(f"  n_received        : {n}")
            lines.append(f"  hz_avg            : {hz:.3f}")
            lines.append(f"  gap_max_sec       : {stats.max_gap:.3f}")
            lines.append(f"  gap_p95_sec       : {stats.gap_p95():.3f}")
            if stats.max_gap_at is not None:
                lines.append(
                    f"  worst_gap_at_t_sec: "
                    f"{(stats.max_gap_at - self._t_start):.1f}"
                )
            if n == 0:
                lines.append(
                    f"  verdict           : DEAD ({label} silent for "
                    f"the whole {duration:.0f}s window)"
                )
            else:
                lines.append(f"  verdict           : "
                             f"{verdict(stats.max_gap, label=label)}")
            lines.append("")

        # TF lookup summary.
        lines.append("TF lookup map -> odom")
        lines.append("-" * 60)
        lines.append(f"  n_ok              : {self._tf_stats.n_ok}")
        lines.append(f"  n_fail            : {self._tf_stats.n_fail}")
        lines.append(
            f"  max_age_sec       : {self._tf_stats.max_age:.3f}"
        )
        if self._tf_stats.max_age_at is not None:
            lines.append(
                f"  worst_age_at_t_sec: "
                f"{(self._tf_stats.max_age_at - self._t_start):.1f}"
            )
        lines.append(
            f"  last_fail_reason  : "
            f"{self._tf_stats.last_fail_reason or '(none)'}"
        )
        if self._tf_stats.n_ok == 0:
            lines.append(
                "  verdict           : DEAD (map->odom never resolved; "
                "slam_toolbox almost certainly never published it — "
                "check /map publishes and slam_toolbox stderr)"
            )
        else:
            lines.append(
                f"  verdict           : "
                f"{verdict(self._tf_stats.max_age, label='map->odom')}"
            )
        lines.append("")

        # Cross-topic interpretation table — the user's primary
        # question is "did MotionBVH fix it?" Codify the pass/fail
        # rule explicitly so the operator doesn't have to eyeball.
        lines.append("Cross-topic rollup")
        lines.append("-" * 60)
        any_red = False
        any_dead = (
            self._lidar.n_received == 0
            or self._scan.n_received == 0
            or self._tf_stats.n_ok == 0
        )
        for label, max_gap in (
            ("/lidar/points", self._lidar.max_gap),
            ("/scan", self._scan.max_gap),
            ("map->odom", self._tf_stats.max_age),
        ):
            if max_gap >= 10.0:
                any_red = True
                lines.append(
                    f"  ! {label} stalled >= 10s — bug still present"
                )
        if any_dead and not any_red:
            missing = [
                lbl for lbl, n in (
                    ("/lidar/points", self._lidar.n_received),
                    ("/scan", self._scan.n_received),
                    ("/map", self._map.n_received),
                    ("/clock", self._clock_topic.n_received),
                    ("map->odom", self._tf_stats.n_ok),
                ) if n == 0
            ]
            lines.append(
                "  Inconclusive: the following topic(s) received 0 "
                f"messages — verdict deferred until they publish: "
                f"{', '.join(missing)}."
            )
        elif not any_red:
            lines.append(
                "  All sensor / TF gaps below 10s — MotionBVH fix appears "
                "effective. /lidar/points hz can be as low as ~4 Hz on a "
                "healthy sim; gap-max is the better stall indicator."
            )
        else:
            lines.append("")
            lines.append(
                "  Recommended next step: try plan C — switch lidar_config "
                "to OS1_REV6_32ch10hz512res to halve BVH ray budget. "
                "Either edit the default in sim/run_go2_warehouse_ros2.py "
                "or pass --lidar-config OS1_REV6_32ch10hz512res on the "
                "Isaac Sim launch command line."
            )
        lines.append("")

        text = "\n".join(lines) + "\n"
        self._summary_path.write_text(text)
        self.get_logger().info(f"summary written: {self._summary_path}")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="record_lidar_health",
        description=(
            "Record /lidar/points + /scan + /map + /clock arrival "
            "rates and map->odom TF freshness. Emits a CSV per "
            "snapshot tick + a summary text file with PASS/WATCH/"
            "STALL verdicts (10s gap = stall)."
        ),
    )
    p.add_argument("--duration-sec", type=float, default=600.0)
    p.add_argument("--snapshot-rate-hz", type=float, default=2.0)
    p.add_argument("--tf-poll-rate-hz", type=float, default=5.0)
    p.add_argument("--output-dir", type=Path, default=Path("logs"))
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
    node = _LidarHealthRecorder(
        duration_sec=args.duration_sec,
        snapshot_rate_hz=args.snapshot_rate_hz,
        tf_poll_rate_hz=args.tf_poll_rate_hz,
        output_dir=args.output_dir,
        print_live=args.print_live,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt — finalising.")
    finally:
        try:
            node.write_summary()
        except Exception as exc:  # pragma: no cover
            node.get_logger().warn(f"summary failed: {exc!r}")
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
