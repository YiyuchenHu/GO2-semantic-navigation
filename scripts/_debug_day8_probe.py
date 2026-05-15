#!/usr/bin/env python3
"""DEBUG MODE instrumentation — Day 8 sensor + TF + state probe.

Spins a temporary rclpy node for ~15 s, subscribes to the topics that
sit on the data-flow chain we suspect is broken, and writes one NDJSON
line per finding into the debug session log file.

Hypotheses tested in one pass (see chat for full list):
  H1  /lidar/points -> pointcloud_to_laserscan -> /scan          (sensor bridge)
  H2  /scan -> slam_toolbox -> /map + map->odom TF               (SLAM up?)
  H3  duplicate /lifecycle_manager_slam                           (env contamination)
  H4  /map exists -> depth_projector publishes /detections_3d    (TF gate works)
  H5  /target/selected stale message                              (FSM hijack)

Usage:
    python3 scripts/_debug_day8_probe.py [run_id]

`run_id` is an arbitrary string used to tag every log entry — pick e.g.
`pre-fix` before launching tf_and_scan, `post-fix` after.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

import tf2_ros


DEBUG_LOG_PATH = os.environ.get(
    "GO2_DEBUG_LOG",
    os.path.expanduser("~/go2_debug.log"),
)
SESSION = "7f68f3"


# region agent log
def emit(location: str, message: str, data: Dict[str, Any],
         hypothesis_id: str, run_id: str) -> None:
    """Append one NDJSON line to the debug session log path."""
    ts_ms = int(time.time() * 1000)
    rec = {
        "sessionId": SESSION,
        "id": f"log_{ts_ms}_{hypothesis_id}",
        "timestamp": ts_ms,
        "location": location,
        "message": message,
        "data": data,
        "runId": run_id,
        "hypothesisId": hypothesis_id,
    }
    os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
    with open(DEBUG_LOG_PATH, "a", buffering=1) as f:
        f.write(json.dumps(rec) + "\n")
# endregion


class Day8Probe(Node):
    def __init__(self, run_id: str) -> None:
        super().__init__("debug_day8_probe")
        self.run_id = run_id
        self.counters: Dict[str, int] = {
            "scan": 0,
            "cloud": 0,
            "map": 0,
            "det3d": 0,
            "objects": 0,
            "frontier_markers": 0,
            "target_selected": 0,
        }
        self.first_seen: Dict[str, Dict[str, Any]] = {}

        # /scan is BEST_EFFORT in chair_perception/tf_and_scan
        from sensor_msgs.msg import LaserScan, PointCloud2
        from nav_msgs.msg import OccupancyGrid
        from visualization_msgs.msg import MarkerArray

        be_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            LaserScan, "/scan", self._mk_cb("scan"), be_qos
        )
        self.create_subscription(
            PointCloud2, "/lidar/points", self._mk_cb("cloud"), be_qos
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._mk_cb("map"), latched_qos
        )

        # Project msgs (may not be on PYTHONPATH if user forgot to source)
        try:
            from vision_msgs.msg import Detection3DArray
            self.create_subscription(
                Detection3DArray, "/detections_3d", self._mk_cb("det3d"), 10
            )
        except Exception as exc:
            emit(
                "_debug_day8_probe.py:init",
                "could not subscribe to /detections_3d",
                {"error": str(exc)[:200]}, "H4", self.run_id,
            )

        try:
            from go2_msgs.msg import SemanticEntityArray, SelectedTarget
            self.create_subscription(
                SemanticEntityArray, "/semantic_map/objects",
                self._mk_cb("objects"), 10,
            )
            self.create_subscription(
                SelectedTarget, "/target/selected",
                self._mk_cb("target_selected"), 10,
            )
        except Exception as exc:
            emit(
                "_debug_day8_probe.py:init",
                "could not subscribe to go2_msgs (workspace not sourced?)",
                {"error": str(exc)[:200]}, "H5", self.run_id,
            )

        self.create_subscription(
            MarkerArray, "/frontier_markers",
            self._mk_cb("frontier_markers"), 10,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _mk_cb(self, kind: str):
        def _cb(msg):
            self.counters[kind] += 1
            if kind not in self.first_seen:
                hdr_frame = ""
                hdr_stamp = None
                if hasattr(msg, "header"):
                    hdr_frame = getattr(msg.header, "frame_id", "")
                    hdr_stamp_msg = getattr(msg.header, "stamp", None)
                    if hdr_stamp_msg is not None:
                        hdr_stamp = (
                            hdr_stamp_msg.sec
                            + hdr_stamp_msg.nanosec * 1e-9
                        )
                extra: Dict[str, Any] = {
                    "frame_id": hdr_frame,
                    "stamp_sec": hdr_stamp,
                }
                if kind == "scan":
                    extra["ranges_len"] = len(msg.ranges)
                elif kind == "cloud":
                    extra["width"] = msg.width
                    extra["height"] = msg.height
                elif kind == "map":
                    extra["w"] = msg.info.width
                    extra["h"] = msg.info.height
                    extra["resolution"] = msg.info.resolution
                elif kind == "det3d":
                    extra["detections_len"] = len(msg.detections)
                elif kind == "objects":
                    extra["entities_len"] = len(msg.entities)
                elif kind == "target_selected":
                    extra["entity_id"] = getattr(msg, "entity_id", "")
                    extra["class_label"] = getattr(msg, "class_label", "")
                    extra["task_id"] = getattr(msg, "task_id", "")
                elif kind == "frontier_markers":
                    extra["markers_len"] = len(msg.markers)
                self.first_seen[kind] = extra
                # Tag hypothesis based on which signal arrived first.
                hyp = {
                    "scan": "H1", "cloud": "H1",
                    "map": "H2",
                    "det3d": "H4", "objects": "H4",
                    "target_selected": "H5",
                    "frontier_markers": "H2",
                }[kind]
                emit(
                    "_debug_day8_probe.py:first_msg",
                    f"first {kind} message received",
                    extra, hyp, self.run_id,
                )
        return _cb

    def probe_tf(self) -> None:
        for parent, child, hyp in [
            ("map", "odom", "H2"),
            ("map", "base_link", "H2"),
            ("odom", "base_link", "H1"),
            ("base_link", "lidar_link", "H1"),
            ("base_link", "camera_color_optical_frame", "H4"),
        ]:
            try:
                tx = self.tf_buffer.lookup_transform(
                    parent, child, Time(),
                    timeout=Duration(seconds=0.2),
                )
                emit(
                    "_debug_day8_probe.py:tf_probe",
                    f"TF {parent} -> {child} OK",
                    {
                        "x": tx.transform.translation.x,
                        "y": tx.transform.translation.y,
                        "z": tx.transform.translation.z,
                    }, hyp, self.run_id,
                )
            except Exception as exc:
                emit(
                    "_debug_day8_probe.py:tf_probe",
                    f"TF {parent} -> {child} FAILED",
                    {"error": str(exc)[:200]}, hyp, self.run_id,
                )

    def probe_node_graph(self) -> None:
        try:
            out = subprocess.check_output(
                ["ros2", "node", "list"],
                stderr=subprocess.STDOUT,
                timeout=5.0,
            ).decode("utf-8", errors="replace")
        except Exception as exc:
            emit(
                "_debug_day8_probe.py:node_graph",
                "ros2 node list failed",
                {"error": str(exc)[:200]}, "H3", self.run_id,
            )
            return
        nodes = sorted(set(line.strip() for line in out.splitlines()
                           if line.strip()))
        # Count duplicates BEFORE dedup (we want to know if duplicates exist).
        raw = [line.strip() for line in out.splitlines() if line.strip()]
        from collections import Counter
        c = Counter(raw)
        dupes = {k: v for k, v in c.items() if v > 1}
        emit(
            "_debug_day8_probe.py:node_graph",
            "ros2 node list snapshot",
            {
                "n_unique": len(nodes),
                "n_total_lines": len(raw),
                "duplicates": dupes,
                "has_pointcloud_to_laserscan": any(
                    "pointcloud_to_laserscan" in n for n in nodes
                ),
                "has_slam_toolbox": any(
                    "slam_toolbox" in n for n in nodes
                ),
                "has_async_slam": any(
                    "async_slam" in n for n in nodes
                ),
                "has_lifecycle_manager_slam": any(
                    "lifecycle_manager_slam" in n for n in nodes
                ),
                "all_nodes": nodes[:80],  # cap to avoid huge log lines
            }, "H3", self.run_id,
        )

    def report_counters(self) -> None:
        for kind, count in self.counters.items():
            hyp = {
                "scan": "H1", "cloud": "H1",
                "map": "H2",
                "det3d": "H4", "objects": "H4",
                "target_selected": "H5",
                "frontier_markers": "H2",
            }[kind]
            emit(
                "_debug_day8_probe.py:rate_summary",
                f"{kind} message count over probe window",
                {"messages": count, "first_seen": kind in self.first_seen},
                hyp, self.run_id,
            )


def main() -> int:
    run_id = sys.argv[1] if len(sys.argv) > 1 else "post-fix"
    duration_s = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    rclpy.init()
    node = Day8Probe(run_id)
    emit(
        "_debug_day8_probe.py:start", "probe started",
        {"run_id": run_id, "duration_s": duration_s,
         "log_path": DEBUG_LOG_PATH}, "H0", run_id,
    )

    # First snapshot of node graph BEFORE the spin window — we want to
    # know whether duplicates exist at probe start.
    node.probe_node_graph()

    deadline = time.time() + duration_s
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    node.report_counters()
    node.probe_tf()
    emit(
        "_debug_day8_probe.py:done", "probe finished",
        {"counters": node.counters,
         "first_seen_topics": sorted(node.first_seen.keys())},
        "H0", run_id,
    )

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
