#!/usr/bin/env python3
"""Day 6.5 gate #2 helper — strafe via /cmd_vel, compare desk/table pose median."""
from __future__ import annotations

import math
import statistics
import sys
import time
from collections import Counter

if len(sys.argv) < 7:
    print(
        "Usage: _check_day65_dynamic.py "
        "<class_csv> <linear_y> <strafe_sec> <sample_sec> <settle_post> <tol>",
        file=sys.stderr,
    )
    sys.exit(2)

class_csv, linear_y_s, strafe_sec_s, sample_sec_s, settle_post_s, tol_s = sys.argv[1:7]
LINEAR_Y = float(linear_y_s)
STRAFE_SEC = float(strafe_sec_s)
SAMPLE_SEC = float(sample_sec_s)
SETTLE_POST = float(settle_post_s)
TOL = float(tol_s)
TARGET_CLASSES = {c.strip().lower() for c in class_csv.split(",") if c.strip()}

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from go2_msgs.msg import SemanticEntityArray
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
except Exception as e:
    print(f"ERROR_IMPORT {e}")
    sys.exit(2)


def _label_matches_targets(lab: str) -> bool:
    if lab in TARGET_CLASSES:
        return True
    return any(t in lab for t in TARGET_CLASSES if t)


def _is_desk_table_class(lab: str) -> bool:
    if lab in ("desk", "table"):
        return True
    return "desk" in lab or "table" in lab


def _primary_median(bucket: list) -> tuple:
    dt = [b for b in bucket if _is_desk_table_class(b[3])]
    use = dt if dt else bucket
    if not use:
        return None, None, None
    pid = Counter(b[0] for b in use).most_common(1)[0][0]
    pts = [(b[1], b[2]) for b in use if b[0] == pid]
    if len(pts) < 2:
        return None, None, pid
    return (
        statistics.median(p[0] for p in pts),
        statistics.median(p[1] for p in pts),
        pid,
    )


def _median_for_id(bucket: list, pid: str) -> tuple:
    pts = [(b[1], b[2]) for b in bucket if b[0] == pid]
    if len(pts) < 2:
        return None, None
    return statistics.median(p[0] for p in pts), statistics.median(p[1] for p in pts)


def _sample_window(node: Node, duration: float, msg_cls) -> list:
    buf: list = []

    def cb(msg):
        for e in msg.entities:
            lab = e.class_label.strip().lower()
            if _label_matches_targets(lab):
                buf.append(
                    (
                        e.entity_id,
                        float(e.pose_map.position.x),
                        float(e.pose_map.position.y),
                        lab,
                    )
                )

    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    sub = node.create_subscription(msg_cls, "/semantic_map/objects", cb, qos)
    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return list(buf)


node = None
pub = None
try:
    rclpy.init()
    node = Node("_check_day65_dynamic")
    qos_cmd = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    pub = node.create_publisher(Twist, "/cmd_vel", qos_cmd)
    time.sleep(0.3)

    b0 = _sample_window(node, SAMPLE_SEC, SemanticEntityArray)
    mx0, my0, pid = _primary_median(b0)
    if mx0 is None or pid is None:
        print(f"ERROR_NO_PRE samples={len(b0)}")
        sys.exit(3)

    twist = Twist()
    twist.linear.y = LINEAR_Y
    t_end = time.time() + STRAFE_SEC
    while time.time() < t_end:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.09)
    pub.publish(Twist())
    time.sleep(SETTLE_POST)

    b1 = _sample_window(node, SAMPLE_SEC, SemanticEntityArray)
    mx1, my1 = _median_for_id(b1, pid)
    if mx1 is None:
        print(f"ERROR_NO_POST samples={len(b1)} primary_id={pid!r}")
        sys.exit(4)

    drift = math.hypot(mx1 - mx0, my1 - my0)
    ok = drift < TOL
    print(
        f"primary_id={pid!r} pre=({mx0:.4f},{my0:.4f}) post=({mx1:.4f},{my1:.4f}) "
        f"drift={drift:.4f}m tol={TOL} pass={1 if ok else 0}"
    )
    sys.exit(0 if ok else 1)
except SystemExit:
    raise
except Exception as e:
    print(f"ERROR_EXC {e!r}")
    sys.exit(5)
finally:
    try:
        if pub is not None and rclpy.ok():
            pub.publish(Twist())
    except Exception:
        pass
    try:
        if node is not None:
            node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
