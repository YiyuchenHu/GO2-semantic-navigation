#!/usr/bin/env python3
"""Day 8 gate #2 — frontier consumption while Go2 walks.

Sequence:
  1. Wait for /get_frontiers service to appear (frontier_explorer up).
  2. Wait for first /map message via TF map->base_link (so we have a
     real robot pose to feed the service request).
  3. Call /get_frontiers → record N1 = len(frontier_goals).
  4. Publish /cmd_vel for `drive_sec` seconds: forward + small yaw,
     pattern lifted from _check_day65_dynamic.py.
  5. Settle, call /get_frontiers again → record N2.
  6. Pass if N2 < N1 (at least one frontier consumed) — also pass if
     N2 == 0 (everything in sight got mapped).

Args (positional):
    drive_sec      e.g. 30.0
    linear_x       e.g. 0.30 m/s
    angular_z      e.g. 0.10 rad/s   (pass 0 for pure forward)
    settle_sec     e.g. 2.0          (wait between strafe and 2nd query)
    service_to     e.g. 10.0         (wait this long for the service)
    pre_to         e.g. 5.0          (wait this long for first response)

Exits:
    0    pass (N2 < N1)
    1    fail (N2 >= N1)
    2    setup error (service missing, robot pose missing, etc.)
    3    runtime error (rclpy raised, response timed out)
"""
from __future__ import annotations

import math
import sys
import time

if len(sys.argv) < 7:
    print(
        "Usage: _check_day8_consumption.py "
        "<drive_sec> <linear_x> <angular_z> <settle_sec> "
        "<service_to> <pre_to>",
        file=sys.stderr,
    )
    sys.exit(2)

DRIVE_SEC = float(sys.argv[1])
LINEAR_X = float(sys.argv[2])
ANGULAR_Z = float(sys.argv[3])
SETTLE_SEC = float(sys.argv[4])
SERVICE_TO = float(sys.argv[5])
PRE_TO = float(sys.argv[6])

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from go2_msgs.srv import GetFrontiers
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception as exc:
    print(f"ERROR_IMPORT {exc}")
    sys.exit(2)


GLOBAL_FRAME = "map"
BASE_FRAME = "base_link"


def _lookup_pose(node: Node, tf_buf: Buffer) -> PoseStamped | None:
    try:
        t = tf_buf.lookup_transform(
            GLOBAL_FRAME, BASE_FRAME, Time(), timeout=Duration(seconds=0.2)
        )
    except TransformException:
        return None
    ps = PoseStamped()
    ps.header.frame_id = GLOBAL_FRAME
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = float(t.transform.translation.x)
    ps.pose.position.y = float(t.transform.translation.y)
    ps.pose.position.z = float(t.transform.translation.z)
    ps.pose.orientation = t.transform.rotation
    return ps


def _wait_for_pose(node: Node, tf_buf: Buffer, timeout_sec: float):
    deadline = time.time() + timeout_sec
    pose = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        pose = _lookup_pose(node, tf_buf)
        if pose is not None:
            return pose
    return None


def _call_get_frontiers(
    node: Node, client, pose: PoseStamped, timeout_sec: float
):
    if not client.service_is_ready():
        return None, "service_not_ready"
    req = GetFrontiers.Request()
    req.robot_pose = pose
    fut = client.call_async(req)
    deadline = time.time() + timeout_sec
    while not fut.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not fut.done():
        return None, "response_timeout"
    try:
        resp = fut.result()
    except Exception as exc:
        return None, f"future_exc:{exc!r}"
    return resp, "ok"


node = None
pub = None
exit_code = 3
try:
    rclpy.init()
    node = Node("_check_day8_consumption")

    tf_buf = Buffer()
    _ = TransformListener(tf_buf, node)

    qos_cmd = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    pub = node.create_publisher(Twist, "/cmd_vel", qos_cmd)

    client = node.create_client(GetFrontiers, "/get_frontiers")
    deadline = time.time() + SERVICE_TO
    while not client.service_is_ready() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not client.service_is_ready():
        print(
            f"ERROR_NO_SERVICE /get_frontiers not ready in "
            f"{SERVICE_TO:.1f}s"
        )
        exit_code = 2
        sys.exit(exit_code)

    pose0 = _wait_for_pose(node, tf_buf, PRE_TO)
    if pose0 is None:
        print(
            f"ERROR_NO_POSE TF {GLOBAL_FRAME}->{BASE_FRAME} "
            f"unavailable in {PRE_TO:.1f}s; is SLAM up and the robot "
            f"localized?"
        )
        exit_code = 2
        sys.exit(exit_code)

    resp1, why = _call_get_frontiers(node, client, pose0, PRE_TO)
    if resp1 is None:
        print(f"ERROR_PRE_CALL why={why}")
        exit_code = 3
        sys.exit(exit_code)
    if not resp1.success:
        print(
            f"ERROR_PRE_NOT_SUCCESS message={resp1.message!r}"
        )
        exit_code = 3
        sys.exit(exit_code)
    n1 = len(resp1.frontier_goals)

    twist = Twist()
    twist.linear.x = LINEAR_X
    twist.angular.z = ANGULAR_Z
    t_end = time.time() + DRIVE_SEC
    while time.time() < t_end:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.09)
    pub.publish(Twist())  # halt
    time.sleep(SETTLE_SEC)

    pose1 = _lookup_pose(node, tf_buf) or pose0
    resp2, why = _call_get_frontiers(node, client, pose1, PRE_TO)
    if resp2 is None:
        print(f"ERROR_POST_CALL why={why}")
        exit_code = 3
        sys.exit(exit_code)
    if not resp2.success:
        print(
            f"ERROR_POST_NOT_SUCCESS message={resp2.message!r}"
        )
        exit_code = 3
        sys.exit(exit_code)
    n2 = len(resp2.frontier_goals)

    moved = math.hypot(
        pose1.pose.position.x - pose0.pose.position.x,
        pose1.pose.position.y - pose0.pose.position.y,
    )

    ok = n2 < n1
    print(
        f"n1={n1} n2={n2} moved={moved:.2f}m drive={DRIVE_SEC:.1f}s "
        f"linear_x={LINEAR_X} angular_z={ANGULAR_Z} pass={1 if ok else 0}"
    )
    exit_code = 0 if ok else 1
except SystemExit:
    raise
except Exception as exc:
    print(f"ERROR_EXC {exc!r}")
    exit_code = 3
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
    sys.exit(exit_code)
