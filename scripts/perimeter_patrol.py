#!/usr/bin/env python3
"""perimeter_patrol.py — drive Go2 along the warehouse perimeter via Nav2.

Use case
--------
You finished Phase A (mapping_explorer locked DONE) but YOLOE never
caught the chair / box / table because mapping_explorer's frontier-
ranking biased Go2 toward unknown space and not toward the walls.
This script lets you say "go around the room one more time, slowly,
looking around" so semantic_memory_aggregator can populate before you
hand the operator the steering wheel for Phase B.

What it does
------------
1. Computes 4 (or 8) waypoints inside the 10x10 m warehouse, inset 1.5 m
   from the walls so Nav2's inflation layer can't refuse them.
2. For each waypoint, sends a Nav2 NavigateToPose goal and waits for
   SUCCEEDED / ABORTED.
3. After arriving, optionally publishes a slow rotational /cmd_vel for
   a full 360 deg so the front-facing camera scans every direction.
4. Returns to (or near) the starting pose at the end.

This script runs OUTSIDE the day8_two_phase launch — it is just a
client of the existing Nav2 stack. Run it in a fresh terminal while
mapping_explorer is in DONE state, or any time the global costmap is
populated. Exploration FSMs and task_coordinator are not consulted; the
goals go straight to /navigate_to_pose, so they preempt anything else.

Frame convention (matches the rest of the repo)
-----------------------------------------------
- Goals are stamped in 'map' (Nav2 global frame).
- 'world' frame in the .usd is offset (-4, -4) from 'map' (see
  tf_and_scan.launch.py: world->map static transform is x=-4 y=-4).
  Therefore in MAP coords the warehouse spans roughly x in [-1, 9],
  y in [-1, 9]. Inset 1.5m gives a safe waypoint box of [0.5, 7.5]^2.
- chair sits at world (3.5, -3.5) -> map (7.5, 0.5), close to the
  SE-corner waypoint, so the SE leg is the one most likely to surface
  the chair to YOLOE.

Usage
-----
    # Default 4-corner perimeter, 360 spin at each corner, ~8 min total.
    python3 scripts/perimeter_patrol.py

    # Add mid-edge waypoints (8 total), no spinning. Faster to traverse,
    # less chance to scan dense detail.
    python3 scripts/perimeter_patrol.py --dense --no-spin

    # Custom inset (more conservative if costmap is fat).
    python3 scripts/perimeter_patrol.py --inset 2.0

    # Single-shot: just go to the SE corner (where the chair is).
    python3 scripts/perimeter_patrol.py --se-only

    # Reverse direction (CCW instead of CW).
    python3 scripts/perimeter_patrol.py --ccw

    # Show plan without sending goals (works without ROS sourced).
    python3 scripts/perimeter_patrol.py --dry-run

Stop with Ctrl+C; the in-flight Nav2 goal will be cancelled and Go2
will stop where it is.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[1]


# Warehouse extents in MAP frame. world_to_map static TF in
# tf_and_scan.launch.py shifts world by (-4, -4) into map, so the
# warehouse [-5..+5] x [-5..+5] in world becomes [-1..+9] x [-1..+9]
# in map.
WAREHOUSE_MAP_X = (-1.0, 9.0)
WAREHOUSE_MAP_Y = (-1.0, 9.0)


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Pure-Z yaw -> (qx, qy, qz, qw)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _bearing(from_xy: Tuple[float, float], to_xy: Tuple[float, float]) -> float:
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])


def _build_waypoints(inset: float, dense: bool, ccw: bool, se_only: bool
                     ) -> List[Tuple[float, float, float]]:
    """Return [(x, y, yaw)] in MAP frame.

    Each yaw points toward the NEXT waypoint, so Nav2 naturally
    rotates Go2 to face its travel direction (good for the front
    camera).
    """
    x_lo = WAREHOUSE_MAP_X[0] + inset
    x_hi = WAREHOUSE_MAP_X[1] - inset
    y_lo = WAREHOUSE_MAP_Y[0] + inset
    y_hi = WAREHOUSE_MAP_Y[1] - inset

    if se_only:
        # The chair sits at map (7.5, 0.5) — exactly where the naive
        # x_hi/y_lo SE-corner waypoint would land. Planning Nav2 to
        # (7.5, 0.5) then fails synchronously with
        #   "Failed to create plan with tolerance of: 0.500000"
        # because the chair (+ inflation_radius 0.5m) eats every cell
        # in goal±0.5m. Instead, stop ~0.9m WEST of the chair so the
        # robot's front camera ends up looking at it. 0.9m matches
        # task_coordinator's approach_distance_chair default in
        # day8_two_phase.launch.py, so behaviour here is consistent
        # with what a 'go to chair' SemanticTask would produce.
        wx = x_hi - 0.9
        wy = y_lo
        return [(wx, wy, 0.0)]  # face +X, looking AT the chair

    # CW perimeter starting from SW (closest to spawn): SW -> SE -> NE -> NW -> SW.
    # CCW: SW -> NW -> NE -> SE -> SW.
    corners = [
        (x_lo, y_lo),  # SW
        (x_hi, y_lo),  # SE
        (x_hi, y_hi),  # NE
        (x_lo, y_hi),  # NW
    ]
    if ccw:
        corners = [corners[0], corners[3], corners[2], corners[1]]

    if dense:
        # Insert midpoints on every edge. Doubles waypoints to 8.
        dense_pts: List[Tuple[float, float]] = []
        for i in range(len(corners)):
            a = corners[i]
            b = corners[(i + 1) % len(corners)]
            mid = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
            dense_pts.append(a)
            dense_pts.append(mid)
        dense_pts.append(corners[0])  # close loop
        sequence = dense_pts
    else:
        sequence = corners + [corners[0]]  # close loop

    # Each waypoint's yaw = bearing toward the next; the very last one
    # reuses the previous yaw so we don't overshoot back to start.
    out: List[Tuple[float, float, float]] = []
    for i, (wx, wy) in enumerate(sequence):
        if i + 1 < len(sequence):
            yaw = _bearing((wx, wy), sequence[i + 1])
        elif out:
            yaw = out[-1][2]
        else:
            yaw = 0.0
        out.append((wx, wy, yaw))
    return out


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--inset", type=float, default=1.5,
                   help="Inset (m) from each wall when computing waypoints. "
                        "Default 1.5; bump to 2.0 if Nav2's inflation is fat.")
    p.add_argument("--dense", action="store_true",
                   help="Use 8 waypoints (corners + edge midpoints) instead "
                        "of 4. Slower but better wall coverage.")
    p.add_argument("--ccw", action="store_true",
                   help="Counter-clockwise order. Default is CW (SW->SE->NE->NW).")
    p.add_argument("--se-only", action="store_true",
                   help="Skip the lap; just drive to the SE corner where the "
                        "chair is and stop. Quick smoke test.")
    p.add_argument("--no-spin", action="store_true",
                   help="Do NOT do a 360 in-place spin at each waypoint.")
    p.add_argument("--spin-speed", type=float, default=0.6,
                   help="rad/s for the in-place spin at each waypoint.")
    p.add_argument("--frame", default="map",
                   help="Frame to stamp goals in. Should match Nav2's "
                        "global_frame (default 'map').")
    p.add_argument("--base-frame", default="base_link",
                   help="Robot base frame, used for the readiness "
                        "check (waits for `frame → base_frame` to "
                        "appear in tf2 before sending the first "
                        "goal). Default 'base_link'.")
    p.add_argument("--nav-action", default="/navigate_to_pose",
                   help="Nav2 action name (default /navigate_to_pose).")
    p.add_argument("--cmd-vel-topic", default="/cmd_vel",
                   help="Topic to publish in-place spin commands to.")
    p.add_argument("--action-timeout", type=float, default=15.0,
                   help="How long (s) to wait for Nav2 server before bailing.")
    p.add_argument("--goal-timeout", type=float, default=90.0,
                   help="Max time (s) per single waypoint before we cancel "
                        "and move to the next.")
    p.add_argument("--dry-run", action="store_true",
                   help="Print the waypoints, don't actually send goals.")
    p.add_argument("--use-sim-time", action="store_true", default=True,
                   help="(default true) Set use_sim_time on this client. "
                        "Use --no-use-sim-time on a real robot.")
    p.add_argument("--no-use-sim-time", dest="use_sim_time",
                   action="store_false")
    return p.parse_args()


def _print_plan(args: argparse.Namespace,
                waypoints: List[Tuple[float, float, float]]) -> None:
    print(f"[perimeter_patrol] Plan ({len(waypoints)} waypoints):")
    for i, (x, y, yaw) in enumerate(waypoints):
        print(f"  #{i}  x={x:+.2f}  y={y:+.2f}  yaw={math.degrees(yaw):+5.0f}deg")
    print(f"  inset={args.inset}m  dense={args.dense}  ccw={args.ccw}  "
          f"se_only={args.se_only}  "
          f"spin_at_each={'no' if args.no_spin else 'yes'}")


def _run_with_ros(args: argparse.Namespace,
                  waypoints: List[Tuple[float, float, float]]) -> int:
    """All ROS-touching code lives here so --dry-run works without
    /opt/ros/<distro>/setup.bash sourced."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from tf2_ros import Buffer, TransformListener

    # #region agent log — debug instrumentation (Session e08657)
    import json as _agent_json
    _AGENT_DEBUG_LOG = str(_REPO_ROOT / ".cursor" / "debug-e08657.log")
    _agent_log_warned = {"flag": False}

    def _agent_dlog(hypothesis_id: str, message: str, data: dict) -> None:
        try:
            entry = {
                "sessionId": "e08657",
                "id": f"log_{int(time.time()*1000)}_{hypothesis_id}",
                "timestamp": int(time.time() * 1000),
                "location": "perimeter_patrol.py",
                "hypothesisId": hypothesis_id,
                "message": message,
                "data": data,
            }
            with open(_AGENT_DEBUG_LOG, "a") as _f:
                _f.write(_agent_json.dumps(entry) + "\n")
        except Exception as _exc:
            if not _agent_log_warned["flag"]:
                _agent_log_warned["flag"] = True
                print(
                    f"[perimeter_patrol/DEBUG] !! NDJSON write failed: "
                    f"{_exc!r}. log_path={_AGENT_DEBUG_LOG}",
                    file=sys.stderr,
                    flush=True,
                )

    print(
        f"[perimeter_patrol/DEBUG] instrumentation v2 active "
        f"(session=e08657). Writing NDJSON to: {_AGENT_DEBUG_LOG}",
        flush=True,
    )
    _agent_dlog(
        "H9",
        "script_startup_marker",
        {
            "log_path": _AGENT_DEBUG_LOG,
            "argv": sys.argv,
            "wallclock_unix": time.time(),
        },
    )
    # #endregion

    class _PatrolClient(Node):
        def __init__(self) -> None:
            super().__init__("perimeter_patrol")
            self._nav_client = ActionClient(
                self, NavigateToPose, args.nav_action
            )
            self._cmd_pub = self.create_publisher(
                Twist, args.cmd_vel_topic, 10
            )
            # tf2 listener used ONLY to confirm Nav2/SLAM is far
            # enough through its lifecycle that we shouldn't get
            # 'Action server is inactive. Rejecting the goal.'
            # back from bt_navigator the moment we send_goal_async.
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.cancelled = False

            # #region agent log — debug instrumentation (Session e08657)
            from rclpy.qos import (
                QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
            )
            self._last_cmd_vel_nav = None       # Twist from controller
            self._last_cmd_vel = None           # Twist as sim sees it
            self._cmd_vel_nav_count = 0
            self._cmd_vel_count = 0
            self._last_cmd_vel_nav_log_t = 0.0
            self._last_cmd_vel_log_t = 0.0
            qos = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
            )

            def _on_cmd_vel_nav(msg: Twist) -> None:
                self._last_cmd_vel_nav = msg
                self._cmd_vel_nav_count += 1
                now_t = time.time()
                # Throttle to 2 Hz to avoid log explosion.
                if now_t - self._last_cmd_vel_nav_log_t > 0.5:
                    self._last_cmd_vel_nav_log_t = now_t
                    _agent_dlog(
                        "H4",
                        "cmd_vel_nav (controller_server -> velocity_smoother)",
                        {
                            "linear_x": float(msg.linear.x),
                            "linear_y": float(msg.linear.y),
                            "angular_z": float(msg.angular.z),
                            "msg_count": self._cmd_vel_nav_count,
                        },
                    )

            def _on_cmd_vel(msg: Twist) -> None:
                self._last_cmd_vel = msg
                self._cmd_vel_count += 1
                now_t = time.time()
                if now_t - self._last_cmd_vel_log_t > 0.5:
                    self._last_cmd_vel_log_t = now_t
                    _agent_dlog(
                        "H2H4",
                        "cmd_vel (post-collision_monitor, what sim receives)",
                        {
                            "linear_x": float(msg.linear.x),
                            "linear_y": float(msg.linear.y),
                            "angular_z": float(msg.angular.z),
                            "msg_count": self._cmd_vel_count,
                        },
                    )

            self._sub_cmd_vel_nav = self.create_subscription(
                Twist, "/cmd_vel_nav", _on_cmd_vel_nav, qos
            )
            self._sub_cmd_vel = self.create_subscription(
                Twist, "/cmd_vel", _on_cmd_vel, qos
            )

            # 1 Hz timer: log TF freshness + robot pose for H1 / H5.
            self._tf_check_timer = self.create_timer(
                1.0, self._agent_tf_health_tick
            )

            # H6: Is /lidar/points arriving with a stale stamp vs sim
            # 'now'? If lidar_lag exceeds slam's TF post-date offset,
            # global_costmap's message_filter will drop every scan
            # ("the timestamp on the message is earlier than all the
            # data in the transform cache"), which is what terminal
            # 10 has been showing nonstop.
            from sensor_msgs.msg import PointCloud2
            from rclpy.qos import qos_profile_sensor_data
            self._lidar_lag_count = 0
            self._lidar_lag_log_t = 0.0

            def _on_lidar(msg: "PointCloud2") -> None:
                now_ros = self.get_clock().now()
                now_sec = now_ros.nanoseconds * 1e-9
                stamp_sec = (
                    msg.header.stamp.sec
                    + msg.header.stamp.nanosec * 1e-9
                )
                lag = now_sec - stamp_sec
                self._lidar_lag_count += 1
                wall_t = time.time()
                if wall_t - self._lidar_lag_log_t > 1.0:
                    self._lidar_lag_log_t = wall_t
                    _agent_dlog(
                        "H6",
                        "lidar_lag_vs_sim_now",
                        {
                            "frame_id": msg.header.frame_id,
                            "stamp_sec": stamp_sec,
                            "sim_now_sec": now_sec,
                            "lidar_lag_seconds": lag,
                            "msg_count": self._lidar_lag_count,
                        },
                    )

            self._sub_lidar = self.create_subscription(
                PointCloud2,
                "/lidar/points",
                _on_lidar,
                qos_profile_sensor_data,
            )

            # H7: Does slam_toolbox publish map->odom with stamp =
            # last_scan_time + transform_publish_period, or with
            # stamp = NOW + transform_publish_period? If lidar's
            # stamp lags real /clock and slam uses NOW, lidar will
            # always be < TF cache earliest entry.
            from tf2_msgs.msg import TFMessage
            self._tf_map_odom_log_t = 0.0
            # H10 watchdog: detect the precise moment slam_toolbox
            # stops emitting fresh stamps so we can correlate with
            # other events (cmd_vel transition, robot motion start).
            self._tf_map_odom_last_stamp = None
            self._tf_map_odom_last_change_wall = time.time()
            self._tf_map_odom_freeze_logged = False

            def _on_tf(msg: "TFMessage") -> None:
                now_sec = (
                    self.get_clock().now().nanoseconds * 1e-9
                )
                wall_t = time.time()
                for tr in msg.transforms:
                    if (tr.header.frame_id == "map"
                            and tr.child_frame_id == "odom"):
                        # H12: count every map->odom message we
                        # observe so the 200 ms rate tick can
                        # measure slam_toolbox's TF publish rate
                        # independent of whether stamp updates.
                        self._tf_map_odom_count += 1
                        stamp_sec = (
                            tr.header.stamp.sec
                            + tr.header.stamp.nanosec * 1e-9
                        )
                        if (self._tf_map_odom_last_stamp is None
                                or stamp_sec
                                != self._tf_map_odom_last_stamp):
                            self._tf_map_odom_last_stamp = stamp_sec
                            self._tf_map_odom_last_change_wall = wall_t
                            self._tf_map_odom_freeze_logged = False
                        else:
                            stuck_for = (
                                wall_t
                                - self._tf_map_odom_last_change_wall
                            )
                            if (stuck_for > 3.0
                                    and not self._tf_map_odom_freeze_logged
                                    ):
                                self._tf_map_odom_freeze_logged = True
                                _agent_dlog(
                                    "H10",
                                    "slam_tf_freeze_detected",
                                    {
                                        "frozen_at_stamp_sec": stamp_sec,
                                        "freeze_duration_wall_seconds": (
                                            stuck_for
                                        ),
                                        "sim_now_sec": now_sec,
                                        "stamp_minus_now": (
                                            stamp_sec - now_sec
                                        ),
                                    },
                                )
                        if wall_t - self._tf_map_odom_log_t > 1.0:
                            self._tf_map_odom_log_t = wall_t
                            _agent_dlog(
                                "H7",
                                "tf_map_odom_published",
                                {
                                    "stamp_sec": stamp_sec,
                                    "sim_now_sec": now_sec,
                                    "stamp_minus_now": (
                                        stamp_sec - now_sec
                                    ),
                                },
                            )
                        return

            self._sub_tf = self.create_subscription(
                TFMessage, "/tf", _on_tf, qos
            )

            # H11: every /map publish from slam_toolbox holds the worker
            # mutex while serialising the OccupancyGrid. If the grid has
            # grown large, this period correlates with the H10 freeze
            # window. We log every /map message we observe so we can
            # cross-reference its arrival times with H7 stamp updates.
            from nav_msgs.msg import OccupancyGrid
            from rclpy.qos import (
                QoSProfile as _QoSProfile,
                DurabilityPolicy as _Dura,
                ReliabilityPolicy as _Rel,
                HistoryPolicy as _Hist,
            )
            self._map_msg_count = 0

            def _on_map(msg: "OccupancyGrid") -> None:
                self._map_msg_count += 1
                wall_t = time.time()
                now_sec = (
                    self.get_clock().now().nanoseconds * 1e-9
                )
                stamp_sec = (
                    msg.header.stamp.sec
                    + msg.header.stamp.nanosec * 1e-9
                )
                _agent_dlog(
                    "H11",
                    "map_published",
                    {
                        "msg_count": self._map_msg_count,
                        "wall_t": wall_t,
                        "sim_now_sec": now_sec,
                        "stamp_sec": stamp_sec,
                        "width": int(msg.info.width),
                        "height": int(msg.info.height),
                        "resolution": float(msg.info.resolution),
                        "origin_xy": [
                            float(msg.info.origin.position.x),
                            float(msg.info.origin.position.y),
                        ],
                        "n_cells": int(msg.info.width)
                        * int(msg.info.height),
                    },
                )

            map_qos = _QoSProfile(
                depth=1,
                reliability=_Rel.RELIABLE,
                durability=_Dura.TRANSIENT_LOCAL,
                history=_Hist.KEEP_LAST,
            )
            self._sub_map = self.create_subscription(
                OccupancyGrid, "/map", _on_map, map_qos
            )

            # H12: Did Isaac Sim stop publishing /lidar/points (or
            # /scan, or /odom) when the robot started rotating? If
            # /lidar/points goes from ~5 Hz to 0 Hz at the same wall
            # second the H7 stamp freezes, the cause is sim-side data
            # starvation, NOT slam_toolbox. We wire dedicated counters
            # for each upstream stream and a 200 ms timer that logs
            # delta-counts so we can see the exact moment any of them
            # stalls — independent of whether the per-message callback
            # gets to run on this executor.
            from sensor_msgs.msg import LaserScan
            from nav_msgs.msg import Odometry
            self._scan_count = 0
            self._scan_last_stamp = 0.0
            self._odom_count = 0
            self._odom_last_stamp = 0.0
            self._tf_map_odom_count = 0

            def _on_scan(msg: "LaserScan") -> None:
                self._scan_count += 1
                self._scan_last_stamp = (
                    msg.header.stamp.sec
                    + msg.header.stamp.nanosec * 1e-9
                )

            def _on_odom(msg: "Odometry") -> None:
                self._odom_count += 1
                self._odom_last_stamp = (
                    msg.header.stamp.sec
                    + msg.header.stamp.nanosec * 1e-9
                )

            self._sub_scan = self.create_subscription(
                LaserScan, "/scan", _on_scan, qos_profile_sensor_data
            )
            self._sub_odom = self.create_subscription(
                Odometry, "/odom", _on_odom, qos_profile_sensor_data
            )

            self._h12_last_lidar_count = 0
            self._h12_last_scan_count = 0
            self._h12_last_odom_count = 0
            self._h12_last_tf_count = 0
            self._h12_last_wall = time.time()

            def _h12_rate_tick() -> None:
                wall_t = time.time()
                dt = wall_t - self._h12_last_wall
                if dt < 1e-6:
                    return
                d_lidar = (
                    self._lidar_lag_count - self._h12_last_lidar_count
                )
                d_scan = (
                    self._scan_count - self._h12_last_scan_count
                )
                d_odom = (
                    self._odom_count - self._h12_last_odom_count
                )
                d_tf = (
                    self._tf_map_odom_count - self._h12_last_tf_count
                )
                self._h12_last_lidar_count = self._lidar_lag_count
                self._h12_last_scan_count = self._scan_count
                self._h12_last_odom_count = self._odom_count
                self._h12_last_tf_count = self._tf_map_odom_count
                self._h12_last_wall = wall_t
                now_sec = (
                    self.get_clock().now().nanoseconds * 1e-9
                )
                _agent_dlog(
                    "H12",
                    "topic_rate_tick",
                    {
                        "wall_dt": dt,
                        "sim_now_sec": now_sec,
                        "lidar_points_hz": d_lidar / dt,
                        "scan_hz": d_scan / dt,
                        "odom_hz": d_odom / dt,
                        "tf_map_odom_hz": d_tf / dt,
                        "lidar_count_total": self._lidar_lag_count,
                        "scan_count_total": self._scan_count,
                        "odom_count_total": self._odom_count,
                        "tf_count_total": self._tf_map_odom_count,
                        "scan_latest_stamp": self._scan_last_stamp,
                        "odom_latest_stamp": self._odom_last_stamp,
                    },
                )

            self._h12_rate_timer = self.create_timer(
                0.2, _h12_rate_tick
            )
            # #endregion

        # #region agent log — debug instrumentation (Session e08657)
        def _agent_tf_health_tick(self) -> None:
            """Periodic TF & pose health probe (debug instrumentation)."""
            try:
                now_ros = self.get_clock().now()
                # Try lookup at "now" (latest available)
                from rclpy.time import Time as _RclpyTime
                tf_ok_latest = self._tf_buffer.can_transform(
                    "map", "base_link", _RclpyTime()
                )
                tf_ok_now = self._tf_buffer.can_transform(
                    "map", "base_link", now_ros
                )
                pose_xy = None
                lag_seconds = None
                if tf_ok_latest:
                    tf_msg = self._tf_buffer.lookup_transform(
                        "map", "base_link", _RclpyTime()
                    )
                    tx = tf_msg.transform.translation.x
                    ty = tf_msg.transform.translation.y
                    pose_xy = (float(tx), float(ty))
                    stamp_sec = (
                        tf_msg.header.stamp.sec
                        + tf_msg.header.stamp.nanosec * 1e-9
                    )
                    now_sec = now_ros.nanoseconds * 1e-9
                    lag_seconds = float(now_sec - stamp_sec)
                _agent_dlog(
                    "H1H5",
                    "tf_health_probe",
                    {
                        "can_transform_latest": bool(tf_ok_latest),
                        "can_transform_at_now": bool(tf_ok_now),
                        "robot_pose_map_xy": pose_xy,
                        "tf_lag_seconds": lag_seconds,
                    },
                )
            except Exception as e:
                _agent_dlog(
                    "H1H5",
                    "tf_health_probe_exception",
                    {"err": repr(e)},
                )
        # #endregion

        def wait_for_nav(self) -> bool:
            """Wait until both:
              1. The Nav2 action server topic is discoverable, AND
              2. tf2 has at least one `map → base_link` transform
                 in its buffer (so we know slam_toolbox has emitted
                 map→odom and bt_navigator has finished its
                 lifecycle Activate transition).

            We learned the hard way that wait_for_server() returns
            true the moment the action *topic* is discovered, well
            before bt_navigator's lifecycle node hits ACTIVE. Goals
            sent in that gap come back with handle.accepted=False
            and the bt_navigator log shows 'Action server is
            inactive. Rejecting the goal.' Waiting for the TF
            chain to be ready empirically gates the goal until
            Nav2 is genuinely ready to accept it.
            """
            self.get_logger().info(
                f"Waiting up to {args.action_timeout:.0f}s for "
                f"{args.nav_action} ..."
            )
            if not self._nav_client.wait_for_server(
                timeout_sec=args.action_timeout
            ):
                self.get_logger().error(
                    f"Nav2 action server {args.nav_action!r} not "
                    f"available. Is nav2.launch.py running?"
                )
                return False

            self.get_logger().info(
                f"Waiting up to {args.action_timeout:.0f}s for "
                f"the TF chain '{args.frame}' → '{args.base_frame}' "
                f"to come up (slam_toolbox publishing map→odom + "
                f"bt_navigator lifecycle Activate finished) ..."
            )
            deadline = time.monotonic() + args.action_timeout
            while time.monotonic() < deadline:
                if self._tf_buffer.can_transform(
                    args.frame, args.base_frame, rclpy.time.Time()
                ):
                    self.get_logger().info(
                        f"TF '{args.frame}' → '{args.base_frame}' "
                        f"is up. Nav2 should now accept goals."
                    )
                    return True
                rclpy.spin_once(self, timeout_sec=0.2)
            self.get_logger().error(
                f"TF '{args.frame}' → '{args.base_frame}' did NOT "
                f"come up within {args.action_timeout:.0f}s. Either "
                f"slam_toolbox isn't publishing map→odom, or "
                f"bt_navigator never finished lifecycle Activate. "
                f"Check the nav2.launch.py terminal for "
                f"'Managed nodes are active'."
            )
            return False

        def send_goal(self, x: float, y: float, yaw: float) -> int:
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = args.frame
            # Use sim_time 'now' (the standard Nav2 idiom). The earlier
            # workaround of (0, 0) was meant to dodge a stamp/TF race, but
            # H7 disproved that hypothesis: the run-time logs show the
            # goal is *accepted* and even *planned* (distance_remaining
            # 2.17m, controller starts driving) — the abort happens 15 s
            # later when slam_toolbox stalls and tf2 cache for map→base
            # expires. So the original stamp is fine.
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.position.z = 0.0
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            goal.pose.pose.orientation.x = qx
            goal.pose.pose.orientation.y = qy
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw

            self.get_logger().info(
                f"-> nav goal x={x:.2f} y={y:.2f} "
                f"yaw={math.degrees(yaw):.0f}deg"
            )

            # #region agent log — debug instrumentation (Session e08657)
            _agent_dlog(
                "H_GOAL",
                "goal_sent",
                {
                    "x": float(x), "y": float(y),
                    "yaw_deg": float(math.degrees(yaw)),
                },
            )

            def _on_feedback(feedback_msg):
                fb = feedback_msg.feedback
                # NavigateToPose.Feedback has:
                #   current_pose, navigation_time, estimated_time_remaining,
                #   number_of_recoveries, distance_remaining
                cp = fb.current_pose.pose
                _agent_dlog(
                    "H3H5",
                    "nav2_feedback",
                    {
                        "current_xy": [
                            float(cp.position.x), float(cp.position.y)
                        ],
                        "distance_remaining": float(
                            fb.distance_remaining
                        ),
                        "number_of_recoveries": int(
                            fb.number_of_recoveries
                        ),
                        "navigation_time_sec": float(
                            fb.navigation_time.sec
                            + fb.navigation_time.nanosec * 1e-9
                        ),
                    },
                )
            # #endregion

            send_future = self._nav_client.send_goal_async(
                goal, feedback_callback=_on_feedback
            )
            rclpy.spin_until_future_complete(self, send_future)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                self.get_logger().warn("Nav2 rejected goal")
                # #region agent log — debug instrumentation (Session e08657)
                _agent_dlog(
                    "H_GOAL",
                    "goal_rejected",
                    {"handle_is_none": handle is None},
                )
                # #endregion
                return GoalStatus.STATUS_ABORTED

            # #region agent log — debug instrumentation (Session e08657)
            _agent_dlog("H_GOAL", "goal_accepted", {})
            # #endregion

            result_future = handle.get_result_async()
            # Deadline is intentionally measured against wall clock
            # (time.monotonic), NOT self.get_clock(). The node has
            # use_sim_time=True, and Isaac Sim's /clock can lag or jump
            # at startup; using sim time here was racing the TimeSource
            # handshake and firing "exceeded 90s" within ~1ms of sending
            # the goal because sim time leapt past the deadline on the
            # very first spin_once.
            deadline = time.monotonic() + args.goal_timeout
            try:
                while rclpy.ok() and not result_future.done():
                    rclpy.spin_once(self, timeout_sec=0.2)
                    if time.monotonic() > deadline:
                        self.get_logger().warn(
                            f"Goal exceeded {args.goal_timeout:.0f}s "
                            f"(wall) — cancelling and moving to next "
                            f"waypoint."
                        )
                        handle.cancel_goal_async()
                        t0 = time.monotonic()
                        while (not result_future.done()
                               and time.monotonic() - t0 < 3.0):
                            rclpy.spin_once(self, timeout_sec=0.1)
                        break
            except KeyboardInterrupt:
                self.cancelled = True
                self.get_logger().warn("Ctrl+C: cancelling current goal.")
                handle.cancel_goal_async()
                t0 = time.monotonic()
                while (not result_future.done()
                       and time.monotonic() - t0 < 3.0):
                    rclpy.spin_once(self, timeout_sec=0.1)

            wrapper = result_future.result() if result_future.done() else None
            final_status = (
                wrapper.status if wrapper else GoalStatus.STATUS_UNKNOWN
            )
            # #region agent log — debug instrumentation (Session e08657)
            _agent_dlog(
                "H_GOAL",
                "goal_finished",
                {
                    "status_code": int(final_status),
                    "status_name": {
                        0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
                        3: "CANCELING", 4: "SUCCEEDED",
                        5: "CANCELED", 6: "ABORTED",
                    }.get(int(final_status), str(final_status)),
                    "result_has_wrapper": wrapper is not None,
                },
            )
            # #endregion
            return final_status

        def spin_in_place(self, total_rad: float, ang_speed: float) -> None:
            if self.cancelled or not rclpy.ok():
                return
            if total_rad <= 0.0 or ang_speed <= 0.0:
                return
            duration_sec = abs(total_rad) / ang_speed
            self.get_logger().info(
                f"   spin {math.degrees(total_rad):.0f}deg @ "
                f"{ang_speed:.2f} rad/s ({duration_sec:.1f}s)"
            )
            twist = Twist()
            twist.angular.z = ang_speed if total_rad > 0 else -ang_speed
            rate_hz = 20.0
            end_t = time.time() + duration_sec
            try:
                while time.time() < end_t and rclpy.ok():
                    self._cmd_pub.publish(twist)
                    rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)
            except KeyboardInterrupt:
                self.cancelled = True

            zero = Twist()
            for _ in range(5):
                self._cmd_pub.publish(zero)
                rclpy.spin_once(self, timeout_sec=0.05)

    rclpy.init()
    node = _PatrolClient()
    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", value=True)])

    try:
        if not node.wait_for_nav():
            return 2

        n_succeeded = 0
        n_failed = 0
        for i, (x, y, yaw) in enumerate(waypoints):
            if not rclpy.ok() or node.cancelled:
                break
            print(f"[perimeter_patrol] === waypoint "
                  f"{i + 1}/{len(waypoints)} ===")
            status = node.send_goal(x, y, yaw)
            if status == GoalStatus.STATUS_SUCCEEDED:
                n_succeeded += 1
                print("   SUCCEEDED")
                if not args.no_spin and not args.se_only:
                    node.spin_in_place(2.0 * math.pi, args.spin_speed)
            elif status == GoalStatus.STATUS_CANCELED:
                print("   CANCELED (timeout or Ctrl+C); moving on")
                n_failed += 1
            else:
                print(f"   ABORTED/UNKNOWN (status={status}); moving on")
                n_failed += 1

        print(f"[perimeter_patrol] DONE. ok={n_succeeded} fail={n_failed}")
        return 0 if n_succeeded > 0 else 1
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main() -> int:
    args = _parse_args()
    waypoints = _build_waypoints(
        args.inset, args.dense, args.ccw, args.se_only
    )
    _print_plan(args, waypoints)

    if args.dry_run:
        print("[perimeter_patrol] --dry-run: not sending goals.")
        return 0

    return _run_with_ros(args, waypoints)


if __name__ == "__main__":
    sys.exit(main())
