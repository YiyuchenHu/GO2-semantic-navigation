#!/usr/bin/env python3
"""Smoke test for scripts/record_mapping_debug.py.

Boots the recorder + a tiny synthetic publisher inside one rclpy
process under an isolated ROS_DOMAIN_ID. Drives the system through a
canned "drive towards goal then arrive" trajectory, then verifies:

  * both CSV files exist and have the expected headers
  * row counts roughly match the requested rates × duration
  * dist_to_goal monotonically decreases through the run
  * angular.z sign-flip count is captured
  * the recorder summary reports DONE reached
  * /cmd_vel rate estimate is in the right ballpark (5-15 Hz)

Run with:

    python3 scripts/_smoke_record_mapping_debug.py
"""
from __future__ import annotations

import csv
import io
import os
import sys
import tempfile
import time
from contextlib import redirect_stdout
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = os.environ.get(
    "SMOKE_ROS_DOMAIN_ID", "215"
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

# Import after sys.path tweak.
import record_mapping_debug as rmd  # noqa: E402


class _Driver(Node):
    """Pretends to be the day8_two_phase stack: publishes /mapping/status
    + /semantic_goal/goal_pose + /cmd_vel + TF map->base_link.

    Trajectory: robot starts at (0,0), goal fixed at (3,0). Linear
    velocity 0.5 m/s. Angular velocity oscillates ±0.4 rad/s every
    0.4s (induces sign flips so the wave-like detector fires). After
    ~6 s arrives at goal — publishes "DONE".
    """

    def __init__(self) -> None:
        super().__init__("smoke_driver")
        from rclpy.qos import (
            QoSProfile,
            ReliabilityPolicy,
            DurabilityPolicy,
            HistoryPolicy,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._mapping_status_pub = self.create_publisher(
            String, "/mapping/status", status_qos
        )
        self._mapping_dbg_pub = self.create_publisher(
            String, "/mapping/debug/status", status_qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, "/semantic_goal/goal_pose", 10
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._tf_br = TransformBroadcaster(self)

        self._t0 = time.time()
        # 12 Hz cmd_vel — well within the 5..15 expected band.
        self.create_timer(1.0 / 12.0, self._tick_cmd_vel)
        # 5 Hz status so the smoke sees the IDLE -> NAV -> DONE
        # transitions inside the short 6 s window.
        self.create_timer(0.2, self._tick_status)
        self._x = 0.0
        # Short goal so the synthetic robot reaches DONE in < 4 s
        # (1.0 m at 0.4 m/s ≈ 2.5 s).
        self._goal = (1.0, 0.0)
        self._done = False
        # First publish so TRANSIENT_LOCAL gets latched value.
        self._publish_goal()
        self._tick_status()

    # ------------------------------------------------------------------
    def _publish_goal(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self._goal[0])
        msg.pose.position.y = float(self._goal[1])
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _tick_status(self) -> None:
        elapsed = time.time() - self._t0
        if elapsed < 0.5:
            verb = "IDLE"
        elif self._done:
            verb = "DONE"
        else:
            verb = "NAVIGATING"
        msg = String()
        msg.data = verb
        self._mapping_status_pub.publish(msg)
        # Mirror real mapping_explorer behaviour:
        #   * while driving — in_flight=1, real goal echoed.
        #   * after arrival — in_flight=0, goal=none, nav2=SUCCEEDED,
        #     reason=no_frontiers_returned. This exercises the
        #     recorder's new "settled between goals" path so a
        #     legitimately-zero cmd_vel after DONE is NOT flagged
        #     as "stalling".
        dbg = String()
        if self._done:
            dbg.data = (
                f"state={verb} goal=none dist=nan "
                "nav2=SUCCEEDED in_flight=0 visited=1(1_active) "
                "frontiers=0 selected=-@- reason=no_frontiers_returned"
            )
        else:
            dbg.data = (
                f"state={verb} goal=(1.00,0.00) "
                f"dist={max(0.0, self._goal[0] - self._x):.2f} "
                "nav2=ACCEPTED in_flight=1 visited=0(0_active) "
                "frontiers=3 selected=0@99.0 reason=picked#0"
            )
        self._mapping_dbg_pub.publish(dbg)
        self._publish_goal()

    def _tick_cmd_vel(self) -> None:
        elapsed = time.time() - self._t0
        if self._done:
            tw = Twist()  # zero
            self._cmd_pub.publish(tw)
            self._publish_tf()
            return
        # Move forward at 0.4 m/s.
        dt = 1.0 / 12.0
        self._x = min(self._goal[0], self._x + 0.4 * dt)
        if self._x >= self._goal[0] - 1e-3:
            self._done = True
        # Oscillating angular.z — flip every 0.12 s so we get
        # ~8 flips/s, well above the wave-like detector's >=4/s
        # threshold.
        sign = 1.0 if int(elapsed / 0.12) % 2 == 0 else -1.0
        tw = Twist()
        tw.linear.x = 0.4
        tw.angular.z = sign * 0.4
        self._cmd_pub.publish(tw)
        self._publish_tf()

    def _publish_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self._x)
        t.transform.translation.y = 0.0
        t.transform.rotation.w = 1.0
        self._tf_br.sendTransform(t)


def main() -> int:
    rclpy.init()
    tmp_dir = Path(tempfile.mkdtemp(prefix="smoke_record_"))
    print(f"[smoke] tmp_dir={tmp_dir}")

    # Boot the recorder for 6 seconds of synthetic data.
    duration = 6.0
    recorder = rmd._MappingDebugRecorder(
        status_rate_hz=2.0,        # bump to 2 Hz so 6 s -> ~12 rows
        control_rate_hz=10.0,
        duration_sec=duration,
        output_dir=tmp_dir,
    )
    driver = _Driver()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(driver)

    # Spin until the recorder expires.
    end = time.time() + duration + 1.0
    while time.time() < end and not recorder.expired():
        executor.spin_once(timeout_sec=0.05)

    # Capture the summary as a string for assertions.
    buf = io.StringIO()
    with redirect_stdout(buf):
        recorder._print_summary()
    summary = buf.getvalue()
    print(summary)

    recorder.shutdown()
    recorder.destroy_node()
    driver.destroy_node()
    rclpy.shutdown()

    # ---- Assertions on CSVs ---------------------------------------------
    status_csvs = sorted(tmp_dir.glob("mapping_debug_status_*.csv"))
    control_csvs = sorted(tmp_dir.glob("mapping_debug_control_*.csv"))
    if len(status_csvs) != 1:
        print(f"[FAIL] expected 1 status CSV, got {status_csvs}")
        return 1
    if len(control_csvs) != 1:
        print(f"[FAIL] expected 1 control CSV, got {control_csvs}")
        return 1
    print(f"[OK] CSVs: {status_csvs[0].name} / {control_csvs[0].name}")

    # Use csv.reader to handle quoted cells — the mapping_debug_status
    # column legitimately contains commas, so naive .split(",") would
    # mis-align the columns.
    with status_csvs[0].open() as fh:
        status_rows = list(csv.reader(fh))
    with control_csvs[0].open() as fh:
        control_rows = list(csv.reader(fh))
    print(
        f"[OK] row counts: status={len(status_rows) - 1} "
        f"control={len(control_rows) - 1}"
    )
    if len(status_rows) < 4:
        print("[FAIL] status CSV has < 4 rows (header + 3+ samples)")
        return 1
    if len(control_rows) < 30:
        print("[FAIL] control CSV has < 30 rows (10Hz × 6s expected ~60)")
        return 1

    expected_status_cols = [
        "timestamp_iso", "t_rel_sec",
        "mapping_status", "mapping_debug_status", "task_status",
        "navigation_status", "arrival_status",
        "target_entity_id", "target_class", "target_reachable",
        "goal_x", "goal_y", "robot_x", "robot_y", "dist_to_goal",
        "notes",
    ]
    if status_rows[0] != expected_status_cols:
        print(f"[FAIL] status header mismatch:\n  {status_rows[0]}")
        return 1
    print("[OK] status header matches expected schema")
    expected_control_cols = [
        "timestamp_iso", "t_rel_sec", "mapping_state",
        "nav_lin_x", "nav_lin_y", "nav_ang_z",
        "smoothed_lin_x", "smoothed_lin_y", "smoothed_ang_z",
        "cmd_lin_x", "cmd_lin_y", "cmd_ang_z",
        "odom_x", "odom_y", "odom_lin_x", "odom_ang_z",
        "nav_msg_age_sec", "smoothed_msg_age_sec",
        "cmd_msg_age_sec", "odom_msg_age_sec",
    ]
    if control_rows[0] != expected_control_cols:
        print(f"[FAIL] control header mismatch:\n  {control_rows[0]}")
        return 1
    print("[OK] control header matches expected schema")

    dist_idx = expected_status_cols.index("dist_to_goal")
    first_dist = None
    last_dist = None
    for parts in status_rows[1:]:
        if len(parts) > dist_idx and parts[dist_idx]:
            d = float(parts[dist_idx])
            if first_dist is None:
                first_dist = d
            last_dist = d
    if first_dist is None or last_dist is None:
        print("[FAIL] no dist_to_goal samples in CSV")
        return 1
    print(f"[OK] dist_to_goal: first={first_dist:.2f} last={last_dist:.2f}")
    if last_dist >= first_dist - 0.5:
        print(f"[FAIL] dist did not decrease (first={first_dist} last={last_dist})")
        return 1
    print("[OK] dist_to_goal decreased over the run")

    # Summary checks
    if "reached_DONE       : yes" not in summary:
        print("[FAIL] summary missing 'reached_DONE: yes'")
        return 1
    print("[OK] summary reports DONE reached")
    if "dist_decreasing    : yes" not in summary:
        print("[FAIL] summary missing 'dist_decreasing: yes'")
        return 1
    print("[OK] summary reports dist_decreasing: yes")
    if "wave-like driving   : YES" not in summary:
        print("[FAIL] expected wave-like detection given oscillating ang.z")
        return 1
    print("[OK] summary detected wave-like driving")
    # /cmd_vel rate roughly 12 Hz -> accept 5..15
    import re
    m = re.search(r"cmd_vel\s+:\s*~([0-9.]+)\s*Hz", summary)
    if m is None:
        print("[FAIL] could not parse cmd_vel rate from summary")
        return 1
    rate = float(m.group(1))
    if not (5.0 <= rate <= 15.0):
        print(f"[FAIL] cmd_vel rate {rate} outside 5..15 Hz band")
        return 1
    print(f"[OK] cmd_vel rate estimate {rate:.2f} Hz")
    # cmd_vel_nav and cmd_vel_smoothed should be silent (we never published)
    if "cmd_vel_nav         : silent" not in summary:
        print("[FAIL] expected cmd_vel_nav silent in summary")
        return 1
    print("[OK] missing topics reported as silent (no crash)")

    # state_changes should include IDLE -> NAVIGATING and NAVIGATING -> DONE
    if "IDLE -> NAVIGATING" not in summary:
        print("[FAIL] expected state change IDLE -> NAVIGATING")
        return 1
    if "NAVIGATING -> DONE" not in summary:
        print("[FAIL] expected state change NAVIGATING -> DONE")
        return 1
    print("[OK] state_changes captured both IDLE -> NAVIGATING and -> DONE")

    # ---- Day 9+ Phase B Task 4 — mapping_debug_dist parsed -------------
    m = re.search(
        r"mapping_debug_dist : first=([\d.]+|n/a)m? "
        r"last=([\d.]+|n/a)m? min=([\d.]+|n/a)m?",
        summary,
    )
    if m is None:
        print("[FAIL] summary missing 'mapping_debug_dist' line")
        return 1
    dbg_first = m.group(1)
    dbg_last = m.group(2)
    if dbg_first == "n/a" or dbg_last == "n/a":
        print(f"[FAIL] mapping_debug_dist not parsed (first={dbg_first} "
              f"last={dbg_last}); expected real numbers")
        return 1
    if float(dbg_last) > float(dbg_first) - 0.3:
        print(
            f"[FAIL] mapping_debug_dist did not decrease "
            f"(first={dbg_first} last={dbg_last})"
        )
        return 1
    print(f"[OK] mapping_debug_dist decreased: first={dbg_first} "
          f"last={dbg_last}")
    if "recorder_tf_dist   :" not in summary:
        print("[FAIL] summary missing 'recorder_tf_dist' line")
        return 1
    print("[OK] recorder_tf_dist line present alongside upstream dist")

    # ---- Day 9+ Phase B Task 3 — in_flight=1 coverage ------------------
    m = re.search(
        r"cmd_vel during in_flight=1: ([\d.]+)% active",
        summary,
    )
    if m is None:
        print("[FAIL] summary missing 'cmd_vel during in_flight=1' line")
        return 1
    inflight_pct = float(m.group(1))
    # Driver kept in_flight=1 throughout the moving phase and pushed
    # non-zero cmd_vel; expect >50% active.
    if inflight_pct < 50.0:
        print(f"[FAIL] in_flight=1 active ratio {inflight_pct}% < 50%")
        return 1
    print(f"[OK] in_flight=1 active ratio {inflight_pct:.1f}%")
    if "longest zero (in_flight=1):" not in summary:
        print("[FAIL] summary missing 'longest zero (in_flight=1)' line")
        return 1
    print("[OK] longest zero (in_flight=1) line present")

    # The driver flips to in_flight=0 / goal=none after arrival, so
    # the post-DONE NAVIGATING-rows-with-zero-cmd should NOT be
    # labeled "stalling". The string "stalling" must NOT appear in
    # the cmd_vel-during-NAV verdict for this run.
    nav_line_match = re.search(
        r"cmd_vel during NAV  :.*?(?:\n|$)", summary
    )
    if nav_line_match is None:
        print("[FAIL] summary missing 'cmd_vel during NAV' line")
        return 1
    nav_line = nav_line_match.group(0)
    # In this smoke the driver does NOT publish in_flight=0 NAVIGATING
    # rows (it flips status -> DONE on arrival), so the OK verdict
    # is fine; we only need to assert no false "stalling" label.
    if "stalling" in nav_line:
        print(f"[FAIL] unexpected 'stalling' verdict in: {nav_line.strip()}")
        return 1
    print("[OK] no false 'stalling' verdict on NAVIGATING coverage")

    print("\n[ALL_OK] record_mapping_debug smoke passed")
    # Cleanup tmp dir for tidiness.
    for f in tmp_dir.iterdir():
        f.unlink()
    tmp_dir.rmdir()
    return 0


if __name__ == "__main__":
    sys.exit(main())
