#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# diagnose_cmd_vel_rates.sh — measure the publish rate at every stage of
# Nav2's cmd_vel pipeline so you can see WHERE the rate is being clamped.
#
# Background:
#   Nav2's cmd_vel chain (Jazzy default) is:
#     controller_server / behavior_server  ──► /cmd_vel_nav        (target rate
#                                                                    = controller_frequency)
#     velocity_smoother (subscribes /cmd_vel_nav)
#                                          ──► /cmd_vel_smoothed   (target rate
#                                                                    = smoothing_frequency)
#     collision_monitor (pass-through, subs /cmd_vel_smoothed)
#                                          ──► /cmd_vel            (≈ smoothing_frequency)
#     Isaac Sim SubTwist subscribes /cmd_vel.
#
#   The slowest stage caps the rate at every downstream stage. On 2026-05-08
#   we observed /cmd_vel pinned at exactly 5.000 Hz (0.200 s steps) — that's
#   smoothing_frequency=5 limiting the chain. Coarse temporal grid → policy
#   sees step impulses → angular cmd oscillates → Go2 falls.
#
# Goal of this script:
#   Confirm /cmd_vel_nav, /cmd_vel_smoothed, /cmd_vel rates while a Nav2
#   goal is active. Expected after the fix (controller=10 Hz, smoother=20 Hz):
#     /cmd_vel_nav        ≈ 10 Hz (or however fast controller_server runs)
#     /cmd_vel_smoothed   ≈ 20 Hz
#     /cmd_vel            ≈ 20 Hz   ← the policy receives this one
#
# Implementation note:
#   We run the three `ros2 topic hz` instances IN PARALLEL during a single
#   window. The original sequential version measured /cmd_vel_nav for 20 s,
#   then /cmd_vel_smoothed for 20 s, then /cmd_vel for 20 s — but a typical
#   short-distance goal completes in 5-15 s, so by the time the script
#   reached /cmd_vel_smoothed the goal had already finished and the topic
#   was silent. Parallel windows capture all three under the same active
#   goal.
#
# IMPORTANT:
#   These topics ONLY publish while Nav2 is actively executing a path.
#   Run this script AFTER:
#     1. Sim    — bash scripts/run_warehouse_ros2.sh --policy
#     2. tf+scan — bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py
#     3. Nav2   — bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py …
#     4. RViz   — drop a 2D Nav Goal so a path becomes active.
#   If you launch this without an active goal, all three measurements
#   will print "no new messages" and you'll think the stack is broken.
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[diagnose_cmd_vel_rates] ERROR: 'ros2' not found." >&2
	echo "  source scripts/dev_env.sh first, or 'source /opt/ros/jazzy/setup.bash'." >&2
	exit 2
fi

# Window for the parallel measurement. 20 s is plenty for ros2 topic hz to
# settle (it averages over messages received) and short enough that the
# operator isn't stuck if a goal completes mid-measurement. Override with $1.
_WINDOW="${1:-20}"

# Working dir for per-topic captures. Cleared on exit so successive runs
# don't accumulate stale files.
_WORK="$(mktemp -d -t diag_cmd_vel.XXXXXX)"
cleanup() {
	rm -rf "${_WORK}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cat <<EOF
[diagnose_cmd_vel_rates]
  Sampling /cmd_vel_nav, /cmd_vel_smoothed, /cmd_vel IN PARALLEL for
  ${_WINDOW}s. Make sure a Nav2 goal is ACTIVE (drop a 2D Nav Goal in
  RViz) BEFORE this script starts measuring, otherwise the topics will
  be silent and the rate will read 'no new messages received'.

  Expected after the controller=10 Hz / smoother=20 Hz fix:
    /cmd_vel_nav        ≈ 10 Hz   (controller's commanded rate)
    /cmd_vel_smoothed   ≈ 20 Hz   (smoother's interpolated output)
    /cmd_vel            ≈ 20 Hz   (collision_monitor pass-through)

  If /cmd_vel < /cmd_vel_smoothed: collision_monitor is gating
  (check its 'enabled: false' flags in nav2_params.yaml).
  If /cmd_vel_smoothed < smoothing_frequency: smoother is bottlenecked
  (CPU contention; check 'controller_server failed to advance').

EOF

# Spawn one ros2 topic hz per topic in the background, redirect to a per-
# topic log, capture pid so we can kill them in lockstep at the end. Note:
# Jazzy's `ros2 topic hz` does NOT support a --window or --use-wall-time
# flag (the latter is what tripped the previous version of this script).
# Default behaviour averages over all received samples, which is what we
# want for steady-state rate.
spawn_hz() {
	local topic="$1"
	local logfile="$2"
	# stdbuf -oL forces line-buffered stdout so the log fills in real time
	# rather than being flushed only at process exit.
	stdbuf -oL -eL ros2 topic hz "${topic}" >"${logfile}" 2>&1 &
	echo $!
}

PID_NAV=$(spawn_hz "/cmd_vel_nav"      "${_WORK}/cmd_vel_nav.log")
PID_SMO=$(spawn_hz "/cmd_vel_smoothed" "${_WORK}/cmd_vel_smoothed.log")
PID_OUT=$(spawn_hz "/cmd_vel"          "${_WORK}/cmd_vel.log")

echo "[diagnose_cmd_vel_rates] sampling for ${_WINDOW}s ..."
sleep "${_WINDOW}"

# SIGINT first (graceful), then SIGTERM if anything's still alive.
kill -INT  "${PID_NAV}" "${PID_SMO}" "${PID_OUT}" 2>/dev/null || true
sleep 0.3
kill -TERM "${PID_NAV}" "${PID_SMO}" "${PID_OUT}" 2>/dev/null || true
wait "${PID_NAV}" "${PID_SMO}" "${PID_OUT}" 2>/dev/null || true

dump_one() {
	local topic="$1"
	local logfile="$2"
	echo "================================================================"
	echo "  ros2 topic hz ${topic}   (parallel window: ${_WINDOW}s)"
	echo "================================================================"
	if [ ! -s "${logfile}" ]; then
		echo "  (no output — topic silent during the window;"
		echo "   was a Nav2 goal active for ${_WINDOW}s?)"
	else
		# `ros2 topic hz` prints "average rate: X" lines repeatedly. The
		# last few are the most representative. tail -5 keeps the dump
		# compact while preserving the final converged number.
		tail -8 "${logfile}"
	fi
	echo
}

dump_one "/cmd_vel_nav"      "${_WORK}/cmd_vel_nav.log"
dump_one "/cmd_vel_smoothed" "${_WORK}/cmd_vel_smoothed.log"
dump_one "/cmd_vel"          "${_WORK}/cmd_vel.log"

cat <<EOF
[diagnose_cmd_vel_rates] done.

If a topic above said "(no output …)":
  * Did Nav2 finish the goal? Drop a fresh 2D Nav Goal and re-run.
  * Or check lifecycle state:
      ros2 lifecycle get /controller_server
      ros2 lifecycle get /velocity_smoother
      ros2 lifecycle get /collision_monitor

If /cmd_vel publishes far below smoothing_frequency, the bottleneck is
downstream of velocity_smoother. The most common cause is collision_monitor
silently aborting (check
  ros2 lifecycle get /collision_monitor
). Other causes: namespace mismatch making collision_monitor subscribe
to a different /cmd_vel_smoothed, or Nav2's lifecycle_manager bond
dropping it.
EOF
