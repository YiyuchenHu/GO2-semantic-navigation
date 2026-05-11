#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# echo_cmd_vel_angular.sh — print the angular.z component of every cmd_vel
# stage so you can see whether the angular command is oscillating (sign-
# flipping every tick) at any point in the Nav2 pipeline.
#
# Why we care:
#   On 2026-05-08 we observed Go2 falling shortly after a Nav2 goal because
#   /cmd_vel.angular.z was alternating sign every 200 ms (e.g. +0.56 →
#   -0.56 → +0.49 → -0.47 → -0.47), which is OOD for a policy trained on
#   velocity commands resampled every 3-8 s.
#
#   With this script you can localize WHERE the oscillation enters:
#     * If /cmd_vel_nav already oscillates → controller_server (RPP) is
#       overcorrecting; increase lookahead_dist, or raise
#       controller_frequency so RPP gets fresher pose between ticks.
#     * If /cmd_vel_nav is smooth but /cmd_vel_smoothed oscillates →
#       smoother accel limits are too aggressive, or smoothing_frequency
#       is too low.
#     * If both upstream stages are smooth but /cmd_vel oscillates →
#       collision_monitor is intermittently zeroing out the command.
#
# Implementation note:
#   The three ros2 topic echo subscribers run IN PARALLEL during a single
#   window (previous serial version missed /cmd_vel_smoothed and /cmd_vel
#   because typical short-distance goals finish in under one window). The
#   final report shows each topic's samples grouped together with timestamp
#   prefixes so oscillation periods can be eyeballed across stages.
#
# Usage:
#   bash scripts/echo_cmd_vel_angular.sh                # 15 s shared window
#   bash scripts/echo_cmd_vel_angular.sh 30             # custom window
#
# IMPORTANT (same as diagnose_cmd_vel_rates.sh):
#   Topics in the cmd_vel chain ONLY publish while a Nav2 goal is active.
#   Drop a 2D Nav Goal in RViz BEFORE running this script.
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[echo_cmd_vel_angular] ERROR: 'ros2' not found." >&2
	echo "  source scripts/dev_env.sh first, or 'source /opt/ros/jazzy/setup.bash'." >&2
	exit 2
fi

_WINDOW="${1:-15}"

_WORK="$(mktemp -d -t echo_cmd_vel.XXXXXX)"
cleanup() {
	rm -rf "${_WORK}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cat <<EOF
[echo_cmd_vel_angular]
  Streaming angular.z from /cmd_vel_nav, /cmd_vel_smoothed, /cmd_vel
  IN PARALLEL for ${_WINDOW}s. Drop a 2D Nav Goal in RViz BEFORE
  running so the topics are actually being published.

  Look for:
    * Sign flips between consecutive samples → oscillation (bad).
    * Step sizes >> max_accel/smoothing_frequency → smoother bypassed.
    * Constant max-magnitude (±1.0) → controller saturated, RPP wants
      faster turn than max_velocity allows.

EOF

# `ros2 topic echo --field angular.z` emits one numeric line per message
# AND a `---` YAML doc separator between messages. We strip the `---`
# lines in awk so the output is just numbers with timestamps. stdbuf -oL
# forces line-buffered stdout in ros2 topic echo so timestamps reflect
# the actual message arrival time (with default block-buffered stdout
# everything dumps at process exit and timestamps cluster at the end).
spawn_echo() {
	local topic="$1"
	local logfile="$2"
	local msg_type
	msg_type=$(ros2 topic info "${topic}" 2>/dev/null \
		| awk -F': ' '/^Type:/ { print $2 }')

	# Choose the field expression for this msg type.
	local field=""
	case "${msg_type}" in
		geometry_msgs/msg/Twist)        field="angular.z" ;;
		geometry_msgs/msg/TwistStamped) field="twist.angular.z" ;;
		*)                              field="" ;;
	esac

	{
		echo "# topic=${topic} type=${msg_type:-unknown} window=${_WINDOW}s"
		if [ -z "${msg_type}" ]; then
			echo "# WARN: topic not advertised — is Nav2 active and a goal set?"
			# Still need to occupy the window so other topics can capture.
			sleep "${_WINDOW}"
			return 0
		fi
		if [ -n "${field}" ]; then
			stdbuf -oL ros2 topic echo --field "${field}" "${topic}" 2>&1 \
				| awk '
					/^---/ { next }
					{ printf "%s  %s\n", strftime("%H:%M:%S"), $0; fflush(); }
				'
		else
			stdbuf -oL ros2 topic echo "${topic}" 2>&1
		fi
	} >"${logfile}" 2>&1 &
	echo $!
}

PID_NAV=$(spawn_echo "/cmd_vel_nav"      "${_WORK}/cmd_vel_nav.log")
PID_SMO=$(spawn_echo "/cmd_vel_smoothed" "${_WORK}/cmd_vel_smoothed.log")
PID_OUT=$(spawn_echo "/cmd_vel"          "${_WORK}/cmd_vel.log")

echo "[echo_cmd_vel_angular] sampling for ${_WINDOW}s ..."
sleep "${_WINDOW}"

kill -INT  "${PID_NAV}" "${PID_SMO}" "${PID_OUT}" 2>/dev/null || true
sleep 0.3
kill -TERM "${PID_NAV}" "${PID_SMO}" "${PID_OUT}" 2>/dev/null || true
wait "${PID_NAV}" "${PID_SMO}" "${PID_OUT}" 2>/dev/null || true

dump_one() {
	local topic="$1"
	local logfile="$2"
	echo
	echo "================================================================"
	echo "  ${topic}    (parallel window: ${_WINDOW}s)"
	echo "================================================================"
	if [ ! -s "${logfile}" ]; then
		echo "  (no output — topic silent during the window;"
		echo "   was a Nav2 goal active for ${_WINDOW}s?)"
		return 0
	fi
	cat "${logfile}"
}

dump_one "/cmd_vel_nav"      "${_WORK}/cmd_vel_nav.log"
dump_one "/cmd_vel_smoothed" "${_WORK}/cmd_vel_smoothed.log"
dump_one "/cmd_vel"          "${_WORK}/cmd_vel.log"

# ---- Quick statistics across the three topics ---------------------------
# Print the count of samples and (if there are >=2 numeric samples) the
# count of sign-flips between consecutive samples — a sign-flip is the
# clearest binary indicator of "is this stage oscillating?". We only
# count flips where both samples have magnitude >= 0.05 to suppress
# noise around zero (where the policy doesn't care).
echo
echo "================================================================"
echo "  per-topic summary (n samples / sign-flips with |val|>=0.05)"
echo "================================================================"
summarize() {
	local topic="$1"
	local logfile="$2"
	# Pull just the numeric column out of the timestamped lines. Lines
	# we want look like "HH:MM:SS  0.5555". Skip header (#) lines.
	awk '
		/^#/ { next }
		NF >= 2 {
			val = $2 + 0   # cast to number; non-numeric → 0 which is fine
			printf "%s\n", val
		}
	' "${logfile}" 2>/dev/null \
	| awk -v topic="${topic}" '
		BEGIN { n = 0; flips = 0; prev = 0; have_prev = 0 }
		{
			n++
			cur = $1 + 0
			if (have_prev && (prev*cur < 0) && (prev*prev >= 0.0025) && (cur*cur >= 0.0025)) {
				flips++
			}
			prev = cur
			have_prev = 1
		}
		END {
			printf "  %-22s n=%d  sign-flips=%d\n", topic, n, flips
		}
	'
}
summarize "/cmd_vel_nav"      "${_WORK}/cmd_vel_nav.log"
summarize "/cmd_vel_smoothed" "${_WORK}/cmd_vel_smoothed.log"
summarize "/cmd_vel"          "${_WORK}/cmd_vel.log"

cat <<EOF

[echo_cmd_vel_angular] done.

Quick interpretation guide:
  * If /cmd_vel_nav itself has many sign-flips: the controller is the
    source. Open nav2_params.yaml > FollowPath. Raise lookahead_dist
    (currently 0.6 m → try 0.9-1.2 m), or raise controller_frequency
    so RPP gets fresher pose.
  * If /cmd_vel_nav has flips but /cmd_vel_smoothed has FEWER flips:
    smoother is doing its job — that's expected and good. Higher
    smoothing_frequency or tighter max_accel will further reduce the
    flip count downstream.
  * If /cmd_vel_smoothed sign-flips ≈ /cmd_vel_nav sign-flips:
    smoother is being bypassed somehow (check it's actually subscribed
    to /cmd_vel_nav, lifecycle is active, max_accel is finite).
  * If everything upstream is smooth but /cmd_vel has gaps / zeros:
    collision_monitor is gating. Verify in nav2_params.yaml that both
    FootprintApproach.enabled and scan.enabled are FALSE.
EOF
