#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# launch_safe.sh — wrapper around `ros2 launch` that guarantees Ctrl+C
# kills every spawned process, including stubborn C++ children
# (static_transform_publisher, Nav2 component_container_isolated,
# slam_toolbox, lifecycle_manager) that ignore polite SIGTERM and would
# otherwise become orphans.
#
# How it works
#   1. setsid creates a new process *session*. The wrapper itself is the
#      session leader, so EVERY descendant inherits the same Session ID.
#   2. We capture the SID/PGID, then on EXIT/INT/TERM we send SIGKILL to
#      `-PGID` (negative pid = whole process group). The kernel then
#      walks down and SIGKILLs every descendant in one shot — even
#      processes that already detached and got reparented to PID 1, as
#      long as they didn't call setpgid() themselves (and ROS nodes
#      don't).
#   3. We forward SIGINT to the launch first (so it can run its
#      OnShutdown handlers cleanly), wait briefly, then nuke whatever's
#      left.
#
# Usage
#   bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py
#   bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
#   bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py
#   bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py \
#       abort_cooldown_sec:=10.0
#
# Tip: alias it once in ~/.bashrc:
#   alias rosrun-safe='bash <PROJECT_ROOT>/scripts/launch_safe.sh'
# -----------------------------------------------------------------------------
set -euo pipefail

if [ "$#" -lt 1 ]; then
	echo "usage: launch_safe.sh <package> <launchfile> [launch_args...]" >&2
	echo "       launch_safe.sh go2_bringup_sim day8_two_phase.launch.py" >&2
	exit 2
fi

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"

# We want to be the session leader for our descendants. If we are NOT
# already a session leader (i.e. our PID != our SID), re-exec under
# setsid so a fresh session is created. After that, our PID == our SID
# == PGID of every descendant, and `kill -- -$$` is a one-shot reap.
if [ "$(ps -o sid= -p $$ | tr -d ' ')" != "$$" ]; then
	exec setsid -w "$0" "$@"
fi

_LAUNCH_PID=0

_cleanup() {
	local rc=$?
	# Block recursive calls (signal storms).
	trap '' INT TERM EXIT
	echo "" >&2
	echo "[launch_safe] cleanup: sending SIGINT to launch group ${$}" >&2
	# Polite first: forward SIGINT to the whole group so launch can
	# print "All log files can be found below ..." and tear down nodes.
	kill -INT -- -$$ 2>/dev/null || true
	# Wait up to 4 seconds for the launch to drain.
	local _waited=0
	while [ "${_waited}" -lt 40 ]; do
		# If pgrep finds no member of our group besides ourself, we're done.
		local _alive
		_alive=$(pgrep -g $$ | grep -v "^$$\$" || true)
		if [ -z "${_alive}" ]; then
			break
		fi
		sleep 0.1
		_waited=$((_waited + 1))
	done
	# Anything left? SIGKILL the whole group.
	echo "[launch_safe] cleanup: SIGKILL on group ${$} (anything that didn't exit cleanly)" >&2
	kill -KILL -- -$$ 2>/dev/null || true
	exit "${rc}"
}

trap _cleanup INT TERM EXIT

# Use exec-style so the launch is a direct child (still in our group).
echo "[launch_safe] starting: ros2 launch $* (session_leader_pid=$$)" >&2
ros2 launch "$@" &
_LAUNCH_PID=$!

# Wait without losing signal handling; `wait` is interruptible by trap.
wait "${_LAUNCH_PID}"
