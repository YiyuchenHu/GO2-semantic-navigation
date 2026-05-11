#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# kill_all.sh — bring the GO2 ROS 2 stack to a clean slate.
#
# Why this exists
#   Just doing `pkill -f "ros2 launch"` only kills the launch *parent*
#   process. ROS 2 launch's children (especially the C++ ones —
#   static_transform_publisher, nav2_container, slam_toolbox,
#   lifecycle_manager, map_saver) often survive and get re-parented to
#   PID 1 as orphans. The next `ros2 launch tf_and_scan ...` then
#   coexists with the previous orphans, you end up with 2-3 instances
#   of every TF, two SLAM publishers fighting over /tf, and Nav2
#   controller_server failing every goal with "Unable to transform
#   robot pose into global plan's frame".
#
#   This script enumerates every ROS executable our launches spawn and
#   force-kills them, then verifies nothing is left.
#
# What this script does NOT kill
#   * Isaac Sim (run_warehouse_ros2.sh / run_isaacsim.sh) — Sim is
#     expensive to restart, keep it running across resets. Pass
#     --include-sim to override.
#   * rviz2 — purely visualisation, harmless to keep. Pass --include-rviz
#     if you want it gone too.
#
# Typical usage
#   bash scripts/kill_all.sh             # kill ROS stack only (Sim & RViz survive)
#   bash scripts/kill_all.sh --all       # kill ROS stack + RViz + Sim
#   bash scripts/kill_all.sh --include-rviz
#   bash scripts/kill_all.sh --include-sim
#   bash scripts/kill_all.sh --dry-run   # show what would be killed
# -----------------------------------------------------------------------------
set -euo pipefail

DRY_RUN=0
INCLUDE_RVIZ=0
INCLUDE_SIM=0
for arg in "$@"; do
	case "${arg}" in
		--dry-run|-n)        DRY_RUN=1 ;;
		--include-rviz)      INCLUDE_RVIZ=1 ;;
		--include-sim)       INCLUDE_SIM=1 ;;
		--all)               INCLUDE_RVIZ=1; INCLUDE_SIM=1 ;;
		-h|--help)
			sed -n '2,/^# -----/p' "$0" | sed 's/^# \{0,1\}//'
			exit 0 ;;
		*)
			echo "[kill_all] unknown arg: ${arg} (try --help)" >&2
			exit 2 ;;
	esac
done

# Patterns to match against `ps -ef` output. We keep these tight enough
# that a casual `python3 my_unrelated_thing.py` will NOT be hit.
_PATTERNS=(
	# launch parents
	"ros2 launch"
	# tf_and_scan — there are 6 of these per launch
	"tf2_ros/static_transform_publisher"
	# Nav2 + SLAM
	"rclcpp_components/component_container_isolated"
	"slam_toolbox/sync_slam_toolbox"
	"slam_toolbox/async_slam_toolbox"
	"nav2_lifecycle_manager/lifecycle_manager"
	"nav2_map_server/map_saver_server"
	"nav2_map_server/map_server"
	"nav2_amcl/amcl"
	# Day-8 pure-Python nodes
	"go2_navigation/lib/go2_navigation"
	"go2_perception/lib/go2_perception"
	"go2_semantic_perception/lib/go2_semantic_perception"
	"go2_task_coordinator/lib/go2_task_coordinator"
	"go2_nl_parser/lib/go2_nl_parser"
)

if [ "${INCLUDE_RVIZ}" = "1" ]; then
	_PATTERNS+=("rviz2")
fi

if [ "${INCLUDE_SIM}" = "1" ]; then
	# Be careful with these — only matched against full `ps -ef` argv,
	# not just executable name. Tighten if your sim entry differs.
	_PATTERNS+=("run_warehouse_ros2.sh")
	_PATTERNS+=("isaac-sim.sh")
	_PATTERNS+=("kit/python/python3 .* isaac")
fi

# IDE / shell-wrapper executables whose argv routinely contains the
# literal string "ros2 launch ..." even when no ROS process is
# actually running. Examples seen in the wild:
#   * Cursor: /usr/share/cursor/resources/app/resources/helpers/cursorsandbox
#     wraps every Shell tool call in `bash -c '... ros2 launch ...'`
#     so the wrapper's argv includes the verb forever.
#   * VS Code: vscode-server/code, code-tunnel, code-helper.
#   * tmux / screen / sshd / login shells that scrolled the same line.
# pkill -f with our patterns would gleefully kill these — and
# possibly the IDE itself. So we filter ANY matched PID through this
# allow-list of *real* ROS executables before signalling.
_IDE_WRAPPER_BLOCKLIST=(
	"cursorsandbox"
	"cursor/resources/app"
	"vscode-server"
	"code-tunnel"
	"code-helper"
	"electron"
	"/sshd"
	"/login"
	"tmux: client"
	"tmux: server"
)

# Helpers that read /proc, available on every Linux. We deliberately
# don't trust `ps -ef` output to filter (commands can be mangled);
# /proc/<pid>/{exe,cmdline,comm} is authoritative.
_pid_alive() {
	local pid="$1"
	[ -e "/proc/${pid}/cmdline" ]
}

_pid_argv() {
	# argv joined with spaces — same content `ps -ef` and `pgrep -f`
	# match against, but read directly from /proc.
	local pid="$1"
	tr '\0' ' ' </proc/"${pid}"/cmdline 2>/dev/null || true
}

_pid_exe() {
	local pid="$1"
	readlink -f /proc/"${pid}"/exe 2>/dev/null || true
}

_pid_is_ide_wrapper() {
	local pid="$1"
	local exe argv block
	exe="$(_pid_exe "${pid}")"
	argv="$(_pid_argv "${pid}")"
	for block in "${_IDE_WRAPPER_BLOCKLIST[@]}"; do
		# Use literal substring matching — the block patterns are
		# plain strings, NOT regexes.
		if [[ "${exe}" == *"${block}"* ]]; then return 0; fi
		if [[ "${argv}" == *"${block}"* ]]; then return 0; fi
	done
	return 1
}

# Skip our own kill_all process tree (and pgrep itself) so we don't
# self-suicide if, for example, the script is invoked via a path
# containing "ros2".
_pid_is_self() {
	local pid="$1"
	if [ "${pid}" = "$$" ] || [ "${pid}" = "${BASHPID:-}" ]; then
		return 0
	fi
	# Walk up the parent chain to spot ourselves as an ancestor.
	local cur="${pid}"
	for _ in 1 2 3 4; do
		local ppid
		ppid="$(awk '{print $4}' "/proc/${cur}/stat" 2>/dev/null \
		         || true)"
		if [ -z "${ppid}" ] || [ "${ppid}" = "0" ]; then
			break
		fi
		if [ "${ppid}" = "$$" ]; then return 0; fi
		cur="${ppid}"
	done
	return 1
}

_filter_real_ros_pids() {
	# Read PIDs on stdin one per line, write back the survivors.
	local p
	while IFS= read -r p; do
		[ -z "${p}" ] && continue
		_pid_alive "${p}" || continue
		_pid_is_self "${p}" && continue
		_pid_is_ide_wrapper "${p}" && continue
		printf '%s\n' "${p}"
	done
}

_anything_killed=0

_kill_pattern() {
	local pat="$1"
	local raw_pids pids
	# -f: match against full command line. Capture first so we can
	# filter out IDE wrapper false-positives BEFORE signalling
	# (pkill -f is unsafe here — it would happily kill a Cursor
	# helper that just happens to have "ros2 launch" in argv).
	mapfile -t raw_pids < <(pgrep -f "${pat}" 2>/dev/null || true)
	mapfile -t pids < <(printf '%s\n' "${raw_pids[@]}" \
	                    | _filter_real_ros_pids)
	if [ "${#pids[@]}" -eq 0 ]; then
		return 0
	fi
	echo "[kill_all] match ${pat@Q} -> PIDs: ${pids[*]}"
	if [ "${DRY_RUN}" = "1" ]; then
		return 0
	fi
	# Try TERM first so processes can clean up; then fall back to
	# KILL. Use `kill` directly on the filtered list rather than
	# pkill -f so we never re-broaden the match.
	kill -TERM "${pids[@]}" 2>/dev/null || true
	_anything_killed=1
}

_kill_pattern_kill() {
	local pat="$1"
	local raw_pids pids
	if [ "${DRY_RUN}" = "1" ]; then
		return 0
	fi
	mapfile -t raw_pids < <(pgrep -f "${pat}" 2>/dev/null || true)
	mapfile -t pids < <(printf '%s\n' "${raw_pids[@]}" \
	                    | _filter_real_ros_pids)
	if [ "${#pids[@]}" -eq 0 ]; then
		return 0
	fi
	kill -KILL "${pids[@]}" 2>/dev/null || true
}

echo "[kill_all] Sending SIGTERM to ROS stack (dry-run=${DRY_RUN}, include_rviz=${INCLUDE_RVIZ}, include_sim=${INCLUDE_SIM})"
for pat in "${_PATTERNS[@]}"; do
	_kill_pattern "${pat}"
done

if [ "${DRY_RUN}" = "1" ]; then
	echo "[kill_all] dry-run done — no signals sent."
	exit 0
fi

# Give them a beat to handle SIGTERM cleanly.
sleep 1.5

# Anything still standing? SIGKILL it.
echo "[kill_all] Sending SIGKILL to anything that survived SIGTERM"
for pat in "${_PATTERNS[@]}"; do
	_kill_pattern_kill "${pat}"
done
sleep 0.5

# Verify — same filter logic as the kill phase. Walk every pattern,
# collect PIDs, drop self / IDE wrappers, see what's left.
_remaining_pids=()
for pat in "${_PATTERNS[@]}"; do
	mapfile -t _raw < <(pgrep -f "${pat}" 2>/dev/null || true)
	mapfile -t _filtered < <(printf '%s\n' "${_raw[@]}" \
	                         | _filter_real_ros_pids)
	for p in "${_filtered[@]:-}"; do
		[ -n "${p}" ] && _remaining_pids+=("${p}")
	done
done

# De-dup.
if [ "${#_remaining_pids[@]}" -gt 0 ]; then
	mapfile -t _remaining_pids < <(
		printf '%s\n' "${_remaining_pids[@]}" | sort -u
	)
fi

if [ "${#_remaining_pids[@]}" -gt 0 ]; then
	echo "[kill_all] WARNING: some processes are still alive:" >&2
	for p in "${_remaining_pids[@]}"; do
		_pid_alive "${p}" || continue
		# Print one tidy line per real survivor.
		_argv="$(_pid_argv "${p}")"
		printf '  pid=%s argv=%s\n' "${p}" \
			"${_argv:0:200}" >&2
	done
	echo "[kill_all] (IDE / cursor / vscode wrapper PIDs were filtered" \
		"out; if you want to see them too, run with KILL_ALL_DEBUG=1)" >&2
	if [ "${KILL_ALL_DEBUG:-0}" = "1" ]; then
		echo "[kill_all] DEBUG: full ps grep (unfiltered):" >&2
		ps -ef | grep -E "$(IFS='|'; echo "${_PATTERNS[*]}")" \
			| grep -v grep >&2 || true
	fi
	exit 1
fi

echo "[kill_all] Clean. (Isaac Sim left running: ${INCLUDE_SIM}=0 means yes.)"
