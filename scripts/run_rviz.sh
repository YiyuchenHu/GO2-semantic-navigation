#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Start RViz2. If a project RViz config exists, use: rviz2 -d <file>.
# Otherwise: plain rviz2 (no -d) so the script never assumes missing files.
# Add your own .rviz under one of the candidate paths when ready.
# -----------------------------------------------------------------------------
set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"

if ! command -v rviz2 >/dev/null 2>&1; then
	echo "[run_rviz] ERROR: 'rviz2' not found. Install ros-${ROS_DISTRO}-rviz2 (or desktop)" >&2
	exit 1
fi

# Preset selection:
#   * demo_recording (DEFAULT) — Fixed Frame=map, full social-aware
#       stack for command-first demo recording: global costmap,
#       /social_obstacles (red spheres), semantic markers, frontiers,
#       45° 3/4 perspective centred on the warehouse. This is the
#       config created for the command_first_demo.launch.py demo.
#   * semantic_nav  — Fixed Frame=world, original full semantic stack.
#   * motion_debug  — Fixed Frame=odom, lightweight displays for SLAM
#       heavy-optimisation debugging.
#
# Override preset:   RVIZ_PRESET=semantic_nav  ./scripts/run_rviz.sh
# Override full path: RVIZ_CONFIG=/abs/path/to/my.rviz  ./scripts/run_rviz.sh
_RVIZ_PRESET="${RVIZ_PRESET:-demo_recording}"
case "${_RVIZ_PRESET}" in
	demo_recording|demo|recording ) _RVIZ_FILE="demo_recording.rviz" ;;
	semantic_nav|semantic         ) _RVIZ_FILE="go2_semantic_nav.rviz" ;;
	motion_debug|motion|debug     ) _RVIZ_FILE="go2_motion_debug.rviz" ;;
	*.rviz)                         _RVIZ_FILE="${_RVIZ_PRESET}" ;;
	*)                              _RVIZ_FILE="${_RVIZ_PRESET}.rviz" ;;
esac

# Approach B — RVIZ_CONFIG env-var allows a full-path override so the
# operator can point at any .rviz without touching this script.
# When unset, resolve via `ros2 pkg prefix` (installed share path)
# first, then fall back to the source-tree candidates so the script
# works even before `colcon build`.
if [ -n "${RVIZ_CONFIG:-}" ]; then
	_CFG="${RVIZ_CONFIG}"
else
	# Installed share path (preferred after colcon build --symlink-install)
	_INSTALLED_PREFIX=""
	if command -v ros2 >/dev/null 2>&1; then
		_INSTALLED_PREFIX="$(ros2 pkg prefix go2_bringup_sim 2>/dev/null || true)"
	fi
	_RVIZ_CONFIG_CANDIDATES=(
		"${_INSTALLED_PREFIX:+${_INSTALLED_PREFIX}/share/go2_bringup_sim/config/rviz/${_RVIZ_FILE}}"
		"${_INSTALLED_PREFIX:+${_INSTALLED_PREFIX}/share/go2_bringup_sim/rviz/${_RVIZ_FILE}}"
		"${PROJECT_ROOT}/src/go2_bringup_sim/config/rviz/${_RVIZ_FILE}"
		"${PROJECT_ROOT}/src/go2_bringup_sim/rviz/${_RVIZ_FILE}"
		"${PROJECT_ROOT}/config/${_RVIZ_FILE}"
		"${PROJECT_ROOT}/rviz/${_RVIZ_FILE}"
	)
	_CFG=""
	for _f in "${_RVIZ_CONFIG_CANDIDATES[@]}"; do
		[ -n "${_f}" ] || continue
		if [ -f "${_f}" ]; then
			_CFG="${_f}"
			break
		fi
	done
fi

# Sim-time alignment. Whole stack (Isaac Sim, slam_toolbox, Nav2,
# perception nodes) runs with use_sim_time:=true and stamps every
# message with /clock. RViz defaults to wall time, so its tf2 cache
# keeps "now"-stamped TF while incoming /map etc carry the much
# smaller sim-time stamps -> "Message Filter dropping message: frame
# 'map' ... timestamp on the message is earlier than all the data in
# the transform cache". Setting use_sim_time on the rviz2 node fixes
# that without touching every Display in the .rviz config. Override
# with NO_SIM_TIME=1 if you ever want RViz on wall time (e.g. when
# debugging against a wall-time bag).
_RVIZ_ROS_ARGS=()
if [ "${NO_SIM_TIME:-0}" != "1" ]; then
	_RVIZ_ROS_ARGS=(--ros-args -p use_sim_time:=true)
fi

if [ -n "${_CFG}" ]; then
	echo "[run_rviz] Using config: ${_CFG}" >&2
	echo "[run_rviz] use_sim_time=${NO_SIM_TIME:+false}${NO_SIM_TIME:-true}" >&2
	exec rviz2 -d "${_CFG}" "$@" "${_RVIZ_ROS_ARGS[@]}"
fi

echo "[run_rviz] No RViz config found in project; starting plain rviz2" >&2
echo "[run_rviz] (Add config/go2_semantic_nav.rviz to enable -d auto-load.)" >&2
exec rviz2 "$@" "${_RVIZ_ROS_ARGS[@]}"
