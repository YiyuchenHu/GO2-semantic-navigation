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

# Day 8+ — two presets:
#   * go2_semantic_nav.rviz : Fixed Frame=world, full semantic stack
#                              (map, semantic markers, frontier debug,
#                              costmaps, plans). Use for the demo.
#   * go2_motion_debug.rviz  : Fixed Frame=odom, light displays
#                              (RobotModel/TF/LaserScan/PointCloud2/
#                              optional Image). Use when RobotModel
#                              feels delayed in the world frame —
#                              this avoids the 0.55 s map→odom
#                              post-date that makes Fixed Frame=world
#                              ghost during heavy SLAM optimisation.
#
# Override which preset by setting RVIZ_PRESET=motion_debug (or the
# bare filename) before invoking the script.
_RVIZ_PRESET="${RVIZ_PRESET:-semantic_nav}"
case "${_RVIZ_PRESET}" in
	semantic_nav|semantic|"" ) _RVIZ_FILE="go2_semantic_nav.rviz" ;;
	motion_debug|motion|debug ) _RVIZ_FILE="go2_motion_debug.rviz" ;;
	*.rviz)                     _RVIZ_FILE="${_RVIZ_PRESET}" ;;
	*)                          _RVIZ_FILE="${_RVIZ_PRESET}.rviz" ;;
esac
# Optional configs — only the first *existing* file is used
_RVIZ_CONFIG_CANDIDATES=(
	"${PROJECT_ROOT}/config/${_RVIZ_FILE}"
	"${PROJECT_ROOT}/rviz/${_RVIZ_FILE}"
	"${PROJECT_ROOT}/go2_bringup_sim/rviz/${_RVIZ_FILE}"
	"${PROJECT_ROOT}/src/go2_bringup_sim/rviz/${_RVIZ_FILE}"
)

_CFG=""
for _f in "${_RVIZ_CONFIG_CANDIDATES[@]}"; do
	if [ -f "${_f}" ]; then
		_CFG="${_f}"
		break
	fi
done

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
