#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Central environment for the GO2 semantic navigation workspace.
# Intended to be *sourced* (not executed) by other scripts:
#   source "$(dirname "$0")/dev_env.sh"
# -----------------------------------------------------------------------------

# Resolve *this* file's directory. Use a private name so "unset" does not clobber
# a caller's _SCRIPT_DIR (some scripts set _SCRIPT_DIR, then "source" us under set -u).
if [ -n "${BASH_SOURCE[0]:-}" ]; then
	_GO2_DEV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
	_GO2_DEV_DIR="$(cd "$(dirname "$0")" && pwd)"
fi
export PROJECT_ROOT="${PROJECT_ROOT:-"$(cd "${_GO2_DEV_DIR}/.." && pwd)"}"

# ROS distro (Jazzy on Ubuntu 24.04 is typical; override before sourcing if needed).
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# Isaac Sim: folder that *contains* isaac-sim.sh (or kit/isaac-sim.sh) or python.sh.
# We prefer Isaac 5.x because its omni.isaac.ros2_bridge -> isaacsim.ros2.bridge
# supports ROS 2 Jazzy. Isaac 4.1's bridge does NOT support Jazzy.
# Override by exporting ISAAC_SIM_ROOT before sourcing this file.
if [ -z "${ISAAC_SIM_ROOT:-}" ]; then
	for _go2_isaac_cand in \
		"${HOME}/isaacsim_5.1_backup" \
		"${HOME}/isaacsim_5.1" \
		"${HOME}/isaacsim" \
		"${HOME}/isaac-sim"; do
		if [ -e "${_go2_isaac_cand}/python.sh" ] || [ -e "${_go2_isaac_cand}/isaac-sim.sh" ]; then
			export ISAAC_SIM_ROOT="${_go2_isaac_cand}"
			break
		fi
	done
	unset _go2_isaac_cand
	export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-"${HOME}/isaac-sim"}"
fi

# Optional ROS2 / RMW (uncomment and set as needed for your team / DDS)
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export CYCLONEDDS_URI="/path/to/cyclonedds.xml"
# export ROS_LOCALHOST_ONLY=0
# export RCUTILS_COLORIZED_OUTPUT=1

# --- System ROS + workspace overlay (required for: ros2, colcon, etc.) ---
# ament/ROS and colcon setup.bash reference env vars that may be unset. If a
# parent script used `set -u` (nounset), those sources will fail (e.g.
# AMENT_TRACE_SETUP_FILES). Turn nounset off for these two sources only.
_ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
_WS_SETUP="${PROJECT_ROOT}/install/setup.bash"
_GO2_SAVED_NOUNSET=0
case $- in *u*) _GO2_SAVED_NOUNSET=1; set +u ;; esac

if [ -f "${_ROS_SETUP}" ]; then
	# shellcheck source=/dev/null
	source "${_ROS_SETUP}"
else
	echo "[dev_env] warn: missing ${_ROS_SETUP} — is ROS ${ROS_DISTRO} installed?" >&2
fi
if [ -f "${_WS_SETUP}" ]; then
	# shellcheck source=/dev/null
	source "${_WS_SETUP}"
else
	echo "[dev_env] info: ${_WS_SETUP} not found; build with: cd ${PROJECT_ROOT} && colcon build --symlink-install" >&2
fi

case ${_GO2_SAVED_NOUNSET} in 1) set -u ;; esac
unset _GO2_SAVED_NOUNSET

unset _ROS_SETUP _WS_SETUP _GO2_DEV_DIR
