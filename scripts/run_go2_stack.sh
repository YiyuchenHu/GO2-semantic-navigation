#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Launch the full Go2 semantic navigation stack (sim bringup + app nodes).
# Requires: colcon build + source install, ROS ${ROS_DISTRO} on path.
# -----------------------------------------------------------------------------
set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[run_go2_stack] ERROR: 'ros2' not found. Source scripts/dev_env.sh in this shell" >&2
	echo "  or run this script from a login shell: bash ${0}" >&2
	exit 1
fi

# Package lives under the workspace; launch file name is fixed in go2_bringup_sim.
exec ros2 launch go2_bringup_sim sim_semantic_nav.launch.py "$@"
