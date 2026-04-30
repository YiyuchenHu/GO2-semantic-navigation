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

# Optional configs — only the first *existing* file is used
_RVIZ_CONFIG_CANDIDATES=(
	"${PROJECT_ROOT}/config/go2_semantic_nav.rviz"
	"${PROJECT_ROOT}/rviz/go2_semantic_nav.rviz"
	"${PROJECT_ROOT}/go2_bringup_sim/rviz/go2_semantic_nav.rviz"
	"${PROJECT_ROOT}/src/go2_bringup_sim/rviz/go2_semantic_nav.rviz"
)

_CFG=""
for _f in "${_RVIZ_CONFIG_CANDIDATES[@]}"; do
	if [ -f "${_f}" ]; then
		_CFG="${_f}"
		break
	fi
done

if [ -n "${_CFG}" ]; then
	echo "[run_rviz] Using config: ${_CFG}" >&2
	exec rviz2 -d "${_CFG}" "$@"
fi

echo "[run_rviz] No RViz config found in project; starting plain rviz2" >&2
echo "[run_rviz] (Add config/go2_semantic_nav.rviz to enable -d auto-load.)" >&2
exec rviz2 "$@"
