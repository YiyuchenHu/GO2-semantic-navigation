#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Convenience wrapper: launch the warehouse with full ROS 2 bridging.
#
# Picks up ISAAC_SIM_ROOT from scripts/dev_env.sh (default: ~/isaacsim_5.1_backup).
# Default is GUI mode (--no-headless) so you actually see the warehouse; pass
# --headless to skip rendering. The runtime script now builds the scene on a
# FRESH stage (sim/warehouse_scene.py) — it no longer opens the saved USD,
# which on Isaac 5.1 + RTX 5090 was triggering a reopenUsd / omni.graph.image
# crash. Use --diag boot-only / after-build / after-sensors / after-graph to
# bisect startup if something regresses.
# All extra args are forwarded to sim/run_go2_warehouse_ros2.py, e.g.:
#     bash scripts/run_warehouse_ros2.sh
#     bash scripts/run_warehouse_ros2.sh --rgb-resolution 640x480
#     bash scripts/run_warehouse_ros2.sh --diag after-build
# -----------------------------------------------------------------------------
set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh"

if [ -z "${ISAAC_SIM_ROOT:-}" ] || [ ! -e "${ISAAC_SIM_ROOT}/python.sh" ]; then
	echo "[run_warehouse_ros2] ERROR: no Isaac Sim python.sh under ISAAC_SIM_ROOT=${ISAAC_SIM_ROOT:-<unset>}" >&2
	echo "  Try: bash scripts/run_isaacsim.sh --list" >&2
	exit 1
fi

# Isaac Sim's python.sh dislikes being wrapped by an active conda env.
if [ -n "${CONDA_PREFIX:-}" ] && command -v conda >/dev/null 2>&1; then
	echo "[run_warehouse_ros2] note: deactivating conda env (${CONDA_DEFAULT_ENV:-?}) for Isaac python.sh" >&2
	# shellcheck disable=SC1091
	source "$(conda info --base)/etc/profile.d/conda.sh"
	conda deactivate || true
fi

# ----- ROS 2 bridge env -----------------------------------------------------
# isaacsim.ros2.bridge requires its bundled Jazzy C++ libs to be on
# LD_LIBRARY_PATH BEFORE Kit boots, otherwise the bridge spits this warning at
# startup ("Could not import internal rclpy ...") and ends up with no working
# DDS backend, which both kills topic publishing AND can trigger a crash later
# in libomni.graph.image.core when render products start firing.
_GO2_BRIDGE_LIB="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/jazzy/lib"
if [ -d "${_GO2_BRIDGE_LIB}" ]; then
	export ROS_DISTRO=jazzy
	export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
	if [ -z "${LD_LIBRARY_PATH:-}" ]; then
		export LD_LIBRARY_PATH="${_GO2_BRIDGE_LIB}"
	else
		case ":${LD_LIBRARY_PATH}:" in
			*":${_GO2_BRIDGE_LIB}:"*) : ;;  # already there
			*) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${_GO2_BRIDGE_LIB}" ;;
		esac
	fi
	echo "[run_warehouse_ros2] ROS_DISTRO=${ROS_DISTRO} RMW=${RMW_IMPLEMENTATION}" >&2
	echo "[run_warehouse_ros2] +LD_LIBRARY_PATH ${_GO2_BRIDGE_LIB}" >&2
else
	echo "[run_warehouse_ros2] WARN: bridge lib dir not found at ${_GO2_BRIDGE_LIB}" >&2
fi

cd "${PROJECT_ROOT}"

# Default to GUI mode so the user can visually confirm the warehouse built
# correctly. Any explicit --no-headless / --headless in the user's args
# overrides this default.
_add_gui=1
for _a in "$@"; do
	case "$_a" in
		--no-headless|--headless) _add_gui=0 ;;
	esac
done
_extra_args=()
if [ "${_add_gui}" = 1 ]; then
	_extra_args+=("--no-headless")
fi

echo "[run_warehouse_ros2] ISAAC_SIM_ROOT=${ISAAC_SIM_ROOT}" >&2
echo "[run_warehouse_ros2] Args: ${_extra_args[*]} $*" >&2
exec "${ISAAC_SIM_ROOT}/python.sh" sim/run_go2_warehouse_ros2.py "${_extra_args[@]}" "$@"
