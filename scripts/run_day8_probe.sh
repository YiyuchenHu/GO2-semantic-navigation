#!/usr/bin/env bash
# scripts/run_day8_probe.sh — one-shot wrapper for DEBUG MODE probe.
#
# Sources the workspace, runs scripts/_debug_day8_probe.py for 15 s,
# and prints a brief summary of where the NDJSON log was written.
#
# Usage:
#   bash scripts/run_day8_probe.sh [run_id]
# default run_id = "post-fix"
#
# Note: do NOT enable `set -u` — ROS setup.bash trips on unbound vars
# (AMENT_TRACE_SETUP_FILES etc.) and would silently abort the script
# with no diagnostic output. We rely on per-step explicit checks below.

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_PATH="${REPO_DIR}/.cursor/debug-7f68f3.log"
RUN_ID="${1:-post-fix}"

echo "[probe] repo_dir=${REPO_DIR}"
echo "[probe] log_path=${LOG_PATH}"
echo "[probe] run_id=${RUN_ID}"

cd "${REPO_DIR}" || {
	echo "[probe] FATAL: cd to ${REPO_DIR} failed" >&2
	exit 2
}

# Sanity checks BEFORE we attempt to import rclpy / go2_msgs.
if [ ! -f "install/setup.bash" ]; then
	echo "[probe] FATAL: install/setup.bash missing — workspace not built." >&2
	exit 2
fi

# Source ROS distro (try jazzy first, fall back to humble).
_ROS_SOURCED=0
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
	echo "[probe] sourcing /opt/ros/jazzy/setup.bash"
	# shellcheck disable=SC1091
	source /opt/ros/jazzy/setup.bash
	_ROS_SOURCED=1
elif [ -f "/opt/ros/humble/setup.bash" ]; then
	echo "[probe] sourcing /opt/ros/humble/setup.bash"
	# shellcheck disable=SC1091
	source /opt/ros/humble/setup.bash
	_ROS_SOURCED=1
fi
if [ "${_ROS_SOURCED}" -ne 1 ]; then
	echo "[probe] FATAL: no /opt/ros/<distro>/setup.bash found." >&2
	exit 2
fi

echo "[probe] sourcing install/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
_src_rc=$?
if [ "${_src_rc}" -ne 0 ]; then
	echo "[probe] FATAL: source install/setup.bash exited ${_src_rc}" >&2
	exit 2
fi

mkdir -p "$(dirname "${LOG_PATH}")"

echo "[probe] launching python probe (15 s window)..."
python3 scripts/_debug_day8_probe.py "${RUN_ID}" 15
_rc=$?
echo "[probe] python probe exit=${_rc}"

if [ -f "${LOG_PATH}" ]; then
	_lines=$(wc -l < "${LOG_PATH}")
	echo "[probe] wrote ${_lines} log lines to ${LOG_PATH}"
	echo "[probe] === last 8 lines ==="
	tail -n 8 "${LOG_PATH}"
	echo "[probe] === end ==="
else
	echo "[probe] WARN: log file ${LOG_PATH} does NOT exist — probe failed to write" >&2
	exit 3
fi

exit 0
