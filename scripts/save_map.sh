#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Save the current /map (slam_toolbox OccupancyGrid) to a pgm + yaml pair
# that Day 4 Nav2 will load via map_server.
#
# Usage:
#   bash scripts/save_map.sh                 # writes maps/warehouse_<timestamp>
#   bash scripts/save_map.sh warehouse_v1    # writes maps/warehouse_v1
#   bash scripts/save_map.sh /abs/path/foo   # writes /abs/path/foo.{pgm,yaml}
#
# The naming convention without a directory prefix puts the artifacts
# under <repo>/maps/, which is .gitignored so we don't accidentally
# commit big binary maps.
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[save_map] ERROR: 'ros2' not found. source scripts/dev_env.sh first." >&2
	exit 2
fi

if ! ros2 pkg list 2>/dev/null | grep -q '^nav2_map_server$'; then
	echo "[save_map] ERROR: 'nav2_map_server' not installed." >&2
	echo "  sudo apt install ros-jazzy-nav2-map-server" >&2
	exit 2
fi

_NAME="${1:-}"
if [ -z "${_NAME}" ]; then
	# Default: maps/warehouse_<UTC timestamp>
	_TS="$(date -u +%Y%m%d_%H%M%S)"
	_NAME="warehouse_${_TS}"
fi

# If user passed a relative name with no slashes, root it under maps/.
case "${_NAME}" in
	/* | */*) _BASE="${_NAME}" ;;
	*) _BASE="${PROJECT_ROOT:-$(pwd)}/maps/${_NAME}" ;;
esac

_DIR="$(dirname "${_BASE}")"
mkdir -p "${_DIR}"

echo "[save_map] writing: ${_BASE}.pgm + ${_BASE}.yaml"
echo "[save_map] (this requires /map to be currently published by slam_toolbox)"

# nav2's map_saver_cli takes -f WITHOUT extension and writes both .pgm + .yaml.
# It exits 0 only after a successful write; on failure it usually times out.
ros2 run nav2_map_server map_saver_cli \
	-f "${_BASE}" \
	--ros-args -p use_sim_time:=true

_RC=$?
if [ "${_RC}" -ne 0 ]; then
	echo "[save_map] ERROR: map_saver_cli exited ${_RC}." >&2
	exit "${_RC}"
fi

if [ -f "${_BASE}.pgm" ] && [ -f "${_BASE}.yaml" ]; then
	echo "[save_map] OK"
	echo "  pgm:  ${_BASE}.pgm  ($(wc -c < "${_BASE}.pgm") bytes)"
	echo "  yaml: ${_BASE}.yaml"
	echo
	echo "Inspect:"
	echo "  eog ${_BASE}.pgm        # visual sanity check"
	echo "  cat ${_BASE}.yaml       # resolution / origin"
else
	echo "[save_map] ERROR: expected output files missing." >&2
	exit 1
fi
