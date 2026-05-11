#!/usr/bin/env bash
# Wrapper around scripts/record_lidar_health.py — sources the
# ROS 2 Jazzy distro and the local overlay (if built) before
# invoking the recorder. Exits with the recorder's status code.
#
# Usage:
#   bash scripts/record_lidar_health.sh                # 600 s default
#   bash scripts/record_lidar_health.sh 120            # 120 s
#   bash scripts/record_lidar_health.sh 300 --no-print-live
#
# All arguments after the first are passed verbatim to the python
# entry point.

set -eo pipefail
# NB: deliberately NOT using `-u`. /opt/ros/jazzy/setup.bash
# references AMENT_TRACE_SETUP_FILES without a default, which trips
# `set -u` on a fresh shell.

DURATION="${1:-600}"
shift || true

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1091
  source install/setup.bash || \
    echo "[record_lidar_health.sh] WARN: failed to source install/setup.bash; this is fine for the lidar/scan/clock topics." >&2
fi

exec python3 scripts/record_lidar_health.py \
  --duration-sec "${DURATION}" \
  "$@"
