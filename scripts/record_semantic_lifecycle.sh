#!/usr/bin/env bash
# Wrapper around scripts/record_semantic_lifecycle.py — sources the
# ROS 2 Jazzy distro and the local overlay (if built) before
# invoking the recorder. Exits with the recorder's status code.
#
# Usage:
#   bash scripts/record_semantic_lifecycle.sh                # 120 s default
#   bash scripts/record_semantic_lifecycle.sh 60             # 60 s
#   bash scripts/record_semantic_lifecycle.sh 120 --target-class table
#
# All arguments after the first are passed verbatim to the python
# entry point, so any extra flag (--no-print-live,
# --perception-rate-hz, --output-dir, ...) works.

set -eo pipefail
# NB: deliberately NOT using `-u`. /opt/ros/jazzy/setup.bash references
# AMENT_TRACE_SETUP_FILES and friends without a default, which trips
# `set -u` immediately. The rest of this wrapper is short enough that
# the marginal safety from `-u` isn't worth the breakage.

DURATION="${1:-120}"
shift || true

# Source the distro. We do NOT --no-rcfile here so the user's
# environment overrides still apply. Failures bubble up via `set -e`,
# but we tolerate sourcing to redefine intentionally-unset env vars.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

# Source the overlay if it exists. install/setup.bash is created
# by colcon build and is what carries our custom message types
# (go2_msgs). Some overlays also reference unset trace variables, so
# treat sourcing as best-effort: if it fails the recorder will warn
# about missing go2_msgs but still produce status / perception CSVs.
if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1091
  source install/setup.bash || \
    echo "[record_semantic_lifecycle.sh] WARN: failed to source install/setup.bash; go2_msgs columns will be blank." >&2
fi

exec python3 scripts/record_semantic_lifecycle.py \
  --duration-sec "${DURATION}" \
  "$@"
