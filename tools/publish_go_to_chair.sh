#!/usr/bin/env bash
set -euo pipefail

echo "[integration] Publishing user command: go to the chair"
ros2 run go2_debug_tools integration_command_publisher \
  --ros-args \
  -p command_text:="go to the chair" \
  -p startup_delay_sec:=1.5 \
  -p min_subscribers:=2 \
  -p max_wait_for_subscribers_sec:=5.0 \
  -p repeat_count:=5 \
  -p repeat_interval_sec:=0.5
