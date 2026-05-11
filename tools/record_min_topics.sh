#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:-bag_go2_chair_validation}"
echo "[integration] Recording rosbag to: ${OUT_DIR}"

ros2 bag record -o "${OUT_DIR}" --topics \
  /user_command \
  /semantic_task/request \
  /perception/detections_2d \
  /perception/objects_3d \
  /semantic_map/entities \
  /semantic_query/selected_target \
  /semantic_goal/goal_pose \
  /navigation/status \
  /arrival/status \
  /task/status \
  /safety/status \
  /tf \
  /odom
