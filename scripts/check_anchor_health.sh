#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# check_anchor_health.sh — Day 9 semantic-anchor quick check.
#
# Goal:
#   Answer five operator questions in one pass:
#     1. Is /detections_3d producing 3D detections for person/table?
#     2. Is the PointCloud2 cluster anchor succeeding?
#     3. Is the occupancy-island anchor succeeding?
#     4. Are there confirmed semantic landmarks?
#     5. Is /target/selected pointing at one of those landmarks?
#
# Each `ros2 topic echo --once` is wrapped in `timeout 10` so the script
# never hangs. We DO NOT call `set -u` because sourcing the ROS env
# trips on unbound vars; per-command fallthroughs handle missing topics.
#
# Usage:
#   bash scripts/check_anchor_health.sh
#
# Run AFTER the day8_two_phase stack is up.
# -----------------------------------------------------------------------------
set -e

ECHO_TIMEOUT="${ECHO_TIMEOUT:-10}"

# Best-effort ROS env source. The script is meant to be runnable from a
# clean shell; if /opt/ros/jazzy isn't there we fall back to whatever the
# caller already sourced.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 || true
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash >/dev/null 2>&1 || true
fi

bar() {
  printf '====================================================================\n'
}

dump_topic_once() {
  # $1 = label, $2 = topic, $3 = optional ros2 type
  local label="$1"
  local topic="$2"
  local typ="${3:-}"
  bar
  echo "[$label] $topic"
  if [[ -n "$typ" ]]; then
    timeout "$ECHO_TIMEOUT" ros2 topic echo "$topic" "$typ" --once \
      2>/dev/null \
      || echo "(no msg in ${ECHO_TIMEOUT}s on $topic)"
  else
    timeout "$ECHO_TIMEOUT" ros2 topic echo "$topic" --once \
      2>/dev/null \
      || echo "(no msg in ${ECHO_TIMEOUT}s on $topic)"
  fi
}

dump_topic_count() {
  # Count messages in <timeout> seconds, return integer.
  local topic="$1"
  local secs="$2"
  ros2 topic hz "$topic" --window 1 \
    >/tmp/_anchor_health_hz_$$ 2>&1 &
  local pid=$!
  sleep "$secs"
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  if grep -q "average rate" /tmp/_anchor_health_hz_$$; then
    grep "average rate" /tmp/_anchor_health_hz_$$ \
      | head -n1 \
      | sed 's/.*average rate: \([0-9.]*\).*/\1/'
  else
    echo "0"
  fi
  rm -f /tmp/_anchor_health_hz_$$
}

bar
echo "Day 9 anchor-health probe (timeout=${ECHO_TIMEOUT}s per topic)"
bar

dump_topic_once "L1 detections_3d (depth_projector)" /detections_3d \
  vision_msgs/msg/Detection3DArray
dump_topic_once "L2 semantic_map/objects (memory)"   /semantic_map/objects \
  go2_msgs/msg/SemanticEntityArray
dump_topic_once "L3 anchor_debug_stats"              /semantic_map/anchor_debug_stats \
  std_msgs/msg/String
dump_topic_once "L3b island_debug_markers"           /semantic_map/island_debug_markers \
  visualization_msgs/msg/MarkerArray
dump_topic_once "L4 target/selected"                 /target/selected \
  go2_msgs/msg/SelectedTarget

bar
echo "Summary"
bar

# --- 1. /detections_3d health -------------------------------------------------
DET3D=$(timeout "$ECHO_TIMEOUT" ros2 topic echo /detections_3d \
        --once 2>/dev/null || echo "")
if echo "$DET3D" | grep -q "class_id: person"; then
  echo "[OK] person 3D detection present in /detections_3d"
else
  echo "[--] no person in latest /detections_3d snapshot"
fi
if echo "$DET3D" | grep -Eq "class_id: (table|desk|workbench|dining table)"; then
  echo "[OK] table-class 3D detection present in /detections_3d"
else
  echo "[--] no table-class in latest /detections_3d snapshot"
fi

# --- 2. anchor_debug_stats parsing -------------------------------------------
STATS=$(timeout "$ECHO_TIMEOUT" ros2 topic echo \
         /semantic_map/anchor_debug_stats --once 2>/dev/null \
         | grep -oE 'data: ".*"' \
         | sed 's/^data: "//; s/"$//' \
         || echo "")
if [[ -z "$STATS" ]]; then
  echo "[!!] /semantic_map/anchor_debug_stats is silent — semantic_memory \
not running, or anchor_debug_stats_period_sec=0?"
else
  pc_ok=$(echo "$STATS" | grep -oE 'pointcloud_anchor_success=[0-9]+' \
            | head -n1 | cut -d= -f2)
  isl_ok=$(echo "$STATS" | grep -oE 'occupancy_island_anchor_success=[0-9]+' \
             | head -n1 | cut -d= -f2)
  none=$(echo "$STATS" | grep -oE 'candidate_no_anchor=[0-9]+' \
           | head -n1 | cut -d= -f2)
  pc_buf=$(echo "$STATS" | grep -oE 'pointcloud_buffer=[0-9]+' \
             | head -n1 | cut -d= -f2)
  map_avail=$(echo "$STATS" | grep -oE 'map_available=[0-9]+' \
                | head -n1 | cut -d= -f2)
  promote_pc=$(echo "$STATS" | grep -oE 'promoted_confirmed_by_pc=[0-9]+' \
                | head -n1 | cut -d= -f2)
  promote_isl=$(echo "$STATS" | grep -oE 'promoted_confirmed_by_island=[0-9]+' \
                | head -n1 | cut -d= -f2)
  pc_disagree=$(echo "$STATS" | grep -oE 'pc_map_disagreement=[0-9]+' \
                | head -n1 | cut -d= -f2)
  invalidated=$(echo "$STATS" | grep -oE 'invalidated=[0-9]+' \
                | head -n1 | cut -d= -f2)
  rej_no_pc=$(echo "$STATS" | grep -oE 'rejected_no_pointcloud=[0-9]+' \
                | head -n1 | cut -d= -f2)
  rej_no_pts=$(echo "$STATS" | \
    grep -oE 'rejected_no_points_near_detection=[0-9]+' \
    | head -n1 | cut -d= -f2)
  rej_no_island=$(echo "$STATS" | \
    grep -oE 'rejected_no_occupied_island_nearby=[0-9]+' \
    | head -n1 | cut -d= -f2)
  rej_wall=$(echo "$STATS" | grep -oE 'rejected_wall_like_island=[0-9]+' \
              | head -n1 | cut -d= -f2)
  echo "[--] PointCloud anchor success counter   : ${pc_ok:-?}"
  echo "[--] Occupancy island success counter    : ${isl_ok:-?}"
  echo "[--] Unanchored / candidate-only counter : ${none:-?}"
  echo "[--] Promoted by PC / island             : ${promote_pc:-?} / ${promote_isl:-?}"
  echo "[--] PC vs map disagreement events       : ${pc_disagree:-?}"
  echo "[--] Invalidated confirmed entities      : ${invalidated:-?}"
  echo "[--] PointCloud buffer / map available   : ${pc_buf:-?} / ${map_avail:-?}"
  echo "[--] Reject pc:no_cloud, no_pts, no_isl, wall_like : "
  echo "      ${rej_no_pc:-?} / ${rej_no_pts:-?} / ${rej_no_island:-?} / ${rej_wall:-?}"
  if [[ "${pc_ok:-0}" -gt 0 ]]; then
    echo "[OK] PointCloud cluster anchor is succeeding."
  else
    if [[ "${pc_buf:-0}" -eq 0 ]]; then
      echo "[!!] No /lidar/points received yet (pointcloud_buffer=0)."
      echo "     Run: ros2 topic hz /lidar/points  — confirm sensor is on."
    elif [[ "${rej_no_pts:-0}" -gt 0 ]]; then
      echo "[!!] Detections fell outside the per-class search radius. "
      echo "     Try raising person_pointcloud_search_radius_m / "
      echo "     table_pointcloud_search_radius_m."
    else
      echo "[!!] PointCloud anchor not yet succeeding — check /lidar/points TF."
    fi
  fi
  if [[ "${isl_ok:-0}" -gt 0 ]]; then
    echo "[OK] Occupancy-island anchor is producing anchors."
  else
    if [[ "${map_avail:-0}" -eq 0 ]]; then
      echo "[!!] /map not yet latched into semantic_memory."
    else
      echo "[--] Occupancy-island anchor has not fired (often expected for "
      echo "     person; PointCloud anchor is the primary path)."
    fi
  fi
fi

# --- 3. confirmed-landmark count via /semantic_map/objects -------------------
OBJS=$(timeout "$ECHO_TIMEOUT" ros2 topic echo /semantic_map/objects --once \
         2>/dev/null || echo "")
if [[ -z "$OBJS" ]]; then
  echo "[!!] /semantic_map/objects is silent."
else
  n_confirmed=$(echo "$OBJS" | grep -c '|confirmed|' || true)
  n_candidate=$(echo "$OBJS" | grep -c '|candidate|' || true)
  n_invalid=$(echo "$OBJS" | grep -ci 'invalid' || true)
  n_pc_anchor=$(echo "$OBJS" | grep -c '|confirmed|pc_' || true)
  n_isl_anchor=$(echo "$OBJS" | grep -c '|confirmed|isl_' || true)
  echo "[--] /semantic_map/objects: confirmed=${n_confirmed:-0} "\
"(pc=${n_pc_anchor:-0}, isl=${n_isl_anchor:-0}), "\
"candidate=${n_candidate:-0}, invalid≈${n_invalid:-0}"
  if [[ "${n_confirmed:-0}" -gt 0 ]]; then
    echo "[OK] Confirmed semantic landmarks exist."
  else
    echo "[!!] No confirmed landmarks. Drive the robot toward person/table."
  fi
fi

# --- 4. /target/selected validity --------------------------------------------
SEL=$(timeout "$ECHO_TIMEOUT" ros2 topic echo /target/selected --once \
       2>/dev/null || echo "")
if [[ -z "$SEL" ]]; then
  echo "[!!] /target/selected is silent (no target_selector publish)."
else
  if echo "$SEL" | grep -q 'class_label: ""'; then
    echo "[--] /target/selected published but class_label is empty."
  else
    cls=$(echo "$SEL" | grep -oE 'class_label: ".*"' | head -n1 \
           | cut -d'"' -f2)
    eid=$(echo "$SEL" | grep -oE 'entity_id: ".*"' | head -n1 \
           | cut -d'"' -f2)
    echo "[OK] /target/selected -> class=${cls:-?} entity=${eid:-?}"
  fi
fi

bar
echo "Tip: run continuously with 'watch -n 2 bash scripts/check_anchor_health.sh'."
bar
