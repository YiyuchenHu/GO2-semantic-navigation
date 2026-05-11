#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# debug_mapping_explorer.sh — Day 9+ Phase-A frontier-mapping probe.
#
# Goal:
#   When Go2's autonomous mapping looks stuck — Go2 standing still,
#   the same frontier sphere flickering in RViz, no progress on the
#   /map — answer in seconds:
#
#       Is mapping_explorer NAVIGATING / IDLE / DONE / FAILED?
#       Does it have a current frontier goal? How far is the robot?
#       Is it re-using the same frontier centroid as last cycle?
#       Are the frontier markers fresh, or stale leftover from before?
#       Is /cmd_vel actually moving the robot?
#
# Real exploration goal pipeline (to disambiguate from RViz "2D Goal Pose"):
#
#       /map (slam_toolbox)
#         ↓
#     frontier_explorer (service: /get_frontiers)
#         ↓                      └─ /frontier_markers
#         ↓                      └─ /frontier/debug/frontier_cells
#         ↓                      └─ /frontier/debug/accepted_centroids
#         ↓                      └─ /frontier/debug/rejected_centroids
#     mapping_explorer
#         ├─ /mapping/status
#         ├─ /mapping/debug/status
#         └─ NavigateToPose action  ──────► Nav2
#                                            ↓
#                                          /cmd_vel
#
# /goal_pose is RViz's manual "2D Goal Pose" topic and is NOT used by
# this autonomous loop — never confuse the two.
#
# Every command is wrapped in `timeout` so this script always returns.
# Each topic is echoed EXACTLY ONCE; the result is cached for both the
# detailed dump and the final summary, mirroring the same race-fix
# applied to debug_nav2_action_chain.sh on Day 9.
#
# Usage:
#   bash scripts/debug_mapping_explorer.sh
#   ECHO_TIMEOUT=20 bash scripts/debug_mapping_explorer.sh
# -----------------------------------------------------------------------------
set -e

ECHO_TIMEOUT="${ECHO_TIMEOUT:-10}"
TMP_ROOT="$(mktemp -d -t debug-mapping-explorer.XXXXXX)"
trap 'rm -rf "$TMP_ROOT"' EXIT

# Source ROS env best-effort. Don't `set -u` — /opt/ros/jazzy/setup.bash
# references unbound vars on first source.
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

# capture_topic SLOT TOPIC [TYPE]
#   1. Runs `ros2 topic echo --once` ONCE.
#   2. Stores the body in $TMP_ROOT/$SLOT (cached for summary).
#   3. Echoes a labelled section to stdout.
capture_topic() {
  local slot="$1"
  local topic="$2"
  local typ="${3:-}"
  local out_file="$TMP_ROOT/$slot"
  bar
  echo "[$slot] $topic --once  (timeout=${ECHO_TIMEOUT}s)"
  if [[ -n "$typ" ]]; then
    timeout "$ECHO_TIMEOUT" ros2 topic echo "$topic" "$typ" --once \
      >"$out_file" 2>/dev/null || true
  else
    timeout "$ECHO_TIMEOUT" ros2 topic echo "$topic" --once \
      >"$out_file" 2>/dev/null || true
  fi
  if [[ -s "$out_file" ]]; then
    cat "$out_file"
  else
    echo "(no msg in ${ECHO_TIMEOUT}s on $topic)"
  fi
}

get_capture() {
  local slot="$1"
  local out_file="$TMP_ROOT/$slot"
  if [[ -s "$out_file" ]]; then
    cat "$out_file"
  fi
}

bar
echo "Day 9+ mapping_explorer probe (timeout=${ECHO_TIMEOUT}s per topic)"
bar

# Layer 1 — mapping_explorer FSM ----------------------------------------------
capture_topic L1_status       /mapping/status        std_msgs/msg/String
capture_topic L1_debug_status /mapping/debug/status  std_msgs/msg/String

# Layer 2 — frontier markers (yellow cells / green / red / legacy 3D) --------
capture_topic L2_frontiers   /frontier_markers \
  visualization_msgs/msg/MarkerArray
capture_topic L2_accepted    /frontier/debug/accepted_centroids \
  visualization_msgs/msg/MarkerArray
capture_topic L2_rejected    /frontier/debug/rejected_centroids \
  visualization_msgs/msg/MarkerArray
capture_topic L2_cells       /frontier/debug/frontier_cells \
  visualization_msgs/msg/Marker

# Layer 3 — locomotion command -----------------------------------------------
capture_topic L3_cmd_vel /cmd_vel geometry_msgs/msg/Twist

# Layer 4 — Nav2 status mirror (only present when approach_planner attaches) -
capture_topic L4_navigation_status /navigation/status std_msgs/msg/String

# Layer 5 — node info (no `--once`; ros2 node info terminates on its own) ----
bar
echo "[L5_mapping_node] ros2 node info /mapping_explorer"
NODE_FILE_M="$TMP_ROOT/L5_mapping_node"
timeout "$ECHO_TIMEOUT" ros2 node info /mapping_explorer \
  >"$NODE_FILE_M" 2>/dev/null || true
if [[ -s "$NODE_FILE_M" ]]; then
  cat "$NODE_FILE_M"
else
  echo "(node /mapping_explorer not reachable within ${ECHO_TIMEOUT}s — not running?)"
fi

bar
echo "[L5_frontier_node] ros2 node info /frontier_explorer"
NODE_FILE_F="$TMP_ROOT/L5_frontier_node"
timeout "$ECHO_TIMEOUT" ros2 node info /frontier_explorer \
  >"$NODE_FILE_F" 2>/dev/null || true
if [[ -s "$NODE_FILE_F" ]]; then
  cat "$NODE_FILE_F"
else
  echo "(node /frontier_explorer not reachable within ${ECHO_TIMEOUT}s — not running?)"
fi

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
bar
echo "Summary"
bar

ST="$(get_capture L1_status)"
DBG="$(get_capture L1_debug_status)"
FR="$(get_capture L2_frontiers)"
ACC="$(get_capture L2_accepted)"
REJ="$(get_capture L2_rejected)"
CELLS="$(get_capture L2_cells)"
CMDVEL="$(get_capture L3_cmd_vel)"
NAVST="$(get_capture L4_navigation_status)"

# --- /mapping/status ---------------------------------------------------------
ST_VAL=""
if [[ -n "$ST" ]]; then
  ST_VAL="$(printf '%s\n' "$ST" | sed -n 's/^data: //p' \
            | sed 's/^"//; s/"$//' | tail -n1)"
fi
if [[ -n "$ST_VAL" ]]; then
  echo "[OK] /mapping/status = ${ST_VAL}"
else
  echo "[!!] /mapping/status silent — mapping_explorer not running OR"
  echo "     latched value not delivered (TRANSIENT_LOCAL mismatch?)."
fi

# --- /mapping/debug/status — pull the operator-friendly fields --------------
DBG_DATA=""
if [[ -n "$DBG" ]]; then
  DBG_DATA="$(printf '%s\n' "$DBG" | sed -n 's/^data: //p' \
              | sed 's/^"//; s/"$//' | tail -n1)"
fi
if [[ -z "$DBG_DATA" ]]; then
  echo "[--] /mapping/debug/status silent — older mapping_explorer build?"
  echo "     Rebuild go2_navigation and relaunch day8_two_phase."
else
  echo "[OK] mapping/debug/status raw: $DBG_DATA"
  # Parse a few key fields for quick triage. Format is
  # `state=NAVIGATING goal=(3.20,-1.40) dist=0.42 nav2=ACCEPTED ...`.
  STATE_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | grep -oE 'state=[A-Z]+' | head -n1 | cut -d= -f2)"
  GOAL_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | grep -oE 'goal=\([^)]*\)|goal=none' | head -n1 \
                  | cut -d= -f2)"
  LAST_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | grep -oE 'last_goal=\([^)]*\)|last_goal=none' | head -n1 \
                  | cut -d= -f2)"
  DIST_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | grep -oE 'dist=[0-9.\-]+|dist=nan' | head -n1 \
                  | cut -d= -f2)"
  NAV2_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | grep -oE 'nav2=[A-Za-z0-9_:.\-]+' | head -n1 \
                  | cut -d= -f2)"
  FRONTIER_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | grep -oE 'frontiers=[0-9]+' | head -n1 \
                  | cut -d= -f2)"
  VISITED_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | grep -oE 'visited=[0-9]+\([0-9]+_active\)' \
                  | head -n1 | cut -d= -f2)"
  REASON_FIELD="$(printf '%s\n' "$DBG_DATA" \
                  | sed -n 's/.*reason=\(.*\)$/\1/p')"
  echo "     state=${STATE_FIELD:-?} goal=${GOAL_FIELD:-?}"
  echo "     dist=${DIST_FIELD:-?} nav2=${NAV2_FIELD:-?}"
  echo "     frontiers_returned=${FRONTIER_FIELD:-?} visited=${VISITED_FIELD:-?}"
  echo "     last_goal=${LAST_FIELD:-?}"
  if [[ -n "$REASON_FIELD" ]]; then
    echo "     reason=${REASON_FIELD}"
  fi
fi

# --- Frontier marker freshness ----------------------------------------------
# A "fresh" marker is one whose body parses as a MarkerArray with at
# least one ADD-action marker; an empty echo or a DELETEALL-only
# message means the topic is silent / cleared.
marker_state() {
  local body="$1"
  if [[ -z "$body" ]]; then
    echo "silent"
    return
  fi
  if echo "$body" | grep -q "action: 0"; then
    # Marker.ADD == 0
    echo "fresh"
  elif echo "$body" | grep -q "action: 3"; then
    # Marker.DELETEALL == 3 — only seen, no ADD
    echo "cleared"
  else
    echo "unknown"
  fi
}

FR_STATE="$(marker_state "$FR")"
ACC_STATE="$(marker_state "$ACC")"
REJ_STATE="$(marker_state "$REJ")"
CELLS_STATE="$(marker_state "$CELLS")"
echo "[--] frontier markers freshness:"
echo "     /frontier_markers (legacy 3D)        = ${FR_STATE}"
echo "     /frontier/debug/accepted_centroids   = ${ACC_STATE}"
echo "     /frontier/debug/rejected_centroids   = ${REJ_STATE}"
echo "     /frontier/debug/frontier_cells       = ${CELLS_STATE}"
case "${ST_VAL:-}" in
  DONE|FAILED:*|FAILED|IDLE|CANCELLED)
    if [[ "$FR_STATE" == "fresh" || "$ACC_STATE" == "fresh" \
          || "$CELLS_STATE" == "fresh" ]]; then
      echo "[!!] mapping_status=${ST_VAL} but frontier markers still fresh —"
      echo "     RViz is showing stale overlay. Make sure frontier_explorer"
      echo "     is on the Day-9+ build (it should DELETEALL on terminal"
      echo "     mapping states)."
    else
      echo "[OK] markers cleared / silent — matches terminal mapping state."
    fi
    ;;
  NAVIGATING)
    if [[ "$ACC_STATE" != "fresh" && "$CELLS_STATE" != "fresh" ]]; then
      echo "[!!] mapping is NAVIGATING but frontier markers silent —"
      echo "     frontier_explorer not publishing (check service /get_frontiers)."
    else
      echo "[OK] markers fresh while NAVIGATING — overlay healthy."
    fi
    ;;
esac

# --- Repeated-frontier suspicion --------------------------------------------
# If goal == last_goal AND nav2 != SUCCEEDED, we are likely chasing the
# same centroid. Combined with a stationary cmd_vel that's a strong hint.
if [[ -n "${GOAL_FIELD:-}" && "$GOAL_FIELD" != "none" \
      && "${LAST_FIELD:-}" == "$GOAL_FIELD" ]]; then
  case "${NAV2_FIELD:-}" in
    SUCCEEDED|ACCEPTED|SENT)
      ;;  # not suspicious yet — current attempt may still be in flight
    *)
      echo "[!!] current goal == last_goal == ${GOAL_FIELD} but nav2=${NAV2_FIELD:-?}"
      echo "     -> mapping_explorer may be repeatedly retargeting the"
      echo "        same frontier centroid. Check 'reason=' field above"
      echo "        and the visited-frontier blacklist size."
      ;;
  esac
fi

# --- /cmd_vel — robot motion ------------------------------------------------
# Detect non-zero linear/angular component (threshold 0.001).
if [[ -n "$CMDVEL" ]]; then
  NONZERO="$(printf '%s\n' "$CMDVEL" | awk '
    /^ *(x|y|z): / {
      v = $2 + 0
      if (v < 0) v = -v
      if (v > 0.001) { print "yes"; exit }
    }')"
  if [[ "$NONZERO" == "yes" ]]; then
    echo "[OK] /cmd_vel active (nonzero linear/angular component)"
  else
    echo "[OK] /cmd_vel publishing but currently zero (stopped)"
  fi
else
  echo "[--] /cmd_vel silent — Nav2 is not commanding the locomotion stack."
fi

# --- /navigation/status (optional — only when approach_planner is up) -------
if [[ -n "$NAVST" ]] && echo "$NAVST" | grep -q "data:"; then
  NAVST_VAL="$(printf '%s\n' "$NAVST" | sed -n 's/^data: //p' \
                | sed 's/^"//; s/"$//' | tail -n1)"
  echo "[OK] /navigation/status latest: ${NAVST_VAL}"
else
  echo "[--] /navigation/status silent (Phase-B planner not active OR"
  echo "     no SUCCEEDED/ABORTED yet — fine in pure mapping mode)"
fi

# --- ros2 node info presence checks -----------------------------------------
if [[ -s "$NODE_FILE_M" ]]; then
  echo "[OK] /mapping_explorer node reachable"
else
  echo "[!!] /mapping_explorer node NOT reachable — Phase A not running"
fi
if [[ -s "$NODE_FILE_F" ]]; then
  echo "[OK] /frontier_explorer node reachable"
else
  echo "[!!] /frontier_explorer node NOT reachable — service /get_frontiers down"
fi

bar
echo "Done. Common findings:"
echo "  * markers stale + status=DONE  -> ensure Day9+ frontier_explorer build"
echo "  * goal repeats + nav2 silent   -> visited blacklist / arrival not firing"
echo "  * status=IDLE for >30s         -> waiting for /map or /get_frontiers"
echo "  * frontiers=0 + status=NAVIGATING-> SLAM finished, mapping_explorer"
echo "                                     about to flip DONE after hold"
bar
