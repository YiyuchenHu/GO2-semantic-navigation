#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# debug_nav2_action_chain.sh — Day 9 NavigateToPose chain probe.
#
# Goal:
#   When the user says "go to table" but Go2 doesn't move, we need to
#   localise the failure on this chain in seconds:
#
#     /target/selected
#         ↓
#     approach_goal_planner
#         ├── /semantic_goal/goal_pose         (RViz debug, no Nav2 yet)
#         ├── /semantic_goal/action_debug      (Day 9 — SEND/ACCEPT/REJECT/RESULT)
#         └── NavigateToPose action
#                 ↓
#             Nav2 bt_navigator
#                 ├── /navigation/status       (mirrored by approach_planner)
#                 ├── /arrival/status          (mirrored on SUCCEEDED/ABORTED)
#                 └── /cmd_vel                 (controller_server output)
#
# Day 9 hot-fix (Task 1): each topic is echoed EXACTLY ONCE and the
# capture is reused for both the per-section dump and the final
# summary. Earlier the summary called ``ros2 topic echo --once`` a
# second time, which routinely raced out on volatile QoS — leading
# to the "the dump section says target=person_001, the summary says
# silent" bug.
#
# Every command is wrapped in `timeout` so this script always returns.
#
# Usage:
#   bash scripts/debug_nav2_action_chain.sh
#   ECHO_TIMEOUT=20 bash scripts/debug_nav2_action_chain.sh
# -----------------------------------------------------------------------------
set -e

ECHO_TIMEOUT="${ECHO_TIMEOUT:-10}"
TMP_ROOT="$(mktemp -d -t debug-nav2-action-chain.XXXXXX)"
trap 'rm -rf "$TMP_ROOT"' EXIT

# Source ROS env best-effort. We deliberately do NOT `set -u` because
# /opt/ros/jazzy/setup.bash references unbound vars on first source.
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
#   2. Stores the body in $TMP_ROOT/$SLOT.
#   3. Echoes a labelled section to stdout.
# Subsequent get_capture SLOT readbacks return the cached body.
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
echo "Day 9 Nav2 action-chain probe (timeout=${ECHO_TIMEOUT}s per topic)"
bar

# Layer 1 — target_selector input/output ----------------------------------
capture_topic L1_target /target/selected go2_msgs/msg/SelectedTarget

# Layer 2 — approach_goal_planner debug topics ---------------------------
capture_topic L2_goal_pose /semantic_goal/goal_pose \
  geometry_msgs/msg/PoseStamped
capture_topic L2_action_debug /semantic_goal/action_debug \
  std_msgs/msg/String

# Layer 2b — pairing invariant (uses cached captures, no extra echo).
bar
echo "[L2b pairing] check action_debug woke up after goal_pose"
GP_BODY="$(get_capture L2_goal_pose)"
AD_BODY="$(get_capture L2_action_debug)"
if [[ -z "$GP_BODY" ]]; then
  echo "(skipped — /semantic_goal/goal_pose itself never showed up)"
elif [[ -z "$AD_BODY" ]]; then
  echo "[FAIL] /semantic_goal/goal_pose is alive but"
  echo "       /semantic_goal/action_debug never emitted — likely:"
  echo "         * approach_planner pre-Day-9 build still loaded"
  echo "           (rebuild & relaunch go2_semantic_perception)"
  echo "         * action_debug_emit_throttled=false"
  echo "         * topic remap of /semantic_goal/action_debug"
else
  echo "[OK] /semantic_goal/action_debug active alongside goal_pose."
fi

# Layer 3 — Nav2 action client visibility ---------------------------------
bar
echo "[L3 actions] timeout ${ECHO_TIMEOUT} ros2 action list | grep navigate"
ACTIONS_FILE="$TMP_ROOT/L3_actions"
timeout "$ECHO_TIMEOUT" ros2 action list 2>/dev/null \
  | grep -i navigate >"$ACTIONS_FILE" || true
if [[ -s "$ACTIONS_FILE" ]]; then
  cat "$ACTIONS_FILE"
else
  echo "(no navigate_* actions found — Nav2 lifecycle not active?)"
fi

# Layer 4 — status mirror published by approach_goal_planner / verifier --
capture_topic L4_navigation_status /navigation/status std_msgs/msg/String
capture_topic L4_arrival_status    /arrival/status    std_msgs/msg/String

# Layer 5 — actual motion command ----------------------------------------
capture_topic L5_cmd_vel /cmd_vel geometry_msgs/msg/Twist

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
bar
echo "Summary"
bar

SEL="$(get_capture L1_target)"
GOAL="$(get_capture L2_goal_pose)"
ACTION="$(get_capture L2_action_debug)"
NAVST="$(get_capture L4_navigation_status)"
ARRST="$(get_capture L4_arrival_status)"
CMDVEL="$(get_capture L5_cmd_vel)"
ACTIONS="$(cat "$ACTIONS_FILE" 2>/dev/null || true)"

# --- /target/selected ---------------------------------------------------------
# `ros2 topic echo` prints `entity_id: person_001` (NO quotes) — so the
# old regex `entity_id: ".+"` was wrong. Match a non-empty token that is
# not "''" (the empty-target literal).
if [[ -n "$SEL" ]]; then
  ENTITY="$(printf '%s\n' "$SEL" | awk -F': ' '$1 ~ /^entity_id$/ {print $2; exit}')"
  CLS="$(printf '%s\n' "$SEL" | awk -F': ' '$1 ~ /^class_label$/ {print $2; exit}')"
  REACH="$(printf '%s\n' "$SEL" | awk -F': ' '$1 ~ /^reachable$/ {print $2; exit}')"
  if [[ -n "$ENTITY" && "$ENTITY" != "''" && "$ENTITY" != '""' ]]; then
    echo "[OK] target selected: ${ENTITY} ${CLS} reachable=${REACH:-?}"
  else
    echo "[--] /target/selected message present but entity_id is empty"
    echo "     (target_selector saw no matching candidate)"
  fi
else
  echo "[--] /target/selected silent — NL parser or target_selector"
  echo "     did not commit a target within ${ECHO_TIMEOUT}s."
fi

# --- /semantic_goal/goal_pose ------------------------------------------------
if [[ -n "$GOAL" ]] && echo "$GOAL" | grep -q "frame_id"; then
  GP_X="$(printf '%s\n' "$GOAL" | awk '/position:/ {flag=1; next} flag && /^ *x:/ {print $2; exit}')"
  GP_Y="$(printf '%s\n' "$GOAL" | awk '/position:/ {flag=1; next} flag && /^ *y:/ {print $2; exit}')"
  echo "[OK] /semantic_goal/goal_pose alive at (${GP_X:-?}, ${GP_Y:-?})"
else
  echo "[!!] /semantic_goal/goal_pose silent — approach_goal_planner"
  echo "     is not running OR target lookup failed (check planner logs)."
fi

# --- /semantic_goal/action_debug --------------------------------------------
DATA="$(printf '%s\n' "$ACTION" | sed -n 's/^data: //p' \
        | sed 's/^"//; s/"$//' | tail -n1)"
if [[ -z "$DATA" ]]; then
  echo "[--] /semantic_goal/action_debug silent. Either approach_planner"
  echo "     never called send_goal_async, or the debug topic was renamed."
else
  echo "[OK] action_debug emitted: $DATA"
  case "$DATA" in
    *"NavigateToPose SEND_FAILED"*)
      echo "[!!] action server unavailable — Nav2 lifecycle inactive?"
      ;;
    *"NavigateToPose REJECTED"*)
      echo "[!!] bt_navigator rejected the goal — investigate goal pose."
      ;;
    *"NavigateToPose RESULT: SUCCEEDED"*)
      echo "[OK] Nav2 SUCCEEDED on the latest goal."
      ;;
    *"NavigateToPose RESULT: ABORTED"*)
      echo "[!!] Nav2 ABORTED — run scripts/check_nav2_costmap.sh."
      ;;
    *"NavigateToPose RESULT: CANCELED"*)
      echo "[--] Nav2 CANCELED — typically self-preemption (new goal)."
      ;;
    *"NavigateToPose NOSEND reason=throttled_target_unchanged"*)
      echo "[OK] same target still being throttled — earlier SEND already in flight."
      ;;
    *"NavigateToPose NOSEND"*)
      echo "[!!] approach_planner had a target but every ring sample"
      echo "     was rejected (see goal_candidates RViz layer)."
      ;;
    *"NavigateToPose IN_FLIGHT"*)
      echo "[OK] Nav2 goal still executing (action result not back yet)."
      ;;
    *"NavigateToPose ACCEPTED"*)
      echo "[OK] Nav2 accepted the goal — execution underway."
      ;;
    *"NavigateToPose SEND"*)
      echo "[OK] approach_planner did send a goal."
      ;;
  esac
fi

# --- Nav2 actions list ------------------------------------------------------
if echo "$ACTIONS" | grep -q "/navigate_to_pose"; then
  echo "[OK] /navigate_to_pose action present"
else
  echo "[!!] /navigate_to_pose action missing — Nav2 not up"
fi

# --- /navigation/status -----------------------------------------------------
if [[ -n "$NAVST" ]] && echo "$NAVST" | grep -q "data:"; then
  NAVST_VAL="$(printf '%s\n' "$NAVST" | sed -n 's/^data: //p' \
              | sed 's/^"//; s/"$//' | tail -n1)"
  echo "[OK] /navigation/status latest: ${NAVST_VAL}"
else
  echo "[--] /navigation/status silent (no SUCCEEDED/ABORTED yet)"
fi

# --- /arrival/status --------------------------------------------------------
if [[ -n "$ARRST" ]] && echo "$ARRST" | grep -q "data:"; then
  ARRST_VAL="$(printf '%s\n' "$ARRST" | sed -n 's/^data: //p' \
              | sed 's/^"//; s/"$//' | tail -n1)"
  echo "[OK] /arrival/status latest: ${ARRST_VAL}"
else
  echo "[--] /arrival/status silent (no ARRIVED_CONFIRMED / ARRIVAL_FAILED yet)"
fi

# --- /cmd_vel ---------------------------------------------------------------
# Detect "active" by looking for any non-zero linear/angular component.
# `ros2 topic echo --once` prints e.g.:
#   linear:
#     x: 0.0
#     y: 0.0
#     z: 0.0
#   angular:
#     x: 0.0
#     y: 0.0
#     z: 0.42475559106321303
# We extract the six numeric fields and check if any is non-zero (>0.001).
if [[ -n "$CMDVEL" ]]; then
  NONZERO="$(printf '%s\n' "$CMDVEL" | awk '
    /^ *(x|y|z): / {
      v = $2 + 0
      if (v < 0) v = -v
      if (v > 0.001) { print "yes"; exit }
    }')"
  if [[ "$NONZERO" == "yes" ]]; then
    echo "[OK] cmd_vel active (nonzero linear/angular component)"
  else
    echo "[OK] cmd_vel publishing but currently zero"
    echo "     (robot stopped — could be arrived or stalled)"
  fi
else
  echo "[--] /cmd_vel silent — Nav2 is not commanding the locomotion stack"
fi

bar
echo "Done. If the chain breaks at:"
echo "  L1: NL parser / target_selector — run scripts/debug_target_select.sh"
echo "  L2: approach_planner — read planner logs for 'no costmap-clear' WARN"
echo "  L3: Nav2 lifecycle — ros2 lifecycle list /lifecycle_manager_navigation"
echo "  L4: status mirror — fires on Nav2 RESULT or distance-based arrival"
echo "  L5: /cmd_vel — see scripts/diagnose_cmd_vel_rates.sh"
bar
