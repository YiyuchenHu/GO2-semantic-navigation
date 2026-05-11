#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# diagnose_semantic_perception.sh — Day 8+ semantic-mapping smoke probe.
#
# Goal:
#   Verify the perception → memory → selection chain end-to-end. After we
#   added island association + persistent confirmed landmarks (Day 8+),
#   debugging "why did Go2 not navigate to person?" splits into 4 layers:
#
#     L1  YOLOE detects person?
#         /detections, /detections/masks
#     L2  depth_projector emits 3D points?
#         /detections_3d
#     L3  semantic_memory anchors them to obstacle islands and produces
#         confirmed landmarks?
#         /semantic_map/objects, /semantic_map/markers,
#         /semantic_map/island_debug_markers
#     L4  target_selector picks one of those landmarks?
#         /target_selector params, /target/selected
#
#   This script prints one snapshot from each layer with a 10 s timeout
#   per call so it never hangs, even if a topic is silent.
#
# Usage:
#   bash scripts/diagnose_semantic_perception.sh
#   TARGET_CLASS=person bash scripts/diagnose_semantic_perception.sh
#   TARGET_CLASS=table  bash scripts/diagnose_semantic_perception.sh
#
# Environment:
#   ECHO_TIMEOUT   default 10  — seconds for each `ros2 topic echo --once`
#   TARGET_CLASS   default "" — when set, also probes /target/selected
#                                 after pushing this class to /target_selector
#
# IMPORTANT:
#   Run this AFTER the perception + memory + selector stack is up:
#     T1: bash scripts/run_warehouse_ros2.sh
#     T2: bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py
#     T3: bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
#     T4: bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py
#   Then in any new terminal:
#     bash scripts/diagnose_semantic_perception.sh
# -----------------------------------------------------------------------------
# DO NOT use `set -u` here: sourcing /opt/ros/jazzy/setup.bash trips on
# unbound variables ($AMENT_PREFIX_PATH on a clean shell etc.) and
# `set -u`'s exit is shell-level, NOT command-level, so even
# `source ... || true` cannot catch it. The script would exit silently
# at line 4 with status 1. Diagnose-style scripts should be lenient on
# the environment they get and forgive missing nodes / topics, so we
# rely on per-command `|| echo "(no msg)"` fallthroughs instead.

ECHO_TIMEOUT="${ECHO_TIMEOUT:-10}"
TARGET_CLASS="${TARGET_CLASS:-}"

# Source ROS + workspace overlay best-effort. Errors here are non-fatal:
# even without overlay, the rest of the script can still call ros2 CLI.
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 || true
WS_INSTALL="$(pwd)/install/setup.bash"
if [ -f "$WS_INSTALL" ]; then
  # shellcheck disable=SC1090
  source "$WS_INSTALL" >/dev/null 2>&1 || true
fi

step() {
  echo
  echo "==============================================================="
  echo "==  $*"
  echo "==============================================================="
}

# ---------------------------------------------------------------------
# Stack health check — fail loud, fast, with a clear remedy.
# An empty / errored `ros2 node list` is by far the most common reason
# this script "produces nothing" when the user expects rich output. We
# probe for it up front and tell the user exactly what to launch.
# ---------------------------------------------------------------------
echo "ROS distro: ${ROS_DISTRO:-unset}"
echo "RMW:        ${RMW_IMPLEMENTATION:-unset}"
echo "Workspace overlay sourced: $([ -f "$WS_INSTALL" ] && echo yes || echo no)"
echo
echo "Probing ROS graph (timeout 4s)..."
NODES="$(timeout 4 ros2 node list 2>/dev/null || true)"
if [ -z "$NODES" ]; then
  cat <<'EOF'

!!!  ros2 node list returned EMPTY.  No ROS nodes are running. !!!

Diagnose script needs the day8_two_phase stack alive. Bring it up
in 3 separate terminals first:

  T1: bash scripts/run_warehouse_ros2.sh     (Isaac Sim + ROS bridge)
  T2: bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py
  T3: bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
  T4: bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py

Then re-run this script.

(Continuing anyway so you can see which CLI call hangs vs which
returns an empty topic — useful when only PART of the stack is up.)

EOF
else
  echo "Node count: $(printf '%s\n' "$NODES" | wc -l)"
  printf '%s\n' "$NODES" | sed 's/^/  /'
fi

echo_topic_once() {
  local topic="$1"
  shift
  echo
  echo "--- timeout ${ECHO_TIMEOUT}s ros2 topic echo --once $topic $* ---"
  timeout "${ECHO_TIMEOUT}" ros2 topic echo --once "$topic" "$@" \
    || echo "(no message within ${ECHO_TIMEOUT}s on ${topic})"
}

step "L1 — YOLOE raw detections"
echo_topic_once /detections
echo_topic_once /detections/masks

step "L2 — depth_projector 3D detections"
echo_topic_once /detections_3d

# Day 8++++ Task 5 — when /detections has content but /detections_3d
# never produces a 3D detection, depth_projector is silently dropping
# them. We grab a fresh snapshot of /detections and /detections_3d and,
# if they disagree, pull /depth_projector/debug_stats to localise the
# culprit (no masks, no depth, TF failure, sync failure, or missing
# bbox-fallback).
DET_RAW="$(timeout "${ECHO_TIMEOUT}" ros2 topic echo --once /detections 2>/dev/null \
  | tr -d '\r' || true)"
DET3_RAW="$(timeout "${ECHO_TIMEOUT}" ros2 topic echo --once /detections_3d 2>/dev/null \
  | tr -d '\r' || true)"
det_count="$(printf '%s\n' "$DET_RAW" \
  | awk '/^- header:/ { c++ } END { print c+0 }')"
det3_count="$(printf '%s\n' "$DET3_RAW" \
  | awk '/^- header:/ { c++ } END { print c+0 }')"
echo
echo "  /detections snapshot:    ${det_count} detection(s)"
echo "  /detections_3d snapshot: ${det3_count} detection(s)"
if [ "${det_count:-0}" -gt 0 ] && [ "${det3_count:-0}" -eq 0 ]; then
  echo
  echo "  *** Depth projector is dropping detections. ***"
  echo "  Pulling /depth_projector/debug_stats to identify the cause ..."
  STATS_RAW="$(timeout "${ECHO_TIMEOUT}" ros2 topic echo --once \
    /depth_projector/debug_stats 2>/dev/null | tr -d '\r' || true)"
  if [ -z "$STATS_RAW" ]; then
    echo "  ?  /depth_projector/debug_stats has no publisher within"
    echo "     ${ECHO_TIMEOUT}s. Likely causes:"
    echo "       - depth_projector_node is not running"
    echo "       - debug_stats_topic was set to '' in the launch"
    echo "       - depth_projector launched but is hung (check L1 / TF)"
  else
    echo "  raw stats: $(printf '%s' "$STATS_RAW" \
      | awk '/data: /{sub(/.*data: /,""); gsub(/[\"\r]/,""); print}')"
    # Pull individual counters out of the single-line stats body.
    get_stat() {
      printf '%s\n' "$STATS_RAW" \
        | awk -v key="$1" 'match($0, key"=[0-9]+"){print substr($0,RSTART,RLENGTH)}' \
        | head -n1 | awk -F= '{print $2+0}'
    }
    masks_received="$(get_stat masks_received)"
    detections_without_masks="$(get_stat detections_without_masks)"
    bbox_fallback_used="$(get_stat bbox_fallback_used)"
    depth_frames_received="$(get_stat depth_frames_received)"
    rejected_no_depth="$(get_stat rejected_no_depth)"
    rejected_bad_depth="$(get_stat rejected_bad_depth)"
    rejected_time_sync="$(get_stat rejected_time_sync)"
    tf_at_stamp_success="$(get_stat tf_at_stamp_success)"
    tf_failed="$(get_stat tf_failed)"
    keyframe_hits="$(get_stat keyframe_hits)"
    latest_fallback_used="$(get_stat latest_fallback_used)"
    published_3d="$(get_stat published_3d)"
    bbox_fallback_enabled="$(get_stat bbox_fallback_enabled)"
    pending_grace_drops="$(get_stat pending_grace_drops)"
    echo
    echo "  Likely causes (in order):"
    if [ "${masks_received:-0}" -eq 0 ]; then
      echo "    [LIKELY] masks empty: /detections/masks never published a"
      echo "             non-empty InstanceMaskArray. The bbox-fallback"
      echo "             path should engage automatically; if it isn't,"
      echo "             check bbox_fallback_enabled below."
    elif [ "${detections_without_masks:-0}" -gt 0 ] && \
         [ "${bbox_fallback_used:-0}" -eq 0 ]; then
      echo "    [LIKELY] masks empty: ${detections_without_masks} det(s)"
      echo "             routed to fallback but bbox_fallback_used=0 ⇒"
      echo "             bbox path itself is rejecting (no_depth/bad_depth)."
    fi
    if [ "${depth_frames_received:-0}" -eq 0 ]; then
      echo "    [LIKELY] no depth: /camera/depth/image_rect_raw has no"
      echo "             publisher. Verify the Isaac Sim depth bridge."
    fi
    if [ "${rejected_no_depth:-0}" -gt 0 ] && \
       [ "${published_3d:-0}" -eq 0 ]; then
      echo "    [LIKELY] no depth in ROI: ${rejected_no_depth} detection(s)"
      echo "             had no valid depth pixels. Check Isaac Sim depth"
      echo "             range, min_depth_m / max_depth_m parameters."
    fi
    if [ "${tf_failed:-0}" -gt 0 ] && [ "${tf_at_stamp_success:-0}" -eq 0 ]; then
      echo "    [LIKELY] TF failure: tf_failed=${tf_failed} but"
      echo "             tf_at_stamp_success=0. The map<-camera transform"
      echo "             is not available at the detection timestamp."
      echo "             Inspect /tf and /tf_static; consider raising"
      echo "             tf_lookup_timeout_sec or enabling"
      echo "             allow_latest_tf_fallback (Day 8++ flag)."
    fi
    if [ "${rejected_time_sync:-0}" -gt 0 ]; then
      echo "    [POSSIBLE] time-sync rejection: rejected_time_sync="
      echo "             ${rejected_time_sync}. Increase sync_slop or check"
      echo "             clock alignment between /detections and /depth."
    fi
    if [ "${bbox_fallback_enabled:-0}" -eq 0 ]; then
      echo "    [LIKELY] no bbox fallback: bbox_fallback_enabled=0 — the"
      echo "             projector is configured to ONLY use masks. Set"
      echo "             bbox_fallback_enabled:=true in the launch."
    fi
    if [ "${pending_grace_drops:-0}" -gt 0 ]; then
      echo "    [INFO] grace-period drops: ${pending_grace_drops} det(s)"
      echo "           dropped after mask_wait_grace_sec elapsed and"
      echo "           bbox_fallback was disabled."
    fi
    if [ "${published_3d:-0}" -gt 0 ]; then
      echo "    [INFO] published_3d=${published_3d} lifetime — projector"
      echo "           has worked at least once; this snapshot may be"
      echo "           transient (Go2 turned away mid-frame)."
    fi
    if [ "${keyframe_hits:-0}" -gt 0 ] || \
       [ "${latest_fallback_used:-0}" -gt 0 ]; then
      echo "    [INFO] tf paths used:"
      echo "           at_stamp=${tf_at_stamp_success}"
      echo "           keyframe=${keyframe_hits}"
      echo "           latest_fallback=${latest_fallback_used}"
    fi
  fi
fi

step "L3a — semantic_memory persistent objects (canonical class + display_name)"
echo "    display_name format: '<raw>|confirmed|<island_id>'  or"
echo "                         '<raw>|candidate|<island_id_or_'-'>'"
echo_topic_once /semantic_map/objects --field entities

step "L3b — semantic_memory marker topics"
# `--field markers` instead of full echo because markers are big.
echo "/semantic_map/markers       confirmed landmarks (persistent, opaque)"
echo "/semantic_map/debug_markers candidate landmarks (transparent)"
echo "/semantic_map/island_debug_markers per-frame island association"
ros2 topic list -t 2>/dev/null | grep -E '^/semantic_map/' \
  || echo "(no /semantic_map/* topics found)"

step "L3c — compact entity table + suspicious-entity highlights"
# Pull /semantic_map/objects once and pretty-print id / class / status /
# island / obs / visible / dist into a single row per entity, plus a
# trailing diagnostic section that calls out:
#
#   * confirmed person with obs_count=1
#   * confirmed person without an island anchor (display_name "...|-")
#   * |invalid| confirmed-turned-bad landmarks
#   * multiple confirmed person landmarks (MVP demo expects 1)
#
# We compute base_link distance via tf2_echo with a 1 s timeout. When TF
# is unavailable we print "?" — exactly the case Task 6 of the
# semantic-perception fixes addresses, so a "?" here means the
# target_selector dist will be NaN, not silently 0.00m.
echo "Probing /map -> base_link via tf2_echo (timeout 2s) ..."
TF_RAW="$(timeout 2 ros2 run tf2_ros tf2_echo --once map base_link 2>/dev/null \
  | tr -d '\r' || true)"
if printf '%s' "$TF_RAW" | grep -q 'Translation:'; then
  ROBOT_X="$(printf '%s' "$TF_RAW" \
    | awk '/Translation:/ {gsub(/[][,]/, " "); print $3}')"
  ROBOT_Y="$(printf '%s' "$TF_RAW" \
    | awk '/Translation:/ {gsub(/[][,]/, " "); print $4}')"
  echo "  base_link@map = (${ROBOT_X:-?}, ${ROBOT_Y:-?})"
else
  ROBOT_X=""
  ROBOT_Y=""
  echo "  base_link@map = UNKNOWN (TF lookup failed; target_selector dist will be NaN)"
fi

ENTITIES_RAW="$(timeout "${ECHO_TIMEOUT}" ros2 topic echo --once \
  /semantic_map/objects 2>/dev/null || true)"
if [ -z "$ENTITIES_RAW" ]; then
  echo "(no /semantic_map/objects message within ${ECHO_TIMEOUT}s)"
else
  printf '%s\n' "$ENTITIES_RAW" \
    | ROBOT_X="${ROBOT_X:-}" ROBOT_Y="${ROBOT_Y:-}" awk '
    BEGIN {
      rx=ENVIRON["ROBOT_X"]; ry=ENVIRON["ROBOT_Y"];
      have_robot = (rx != "" && ry != "");
      printf "  %-20s %-8s %-9s %-25s %-5s %-7s %-7s %-6s\n",
             "entity_id", "class", "status", "island_id_or_-",
             "obs", "vis", "dist_m", "z(m)";
      printf "  %s\n", \
             "--------------------------------------------------------------------------------";
    }
    /^entities:/ { in_arr=1; next }
    function flush_row() {
      if (eid == "") return;
      status="?"; island="?";
      n=split(dn, parts, "|");
      if (n >= 1) raw_label=parts[1];
      if (n >= 2) status=parts[2];
      if (n >= 3) island=parts[3];
      d="?";
      if (have_robot && px != "" && py != "") {
        dx=px-rx; dy=py-ry;
        d=sqrt(dx*dx+dy*dy);
      }
      flag="";
      if (cls=="person" && status=="confirmed" && obs+0 == 1)
        flag=flag " SUSPICIOUS:obs=1";
      if (cls=="person" && status=="confirmed" && island=="-")
        flag=flag " SUSPICIOUS:no_island";
      if (status=="invalid")
        flag=flag " INVALID";
      if (cls=="person" && status=="confirmed") confirmed_person_count++;
      printf "  %-20s %-8s %-9s %-25s %-5s %-7s %-7s %-6s%s\n",
             eid, cls, status, island, obs, vis,
             (d=="?" ? "?" : sprintf("%.2f", d)),
             (pz=="" ? "?" : sprintf("%.2f", pz)),
             flag;
      eid=""; cls=""; obs=""; vis=""; dn="";
      px=""; py=""; pz="";
    }
    in_arr && /^- header:/ { flush_row(); next }
    in_arr && /  entity_id: / {
      sub(/.*entity_id: /, "", $0); gsub(/[\"\r]/, "", $0); eid=$0; next
    }
    in_arr && /  class_label: / {
      sub(/.*class_label: /, "", $0); gsub(/[\"\r]/, "", $0); cls=$0; next
    }
    in_arr && /  display_name: / {
      sub(/.*display_name: /, "", $0); gsub(/[\"\r]/, "", $0); dn=$0; next
    }
    in_arr && /^      x:/ && px=="" { sub(/.*x: /, "", $0); px=$0+0; next }
    in_arr && /^      y:/ && py=="" { sub(/.*y: /, "", $0); py=$0+0; next }
    in_arr && /^      z:/ && pz=="" { sub(/.*z: /, "", $0); pz=$0+0; next }
    in_arr && /  observations_count: / {
      sub(/.*observations_count: /, "", $0); obs=$0; next
    }
    in_arr && /  currently_visible: / {
      sub(/.*currently_visible: /, "", $0); vis=$0; next
    }
    END {
      flush_row();
      if (confirmed_person_count > 1) {
        printf "\n  WARNING: %d confirmed person landmarks visible. ",
               confirmed_person_count;
        printf "MVP demo expects ONE.\n";
        printf "    Run: ros2 topic pub --once /semantic_map/control ";
        printf "std_msgs/String \"data: '\''keep_best_class person'\''\"\n";
      }
    }
  '
fi

step "L4a — target_selector parameters"
echo "--- timeout ${ECHO_TIMEOUT}s ros2 param get /target_selector target_class ---"
timeout "${ECHO_TIMEOUT}" ros2 param get /target_selector target_class \
  || echo "(no /target_selector param available)"
echo "--- timeout ${ECHO_TIMEOUT}s ros2 param get /target_selector min_confidence ---"
timeout "${ECHO_TIMEOUT}" ros2 param get /target_selector min_confidence \
  || true
echo "--- timeout ${ECHO_TIMEOUT}s ros2 param get /target_selector require_confirmed_for_target ---"
timeout "${ECHO_TIMEOUT}" ros2 param get /target_selector \
  require_confirmed_for_target || true

step "L4b — current /target/selected snapshot"
if [ -n "$TARGET_CLASS" ]; then
  echo "Setting /target_selector target_class=${TARGET_CLASS} ..."
  timeout "${ECHO_TIMEOUT}" ros2 param set /target_selector \
    target_class "$TARGET_CLASS" || true
  sleep 1
fi
echo_topic_once /target/selected

step "L5 — temporal alignment + duplicate-cleanup helpers"
echo "  Day 8++ TF policy: depth_projector now uses detection.header.stamp"
echo "  for the camera->map transform (use_detection_timestamp_tf=true)."
echo "  If you see throttled warnings of:"
echo "      'TF tf_at_detection_time_unavailable' or"
echo "      'TF stale_detection_pose'"
echo "  it means perception latency exceeds the TF buffer / keyframe cache."
echo "  The cache is held for ${KEYFRAME_AGE:-2}s (keyframe_cache_max_age_sec)."
echo
echo "  Useful operator commands while this script runs:"
echo "    # one confirmed person at a time (MVP demo)"
echo "    ros2 topic pub --once /semantic_map/control std_msgs/String \\"
echo "      \"data: 'keep_best_class person'\""
echo
echo "    # nuke unanchored person ghosts"
echo "    ros2 topic pub --once /semantic_map/control std_msgs/String \\"
echo "      \"data: 'clear_unanchored person'\""
echo
echo "    # forcibly merge ALL person landmarks regardless of distance"
echo "    ros2 topic pub --once /semantic_map/control std_msgs/String \\"
echo "      \"data: 'merge_class person'\""
echo
echo "    # drop everything that never made it to confirmed"
echo "    ros2 topic pub --once /semantic_map/control std_msgs/String \\"
echo "      \"data: 'clear_candidates'\""

step "DONE"
echo "Reading guide:"
echo "  • Each L3c row of the form"
echo "      person_001  person  confirmed isl_+0025_-0010  18 true 1.42 0.85"
echo "    means: anchored to obstacle island, confirmed, ~1.4 m from base_link."
echo "    /semantic_map/markers will keep this marker even when Go2 looks away."
echo "  • '<raw>|candidate|-' means the entity is still un-confirmed and"
echo "    has no island anchor (no /map yet, or detection landed in free"
echo "    space). Should appear on /semantic_map/debug_markers, not"
echo "    /semantic_map/markers."
echo "  • Rows ending with SUSPICIOUS:obs=1 are confirmed person landmarks"
echo "    with only one observation — Day 8++ promotion gates SHOULD prevent"
echo "    this. If you still see them, double-check"
echo "    person_min_observations_to_confirm=2 on /semantic_memory_aggregator."
echo "  • Rows ending with SUSPICIOUS:no_island mean a confirmed person"
echo "    without an obstacle anchor; target_selector hides these as"
echo "    'missing_required_island_anchor' rejection (Task 5)."
echo "  • dist_m=? means the diagnose script could not look up map->base_link"
echo "    via tf2_echo. target_selector handles this case as NaN distance,"
echo "    NOT 0.00m, and surfaces 'distance_unknown_tf_failed' in its"
echo "    ranking_reasons."
echo "  • If L3c is empty, check L2 — the 3D projector may be the culprit."
echo "  • If L4b's entity_id is empty, the ranking_reasons line tells you"
echo "    exactly why: class_mismatch / low_conf / low_obs / unconfirmed /"
echo "    missing_required_island_anchor / distance_unknown_tf_failed."
