#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# check_table_semantic_health.sh — Day 9+ Phase C focused triage for the
# *table-only* semantic mapping pipeline.
#
# Why a table-specific script?
#   The general-purpose ``diagnose_semantic_perception.sh`` covers both
#   person and table, but tables have failure modes person doesn't:
#     * YOLOE bbox-confidence threshold filters ``desk`` out of
#       /detections while the segmentation head still publishes a mask
#       for it on /detections/masks. The pre-Day-9 depth_projector
#       indexed masks BY DETECTION INDEX so the unmatched mask sat
#       unused and /detections_3d never saw a table.
#     * Multiple aliases (table / desk / dining_table / workbench)
#       collapse onto a single canonical "table" entity at the
#       semantic_memory layer, so a healthy mask of class "desk" must
#       still surface as class_label="table" in /semantic_map/objects.
#
# This script answers, in order, the four "is the layer working?"
# questions the user asks when they see a table on RViz YOLOE overlay
# but no table marker on the semantic map:
#
#   L1 — Does YOLOE see the table at all?
#        /detections + /detections/masks (any of: table / desk /
#        dining table / workbench)
#   L2 — Did the depth projector promote it to 3D?
#        /detections_3d, /depth_projector/debug_stats
#   L3 — Did semantic_memory anchor + persist it?
#        /semantic_map/objects (class_label=table),
#        /semantic_map/anchor_debug_stats
#   L4 — Summary: which layer is broken?
#
# Every external command runs through ``timeout 10`` so the script
# never hangs even if a topic is silent.
#
# Usage:
#   bash scripts/check_table_semantic_health.sh
#
# Environment overrides:
#   ECHO_TIMEOUT  — seconds per ``ros2 topic echo --once``  (default 10)
#
# Run order assumption (same as diagnose_semantic_perception.sh):
#   T1: bash scripts/run_warehouse_ros2.sh
#   T2: bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py
#   T3: bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
#   T4: bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py
# -----------------------------------------------------------------------------

# DO NOT use `set -u`: sourcing /opt/ros/jazzy/setup.bash trips on unset
# vars on clean shells and the failure is shell-level (no per-cmd
# fallback). We rely on the explicit `|| echo "(...)"` guards below.
set +u

ECHO_TIMEOUT="${ECHO_TIMEOUT:-10}"

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

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[check_table_semantic_health] ERROR: 'ros2' not found." >&2
  echo "  source /opt/ros/jazzy/setup.bash and try again." >&2
  exit 2
fi

# Cache one --once snapshot of each topic so we can both *print* the
# raw message AND extract counters / class labels out of it without
# paying for a second timeout each. Failure → empty string, which the
# downstream parsing treats as "topic silent".
_capture_dir="$(mktemp -d -t check_table_semantic.XXXXXX)"
trap 'rm -rf "${_capture_dir}" 2>/dev/null || true' EXIT INT TERM

capture_topic() {
  # capture_topic <topic> <msgtype>     → cache file under ${_capture_dir}
  local topic="$1"
  local msgtype="$2"
  local outfile="${_capture_dir}/$(echo "$topic" | tr '/' '_').yaml"
  : >"${outfile}"
  timeout "${ECHO_TIMEOUT}" ros2 topic echo --once "${topic}" "${msgtype}" \
    >"${outfile}" 2>/dev/null || true
  echo "${outfile}"
}

# Long single-line std_msgs/String stats must not be YAML-truncated.
capture_topic_full_length() {
  local topic="$1"
  local msgtype="$2"
  local outfile="${_capture_dir}/$(echo "$topic" | tr '/' '_')_full.yaml"
  : >"${outfile}"
  timeout "${ECHO_TIMEOUT}" ros2 topic echo --once --full-length "${topic}" "${msgtype}" \
    >"${outfile}" 2>/dev/null || true
  echo "${outfile}"
}

get_capture() {
  local f="$1"
  if [ -s "$f" ]; then
    cat "$f"
  else
    echo "(no message captured within ${ECHO_TIMEOUT}s)"
  fi
}

# ---------------------------------------------------------------------
# L1 — YOLOE detection layer
# ---------------------------------------------------------------------
step "L1 — /detections (table-like entries)"
DET_FILE="$(capture_topic /detections vision_msgs/msg/Detection2DArray)"
echo "raw /detections snapshot:"
get_capture "${DET_FILE}"
echo
echo "table-like detections found in /detections:"
# Detection2DArray YAML embeds class_id under
#   detections[i].results[j].hypothesis.class_id
# We grep for the four allowed labels case-insensitively.
det_table_lines="$(
  grep -i -E "class_id:[[:space:]]*['\"]?(table|desk|dining[_ ]table|workbench)" \
    "${DET_FILE}" 2>/dev/null || true
)"
if [ -z "${det_table_lines}" ]; then
  echo "  (none)"
else
  echo "${det_table_lines}"
fi

step "L1 — /detections/masks (table-like entries)"
MASK_FILE="$(capture_topic /detections/masks go2_msgs/msg/InstanceMaskArray)"
echo "raw /detections/masks snapshot:"
get_capture "${MASK_FILE}"
echo
echo "table-like masks found in /detections/masks:"
# InstanceMaskArray YAML: each masks[i] has class_label + score on
# adjacent lines. We use a 3-line context window so the operator can
# eyeball the score directly.
mask_table_lines="$(
  grep -i -A1 -E "class_label:[[:space:]]*['\"]?(table|desk|dining[_ ]table|workbench)" \
    "${MASK_FILE}" 2>/dev/null || true
)"
if [ -z "${mask_table_lines}" ]; then
  echo "  (none)"
else
  echo "${mask_table_lines}"
fi

# ---------------------------------------------------------------------
# L2 — Depth projector layer
# ---------------------------------------------------------------------
step "L2 — /detections_3d (table-like entries)"
DET3D_FILE="$(capture_topic /detections_3d vision_msgs/msg/Detection3DArray)"
echo "raw /detections_3d snapshot:"
get_capture "${DET3D_FILE}"
echo
echo "table-like 3D detections found in /detections_3d:"
det3d_table_lines="$(
  grep -i -E "class_id:[[:space:]]*['\"]?(table|desk|dining[_ ]table|workbench)" \
    "${DET3D_FILE}" 2>/dev/null || true
)"
if [ -z "${det3d_table_lines}" ]; then
  echo "  (none)"
else
  echo "${det3d_table_lines}"
fi

step "L2 — /depth_projector/debug_stats (full, untruncated)"
PDBG_FILE="$(capture_topic_full_length /depth_projector/debug_stats std_msgs/msg/String)"
echo "raw /depth_projector/debug_stats snapshot (ros2 topic echo --full-length --once):"
get_capture "${PDBG_FILE}"
echo
echo "highlighted table-pipeline counters:"
# Day 9+ Phase C2 — print every counter the operator typically needs
# to bisect "is the bug in YOLOE, depth_projector detection-driven
# path, or depth_projector mask-only path?". One key per line so the
# script reads cleanly even when stats text is wider than the
# terminal.
for key in \
    table_detection_seen \
    table_mask_seen \
    table_detection_driven_attempted \
    table_detection_driven_published \
    table_detection_driven_failed_no_mask \
    table_detection_driven_failed_no_depth \
    table_detection_driven_failed_bad_depth \
    table_detection_driven_failed_tf \
    table_mask_only_attempted \
    table_mask_only_published \
    table_mask_only_skipped_used_mask \
    table_mask_only_failed_low_score \
    table_mask_only_failed_no_depth \
    table_mask_only_failed_bad_depth \
    table_mask_only_failed_tf \
    table_3d_published \
    force_table_mask_only_projection
do
  hit="$(grep -oE "${key}=[^[:space:]]+" "${PDBG_FILE}" 2>/dev/null | tail -n1)"
  if [ -n "${hit}" ]; then
    echo "  ${hit}"
  else
    echo "  ${key}=? (counter missing — projector predates Phase C2 build?)"
  fi
done

# ---------------------------------------------------------------------
# L3 — Semantic memory layer
# ---------------------------------------------------------------------
step "L3 — /semantic_map/objects (class_label=table only)"
SEM_FILE="$(capture_topic /semantic_map/objects go2_msgs/msg/SemanticEntityArray)"
echo "raw /semantic_map/objects snapshot:"
get_capture "${SEM_FILE}"
echo
echo "class_label=table entries found:"
# SemanticEntity has class_label as a top-level string field.
sem_table_lines="$(
  grep -B1 -A4 "class_label: table" "${SEM_FILE}" 2>/dev/null || true
)"
if [ -z "${sem_table_lines}" ]; then
  echo "  (none)"
else
  echo "${sem_table_lines}"
fi

step "L3 — table landmark summary (candidate vs confirmed, anchor type)"
if [ -s "${SEM_FILE}" ]; then
  python3 - <<'PY' "${SEM_FILE}"
import sys

path = sys.argv[1]
text = open(path, encoding="utf-8", errors="replace").read()
lines = text.splitlines()

def val_after(key: str, line: str) -> str:
    if not line.strip().startswith(key):
        return ""
    return line.split(":", 1)[1].strip().strip("'\"")


def anchor_type(display_name: str) -> str:
    parts = display_name.split("|")
    tail = parts[-1].strip() if parts else ""
    if tail.startswith("pc_"):
        return "pc_"
    if tail.startswith("isl_"):
        return "isl_"
    if tail in ("-", ""):
        return "none"
    return "other"

tables: list[dict] = []
i = 0
while i < len(lines):
    ln = lines[i]
    if "class_label:" in ln and val_after("class_label", ln) == "table":
        rec: dict = {"class_label": "table"}
        j = i
        # scan backward a few lines for entity_id
        for k in range(max(0, i - 12), i + 1):
            if "entity_id:" in lines[k]:
                rec["entity_id"] = val_after("entity_id", lines[k])
        # scan forward for fields
        for k in range(i, min(len(lines), i + 25)):
            l2 = lines[k]
            if l2.strip().startswith("display_name:"):
                rec["display_name"] = val_after("display_name", l2)
            if l2.strip().startswith("confidence:"):
                try:
                    rec["confidence"] = float(l2.split(":", 1)[1].strip())
                except ValueError:
                    rec["confidence"] = l2
            if l2.strip().startswith("observations_count:"):
                try:
                    rec["observations_count"] = int(l2.split(":", 1)[1].strip())
                except ValueError:
                    rec["observations_count"] = l2
        tables.append(rec)
    i += 1

if not tables:
    print("  (no class_label=table in /semantic_map/objects)")
    sys.exit(0)

has_cand = any(
    "|candidate|" in r.get("display_name", "") for r in tables
)
has_conf = any(
    "|confirmed|" in r.get("display_name", "") for r in tables
)
print(f"  table candidate present : {'yes' if has_cand else 'no'}")
print(f"  table confirmed present : {'yes' if has_conf else 'no'}")
for idx, r in enumerate(tables, 1):
    dn = r.get("display_name", "")
    status = "unknown"
    if "|confirmed|" in dn:
        status = "confirmed"
    elif "|candidate|" in dn:
        status = "candidate"
    elif "|invalid|" in dn:
        status = "invalid"
    atype = anchor_type(dn)
    print(f"  --- table entity #{idx} ---")
    print(f"    entity_id          : {r.get('entity_id', '?')}")
    print(f"    status (from name) : {status}")
    print(f"    anchor type        : {atype}")
    print(f"    observations_count : {r.get('observations_count', '?')}")
    print(f"    confidence         : {r.get('confidence', '?')}")
    print(f"    display_name       : {dn}")
    if status == "candidate" and "|candidate|" in dn:
        tail = dn.split("|")[-1] if dn else ""
        if tail in ("-", ""):
            dbg = " (no anchor id in display_name)"
        else:
            dbg = ""
        print(f"    debug              : candidate — check RViz candidate_not_confirmed line if stale{dbg}")
PY
else
  echo "  (skipped — empty /semantic_map/objects capture)"
fi

step "L3 — /semantic_map/anchor_debug_stats"
ASTAT_FILE="$(capture_topic /semantic_map/anchor_debug_stats std_msgs/msg/String)"
echo "raw /semantic_map/anchor_debug_stats snapshot:"
get_capture "${ASTAT_FILE}"

# ---------------------------------------------------------------------
# Summary section — verdict per layer + likely break point.
# ---------------------------------------------------------------------
step "Summary — table semantic health verdict"

# L1 verdicts.
if [ -n "${det_table_lines}" ]; then
  L1_DET="yes"
else
  L1_DET="no"
fi
if [ -n "${mask_table_lines}" ]; then
  L1_MASK="yes"
else
  L1_MASK="no"
fi

# L2 verdict.
if [ -n "${det3d_table_lines}" ]; then
  L2_3D="yes"
else
  L2_3D="no"
fi

# L3 verdict.
if [ -n "${sem_table_lines}" ]; then
  L3_SEM="yes"
else
  L3_SEM="no"
fi

# Counter snapshot from /depth_projector/debug_stats. Optional — if
# the stats topic is silent we just skip these lines.
extract_kv() {
  local key="$1"
  local file="$2"
  grep -oE "${key}=[0-9]+" "$file" 2>/dev/null | tail -n1 || echo ""
}
extract_kv_num() {
  # Returns the numeric value of <key>=<n> in <file>, or "" on miss.
  local key="$1"
  local file="$2"
  grep -oE "${key}=[0-9]+" "$file" 2>/dev/null | tail -n1 \
    | awk -F= '{print $2}' || echo ""
}
TBL_MASKS_RX="$(extract_kv table_masks_received "${PDBG_FILE}")"
TBL_USED="$(extract_kv table_mask_only_used "${PDBG_FILE}")"
TBL_LOWSCORE="$(extract_kv table_mask_low_score_rejected "${PDBG_FILE}")"
TBL_DEPTH_OK="$(extract_kv table_mask_depth_valid "${PDBG_FILE}")"
TBL_DEPTH_BAD="$(extract_kv table_mask_depth_invalid "${PDBG_FILE}")"
TBL_PUB="$(extract_kv table_3d_published "${PDBG_FILE}")"
# Phase C2 numeric extracts — used in the diagnosis block below.
N_DET_SEEN="$(extract_kv_num table_detection_seen "${PDBG_FILE}")"
N_MASK_SEEN="$(extract_kv_num table_mask_seen "${PDBG_FILE}")"
N_DET_DRV_ATT="$(extract_kv_num table_detection_driven_attempted "${PDBG_FILE}")"
N_DET_DRV_PUB="$(extract_kv_num table_detection_driven_published "${PDBG_FILE}")"
N_DET_DRV_NO_MASK="$(extract_kv_num table_detection_driven_failed_no_mask "${PDBG_FILE}")"
N_DET_DRV_NO_DEPTH="$(extract_kv_num table_detection_driven_failed_no_depth "${PDBG_FILE}")"
N_DET_DRV_TF="$(extract_kv_num table_detection_driven_failed_tf "${PDBG_FILE}")"
N_MASK_ONLY_ATT="$(extract_kv_num table_mask_only_attempted "${PDBG_FILE}")"
N_MASK_ONLY_PUB="$(extract_kv_num table_mask_only_published "${PDBG_FILE}")"
N_MASK_ONLY_USED_SKIP="$(extract_kv_num table_mask_only_skipped_used_mask "${PDBG_FILE}")"
N_MASK_ONLY_LOWSCORE="$(extract_kv_num table_mask_only_failed_low_score "${PDBG_FILE}")"
N_MASK_ONLY_NO_DEPTH="$(extract_kv_num table_mask_only_failed_no_depth "${PDBG_FILE}")"
N_MASK_ONLY_BAD_DEPTH="$(extract_kv_num table_mask_only_failed_bad_depth "${PDBG_FILE}")"
N_MASK_ONLY_TF="$(extract_kv_num table_mask_only_failed_tf "${PDBG_FILE}")"

echo "  table seen in RGB (/detections)       : ${L1_DET}"
echo "  table seen in masks (/detections/masks): ${L1_MASK}"
echo "  table projected to 3D (/detections_3d): ${L2_3D}"
echo "  table confirmed in semantic memory   : ${L3_SEM}"
if [ -n "${TBL_MASKS_RX}${TBL_PUB}" ]; then
  echo "  legacy counters: ${TBL_MASKS_RX:-table_masks_received=?} ${TBL_USED:-table_mask_only_used=?} ${TBL_LOWSCORE:-table_mask_low_score_rejected=?} ${TBL_DEPTH_OK:-table_mask_depth_valid=?} ${TBL_DEPTH_BAD:-table_mask_depth_invalid=?} ${TBL_PUB:-table_3d_published=?}"
fi

# Likely-broken-layer diagnosis. Phase C2 — when /detections_3d is
# silent we now decide between "detection-driven failed" and
# "mask-only failed" using the new per-stage counters.
echo
if [ "${L1_DET}" = "no" ] && [ "${L1_MASK}" = "no" ]; then
  echo "  >> Likely break point: L1 (YOLOE)."
  echo "     YOLOE does NOT see any table-like class. Common causes:"
  echo "     * table out of camera FOV — drive Go2 closer / rotate to face it,"
  echo "     * YOLOE prompts/whitelist exclude table — check yoloe_detector"
  echo "       parameters: target_class_whitelist must include 'table' / 'desk',"
  echo "     * table USD failed to load — check /World/Table in Isaac Sim."
elif [ "${L1_MASK}" = "yes" ] && [ "${L1_DET}" = "no" ] && [ "${L2_3D}" = "no" ]; then
  echo "  >> Likely break point: L2 (depth_projector mask-only path)."
  echo "     YOLOE has a mask but no bbox; the projector either"
  echo "     filtered the mask under the table_like_min_score gate"
  echo "     or could not find a depth-valid mask region."
  echo "     Inspect the highlighted counters above:"
  if [ -n "${N_MASK_ONLY_LOWSCORE}" ] && [ "${N_MASK_ONLY_LOWSCORE}" != "0" ]; then
    echo "       * table_mask_only_failed_low_score=${N_MASK_ONLY_LOWSCORE} —"
    echo "         bump score gate down:"
    echo "           ros2 param set /depth_projector table_like_min_score 0.30"
  fi
  if [ -n "${N_MASK_ONLY_NO_DEPTH}" ] && [ "${N_MASK_ONLY_NO_DEPTH}" != "0" ]; then
    echo "       * table_mask_only_failed_no_depth=${N_MASK_ONLY_NO_DEPTH} —"
    echo "         not enough valid depth pixels under the mask. Bump"
    echo "         table_mask_min_valid_depth_pixels lower or move Go2"
    echo "         closer to the table."
  fi
  if [ -n "${N_MASK_ONLY_BAD_DEPTH}" ] && [ "${N_MASK_ONLY_BAD_DEPTH}" != "0" ]; then
    echo "       * table_mask_only_failed_bad_depth=${N_MASK_ONLY_BAD_DEPTH} —"
    echo "         all percentiles out of [min_depth,max_depth]."
    echo "         Loosen the depth bounds or extend the retry list:"
    echo "           ros2 param set /depth_projector table_mask_depth_retry_percentiles '20,35,50,65,80'"
  fi
  if [ -n "${N_MASK_ONLY_TF}" ] && [ "${N_MASK_ONLY_TF}" != "0" ]; then
    echo "       * table_mask_only_failed_tf=${N_MASK_ONLY_TF} —"
    echo "         TF buffer can't serve camera_optical_frame->map at"
    echo "         the detection stamp. Check that slam_toolbox /"
    echo "         tf_static publishers are alive."
  fi
elif [ "${L1_DET}" = "yes" ] && [ "${L2_3D}" = "no" ]; then
  echo "  >> Likely break point: L2 (detection-driven path of depth_projector)."
  echo "     /detections has a desk bbox but /detections_3d is empty."
  echo "     This is the bug pattern from 2026-05-10."
  if [ -n "${N_DET_DRV_NO_MASK}" ] && [ "${N_DET_DRV_NO_MASK}" != "0" ]; then
    echo "       * table_detection_driven_failed_no_mask=${N_DET_DRV_NO_MASK} —"
    echo "         mask either absent or empty. Mask-only fallback should"
    echo "         then catch it; check table_mask_only_attempted below."
  fi
  if [ -n "${N_DET_DRV_NO_DEPTH}" ] && [ "${N_DET_DRV_NO_DEPTH}" != "0" ]; then
    echo "       * table_detection_driven_failed_no_depth=${N_DET_DRV_NO_DEPTH} —"
    echo "         neither the mask path nor bbox-fallback found a"
    echo "         depth-valid sample under the desk bbox. Mask-only"
    echo "         fallback should still try; if that ALSO fails, drop"
    echo "         table_mask_min_valid_depth_pixels (default 20) or"
    echo "         move closer."
  fi
  if [ -n "${N_DET_DRV_TF}" ] && [ "${N_DET_DRV_TF}" != "0" ]; then
    echo "       * table_detection_driven_failed_tf=${N_DET_DRV_TF} —"
    echo "         tf2 lookup at the detection stamp failed. Same cause"
    echo "         as table_mask_only_failed_tf."
  fi
  if [ -n "${N_MASK_ONLY_USED_SKIP}" ] && [ "${N_MASK_ONLY_USED_SKIP}" != "0" ]; then
    echo "       * table_mask_only_skipped_used_mask=${N_MASK_ONLY_USED_SKIP} —"
    echo "         WARNING: mask-only path was blocked by the"
    echo "         used_mask_indices guard even though detection-driven"
    echo "         failed. Either upgrade to the Phase C2 build (this"
    echo "         counter should be 0 unless the detection-driven path"
    echo "         actually published) or set"
    echo "           ros2 param set /depth_projector force_table_mask_only_projection True"
  fi
  if [ -n "${N_MASK_ONLY_ATT}" ] && [ "${N_MASK_ONLY_ATT}" != "0" ] \
        && [ "${N_MASK_ONLY_PUB}" = "0" ]; then
    echo "     mask-only fallback ran ${N_MASK_ONLY_ATT}× but published 0×."
    echo "     Check the table_mask_only_failed_* row above for the"
    echo "     specific stage."
  fi
elif [ "${L2_3D}" = "yes" ] && [ "${L3_SEM}" = "no" ]; then
  echo "  >> Likely break point: L3 (semantic_memory anchoring)."
  echo "     /detections_3d publishes table but /semantic_map/objects"
  echo "     does not list class_label=table. Inspect"
  echo "     /semantic_map/anchor_debug_stats above for"
  echo "     pointcloud_anchor_failed_<reason> / island_anchor_failed_<reason>"
  echo "     spikes. The aggregator may be marking the table candidate"
  echo "     INVALID because it cannot find a LiDAR cluster or"
  echo "     occupancy island under the projected centroid."
elif [ "${L3_SEM}" = "yes" ]; then
  echo "  >> Healthy. Table semantic landmark is live."
else
  echo "  >> Mixed signals — re-run after a few seconds; the perception"
  echo "     stack may be mid-warmup."
fi

# Always print a one-line "force-on hint" when /detections_3d is
# silent. The operator can use it to confirm the bug really is in the
# detection-driven path (Task 6 debug knob).
if [ "${L2_3D}" = "no" ] && [ "${L1_MASK}" = "yes" ]; then
  echo
  echo "  Tip: to confirm the mask-only path itself is healthy, force it:"
  echo "    ros2 param set /depth_projector force_table_mask_only_projection True"
  echo "    bash scripts/check_table_semantic_health.sh"
  echo "  Then revert with:"
  echo "    ros2 param set /depth_projector force_table_mask_only_projection False"
fi

echo
echo "Done."
