#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Day 6.5 acceptance gate — perception persistence + projection accuracy.
#
# Day 7's algorithm layer + Nav2 integration was verified end-to-end by
# `bash scripts/check_day7.sh`. Day 6.5 closes the two perception-layer
# blockers that prevent the "Go2 actually reaches the chair" finale:
#
#   #8 — depth_projector mis-projects entities ~1.7× too far
#         (mask-edge depth bleed)
#   #9 — semantic_memory entity registry empties mid-traverse
#         (NMS / decay / visibility timeout too aggressive)
#
# Per docs/day7_target_navigation_status.md "Day 6.5 acceptance gate",
# four checks must pass before opening Day 8 (frontier exploration):
#
#   1. STATIC accuracy: Go2 stationary at spawn, facing the desk →
#      desk_xxx.pose_map error vs ground-truth sim location < 0.30 m,
#      confidence ≥ 0.5 sustained for 30 s.
#   2. DYNAMIC stability: Go2 walks 5 m parallel to the desk (no
#      turning) → marker drift in odom < 0.30 m wander.
#   3. TRACK ID persistence: Go2 turns away from desk for 5 s, turns
#      back → same entity_id re-acquires (no chair_002 / desk_003).
#   4. END-TO-END APPROACH: send target_class:=desk → Go2 reaches
#      the approach pose → NavigateToPose action returns SUCCEEDED
#      AND Go2 stops within 0.30 m of the planned approach distance.
#
# This script runs check #1 (static accuracy) automatically and walks
# the operator through #2-#4 with explicit prompts — unless you pass
# ``--auto`` (no ENTER prompts; checks 2–4 are WARN-skipped because they
# require a human in the loop). Add ``--auto-dynamic`` to run check #2
# automatically: publishes lateral /cmd_vel, samples desk/table pose
# before vs after, PASS if median XY drift < 0.30 m (same entity_id).
#
# Usage:
#   bash scripts/check_day6_5.sh
#   bash scripts/check_day6_5.sh --auto
#   bash scripts/check_day6_5.sh --auto --auto-dynamic
#   bash scripts/check_day6_5.sh --auto --static-classes desk,table \\
#       --static-seconds 30
#   bash scripts/check_day6_5.sh --auto --gt-x 1.5 --gt-y 1.0
#       (override GT if your /odom axes stay world-aligned at spawn)
#
# Optional env (same effect as flags where noted):
#   CHECK_DAY65_STATIC_CLASSES   comma list, default desk,table
#   CHECK_DAY65_DYN_LINEAR_Y     strafe linear.y (default 0.5)
#   CHECK_DAY65_DYN_STRAFE_SEC   strafe duration s (default 10)
#   CHECK_DAY65_DYN_SAMPLE_SEC   seconds to median pose each phase (default 2)
#   CHECK_DAY65_DYN_SETTLE_POST  wait after strafe before phase-2 sample (default 1.5)
#
# Run AFTER:
#   bash scripts/run_warehouse_ros2.sh                       (sim with LiDAR)
#   ros2 launch go2_bringup_sim chair_perception.launch.py   (TFs + /scan)
#   ros2 launch go2_bringup_sim nav2.launch.py               (lifecycle 'active')
#   ros2 launch go2_bringup_sim day7.launch.py target_class:=desk \\
#                                              target_frame:=odom
# -----------------------------------------------------------------------------
set -uo pipefail

_AUTO=false
_AUTO_DYNAMIC=false
_STATIC_SEC=30
_STATIC_CLASSES="${CHECK_DAY65_STATIC_CLASSES:-desk,table}"
_DYN_LINEAR_Y="${CHECK_DAY65_DYN_LINEAR_Y:-0.5}"
_DYN_STRAFE_SEC="${CHECK_DAY65_DYN_STRAFE_SEC:-10}"
_DYN_SAMPLE_SEC="${CHECK_DAY65_DYN_SAMPLE_SEC:-2.0}"
_DYN_SETTLE_POST="${CHECK_DAY65_DYN_SETTLE_POST:-1.5}"

while [ "${#}" -gt 0 ]; do
	case "$1" in
		--auto)
			_AUTO=true
			shift
			;;
		--auto-dynamic)
			_AUTO_DYNAMIC=true
			shift
			;;
		--static-seconds)
			_STATIC_SEC="${2:?--static-seconds requires a number}"
			shift 2
			;;
		--static-classes)
			_STATIC_CLASSES="${2:?--static-classes requires a comma-separated list}"
			shift 2
			;;
		--gt-x)
			_GT_OVERRIDE_X="${2:?--gt-x requires a number}"
			shift 2
			;;
		--gt-y)
			_GT_OVERRIDE_Y="${2:?--gt-y requires a number}"
			shift 2
			;;
		-h|--help)
			# Skip shebang + shellcheck directive lines so we do not print a broken first line.
			tail -n +3 "${BASH_SOURCE[0]:-$0}" | grep '^#' | head -n 72 | sed 's/^# \{0,1\}//'
			exit 0
			;;
		*)
			echo "[check_day6_5] ERROR: unknown argument: $1 (try --help)" >&2
			exit 2
			;;
	esac
done

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[check_day6_5] ERROR: 'ros2' not found." >&2
	exit 2
fi

if [ -t 1 ]; then
	C_RED=$'\e[31m'; C_GRN=$'\e[32m'; C_YLW=$'\e[33m'; C_BLD=$'\e[1m'; C_END=$'\e[0m'
else
	C_RED=""; C_GRN=""; C_YLW=""; C_BLD=""; C_END=""
fi

_FAIL=0; _PASS=0; _WARN=0
_pass()    { echo "  ${C_GRN}PASS${C_END} $*"; _PASS=$((_PASS + 1)); }
_fail()    { echo "  ${C_RED}FAIL${C_END} $*"; _FAIL=$((_FAIL + 1)); }
_warn()    { echo "  ${C_YLW}WARN${C_END} $*"; _WARN=$((_WARN + 1)); }
_section() { echo; echo "${C_BLD}== $* ==${C_END}"; }
_prompt()  { printf "${C_BLD}>>> %s${C_END} " "$*"; }
# Bash has no ${var!r}; keep operator answers printable without histexpand issues.
_repr() { printf '%s' "${1:-}" | tr -d '\r\n' | cut -c1-120; }

# Ground-truth **table centre XY** for static gate #1 (must match sim).
#
# Source of truth: sim/warehouse_scene.py — TABLE_XYZ=(1.5,1.0),
# GO2_SPAWN_XYZ=(-4,-4), GO2_SPAWN_YAW_DEG=45. With REP-105 odom at spawn
# (identity base in odom, odom axes aligned with base), the table point in
# odom is R(-yaw) * (T - spawn_xy):
#   x = (5.5+5.0)/√2,  y = (-5.5+5.0)/√2
# Override with  --gt-x / --gt-y  if your stack publishes odom world-aligned
# at t=0 (then expect ~(1.5, 1.0) in odom ≈ world).
_GT_X="${_GT_OVERRIDE_X:-7.424621}"
# Quoted default so bash does not parse ${var:--0.35} as ${var:-} minus 0.35.
_GT_Y="${_GT_OVERRIDE_Y:-"-0.353553"}"
_GT_TOL="0.30"

# ----------------------------------------------------------------------------
# Pre-flight: nodes alive
# ----------------------------------------------------------------------------
_section "0. Day 6.5 pre-flight (sim + nav2 + day7 stack must be running)"
_NODES_RAW="$(timeout 4 ros2 node list 2>/dev/null || true)"
for n in /yoloe_detector /depth_projector /semantic_memory_aggregator \
         /target_selector /approach_goal_planner; do
	if echo "${_NODES_RAW}" | grep -Fxq "$n"; then
		_pass "${n} up"
	else
		_fail "${n} NOT in node list — start day7.launch.py first"
	fi
done
if [ "${_FAIL}" -gt 0 ]; then
	echo
	echo "  ${C_RED}Pre-flight failed.${C_END} Bring up sim + chair_perception + nav2 +"
	echo "  day7.launch.py before re-running this script."
	exit 1
fi

echo
echo "  ${C_BLD}Day 6.5 gate #1 提示:${C_END} 脚本里的 GT 桌子坐标在 **odom** 系 (~${_GT_X}, ${_GT_Y}) m。"
echo "  请用  target_frame:=odom  启动 day7（否则与 map 下的 pose 比会误判）。"
echo "  建议  target_class:=desk ，并让机器人**朝向仓库桌子**以便语义图里有 desk/table。"
echo "  仿真里 depth_projector 请保持  tf_fallback_latest_on_time_error:=true （默认），否则 TF 外推会丢检测。"
echo

# ----------------------------------------------------------------------------
# Check 1 — STATIC accuracy + persistence
# ----------------------------------------------------------------------------
_section "1. STATIC accuracy: Go2 stationary, desk pose vs ground-truth"
if ${_AUTO}; then
	echo "  (--auto) Skipping positioning prompt; sampling ${_STATIC_SEC}s now."
else
	_prompt "Position Go2 facing the table at the spawn pose. Press ENTER when ready (or Ctrl-C to abort):"
	read -r _
fi

# Sample /semantic_map/objects over STATIC_SEC seconds, keeping entities
# whose class_label matches STATIC_CLASSES (default desk,table): exact
# match OR substring (e.g. token ``table`` matches ``dining table``). Compute
# (a) mean position error vs ground truth, (b) confidence min over the
# window, (c) entity_id stability (number of distinct ids seen).
_TO=$((_STATIC_SEC + 8))
_static_summary="$(timeout "${_TO}" python3 - "${_GT_X}" "${_GT_Y}" "${_GT_TOL}" \
    "${_STATIC_SEC}" "${_STATIC_CLASSES}" <<'PYTHONEOF'
import math, sys, time
from collections import Counter

GT_X, GT_Y, TOL = (float(a) for a in sys.argv[1:4])
sample_sec = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0
class_csv = sys.argv[5] if len(sys.argv) > 5 else "desk,table"
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from go2_msgs.msg import SemanticEntityArray
except Exception as e:
    print(f"ERROR_IMPORT: {e}")
    sys.exit(0)

TARGET_CLASSES = {
    c.strip().lower()
    for c in class_csv.split(",")
    if c.strip()
}


def _label_matches_targets(lab: str, targets: set[str]) -> bool:
    """Exact match or substring (e.g. 'dining table' matches token 'table')."""
    if lab in targets:
        return True
    return any(t in lab for t in targets if t)


def _is_desk_table_class(lab: str) -> bool:
    if lab in ("desk", "table"):
        return True
    return "desk" in lab or "table" in lab


samples = []  # (entity_id, pose_xy, conf, t_recv, class_label)
seen_ids: set[str] = set()
all_labels_seen: set[str] = set()
state = {"msgs": 0, "max_entities": 0}

try:
    rclpy.init()
    node = Node("_check_day65_static")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)

    def cb(msg):
        state["msgs"] += 1
        n_ent = len(msg.entities)
        state["max_entities"] = max(state["max_entities"], n_ent)
        for e in msg.entities:
            lab = e.class_label.strip().lower()
            all_labels_seen.add(lab)
            if _label_matches_targets(lab, TARGET_CLASSES):
                samples.append(
                    (
                        e.entity_id,
                        (e.pose_map.position.x, e.pose_map.position.y),
                        float(e.confidence),
                        time.time(),
                        lab,
                    )
                )
                seen_ids.add(e.entity_id)

    node.create_subscription(
        SemanticEntityArray, "/semantic_map/objects", cb, qos
    )
    deadline = time.time() + sample_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"ERROR_EXC: {e}")
    sys.exit(0)

if not samples:
    labels = ",".join(sorted(all_labels_seen)) if all_labels_seen else ""
    print(
        f"ERROR_NO_MATCH n_msgs={state['msgs']} "
        f"max_entities={state['max_entities']} labels_seen={labels!r}"
    )
    sys.exit(0)

# Gate spatial error vs sim GT applies only to desk/table-like tracks.
samples_dt = [s for s in samples if _is_desk_table_class(s[4])]

if samples_dt:
    id_counter = Counter(s[0] for s in samples_dt)
    primary_id, primary_count = id_counter.most_common(1)[0]
    primary = [s for s in samples_dt if s[0] == primary_id]
    spatial_valid = 1
else:
    id_counter = Counter(s[0] for s in samples)
    primary_id, primary_count = id_counter.most_common(1)[0]
    primary = [s for s in samples if s[0] == primary_id]
    spatial_valid = 0

confs = [p[2] for p in primary]
min_conf = min(confs)
mean_conf = sum(confs) / len(confs)

if spatial_valid:
    errs = [math.hypot(GT_X - p[1][0], GT_Y - p[1][1]) for p in primary]
    mean_err = sum(errs) / len(errs)
    max_err = max(errs)
else:
    mean_err = float("nan")
    max_err = float("nan")

primary_class = primary[0][4] if primary else ""
print(
    f"primary_id={primary_id!r} class={primary_class!r} "
    f"spatial_valid={spatial_valid} "
    f"count={primary_count}/{len(samples)} "
    f"distinct_ids={len(seen_ids)} mean_err={mean_err}m "
    f"max_err={max_err}m min_conf={min_conf:.3f} "
    f"mean_conf={mean_conf:.3f}"
)
PYTHONEOF
)"

if [ -z "${_static_summary}" ] || echo "${_static_summary}" | grep -qE '^ERROR_'; then
	_fail "STATIC accuracy: helper crashed or no matching-class entity"
	echo "    ${_static_summary:-<empty>}"
	if echo "${_static_summary}" | grep -qF 'ERROR_NO_MATCH'; then
		_max_ent=$(echo "${_static_summary}" | sed -n 's/.*max_entities=\([0-9]*\).*/\1/p')
		_nmsg=$(echo "${_static_summary}" | sed -n 's/.*n_msgs=\([0-9]*\).*/\1/p')
		if [ "${_nmsg:-0}" = "0" ]; then
			echo "    Diagnosis: n_msgs=0 — no /semantic_map/objects callbacks (wrong DDS"
			echo "      namespace, sim not publishing, or QoS mismatch). Try:"
			echo "      ros2 topic info -v /semantic_map/objects"
		elif [ "${_max_ent:-0}" = "0" ]; then
			echo "    Diagnosis: topic is alive (n_msgs=${_nmsg}) but max_entities=0 — every"
			echo "      SemanticEntityArray has len(entities)==0 (pipeline before memory is dry)."
			echo "    Check upstream (in order):"
			echo "      ros2 topic hz /camera/color/image_raw"
			echo "      ros2 topic hz /detections"
			echo "      ros2 topic hz /detections_3d"
			echo "      ros2 param get /semantic_memory_aggregator min_detection_confidence"
		else
			echo "    Hint: no class_label matched --static-classes (${_STATIC_CLASSES})."
			echo "    Widen tokens (substring match), e.g.:"
			echo "      bash scripts/check_day6_5.sh --auto --static-classes desk,table,chair"
		fi
		echo "    Debug: ros2 topic echo /semantic_map/objects --once"
	fi
else
	echo "  ${_static_summary}"
	_mean_err=$(echo "${_static_summary}" | sed -n 's/.*mean_err=\([^m]*\)m.*/\1/p')
	_max_err=$(echo "${_static_summary}" | sed -n 's/.*max_err=\([^m]*\)m.*/\1/p')
	_min_conf=$(echo "${_static_summary}" | sed -n 's/.*min_conf=\([0-9.]*\).*/\1/p')
	_distinct_ids=$(echo "${_static_summary}" | sed -n 's/.*distinct_ids=\([0-9]*\).*/\1/p')
	_spatial_valid=$(echo "${_static_summary}" | sed -n 's/.*spatial_valid=\([01]\).*/\1/p')
	if [ "${_spatial_valid:-0}" != "1" ]; then
		_fail "STATIC gate: no desk/table track in the ${_STATIC_SEC}s window " \
		      "(need /semantic_map/objects with class desk or table vs sim GT). " \
		      "Face the warehouse table or add desk/table to YOLOE classes."
	else
		if awk -v e="${_mean_err:-99}" -v t="${_GT_TOL}" 'BEGIN { exit (e+0 < t+0) ? 0 : 1 }'; then
			_pass "STATIC mean position error ${_mean_err}m < ${_GT_TOL}m (desk/table vs GT)"
		else
			_fail "STATIC mean position error ${_mean_err}m >= ${_GT_TOL}m. " \
			      "depth_projector still mis-projecting; verify use_masks:=true " \
			      "and /detections/masks; mask-less tuning: depth_percentile / bbox_shrink."
		fi
	fi
	if awk -v c="${_min_conf:-0}" 'BEGIN { exit (c+0 >= 0.5) ? 0 : 1 }'; then
		_pass "STATIC min confidence ${_min_conf} >= 0.5 over 30 s"
	else
		_warn "STATIC min confidence ${_min_conf} dipped below 0.5 — entity " \
		      "is wobbling. Lower confidence_decay_rate or raise " \
		      "confidence_step_up."
	fi
	if [ "${_distinct_ids:-99}" = "1" ]; then
		_pass "STATIC primary track stable: only 1 entity_id seen"
	else
		_warn "STATIC ${_distinct_ids} distinct entity_ids over 30 s — " \
		      "projection jitter exceeds nms_radius_m. Raise nms_radius_m " \
		      "or fix mask-edge bleed."
	fi
fi

# ----------------------------------------------------------------------------
# Check 2 — DYNAMIC stability (operator-driven or --auto-dynamic)
# ----------------------------------------------------------------------------
_section "2. DYNAMIC stability: walk Go2 5 m parallel to desk, marker drift"
if ${_AUTO_DYNAMIC}; then
	echo "  ${C_BLD}--auto-dynamic:${C_END} publishing lateral /cmd_vel for ~${_DYN_STRAFE_SEC}s"
	echo "    (linear.y=${_DYN_LINEAR_Y}), then comparing desk/table median pose."
	echo "    Cancel active Nav2 goals; ensure open floor. Requires same stack as gate #1"
	echo "    (target_frame pose on /semantic_map/objects, typically odom)."
	echo
	# Float *_DYN_* breaks bash $((…)); keep a generous wall timeout.
	_dyn_to=90
	_dyn_py="${_SCRIPT_DIR}/_check_day65_dynamic.py"
	if [ ! -r "${_dyn_py}" ]; then
		_fail "DYNAMIC auto: missing ${_dyn_py}"
	else
		_dyn_summary="$(timeout "${_dyn_to}" python3 "${_dyn_py}" "${_STATIC_CLASSES}" \
		    "${_DYN_LINEAR_Y}" "${_DYN_STRAFE_SEC}" "${_DYN_SAMPLE_SEC}" \
		    "${_DYN_SETTLE_POST}" "${_GT_TOL}" 2>&1)"
		_dyn_ec=$?
		echo "  ${_dyn_summary:-}"
		case "${_dyn_ec}" in
			0)
				if echo "${_dyn_summary}" | grep -q "pass=1"; then
					_pass "DYNAMIC drift < ${_GT_TOL}m (auto: cmd_vel strafe + /semantic_map/objects)"
				else
					_warn "DYNAMIC helper exited 0 but missing pass=1 line — treating as inconclusive."
				fi
				;;
			1)
				if echo "${_dyn_summary}" | grep -q "pass=0"; then
					_fail "DYNAMIC drift >= ${_GT_TOL}m (auto-measured) — ${_dyn_summary}"
				else
					_fail "DYNAMIC auto-check failed (exit 1): ${_dyn_summary}"
				fi
				;;
			*)
				if echo "${_dyn_summary}" | grep -q "ERROR_"; then
					_fail "DYNAMIC auto-check: ${_dyn_summary}"
				else
					_fail "DYNAMIC auto-check crashed (exit ${_dyn_ec}): ${_dyn_summary:-no output}"
				fi
				;;
		esac
	fi
elif ${_AUTO}; then
	_warn "DYNAMIC check SKIPPED (--auto without --auto-dynamic). Re-run with --auto-dynamic or without --auto."
else
	echo "  This check needs you to drive Go2 manually:"
	echo "    1. Open RViz (Image (YOLOE detections) + Semantic memory (Day 6))."
	echo "    2. Note the desk cylinder's odom position (RViz: hover over it)."
	echo "    3. Drive Go2 sideways ~5 m WITHOUT TURNING the body (use:"
	echo "         ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist \\"
	echo "             '{linear: {y: 0.5}}'  for ~10 s,"
	echo "       or teleop_twist_keyboard's 'j'/'l' keys to strafe)."
	echo "    4. Note the new desk cylinder position."
	echo "    5. The two positions should differ by < 0.30 m (drift)."
	echo "    Or re-run with:  bash scripts/check_day6_5.sh --auto --auto-dynamic"
	_prompt "Press ENTER after the walk + visual check, then type Y if drift < 0.30 m, N otherwise:"
	read -r _dyn_ans
	case "${_dyn_ans}" in
		Y|y|YES|yes) _pass "DYNAMIC drift < 0.30 m (operator-confirmed)" ;;
		N|n|NO|no)   _fail "DYNAMIC drift >= 0.30 m — projection isn't yet stable." ;;
		*)           _warn "DYNAMIC check skipped (operator answered '$(_repr "${_dyn_ans}")')." ;;
	esac
fi

# ----------------------------------------------------------------------------
# Check 3 — TRACK ID persistence (operator-driven)
# ----------------------------------------------------------------------------
_section "3. TRACK ID persistence: turn-away-and-back keeps the same entity_id"
if ${_AUTO}; then
	_warn "TRACK ID check SKIPPED (--auto). Re-run without --auto to perform manually."
else
	echo "  1. Note the current desk entity_id:"
	echo "     ros2 topic echo --once /semantic_map/objects | grep entity_id"
	echo "  2. Rotate Go2 in place by 180° (face away from the desk):"
	echo "       ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 0.7}}' &"
	echo "       sleep 5 && pkill -f 'pub /cmd_vel'"
	echo "  3. Wait 5 s without moving."
	echo "  4. Rotate back another 180° to face the desk again."
	echo "  5. Echo the topic again — should be the SAME entity_id as step 1."
	_prompt "Press ENTER after the test, then type Y if entity_id was preserved, N otherwise:"
	read -r _track_ans
	case "${_track_ans}" in
		Y|y|YES|yes) _pass "TRACK ID preserved across 5 s out-of-FOV gap" ;;
		N|n|NO|no)   _fail "TRACK ID changed — entity got pruned. Increase " \
		                   "visibility_timeout_sec / lower confidence_decay_rate / " \
		                   "lower selector_min_confidence." ;;
		*)           _warn "TRACK ID check skipped (operator answered '$(_repr "${_track_ans}")')." ;;
	esac
fi

# ----------------------------------------------------------------------------
# Check 4 — END-TO-END approach (operator-driven, monitored)
# ----------------------------------------------------------------------------
_section "4. END-TO-END APPROACH: NavigateToPose succeeds + Go2 stops within 0.30 m"
if ${_AUTO}; then
	_warn "END-TO-END check SKIPPED (--auto). Re-run without --auto to perform manually."
else
	echo "  Watch the nav2.launch.py shell for:"
	echo "      [bt_navigator] : Begin navigating from current location ..."
	echo "      [bt_navigator] : Goal succeeded"
	echo
	echo "  Make sure target_class is set to a class that exists in"
	echo "  /semantic_map/objects (likely 'desk', not 'chair', because YOLOE"
	echo "  labels the sim table as 'desk'):"
	echo
	echo "      ros2 param set /target_selector target_class desk"
	echo
	echo "  Approach distance (default 0.9 m for chair, 1.0 m for desk) is"
	echo "  the radius of the ring. Tolerance: Go2 should stop within"
	echo "  0.30 m of that distance (i.e. between 0.7 m and 1.3 m for the"
	echo "  desk default)."
	_prompt "Press ENTER after the approach test, then type Y if Go2 stopped near the approach pose, N otherwise:"
	read -r _e2e_ans
	case "${_e2e_ans}" in
		Y|y|YES|yes) _pass "END-TO-END Go2 reached approach pose within tolerance" ;;
		N|n|NO|no)   _fail "END-TO-END Go2 did not reach the approach pose. Either " \
		                   "(a) entity got pruned mid-traverse — see check 3 — or " \
		                   "(b) Nav2 controller_server / collision_monitor aborted; " \
		                   "see nav2 shell logs." ;;
		*)           _warn "END-TO-END check skipped (operator answered '$(_repr "${_e2e_ans}")')." ;;
	esac
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
_section "Summary"
echo "  ${C_GRN}PASS=${_PASS}${C_END}  ${C_YLW}WARN=${_WARN}${C_END}  ${C_RED}FAIL=${_FAIL}${C_END}"

cat <<'EOF'

  Day 6.5 gate: ALL 4 checks must report PASS to open Day 8.

  If a check failed, parameters to tune (in order of expected impact):
    [1] depth_projector_node:
        bbox_shrink         (current 0.20 — try 0.25 if static err high)
        depth_percentile    (current 30   — try 25 if static err high,
                                            try 35 if missing detections)
    [2] semantic_memory_aggregator_node:
        nms_radius_m        (current 0.8 — keep loose until masks fix)
        confidence_decay_rate (current 0.02 — lower if track id flapping)
        visibility_timeout_sec (current 5.0 — raise to 8.0 if 360° turns
                                              prune the entity)
    [3] target_selector_node:
        min_confidence      (current 0.20 — keep with looser memory)

  Live tuning, no relaunch needed:
    ros2 param set /depth_projector bbox_shrink 0.25
    ros2 param set /semantic_memory_aggregator nms_radius_m 1.0
    ...
EOF

if [ "${_FAIL}" -eq 0 ]; then
	echo "  Day 6.5 acceptance: ${C_GRN}OK${C_END} — Day 8 (frontier) cleared to start."
	exit 0
else
	echo "  Day 6.5 acceptance: ${C_RED}FAILED${C_END} (${_FAIL} hard) — DO NOT start Day 8."
	exit 1
fi
