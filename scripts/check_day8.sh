#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Day 8 acceptance gate — Frontier exploration end-to-end.
#
# Day 8 turns the FSM's "I don't know where the target is" branch from a
# placeholder in-place spin into actual frontier-driven exploration. Per
# docs/day8_status.md, four checks must pass before opening Day 9:
#
#   1. FRONTIER UNIT — the detector produces frontier centroids on a
#      hand-built 50x50 OccupancyGrid where left half = free, right half
#      = unknown. Centroids must land within ±5 cells of the boundary
#      column. Pure offline test, no rclpy / sim required.
#
#   2. FRONTIER CONSUMPTION — with the live stack up, calling
#      /get_frontiers, then driving Go2 ~30 s, then calling again must
#      return strictly fewer frontiers (or zero — fully explored).
#
#   3. AUTONOMOUS DISCOVERY (operator-driven) — put a chair in the sim
#      where Go2 cannot see it from spawn, send SemanticTask
#      target_class=chair, watch Go2 explore until ARRIVED.
#
#   4. EXHAUSTED → FAILED — send SemanticTask for a class that does not
#      exist in the scene (microwave). Go2 should explore until the
#      frontier list is empty AND task_coordinator must transition to
#      FAILED with reason mentioning "explored" / "not found". The
#      script monitors /task/status with a hard timeout.
#
# Modes:
#   bash scripts/check_day8.sh
#       Runs all 4 with manual prompts for #3.
#
#   bash scripts/check_day8.sh --auto
#       Skips #3 (operator confirmation only), still runs #1, #2, #4.
#
#   bash scripts/check_day8.sh --auto-all
#       Skips both #3 and #4's manual confirmation; relies on the
#       /task/status FAILED-timeout heuristic only.
#
# Useful flags:
#   --drive-sec N      duration of cmd_vel pulse in #2 (default 30)
#   --linear-x V       forward velocity for #2 (default 0.30)
#   --angular-z V      yaw rate for #2 (default 0.10)
#   --discovery-to N   #3 wall timeout in seconds (default 300)
#   --absent-class S   class for #4 (default microwave)
#   --absent-to N      #4 wall timeout in seconds (default 300)
#
# Optional env (same effect as flags where noted):
#   CHECK_DAY8_DRIVE_SEC, CHECK_DAY8_LINEAR_X, CHECK_DAY8_ANGULAR_Z,
#   CHECK_DAY8_DISCOVERY_TO, CHECK_DAY8_ABSENT_CLASS, CHECK_DAY8_ABSENT_TO
#
# Run AFTER:
#   bash scripts/run_warehouse_ros2.sh
#   ros2 launch go2_bringup_sim chair_perception.launch.py
#   ros2 launch go2_bringup_sim nav2.launch.py            # slam:=True OK
#   ros2 launch go2_bringup_sim day8.launch.py target_class:=chair
# -----------------------------------------------------------------------------
set -uo pipefail

_AUTO=false
_AUTO_ALL=false
_DRIVE_SEC="${CHECK_DAY8_DRIVE_SEC:-30}"
_LINEAR_X="${CHECK_DAY8_LINEAR_X:-0.30}"
_ANGULAR_Z="${CHECK_DAY8_ANGULAR_Z:-0.10}"
_DISCOVERY_TO="${CHECK_DAY8_DISCOVERY_TO:-300}"
_ABSENT_CLASS="${CHECK_DAY8_ABSENT_CLASS:-microwave}"
_ABSENT_TO="${CHECK_DAY8_ABSENT_TO:-300}"

while [ "${#}" -gt 0 ]; do
	case "$1" in
		--auto)
			_AUTO=true
			shift
			;;
		--auto-all)
			_AUTO=true
			_AUTO_ALL=true
			shift
			;;
		--drive-sec)
			_DRIVE_SEC="${2:?--drive-sec requires a number}"
			shift 2
			;;
		--linear-x)
			_LINEAR_X="${2:?--linear-x requires a number}"
			shift 2
			;;
		--angular-z)
			_ANGULAR_Z="${2:?--angular-z requires a number}"
			shift 2
			;;
		--discovery-to)
			_DISCOVERY_TO="${2:?--discovery-to requires seconds}"
			shift 2
			;;
		--absent-class)
			_ABSENT_CLASS="${2:?--absent-class requires a class name}"
			shift 2
			;;
		--absent-to)
			_ABSENT_TO="${2:?--absent-to requires seconds}"
			shift 2
			;;
		-h|--help)
			tail -n +3 "${BASH_SOURCE[0]:-$0}" | grep '^#' | head -n 80 | sed 's/^# \{0,1\}//'
			exit 0
			;;
		*)
			echo "[check_day8] ERROR: unknown argument: $1 (try --help)" >&2
			exit 2
			;;
	esac
done

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[check_day8] ERROR: 'ros2' not found." >&2
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
_repr()    { printf '%s' "${1:-}" | tr -d '\r\n' | cut -c1-120; }

# Per-gate verdicts. The Day 8 design says "ALL 4 checks must report
# PASS to open Day 9", so the final summary needs to distinguish:
#   PASS  — gate ran and passed
#   FAIL  — gate ran and failed
#   SKIP  — gate did not run (no infrastructure, --auto, etc.)
# A SKIP is NOT a pass — it leaves Day 8 INCOMPLETE.
_GATE1_RESULT="SKIP"
_GATE2_RESULT="SKIP"
_GATE3_RESULT="SKIP"
_GATE4_RESULT="SKIP"
_set_gate() { eval "_GATE${1}_RESULT=\"${2}\""; }

# ----------------------------------------------------------------------------
# Check 1 — FRONTIER UNIT (offline, ROS not required)
# ----------------------------------------------------------------------------
_section "1. FRONTIER UNIT — algorithm on a hand-built OccupancyGrid"
_unit_py="${_SCRIPT_DIR}/_check_day8_frontier_unit.py"
if [ ! -r "${_unit_py}" ]; then
	_fail "FRONTIER UNIT: missing helper ${_unit_py}"
	_set_gate 1 FAIL
else
	_unit_summary="$(timeout 30 python3 "${_unit_py}" 2>&1)"
	_unit_ec=$?
	echo "  ${_unit_summary:-<empty>}"
	case "${_unit_ec}" in
		0)
			if echo "${_unit_summary}" | grep -q "pass=1"; then
				_pass "FRONTIER UNIT: centroids near boundary column, scores sorted"
				_set_gate 1 PASS
			else
				_warn "FRONTIER UNIT: exit 0 but no pass=1 line — treating as inconclusive"
				_set_gate 1 FAIL
			fi
			;;
		1)
			if echo "${_unit_summary}" | grep -qE "^ERROR_(EMPTY|BAD_CENTROIDS|NOT_SORTED|NOT_SUCCESS|NONFINITE|ZERO_INFO_GAIN)"; then
				_fail "FRONTIER UNIT: algorithm misbehaved — ${_unit_summary}"
			else
				_fail "FRONTIER UNIT: exit 1 — ${_unit_summary}"
			fi
			_set_gate 1 FAIL
			;;
		2)
			if echo "${_unit_summary}" | grep -q "ERROR_IMPORT_PACKAGE"; then
				_fail "FRONTIER UNIT: cannot import go2_navigation. " \
				      "Run: colcon build --packages-select go2_msgs go2_navigation && source install/setup.bash"
			else
				_fail "FRONTIER UNIT: dependency missing — ${_unit_summary}"
			fi
			_set_gate 1 FAIL
			;;
		*)
			_fail "FRONTIER UNIT: unexpected exit ${_unit_ec} — ${_unit_summary:-no output}"
			_set_gate 1 FAIL
			;;
	esac
fi

# ----------------------------------------------------------------------------
# Pre-flight for #2 #3 #4: live stack must be up
# ----------------------------------------------------------------------------
_section "0. Day 8 LIVE pre-flight (sim + nav2 + day8.launch.py must be running)"
_NODES_RAW="$(timeout 4 ros2 node list 2>/dev/null || true)"
_REQ_NODES=(
	/yoloe_detector
	/depth_projector
	/semantic_memory_aggregator
	/target_selector
	/approach_goal_planner
	/frontier_explorer
	/task_coordinator
)
_LIVE_OK=true
for n in "${_REQ_NODES[@]}"; do
	if echo "${_NODES_RAW}" | grep -Fxq "$n"; then
		_pass "${n} up"
	else
		_warn "${n} NOT in node list — checks 2/3/4 will SKIP"
		_LIVE_OK=false
	fi
done

if ${_LIVE_OK}; then
	# /map present
	_MAP_HZ_RAW="$(timeout 5 ros2 topic hz /map 2>&1 || true)"
	if echo "${_MAP_HZ_RAW}" | grep -q "no new messages"; then
		# Latched topics often show "no new messages" — confirm presence with `echo --once`.
		if timeout 4 ros2 topic echo --once /map >/dev/null 2>&1; then
			_pass "/map latched (one-shot echo succeeded)"
		else
			_warn "/map present but no payload received in 4 s — SLAM may not be publishing yet"
			_LIVE_OK=false
		fi
	elif echo "${_MAP_HZ_RAW}" | grep -qE "average rate"; then
		_pass "/map publishing"
	else
		_warn "/map not seen in 5 s — start mapping.launch.py or nav2.launch.py with slam:=True"
		_LIVE_OK=false
	fi

	# /global_costmap/costmap is TRANSIENT_LOCAL, latched.
	_CM_INFO="$(timeout 4 ros2 topic echo --once --field info /global_costmap/costmap 2>&1 || true)"
	_cm_w="$(echo "${_CM_INFO}" | sed -n 's/^[[:space:]]*width: \([0-9]*\).*/\1/p' | head -n1)"
	_cm_h="$(echo "${_CM_INFO}" | sed -n 's/^[[:space:]]*height: \([0-9]*\).*/\1/p' | head -n1)"
	if [ -z "${_cm_w}" ] || [ -z "${_cm_h}" ]; then
		_warn "/global_costmap/costmap not latched yet — Nav2 may still be activating"
		_LIVE_OK=false
	elif [ "${_cm_w}" -le 0 ] || [ "${_cm_h}" -le 0 ]; then
		_fail "/global_costmap/costmap is empty (${_cm_w}x${_cm_h}). " \
		      "Drive Go2 with teleop first to seed the SLAM map."
		_LIVE_OK=false
	else
		_pass "/global_costmap/costmap latched ${_cm_w}x${_cm_h}"
	fi
fi

if ! ${_LIVE_OK}; then
	echo
	echo "  ${C_YLW}LIVE pre-flight incomplete.${C_END} Checks 2/3/4 require the full Day 8"
	echo "  stack. Bring it up:"
	echo "    bash scripts/run_warehouse_ros2.sh"
	echo "    ros2 launch go2_bringup_sim chair_perception.launch.py"
	echo "    ros2 launch go2_bringup_sim nav2.launch.py"
	echo "    ros2 launch go2_bringup_sim day8.launch.py target_class:=chair"
fi

# ----------------------------------------------------------------------------
# Check 2 — FRONTIER CONSUMPTION
# ----------------------------------------------------------------------------
_section "2. FRONTIER CONSUMPTION — drive Go2, frontier count must drop"
if ! ${_LIVE_OK}; then
	_warn "FRONTIER CONSUMPTION SKIPPED (live pre-flight failed)"
	# _GATE2_RESULT stays SKIP
else
	_consume_py="${_SCRIPT_DIR}/_check_day8_consumption.py"
	if [ ! -r "${_consume_py}" ]; then
		_fail "FRONTIER CONSUMPTION: missing helper ${_consume_py}"
		_set_gate 2 FAIL
	else
		_consume_to=$((_DRIVE_SEC + 60))
		_consume_summary="$(timeout "${_consume_to}" python3 "${_consume_py}" \
		    "${_DRIVE_SEC}" "${_LINEAR_X}" "${_ANGULAR_Z}" \
		    "2.0" "10.0" "10.0" 2>&1)"
		_consume_ec=$?
		echo "  ${_consume_summary:-<empty>}"
		case "${_consume_ec}" in
			0)
				if echo "${_consume_summary}" | grep -q "pass=1"; then
					_pass "FRONTIER CONSUMPTION: post < pre"
					_set_gate 2 PASS
				else
					_warn "FRONTIER CONSUMPTION: exit 0 but no pass=1 line"
					_set_gate 2 FAIL
				fi
				;;
			1)
				if echo "${_consume_summary}" | grep -q "pass=0"; then
					_n1=$(echo "${_consume_summary}" | sed -n 's/.*n1=\([0-9]*\).*/\1/p')
					_n2=$(echo "${_consume_summary}" | sed -n 's/.*n2=\([0-9]*\).*/\1/p')
					_moved=$(echo "${_consume_summary}" | sed -n 's/.*moved=\([0-9.]*\)m.*/\1/p')
					_fail "FRONTIER CONSUMPTION: n2(${_n2}) >= n1(${_n1}) after moving ${_moved}m. " \
					      "Either Go2 didn't actually move (cmd_vel rejected by safety/teleop), " \
					      "or SLAM map isn't extending fast enough — check /map updates in RViz."
				else
					_fail "FRONTIER CONSUMPTION: exit 1 — ${_consume_summary}"
				fi
				_set_gate 2 FAIL
				;;
			2)
				_fail "FRONTIER CONSUMPTION: setup error — ${_consume_summary}"
				_set_gate 2 FAIL
				;;
			*)
				_fail "FRONTIER CONSUMPTION: runtime error (exit ${_consume_ec}) — ${_consume_summary:-no output}"
				_set_gate 2 FAIL
				;;
		esac
	fi
fi

# ----------------------------------------------------------------------------
# Check 3 — AUTONOMOUS DISCOVERY (operator-driven)
# ----------------------------------------------------------------------------
_section "3. AUTONOMOUS DISCOVERY — Go2 finds an out-of-FOV chair via EXPLORE"
if ${_AUTO}; then
	_warn "AUTONOMOUS DISCOVERY SKIPPED (--auto). Re-run without --auto to perform manually."
	# _GATE3_RESULT stays SKIP — operator chose to skip; Day 8 incomplete.
elif ! ${_LIVE_OK}; then
	_warn "AUTONOMOUS DISCOVERY SKIPPED (live pre-flight failed)"
	# _GATE3_RESULT stays SKIP
else
	echo "  Setup:"
	echo "    1. Read docs/day8_test_setup.md for chair placement (out of"
	echo "       Go2's spawn FOV — typical: place behind a wall or around"
	echo "       a corner so Go2 needs >=1 frontier hop to discover it)."
	echo "    2. Make sure RViz shows /frontier_markers (green = best),"
	echo "       /semantic_map/markers, and the SLAM /map."
	echo
	echo "  This script will:"
	echo "    a) tail /task/status into a log file (printed at the end)"
	echo "    b) publish a SemanticTask(target_class=chair) once"
	echo "    c) wait up to ${_DISCOVERY_TO}s for ARRIVED, then prompt you"
	_prompt "Press ENTER when chair is placed and RViz is open:"
	read -r _

	_disc_log="$(mktemp -t check_day8_discovery.XXXXXX.log)"
	# Tail /task/status in the background — captures full state sequence.
	(timeout "${_DISCOVERY_TO}" ros2 topic echo /task/status \
	    --field data 2>/dev/null \
	    | awk '{ printf("%s %s\n", strftime("%H:%M:%S"), $0) }' \
	    > "${_disc_log}") &
	_disc_pid=$!

	echo "  publishing SemanticTask(target_class=chair, task_id=disc-001)..."
	timeout 5 ros2 topic pub --once /semantic_task/request \
	    go2_msgs/msg/SemanticTask \
	    "{header: {frame_id: 'map'}, task_id: 'disc-001', target_class: 'chair', requires_search: true}" \
	    >/dev/null 2>&1 || _warn "topic pub /semantic_task/request returned non-zero"

	echo "  monitoring /task/status until ARRIVED or timeout (${_DISCOVERY_TO}s)..."
	_disc_ok=false
	_disc_deadline=$(( $(date +%s) + _DISCOVERY_TO ))
	while [ "$(date +%s)" -lt "${_disc_deadline}" ]; do
		if grep -qE "^[0-9:]+ ARRIVED$" "${_disc_log}" 2>/dev/null; then
			_disc_ok=true
			break
		fi
		if grep -qE "^[0-9:]+ FAILED" "${_disc_log}" 2>/dev/null; then
			break
		fi
		sleep 2
	done
	# stop the background tail
	kill "${_disc_pid}" 2>/dev/null || true
	wait "${_disc_pid}" 2>/dev/null || true

	echo "  --- /task/status sequence (last 60 transitions) ---"
	awk '!seen[$0]++' "${_disc_log}" | tail -n 60 | sed 's/^/    /'
	echo "  --- end sequence (full log: ${_disc_log}) ---"

	if ${_disc_ok}; then
		_pass "AUTONOMOUS DISCOVERY: task_coordinator reached ARRIVED automatically"
		_set_gate 3 PASS
	else
		_prompt "Did Go2 visually reach the chair? (Y/N): "
		read -r _disc_ans
		case "${_disc_ans}" in
			Y|y|YES|yes)
				_pass "AUTONOMOUS DISCOVERY: operator-confirmed (timed out before FSM hit ARRIVED, but visually OK)"
				_set_gate 3 PASS
				;;
			N|n|NO|no)
				_fail "AUTONOMOUS DISCOVERY: Go2 did not reach the chair within ${_DISCOVERY_TO}s."
				_set_gate 3 FAIL
				;;
			*)
				_warn "AUTONOMOUS DISCOVERY skipped (operator answered '$(_repr "${_disc_ans}")')."
				# _GATE3_RESULT stays SKIP
				;;
		esac
	fi
fi

# ----------------------------------------------------------------------------
# Check 4 — EXHAUSTED → FAILED  (semi-automatic)
# ----------------------------------------------------------------------------
_section "4. EXHAUSTED → FAILED — looking for an absent class must end in FAILED"
if ! ${_LIVE_OK}; then
	_warn "EXHAUSTED check SKIPPED (live pre-flight failed)"
	# _GATE4_RESULT stays SKIP
else
	if ! ${_AUTO_ALL} && ! ${_AUTO}; then
		echo "  This check sends SemanticTask(target_class=${_ABSENT_CLASS}) — a class"
		echo "  that should NOT exist in the warehouse scene — and expects the"
		echo "  task_coordinator to enter FAILED with reason mentioning 'explored'"
		echo "  or 'not found' within ${_ABSENT_TO}s."
		_prompt "Press ENTER to begin (or Ctrl-C to skip):"
		read -r _
	else
		echo "  (--auto / --auto-all) sending SemanticTask(target_class=${_ABSENT_CLASS}) automatically."
	fi

	_abs_log="$(mktemp -t check_day8_absent.XXXXXX.log)"
	(timeout "${_ABSENT_TO}" ros2 topic echo /task/status \
	    --field data 2>/dev/null \
	    | awk '{ printf("%s %s\n", strftime("%H:%M:%S"), $0) }' \
	    > "${_abs_log}") &
	_abs_pid=$!

	echo "  publishing SemanticTask(target_class=${_ABSENT_CLASS}, task_id=absent-001)..."
	timeout 5 ros2 topic pub --once /semantic_task/request \
	    go2_msgs/msg/SemanticTask \
	    "{header: {frame_id: 'map'}, task_id: 'absent-001', target_class: '${_ABSENT_CLASS}', requires_search: true}" \
	    >/dev/null 2>&1 || _warn "topic pub /semantic_task/request returned non-zero"

	echo "  monitoring /task/status until FAILED or timeout (${_ABSENT_TO}s)..."
	_abs_done=false
	_abs_arrived=false
	_abs_deadline=$(( $(date +%s) + _ABSENT_TO ))
	while [ "$(date +%s)" -lt "${_abs_deadline}" ]; do
		if grep -qE "^[0-9:]+ FAILED" "${_abs_log}" 2>/dev/null; then
			_abs_done=true
			break
		fi
		if grep -qE "^[0-9:]+ ARRIVED$" "${_abs_log}" 2>/dev/null; then
			_abs_arrived=true
			break
		fi
		sleep 2
	done
	kill "${_abs_pid}" 2>/dev/null || true
	wait "${_abs_pid}" 2>/dev/null || true

	echo "  --- /task/status sequence (last 60 transitions) ---"
	awk '!seen[$0]++' "${_abs_log}" | tail -n 60 | sed 's/^/    /'
	echo "  --- end sequence (full log: ${_abs_log}) ---"

	if ${_abs_arrived}; then
		_fail "EXHAUSTED: FSM reached ARRIVED for class '${_ABSENT_CLASS}' — " \
		      "perception falsely matched something. Tighten classes list / confidence."
		_set_gate 4 FAIL
	elif ${_abs_done}; then
		# Pull the FAILED reason: state line is "FAILED:reason..." per coordinator.
		_reason=$(grep -E "^[0-9:]+ FAILED" "${_abs_log}" | tail -n1 | sed 's/^[0-9:]* //')
		if echo "${_reason}" | grep -qiE "explored|not found"; then
			_pass "EXHAUSTED: FAILED with expected reason — ${_reason}"
			_set_gate 4 PASS
		else
			_warn "EXHAUSTED: FAILED but reason mentions neither 'explored' nor 'not found' — ${_reason}"
			# Treat as FAIL for gate verdict — the message contract isn't satisfied.
			_set_gate 4 FAIL
		fi
	else
		_fail "EXHAUSTED: ${_ABSENT_TO}s timeout, FSM still not in FAILED. " \
		      "Likely EXPLORE keeps finding new frontiers (warehouse too open) " \
		      "OR coordinator's environment-fully-explored path is dead. " \
		      "Inspect with: ros2 topic echo /task/status"
		_set_gate 4 FAIL
	fi
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
_section "Summary"

_color_for() {
	# Echo the colour code for a per-gate verdict word.
	case "$1" in
		PASS) printf '%s' "${C_GRN}" ;;
		FAIL) printf '%s' "${C_RED}" ;;
		SKIP) printf '%s' "${C_YLW}" ;;
		*)    printf '%s' "" ;;
	esac
}

_print_gate() {
	# $1 = gate number, $2 = verdict, $3 = short label
	local col; col=$(_color_for "$2")
	printf "  Gate %s %-22s : %s%s%s\n" "$1" "$3" "${col}" "$2" "${C_END}"
}

_print_gate 1 "${_GATE1_RESULT}" "FRONTIER UNIT"
_print_gate 2 "${_GATE2_RESULT}" "FRONTIER CONSUMPTION"
_print_gate 3 "${_GATE3_RESULT}" "AUTONOMOUS DISCOVERY"
_print_gate 4 "${_GATE4_RESULT}" "EXHAUSTED → FAILED"
echo
echo "  Counters: ${C_GRN}PASS=${_PASS}${C_END}  ${C_YLW}WARN=${_WARN}${C_END}  ${C_RED}FAIL=${_FAIL}${C_END}"

# Verdict logic (the bit Day 6.5's script gets wrong for our needs):
#   * Any gate FAIL              → FAILED   (exit 1)
#   * Any gate SKIP              → INCOMPLETE  (exit 1)
#   * All four gates PASS        → OK       (exit 0)
# A SKIP is NOT a pass. --auto skipping #3 still leaves Day 8 incomplete
# from the acceptance-gate perspective.
_n_fail=0
_n_skip=0
for v in "${_GATE1_RESULT}" "${_GATE2_RESULT}" "${_GATE3_RESULT}" "${_GATE4_RESULT}"; do
	case "$v" in
		FAIL) _n_fail=$((_n_fail + 1)) ;;
		SKIP) _n_skip=$((_n_skip + 1)) ;;
	esac
done

cat <<'EOF'

  Day 8 gate: ALL 4 checks must report PASS to open Day 9.

  If a check failed, parameters to tune (in order of expected impact):
    [1] FRONTIER UNIT failures usually mean the algorithm itself is
        broken — run scripts/_check_day8_frontier_unit.py standalone
        and read the printed centroid_cell_x list.
    [2] FRONTIER CONSUMPTION:
        check_day8.sh --linear-x 0.5 --drive-sec 45    (drive faster/longer)
        Confirm /cmd_vel actually reaches the robot (safety_monitor
        not eating the message), and that SLAM is updating /map.
    [3] AUTONOMOUS DISCOVERY needs the chair placed truly out of the
        spawn FOV. If Go2 sees it from spawn, EXPLORE is not exercised.
    [4] EXHAUSTED → FAILED:
        Check task_coordinator log for "EXPLORE: environment fully
        explored, target ... not found". If frontier_explorer keeps
        returning frontiers, raise min_cluster_size or shrink the
        scene. If FAILED never fires, inspect _maybe_fail_after_aborts
        and the explorer empty-list code path.

  Live tuning, no relaunch needed:
    ros2 param set /frontier_explorer min_cluster_size 20
    ros2 param set /frontier_explorer distance_weight 8.0
    ros2 param set /task_coordinator parse_command_fallback_sec 2.5
EOF

if [ "${_n_fail}" -gt 0 ]; then
	echo "  Day 8 acceptance: ${C_RED}FAILED${C_END} (${_n_fail} gate(s) failed) — DO NOT start Day 9."
	exit 1
elif [ "${_n_skip}" -gt 0 ]; then
	echo "  Day 8 acceptance: ${C_YLW}INCOMPLETE${C_END} (${_n_skip} gate(s) skipped) — bring up the live stack and re-run without --auto."
	exit 1
else
	echo "  Day 8 acceptance: ${C_GRN}OK${C_END} — Day 9 cleared to start."
	exit 0
fi
