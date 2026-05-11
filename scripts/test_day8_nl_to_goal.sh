#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# test_day8_nl_to_goal.sh — NL chain + TRANSIENT_LOCAL stale guard
#
# TRANSIENT_LOCAL topics may echo the *previous* command; this script classifies
# feedback / semantic_task as OK vs STALE vs MISSING and only FAILs when
# target_selector did not update (unless you only care about parser).
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	# shellcheck disable=SC1091
	source /opt/ros/jazzy/setup.bash 2>/dev/null || true
fi

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[test_day8_nl_to_goal] ERROR: 'ros2' not found." >&2
	exit 2
fi

_ECHO_TIMEOUT="${ECHO_TIMEOUT:-10}"
USER_CMD="${USER_CMD:-go to person}"
EXPECTED_CLASS="${EXPECTED_CLASS:-person}"

banner() {
	echo
	echo "===================================================================="
	echo "  $1"
	echo "===================================================================="
}

get_target_class_raw() {
	ros2 param get /target_selector target_class 2>&1 || true
}

# Uses nl_parser's ``raw={text!r}`` encoding — parses ``data`` from ros2 YAML.
feedback_ok_for_cmd() {
	export FB_PAYLOAD="$1"
	python3 - <<'PY' || true
import ast
import os


def parse_string_payload(blob: str) -> str:
    lines = blob.splitlines()
    for ln in lines:
        s = ln.strip()
        if not s.startswith("data:"):
            continue
        val = s[5:].strip()
        try:
            if val and val[0] in "'\"":
                parsed = ast.literal_eval(val.split(" #", 1)[0].strip())
                if isinstance(parsed, str):
                    return parsed
        except Exception:
            pass
        quote = "'"
        if val.startswith('"'):
            quote = '"'
        if val.startswith("'") or val.startswith('"'):
            if len(val) >= 2 and val.endswith(quote):
                return val[1:-1]
        return val
    return blob


payload = parse_string_payload(os.environ.get("FB_PAYLOAD", "") or "")
cmd = (os.environ.get("USER_CMD", "") or "").strip()
needle = "raw=" + repr(cmd)
raise SystemExit(0 if needle in payload else 1)
PY
}

# Parse one ros2-topic-echo SemanticTask YAML block for this USER_CMD run.
semantic_ok_for_cmd() {
	export ST_BLOCK="$1"
	export EXPECTED_CLASS="${EXPECTED_CLASS:-}"
	export USER_CMD="${USER_CMD:-}"
	python3 - <<'PY' || true
import os


def yaml_simple_value(line: str) -> str:
    tail = line.split(":", 1)[1].strip()
    if len(tail) >= 2:
        if (tail[0] == tail[-1] == "'") or (tail[0] == tail[-1] == '"'):
            return tail[1:-1]
    return tail


want_cmd = (os.environ.get("USER_CMD", "") or "").strip()
want_cls = (os.environ.get("EXPECTED_CLASS", "") or "").strip()
raw_cmd = tgt_cls = None
for raw_ln in (os.environ.get("ST_BLOCK", "") or "").splitlines():
    ln = raw_ln.strip()
    if ln.startswith("raw_command:"):
        raw_cmd = yaml_simple_value(ln).strip()
    elif ln.startswith("target_class:"):
        tgt_cls = yaml_simple_value(ln).strip()
ok = raw_cmd == want_cmd and tgt_cls == want_cls
raise SystemExit(0 if ok else 1)
PY
}

banner "Topic wiring (before command)"
echo "  ros2 topic info /user_command -v"
timeout 5 ros2 topic info /user_command -v 2>&1 || echo "  (topic info failed)"
echo
echo "  ros2 topic info /semantic_task/request -v"
timeout 5 ros2 topic info /semantic_task/request -v 2>&1 || echo "  (topic info failed)"

banner "Step 1 — /target_selector target_class (BEFORE)"
BEFORE_TC=$(get_target_class_raw)
echo "  ${BEFORE_TC}"

banner "Step 2 — publish /user_command"
echo "  USER_CMD='${USER_CMD}'  EXPECTED_CLASS='${EXPECTED_CLASS}'"
echo "  ros2 topic pub --once /user_command std_msgs/msg/String \"{data: '${USER_CMD}'}\""
# shellcheck disable=SC2086
ros2 topic pub --once /user_command std_msgs/msg/String "{data: '${USER_CMD}'}"

_SETTLE="${POST_PUB_SLEEP:-2}"
echo "  (sleep ${_SETTLE}s)"
sleep "${_SETTLE}"

banner "Step 3 — /nl_parser/feedback"
_FB_OUT=""
if _FB_OUT=$(timeout "${_ECHO_TIMEOUT}" ros2 topic echo --once /nl_parser/feedback std_msgs/msg/String 2>&1); then
	echo "${_FB_OUT}"
else
	echo "  (MISSING — no feedback within ${_ECHO_TIMEOUT}s)"
fi

FB_STATUS="MISSING"
if [[ -n "${_FB_OUT}" ]] && echo "${_FB_OUT}" | grep -q '^data:'; then
	if feedback_ok_for_cmd "${_FB_OUT}"; then
		FB_STATUS="OK"
	else
		FB_STATUS="STALE"
		echo ""
		echo "  STALE /nl_parser/feedback: raw command does not match current USER_CMD"
		echo "  (TRANSIENT_LOCAL may be replaying an older RECEIVED/OK line.)"
	fi
fi

banner "Step 4 — /semantic_task/request"
_ST_OUT=""
if _ST_OUT=$(timeout "${_ECHO_TIMEOUT}" ros2 topic echo --once /semantic_task/request go2_msgs/msg/SemanticTask 2>&1); then
	echo "${_ST_OUT}"
else
	echo "  (MISSING — no semantic_task within ${_ECHO_TIMEOUT}s)"
fi

ST_STATUS="MISSING"
if [[ -n "${_ST_OUT}" ]] && echo "${_ST_OUT}" | grep -qE '^(task_id|raw_command):'; then
	if semantic_ok_for_cmd "${_ST_OUT}"; then
		ST_STATUS="OK"
	else
		ST_STATUS="STALE"
		echo ""
		echo "  STALE /semantic_task/request: raw_command/target_class does not match this USER_CMD run"
	fi
fi

banner "Step 5 — /target_selector target_class (AFTER)"
AFTER_TC=$(get_target_class_raw)
echo "  ${AFTER_TC}"
_TC_LINE=$(echo "${AFTER_TC}" | grep -E 'String value is:' || true)

TS_STATUS="FAIL"
if echo "${_TC_LINE}" | grep -qF "String value is: ${EXPECTED_CLASS}"; then
	TS_STATUS="OK"
fi

# ---- Entity / goal spot checks -------------------------------------------
_SEL_OUT=""
if _SEL_OUT=$(timeout "${_ECHO_TIMEOUT}" ros2 topic echo --once /target/selected go2_msgs/msg/SelectedTarget 2>&1); then
	: # keep
else
	_SEL_OUT=""
fi
ENTITY_FOUND="$(
	export ENTITY_BLOCK="${_SEL_OUT}"
	export ENTITY_CLASS="${EXPECTED_CLASS}"
	python3 - <<'PY'
import os

block = os.environ.get("ENTITY_BLOCK", "") or ""
want = (os.environ.get("ENTITY_CLASS", "") or "").strip()
entity_val = cls_val = ""
for ln in block.splitlines():
    s = ln.strip()
    if s.startswith("entity_id:"):
        v = s.split(":", 1)[1].strip().strip("'\"")
        if v and not entity_val:
            entity_val = v
    if s.startswith("class_label:"):
        v = s.split(":", 1)[1].strip().strip("'\"")
        if v and not cls_val:
            cls_val = v
print("YES" if (entity_val and cls_val == want) else "NO")
PY
)" || ENTITY_FOUND="NO"

_GOAL_OUT=""
if _GOAL_OUT=$(timeout "${_ECHO_TIMEOUT}" ros2 topic echo --once /semantic_goal/goal_pose geometry_msgs/msg/PoseStamped 2>&1); then
	:
else
	_GOAL_OUT=""
fi
GOAL_POSE="NO"
[[ -n "${_GOAL_OUT}" ]] && echo "${_GOAL_OUT}" | grep -q 'position:' && GOAL_POSE="YES"

banner "Interpretation summary"
echo "  NL parser current feedback:     ${FB_STATUS}"
echo "  semantic_task current request:  ${ST_STATUS}"
echo "  target_selector class update:   ${TS_STATUS}"
echo "  target entity found (${EXPECTED_CLASS}): ${ENTITY_FOUND}"
echo "  goal_pose published:            ${GOAL_POSE}"

# ---- PASS if target_selector updated (fallback path acceptable) --------
if [[ "${TS_STATUS}" == "OK" ]]; then
	if [[ "${FB_STATUS}" == "STALE" ]] || [[ "${ST_STATUS}" == "STALE" ]]; then
		echo ""
		echo "  feedback stale, but target_selector updated correctly."
		echo "  (task_coordinator / fallback parser may have set target_class)."
	fi
	echo ""
	echo "[test_day8_nl_to_goal] OK — target_selector target_class=${EXPECTED_CLASS}"
	exit 0
fi

echo
echo "======================================================================"
echo "  NL command chain failed (target_selector not updated)"
echo "======================================================================"
if [[ "${FB_STATUS}" == "STALE" ]]; then
	echo "  (/nl_parser/feedback was stale for this USER_CMD — not sufficient alone"
	echo "   to diagnose parser vs coordinator.)"
fi
if [[ "${ST_STATUS}" == "STALE" ]]; then
	echo "  (/semantic_task/request was stale — same caveat.)"
fi
echo "  Diagnostics:"
echo "--- /nl_parser/feedback ---"
echo "${_FB_OUT:-<empty>}"
echo "--- /semantic_task/request ---"
echo "${_ST_OUT:-<empty>}"
echo "--- /task/status ---"
timeout 5 ros2 topic echo --once /task/status std_msgs/msg/String 2>&1 || true
echo "--- /task/status/debug ---"
timeout 5 ros2 topic echo --once /task/status/debug std_msgs/msg/String 2>&1 || true
echo "--- target_selector ---"
echo "${AFTER_TC}"
exit 3
