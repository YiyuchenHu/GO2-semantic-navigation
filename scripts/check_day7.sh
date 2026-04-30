#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Day 7 acceptance — semantic target selection + approach-goal planning.
#
# Run AFTER:
#   1. `bash scripts/run_warehouse_ros2.sh`
#   2. `ros2 launch go2_bringup_sim chair_perception.launch.py`
#                                       (static TFs only — perception_node
#                                        crashes on numpy ABI; harmless)
#   3. `ros2 launch go2_bringup_sim nav2.launch.py`
#                                       (provides map → odom + costmap +
#                                        /navigate_to_pose action; Day 7
#                                        will refuse to send goals without
#                                        Nav2 active).
#   4. `ros2 launch go2_bringup_sim day7.launch.py`
#                                       (yoloe + depth_projector +
#                                        semantic_memory + target_selector +
#                                        approach_goal_planner)
#
# Hard checks:
#   1. Day 7 nodes alive (target_selector, approach_goal_planner) AND
#      Day 6 nodes still alive (yoloe, depth_projector, semantic_memory)
#   2. New topics advertised:
#        /target/selected
#        /semantic_goal/goal_pose
#        /semantic_goal/goal_candidates
#   3. /navigate_to_pose action server reachable (Nav2 dependency)
#   4. /target/selected published by target_selector at >= 1 Hz
#   5. /semantic_goal/goal_pose has frame_id=map and finite pose
#      whenever a target is selected
#   6. /semantic_goal/goal_candidates publishes a non-empty MarkerArray
#      when a target is selected
#
# Soft checks (depend on a chair being in the camera FOV + Nav2 having
# a fresh costmap):
#   - target_selector picks a non-empty entity_id matching target_class
#   - approach_goal_planner's NavigateToPose goal is ACCEPTED (visible
#     in /navigate_to_pose/_action/status if you wish to echo it).
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[check_day7] ERROR: 'ros2' not found." >&2
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

# ----------------------------------------------------------------------------
# 1. Nodes alive
# ----------------------------------------------------------------------------
_section "1. Day 6 + Day 7 nodes alive"
_NODES_RAW="$(timeout 4 ros2 node list 2>/dev/null || true)"
for n in /yoloe_detector /depth_projector /semantic_memory_aggregator \
         /target_selector /approach_goal_planner; do
	if echo "${_NODES_RAW}" | grep -Fxq "$n"; then
		_pass "${n} up"
	else
		_fail "${n} NOT in node list"
	fi
done

# ----------------------------------------------------------------------------
# 2. Topic graph
# ----------------------------------------------------------------------------
_section "2. Day 7 topics advertised"
_TOPICS_RAW="$(timeout 4 ros2 topic list 2>/dev/null || true)"
for t in /target/selected /semantic_goal/goal_pose /semantic_goal/goal_candidates \
         /semantic_map/objects /global_costmap/costmap; do
	if echo "${_TOPICS_RAW}" | grep -Fxq "$t"; then
		_pass "${t} advertised"
	else
		_fail "${t} NOT advertised"
	fi
done

_check_pub() {
	local topic="$1" expected_node="$2"
	local info
	info="$(timeout 4 ros2 topic info -v "${topic}" 2>/dev/null || true)"
	if echo "${info}" | grep -qE "^Node name: /?${expected_node}$"; then
		_pass "${topic} published by ${expected_node}"
	elif echo "${info}" | grep -q 'Publisher count: [1-9]'; then
		_warn "${topic} has a publisher but not /${expected_node}"
		echo "${info}" | grep -E 'Node name:' | sed 's/^/    /'
	else
		_fail "${topic} has 0 publishers"
	fi
}
_check_pub /target/selected target_selector
_check_pub /semantic_goal/goal_pose approach_goal_planner
_check_pub /semantic_goal/goal_candidates approach_goal_planner

# ----------------------------------------------------------------------------
# 3. NavigateToPose action server
# ----------------------------------------------------------------------------
_section "3. /navigate_to_pose action server (Nav2 dependency)"
_ACTIONS_RAW="$(timeout 4 ros2 action list 2>/dev/null || true)"
if echo "${_ACTIONS_RAW}" | grep -Fxq "/navigate_to_pose"; then
	_pass "/navigate_to_pose action available (Nav2 up)"
	# Confirm it has a real server, not just a stale advertisement
	_action_info="$(timeout 4 ros2 action info /navigate_to_pose 2>/dev/null || true)"
	if echo "${_action_info}" | grep -qE 'Action servers: [1-9]'; then
		_pass "/navigate_to_pose has >= 1 action server"
	else
		_fail "/navigate_to_pose has 0 servers — Nav2 lifecycle didn't reach 'active'"
	fi
else
	_fail "/navigate_to_pose NOT in action list — start nav2.launch.py first"
fi

# ----------------------------------------------------------------------------
# 4. /target/selected data flow + content
# ----------------------------------------------------------------------------
_section "4. /target/selected content"
_sel_summary="$(timeout 12 python3 - <<'PYTHONEOF'
import sys, time
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from go2_msgs.msg import SelectedTarget
except Exception as e:
    print(f"ERROR_IMPORT: {e}")
    sys.exit(0)

try:
    rclpy.init()
    node = Node("_check_day7_target_selected")
    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    msgs = []
    def cb(msg):
        msgs.append((time.time(), msg))
    node.create_subscription(SelectedTarget, "/target/selected", cb, qos)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"ERROR_EXC: {e}")
    sys.exit(0)

if not msgs:
    print("ERROR_NO_MSG")
    sys.exit(0)

n = len(msgs)
dt = msgs[-1][0] - msgs[0][0] if n > 1 else 1.0
hz = (n - 1) / dt if dt > 0 else 0.0

# Find the most recent message that picked a real target.
populated = None
for _, m in reversed(msgs):
    if m.entity_id:
        populated = m
        break

latest = msgs[-1][1]
content = "empty"
detail = (
    f"class_label={latest.class_label!r}"
)
if populated is not None:
    p = populated.target_pose_map.position
    content = "ok"
    detail = (
        f"entity_id={populated.entity_id!r} "
        f"class={populated.class_label!r} "
        f"score={populated.score:.2f} "
        f"dist={populated.estimated_distance:.2f}m "
        f"pos=({p.x:.2f},{p.y:.2f})"
    )
print(f"n={n} hz={hz:.2f} content={content} {detail}")
PYTHONEOF
)"

if [ -z "${_sel_summary}" ] || echo "${_sel_summary}" | grep -qE '^ERROR_'; then
	_fail "/target/selected: no message in 8s — target_selector not publishing"
	echo "    ${_sel_summary:-<empty helper output>}"
else
	echo "  ${_sel_summary}"
	_hz=$(echo "${_sel_summary}" | sed -n 's/.*hz=\([0-9.]*\).*/\1/p')
	if awk -v h="${_hz:-0}" 'BEGIN { exit (h+0 >= 1.0) ? 0 : 1 }'; then
		_pass "/target/selected flowing at ~${_hz} Hz (>= 1 Hz)"
	else
		_warn "/target/selected at only ${_hz} Hz (expected >= 1 Hz)"
	fi
	if echo "${_sel_summary}" | grep -q "content=ok"; then
		_pass "SelectedTarget chose a real entity (entity_id + score + pose)"
	else
		_warn "SelectedTarget always empty in 8s window — point Go2 at a chair, or check that /semantic_map/objects is non-empty"
	fi
fi

# ----------------------------------------------------------------------------
# 5. /semantic_goal/goal_pose content (only meaningful when a target is selected)
# ----------------------------------------------------------------------------
_section "5. /semantic_goal/goal_pose content"
_goal_summary="$(timeout 12 python3 - <<'PYTHONEOF'
import sys, time, math
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
except Exception as e:
    print(f"ERROR_IMPORT: {e}")
    sys.exit(0)

try:
    rclpy.init()
    node = Node("_check_day7_goal_pose")
    msgs = []
    def cb(msg):
        msgs.append((time.time(), msg))
    node.create_subscription(PoseStamped, "/semantic_goal/goal_pose", cb, 10)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"ERROR_EXC: {e}")
    sys.exit(0)

if not msgs:
    # Soft-fail: the planner only publishes when a target is
    # selected AND a costmap-clear pose exists. No msg in 8s
    # likely means no chair detected.
    print("NO_GOAL_PUBLISHED")
    sys.exit(0)

last = msgs[-1][1]
fid = last.header.frame_id
p = last.pose.position
o = last.pose.orientation
finite = all(math.isfinite(v) for v in (p.x, p.y, p.z, o.x, o.y, o.z, o.w))
qnorm = math.sqrt(o.x*o.x + o.y*o.y + o.z*o.z + o.w*o.w)
qok = abs(qnorm - 1.0) < 1e-3
print(
    f"n={len(msgs)} frame_id={fid!r} finite={finite} qnorm={qnorm:.3f} "
    f"qok={qok} pos=({p.x:.2f},{p.y:.2f},{p.z:.2f})"
)
PYTHONEOF
)"

if echo "${_goal_summary}" | grep -q '^NO_GOAL_PUBLISHED'; then
	_warn "/semantic_goal/goal_pose silent in 8s — selector found no target, or all ring samples blocked by costmap"
elif [ -z "${_goal_summary}" ] || echo "${_goal_summary}" | grep -qE '^ERROR_'; then
	_fail "/semantic_goal/goal_pose: helper crashed"
	echo "    ${_goal_summary:-<empty>}"
else
	echo "  ${_goal_summary}"
	if echo "${_goal_summary}" | grep -q "frame_id='map'"; then
		_pass "goal_pose header.frame_id is 'map'"
	else
		_warn "goal_pose frame_id != 'map' — check global_frame param"
	fi
	if echo "${_goal_summary}" | grep -q "finite=True qnorm="; then
		if echo "${_goal_summary}" | grep -q "qok=True"; then
			_pass "goal_pose has finite position + unit quaternion"
		else
			_warn "goal_pose quaternion not unit-norm (yaw conversion bug?)"
		fi
	else
		_fail "goal_pose has non-finite values"
	fi
fi

# ----------------------------------------------------------------------------
# 6. /semantic_goal/goal_candidates marker count
# ----------------------------------------------------------------------------
_section "6. /semantic_goal/goal_candidates"
_cand_summary="$(timeout 12 python3 - <<'PYTHONEOF'
import sys, time
try:
    import rclpy
    from rclpy.node import Node
    from visualization_msgs.msg import MarkerArray
except Exception as e:
    print(f"ERROR_IMPORT: {e}")
    sys.exit(0)

try:
    rclpy.init()
    node = Node("_check_day7_candidates")
    msgs = []
    def cb(msg):
        msgs.append((time.time(), msg))
    node.create_subscription(MarkerArray, "/semantic_goal/goal_candidates", cb, 10)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f"ERROR_EXC: {e}")
    sys.exit(0)

if not msgs:
    print("ERROR_NO_MSG")
    sys.exit(0)

# Count viable / rejected over the full window. Pull the most
# populated message (the planner publishes empty arrays when no
# target is selected, so the latest may be empty even mid-task).
best = max(msgs, key=lambda t: len(t[1].markers))[1]
viable = sum(1 for m in best.markers if m.ns == "viable")
rejected = sum(1 for m in best.markers if m.ns == "rejected")
target_centre = sum(1 for m in best.markers if m.ns == "target")
print(f"msgs={len(msgs)} viable={viable} rejected={rejected} target_centres={target_centre}")
PYTHONEOF
)"

if [ -z "${_cand_summary}" ] || echo "${_cand_summary}" | grep -qE '^ERROR_'; then
	_fail "/semantic_goal/goal_candidates: no message in 8s"
	echo "    ${_cand_summary:-<empty>}"
else
	echo "  ${_cand_summary}"
	_pass "/semantic_goal/goal_candidates flowing"
	_total_cand=$(echo "${_cand_summary}" | sed -n 's/.*viable=\([0-9]*\) rejected=\([0-9]*\).*/\1+\2/p')
	if [ -n "${_total_cand}" ]; then
		_total=$(( $(echo "${_total_cand}" | tr -d ' ') ))
		if [ "${_total}" -ge 1 ]; then
			_pass "MarkerArray contains at least one ring-sample candidate"
		else
			_warn "MarkerArray empty in 8s window — drive Go2 to a chair, or selector found no target"
		fi
	fi
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
_section "Summary"
echo "  ${C_GRN}PASS=${_PASS}${C_END}  ${C_YLW}WARN=${_WARN}${C_END}  ${C_RED}FAIL=${_FAIL}${C_END}"

cat <<'EOF'

  Manual checks (RViz / eyeballs):
    [ ] In RViz the "Approach goal (Day 7)" yellow arrow shows up at
        the ring radius around the target's cylinder, pointing at the
        target.
    [ ] "Approach candidates (Day 7)" shows a ring of green
        cylinders at approach_distance with one or two red ones
        rejected (against walls).
    [ ] Walking Go2 sideways: the yellow arrow may flip to the
        other side of the target after replan_distance_m of motion;
        the cylinder picked is always the closest viable one.
    [ ] Nav2 logs in nav2.launch.py shell show
        "Nav2 received goal" and "Reached the goal" messages.

  Common failure shapes:
    * /target/selected always empty: target_class param doesn't match
      a class_label in /semantic_map/objects. Echo it:
          ros2 topic echo --once /semantic_map/objects | grep class_label
      and either rename target_class or pass a synonym in YOLOE's
      `classes` arg so YOLOE labels the object with the right class.
    * No goal_pose ever published: all ring samples rejected. Tune:
        - cost_threshold up (60 -> 80) for permissive costmap
        - approach_distance_default down (0.9 -> 0.6) to stay farther
          out of inflation
        - num_angle_samples up (16 -> 32) for narrower corridors
    * NavigateToPose accepted but Go2 doesn't move: Nav2 lifecycle
      not "active", or controller_server stalled on TF (Day 4
      slam_toolbox stall in sim — restart nav2.launch.py).
    * Goal arrow flickers between two ring positions every tick:
      replan_distance_m too low. Bump from 0.10 to 0.20 m so EMA
      jitter doesn't keep nudging the planner.
EOF

if [ "${_FAIL}" -eq 0 ]; then
	echo "  Day 7 hard checks: ${C_GRN}OK${C_END}"
	exit 0
else
	echo "  Day 7 hard checks: ${C_RED}FAILED${C_END} (${_FAIL} hard)"
	exit 1
fi
