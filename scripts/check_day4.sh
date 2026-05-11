#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Day 4 acceptance — Nav2 (AMCL + global/local costmap + planner +
# controller + behaviors) is up and capable of producing a plan from a
# goal pose. Six hard checks; one soft (recovery) is left for manual
# verification because it requires interaction.
#
# Run AFTER:
#   1. `bash scripts/run_warehouse_ros2.sh`             (sim)
#   2. `ros2 launch go2_bringup_sim chair_perception.launch.py`
#                                                       (TFs + /scan)
#   3. `ros2 launch go2_bringup_sim nav2.launch.py \
#                       map:=$PWD/maps/warehouse_v1.yaml`
#
# slam_toolbox MUST NOT be running — both AMCL and slam_toolbox publish
# `map → odom` and would fight for the transform.
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[check_day4] ERROR: 'ros2' not found." >&2
	exit 2
fi

if [ -t 1 ]; then
	C_RED=$'\e[31m'; C_GRN=$'\e[32m'; C_YLW=$'\e[33m'; C_BLD=$'\e[1m'; C_END=$'\e[0m'
else
	C_RED=""; C_GRN=""; C_YLW=""; C_BLD=""; C_END=""
fi

_FAIL_COUNT=0
_PASS_COUNT=0
_WARN_COUNT=0

_pass() { echo "  ${C_GRN}PASS${C_END} $*"; _PASS_COUNT=$((_PASS_COUNT + 1)); }
_fail() { echo "  ${C_RED}FAIL${C_END} $*"; _FAIL_COUNT=$((_FAIL_COUNT + 1)); }
_warn() { echo "  ${C_YLW}WARN${C_END} $*"; _WARN_COUNT=$((_WARN_COUNT + 1)); }
_section() { echo; echo "${C_BLD}== $* ==${C_END}"; }

# ----------------------------------------------------------------------------
# 1. Nav2 lifecycle nodes — all must be ACTIVE
# ----------------------------------------------------------------------------
_section "1. Nav2 lifecycle nodes ACTIVE"

# Nav2's lifecycle_manager wraps and activates these nodes. Each must
# report state == "active". `ros2 lifecycle get` returns lines like
# "active [3]" on success.
_check_lifecycle() {
	local node="$1"
	local state
	state="$(timeout 5 ros2 lifecycle get "/${node}" 2>/dev/null | head -n1)"
	# `ros2 lifecycle get` returns lines like "active [3]", "inactive [2]",
	# "unconfigured [1]". We need a WHOLE-WORD match on "active",
	# otherwise "inactive" matches and gets a false PASS.
	if echo "${state}" | grep -qE '^active([[:space:]]|$)'; then
		_pass "/${node}: ${state}"
	elif [ -z "${state}" ]; then
		_fail "/${node}: not running (or lifecycle service unavailable)"
	else
		_fail "/${node}: ${state} (expected 'active')"
	fi
}
# Nav2 Jazzy's navigation_launch.py spins up MORE servers than the
# 7-node minimal set we used initially. They all need to reach `active`
# for /navigate_to_pose to work. If any one fails to configure (e.g.
# collision_monitor without observation_sources in params.yaml), the
# lifecycle manager aborts and ALL of them stay in `inactive`.
#
# Nav-side servers — these are always present regardless of backend.
for n in bt_navigator controller_server planner_server \
         behavior_server velocity_smoother \
         smoother_server waypoint_follower collision_monitor \
         docking_server route_server; do
	_check_lifecycle "${n}"
done

# Localization backend: either AMCL+map_server (slam:=False) OR
# slam_toolbox (slam:=True). Detect which one is running and check
# accordingly. Running both at once would be a misconfiguration.
_NODES_RAW="$(timeout 4 ros2 node list 2>/dev/null | sort || true)"
_has_amcl=0
_has_slam=0
echo "${_NODES_RAW}" | grep -q '^/amcl$' && _has_amcl=1
echo "${_NODES_RAW}" | grep -q '^/slam_toolbox$' && _has_slam=1

if [ "${_has_amcl}" -eq 1 ] && [ "${_has_slam}" -eq 1 ]; then
	_fail "BOTH amcl and slam_toolbox are running — they will fight over map→odom. Pick one."
elif [ "${_has_amcl}" -eq 1 ]; then
	echo "  (localization backend: amcl + map_server)"
	_check_lifecycle amcl
	_check_lifecycle map_server
elif [ "${_has_slam}" -eq 1 ]; then
	echo "  (localization backend: slam_toolbox in mapping mode)"
	_check_lifecycle slam_toolbox
else
	_fail "No localization backend running (need either /amcl + /map_server or /slam_toolbox)"
fi

# ----------------------------------------------------------------------------
# 2. Topics the user / RViz / sim need
# ----------------------------------------------------------------------------
_section "2. Required topics"
_TOPICS_RAW="$(timeout 5 ros2 topic list 2>/dev/null || true)"
_topic_present() { echo "${_TOPICS_RAW}" | grep -Fxq "$1"; }

# Topics that exist regardless of localization backend.
for t in /map /global_costmap/costmap /local_costmap/costmap \
         /plan /cmd_vel; do
	if _topic_present "${t}"; then _pass "${t}"; else _fail "missing: ${t}"; fi
done

# AMCL-only topics. slam_toolbox doesn't expose /particle_cloud or
# /amcl_pose, so only require them when the AMCL backend is the one
# active (detected via /amcl in node list above).
if [ "${_has_amcl:-0}" -eq 1 ]; then
	for t in /particle_cloud /amcl_pose; do
		if _topic_present "${t}"; then _pass "${t}"; else _fail "missing: ${t}"; fi
	done
else
	echo "  (skipped /particle_cloud /amcl_pose — slam_toolbox backend)"
fi

# Critical: a topic existing in `topic list` only proves SOME node has
# bound the name, NOT that anyone is publishing. After a sim restart
# without nav2 restart, costmap topics still appear in topic list (the
# global_costmap node hasn't crashed) but the publisher count drops to
# 0 because the costmap is stuck on stale sim_time and refuses to
# publish updates. Validate publisher counts on the costmaps and /map
# directly — these are the data-flow choke points planner_server
# depends on.
_section "2b. Costmap data flow (sim/nav2 sim_time sync)"
_check_pub_count() {
	local topic="$1"
	local min_pubs="$2"
	local info
	info="$(timeout 4 ros2 topic info "${topic}" 2>/dev/null || true)"
	local pubs
	pubs="$(echo "${info}" | awk '/^Publisher count:/ {print $3; exit}')"
	if [ "${pubs:-0}" -ge "${min_pubs}" ] 2>/dev/null; then
		_pass "${topic} has ${pubs} publisher(s)"
	else
		_fail "${topic} has ${pubs:-0} publishers (expected ≥ ${min_pubs}) — sim & nav2 sim_time desync? Restart nav2 AFTER sim."
	fi
}
_check_pub_count /map 1
_check_pub_count /global_costmap/costmap 1
_check_pub_count /local_costmap/costmap 1

# /scan must actually flow — `topic info` showing a publisher is not
# enough because pointcloud_to_laserscan silently drops every cloud
# when its target_frame TF lookup fails (sim stamp out of TF cache,
# or PCL2 frame_id ≠ target_frame). If /scan is dead, AMCL never
# updates and never publishes map→odom; everything downstream then
# aborts the moment a goal is sent (BT halts on TF lookup before
# planner runs, error_code=0).
_section "2c. /scan data flow (AMCL gating)"
_scan_summary="$(timeout 12 python3 - <<'PYTHONEOF'
import time, sys
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    )
    from sensor_msgs.msg import LaserScan
except Exception as e:
    print(f"ERROR_IMPORT: {e}")
    sys.exit(0)

try:
    rclpy.init()
    node = Node("_check_day4_scan")
    # Explicit QoS to match pointcloud_to_laserscan publisher:
    # BEST_EFFORT + VOLATILE + KEEP_LAST. Set all four fields to
    # avoid relying on rclpy default-profile interactions across
    # ROS distros, which silently drop the subscription on Jazzy
    # in some test configurations.
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    got = []
    def cb(msg):
        got.append(time.time())
    node.create_subscription(LaserScan, "/scan", cb, qos)
    # Give DDS discovery + matching ~1 s before counting.
    deadline = time.time() + 8.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    n = len(got)
    if n == 0:
        print("ERROR_NO_MSG")
    else:
        dt = got[-1] - got[0] if n > 1 else 1.0
        hz = (n - 1) / dt if dt > 0 else 0.0
        print(f"count={n} hz={hz:.2f}")
except Exception as e:
    print(f"ERROR_EXC: {e}")
PYTHONEOF
)"
if [ -z "${_scan_summary}" ] || echo "${_scan_summary}" | grep -q ERROR_NO_MSG; then
	_fail "/scan: 0 messages in 6s — pointcloud_to_laserscan is dropping every input. Check /lidar/points frame_id matches target_frame, or bump transform_tolerance."
else
	echo "  ${_scan_summary}"
	_hz=$(echo "${_scan_summary}" | sed -n 's/.*hz=\([0-9.]*\).*/\1/p')
	if awk -v h="${_hz:-0}" 'BEGIN { exit (h+0 > 1.5) ? 0 : 1 }'; then
		_pass "/scan flowing at ~${_hz} Hz"
	else
		_warn "/scan at ${_hz} Hz — too slow for AMCL update_min_d (default 0.25 m)"
	fi
fi

# ----------------------------------------------------------------------------
# 3. /cmd_vel routing — Nav2 → sim consumer
# ----------------------------------------------------------------------------
_section "3. /cmd_vel routing"
# We need at least one Nav2-side publisher AND the sim-side subscriber.
# Common publishers: velocity_smoother (preferred) or controller_server.
_cmd_info="$(timeout 4 ros2 topic info /cmd_vel -v 2>/dev/null || true)"
_pub_count="$(echo "${_cmd_info}" | awk '/^Publisher count:/ {print $3; exit}')"
_sub_count="$(echo "${_cmd_info}" | awk '/^Subscription count:/ {print $3; exit}')"
if [ "${_pub_count:-0}" -ge 1 ] 2>/dev/null; then
	_pub_node="$(echo "${_cmd_info}" | awk '/^Endpoint type: PUBLISHER$/ {f=1} f && /^Node name:/ {print $3; f=0; exit}')"
	_pass "/cmd_vel publishers: ${_pub_count} (e.g. ${_pub_node})"
else
	_fail "/cmd_vel has 0 publishers — Nav2 is not driving cmd_vel"
fi
if [ "${_sub_count:-0}" -ge 1 ] 2>/dev/null; then
	_pass "/cmd_vel subscribers: ${_sub_count} (sim should be one of them)"
else
	_fail "/cmd_vel has 0 subscribers — sim's SubTwist isn't bound. Restart sim."
fi

# ----------------------------------------------------------------------------
# 4. AMCL has converged (particle spread reasonable) — AMCL backend only
# ----------------------------------------------------------------------------
_section "4. AMCL convergence"
if [ "${_has_amcl:-0}" -ne 1 ]; then
	echo "  (skipped — slam_toolbox backend; localization quality is verified by the TF chain check below)"
else
# Use Python helper to subscribe to /amcl_pose once and read the
# 6×6 covariance matrix's xx and yy entries. Sane converged AMCL has
# σ_x, σ_y < 0.5 m (i.e. cov < 0.25).
_amcl_summary="$(timeout 12 python3 - <<'PYTHONEOF' 2>/dev/null
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
node = Node("_check_day4_amcl")
qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.RELIABLE
got = [None]
def cb(msg): got[0] = msg
node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", cb, qos)
start = time.time()
while got[0] is None and time.time() - start < 8.0:
    rclpy.spin_once(node, timeout_sec=0.25)
node.destroy_node()
rclpy.shutdown()
if got[0] is None:
    print("ERROR_NO_MSG")
else:
    p = got[0].pose
    cov = p.covariance  # 36 floats, row-major 6x6
    sxx, syy, syaw = cov[0], cov[7], cov[35]
    sx = math.sqrt(max(0.0, sxx))
    sy = math.sqrt(max(0.0, syy))
    syaw_deg = math.degrees(math.sqrt(max(0.0, syaw)))
    print(f"x={p.pose.position.x:.2f} y={p.pose.position.y:.2f} "
          f"sigma_x={sx:.3f} sigma_y={sy:.3f} sigma_yaw_deg={syaw_deg:.1f}")
PYTHONEOF
)"
if [ -z "${_amcl_summary}" ] || echo "${_amcl_summary}" | grep -q ERROR_NO_MSG; then
	# AMCL only republishes /amcl_pose when Go2 moves > update_min_d
	# (default 25 cm) or rotates > update_min_a (default ~11°). At
	# rest, the only /amcl_pose is the one emitted at activation —
	# which is gone from the topic by the time check_day4 runs. So
	# treat "no message" as a soft warning, not a hard fail; the
	# lifecycle_active checks above already prove AMCL is alive.
	_warn "/amcl_pose: no message in 8s. Normal at rest — AMCL only republishes after motion. Drive Go2 a bit (or click '2D Pose Estimate' in RViz) and re-check."
else
	echo "  pose: ${_amcl_summary}"
	# Pull sigma_x, sigma_y, sigma_yaw_deg
	_sx=$(echo "${_amcl_summary}" | sed -n 's/.*sigma_x=\([0-9.]*\).*/\1/p')
	_sy=$(echo "${_amcl_summary}" | sed -n 's/.*sigma_y=\([0-9.]*\).*/\1/p')
	_syaw=$(echo "${_amcl_summary}" | sed -n 's/.*sigma_yaw_deg=\([0-9.]*\).*/\1/p')
	if awk -v x="${_sx:-99}" -v y="${_sy:-99}" \
	    'BEGIN { exit (x+0 < 0.5 && y+0 < 0.5) ? 0 : 1 }'; then
		_pass "AMCL converged (σ_xy ≈ ${_sx}, ${_sy} m)"
	else
		_fail "AMCL spread too large σ_xy=(${_sx}, ${_sy}) m — drive Go2 a bit OR re-set 2D Pose Estimate in RViz"
	fi
	if awk -v y="${_syaw:-999}" 'BEGIN { exit (y+0 < 30.0) ? 0 : 1 }'; then
		_pass "AMCL yaw spread σ_yaw ≈ ${_syaw}° (< 30°)"
	else
		_warn "AMCL yaw spread σ_yaw ≈ ${_syaw}° large — add a forward+turn motion to disambiguate"
	fi
fi
fi  # end of: if AMCL backend

# ----------------------------------------------------------------------------
# 5. TF chain  map → odom (from AMCL or slam_toolbox) and the rest
# ----------------------------------------------------------------------------
_section "5. TF chain"
_check_tf() {
	local parent="$1" child="$2"
	local logf="/tmp/_check_day4_tf.log"
	timeout 6 ros2 run tf2_ros tf2_echo "${parent}" "${child}" \
	    >"${logf}" 2>&1 || true
	if grep -qE 'Translation:|At time' "${logf}"; then
		local tline
		tline="$(grep -m1 'Translation:' "${logf}" || true)"
		_pass "tf2 lookup: ${parent} → ${child}  ${tline}"
	else
		_fail "tf2 lookup FAILED: ${parent} → ${child}"
		echo "    tf2_echo log tail:"
		tail -n 5 "${logf}" | sed 's/^/      /'
	fi
}
_check_tf map         odom
_check_tf odom        base_link
_check_tf base_link   lidar_link

# ----------------------------------------------------------------------------
# 6. Static map content — make sure map_server actually loaded the .pgm
# ----------------------------------------------------------------------------
_section "6. /map content (from map_server, not slam_toolbox)"
# Same Python helper trick as Day 3 — works around topic echo's array
# truncation on big OccupancyGrids.
_map_summary="$(timeout 12 python3 - <<'PYTHONEOF' 2>/dev/null
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = Node("_check_day4_map")
qos = QoSProfile(depth=1)
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
qos.reliability = ReliabilityPolicy.RELIABLE
got = [None]
def cb(msg): got[0] = msg
node.create_subscription(OccupancyGrid, "/map", cb, qos)
start = time.time()
while got[0] is None and time.time() - start < 8.0:
    rclpy.spin_once(node, timeout_sec=0.25)
node.destroy_node()
rclpy.shutdown()
if got[0] is None:
    print("ERROR_NO_MSG")
else:
    arr = got[0].data
    n = len(arr)
    unk = sum(1 for v in arr if v == -1)
    free = sum(1 for v in arr if v == 0)
    occ = sum(1 for v in arr if v >= 50)
    info = got[0].info
    print(f"w={info.width} h={info.height} res={info.resolution:.3f} "
          f"n={n} unk={unk} free={free} occ={occ}")
PYTHONEOF
)"
if [ -z "${_map_summary}" ] || echo "${_map_summary}" | grep -q ERROR_NO_MSG; then
	_fail "/map: no OccupancyGrid in 8s — map_server failed to load yaml?"
else
	echo "  ${_map_summary}"
	_occ=$(echo "${_map_summary}" | sed -n 's/.*occ=\([0-9]*\).*/\1/p')
	if [ "${_occ:-0}" -gt 50 ] 2>/dev/null; then
		_pass "/map has occupied cells (${_occ}) — static map loaded correctly"
	else
		_fail "/map has ${_occ:-0} occupied cells — yaml path correct?"
	fi
fi

# ----------------------------------------------------------------------------
# 7. Goal-pose action server is reachable
# ----------------------------------------------------------------------------
_section "7. NavigateToPose action server"
# Check that bt_navigator's /navigate_to_pose action server exists.
# We don't actually send a goal here — that requires user to also
# observe Go2 moving in RViz, which is a manual step.
_actions="$(timeout 4 ros2 action list 2>/dev/null || true)"
if echo "${_actions}" | grep -q '^/navigate_to_pose$'; then
	_pass "/navigate_to_pose action server present"
else
	_fail "/navigate_to_pose NOT advertised — bt_navigator not active"
fi
if echo "${_actions}" | grep -q '^/navigate_through_poses$'; then
	_pass "/navigate_through_poses action server present"
else
	_warn "/navigate_through_poses NOT advertised (cosmetic for Day 4)"
fi

# ----------------------------------------------------------------------------
# Summary + manual checks reminder
# ----------------------------------------------------------------------------
_section "Summary"
echo "  ${C_GRN}PASS=${_PASS_COUNT}${C_END}  ${C_YLW}WARN=${_WARN_COUNT}${C_END}  ${C_RED}FAIL=${_FAIL_COUNT}${C_END}"

cat <<'EOF'

  Manual hard checks (require RViz or eyeballs — script can't verify):
    [ ] AMCL particle cloud in RViz looks like a tight cluster (< 0.3 m)
    [ ] /global_costmap aligns visually with the static map (no offset)
    [ ] Click "2D Goal Pose" in RViz → green /plan appears within 2s
    [ ] Go2 actually drives along the plan and stops within 0.25 m of goal
    [ ] Three scenarios pass: open straight line, around-obstacle, U-turn

  If hard checks above all PASS but Go2 won't move when you click Goal:
    1. ros2 topic echo /cmd_vel  (Nav2 should be sending Twist messages)
    2. ros2 topic info /cmd_vel  (should have 1 publisher + 1 subscriber)
    3. If publisher is velocity_smoother and subscriber is missing, the
       sim's SubTwist node didn't connect — restart sim.
EOF

if [ "${_FAIL_COUNT}" -eq 0 ]; then
	echo "  Day 4 hard checks: ${C_GRN}OK${C_END}"
	exit 0
else
	echo "  Day 4 hard checks: ${C_RED}FAILED${C_END} (${_FAIL_COUNT} hard)"
	exit 1
fi
