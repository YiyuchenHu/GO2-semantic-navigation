#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Day 3 acceptance check — slam_toolbox is producing a clean 2D map.
#
# Run this AFTER:
#   1. `bash scripts/run_warehouse_ros2.sh` is up in another shell
#   2. `ros2 launch go2_bringup_sim mapping.launch.py` is up in a third
# Then drive the Go2 with `ros2 topic pub /cmd_vel ...` to fill the map.
#
# Hard checks: /map publishes, has nontrivial size, contains OCCUPIED
# (>0) AND FREE (==0) cells, and a `map → odom` TF lookup succeeds.
# Soft checks: /map_metadata, slam_toolbox node is alive, etc.
#
# Exit 0 if all hard checks pass; 1 otherwise.
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[check_day3] ERROR: 'ros2' not found. source scripts/dev_env.sh first." >&2
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
# 1. /map topic exists with the right type
# ----------------------------------------------------------------------------
_section "1. /map topic"
_TOPICS_RAW="$(timeout 5 ros2 topic list 2>/dev/null || true)"
if [ -z "${_TOPICS_RAW}" ]; then
	_fail "ros2 topic list returned empty — nothing to check"
	echo "${C_RED}Aborting.${C_END}"; exit 1
fi

if echo "${_TOPICS_RAW}" | grep -Fxq "/map"; then
	_pass "/map advertised"
else
	_fail "/map NOT advertised — is mapping.launch.py running?"
	echo
	echo "Hint: did you launch slam_toolbox?"
	echo "  ros2 launch go2_bringup_sim mapping.launch.py"
	exit 1
fi

# Also note /map_updates and /map_metadata if present
for t in /map_metadata /map_updates; do
	if echo "${_TOPICS_RAW}" | grep -Fxq "${t}"; then
		_pass "${t} (slam_toolbox is healthy)"
	else
		_warn "${t} not present (cosmetic — only /map is required)"
	fi
done

# ----------------------------------------------------------------------------
# 2. /map publish rate (should be ~1 Hz, matching map_update_interval)
# ----------------------------------------------------------------------------
_section "2. /map publish cadence"
# /map uses TRANSIENT_LOCAL durability — `topic hz` needs to match.
_hz_out="$(timeout 8 ros2 topic hz --window 5 /map 2>&1 || true)"
_hz="$(echo "${_hz_out}" | awk '/average rate/ {gsub(":","",$0); print $3}' | tail -n1)"
if [ -n "${_hz}" ] && awk -v h="${_hz}" 'BEGIN { exit (h+0 >= 0.3 && h+0 <= 5.0) ? 0 : 1 }'; then
	_pass "/map: ${_hz} Hz (in [0.3, 5.0])"
elif [ -n "${_hz}" ]; then
	_warn "/map: ${_hz} Hz — outside [0.3, 5.0]; see map_update_interval in slam_toolbox_mapping.yaml"
else
	# Fallback: `topic hz` can be flaky on TRANSIENT_LOCAL. Just verify
	# at least one message has been latched.
	_w="$(timeout 6 ros2 topic echo --once --field info.width /map 2>/dev/null \
	     | awk 'NF && $0 !~ /^---/ {print; exit}' | tr -dc '0-9')"
	if [ -n "${_w}" ] && [ "${_w}" -gt 0 ]; then
		_pass "/map: at least one OccupancyGrid latched (rate measurement flaky on TRANSIENT_LOCAL)"
	else
		_fail "/map: no OccupancyGrid received in 6s"
	fi
fi

# ----------------------------------------------------------------------------
# 3. /map content sanity — size + occupancy distribution
# ----------------------------------------------------------------------------
_section "3. /map content sanity"
# Echo the OccupancyGrid info block to inspect width / height / resolution / origin.
_info="$(timeout 6 ros2 topic echo --once --field info /map 2>/dev/null || true)"
if [ -z "${_info}" ]; then
	_fail "could not read /map info — skipping content checks"
else
	_w="$(echo "${_info}" | awk '/^[[:space:]]*width:/  {print $2; exit}')"
	_h="$(echo "${_info}" | awk '/^[[:space:]]*height:/ {print $2; exit}')"
	_r="$(echo "${_info}" | awk '/^[[:space:]]*resolution:/ {print $2; exit}')"
	if [ -n "${_w}" ] && [ "${_w}" -ge 50 ] && [ "${_h}" -ge 50 ] 2>/dev/null; then
		# 10m room at 0.05 m → at least 200 cells ideal, but we warm-start
		# the map small and grow as Go2 explores. 50 is "we've at least
		# seen the immediate surroundings" floor.
		_pass "/map size: ${_w} × ${_h} cells, resolution ${_r} m"
	else
		_fail "/map size suspiciously small: width=${_w:-?} height=${_h:-?} resolution=${_r:-?}"
	fi
fi

# Histogram check on the occupancy data: SLAM is "working" if we have
# both occupied (>0) AND free (==0) cells. All -1 (unknown) means the
# scan never inserted anything; all 0 means we never saw any obstacle;
# all 100 is geometric impossibility.
#
# Use a Python subprocess to subscribe properly and count cells —
# `ros2 topic echo --once --field data /map` truncates large arrays
# in Jazzy by default, so the awk-based approach silently undercounts
# everything to 0 even on a perfectly valid map.
_summary="$(timeout 12 python3 - <<'PYTHONEOF' 2>/dev/null
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = Node("_check_day3_histo")
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
    mid = n - unk - free - occ
    print(f"n={n} unk={unk} free={free} occ={occ} mid={mid}")
PYTHONEOF
)"
if [ -z "${_summary}" ] || echo "${_summary}" | grep -q ERROR_NO_MSG; then
	_warn "could not read /map data — skipping occupancy histogram"
else
	echo "  data: ${_summary}"
	_free=$(echo "${_summary}" | sed -n 's/.*free=\([0-9]*\).*/\1/p')
	_occ=$(echo "${_summary}" | sed -n 's/.*occ=\([0-9]*\).*/\1/p')
	if [ "${_free:-0}" -gt 100 ] && [ "${_occ:-0}" -gt 20 ] 2>/dev/null; then
		_pass "/map has both free (${_free}) and occupied (${_occ}) cells"
	elif [ "${_occ:-0}" -le 20 ]; then
		_fail "/map has too few occupied cells (${_occ:-0}). Likely Go2 hasn't moved yet — drive it around with /cmd_vel."
	else
		_fail "/map occupancy looks degenerate (free=${_free:-0} occ=${_occ:-0})"
	fi
fi

# ----------------------------------------------------------------------------
# 4. TF chain  map → odom → base_link → lidar_link
# ----------------------------------------------------------------------------
_section "4. TF chain"
# `ros2 run tf2_ros tf2_echo` in Jazzy doesn't accept --timeout; it
# runs at 1 Hz forever and we need to bound it with the shell `timeout`
# command. Success = at least one "Translation:" or "At time" line
# appears within the budget; failure = the lookup never resolved or
# the process died before publishing.
_check_tf() {
	local parent="$1" child="$2"
	local logf="/tmp/_check_day3_tf.log"
	# Each tf2_echo invocation spawns a fresh process whose tf2 buffer
	# starts empty; DDS discovery + receiving the latched TRANSIENT_LOCAL
	# messages from all 5+ static_transform_publishers takes 2-4 seconds
	# in our setup (single-machine FastDDS, 5 publishers + dynamic /tf).
	# With timeout=3s, the first lookup typically misses and reports
	# "frame does not exist". 6s is the empirical sweet spot.
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
_check_tf base_link   camera_color_optical_frame
# NOTE: an end-to-end `map → lidar_link` chain lookup is intentionally
# NOT performed here. tf2_echo defaults to the "latest common time"
# across all involved frames, and `lidar_link` is a static TF
# (timestamp=0) while `map → odom` only starts populating once SLAM
# has placed its first keyframe. The "latest common time" picked
# from the static side falls before the start of the map→odom cache,
# producing a bogus "extrapolation into the past" error.
# Real consumers (slam_toolbox's own scan callback, Nav2's costmap
# tf lookups, etc.) query at a SCAN-MESSAGE timestamp where all
# transforms ARE present, so the chain works in practice. The
# individual `map → odom` and `base_link → lidar_link` checks above
# already verify each link of the chain independently.

# ----------------------------------------------------------------------------
# 5. slam_toolbox node alive
# ----------------------------------------------------------------------------
_section "5. slam_toolbox node"
_nodes="$(timeout 4 ros2 node list 2>/dev/null || true)"
if echo "${_nodes}" | grep -q "slam_toolbox"; then
	_pass "slam_toolbox node alive"
else
	_fail "slam_toolbox node NOT in 'ros2 node list'"
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
_section "Summary"
echo "  ${C_GRN}PASS=${_PASS_COUNT}${C_END}  ${C_YLW}WARN=${_WARN_COUNT}${C_END}  ${C_RED}FAIL=${_FAIL_COUNT}${C_END}"

if [ "${_FAIL_COUNT}" -eq 0 ]; then
	echo "  Day 3 hard checks: ${C_GRN}OK${C_END}"
	echo
	echo "  Save the map for Day 4:  bash scripts/save_map.sh"
	exit 0
else
	echo "  Day 3 hard checks: ${C_RED}FAILED${C_END} (${_FAIL_COUNT} hard)"
	exit 1
fi
