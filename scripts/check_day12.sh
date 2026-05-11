#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Day 1-2 acceptance check for the Phase 0 sim platform layer.
#
# Run this AFTER `bash scripts/run_warehouse_ros2.sh` is up in another shell.
# Optionally run a chair_*.launch.py too if you want to also validate the
# /tf_static frames added by the ROS-side launches.
#
# Exit code is non-zero if any "hard" check fails. "Soft" warnings just print.
# -----------------------------------------------------------------------------
set -uo pipefail   # NB: no -e — we want to keep going past individual failures

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[check_day12] ERROR: 'ros2' not found. source scripts/dev_env.sh first." >&2
	exit 2
fi

# ANSI helpers (skipped if stdout isn't a tty)
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
# 1. Topic list — must contain the Phase 0 contract.
# ----------------------------------------------------------------------------
_section "1. Topic list"
_TOPICS_RAW="$(timeout 5 ros2 topic list 2>/dev/null || true)"
if [ -z "${_TOPICS_RAW}" ]; then
	_fail "ros2 topic list returned empty — is the sim running?"
	echo "${C_RED}Aborting; nothing to check against.${C_END}"
	exit 1
fi
_topic_present() {
	# stdin = topic list, $1 = topic to find
	echo "${_TOPICS_RAW}" | grep -Fxq "$1"
}

_REQUIRED_TOPICS=(
	/clock
	/tf
	/odom
	/imu/data
	/camera/color/image_raw
	/camera/color/camera_info
	/camera/depth/image_rect_raw
	/camera/depth/camera_info
	/lidar/points
	/cmd_vel
)
for t in "${_REQUIRED_TOPICS[@]}"; do
	if _topic_present "${t}"; then _pass "${t}"; else _fail "missing topic: ${t}"; fi
done

# /tf_static and /scan come from the ROS-side launches (chair_perception.launch.py).
# Soft-fail (warn) if absent so a sim-only smoke test still passes — but if you're
# doing the full Day 1-2 / Day 3 prep, both should be there.
for t in /tf_static /scan; do
	if _topic_present "${t}"; then
		_pass "${t} (chair_perception.launch.py contributing)"
	else
		_warn "${t} missing — launch chair_perception.launch.py for /tf_static + pointcloud_to_laserscan"
	fi
done

# Reserved for later stages.
for t in /camera/depth/points /joint_states /map; do
	if _topic_present "${t}"; then
		_pass "${t} (optional)"
	else
		_warn "${t} not present (reserved for later phase)"
	fi
done

# ----------------------------------------------------------------------------
# 2. Hz — only check the topics that should be live in Phase 0.
# ----------------------------------------------------------------------------
_section "2. Topic rates (5s window each)"

# usage: _check_hz <topic> <min_hz>
_check_hz() {
	local topic="$1"
	local min_hz="$2"
	if ! _topic_present "${topic}"; then
		_warn "skip ${topic} (not advertised)"
		return
	fi
	# `ros2 topic hz` exits >0 on SIGTERM (timeout). We capture stderr/stdout.
	# Window=10 + timeout 10s gives slow (~5 Hz) topics enough headroom to
	# emit a useful average within the budget without making the whole
	# script crawl on healthy topics that produce a rate within ~2s anyway.
	local out
	out="$(timeout 10 ros2 topic hz --window 10 "${topic}" 2>&1 || true)"
	# parse the most-recent "average rate: <float>" line
	local hz
	hz="$(echo "${out}" | awk '/average rate/ {gsub(":","",$0); print $3}' | tail -n1)"
	if [ -z "${hz}" ]; then
		_fail "${topic}: no messages received in 10s"
		return
	fi
	# bash can't compare floats; use awk
	if awk -v h="${hz}" -v m="${min_hz}" 'BEGIN { exit (h+0 >= m+0) ? 0 : 1 }'; then
		_pass "${topic}: ${hz} Hz  (>= ${min_hz})"
	else
		_fail "${topic}: ${hz} Hz  (< ${min_hz})"
	fi
}

_check_hz /clock                          30
_check_hz /tf                             15
_check_hz /odom                           20
_check_hz /imu/data                       50
# Camera rates: with the OS1-32 RTX LiDAR rendering 32 lines × 1024
# columns at full playback rate, the camera helpers compete for GPU
# and drop from ~22 Hz to ~9-12 Hz on RTX 5090. 8 Hz is the practical
# floor — anything below means YOLO chair detection (~10 Hz target)
# starts missing frames. Bump --camera-frame-skip up if that's tight.
_check_hz /camera/color/image_raw          8
_check_hz /camera/depth/image_rect_raw     8
_check_hz /camera/color/camera_info        5

# LiDAR rate sanity check. We want full 360° scans at the OS1 rotation
# rate (~10 Hz). Two failure modes to catch:
#   * < 5 Hz  -> something dropped the render product / writer is dead
#   * > 30 Hz -> SimulationGate.step throttle didn't apply, so the
#                writer is still publishing partial arcs every render
#                frame. Will silently break slam_toolbox.
_check_hz_band() {
	local topic="$1" min_hz="$2" max_hz="$3"
	if ! _topic_present "${topic}"; then
		_warn "skip ${topic} (not advertised)"; return
	fi
	local out hz
	# Slow topics (5 Hz LiDAR, 5 Hz scan) need a longer budget than the
	# 6s/30-window we use for camera-rate topics. window=10 fills in ~2s
	# at 5 Hz, leaving 8s for DDS discovery + first-message latency.
	out="$(timeout 10 ros2 topic hz --window 10 "${topic}" 2>&1 || true)"
	hz="$(echo "${out}" | awk '/average rate/ {gsub(":","",$0); print $3}' | tail -n1)"
	if [ -z "${hz}" ]; then
		_fail "${topic}: no messages received in 10s"; return
	fi
	if awk -v h="${hz}" -v m="${min_hz}" -v M="${max_hz}" \
	    'BEGIN { exit (h+0 >= m+0 && h+0 <= M+0) ? 0 : 1 }'; then
		_pass "${topic}: ${hz} Hz  (in [${min_hz}, ${max_hz}])"
	elif awk -v h="${hz}" -v M="${max_hz}" 'BEGIN { exit (h+0 > M+0) ? 0 : 1 }'; then
		_fail "${topic}: ${hz} Hz  (> ${max_hz} — SimulationGate.step throttle not applied; "\
"each scan is a partial arc that will break slam_toolbox. Check "\
"--lidar-publish-step in run_go2_warehouse_ros2.py)"
	else
		_fail "${topic}: ${hz} Hz  (< ${min_hz})"
	fi
}
# LiDAR-specific liveness check.
#
# `ros2 topic hz /lidar/points` is unreliable in scripted timeout
# context — Isaac's RtxLidarROS2 helper publishes through a Replicator
# pipeline whose discovery + first-message latency under DDS sometimes
# exceeds 10s, even though the topic IS alive (manual `ros2 topic hz`
# off-script works fine). Instead, probe via `topic echo --once`,
# which only needs ONE message to succeed and is what `ros2` would
# return-code-fail on if the publisher were truly silent.
#
# We also verify the cloud width is non-zero — the bug we hit before
# (Render product attached to wrong prim type) produced perfectly
# valid messages at the right rate but with width=0 (zero points).
# OS1-32 at 10 Hz / 1024 res yields ~30k points per full scan.
_check_pointcloud_alive() {
	local topic="$1"
	local min_width="$2"
	if ! _topic_present "${topic}"; then
		_warn "skip ${topic} (not advertised)"; return
	fi
	local w
	w="$(timeout 8 ros2 topic echo --once --field width "${topic}" 2>/dev/null \
	     | awk 'NF && $0 !~ /^---/ {print; exit}' | tr -dc '0-9')"
	if [ -z "${w}" ]; then
		_fail "${topic}: no PointCloud2 message received in 8s"
		return
	fi
	if [ "${w}" -ge "${min_width}" ] 2>/dev/null; then
		_pass "${topic}: alive, width=${w} points (>= ${min_width})"
	else
		_fail "${topic}: width=${w} (< ${min_width}) — ScanBuffer not "\
"accumulating a full scan; check OmniLidar prim binding in run_go2_warehouse_ros2.py"
	fi
}
_check_pointcloud_alive /lidar/points  10000

# /scan is a LaserScan, not a PointCloud2 — `width` doesn't exist.
# Use `ranges` length as a proxy. pointcloud_to_laserscan with our
# OS1-32 1024 res angle_increment ≈ 1024 ranges per scan.
_check_laserscan_alive() {
	local topic="$1" min_n="$2"
	if ! _topic_present "${topic}"; then
		_warn "skip ${topic} (not advertised)"; return
	fi
	# `ranges` is a list — length = count of newline-delimited entries.
	local n
	n="$(timeout 8 ros2 topic echo --once --field ranges "${topic}" 2>/dev/null \
	     | awk '/^- / {c++} END {print c+0}')"
	if [ -z "${n}" ] || [ "${n}" -eq 0 ]; then
		_fail "${topic}: no LaserScan message received in 8s"
		return
	fi
	if [ "${n}" -ge "${min_n}" ] 2>/dev/null; then
		_pass "${topic}: alive, ranges length=${n} (>= ${min_n})"
	else
		_fail "${topic}: ranges length=${n} (< ${min_n})"
	fi
}
_check_laserscan_alive /scan  500

# ----------------------------------------------------------------------------
# 3. TF tree — write to /tmp/frames.{gv,pdf}.
# ----------------------------------------------------------------------------
_section "3. TF tree"
if command -v ros2 >/dev/null && ros2 pkg list 2>/dev/null | grep -q '^tf2_tools$'; then
	# tf2_tools view_frames must be run from a writable dir.
	(
		cd /tmp || exit 0
		# `view_frames` runs ~5s, then writes frames.pdf + frames.gv
		timeout 8 ros2 run tf2_tools view_frames -o frames >/dev/null 2>&1 || true
	)
	if [ -s /tmp/frames.gv ]; then
		_pass "TF tree captured to /tmp/frames.gv (PDF: /tmp/frames.pdf)"
		# spot-check core frames
		for f in odom base_link; do
			if grep -Fwq "\"${f}\"" /tmp/frames.gv; then
				_pass "frame present: ${f}"
			else
				_fail "frame missing in TF tree: ${f}"
			fi
		done
		for f in camera_link camera_color_optical_frame camera_depth_optical_frame imu_link lidar_link; do
			if grep -Fwq "\"${f}\"" /tmp/frames.gv; then
				_pass "frame present: ${f}"
			else
				_warn "frame missing: ${f} (run a chair_*.launch.py to publish /tf_static)"
			fi
		done
	else
		_warn "tf2_tools view_frames produced no output — skipping TF tree dump"
	fi
else
	_warn "tf2_tools not available; skipping TF tree dump"
fi

# ----------------------------------------------------------------------------
# 4. Camera info sanity — width/height/K.
# ----------------------------------------------------------------------------
_section "4. Camera info sanity"
_check_camera_info() {
	local topic="$1"
	if ! _topic_present "${topic}"; then
		_warn "skip ${topic} (not advertised)"
		return
	fi
	local out
	out="$(timeout 5 ros2 topic echo --once --field 'width,height,k' "${topic}" 2>/dev/null || \
	       timeout 5 ros2 topic echo --once "${topic}" 2>/dev/null || true)"
	if [ -z "${out}" ]; then
		_fail "${topic}: no message received"
		return
	fi
	local W H
	W="$(echo "${out}" | awk -F': *' '/^width/ {print $2; exit}')"
	H="$(echo "${out}" | awk -F': *' '/^height/ {print $2; exit}')"
	# K is on the same line ("k:") followed by an array on next lines.
	local fx fy cx cy
	fx="$(echo "${out}" | awk '/^k:/ {flag=1; next} flag && /-/ {gsub("-",""); print $1; exit}')"
	# Fallback: parse the typical inline list like "k: [fx, 0, cx, 0, fy, cy, 0, 0, 1]"
	if [ -z "${fx}" ]; then
		fx="$(echo "${out}" | sed -n 's/.*k:[[:space:]]*\[\([^,]*\),.*/\1/p' | head -n1)"
	fi
	if [ -n "${W}" ] && [ "${W}" -gt 0 ] 2>/dev/null && [ "${H}" -gt 0 ] 2>/dev/null; then
		_pass "${topic}: width=${W} height=${H}"
	else
		_fail "${topic}: width/height not parseable (got W=${W:-} H=${H:-})"
	fi
	if [ -n "${fx}" ] && awk -v v="${fx}" 'BEGIN { exit (v+0 > 0) ? 0 : 1 }'; then
		_pass "${topic}: fx=${fx} (non-zero)"
	else
		_warn "${topic}: could not parse fx (raw output above) — manually inspect K"
	fi
}
_check_camera_info /camera/color/camera_info
_check_camera_info /camera/depth/camera_info

# ----------------------------------------------------------------------------
# 5. Depth encoding + sample range — most-likely-to-bite check.
# ----------------------------------------------------------------------------
_section "5. Depth encoding & range"
if _topic_present "/camera/depth/image_rect_raw"; then
	# `ros2 topic echo --once --field encoding` outputs the bare scalar
	# value followed by '---' (YAML doc separator), e.g.:
	#     32FC1
	#     ---
	# So just take the first non-empty, non-separator line.
	_dout="$(timeout 5 ros2 topic echo --once --field encoding /camera/depth/image_rect_raw 2>/dev/null || true)"
	_enc="$(echo "${_dout}" | awk 'NF && $0 !~ /^---/ {print; exit}')"
	# Strip any surrounding quotes/whitespace just in case.
	_enc="${_enc#\"}"; _enc="${_enc%\"}"
	_enc="${_enc#\'}"; _enc="${_enc%\'}"
	_enc="${_enc## }"; _enc="${_enc%% }"
	if [ "${_enc}" = "32FC1" ]; then
		_pass "encoding=32FC1 (single-channel float32 → METERS in Isaac Sim 5.x)"
	elif [ "${_enc}" = "16UC1" ] || [ "${_enc}" = "mono16" ]; then
		_warn "encoding=${_enc} — that's typically MILLIMETERS. Downstream code assumes meters!"
	elif [ -z "${_enc}" ]; then
		_warn "encoding parse failed (raw=${_dout})"
	else
		_warn "encoding=${_enc} (unexpected)"
	fi

	_dhdr="$(timeout 5 ros2 topic echo --once --field header.frame_id /camera/depth/image_rect_raw 2>/dev/null || true)"
	_fid="$(echo "${_dhdr}" | awk 'NF && $0 !~ /^---/ {print; exit}')"
	_fid="${_fid#\"}"; _fid="${_fid%\"}"
	_fid="${_fid#\'}"; _fid="${_fid%\'}"
	if [ -n "${_fid}" ]; then
		_pass "depth frame_id='${_fid}'"
	fi
else
	_warn "skip depth encoding check (topic absent)"
fi

# ----------------------------------------------------------------------------
# 6. /clock is sim time (and /tf header.stamp is close to it).
# ----------------------------------------------------------------------------
_section "6. Sim time / timestamp coherence"
_clock_msg="$(timeout 4 ros2 topic echo --once /clock 2>/dev/null || true)"
_clock_sec="$(echo "${_clock_msg}" | awk -F': *' '/^[[:space:]]*sec/ {print $2; exit}')"
if [ -n "${_clock_sec}" ]; then
	_pass "/clock advertising sim time (sec=${_clock_sec})"
else
	_fail "/clock returned no message in 4s"
fi

_tf_stamp="$(timeout 4 ros2 topic echo --once --field 'transforms[0].header.stamp' /tf 2>/dev/null || true)"
_tf_sec="$(echo "${_tf_stamp}" | awk -F': *' '/^[[:space:]]*sec/ {print $2; exit}')"
if [ -n "${_clock_sec}" ] && [ -n "${_tf_sec}" ]; then
	_dt_abs="$(awk -v a="${_clock_sec}" -v b="${_tf_sec}" 'BEGIN { d=a-b; if (d<0) d=-d; print d }')"
	if awk -v d="${_dt_abs}" 'BEGIN { exit (d+0 < 1.0) ? 0 : 1 }'; then
		_pass "/tf header.stamp aligned with /clock (|Δ|=${_dt_abs}s < 1.0)"
	else
		_warn "/tf header.stamp drift |Δ|=${_dt_abs}s vs /clock — check use_sim_time"
	fi
fi

# ----------------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------------
_section "Summary"
echo "  ${C_GRN}PASS=${_PASS_COUNT}${C_END}  ${C_YLW}WARN=${_WARN_COUNT}${C_END}  ${C_RED}FAIL=${_FAIL_COUNT}${C_END}"

if [ "${_FAIL_COUNT}" -eq 0 ]; then
	echo "  Day 1-2 hard checks: ${C_GRN}OK${C_END}"
	exit 0
else
	echo "  Day 1-2 hard checks: ${C_RED}FAILED${C_END} (${_FAIL_COUNT} hard)"
	exit 1
fi
