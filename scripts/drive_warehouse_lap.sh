#!/usr/bin/env bash
# shellcheck shell=bash
# -----------------------------------------------------------------------------
# Drive the simulated Go2 on a pre-computed safe lap of the warehouse to
# collect enough scan data for slam_toolbox to build a clean /map.
#
# IMPORTANT: the kinematic /cmd_vel integrator does NOT collide with walls.
# If we steer wrong, Go2 teleports through the wall and SLAM scans the void.
# Every leg below has been verified on paper to keep the robot inside
# x, y ∈ [-4.7, +4.7]m (0.3m margin from the walls at ±5m).
#
# Geometry:
#   Warehouse 10×10m, walls at world x,y = ±5m.
#   Go2 spawn world (-4, -4), yaw = +45° (facing NE corner).
#   In `odom` Fixed Frame anchored at spawn: Go2 starts at (0, 0) yaw=0,
#   but world coords below are easier to reason about so we use those
#   for the leg-by-leg verification.
#
# Lap pattern (visits all four quadrants + crosses centre 4× for loop
# closure):
#   start:  ( -4.00, -4.00 )  yaw= +45°  (face NE)
#   1) fwd 6m   →  ( +0.24, +0.24 )  yaw= +45°    [diagonal across centre]
#   2) turn -90°               yaw= -45°  (face SE)
#   3) fwd 5m   →  ( +3.78, -3.30 )  yaw= -45°    [explore SE]
#   4) turn 180°               yaw=+135°  (face NW)
#   5) fwd 5m   →  ( +0.24, +0.24 )  yaw=+135°    [back to centre — loop]
#   6) turn +90°               yaw=+225°  (face SW)
#   7) fwd 5m   →  ( -3.30, -3.30 )  yaw=+225°    [explore SW, near spawn]
#   8) turn 180°               yaw= +45°  (face NE)
#   9) fwd 5m   →  ( +0.24, +0.24 )  yaw= +45°    [back to centre — loop]
#  10) turn +90°               yaw=+135°  (face NW)
#  11) fwd 5m   →  ( -3.30, +3.78 )  yaw=+135°    [explore NW]
#  12) turn 180°               yaw= -45°  (face SE)
#  13) fwd 5m   →  ( +0.24, +0.24 )  yaw= -45°    [back to centre — loop]
#  14) stop
#
# Every leg ends with x, y ∈ [-3.3, +3.78] which is well inside the
# 10×10m room. Total drive time at v=0.30 m/s, w=0.50 rad/s ≈ 116s.
# slam_toolbox's keyframe gates fire constantly along the way and the
# four returns to the centre give it four loop-closure opportunities.
#
# Usage:
#   bash scripts/drive_warehouse_lap.sh        # drive the full lap
#   bash scripts/drive_warehouse_lap.sh --dry-run    # print plan, no drive
# -----------------------------------------------------------------------------
set -uo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=dev_env.sh
source "${_SCRIPT_DIR}/dev_env.sh" >/dev/null 2>&1 || true

if ! command -v ros2 >/dev/null 2>&1; then
	echo "[drive_warehouse_lap] ERROR: 'ros2' not found." >&2
	exit 2
fi

DRY_RUN=0
if [ "${1:-}" = "--dry-run" ]; then
	DRY_RUN=1
fi

# Conservative motion limits. Stay well under sim/run_go2_warehouse_ros2.py's
# --max-lin (1.0 m/s) and --max-ang (1.5 rad/s) defaults so the kinematic
# integrator + slam_toolbox both have headroom.
V=0.30      # m/s linear
W=0.50      # rad/s angular

# Send a Twist message at 10 Hz for `duration_s` seconds.
#  args: 1=label  2=lin_x  3=ang_z  4=duration_s
_drive() {
	local label="$1"
	local lin="$2"
	local ang="$3"
	local dur="$4"
	echo "[drive] $(printf '%-22s' "${label}")  lin=${lin}  ang=${ang}  for ${dur}s"
	if [ "${DRY_RUN}" -eq 1 ]; then return; fi
	timeout --preserve-status "${dur}" \
		ros2 topic pub /cmd_vel geometry_msgs/Twist \
		"{linear: {x: ${lin}}, angular: {z: ${ang}}}" --rate 10 \
		>/dev/null 2>&1 || true
}

_stop() {
	if [ "${DRY_RUN}" -eq 1 ]; then echo "[drive] STOP (dry)"; return; fi
	# Two redundant zeros: --once is sometimes missed if no
	# subscriber is currently latched; the rate-style 1s pub
	# guarantees the kinematic integrator latches a 0-velocity tick.
	ros2 topic pub /cmd_vel geometry_msgs/Twist \
		'{linear: {x: 0.0}, angular: {z: 0.0}}' --once \
		>/dev/null 2>&1 || true
	timeout 1 ros2 topic pub /cmd_vel geometry_msgs/Twist \
		'{linear: {x: 0.0}, angular: {z: 0.0}}' --rate 10 \
		>/dev/null 2>&1 || true
}

# Compute durations from V / W so changing the constants stays correct.
F6="$(awk -v v="${V}" 'BEGIN { printf "%.1f", 6.0 / v }')"
F5="$(awk -v v="${V}" 'BEGIN { printf "%.1f", 5.0 / v }')"
T180="$(awk -v w="${W}" 'BEGIN { printf "%.1f", 3.14159265 / w }')"
T90="$(awk -v w="${W}" 'BEGIN { printf "%.1f", 1.57079632 / w }')"

echo "=========================================================="
echo " Go2 warehouse SLAM data-collection lap"
echo "   v = ${V} m/s, w = ${W} rad/s"
echo "   forward-6m: ${F6}s, forward-5m: ${F5}s"
echo "   turn-180°: ${T180}s, turn-90°: ${T90}s"
[ "${DRY_RUN}" -eq 1 ] && echo "   (dry-run — no /cmd_vel will be published)"
echo "=========================================================="

# Settle: many slam_toolbox builds drop the very first scan because
# the TF buffer hasn't filled. Sit for 1 s before moving.
_drive "settle"                "0.0"   "0.0"   "1.0"

_drive "1) fwd 6m  (NE → centre)"  "${V}"  "0.0"  "${F6}"
_drive "2) turn -90°"              "0.0" "-${W}" "${T90}"
_drive "3) fwd 5m  (centre → SE)"  "${V}"  "0.0"  "${F5}"
_drive "4) turn +180°"             "0.0"  "${W}" "${T180}"
_drive "5) fwd 5m  (SE → centre)"  "${V}"  "0.0"  "${F5}"
_drive "6) turn +90°"              "0.0"  "${W}" "${T90}"
_drive "7) fwd 5m  (centre → SW)"  "${V}"  "0.0"  "${F5}"
_drive "8) turn +180°"             "0.0"  "${W}" "${T180}"
_drive "9) fwd 5m  (SW → centre)"  "${V}"  "0.0"  "${F5}"
_drive "10) turn +90°"             "0.0"  "${W}" "${T90}"
_drive "11) fwd 5m  (centre → NW)" "${V}"  "0.0"  "${F5}"
_drive "12) turn +180°"            "0.0"  "${W}" "${T180}"
_drive "13) fwd 5m  (NW → centre)" "${V}"  "0.0"  "${F5}"

_stop
echo "[drive_warehouse_lap] Done. Now run: bash scripts/check_day3.sh"
