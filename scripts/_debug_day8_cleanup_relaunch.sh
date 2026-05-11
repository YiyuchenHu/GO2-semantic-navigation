#!/usr/bin/env bash
# scripts/_debug_day8_cleanup_relaunch.sh
#
# DEBUG-MODE-only helper: kills ALL ROS processes (except the Isaac Sim),
# then relaunches the Day 8 stack in the correct order, then runs the
# probe twice (mid-cleanup + final).
#
# Why: the previous probe showed 14 ROS nodes with 2-3 duplicates each
# (H3), no /scan (H1), no /map (H2), AND no /lidar/points (H6 — sim
# LiDAR may have been silenced by DDS discovery chaos). This script
# clears the chaos in one go so we can take a clean post-fix
# measurement and prove or disprove H6.
#
# Sim is NOT touched — it's a different process tree (kit + python via
# isaacsim_5.1_backup) and restarting it costs minutes.
#
# Usage:
#   bash scripts/_debug_day8_cleanup_relaunch.sh

set -e

# Variant: which Day 8 launch file to bring up at the end.
#   (default)        -> day8.launch.py target_class:=chair  (legacy)
#   --two-phase      -> day8_two_phase.launch.py            (new)
DAY8_VARIANT="legacy"
if [ "${1:-}" = "--two-phase" ]; then
	DAY8_VARIANT="two_phase"
	shift
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${REPO_DIR}/.cursor"
TF_LOG="${LOG_DIR}/relaunch-tf_and_scan.log"
NAV2_LOG="${LOG_DIR}/relaunch-nav2.log"
DAY8_LOG="${LOG_DIR}/relaunch-day8.log"

mkdir -p "${LOG_DIR}"

cd "${REPO_DIR}"

# Force system python (rclpy C extension is built for cpython-3.12, so
# any conda env with python 3.13 etc. would import-fail with
# "No module named 'rclpy._rclpy_pybind11'").
PY3=/usr/bin/python3
unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE

echo "[relaunch] === Step 1: aggressive pkill of ALL ROS processes (keeping sim) ==="
# Kill order matters: launch_ros first (so it doesn't respawn children),
# then component_container, then individual launches, then per-node
# python processes.
for pat in 'launch_ros' 'launch.go2' 'go2_bringup_sim' 'tf_and_scan.launch' \
           'nav2.launch' 'day8.launch' 'day7.launch' 'chair_perception' \
           'component_container' 'lifecycle_manager' 'rviz2' 'rviz' \
           '_debug_day8_probe' 'frontier_explorer_node' \
           'task_coordinator_node' 'yoloe_detector_node' \
           'depth_projector_node' 'semantic_memory_aggregator_node' \
           'target_selector_node' 'approach_goal_planner_node' \
           'object_localizer_3d_node' 'perception_node' \
           'pointcloud_to_laserscan' 'static_transform_publisher' \
           'slam_toolbox' 'controller_server' 'planner_server' \
           'bt_navigator' 'behavior_server' 'velocity_smoother' \
           'smoother_server' 'waypoint_follower' 'teleop_twist_keyboard' \
           'collision_monitor' 'docking_server' 'route_server' \
           'map_saver' 'global_costmap' 'local_costmap' ; do
	pkill -f "${pat}" 2>/dev/null || true
done
sleep 2

# Second pass with -9 for stragglers, but explicitly EXCLUDE Isaac Sim
# (kit, isaacsim_5.1_backup, run_warehouse_ros2, run_go2_warehouse).
echo "[relaunch] === Step 2: SIGKILL stragglers (sparing sim + this script) ==="
# pgrep -af lists 'pid cmd' so we can filter precisely.
mapfile -t _stragglers < <(
	pgrep -af 'ros2|component_container|rviz|launch_ros|task_coordinator|frontier_explorer|yoloe_detector|depth_projector|semantic_memory|target_selector|approach_goal|slam_toolbox|teleop_twist|nav2|pointcloud_to_laserscan|static_transform_publisher|chair_perception|object_localizer|perception_node' 2>/dev/null \
		| grep -v 'isaacsim_5\.1_backup\|run_warehouse_ros2\|run_go2_warehouse\|_debug_day8_cleanup_relaunch\|/usr/bin/grep\|^$' || true
)
for line in "${_stragglers[@]}"; do
	_pid="${line%% *}"
	# Skip my own pid + my parent shell.
	if [ "${_pid}" = "$$" ] || [ "${_pid}" = "${PPID}" ]; then continue; fi
	echo "[relaunch] kill -9 ${_pid} : ${line#* }"
	kill -9 "${_pid}" 2>/dev/null || true
done

ros2 daemon stop 2>/dev/null || true
sleep 2

echo
echo "[relaunch] === Step 3: source ROS + workspace ==="
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash || \
	source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

echo "[relaunch] === Step 4: probe BEFORE relaunch (cleanup-only baseline) ==="
echo "[relaunch] this confirms H3 cleanup worked + tells us if /lidar/points (H6) is alive"
echo "[relaunch] using python: ${PY3} ($(${PY3} --version 2>&1))"
"${PY3}" scripts/_debug_day8_probe.py post-cleanup 8

echo
echo "[relaunch] === Step 5: launch tf_and_scan in background ==="
nohup ros2 launch go2_bringup_sim tf_and_scan.launch.py \
	>"${TF_LOG}" 2>&1 &
TF_PID=$!
echo "[relaunch] tf_and_scan pid=${TF_PID}, log=${TF_LOG}"

# Wait for /scan to appear (up to 20 s). pointcloud_to_laserscan needs
# both static TFs and /lidar/points to be flowing — if /lidar/points
# is dead this loop will time out.
echo "[relaunch] waiting up to 20 s for /scan to flow..."
for i in $(seq 1 20); do
	# Use ros2 topic info to check publisher count (cheap, no spin).
	if ros2 topic info /scan 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
		echo "[relaunch] /scan publisher detected at t+${i}s"
		break
	fi
	sleep 1
done

echo
echo "[relaunch] === Step 6: launch Nav2 (slam:=True) in background ==="
nohup ros2 launch go2_bringup_sim nav2.launch.py slam:=True \
	>"${NAV2_LOG}" 2>&1 &
NAV2_PID=$!
echo "[relaunch] nav2 pid=${NAV2_PID}, log=${NAV2_LOG}"

# slam_toolbox needs ~10-20 s to lifecycle-activate.
echo "[relaunch] waiting up to 30 s for /map to publish..."
for i in $(seq 1 30); do
	if ros2 topic info /map 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
		echo "[relaunch] /map publisher detected at t+${i}s"
		break
	fi
	sleep 1
done

echo
echo "[relaunch] === Step 7: launch Day 8 in background ==="
if [ "${DAY8_VARIANT}" = "two_phase" ]; then
	echo "[relaunch] variant: TWO-PHASE — day8_two_phase.launch.py"
	nohup ros2 launch go2_bringup_sim day8_two_phase.launch.py \
		>"${DAY8_LOG}" 2>&1 &
else
	echo "[relaunch] variant: LEGACY — day8.launch.py target_class:=chair"
	nohup ros2 launch go2_bringup_sim day8.launch.py target_class:=chair \
		>"${DAY8_LOG}" 2>&1 &
fi
DAY8_PID=$!
echo "[relaunch] day8 pid=${DAY8_PID}, log=${DAY8_LOG}"

# Give Day 8 nodes time to come up + start producing /target/selected etc.
sleep 8

echo
echo "[relaunch] === Step 8: probe POST-RELAUNCH (clean stack) ==="
"${PY3}" scripts/_debug_day8_probe.py post-fix-clean 12

echo
echo "[relaunch] === Step 9: tail of each launch log ==="
for f in "${TF_LOG}" "${NAV2_LOG}" "${DAY8_LOG}"; do
	echo "--- ${f} (last 8 lines) ---"
	tail -n 8 "${f}" 2>/dev/null || echo "(empty)"
done

echo
echo "[relaunch] DONE."
echo "[relaunch]   tf_and_scan pid=${TF_PID}"
echo "[relaunch]   nav2 pid=${NAV2_PID}"
echo "[relaunch]   day8 pid=${DAY8_PID}"
echo "[relaunch] These launches are now running in the background (nohup);"
echo "[relaunch] kill them with: kill ${TF_PID} ${NAV2_PID} ${DAY8_PID}"

exit 0
