# Go2 Semantic Navigation — End-to-End Operations Manual

> A reference for your future self (coming back a few weeks later): the commands below are **ready to copy-paste**; launch file names have been **statically verified** against `src/go2_bringup_sim/launch/` in this repository.  
> The simulation and `colcon build` have **not been executed on this machine**; if packages are not built or `install/setup.bash` is outdated, run a build first.

---

## A. Environment Setup

### A.1 Sourcing Required in Every New Terminal

```bash
# If conda is active, deactivate it first (avoids Python/rclpy version conflicts)
conda deactivate 2>/dev/null || true

cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation

# Method 1: project script (recommended — sets PROJECT_ROOT / ROS_DISTRO / workspace overlay)
source scripts/dev_env.sh

# Method 2: manual (equivalent skeleton)
# source /opt/ros/jazzy/setup.bash
# source install/setup.bash
```

### A.2 Process Cleanup — Kill All Leftover Processes (recommended before every restart)

```bash
cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation
bash scripts/kill_all.sh                  # default: keeps Isaac Sim and RViz alive
bash scripts/kill_all.sh --include-rviz   # also kills RViz
bash scripts/kill_all.sh --all            # kills Isaac Sim as well (use sparingly)
bash scripts/kill_all.sh --dry-run        # show what would be killed without actually killing
```

> **Why is this needed?** A plain `Ctrl+C` or `pkill -f "ros2 launch"` only kills the launch
> **parent** process; C++ child processes such as `static_transform_publisher`,
> Nav2's `component_container_isolated`, and `slam_toolbox` frequently become orphans and keep
> running. The next time `ros2 launch` is invoked, both the old and new instances coexist,
> causing: duplicate TF publishers, two SLAM nodes competing for `map->odom`, Nav2 controller
> reporting `Unable to transform robot pose into global plan's frame`, flickering
> `frontier_explorer` colors, and long "thinking" delays before the robot moves.
> `kill_all.sh` sends SIGTERM first, then SIGKILL, and finally verifies with `ps` that no
> processes remain.

### A.3 Launch Method — Recommended: Use `launch_safe.sh` as a Wrapper

```bash
# Use instead of `ros2 launch ...` directly:
bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py
bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py \
     abort_cooldown_sec:=10.0
```

What it does:
1. Uses `setsid` to place the launch process in an **independent session**, so all descendants share the same PGID.
2. On Ctrl+C, sends a graceful SIGINT to the entire process group (allowing `OnShutdown` hooks to run), waiting up to 4 seconds.
3. Then sends SIGKILL to the entire group, **including any already-detached orphans** — no more "leftover from last time".

Using `ros2 launch` directly provides none of these guarantees. Make `launch_safe.sh` a habit.

### A.4 ROS Log Directory Permissions (can cause `ros2` CLI failures)

```bash
sudo chown -R "$USER:$USER" ~/.ros
```

---

## B. Standard Launch Sequence (T1–T5)

The following order matches the header comments in `day8_two_phase.launch.py` (~L27–36).

### T1 — Simulation (Isaac + warehouse)

```bash
cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation
bash scripts/run_warehouse_ros2.sh
```

**Wait for signal (visual/log)**: the simulation window shows the warehouse; sensor-related topics begin to appear on the ROS bridge side (timing varies by machine).

**Verification (new sourced terminal)**:

```bash
ros2 topic info /clock
ros2 topic info /lidar/points
```

---

### T2 — Static TF + LaserScan (`tf_and_scan` — replaces `chair_perception` as the primary path)

```bash
# Recommended (clean exit on Ctrl+C)
bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py

# Or run directly (remember to clean up with kill_all.sh)
ros2 launch go2_bringup_sim tf_and_scan.launch.py
```

**Wait for signal**: `/scan` has a publisher.

**Verification**:

```bash
ros2 topic info /scan | grep -E 'Publisher|Type'
ros2 topic hz /scan
```

**Alternative (legacy full-perception bringup)**: if still using old tutorial commands, this also exists in the repository:

```bash
ros2 launch go2_bringup_sim chair_perception.launch.py
```

---

### T3 — Nav2 + SLAM

```bash
bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
```

**Wait for signal**: the terminal shows Nav2 lifecycle / `Managed nodes are active` or similar ready-state logs (exact wording depends on Nav2 version).

**Verification**:

```bash
ros2 topic info /map
ros2 topic info /global_costmap/costmap
ros2 node list | grep -E 'slam_toolbox|controller_server|bt_navigator' || true
```

---

### T4 — Day 8 Full Stack (two-phase, recommended)

```bash
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py

# To shorten "thinking time" (soft cooldown after each Nav2 ABORT), lower the default from 15s to 10s:
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py \
     abort_cooldown_sec:=10.0
```

**Verification (each name should appear only once)**:

```bash
ros2 node list | sort | uniq -c | sort -rn | head
ros2 node list | grep -E 'mapping_explorer|frontier_explorer|nl_parser|task_coordinator|yoloe|semantic_memory'
ros2 service list | grep get_frontiers
ros2 topic info /frontier_markers --verbose | grep "Publisher count"   # should be 1
ros2 topic info /semantic_map/markers --verbose | grep "Publisher count" # should be 1
```

> If any node appears ≥ 2 times in `ros2 node list | uniq -c`, `kill_all.sh` was not run
> or the previous run left orphans without `launch_safe.sh`. Run `bash scripts/kill_all.sh`
> first, then restart.

**Legacy single-phase Day 8 (target_class-driven + FSM EXPLORE) is still available**:

```bash
ros2 launch go2_bringup_sim day8.launch.py target_class:=chair
```

---

### T5 — RViz

```bash
cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation
bash scripts/run_rviz.sh
```

The script enables `use_sim_time:=true` for RViz (see `scripts/run_rviz.sh` ~L35–48) to avoid discrepancies between sim time and RViz's default wall clock.

**Semantic memory — RViz Marker topics** (`semantic_memory_aggregator`):

| Topic | Description |
| --- | --- |
| `/semantic_map/markers` | All **confirmed** landmarks (visible + remembered); **legacy combined topic**, backward-compatible |
| `/semantic_map/markers_visible` | **Confirmed** landmarks with `currently_visible=true` |
| `/semantic_map/markers_remembered` | **Confirmed** landmarks with `currently_visible=false` |
| `/semantic_map/debug_markers` | Debug stream: candidates, invalid rejects, anchor-missing confirmed entries, etc. |

Parameters (split publishing enabled by default): `publish_split_visibility_markers` (default True), `visible_markers_topic`, `remembered_markers_topic`. When False, only `/semantic_map/markers` is published; the two split topics are not published.

**Demo Recommendations**:

- Recording "remembered table/person": enable only **`/semantic_map/markers_remembered`** in RViz.
- Debugging live perception: enable **`/semantic_map/markers_visible`** + **`/semantic_map/debug_markers`**.
- Full debug: all four topics can be enabled simultaneously; in **`go2_semantic_nav.rviz`**, the legacy combined layer is off by default, and the split **visible / remembered** layers are on by default.

---

## C. End-to-End Demo Test Procedure

### Mode 1 — Sanity Check (table visible near spawn; chair may require exploration)

In the default warehouse layout, the table is more likely to be within the initial camera field of view; **verify against RViz `/semantic_map/objects`**.

⚠️ **NL commands must not be sent until `/mapping/status` consistently outputs `DONE`** — both `mapping_explorer` and `task_coordinator` hold Nav2 ActionClients, and concurrent clients will contend for the `/navigate_to_pose` action server.

**Command sequence**: after completing **T1→T5**:

```bash
# Phase A: observe mapping status (latched string)
ros2 topic echo /mapping/status --once

# Phase B: natural language command (example)
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'go to chair'"
```

**Expected behavior in RViz**:

- `/map` expands; Frontiers (if configured) and semantic Markers gradually appear.
- The target class (e.g., `chair`/`table`, per detection) appears in the `semantic_map` entity list.

**`/task/status` (`std_msgs/String`)**: the typical status path on success is  
`... → CHECK_MEMORY → TARGET_FOUND → PLAN_APPROACH_GOAL → NAVIGATE_TO_GOAL → VERIFY_TARGET → ARRIVED`  
(defined in `task_coordinator_node.py` state enum ~L62–79).

**ARRIVED Determination**:

- `ros2 topic echo /task/status --once` contains `ARRIVED`;
- the robot has stopped near the target with a semantically valid position (confirmed via RViz robot pose and entity marker).

---

### Mode 2 — "True" Semantic Navigation (object initially out of view)

#### Adjusting the Chair Position

Edit `sim/warehouse_scene.py`:

- **Position constant**: `CHAIR_XYZ = (3.5, -3.5, 0.0)` (approximately **L101**)
- Change to a more distant or occluded position as needed; save and **restart Isaac Sim** for the scene to be rebuilt.

#### Restart the Stack

Clean up as in **A.2** and re-run **T1→T5**.

#### Test Command

Still recommended to send after Phase A completes:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'go to chair'"
```

#### Note on Preempting `mapping_explorer` (must read)

**The current `day8_two_phase.launch.py` does not implement at the code level**: automatically stopping Phase A and switching to Approach as soon as `chair` appears in `semantic_memory`.  
The reliable demo path is:

1. Wait until **`/mapping/status` is `DONE`** (or the operator sends `/mapping/control` with `abort`/`restart` — see `mapping_explorer_node.py` ~L276–301 for semantics);  
2. Then send `/user_command`.

If navigation must proceed when no entity exists in memory yet: `nl_parser` emits `SemanticTask.requires_search = True` (`nl_parser_node.py` L353), which may trigger the **EXPLORE** state in `task_coordinator`, simultaneously contending with `mapping_explorer` for Nav2 — **not recommended as a formal demo path**.

**If you want to reproduce the "single FSM exploring while searching for chair" scenario**: use **`day8.launch.py target_class:=chair`** (`task_coordinator` has built-in EXPLORE) rather than the two-phase parallel approach.

---

### Mode 3 — Failure Scenario (class with no existing instance)

By default, **`nl_known_classes` in `day8_two_phase.launch.py` does not include `microwave`** (~L255–261). To trigger "parse microwave → then fail", override the parameter:

```bash
ros2 launch go2_bringup_sim day8_two_phase.launch.py \
  nl_known_classes:='chair,table,desk,box,microwave'
```

Otherwise the NL layer will reject `microwave` outright, instead of triggering the `FAILED` state in `task_coordinator`.

Then:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'go to microwave'"
```

**Expected `/task/status`**: when no instance is found in memory, enters **`EXPLORE`** (if the frontier is eventually exhausted) and is handled as a failure by the coordinator; the typical failure string contains **`environment fully explored`** and **`microwave`** (see `task_coordinator_node.py` L533–537).

> If `microwave` is **not** added to `nl_known_classes`, you may see only a low-confidence/rejection message in **`/nl_parser/feedback`**, with `task_coordinator` remaining in **IDLE** — this is also a form of "failure", but it is an **NL-layer rejection** rather than a navigation FSM failure.

---

### Mode 4 — Target Not Found During Phase A? Run a Perimeter Patrol (No Stack Restart Required)

`mapping_explorer`'s frontier scoring favors "maximum unknown space", so the robot often stops at the center of the room and transitions to `DONE` before YOLOE has had a chance to scan objects near walls. There is no need to restart the full stack in this case — temporarily drive Go2 in a perimeter loop so that YOLOE can scan chair/table/box into `semantic_memory_aggregator`.

```bash
# Preview the default waypoints (no ROS source needed)
python3 scripts/perimeter_patrol.py --dry-run

# Execute: CW 4 corners + 360° in-place spin at each corner, approximately 6–8 minutes
python3 scripts/perimeter_patrol.py

# Quick test: go only to the SE corner where the chair is (map(7.5, 0.5))
python3 scripts/perimeter_patrol.py --se-only

# 8 waypoints (corners + edge midpoints), no spin at each point; faster but lower scan density
python3 scripts/perimeter_patrol.py --dense --no-spin

# Reverse direction (CCW), inset increased to 2 m to avoid inflation cost
python3 scripts/perimeter_patrol.py --ccw --inset 2.0
```

Waypoints are computed from the warehouse dimensions (10 m × 10 m) plus the `world_to_map` static offset (-4, -4), giving a navigable map-frame range of approximately `x∈[-1,9], y∈[-1,9]`. With a 1.5 m inset, the corner waypoints are `(0.5,0.5) (7.5,0.5) (7.5,7.5) (0.5,7.5)`. The chair's map coordinate `(7.5, 0.5)` corresponds exactly to the SE corner.

This script directly calls `/navigate_to_pose`, so it **preempts** the current Nav2 goal held by `mapping_explorer` or `task_coordinator` — only run after `/mapping/status == DONE` (or when you intentionally want to intervene). Ctrl+C cancels the current goal and stops the robot.

After completion, use `ros2 topic echo /semantic_map/objects --once` to check whether `chair`/`table` appear in the entity list, then send `/user_command "go to chair"`.

---

## D. Troubleshooting Quick Reference

| Symptom | Possible Cause | Diagnostic Command |
|------|----------|----------|
| `/mapping/status` does not reach DONE | Frontiers never exhausted, Nav2 repeatedly ABORTs, TF not ready | `ros2 topic echo /mapping/status --once`; check mapping_explorer logs; `ros2 topic echo /tf_static --once` |
| DONE but `/semantic_map/objects` is empty | Robot never observed any object; YOLOE not running or class mismatch | `ros2 topic echo /semantic_map/objects --once`; `ros2 topic hz /detections` |
| Coordinator unresponsive after NL command | `nl_parser` confidence too low; or no subscriber on `/semantic_task/request` | `ros2 topic echo /nl_parser/feedback --once`; `ros2 topic info /semantic_task/request` |
| Navigation ABORTED mid-route | Sim TF lag, costmap issue, goal inside obstacle | Nav2 logs; `ros2 topic echo /task/status --once` |
| `/global_costmap/costmap` abnormally empty | QoS/lifecycle issue, upstream `/map` or SLAM not running | `ros2 topic info -v /global_costmap/costmap`; `ros2 topic hz /map` |
| `/scan` rate anomaly | `pointcloud_to_laserscan` stream dropped, TF issue, duplicate nodes | `ros2 topic hz /scan`; `ros2 node list \| sort \| uniq -d` |

---

## E. Key `ros2` Command Reference

```bash
# Entity list
ros2 topic echo /semantic_map/objects --once

# Detection rate
ros2 topic hz /detections

# Mapping explorer key parameters (names match node declarations)
ros2 param describe /mapping_explorer
ros2 param set /mapping_explorer max_consecutive_aborts 8
ros2 param set /mapping_explorer done_confirm_sec 8.0
ros2 param set /mapping_explorer done_fast true
# Shorten "thinking time" — soft cooldown per frontier after Nav2 ABORT (default 15 s).
# Setting to 5–10 s lets Go2 retry almost immediately, but be aware that logging frequency
# increases when Nav2 truly cannot reach a goal.
ros2 param set /mapping_explorer abort_cooldown_sec 10.0

# Cancel the current navigation goal (cancel the active goal on the action server)
ros2 action cancel /navigate_to_pose

# Keyboard teleoperation (if needed)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Force-stop Phase A (mapping explorer)
ros2 topic pub --once /mapping/control std_msgs/msg/String "data: 'abort'"

# Perimeter patrol / object scan (loop around walls, no stack restart needed)
python3 scripts/perimeter_patrol.py --dry-run     # Preview the plan
python3 scripts/perimeter_patrol.py               # Start patrol
python3 scripts/perimeter_patrol.py --se-only     # Go only to the SE corner (where the chair is)
```

---

## F. Recording a Demo Video (OBS Studio)

Common Linux launch methods (depends on installation source):

```bash
obs
# or
flatpak run com.obsproject.Studio
```

**Recommended recording content**:

1. Isaac Sim viewport (robot motion and environment).  
2. RViz (`/map`, semantic markers, optional frontiers).  
3. Terminal: `/mapping/status` echo, `/user_command` pub, `/task/status` echo.

**Layout**: on a 1080p single monitor, use "Sim top-left + RViz right + terminal strip at bottom"; on a dual-monitor setup, Sim and RViz can be shown on separate screens.

---

## Appendix: Launch File Index in Repository (existence verified)

`chair_execute_goal.launch.py`, `chair_perception.launch.py`, `chair_goto_goal.launch.py`, `chair_semantic_memory.launch.py`, `chair_with_search.launch.py`, `day6.launch.py`, `day7.launch.py`, `day8.launch.py`, `day8_two_phase.launch.py`, `mapping.launch.py`, `nav2.launch.py`, `sim_semantic_nav.launch.py`, `tf_and_scan.launch.py`, `yoloe.launch.py`

Path: `src/go2_bringup_sim/launch/`
