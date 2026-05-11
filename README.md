# Go2 Semantic Navigation

A simulation-first autonomous semantic navigation system for the Unitree Go2
quadruped robot, built on Isaac Sim, ROS 2 Jazzy, SLAM Toolbox, Nav2, and
open-vocabulary perception.  The robot maps an unknown warehouse scene
autonomously, builds a persistent semantic memory of discovered landmarks
(person and table), and then navigates to them in response to natural-language
commands such as `"go to person"` or `"go to table"`.

---

## Demo

[Demo 1 — Table and Person Semantic Navigation](PUT_DEMO_LINK_HERE)

The demo shows the full pipeline: autonomous SLAM-based frontier exploration,
YOLOE-based person/table perception, depth-projected PointCloud anchoring,
semantic memory accumulation, and natural-language goal execution via Nav2.

---

## Key Features

- Isaac Sim 5.1 warehouse simulation with Unitree Go2
- Stable RTX LiDAR setup using `OS1_REV6_32ch10hz512res` profile
- SLAM Toolbox online mapping + Nav2 stack (ROS 2 Jazzy)
- Frontier-based autonomous exploration (`mapping_explorer_node`)
- YOLOE open-vocabulary detection for person and table
- Depth projection and PointCloud semantic anchoring
- Semantic memory with visible vs. remembered RViz marker split
- Natural-language goal commands over `/user_command`
- Approach goal planner with 16-point ring sampling around targets
- Nav2 action debug tools and diagnostic scripts
- Lifecycle recording and semantic health check scripts

---

## System Architecture

```
Isaac Sim (warehouse scene)
        │  ROS 2 bridge
        ▼
  /scan  /odom  /tf  /depth/image_raw  /rgb/image_raw
        │
        ├──► slam_toolbox ──► /map
        │
        ├──► Nav2 (RPP controller, costmaps, BT)
        │         ▲
        │         │ NavigateToPose action
        │         │
        ├──► yoloe_detector_node ──► /perception/detections_2d
        │
        ├──► depth_projector_node ──► /perception/objects_3d (PointCloud anchors)
        │
        ├──► semantic_memory_aggregator_node ──► /semantic_map/entities
        │         (visible confirmed  → /semantic_map/markers_visible)
        │         (remembered         → /semantic_map/markers_remembered)
        │
        ├──► target_selector_node ──► /semantic_query/selected_target
        │
        ├──► approach_goal_planner_node ──► /semantic_goal/goal_pose
        │
        ├──► nl_parser_node ◄── /user_command  (natural-language input)
        │         │
        │         └──► task_coordinator_node ──► NavigateToPose (Nav2)
        │
        └──► mapping_explorer_node ──► frontier goals (during mapping phase)
```

---

## Repository Structure

```
sim/                          Isaac Sim entry point and locomotion backends
src/
  go2_bringup_sim/            Launch files, Nav2 / SLAM config, RViz, URDF
  go2_semantic_perception/    depth_projector, semantic_memory, target_selector,
                              approach_goal_planner, arrival_verifier
  go2_navigation/             frontier_explorer_node, mapping_explorer_node
  go2_nl_parser/              natural-language command parser
  go2_task_coordinator/       high-level task coordination node
  go2_msgs/                   custom message and service definitions
  go2_perception/             YOLOE detector node
scripts/                      shell and Python helper/diagnostic scripts
docs/                         runbooks, project status, design notes
maps/                         pre-saved warehouse maps
policies/                     locomotion policy weights (not committed)
```

---

## Environment

| Requirement         | Version / Note                        |
|---------------------|---------------------------------------|
| OS                  | Ubuntu 24.04 (tested)                 |
| ROS 2               | Jazzy                                 |
| Isaac Sim           | 5.1                                   |
| GPU                 | NVIDIA GPU with RTX support           |
| Python              | 3.10+ (colcon workspace)              |
| Local env setup     | `source scripts/dev_env.sh`           |

> Machine-specific paths are kept in `scripts/dev_env.sh` and are not
> assumed to be reproducible as-is.  Adjust that file for your local
> Isaac Sim and ROS 2 installation paths.

---

## Quick Start — Demo Workflow

Open **five terminals**.  In each one, first run:

```bash
source scripts/dev_env.sh
```

Then start the stack in order:

```bash
# T0 — clean up any leftover processes (optional but recommended)
bash scripts/kill_all.sh

# T1 — Isaac Sim warehouse simulation
bash scripts/run_warehouse_ros2.sh

# T2 — TF and scan bridge
bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py

# T3 — SLAM + Nav2
bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True

# T4 — Day 8 two-phase demo (mapping phase → semantic navigation phase)
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py

# T5 — RViz
bash scripts/run_rviz.sh
```

> **Wait for mapping to finish before sending NL commands.**
> Monitor `/mapping/status` — when it reports `DONE`, the robot has
> completed frontier exploration and the semantic navigation phase begins.

For the full runbook including troubleshooting, see
[`docs/HOW_TO_RUN.md`](docs/HOW_TO_RUN.md).

---

## Natural-Language Commands

Send commands over `/user_command` once mapping is complete:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'go to person'}"
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'go to table'}"
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'go to desk'}"
```

**Label canonicalization:**

| Input phrase(s)                          | Resolved target |
|------------------------------------------|-----------------|
| `desk`, `workbench`, `dining table`      | `table`         |
| `human`, `man`, `worker`, `people`       | `person`        |

---

## RViz Visualization

| Topic                             | Description                                                |
|-----------------------------------|------------------------------------------------------------|
| `/semantic_map/markers_visible`   | Currently visible, confirmed semantic landmarks            |
| `/semantic_map/markers_remembered`| Remembered landmarks (seen before, not currently visible)  |
| `/semantic_map/markers`           | Legacy combined marker array                               |
| `/semantic_map/debug_markers`     | Detection candidates and debug visualization               |

**Recommended demo view:** enable `markers_remembered` + `/map` + robot model
+ goal pose arrow.  Remembered markers persist across the full exploration
trajectory and are the primary navigation targets.

---

## Important Scripts

| Script                                | Purpose                                         |
|---------------------------------------|-------------------------------------------------|
| `scripts/run_warehouse_ros2.sh`       | Launch Isaac Sim warehouse scene                |
| `scripts/launch_safe.sh`              | Safe wrapper for `ros2 launch` with logging     |
| `scripts/run_rviz.sh`                 | Start RViz with the semantic navigation config  |
| `scripts/test_day8_nl_to_goal.sh`     | End-to-end NL→goal smoke test                   |
| `scripts/debug_nav2_action_chain.sh`  | Diagnose Nav2 action server and BT state        |
| `scripts/check_table_semantic_health.sh` | Semantic memory health check for table       |
| `scripts/check_anchor_health.sh`      | PointCloud anchor diagnostic                    |
| `scripts/record_lidar_health.sh`      | Record LiDAR scan health over time              |
| `scripts/record_semantic_lifecycle.sh`| Record full semantic lifecycle to bag           |
| `scripts/cleanup_semantic_landmarks.sh` | Remove stale landmarks from memory             |

---

## Known Limitations

- **Simulation only.** This demo runs exclusively in Isaac Sim; it has not
  been deployed on a physical Unitree Go2.
- **No locomotion policy in the final MVP path.** The kinematic / sim backend
  is used for stable demo behavior.  Locomotion policy scaffolding exists in
  the tree but is not part of the validated end-to-end flow.
- **Table detection is view-dependent.** YOLOE must see a clear frontal or
  top-down view; detection may miss the table from certain angles.
- **Semantic landmarks depend on YOLOE + PointCloud anchoring.** False
  positives or sparse depth returns can produce ghost landmarks.
- **LiDAR required careful tuning.** The default `OS1_REV6_32ch10hz512res`
  profile was chosen for stability; higher-resolution profiles may cause
  scan drop-outs in sim.
- **Some legacy Day-numbered scripts and chair references** remain in the tree
  for historical reference and are not part of the current demo path.

---

## Future Work

- Real Unitree Go2 deployment and hardware-in-the-loop testing
- Locomotion policy integration for dynamic locomotion on real terrain
- Object-level SLAM and more robust semantic perception
- Multi-object disambiguation (e.g., multiple tables in the scene)
- Cleaner automated integration tests and packaging
- Full English documentation cleanup and contribution guidelines

---

## Documentation

- [`docs/HOW_TO_RUN.md`](docs/HOW_TO_RUN.md) — Full step-by-step runbook
- [`docs/PROJECT_STATUS.md`](docs/PROJECT_STATUS.md) — Development notes and
  per-Day acceptance status

---

## Author

**Yiyuchen Hu** — Lehigh University, 2026
