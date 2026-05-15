# Go2 Semantic Navigation

> **Open-vocabulary object navigation for the Unitree Go2 quadruped, on Isaac Sim warehouse.**
> A natural-language command (e.g. `"find chair"`) triggers autonomous frontier exploration, open-vocabulary YOLOE detection, semantic memory accumulation, and goal-directed navigation — all in one FSM.

![Find chair demo](docs/media/demo_find_chair.gif)

*Above: `/user_command "find chair"` → autonomous exploration → YOLOE target detection → FSM transitions `EXPLORE → TARGET_FOUND → NAVIGATE_TO_GOAL → ARRIVED`. Isaac Sim 5.1, real-time (1×).*

📺 [Full demo video (mp4)](docs/media/demo_full.mp4)

---

## What this is

A simulation-first semantic navigation stack for the Unitree Go2, built on Isaac Sim 5.1, ROS 2 Jazzy, SLAM Toolbox, Nav2, and **YOLOE open-vocabulary detection**. The robot:

1. Receives a natural-language command on `/user_command` (e.g. `"find chair"`, `"find table"`).
2. Autonomously explores the unknown warehouse via frontier-based exploration.
3. Detects target objects with YOLOE — open-vocabulary, so adding a new class requires only extending a string list, no retraining.
4. Builds a persistent semantic memory of detected landmarks (visible vs. remembered).
5. Plans an approach goal around the target and navigates there via Nav2 with social-aware costmap inflation.

---

## Environment

| Requirement   | Version / Note                              |
|---------------|---------------------------------------------|
| OS            | Ubuntu 24.04                                |
| ROS 2         | Jazzy                                       |
| Isaac Sim     | 5.1                                         |
| GPU           | NVIDIA RTX (tested on RTX 5090)             |
| Python        | 3.10+                                       |

---

## Pre-flight setup (run once)

```bash
# 0. Clone and enter the project root
git clone https://github.com/YiyuchenHu/GO2-semantic-navigation.git
cd GO2-semantic-navigation

# 1. ROS 2 dependencies
sudo apt update && sudo apt install -y \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-rviz2 \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-topic-tools \
  ros-jazzy-tf2-geometry-msgs ros-jazzy-vision-msgs \
  ros-jazzy-cv-bridge

# 2. Python dependencies (YOLOE via ultralytics)
pip install --user -r requirements.txt

# 3. Pre-fetch YOLOE weights (~600 MB, one-time)
python3 -c "from ultralytics import YOLOE; YOLOE('yoloe-11s-seg.pt')"

# 4. Build the colcon workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 5. Configure local paths
# Edit scripts/dev_env.sh and set ISAAC_SIM_ROOT to your Isaac Sim install path.
```

> **Note on the NVIDIA People Pack:** the social-cost halo around the
> `person` class requires Isaac Sim's optional People Pack assets. The
> default `find chair` / `find table` demo does **not** depend on this —
> it is only used when extending the demo to person-aware avoidance.

---

## Quick Start — Command-first demo

Open **three terminals**. In each one, first source the local env:

```bash
source scripts/dev_env.sh
```

Then:

```bash
# T1 — Isaac Sim warehouse (keep running)
bash scripts/run_warehouse_ros2.sh

# T2 — Full semantic navigation stack
# (wait until Isaac Sim is fully up and publishing /camera/* and /lidar/points)
ros2 launch go2_bringup_sim command_first_demo.launch.py

# T3 — RViz
bash scripts/run_rviz.sh
```

Wait ~20 seconds for SLAM Toolbox to publish the first `/map`. Then publish a natural-language command:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'find chair'"
# or:
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'find table'"
```

Monitor the FSM:

```bash
ros2 topic echo /task_coordinator/state
```

**Expected progression:**

```
IDLE → PARSE_COMMAND → CHECK_MEMORY → TARGET_NOT_FOUND
     → EXPLORE → ... → TARGET_FOUND → PLAN_APPROACH_GOAL
     → NAVIGATE_TO_GOAL → ARRIVED
```

---

## System architecture

```
Isaac Sim (warehouse) ──► /camera/color, /camera/depth, /lidar/points, /tf, /clock
        │
        ├──► pointcloud_to_laserscan ──► /scan
        ├──► slam_toolbox ──► /map
        │
        ├──► YOLOE (open-vocab) ──► /detections
        │         │
        │         ▼
        ├──► depth_projector ──► /detections_3d
        │         │
        │         ▼
        ├──► semantic_memory_aggregator ──► /semantic_map/markers_{visible,remembered}
        │         │
        │         ▼
        ├──► target_selector ──► /semantic_query/selected_target
        │         │
        │         ▼
        ├──► approach_goal_planner ──► /semantic_goal/goal_pose
        │
        ├──► frontier_explorer ◄── task_coordinator (EXPLORE state)
        │
        ├──► social_obstacle_publisher ──► /social_obstacles (PointCloud2, marking layer)
        │
        ├──► Nav2 (RPP controller, social-inflated costmaps)
        │         ▲
        │         │ NavigateToPose
        │         │
        └──► task_coordinator ◄── nl_parser ◄── /user_command
```

---

## Natural-language commands

The `nl_parser` recognises any phrase that resolves to a class in YOLOE's allowlist. Out of the box:

| Input phrase(s)                          | Resolved class  |
|------------------------------------------|-----------------|
| `find chair`, `go to chair`              | `chair`         |
| `find table`, `find desk`, `find workbench` | `table`     |
| `find person`, `find people`             | `person`        |

**Adding a new target class** requires only extending the `classes` and `nl_known_classes` launch arguments in `command_first_demo.launch.py` — no retraining, no code changes elsewhere.

---

## Key features

- Isaac Sim 5.1 warehouse with RTX LiDAR (`OS1_REV6_32ch10hz512res` profile)
- SLAM Toolbox online mapping + Nav2 (ROS 2 Jazzy)
- Single-launch command-first FSM (`command_first_demo.launch.py`)
- **YOLOE open-vocabulary detection** — chair, table, person, extensible
- Depth projection + PointCloud-anchored semantic memory
- Visible vs. remembered marker split in RViz
- Approach goal planner with 16-point ring sampling around targets
- Social-aware Nav2 costmap (0.8 m inflation, `/social_obstacles` marking layer)

---

## Repository layout

```
sim/                          Isaac Sim entry point and locomotion backends
src/
  go2_bringup_sim/            Launch files, Nav2 / SLAM config, RViz, URDF
  go2_semantic_perception/    depth_projector, semantic_memory, target_selector,
                              approach_goal_planner, arrival_verifier
  go2_navigation/             frontier_explorer, social_obstacle_publisher
  go2_nl_parser/              natural-language command parser
  go2_task_coordinator/       high-level FSM
  go2_msgs/                   custom messages and services
  go2_perception/             YOLOE detector
scripts/                      shell and Python helpers, diagnostics
docs/                         runbooks, project status, design notes
  media/                      demo gif and full mp4
```

---

## Known limitations

- **Simulation only.** This demo runs exclusively in Isaac Sim; it has not been deployed on a physical Unitree Go2.
- **Navigation arrival reporting.** Under certain costmap inflation conditions, the FSM may report `FAILED` after the robot has physically reached the target vicinity (Nav2 `xy_goal_tolerance` vs. inflated obstacle layer near large furniture). Workaround: increase `xy_goal_tolerance` in `nav2_params_social.yaml` for the controller.
- **Semantic marker deduplication is per-session.** Re-detections of the same object across long horizons may register as new markers; cross-session persistence is future work.
- **View-dependent detection.** YOLOE confidence on table/chair depends on viewing angle; sparse depth returns near object edges can produce occasional ghost anchors.

---

## Future Work

Three directions naturally extend this work:

1. **Long-term semantic memory** — cross-session marker persistence and deduplication, enabling long-horizon inspection in large-scale environments.
2. **Richer command grounding** — replacing the keyword-based `/user_command` with a vision-language-action (VLA) front-end for compositional instructions.
3. **Dynamic obstacle handling and social-aware planning** — extending beyond marking-layer inflation to model human motion and personal-space constraints when sharing the warehouse with workers.

The Unitree Go2 platform is particularly well-suited to these directions: a legged base allows close interaction with humans on unstructured floors, beyond what wheeled platforms (e.g., TurtleBot) can offer.

---

## Documentation

- [`docs/HOW_TO_RUN.md`](docs/HOW_TO_RUN.md) — legacy runbook (deprecated, kept for reference only)
- [`docs/PROJECT_STATUS.md`](docs/PROJECT_STATUS.md) — development notes

---

## Author

**Yiyuchen Hu** — Lehigh University, 2026
