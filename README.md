# Go2 Semantic Navigation MVP

A chair-only semantic navigation MVP for the Unitree Go2 quadruped in
Isaac Sim + ROS 2 Jazzy — perception → semantic memory → target
selection → approach goal → closed-loop execution → arrival /
reacquisition, all wired on top of a pluggable locomotion backend.

> **Week 1 milestone — complete (2026-04-30).** Sim platform, 2D SLAM
> with `slam_toolbox`, Nav2 navigation, and YOLOE open-vocabulary
> detection are all independently verified. See
> [Week 1 acceptance ladder](#week-1-acceptance-ladder) below for
> the new Day-numbered acceptance docs that supersede the chair-
> only Phase 0-5 plan in active development.

---

## Week 1 acceptance ladder

The original Phase 0-5 plan (still documented further down) was
re-phased into a one-Day-per-stack-layer acceptance ladder. Each
Day has a one-axis hard test plus an automated check script. As of
end-of-Week-1 every Day passes; the chair-only Phase 0-5 stack
remains in the tree but is being deprecated phase-by-phase as the
Day ladder grows past it.

| Day | Stack layer | Doc | Check script | Status |
|:---:|-------------|-----|--------------|--------|
| 1-2 | Isaac Sim platform + RGB-D + IMU + RTX LiDAR over ROS 2 | (covered by `phase0_status.md`) | `scripts/check_day12.sh` | ✓ |
| 3 | 2D SLAM via `slam_toolbox` online_async; saved warehouse map | `docs/phase3_status.md` | `scripts/check_day3.sh` | ✓ |
| 4 | Nav2 (slam_toolbox or AMCL) + RPP controller + costmaps | `docs/day4_nav2_status.md` | `scripts/check_day4.sh` | ✓ (intermittent in sim, see doc) |
| 5 | YOLOE open-vocabulary detection (`vision_msgs/Detection2DArray`) | `docs/day5_yoloe_status.md` | `scripts/check_day5.sh` | ✓ |

> See [`docs/README.md`](docs/README.md) for an index of the dual
> Phase / Day naming schemes, [`docs/known_issues.md`](docs/known_issues.md)
> for the live bug log, and [`docs/decisions.md`](docs/decisions.md)
> for ADR-style design rationale (slam_toolbox vs RTAB-Map, YOLOE
> vs GroundingDINO, etc.).

**Week 2 starts at Day 6** (depth reprojection + semantic memory
on top of the Day 5 `/detections` stream). Until that lands, the
legacy Phase 2-4 chair-only stack documented below is the working
end-to-end pipeline.

---

## Overview

This repository is a phased, simulation-first MVP for running a
language-style navigation request — *"go to the chair"* — end-to-end
on a simulated Go2:

- **Platform**: Isaac Sim 5.1 (warehouse scene) + ROS 2 Jazzy.
- **Sensing**: RGB-D + IMU, published through Isaac Sim's ROS 2
  bridge (`/clock`, `/odom`, `/tf`, `/imu/data`, camera topics).
- **Perception**: YOLOv8-seg chair detector with label normalisation
  (`bench`, `couch`, `sofa` → `chair`) feeding 2D detections, masks,
  and 3D observations.
- **Semantic memory**: dropout-tolerant object tracker → persistent
  `SemanticEntity` promotion, visualised in RViz.
- **Planning**: single-class target selection + approach-ring goal
  sampling around the chair.
- **Execution**: a small P controller that drives `/cmd_vel` to the
  approach pose, with hysteresis so the robot stops on arrival
  rather than chasing a drifting goal.
- **Arrival / guidance**: distance + heading + recent-visibility
  check → human-readable guidance message.
- **Search / reacquisition**: rotate-in-place sweep when the chair
  goes out of view, with `SEARCHING → REACQUIRED → PURSUING →
  ARRIVED → LOST` state reporting.
- **Locomotion backend**: a pluggable abstraction in the simulator;
  the default backend is the Phase 0 kinematic `set_world_pose`
  integrator, with a scaffolded Isaac Lab policy backend seat.

"*Chair-only MVP*" means exactly that: selection, goal planning,
arrival gates, and search behaviour have been exercised only for the
`chair` class. The stack is structured to generalise but has not
been validated beyond a single chair in a warehouse scene.

---

## Current system capabilities

End-to-end, the system can (when running against the warehouse sim
with a chair in scene):

1. **Stream sensor + pose topics** from Isaac Sim over ROS 2
   (`/clock`, `/odom`, `/tf`, `/imu/data`, RGB-D).
2. **Detect the chair** in the RGB image at ~10 Hz via YOLOv8-seg,
   normalise the label to `chair`, and publish 2D detections,
   masks, and 3D observations.
3. **Track and promote** chair observations into a persistent
   `SemanticEntity` that tolerates short perception dropouts.
4. **Select** the best chair entity and publish
   `/semantic_query/selected_target`.
5. **Generate** an approach goal pose on a 16-point ring at 0.9 m
   around the chair, facing back at it, and publish
   `/semantic_goal/goal_pose` + RViz markers.
6. **Drive** the robot toward that goal via `/cmd_vel` with a
   simple proportional controller (`ROTATING → MOVING → REACHED`)
   and stop cleanly on arrival.
7. **Verify arrival** against distance + heading + recent visibility,
   and emit a plain-English guidance message on
   `/user_guidance/message`.
8. **Search** (rotate in place) when the chair is not currently
   visible, re-enter pursuit immediately on reacquisition, and
   declare `LOST` if the sweep times out.
9. **Swap motion backends** at launch time between the kinematic
   integrator (default, validated) and a scaffolded Isaac Lab
   locomotion policy backend (see limitations below).

---

## Phase status

| Phase | Name                                      | Status                                                                    | Adds                                                                       |
|:-----:|:------------------------------------------|:--------------------------------------------------------------------------|:---------------------------------------------------------------------------|
| 0     | Simulation + ROS 2 platform bring-up      | **Complete**                                                              | Warehouse sim, Go2 spawn, ROS 2 bridge, RGB-D + IMU, `/cmd_vel` kinematic driver |
| 1     | Chair-only perception                     | **Complete**                                                              | `/perception/detections_2d`, `/perception/masks`, `/perception/objects_3d`, label normalisation |
| 2     | Semantic memory                           | **Complete**                                                              | Object tracking, `SemanticEntity` promotion, `/semantic_map/entities`, RViz markers |
| 3A    | Target selection + goal generation        | **Complete**                                                              | `/semantic_query/selected_target`, `/semantic_goal/goal_pose`, `/semantic_goal/goal_candidates` |
| 3B    | Goal execution + arrival / guidance       | **MVP-complete** (stabilisation/polish opportunities remain)              | P-controller backend, `/navigation/status`, `/arrival/status`, `/user_guidance/message` |
| 4     | Search / reacquisition enhancement        | **Complete** (rotate-in-place sweep; no frontier exploration)             | `search_manager_node`, `/search/status`, `SEARCHING / REACQUIRED / PURSUING / ARRIVED / LOST` |
| 5     | Locomotion backend upgrade                | **Scaffold only** — backend abstraction landed, real walking policy still requires an external Isaac Lab checkpoint | `LocomotionBackend` protocol, `KinematicLocomotionBackend`, `PolicyLocomotionBackend` (scaffold), `--locomotion` CLI |

> **Phase 5 caveat** — the kinematic backend is identical to the
> Phase 0 behaviour and is the only backend that has been validated
> in this repo. The policy backend loads TorchScript checkpoints and
> applies joint targets, but no checkpoint ships with the repository,
> so real quadruped walking is **not** demonstrated here.

---

## Documentation map

Every phase has a dedicated status document; they are the source of
truth for acceptance criteria, known limitations, and run commands.

- [Phase 0 — Platform bring-up](docs/phase0_status.md)
- [Phase 1 — Chair-only perception](docs/phase1_status.md)
- [Phase 2 — Semantic memory](docs/phase2_status.md)
- [Phase 3A — Target selection + goal generation](docs/phase3a_status.md)
- [Phase 3B — Goal execution + arrival verification](docs/phase3b_status.md)
- [Phase 4 — Search / reacquisition](docs/phase4_status.md)
- [Phase 5 — Locomotion backend upgrade (scaffold)](docs/phase5_status.md)

Additional notes:

- Sim-side notes: [`sim/README.md`](sim/README.md)

---

## Architecture / pipeline

```
 user / default task (chair)
        │
        ▼
 ┌──────────────────────────┐  Phase 1
 │ go2_perception           │  RGB-D → YOLOv8-seg → chair detection
 │  perception_node         │  /perception/detections_2d  /masks
 │  object_localizer_3d     │  /perception/objects_3d
 └──────────┬───────────────┘
            ▼
 ┌──────────────────────────┐  Phase 2
 │ go2_semantic_memory      │  tracks + persistent entities
 │  object_tracker_node     │  /semantic/tracked_objects
 │  semantic_map_node       │  /semantic_map/entities  /markers
 └──────────┬───────────────┘
            ▼
 ┌──────────────────────────┐  Phase 3A
 │ go2_navigation           │  ring sampling approach goal
 │  target_selector_node    │  /semantic_query/selected_target
 │  goal_planner_node       │  /semantic_goal/goal_pose  /goal_candidates
 └──────────┬───────────────┘
            ▼
 ┌──────────────────────────┐  Phase 3B
 │ go2_navigation           │  P controller + arrival gates
 │  nav_executor_node       │  /cmd_vel  /navigation/status
 │  arrival_verifier_node   │  /arrival/status  /user_guidance/message
 └──────────┬───────────────┘
            │
 ┌──────────┴───────────────┐  Phase 4
 │ go2_navigation           │  rotate-in-place reacquisition
 │  search_manager_node     │  /search/status  (owns /cmd_vel in SEARCHING)
 └──────────┬───────────────┘
            ▼
         /cmd_vel   (geometry_msgs/Twist, body frame)
            │
 ┌──────────┴───────────────┐  Phase 5
 │ sim/locomotion_backends  │  LocomotionBackend protocol
 │  KinematicLocomotion…    │  default — set_world_pose integrator
 │  PolicyLocomotion…       │  scaffold — Isaac Lab TorchScript policy
 └──────────────────────────┘
            │
            ▼
     Isaac Sim PhysX articulation → /odom, /tf (back up the stack)
```

Notes on the backend boundary:

- Phase 3B's `SimplePControllerBackend` is the **high-level local
  controller** (goal → Twist). It is not the locomotion backend.
- Phase 5's `LocomotionBackend` is the **sim-side bottom layer**
  (Twist → articulation motion). The default is kinematic; the
  real-gait path is scaffolded only.

---

## How to run the MVP

There are two halves to any run: **sim side** (Isaac Sim + ROS 2
bridge) and **ROS side** (perception / memory / navigation stack).
They run in separate shells.

### 1. Sim side (Phase 0)

```bash
# shell A — boots Isaac Sim, builds the warehouse, spawns Go2,
# starts the ROS 2 bridge and the /cmd_vel kinematic driver
bash scripts/run_warehouse_ros2.sh
```

Expected: `/clock`, `/odom`, `/tf`, `/imu/data`, and the RGB-D +
`camera_info` topics all publish at their configured rates. See
[`docs/phase0_status.md`](docs/phase0_status.md) for the full list.

### 2. ROS side — pick a launch file matching the phases you want

```bash
# shell B — source ROS 2 + this workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Choose one launch file based on how far up the pipeline you want to
go:

```bash
# Phase 1 only (chair perception)
ros2 launch go2_bringup_sim chair_perception.launch.py

# Phase 1 + 2 (add semantic memory)
ros2 launch go2_bringup_sim chair_semantic_memory.launch.py

# Phase 1 + 2 + 3A (add target selection + goal generation)
ros2 launch go2_bringup_sim chair_goto_goal.launch.py

# Phase 1 + 2 + 3A + 3B (add goal execution + arrival)
ros2 launch go2_bringup_sim chair_execute_goal.launch.py

# Phase 1 + 2 + 3A + 3B + 4 (add search / reacquisition)
ros2 launch go2_bringup_sim chair_with_search.launch.py
```

Each launch file includes the phases below it, so you typically only
launch the top of the stack you want.

### 3. Inspect topics / RViz

```bash
ros2 topic list
ros2 topic hz /perception/detections_2d        # Phase 1
ros2 topic hz /semantic_map/entities           # Phase 2
ros2 topic hz /semantic_query/selected_target  # Phase 3A
ros2 topic hz /navigation/status               # Phase 3B
ros2 topic hz /search/status                   # Phase 4
ros2 topic echo /user_guidance/message         # plain-English status
```

For RViz:

```bash
bash scripts/run_rviz.sh
```

Suggested displays: `/tf`, camera image, `/semantic_map/markers`,
`/semantic_query/selected_target_marker`,
`/semantic_goal/goal_candidates`, `/search/markers`.

### 4. Phase 5 — pick a locomotion backend (optional)

Default is the kinematic integrator (equivalent to Phase 0). To try
the scaffolded policy backend with an external TorchScript
checkpoint:

```bash
# scripts/run_warehouse_ros2.sh forwards flags to sim/run_go2_warehouse_ros2.py
bash scripts/run_warehouse_ros2.sh \
     --locomotion policy \
     --policy-checkpoint /path/to/go2_flat_ts.pt
```

If the checkpoint is missing or incompatible, the factory **falls
back to kinematic** and logs a warning; the sim always comes up.

For phase-specific run commands, topic details, and acceptance
checklists, refer to the individual phase documents under
[`docs/`](docs/).

---

## Current limitations

This is an MVP. Specifically:

- **Chair-only.** Only the `chair` class has been exercised
  end-to-end. The stack is structured for more classes, but
  selection, arrival gates, and search have only been validated
  against a single chair in the warehouse scene.
- **Simulation-only.** Everything runs in Isaac Sim. No hardware
  DDS bridge, no motor calibration, no sim-to-real bridge.
- **`global_frame = odom`.** There is no SLAM / `map` frame yet;
  entity poses drift with odometry drift. All nodes are
  frame-parameterised (`global_frame:=map` flips the whole stack),
  but `map` is not produced.
- **Phase 3B motion is a P-controller-on-Twist, not a planner.** No
  obstacle avoidance, no costmap, no Nav2. The warehouse is empty
  enough for this to work.
- **Phase 5 is a scaffold.** The kinematic backend is the only one
  that has been runtime-validated inside this repo. The policy
  backend loads TorchScript and applies joint targets, but no
  checkpoint ships with the repository and no real-gait run is
  demonstrated here.
- **"Walking" is still kinematic.** Every validated run — including
  Phase 3B and Phase 4 acceptance — uses the kinematic integrator
  with articulation gravity disabled. The robot slides at the
  commanded velocity; it does not step.
- **No safety layer.** `go2_safety` is in the tree but is **not**
  wired into any launch file. No e-stop watchdog, no collision
  monitor, no shared-autonomy veto.
- **No task coordinator loop.** `go2_task_coordinator` exists as
  scaffolding, but the end-to-end
  `/semantic_task/request → /semantic_task/result` orchestration
  is not wired in the current launches.
- **Search is in-place only.** Phase 4 rotates; it does not
  translate, does not enter unexplored rooms, and does not do
  frontier-based exploration.

---

## Next steps

Plausible directions, in roughly decreasing priority:

- **Validate a real Go2 locomotion policy.** Obtain an Isaac Lab
  `Isaac-Velocity-Flat-Unitree-Go2` (or equivalent) checkpoint,
  export it as TorchScript, and exercise
  `PolicyLocomotionBackend` against Phases 1 – 4. This is the
  shortest path from "MVP slides" to "MVP walks".
- **Add a minimal `map` frame.** Even a static 2D map would remove
  the `odom`-drift caveat, enable proper costmap-aware goal
  filtering in Phase 3A, and unlock a Nav2 backend behind the
  existing `nav_executor_node` `backend` parameter.
- **Broaden beyond chair-only.** Exercise the
  `table / person / door / cup` entries already present in
  approach / arrival tables; reason about multiple visible
  instances; wire multi-class target selection.
- **Stronger search / exploration.** Replace the Phase 4
  in-place sweep with a viewpoint-selection or frontier-based
  exploration layer once a map is available.
- **Task coordinator loop.** Wire
  `go2_task_coordinator::task_coordinator_node` to close the
  `/semantic_task/request → /semantic_task/result` contract and
  coordinate search / pursuit / arrival / cancel transitions.
- **Safety integration.** Bring up `go2_safety`, wire e-stop and
  collision monitoring into the `/cmd_vel` path, particularly once
  the policy backend is walking.
- **RViz config + demo bag.** Ship a saved RViz layout and a
  reference `.bag` so a first-time user can reproduce the arrival
  demo without running Isaac Sim.

---

## Repository layout

```
sim/                                  Isaac Sim scene + locomotion backends
  warehouse_scene.py                  reusable scene library
  build_go2_warehouse.py              one-shot scene builder
  run_go2_warehouse_ros2.py           Phase 0 sim + ROS 2 bridge + CmdVelDriver façade
  locomotion_backends.py              Phase 5 LocomotionBackend + kinematic/policy impls

src/                                  ROS 2 packages (colcon workspace)
  go2_msgs/                           SemanticTask, SemanticEntity, SelectedTarget, ...
  go2_bringup_sim/                    launch files for each phase
  go2_perception/                     Phase 1 — YOLO detector + label normalisation
  go2_object_localization/            Phase 1 — 2D→3D projection
  go2_semantic_memory/                Phase 2 — tracker + semantic map
  go2_navigation/                     Phase 3A/3B/4 — selection, planning, execution, search
  go2_command_interface/              parse /user_command → SemanticTask (side channel)
  go2_task_coordinator/               FSM scaffolding (not wired into default launches)
  go2_safety/                         safety scaffolding (not wired into default launches)
  go2_debug_tools/                    logs, trace watchers, synthetic publishers

scripts/                              convenience launchers
  run_warehouse_ros2.sh               Phase 0 sim bring-up (forwards flags)
  run_rviz.sh                         RViz with sensible defaults
  run_go2_stack.sh, run_all.sh, run_tmux.sh, ...

docs/                                 phase status documents (source of truth)
```

---

## Python environment

Day 5 added pip-side dependencies (`ultralytics`, `torch`,
`opencv-python`, ...). See [`requirements.txt`](requirements.txt)
for the pinned set with notes on Ubuntu 24.04 PEP 668 +
`--break-system-packages` quirks and on the YOLOE / MobileCLIP
auto-download behaviour.

ROS 2 side dependencies (apt) — beyond the base `ros-jazzy-desktop`:

```bash
sudo apt install \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-vision-msgs \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-cv-bridge \
    ros-jazzy-topic-tools
```

---

## License

No license is set at the repository level yet. Treat the code as
"all rights reserved" until a `LICENSE` file lands. Individual ROS
2 packages under `src/*/package.xml` declare `Apache-2.0`; once the
root license is settled, the per-package declarations should be
confirmed against it.

---

## Acknowledgements

Built on top of NVIDIA Isaac Sim, the ROS 2 Jazzy Jalisco
distribution, Ultralytics YOLOv8, and the Unitree Go2 URDF/USD
assets.
