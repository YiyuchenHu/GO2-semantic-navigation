# Phase 3A — Target Selection + Approach Goal Generation

> **Evidence used for this document**
>
> - Source of truth: the current code at
>   `src/go2_navigation/go2_navigation/target_selector_node.py`,
>   `src/go2_navigation/go2_navigation/goal_planner_node.py`,
>   `src/go2_navigation/go2_navigation/utils.py`, and the launch wiring
>   at `src/go2_bringup_sim/launch/chair_goto_goal.launch.py`.
> - Runtime validation: the live-topic checks captured while Phase 3A
>   was running against the Phase 0 sim + Phase 1 perception + Phase 2
>   semantic memory — specifically the three Phase 3A output topics at
>   a steady 2 Hz and one full message captured on each via
>   `ros2 topic echo --once` (a `SelectedTarget` for the chair in
>   `odom`, and a matching `PoseStamped` approach goal ~0.9 m in front
>   of the chair centroid).

## 1. Phase 3A overview

Phase 3A is the **target selection + approach goal generation** layer.
It sits between the Phase 2 semantic memory and whatever eventually
drives the robot. Its only job is to read the persistent semantic
entities published by Phase 2, pick a best chair entity, and publish a
single 2D approach pose the robot should go to.

Phase 3A **does not** drive the robot, does not subscribe to
`/cmd_vel`, does not verify arrival, and does not emit user-facing
messages. Those are Phase 3B concerns and are deliberately out of
scope here.

## 2. Phase 3A goals

- Consume `/semantic_map/entities` (Phase 2 output).
- Choose the best `chair` entity (single-class MVP) using a simple,
  explainable ranking.
- Publish the choice on `/semantic_query/selected_target` at a steady
  rate, with ranking reasons for operator debugging.
- Generate a single 2D approach goal pose around the chosen entity and
  publish it on `/semantic_goal/goal_pose`.
- Publish the whole candidate ring as RViz markers on
  `/semantic_goal/goal_candidates` so the approach geometry is visible
  in real time.
- Keep everything frame-parameterised (`global_frame`) so the same
  nodes run unchanged once a `map` frame arrives in a later phase.
- Stay chair-only and MVP-friendly: no costmap planning, no sampling
  heavy-duty local planners, no task coordinator dependency.

## 3. What Phase 3A implemented

### 3.1 `target_selector_node` (`go2_navigation/target_selector_node.py`)

Subscribes:

| Topic | Type | Role |
|---|---|---|
| `/semantic_map/entities` | `go2_msgs/SemanticEntityArray` | candidate pool |
| `/semantic_task/request` | `go2_msgs/SemanticTask` | optional override of target class |
| `/odom` | `nav_msgs/Odometry` | fallback robot pose when TF is missing |
| `/map`, `/costmap/global` | `nav_msgs/OccupancyGrid` | optional reachability check |

Publishes:

| Topic | Type | Rate |
|---|---|---|
| `/semantic_query/selected_target` | `go2_msgs/SelectedTarget` | 2 Hz (timer = `select_period_sec` = 0.5 s) |
| `/semantic_query/selected_target_marker` | `visualization_msgs/MarkerArray` | 2 Hz when a target is chosen |

Parameters (all defaults reflect what `chair_goto_goal.launch.py`
passes):

- `global_frame` = `odom` — Phase 2 publishes entities in `odom`
  because no SLAM map exists yet.
- `base_frame` = `base_link`.
- `default_target_class` = `chair` — the MVP default when no
  `SemanticTask` has arrived.
- `log_period_sec` = 1.0.
- `select_period_sec` = 0.5.

Selection logic (verbatim from the code, kept explainable by design):

```
score =  2.0·confidence
       + max(0, 2.0 − 0.05·recency_sec)   # recency bonus (2 s half-life-ish)
       + (0.25 if currently_visible else 0.0)
       + (1.0 if reachable else −1.5)     # reachable = occupancy safe
       − min(3.0, distance_m / 4.0)
       − min(1.0, uncertainty)
```

Only entities whose `class_label` matches the active target class
compete (case-insensitive; Phase 1 already normalises detector labels
like `bench`/`couch` → `chair`). The winner is published as a
`SelectedTarget` that carries:

- `entity_id` (from Phase 2 memory, stable across frames)
- `class_label`
- `target_pose_map` (copied directly from the Phase 2 entity's
  `pose_map`)
- `score`
- `reachable`
- `estimated_distance`
- `ranking_reasons[]` — the per-gate contributions
  (`confidence=0.24`, `distance_m=1.20`, `reachable=True`, …) so
  operators can see **why** this entity won without attaching a
  debugger.

An RViz marker array is published whenever a target is selected: a
large translucent yellow sphere on the entity plus a white
`TEXT_VIEW_FACING` `TARGET: chair` label above it. A 1 s heartbeat
summarises candidate count, selections-so-far, and the most recent
selection reason — distinct from the per-entity "SELECTED" log line
that fires only when the chosen entity actually changes.

### 3.2 `goal_planner_node` (`go2_navigation/goal_planner_node.py`)

Subscribes:

| Topic | Type | Role |
|---|---|---|
| `/semantic_query/selected_target` | `go2_msgs/SelectedTarget` | input target |
| `/odom` | `nav_msgs/Odometry` | fallback robot pose |
| `/costmap/global`, `/costmap/local` | `nav_msgs/OccupancyGrid` | optional cost filtering |

Publishes:

| Topic | Type | Rate |
|---|---|---|
| `/semantic_goal/goal_pose` | `geometry_msgs/PoseStamped` | 2 Hz (timer = 0.5 s) |
| `/semantic_goal/goal_candidates` | `visualization_msgs/MarkerArray` | 2 Hz |

Parameters (defaults from the launch file):

- `global_frame` = `odom`.
- `base_frame` = `base_link`.
- `num_angle_samples` = 16.
- `cost_threshold` = 60.
- `log_period_sec` = 1.0.
- Per-class approach stand-off (hard-coded table): `chair=0.9 m`,
  `table=1.0`, `person=1.2`, `door=1.1`, `cup=0.7`.

Planning is an intentionally tiny "ring sampler":

1. Take the selected target's `(tx, ty)` and the class-specific
   approach distance `d` (e.g. 0.9 m for `chair`).
2. Generate `num_angle_samples` evenly-spaced points on the circle
   of radius `d` around the target. 16 points is plenty for MVP.
3. Cost-filter each candidate through `occupancy_at_xy` on both the
   global and local costmap with threshold 60; `safe_cost(None, …)`
   returns True, so when no costmap is published (the current sim
   case) all candidates pass.
4. Score surviving candidates with `score = -distance_from_robot` —
   i.e. prefer the ring point closest to the robot. This keeps the
   approach on the robot's side of the chair and minimises drive
   distance.
5. The winner's yaw is set to point from the goal back at the target
   (`heading_to(goal, target)`) so that a robot at the goal faces the
   chair, and the `PoseStamped` is published.

Markers on `/semantic_goal/goal_candidates` include:
- all 16 ring candidates as small spheres (green = safe, red = unsafe),
- a blue `ARROW` marker at the chosen goal with the chosen yaw,
- a blue `LINE_STRIP` from the current robot position to the goal.

Logging: a one-shot INFO line when a **new selected entity's** goal is
first published, plus a 1 s heartbeat reporting
`valid_candidates / num_samples`, `goals_total`, whether this tick
published, and the current goal pose. When zero candidates survive
cost filtering the markers are still published (all red) and the
heartbeat records `0/16 candidates safe`.

### 3.3 Launch wiring (`chair_goto_goal.launch.py`)

- Includes `chair_semantic_memory.launch.py` (Phase 2 → which in turn
  includes Phase 1). Forwards `global_frame` and `target_class` so the
  whole stack runs in one frame.
- Starts `target_selector_node` and `goal_planner_node` with matching
  `global_frame` / `base_frame` / `log_period_sec`.
- Declared launch args: `global_frame` (default `odom`), `target_class`
  (default `chair`), `cost_threshold` (default `60`),
  `num_angle_samples` (default `16`).
- Explicitly **does not** start any execution, arrival, task
  coordinator, search, or safety node. Those are later phases.

### 3.4 Upstream expectations (not changed by Phase 3A)

Phase 3A consumes Phase 2's outputs exactly as they are. No
modifications to Phase 1 perception, Phase 2 semantic memory, or
Phase 0 sim/ROS bridge.

## 4. Test / validation results

All three Phase 3A topics were observed live against the full stack
(Phase 0 sim + Phase 1 chair perception + Phase 2 semantic memory + the
two Phase 3A nodes):

```
timeout 5 ros2 topic hz /semantic_query/selected_target   # average 2.000 Hz
timeout 5 ros2 topic hz /semantic_goal/goal_pose          # average 2.000 Hz
timeout 5 ros2 topic hz /semantic_goal/goal_candidates    # average 2.000 Hz
```

A single captured `SelectedTarget` message (the chair in the warehouse
scene):

```yaml
header: { frame_id: odom }
task_id: mvp-default
entity_id: 46f4a02c-5139-4959-9ba9-128ad88296ab
class_label: chair
target_pose_map:
  position:   { x: -1.197, y: 0.067, z: 0.683 }
  orientation:{ x: 0, y: 0, z: 0, w: 1 }
score: 2.57
reachable: true
estimated_distance: 1.195
ranking_reasons:
  - class_match=chair
  - confidence=0.24
  - recency_sec=0.1
  - distance_m=1.20
  - reachable=True
  - visible=True
  - score=2.57
```

The matching `PoseStamped` on `/semantic_goal/goal_pose` at the same
time:

```yaml
header: { frame_id: odom }
pose:
  position:    { x: -0.297, y: 0.067, z: 0.0 }
  orientation: { x: 0, y: 0, z: 1.0, w: ≈0 }   # yaw ≈ π (face the chair)
```

Interpretation:

- The goal sits on the **robot side** of the chair (`x ≈ -0.30`
  between the robot at `x ≈ 0` and the chair at `x ≈ -1.20`), at
  ~0.9 m from the chair centroid — which matches
  `_approach_dist["chair"] = 0.9` exactly.
- The orientation is yaw ≈ π, i.e. pointing back toward the chair —
  matches `heading_to(goal, target)` with the chair being in the
  `-x` direction from the goal.
- `ranking_reasons` shows the weights agreeing with what the scene
  provides: close (`1.20 m`), visible, reachable, low confidence but
  fresh.

RViz (run separately) showed the yellow `TARGET: chair` label on the
chair, 16 green/red candidate dots on the approach ring, the blue
arrow at the chosen ring point, and the blue line from the robot to
that arrow — confirming the visualization path end to end.

## 5. Known limitations of Phase 3A

1. **Goal oscillates as the robot moves.** Because `goal_planner`
   picks "ring point closest to robot" and refreshes every 0.5 s, the
   published goal slides around the ring as the robot moves. This is
   acceptable for a publisher; any consumer that wants a stable goal
   must add its own hysteresis.
2. **Cost filtering is effectively a no-op in the current sim.** There
   is no `/map` / `/costmap/global` / `/costmap/local` publisher in
   Phase 0, so `safe_cost(None, …)` returns `True` and every ring
   sample passes. The unsafe path is implemented and markers render
   red, but it is untested end-to-end.
3. **Chair-only MVP.** Only `chair` is considered for selection; the
   `approach_dist` table has entries for other classes but those paths
   are not exercised.
4. **`global_frame = odom`, not `map`.** No SLAM, so entity poses drift
   with odometry drift. `target_selector` and `goal_planner` are
   frame-parameterised to flip to `map` without a code change, but the
   MVP runs in `odom`.
5. **`/semantic_query/selected_target` can stall across occlusions.**
   Phase 2 already tolerates dropouts for promoted entities, but if
   the chair is re-promoted under a new `entity_id`,
   `goal_planner`'s per-entity one-shot "PUBLISHED" log line fires
   again; no semantic consequence, just a log artefact.
6. **Simple scalar score.** No learned cost, no side-preference, no
   left/right handedness, no social distance. All the weights are
   fixed constants in `target_selector_node.py`.
7. **Target orientation is "face the target", not "face how the user
   asked".** Phase 3A has no notion of "approach from the front" vs
   any other side — it just picks the closest safe ring sample.

## 6. How to run Phase 3A

Prerequisite: Phase 0 sim is already running in a separate shell
(`bash scripts/run_warehouse_ros2.sh` or equivalent) and `ros2 topic
hz /odom` shows the expected rate.

```bash
cd ~/<repo>/GO2-semantic-navigation
colcon build --symlink-install --packages-select go2_navigation go2_bringup_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch go2_bringup_sim chair_goto_goal.launch.py
```

Optional launch-arg overrides:

```bash
ros2 launch go2_bringup_sim chair_goto_goal.launch.py \
     global_frame:=odom target_class:=chair \
     num_angle_samples:=16 cost_threshold:=60
```

Live topic checks:

```bash
timeout 5 ros2 topic hz /semantic_query/selected_target
timeout 5 ros2 topic hz /semantic_goal/goal_pose
timeout 5 ros2 topic hz /semantic_goal/goal_candidates
ros2 topic echo /semantic_query/selected_target --once
ros2 topic echo /semantic_goal/goal_pose --once
```

RViz (run separately) — add `MarkerArray` displays for:

- `/semantic_map/markers` (Phase 2 entity markers, for context)
- `/semantic_query/selected_target_marker` (yellow highlight)
- `/semantic_goal/goal_candidates` (ring + arrow + line)

If the chair has not been promoted yet into semantic memory,
Phase 3A's two nodes will sit on their heartbeats (`candidates=0`,
`waiting for /semantic_query/selected_target`) until it is — this is
an upstream condition, not a Phase 3A bug.

## 7. Acceptance criteria

All of the following must hold while Phase 2 reports at least one
promoted `chair` entity:

1. `/semantic_query/selected_target` publishes at a steady 2 Hz, in
   `global_frame`, with a non-empty `entity_id`, `class_label = chair`
   (case-insensitive), a `target_pose_map` that matches the chair's
   entity pose, and a non-empty `ranking_reasons`.
2. `/semantic_goal/goal_pose` publishes at a steady 2 Hz, in the same
   `global_frame`, with a `position` that is `approach_dist["chair"]`
   (~0.9 m) away from the target centroid, and an `orientation` whose
   yaw points the robot back at the chair.
3. `/semantic_goal/goal_candidates` publishes at a steady 2 Hz and
   contains `num_angle_samples` SPHERE markers plus the goal ARROW
   plus the robot→goal LINE_STRIP marker.
4. `target_selector_node` logs a one-shot `SELECTED entity=…` line
   whenever the chosen `entity_id` changes, and a periodic
   `[target-selector] …` heartbeat at ~1 Hz.
5. `goal_planner_node` logs a one-shot `PUBLISHED goal for entity=…`
   line whenever the selected entity changes, and a periodic
   `[goal-planner] …` heartbeat at ~1 Hz.
6. `rviz2` with the three marker displays shows a coherent picture:
   yellow highlight on the chair, green/red candidate ring, blue
   arrow at the chosen ring point, blue line from the robot to it.
7. Flipping `global_frame:=<anything_else>` at launch time does not
   require code changes — the nodes simply start waiting for that
   frame's TF.

## 8. Next step (Phase 3B — NOT completed here)

Phase 3B will consume the Phase 3A outputs (`/semantic_goal/goal_pose`
and `/semantic_query/selected_target`) and close the loop down to the
robot:

- Drive the robot to the approach goal via the existing `/cmd_vel`
  path.
- Check arrival against the selected target.
- Surface a human-readable guidance message.

None of the above is part of Phase 3A and is **not** summarised as
complete in this document. Phase 3A is strictly the "what should the
robot go to, and where should it stop?" publisher.
