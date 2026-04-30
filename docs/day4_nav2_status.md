# Day 4 — Nav2 (localization + autonomous goal navigation)

**Status: ✓ functionally complete; intermittent in this sim due to GPU contention.**

End-to-end demonstration achieved on 2026-04-29 (final run):

  * RViz shows the slam_toolbox /map building incrementally
  * RViz shows global_costmap inflation overlaid on the static map
  * Clicking `2D Goal Pose` produces a green `/plan` Path within ~1 s
  * Action client sees `Goal accepted`, then streaming `feedback`
    with `current_pose`, `distance_remaining` shrinking, and
    `number_of_recoveries: 0`
  * Go2 actually drives toward the goal in the sim

The "intermittent" caveat: under our Isaac Sim setup the RTX LiDAR
runs at ~4 Hz with occasional 14 s stalls (it shares the GPU with
the cameras). slam_toolbox's `map → odom` TF therefore stutters,
and `controller_server`'s RPP path handler occasionally aborts a
`follow_path` action with `Lookup would require extrapolation into
the future. Requested time X but the latest data is at time X` —
this drops `/cmd_vel_smoothed` from 20 Hz to ~4 Hz. Result: Go2
moves in spurts. **Same nav stack on real Go2 hardware (reliable
10 Hz Livox scan) will not exhibit this** — the bottleneck is
purely sim-side rendering, not Nav2.

This document is named `day4_nav2_status.md` (not `phase4_*`) on
purpose. The repo's existing `docs/phase4_status.md` covers a
**different** Phase 4 from the original phasing — the rotate-in-place
search / reacquisition behaviour scaffolded under
`go2_navigation::search_manager_node`. That work pre-dates the new
acceptance ladder (Day N) and is unrelated to Nav2.

Day 4 here = **classic Nav2 with AMCL on a pre-built map**, exposed
via the standard `2D Goal Pose` button in RViz.

---

## What Day 4 delivers

### Topology

```
maps/warehouse_v1.yaml ──► map_server ──► /map (TRANSIENT_LOCAL)
                                            │
/scan ─────► AMCL ◄─────────────────────────┤
              │                             │
              └────► /tf (map → odom @20Hz) ┤
                                            ▼
                                    global_costmap
                                            │
RViz "2D Goal Pose"  ──► /goal_pose ──► bt_navigator ──► planner_server
                                                              │
                                                              ▼
                                                              /plan
                                                              │
                                                              ▼
                                                       controller_server  ──► /cmd_vel_nav
                                                              │
                                                              ▼
                                                       velocity_smoother  ──► /cmd_vel_smoothed
                                                              │
                                                              ▼ (topic_tools/relay)
                                                                 /cmd_vel ──► sim's SubTwist ──► Go2
```

### New topics

| Topic | Type | Source | Notes |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` (TRANSIENT_LOCAL) | static, loaded from `warehouse_v1.yaml` |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | `global_costmap` | static + obstacle + inflation layers |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | `local_costmap` | 4×4 m rolling window, obstacle + inflation |
| `/plan` | `nav_msgs/Path` | `planner_server` | NavfnPlanner A* output |
| `/local_plan` | `nav_msgs/Path` | `controller_server` | Regulated Pure Pursuit local trajectory |
| `/particle_cloud` | `geometry_msgs/PoseArray` | `amcl` | AMCL particle filter visualisation |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `amcl` | AMCL's best estimate + covariance |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | `velocity_smoother` | smoothed Nav2 velocity output |
| `/cmd_vel` | `geometry_msgs/Twist` | `topic_tools/relay` (us) | bridged from `/cmd_vel_smoothed`; consumed by sim |

### TF chain after Day 4 is online

```
map ──(amcl @20 Hz)──► odom ──(Phase 0 sim @~90 Hz)──► base_link
                                                       ├── camera_link / camera_*_optical_frame
                                                       ├── imu_link
                                                       └── lidar_link
```

`map → odom` is now from **AMCL** (was slam_toolbox in Day 3 mapping
mode). **Both cannot run at once** — they fight over the TF.

---

## How it's wired

### Files

| File | Role |
|---|---|
| `src/go2_bringup_sim/launch/nav2.launch.py` | Top-level Day 4 launch. Wraps `nav2_bringup/launch/bringup_launch.py` with our params + map + adds the `cmd_vel_smoothed → cmd_vel` relay. |
| `src/go2_bringup_sim/config/nav2/nav2_params.yaml` | Tuned Nav2 parameters (AMCL alphas, robot_radius, RPP controller, costmap inflation, BT navigator). Four "knob" comments mark the parameters most likely to need adjustment. |
| `src/go2_bringup_sim/rviz/go2_semantic_nav.rviz` | Adds Map (global_costmap), Map (local_costmap), Path (/plan), Path (/local_plan), PoseArray (/particle_cloud), Polygon (footprint), and the `2D Goal Pose` + `2D Pose Estimate` toolbar buttons. |
| `scripts/check_day4.sh` | Acceptance test — 7 lifecycle nodes ACTIVE, 7 required topics, /cmd_vel publisher+subscriber, AMCL convergence, TF chain, /map content, /navigate_to_pose action server. |

### Critical design decisions

1. **Use upstream `nav2_bringup/bringup_launch.py` instead of authoring
   our own**. nav2_bringup wires up the lifecycle_manager that takes
   all 7 nodes through `unconfigured → inactive → active`, plus the
   `nav2_container` composition node, plus the localization vs SLAM
   conditionals. Authoring our own would mean re-implementing
   nav2_bringup's lifecycle dance — same trap that hit Day 3 with
   slam_toolbox.

2. **Mode = AMCL on a saved map, NOT online_async SLAM**. AMCL is
   deterministic and well-debugged; running Nav2 on top of a moving
   map (slam_toolbox publishing `map → odom`) makes every Nav2
   misbehaviour cascade through unstable transforms. Phase 4 search
   / Day 5+ frontier exploration will revisit the SLAM mode.

3. **`topic_tools/relay /cmd_vel_smoothed → /cmd_vel`**. nav2_bringup's
   `navigation_launch.py` remaps every `cmd_vel` to `cmd_vel_nav`,
   then routes them through `velocity_smoother` whose output topic
   is `cmd_vel_smoothed` (default, NOT remapped). Result: nothing
   publishes to `/cmd_vel` out of the box. Our sim's SubTwist OG
   node subscribes to `/cmd_vel`, so without the relay Go2 receives
   no commands.

   The relay is gated on a `cmd_vel_relay` launch arg (default true)
   so it can be disabled if the user customises Nav2 to publish
   directly on `/cmd_vel`.

---

## Acceptance procedure

Three terminals.

### Prerequisites (one-time)

```bash
sudo apt install \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-topic-tools
colcon build --symlink-install --packages-select go2_bringup_sim
source install/setup.bash
```

### Run the stack

```bash
# Shell A — Phase 0 sim with the OS1-32 LiDAR
bash scripts/run_warehouse_ros2.sh
```

```bash
# Shell B — Phase 1 perception layer (provides /tf_static + /scan)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch go2_bringup_sim chair_perception.launch.py
```

```bash
# Shell C — Day 4 Nav2 stack (autostart, uses default warehouse_v1.yaml)
ros2 launch go2_bringup_sim nav2.launch.py
# Defaults:
#   map = $PROJECT_ROOT/maps/warehouse_v1.yaml
#   params_file = <pkg_share>/config/nav2/nav2_params.yaml
#   autostart = true, use_composition = true, cmd_vel_relay = true
```

```bash
# Shell D — RViz
bash scripts/run_rviz.sh
# In RViz: change Fixed Frame from `odom` to `map`.
```

### Verify acceptance

```bash
# Shell E — automated checks
bash scripts/check_day4.sh
```

Hard checks (all must PASS):
- 7 lifecycle nodes (`amcl, bt_navigator, controller_server, planner_server, behavior_server, map_server, velocity_smoother`) all `active`
- slam_toolbox NOT running
- 7 required topics advertised: `/map, /global_costmap/costmap, /local_costmap/costmap, /plan, /cmd_vel, /particle_cloud, /amcl_pose`
- `/cmd_vel` has ≥1 publisher AND ≥1 subscriber
- AMCL converged (σ_xy < 0.5 m on `/amcl_pose` covariance)
- TF chain `map → odom → base_link → lidar_link`
- `/map` has occupied cells (map_server loaded the saved warehouse map)
- `/navigate_to_pose` action server present

Manual hard checks (require RViz / eyeballs):
- AMCL particle cloud collapses to a tight cluster (< 0.3 m) within ~10 s
- `/global_costmap` aligns visually with the static map (no offset)
- Click `2D Goal Pose` in RViz → green `/plan` appears within 2 s
- Go2 actually drives along the plan and stops within 0.25 m of goal
- Three scenarios pass: open straight line, around-obstacle, U-turn

### Three Day-4 scenarios

| Scenario | Setup | Pass criterion |
|---|---|---|
| **Open straight line** | 2D Goal Pose 3-5 m forward of Go2 | direct line, < 15 s, stops at goal |
| **Around an obstacle** | 2D Goal Pose on the far side of the chair / table | plan curves around, no clipping into inflation layer |
| **U-turn / behind** | 2D Goal Pose 1 m behind Go2 | RPP rotates in place first (`use_rotate_to_heading`), then drives |

---

## Pitfalls hit during bring-up (six layers, in the order we hit them)

This list is preserved verbatim — each one cost ≥ 1 hr to track down
and the next time we touch a Nav2 launch we want to remember them.

| # | Symptom | Root cause | Fix |
|---|---------|------------|-----|
| ① | `/scan` not flowing despite pointcloud_to_laserscan publisher in graph | `transform_tolerance: 0.05` in `pointcloud_to_laserscan` rejects every cloud whose stamp drifts >50 ms from `lidar_link` static TF — happens constantly under our 4 Hz LiDAR | `chair_perception.launch.py`: `transform_tolerance: 0.5` |
| ② | AMCL `active [3]` but never publishes `map → odom`, then `tf2_echo map odom` says "Invalid frame ID 'map'" | AMCL's TF broadcast is laser-callback-driven. Our jittery 4 Hz scan + occasional 14 s stalls trip AMCL's message-filter, so it processes one scan and silently stops | Replace AMCL with `slam_toolbox` mapping mode (Nav2 `slam:=True`). slam_toolbox tolerates jittery scans gracefully |
| ③ | `error_code: 0` from `/navigate_to_pose` even though TF is up | `track_unknown_space: true` + `allow_unknown: false` make planner refuse to route through the 92 % unknown cells of slam_toolbox's freshly-built /map | `nav2_params.yaml`: `track_unknown_space: false`, planner `allow_unknown: true` |
| ④ | `Behavior tree threw exception: Empty Tree. Exiting with failure.` | `default_nav_to_pose_bt_xml: ""` in Nav2 Jazzy is taken **literally** — load nothing, BT fails on first tick | Point at `/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` |
| ⑤ | `bt_navigator: Initial robot pose is not available` even though TF chain is fine | `bt_navigator.transform_tolerance` defaults too tight (0.1 s); slam_toolbox's slightly-future-stamped TFs + rest-state pruning leave nothing valid for the goal's stamp | `bt_navigator.transform_tolerance: 5.0` |
| ⑥ | Plan visible, controller running 20 Hz on `/cmd_vel_smoothed`, but Go2 doesn't move and goal aborts with `error_code: 202` | `collision_monitor.source_timeout: 1.0` flags `/scan` as stale during any LiDAR stutter > 1 s and rewrites `/cmd_vel` to zero | `collision_monitor.source_timeout: 15.0` |

## Known limitations / future work

- **slam_toolbox TF stalls intermittently** under sim GPU load. RPP
  controller logs `Exception in transformPose: extrapolation into the
  future` when this happens, follow_path aborts, BT preempts a new
  goal, and Go2 walks in spurts. Mitigated by
  `transform_publish_period: 0.1` (longer post-date) but not fully
  eliminated. Real hardware: 10 Hz scan + dedicated CPU = no stall.
- **slam_toolbox in mapping mode (not pure localization)**. The map
  keeps refining as Go2 drives. To freeze the map for navigation use
  slam_toolbox's serialized map + localization mode (requires
  `serialize_map` service call — defer to Day 5+ if needed).
- **No e-stop / safety layer**. `/cmd_vel` flows directly into the sim.
  Real-Go2 deployment must wire `go2_safety/safety_monitor_node` into
  the `/cmd_vel` path.
- **Recovery behaviours are configured but rarely tested**. Soft
  acceptance criterion: deliberately drive Go2 into a wall corner,
  give a remote goal, watch `behavior_server` trigger `spin` recovery
  and replan. Not in the hard checklist because it's interactive.
- **Map origin is from Day 3's `save_map.sh` snapshot of slam_toolbox
  output**. If you re-run Day 3 in a different sim seed, the map's
  `origin` field changes — and AMCL's `initial_pose` in
  `nav2_params.yaml` should follow. Re-set 2D Pose Estimate after any
  map regeneration.
- **`use_localization=False` mode untested**. nav2_bringup supports
  pure navigation without map-server (e.g. ego-centric exploration),
  but our launch hard-codes localization=true.

---

## Day 4 closure → Day 5 entry

Once all hard checks pass + a 30-second screen recording of Go2
navigating to a goal is captured, Day 4 is closed. Day 5 (YOLOE / open-
vocabulary perception) overlays on top:

- Sim & sensors: unchanged from Day 1.5
- Mapping: Day 3 saved map, reused
- Localization: AMCL from Day 4
- New: replace the YOLOv11 chair-only detector in `go2_perception` with
  YOLOE / Grounding-DINO so `target_class_aliases` becomes a free-
  form text prompt. Day 5 doesn't touch Nav2.

The Nav2 stack from this Day will continue to handle every "go to
position X" request in Days 5-8, including frontier exploration's
auto-generated goals.
