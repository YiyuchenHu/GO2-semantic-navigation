# Day 3 — 2D SLAM with slam_toolbox

**Status: scaffolded, awaiting first end-to-end run.**

This document is the source of truth for the SLAM bring-up. It pairs
with `docs/phase0_status.md` (sim platform), `docs/phase1_status.md`
(perception), and the **Day 1.5 LiDAR add-on** (RTX OS1-32 + 360°
PointCloud2 + pointcloud_to_laserscan → /scan).

---

## What Day 3 delivers

### Topics

| Topic | Type | Source | Rate | Notes |
|---|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | ~1 Hz | TRANSIENT_LOCAL — late RViz still gets the latest |
| `/map_metadata` | `nav_msgs/MapMetaData` | `slam_toolbox` | latched | width / height / resolution / origin |
| `/map_updates` | `map_msgs/OccupancyGridUpdate` | `slam_toolbox` | ad-hoc | incremental patches |
| `/tf` (`map → odom`) | `tf2_msgs/TFMessage` | `slam_toolbox` | 20 Hz | the SLAM correction the rest of the stack hangs off |

### TF chain after Day 3 is online

```
map ──(slam_toolbox @ 20 Hz)──► odom ──(Phase 0 sim @ ~90 Hz)──► base_link
                                                                ├── camera_link / camera_color_optical_frame / camera_depth_optical_frame
                                                                ├── imu_link
                                                                └── lidar_link
```

`map → odom` is what makes the rest of the navigation stack tolerant
of odom drift: any node that consumes 3D entity positions in `map`
sees a globally-consistent location even as `odom → base_link`
slides around.

---

## How it's wired

### Inputs
- `/scan` (LaserScan) from `pointcloud_to_laserscan` (running inside
  `chair_perception.launch.py`), itself fed by `/lidar/points`
  (PointCloud2 from the Isaac Sim RTX OS1-32 sensor).
- `/odom` and `odom → base_link` TF, from the sim's
  `IsaacComputeOdometry` node.
- `/clock` from the sim — `use_sim_time:=true` is set by
  `mapping.launch.py` so slam_toolbox queries TF at sim time, not
  wall time.

### Components
| File | Role |
|---|---|
| `src/go2_bringup_sim/launch/mapping.launch.py` | Top-level Day 3 launch. Includes `chair_perception.launch.py` (gets /scan + /tf_static) and starts `slam_toolbox::async_slam_toolbox_node` with the project YAML. |
| `src/go2_bringup_sim/config/slam/slam_toolbox_mapping.yaml` | Tuned slam_toolbox parameters for the OS1-32 + 10×10 m enclosed warehouse + ~5 Hz scan rate (see in-file comments). |
| `src/go2_bringup_sim/rviz/go2_semantic_nav.rviz` | Adds a `Map` display subscribed to `/map` with TRANSIENT_LOCAL durability. |
| `scripts/check_day3.sh` | Acceptance test — `/map` alive, has nontrivial size, contains both free + occupied cells, `map → odom` and `map → lidar_link` TFs resolve, slam_toolbox node visible. |
| `scripts/save_map.sh` | Wraps `nav2_map_server map_saver_cli`. Output goes under `maps/` (gitignored). |

### Mode choice — why `online_async`

slam_toolbox ships two `*_slam_toolbox_node` entrypoints:
- **`sync_slam_toolbox_node`** — every scan blocks on its TF lookup;
  any render-rate dip in the sim makes it spew "Could not transform"
  warnings and miss scans entirely.
- **`async_slam_toolbox_node`** — the scan callback queues the work;
  the matcher runs on its own thread; transform lookups have a
  longer tolerance.

Our Isaac Sim setup runs at ~90 Hz playback but the LiDAR's RTX
render product effectively halves to ~45 Hz, then the
SimulationGate.step throttle pulls /lidar/points down to ~5 Hz
with full 360° scans. The variance on those handoffs is exactly the
class of jitter `online_async` was built for.

---

## Acceptance procedure

Three shells.

```bash
# Shell A — Phase 0 sim with the OS1-32 LiDAR
bash scripts/run_warehouse_ros2.sh
```

```bash
# Shell B — Day 3 mapping stack (perception + slam_toolbox)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch go2_bringup_sim mapping.launch.py
```

```bash
# Shell C — drive the Go2 around to fill the map.
# Slow + small loops first; come back to the spawn corner so loop
# closure has a chance to fire before declaring success.
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.4}, angular: {z: 0.2}}' --rate 10
# ... several minutes of figure-8 driving ...
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0}}' --once
```

```bash
# Shell D — verify Day 3 acceptance
bash scripts/check_day3.sh
```

Hard checks (all must PASS):
- `/map` advertised
- `/map` size ≥ 50 × 50 cells
- `/map` contains both free (== 0) and occupied (≥ 50) cells
- `tf2_echo map odom`, `map base_link`, `map lidar_link` all resolve
- `slam_toolbox` node visible in `ros2 node list`

Soft checks (warn only):
- `/map_metadata`, `/map_updates` advertised
- `/map` rate ≈ 1 Hz

### Visual acceptance (RViz)

```bash
bash scripts/run_rviz.sh
```

Switch `Fixed Frame` from `odom` to `map` in Global Options. Expected:
- The `Map (SLAM)` display fills with a cleaner version of the
  PointCloud2 outline: white = free space, black = walls / chair /
  table, grey = unobserved.
- Walls appear straight (no curved or doubled-up versions).
- Returning to the spawn corner should NOT produce a "ghost wall"
  parallel to the original.
- Driving the Go2 forward then back should produce minimal movement
  in the `map → odom` transform (the loop-closure correction is
  already integrating odometry drift).

### Save the map

After visual acceptance is clean:

```bash
bash scripts/save_map.sh warehouse_v1
# writes maps/warehouse_v1.{pgm,yaml}
```

`maps/warehouse_v1.yaml` is what Day 4's Nav2 `map_server` will load.

---

## Known limitations

- **Walls only at chair height.** The OS1-32 has a ±22.5° vertical
  span. With the LiDAR mounted at z = 0.75 m world (Go2 base 0.55 m
  + 0.20 m mount), the lower rays sweep the floor at ~1.8 m radius
  and the upper rays sweep ~1.8 m up at the same radius. Anything
  significantly below that (low chair seats?) or above (ceiling
  features) won't reliably register on /scan. For our warehouse this
  doesn't matter — walls are 2 m tall — but a denser furniture
  scene might confuse SLAM.
- **No IMU fusion.** `/imu/data` is published but slam_toolbox in
  this config doesn't consume it. Adding `robot_localization`'s
  EKF as a `odom_combined` step in Day 4 is plausible if odom
  drift becomes a problem. For the kinematic /cmd_vel integrator,
  drift is negligible.
- **Single-floor, static-world only.** No multi-floor support, no
  dynamic obstacles. Our Go2 + chair + table scene satisfies this.
- **Loop closure search radius = 8 m.** Hard-coded in the YAML.
  The warehouse diagonal is ~14 m, but Go2 typically drives small
  loops, so 8 m suffices. Bump up if you want round-trip-the-room
  loops.

---

## What's next

Day 4 will swap out the `simple_p_controller` backend in
`nav_executor_node` for a Nav2 `bt_navigator` + `controller_server`
+ `planner_server` stack, fed by `/map` (latched) and
`/scan` (live), producing `nav2_msgs/NavigateToPose` actions. The
Day 3 saved map becomes the static input to Nav2's `map_server`,
and slam_toolbox can be left running in the background to keep
correcting `map → odom` while Nav2 plans on the static cell grid.
