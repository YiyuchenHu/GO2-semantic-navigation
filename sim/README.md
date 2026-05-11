# `sim/` — Isaac Sim scene & ROS 2 bridge for Go2 semantic navigation

Pure Isaac Sim Python lives here. **Nothing in this folder is a ROS 2
package**, so `colcon` does not touch it. The ROS 2 packages under `src/`
are a separate concern.

For the Phase 0 status write-up (what is finished, what is still pending,
validation evidence, how to run it), see
[`docs/phase0_status.md`](../docs/phase0_status.md). This README only
describes the files in this directory.

## Files

| File | Purpose |
|------|---------|
| `warehouse_scene.py` | Library: programmatic construction of the 10×10 m warehouse (floor, walls, Go2, table, chair). No CLI, does not boot Kit. |
| `build_go2_warehouse.py` | Thin CLI wrapper that boots `SimulationApp`, calls `warehouse_scene.build_full_warehouse(...)`, and exports a flattened USD to `sim/worlds/`. |
| `run_go2_warehouse_ros2.py` | Phase 0 runtime: builds the warehouse on a fresh stage, attaches an RGB-D camera + IMU, creates the `ROS2ActionGraph`, runs the kinematic `/cmd_vel` driver. This is what `scripts/run_warehouse_ros2.sh` launches. |
| `worlds/` | Saved USDs from `build_go2_warehouse.py` live here. Not required by `run_go2_warehouse_ros2.py`. |

## How to use

Everything below assumes `ISAAC_SIM_ROOT` is set (either via
`scripts/dev_env.sh` or exported manually) and that `python.sh` exists
under it.

### Phase 0 simulator + ROS 2 bridge (the main flow)

```bash
bash scripts/run_warehouse_ros2.sh          # GUI
bash scripts/run_warehouse_ros2.sh --headless   # headless (see caveat in script)
```

The wrapper sets `ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
and adds Isaac Sim's bundled Jazzy bridge libs to `LD_LIBRARY_PATH`
before Kit boots, then execs `run_go2_warehouse_ros2.py`.

See `docs/phase0_status.md` for the full list of topics, validation
commands, and expected boot-time log lines.

### Export a static USD of just the world (optional)

```bash
$ISAAC_SIM_ROOT/python.sh sim/build_go2_warehouse.py              # headless
$ISAAC_SIM_ROOT/python.sh sim/build_go2_warehouse.py --no-headless
```

Default output:

```
sim/worlds/go2_warehouse_10x10.usd
```

This is convenient for visual inspection in Isaac Sim's GUI via
`File > Open…`. **`run_go2_warehouse_ros2.py` does NOT consume this
file** — the runtime builds the scene from scratch on every launch,
because `reopenUsd()` on the saved warehouse was crashing
`libomni.graph.image.core.plugin.so` on Isaac Sim 5.1 + RTX 5090.

## Scene layout (authoritative values in `warehouse_scene.py`)

- 10×10 m floor centred at the world origin; default Isaac ground plane.
- Walls 2.5 m tall, 0.2 m thick. **Fully enclosed** (no doorway). Toggle
  `ENCLOSED_ROOM` in the module to reintroduce a gap on the +X wall.
- Dome light under `/World/Lights/Dome`.
- `/World/Go2` at `(-4.0, -4.0, 0.55)` yaw `+45°`.
- `/World/Table` at `(1.5, 1.0)` — `EastRural_Table` reference.
- `/World/Chair` at `(2.7, 1.0)` yaw `180°` — `EastRural_Chair` reference
  (scaled to `0.5` because the ArchVis asset ships oversized).

The earlier `Person` and `Cup` entries were **removed** for MVP
stability and are no longer referenced anywhere in this folder.

## Asset strategy (fallback-first)

The builder tries the USDs listed in `ASSET_CANDIDATES` inside
`warehouse_scene.py`. Candidates can be either paths relative to
`get_assets_root_path()` (NVIDIA Nucleus) or absolute URLs
(`http(s)://…`, `omniverse://…`, `file://…`, or an absolute local path).
If every candidate fails, the builder falls back to a simple primitive so
the pipeline keeps running.

| Object | Primary candidate | Fallback |
|--------|-------------------|----------|
| Go2 | `Isaac/Robots/Unitree/Go2/go2.usd` (Nucleus) | Blue capsule under `/World/Go2/placeholder_body` |
| Table | `EastRural_Table.usd` (S3 URL) | 1.2 × 0.6 × 0.75 m brown box |
| Chair | `EastRural_Chair.usd` (S3 URL) | 0.5 × 0.5 × 0.45 m black cube |
| Floor | Isaac `add_default_ground_plane` | — |
| Walls | Always `FixedCuboid` primitives | — |

## Runtime hooks created by `run_go2_warehouse_ros2.py`

These prim paths are created live when the ROS 2 runtime launches. They
are **not** present in the saved USD produced by `build_go2_warehouse.py`.

```
/World/Go2/base/front_cam   # RGB-D camera (USD Camera)
/World/Go2/base/imu         # IsaacImuSensor prim + registered IMUSensor wrapper
/World/ROS2ActionGraph      # OmniGraph: PubClock / ReadIMU / PubIMU /
                            #            ComputeOdom / PubOdom / PubRawTF /
                            #            CamRGB / CamDepth / CamInfo /
                            #            SubTwist / Ctx / SimTime / RunOne
```

Topic wiring, exec graph, and Phase 0 validation are described in
`docs/phase0_status.md`.

## Known limitations (Phase 0 scope)

- `/cmd_vel` drives the articulation kinematically via
  `SingleArticulation.set_world_pose(...)`. There is **no walking
  policy** — legs do not animate.
- Gravity is **disabled** on the Go2 articulation while in Phase 0. With
  no gait controller, leaving gravity on would collapse the articulation
  and invalidate every platform observation. Re-enable it when a gait
  controller is introduced.
- The NVIDIA Go2 USD has `primvar 'st'` size-mismatch warnings on Isaac
  Sim 5.1 / RTX 5090 which cause Hydra to drop a few visual meshes. This
  is cosmetic only — physics, articulation, sensors and ROS topics are
  unaffected.
- IMU pipeline integrity (topic advertised + messages flowing) is
  validated. IMU **numerical fidelity** has not been characterized;
  downstream code that relies on accurate IMU values should re-validate
  before trusting them.

## Editing the scene

All layout constants live at the top of `warehouse_scene.py`
(`ROOM_SIZE`, `WALL_HEIGHT`, `GO2_SPAWN_XYZ`, `TABLE_XYZ`, `CHAIR_XYZ`,
`CHAIR_SCALE`, `ENCLOSED_ROOM`, `DOORWAY_WIDTH`, `DOORWAY_CENTER_Y`,
etc.). Change them there and rerun either `build_go2_warehouse.py` (for
the static export) or `scripts/run_warehouse_ros2.sh` (for the live
Phase 0 runtime).
