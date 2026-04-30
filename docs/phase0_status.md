# Phase 0 — Simulation & ROS 2 Platform Layer

**Status: complete.** The simulator + Go2 platform + ROS 2 Jazzy bridge are
stable enough to hand off to Phase 1 (chair-only semantic perception).

This document is the single source of truth for what Phase 0 actually
delivered. It is deliberately scoped to platform / infrastructure — it does
not describe perception, semantic memory, planning, or navigation behavior,
because those are Phase 1+ work.

---

## Sources of truth

This status document was written from the following in-repo artifacts and
recent runtime validation:

| Source | Role |
|---|---|
| `sim/run_go2_warehouse_ros2.py` | Runtime entry point; builds the scene, attaches sensors, constructs the ROS 2 action graph, runs the kinematic `/cmd_vel` driver. |
| `sim/warehouse_scene.py` | Programmatic scene library (floor, walls, Go2 spawn, table, chair). |
| `sim/build_go2_warehouse.py` | Thin CLI for exporting a flattened USD of the scene. |
| `scripts/run_warehouse_ros2.sh` | Launch wrapper that sets `ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and the bridge `LD_LIBRARY_PATH`. |
| Recent terminal validation | `ros2 topic info -v` / `ros2 topic hz` / `ros2 topic echo` confirmations against a live warehouse session. |

Residual uncertainty:

- IMU topic flow (`/imu/data`) is confirmed at the pipeline level (topic is
  advertised, messages are emitted on the bridge). The **numerical quality**
  of IMU readings has not been characterized; downstream code that actually
  depends on accurate IMU values should re-validate that aspect before trust.
- The Go2 asset has known cosmetic mesh/primvar issues on Isaac Sim 5.1 +
  RTX 5090 (Hydra rejects a few `st` primvars). This is a rendering-only
  issue; physics, articulation and topic pipelines are unaffected.

Everywhere Phase 0 status is unclear, this document marks it explicitly.

---

## What Phase 0 delivered

### Scene

- 10×10 m, fully enclosed warehouse built programmatically on a fresh stage.
- Default Isaac ground plane + 4 walls + dome light.
- Referenced props: `EastRural_Table` and `EastRural_Chair`.
- Go2 spawned at `(-4.0, -4.0, 0.55)` yaw `+45°` under `/World/Go2`.
- The legacy Person and Cup from earlier drafts are **removed** for MVP
  stability; `sim/warehouse_scene.py` is the authoritative scene definition.

### Robot platform

- Go2 USD reference under `/World/Go2`.
- Articulation root at `/World/Go2/base` is bound via
  `isaacsim.core.prims.SingleArticulation` and reachable through
  `set_world_pose(...)`.
- Gravity is **intentionally disabled** on the articulation. Phase 0 does
  not include a walking policy, so with gravity on the un-driven leg joints
  would collapse and invalidate every platform observation. Disabling
  gravity is the Phase 0 equivalent of "freeze the robot upright". This
  will be revisited when a gait controller is introduced.
- A front-facing RGB-D camera prim is created under
  `/World/Go2/base/front_cam`.
- An IMU sensor is created under `/World/Go2/base/imu` and registered with
  the `isaacsim.sensors.physics` sensor interface via an `IMUSensor` Python
  wrapper (kept alive at module scope so the C++ backend doesn't drop it).

### ROS 2 bridge (OmniGraph)

A single action graph is built at `/World/ROS2ActionGraph`, wired as:

- `OnPlaybackTick.tick` drives each per-frame node directly:
  - `PubClock` (for `/clock`)
  - `ReadIMU` (latest-data mode, no buffer dependence)
  - `ComputeOdom`
  - `SubTwist` (for `/cmd_vel`)
  - `PubIMU` (for `/imu/data`)
- `ReadIMU.execOut` and `ComputeOdom.execOut` only cascade into the
  downstream publishers that care about data validity (i.e. Odom / TF data
  ports, IMU data ports). Execution gating for `PubIMU` itself is
  `OnPlaybackTick` to guarantee the topic advertises and streams at the
  playback rate.
- `OgnIsaacRunOneSimulationFrame` is a **one-shot** helper here — it is
  used *only* to bind the three `ROS2CameraHelper` nodes to the render
  product once at startup (which is exactly what those helpers need).
- `ROS2Context` is wired into every publisher/subscriber's `context` port.
- `IsaacReadSimulationTime.simulationTime` feeds `timeStamp` on every
  publisher so messages carry sim time.
- USD relationships for `ReadIMU.imuPrim` and `ComputeOdom.chassisPrim`
  are written directly via `Usd.Relationship.SetTargets([...])`, because
  setting target-typed inputs via the generic `og.Controller.attribute(...)
  .set(...)` path does not persist across `world.reset()` on Isaac 5.1.

### Published topics

| Topic | Type | Direction | Source node |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | pub | `PubClock` |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | pub | `CamRGB` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | pub | `CamDepth` |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | pub | `CamInfo` |
| `/imu/data` | `sensor_msgs/msg/Imu` | pub | `PubIMU` |
| `/odom` | `nav_msgs/msg/Odometry` | pub | `PubOdom` |
| `/tf` | `tf2_msgs/msg/TFMessage` | pub | `PubRawTF` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | sub | `SubTwist` |

All topic names are configured in `build_action_graph()` in
`sim/run_go2_warehouse_ros2.py`.

### `/cmd_vel` driver

`CmdVelDriver` reads the latest twist from `SubTwist.outputs:linearVelocity`
/ `outputs:angularVelocity` each simulation step, integrates body-frame
velocity into a world pose, and applies it to the articulation root via
`SingleArticulation.set_world_pose(...)`. This is a deliberately simplified
kinematic driver for Phase 0 validation — it is **not** a quadruped gait
controller.

### Stability / boot robustness

The following were observed and paid for during Phase 0 bring-up and are
now baked into the runtime:

- Scene is built fresh on every run — `reopenUsd()` on a saved warehouse
  USD was reliably crashing `libomni.graph.image.core.plugin.so` on Isaac
  5.1 + RTX 5090.
- The ROS 2 bridge extension's bundled Jazzy C++ libs are added to
  `LD_LIBRARY_PATH` **before** Kit boots (done by
  `scripts/run_warehouse_ros2.sh`).
- `/exts/isaacsim.ros2.bridge/publish_without_verification` is set to
  `True` at boot so publishers fire as soon as their OG node ticks,
  without waiting for a DDS subscriber handshake.

---

## Validation evidence

The following checks were run against a live session and all pass:

- `ros2 topic info /clock -v` — Publisher count = 1, owner node
  `_World_ROS2ActionGraph_PubClock`.
- `ros2 topic info /odom -v` — Publisher count = 1, owner node
  `_World_ROS2ActionGraph_PubOdom`.
- `ros2 topic info /tf -v` — Publisher count = 1, owner node
  `_World_ROS2ActionGraph_PubRawTF`.
- `ros2 topic info /imu/data -v` — Publisher count = 1, owner node
  `_World_ROS2ActionGraph_PubIMU`.
- `ros2 topic hz /clock` / `/odom` / `/tf` / `/imu/data` all show steady
  message flow at approximately the playback tick rate.
- `ros2 topic echo /clock --once`, `ros2 topic echo /odom --once`,
  `ros2 topic echo /tf --once`, and `ros2 topic echo /imu/data --once`
  each return a valid, type-correct message.
- `/camera/color/image_raw`, `/camera/depth/image_rect_raw` and
  `/camera/color/camera_info` are advertised and carry data at the camera
  frame-skip-adjusted rate.
- A `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x:
  0.3}}"` command drives the articulation root forward; `/odom` reflects
  the new pose in real time.
- The Isaac Sim session remains stable over extended runtime (hundreds of
  sim-seconds) without crashes, re-initializations, or topic dropouts.

---

## Current limitations

Everything below is **deliberately deferred** to later phases.

### Scope not yet implemented

- No chair detection or any 2D/3D perception.
- No semantic segmentation, grounding, or object descriptors.
- No semantic memory / entity store pipeline.
- No target selection, goal planning, or navigation behavior.
- No task coordinator loop.
- No "go to the chair" end-to-end behavior.

### Known platform-layer nuances

- `/cmd_vel` is wired into a **kinematic** pose driver, not a gait
  controller. The robot translates and yaws as a rigid articulation; its
  legs do not animate. This is sufficient for Phase 0 but is not final
  locomotion behavior.
- Gravity is disabled on the Go2 articulation during Phase 0. It must be
  re-enabled before any work that depends on physically realistic leg /
  ground contact behavior.
- `/imu/data` is part of the verified bridge pipeline. The **numerical
  content** (linear acceleration / angular velocity / orientation) has not
  been characterized beyond "messages are flowing with the right schema".
  Downstream consumers that need accurate IMU values should re-validate
  before trusting them.
- The Go2 USD has Hydra `primvar 'st'` size-mismatch warnings on Isaac
  Sim 5.1; a few visual meshes are dropped by the renderer. This is
  cosmetic — physics, articulation, sensors and ROS topics are unaffected.
- The ROS 2 `/cmd_vel` subscriber path is validated at the simulation end,
  but the `src/` ROS 2 packages in this repository (perception, safety,
  task coordination, etc.) are **not** part of Phase 0 and have not been
  validated against the simulator yet.

---

## Next step — Phase 1: chair-only semantic perception

Phase 1 scope should be intentionally narrow:

1. Consume `/camera/color/image_raw` and `/camera/depth/image_rect_raw`
   from this simulator.
2. Detect **only chairs** (2D bounding boxes or mask, whichever is
   simplest to validate).
3. Back-project chair detections into the `odom`/`tf` frame to produce a
   coarse chair pose observation.
4. Publish those chair observations as a ROS 2 topic for downstream
   consumption.
5. Validate chair perception against the existing warehouse scene, which
   contains exactly one `EastRural_Chair`.

Phase 1 should **not** yet include target selection, navigation, or task
orchestration — those belong to later phases once chair perception is
trustworthy.

---

## Running Phase 0

### Prerequisites

- Isaac Sim 5.1 installed (this repo assumes `~/isaacsim_5.1_backup` via
  `scripts/dev_env.sh`; override by exporting `ISAAC_SIM_ROOT` before
  launching).
- ROS 2 Jazzy installed on the host.
- A machine with a CUDA-capable GPU. This has been validated on an
  RTX 5090.

### Launch the simulator + ROS 2 bridge

From the repository root:

```bash
bash scripts/run_warehouse_ros2.sh
```

Expected stdout markers during a healthy boot:

- `[run_warehouse_ros2] ROS_DISTRO=jazzy RMW=rmw_fastrtps_cpp`
- `[run_ros2] carb: publish_without_verification=True`
- `[run_ros2] Scene built.`
- `[run_ros2] Camera created at /World/Go2/base/front_cam`
- `[run_ros2] IMU registered with sensor interface at /World/Go2/base/imu`
- `[run_ros2] target rels written  ReadIMU.imuPrim=...  ComputeOdom.chassisPrim=...`
- `[run_ros2] CmdVelDriver: articulation gravity DISABLED (Phase 0 / no walking policy)`
- `[run_ros2] /cmd_vel driver attached (articulation). Topic: /cmd_vel`
- `[run_ros2] Ready. Publishing ROS 2 topics. Ctrl-C to stop.`

Extra flags are forwarded verbatim to `sim/run_go2_warehouse_ros2.py`,
e.g.:

```bash
bash scripts/run_warehouse_ros2.sh --rgb-resolution 640x480
bash scripts/run_warehouse_ros2.sh --diag after-build
```

### Verify topics (second terminal)

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic list | sort
```

Expected rows that belong to this simulator:

```
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/image_rect_raw
/clock
/cmd_vel
/imu/data
/odom
/tf
```

Per-topic checks — let each `hz` call run for ~10 s before interrupting:

```bash
ros2 topic info /clock     -v
ros2 topic info /odom      -v
ros2 topic info /tf        -v
ros2 topic info /imu/data  -v

timeout 12 ros2 topic hz /clock
timeout 12 ros2 topic hz /odom
timeout 12 ros2 topic hz /tf
timeout 12 ros2 topic hz /imu/data

ros2 topic echo /clock    --once
ros2 topic echo /odom     --once
ros2 topic echo /tf       --once
ros2 topic echo /imu/data --once
```

### Drive the robot via `/cmd_vel`

In one terminal:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

In another:

```bash
ros2 topic echo /odom
```

You should see `pose.pose.position.x` increase over time while
`pose.pose.position.z` stays near the spawn height (~0.55 m).

---

## Phase 0 closure

Phase 0 is **closed** as far as this repository is concerned. Further work
on the simulator or ROS 2 bridge should only happen when Phase 1+ exposes a
concrete requirement that the current infrastructure cannot meet.

The repository is ready for Phase 1: chair-only semantic perception.
