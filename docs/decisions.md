# Design decisions

Short rationale entries — why we picked X over Y. One paragraph
each. Update as you make new architectural choices; this is the
"future me will thank current me" file.

---

## ADR-001: SLAM via `slam_toolbox`, not RTAB-Map

**Decision**: 2D `slam_toolbox` in `online_async` mode for Day 3
mapping.

**Alternatives considered**:

* **RTAB-Map** — full 3D SLAM with appearance-based loop closure.
  Heavier, needs RGB-D + IMU fusion, and ships its own visual odom.
* **Cartographer** — robust 2D/3D SLAM with strong loop closure.
  Build / install footprint is ~1 GB; documentation skews toward
  bag-file workflows rather than live ROS 2 streams.

**Why slam_toolbox**:

1. **2D is enough for a flat warehouse**. The scene has no stairs,
   ramps, or multi-floor geometry. A 2D occupancy grid plugs
   straight into Nav2's `costmap_2d` without a 3D→2D projection
   layer.
2. **Native ROS 2 lifecycle node**. `slam_toolbox/launch/online_async_launch.py`
   handles the configure→activate transitions for us; we found
   this out the hard way after first wiring it as a plain
   `launch_ros.actions.Node` and getting a silently-unconfigured
   node (Day 3 pitfall log).
3. **Tolerates jittery `/scan`**. The Isaac Sim RTX LiDAR drops to
   ~4 Hz under GPU contention (see `known_issues.md` #3).
   slam_toolbox's `online_async` swallows that without complaint.
   AMCL did not, which is what pushed us to slam_toolbox-as-
   localization in Day 4 too.
4. **Map saving is `nav2_map_server map_saver_cli`**, the same tool
   Nav2 uses on the consumer side. No format conversion.

**Risks accepted**:

* No vertical / staircase awareness. If the project ever moves
  beyond a single-floor warehouse, RTAB-Map becomes necessary.
* `slam_toolbox` mapping mode keeps refining the map as Go2 drives.
  Acceptable for sim demos; for a frozen reference map switch to
  the `localization_slam_toolbox_node` variant + a serialized
  `.posegraph`.

---

## ADR-002: Day 4 localization via `slam_toolbox` mapping mode, NOT AMCL

**Decision**: Nav2 launched with `slam:=True` (default) so
slam_toolbox publishes `/map` and `map → odom` simultaneously.
AMCL backend is supported via `slam:=False` but is the secondary
path.

**Alternatives considered**:

* **AMCL on the Day 3 saved map** — the textbook Nav2 setup.
  Lower CPU, frozen reference map, every Nav2 tutorial uses it.

**Why slam_toolbox over AMCL for Day 4**:

In testing, AMCL on our Isaac Sim setup did **not** publish
`map → odom` reliably. Symptom: AMCL lifecycle reports `active [3]`
but `tf2_echo map odom` says `Invalid frame ID 'map' - frame does
not exist`, intermittently. Root cause: AMCL's TF broadcast happens
inside its `laserReceived()` callback, gated by an internal
message-filter that drops scans whose stamps are older than the
latest TF in cache. Our 4 Hz Lidar with 14 s stalls trips that
filter; AMCL processes one scan, drops the rest, and stops
broadcasting.

slam_toolbox's `online_async_slam_toolbox_node` doesn't have that
fragility — it pulls scans, integrates them async, and publishes
`map → odom` from a separate timer at `transform_publish_period`.
Same input stream, fundamentally more robust output.

**Risks accepted**:

* Map keeps refining (see ADR-001 risks).
* Slightly higher CPU than AMCL, irrelevant on a workstation.
* If we ever want to demo "Go2 navigates a STATIC reference map"
  for a poster, we'll need to switch the backend or freeze the
  serialized map.

---

## ADR-003: Open-vocabulary detection via YOLOE, not GroundingDINO + SAM2

**Decision**: YOLOE-11s-seg as the Day 5 perception backbone.

**Alternatives considered**:

* **GroundingDINO + SAM2** — two-stage open-vocab pipeline.
  Best zero-shot accuracy, particularly on long-tail classes.
* **YOLO-World** — Tsinghua's earlier open-vocab YOLO. Now
  superseded by YOLOE (same authors).

**Why YOLOE**:

1. **Real-time on a 4060/4070-class GPU**. We measure ~14 Hz at
   1280×720 input on RTX 4060 Mobile, batched within YOLOE's
   internal preprocessing. GroundingDINO + SAM2 in our setup is
   ~2 Hz on the same input — adequate for offline analysis,
   inadequate for closed-loop nav.
2. **`set_classes()` API matches our control loop**. YOLOE
   compiles a text-prompt embedding once at `set_classes()` time;
   per-frame inference is then no slower than fixed-class YOLO.
   GroundingDINO recomputes the text features every call.
3. **Drop-in `ultralytics` integration**. `from ultralytics import
   YOLOE` mirrors the existing chair-only `YOLO` import; the
   project's existing `YoloBackend` graceful-fallback pattern
   transferred directly to `YoloeBackend`.
4. **Bundled segmentation**. YOLOE-seg variants emit instance
   masks alongside boxes, which Day 6+ can use for tighter median-
   depth reprojection without a second pass through SAM.

**Risks accepted**:

* Lower zero-shot accuracy than GroundingDINO on adversarial
  long-tail classes. Acceptable: the warehouse vocabulary is
  small (chair / table / box / etc.) and YOLOE's text encoder is
  CLIP-grade on common nouns.
* Requires `ultralytics>=8.3.0`; older installs fail import (we
  document this in `known_issues.md` and `requirements.txt`).
* MobileCLIP-Blt backbone download (~572 MB) on first run is a
  one-time annoyance; mitigated via README pre-flight step.

---

## ADR-004: MVP runs on bare Isaac Sim 5.1, not Isaac Lab

**Decision**: Day 1-5 use raw Isaac Sim 5.1 with our own scripted
warehouse + Go2 spawn + ROS 2 bridge (`sim/run_go2_warehouse_ros2.py`).

**Alternatives considered**:

* **Isaac Lab on top of Isaac Sim** — Nvidia's robotics learning
  framework. Provides pre-built `Isaac-Velocity-Flat-Unitree-Go2`
  environments with trained walking policies, gym-style API,
  multi-env parallelization.

**Why bare Isaac Sim for the MVP**:

1. **MVP scope is navigation, not locomotion**. We need a Go2 that
   moves under `/cmd_vel`, not one that learns to walk. The
   kinematic `set_world_pose` integrator in
   `KinematicLocomotionBackend` is enough; gait realism is a Day
   N+ concern.
2. **Isaac Lab adds 2-3 GB of dependencies**. Without a need for
   RL training infrastructure, the cost is unjustified.
3. **Direct USD scripting is the path of least resistance for
   sensor placement**. We control camera / LiDAR / IMU pose
   directly via PRIM mutation in
   `sim/run_go2_warehouse_ros2.py`, bypassing Isaac Lab's
   Manager-API abstraction. This made the (painful) Day 1-2
   sensor calibration work tractable.

**Risks accepted**:

* No real-walking gait. Documented in the original Phase 5
  scaffolding. Future work loads an Isaac Lab TorchScript
  checkpoint into `PolicyLocomotionBackend`.
* No environment parallelism. Irrelevant for nav demos.

---

## ADR-005: Two-stage perception in transition (legacy + open-vocab)

**Decision**: keep both Phase 1's `perception_node` (chair-only
YOLOv11) and Day 5's `yoloe_detector_node` in the tree until Day
7+. The two publish on disjoint topic namespaces (`/perception/...`
vs `/detections...`) so they coexist.

**Why both, for now**:

* The Phase 2-4 launches (`chair_semantic_memory.launch.py` etc.)
  consume `/perception/detections_2d`. Removing Phase 1 immediately
  would break those launches without a Day 6 replacement landing
  first.
* The legacy stack documents working examples of `go2_msgs/*`
  custom messages that we may want to study before Day 6 picks
  between vision_msgs and a custom msg for 3D output.

**Migration plan**:

* Day 6: write `depth_projector_node` consuming `/detections` →
  `/detections_3d` (or `vision_msgs/Detection3DArray` if we
  decide to standardise). Once Day 6 is verified, the
  Phase 2 `object_tracker_node` rewrites against the new topic.
* Day 7+: delete the legacy chair-only launches and the
  `perception_node` / `object_localizer_3d_node` pair.

This entry stays in the file until that pruning happens.

---

## ADR-006: `vision_msgs/Detection2DArray` for the Day 5 output, not custom `go2_msgs/...`

**Decision**: `yoloe_detector_node` publishes upstream
`vision_msgs/Detection2DArray`, not the project's custom
`go2_msgs/Detection2DArray`.

**Why**:

1. **RViz default plugin** — `vision_msgs_rviz_plugins` ships in
   apt; visualising bboxes on top of a camera image is
   "tick-the-display" territory rather than "write a custom
   marker pipeline".
2. **Forward compatibility with non-MVP perception nodes**. If we
   later swap YOLOE for GroundingDINO + SAM2, or run a
   pre-trained ML model from a third-party repo, almost all
   open-source ROS 2 perception code emits `vision_msgs`. Sticking
   to the standard means downstream code doesn't care which
   detector produced the message.
3. **`go2_msgs/Detection2DArray` is chair-aware**. The legacy
   message carries a `is_target_candidate` boolean and other
   project-specific fields tightly coupled to the chair-only
   pipeline. Breaking that coupling is the whole point of Day 5.

**Risks accepted**:

* Phase 2 legacy `object_tracker_node` can't consume `/detections`
  directly. That's intentional — it's the deprecation lever
  forcing Day 6 to write a new tracker against the standard
  message rather than retrofitting the chair-shaped one.
