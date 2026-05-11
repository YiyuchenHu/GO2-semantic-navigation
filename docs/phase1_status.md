# Phase 1 — Chair-Only Semantic Perception

**Status: complete, chair only.** The Phase 0 RGB-D stream is now consumed
by a chair-only 2D detector + 3D localizer pair, producing stable output on
`/perception/detections_2d`, `/perception/masks`, and `/perception/objects_3d`
with the published `class_label` normalized to the canonical `chair`.

This document is deliberately scoped to what Phase 1 actually delivered.
Anything involving semantic memory, tracked-object persistence, target
selection, approach planning, or navigation loops is **Phase 2+** and is
not described here.

---

## Sources of truth

Written from the current contents of:

| File | Role |
|---|---|
| `src/go2_perception/go2_perception/perception_node.py` | 2D detector node; chair-only filter, alias matching, canonical label normalization, heartbeat / diagnostic logging. |
| `src/go2_perception/go2_perception/yolo_backend.py` | Ultralytics YOLO wrapper; exposes `unavailable_reason` when init fails. |
| `src/go2_perception/go2_perception/grounding_sam_backend.py` | GroundingDINO/SAM2 placeholder (intentionally inert in Phase 1). |
| `src/go2_object_localization/go2_object_localization/object_localizer_3d_node.py` | RGB-D → 3D centroid node; parameterized frames, TF-fallback behavior. |
| `src/go2_bringup_sim/launch/chair_perception.launch.py` | Phase 1 launch: perception + localizer + `base_link → camera_link` static TF. |
| `src/go2_bringup_sim/setup.py` | Registers the launch file under `share/`. |
| Runtime validation | `ros2 launch go2_bringup_sim chair_perception.launch.py` against the live Phase 0 sim; per-topic checks shown below. |

Everything below is grounded in those files and in the most recent runtime
run; no speculation about unimplemented behavior.

---

## 1. Phase 1 overview

**Phase 1 adds the first semantic signal to the Phase 0 platform.** It
consumes `/camera/color/image_raw` + `/camera/depth/image_rect_raw` +
`/camera/color/camera_info` plus the `odom → base_link → camera_link` TF
tree, runs a YOLO11-seg detector, filters to a single target class
(default `chair`), projects the masked detection into 3D, and publishes
both the 2D and 3D observations.

It comes after Phase 0 because every input it needs — camera streams, TF,
odom, a stable sim scene — is exactly what Phase 0 certified as working.
Phase 1 does not change Phase 0 behavior in any way; `sim/` is untouched.

Phase 1 explicitly does **not** include:

- semantic memory / entity tracking
- `/semantic/tracked_objects`, `/semantic_map/entities`
- target selection (`target_selector_node`)
- approach goal planning (`goal_planner_node`)
- navigation execution (`nav_executor_node`, `arrival_verifier_node`)
- task coordinator loops
- safety monitor integrated with the active perception loop
- any class other than `chair`

---

## 2. Phase 1 goals

1. Detect chair 2D bounding boxes from the color image stream.
2. Produce an instance mask when the detector supports segmentation.
3. Project the masked chair into 3D using the depth image, color camera
   intrinsics, and TF, producing a centroid in both `base_link` and `odom`.
4. Publish a `go2_msgs/ObjectObservationArray` with a clean, downstream-
   friendly `class_label` == canonical task label (`chair`), independent
   of whether the raw detector said `chair`, `bench`, `couch`, or similar.
5. Keep the implementation small, chair-only, and backward-compatible with
   the existing `go2_msgs` schema.

---

## 3. What Phase 1 implemented

### 3.1 `go2_perception/perception_node.py`

Concrete changes layered on top of the pre-existing skeleton:

- **Default target class.** New ROS parameter
  `default_target_class` (default `"chair"`). The node starts with this
  target already armed, so it works without a `/semantic_task/current`
  publisher. An incoming `SemanticTask` message still overrides it.
- **Chair-only filter.** New parameter `only_publish_target_class`
  (default `True`). When True, only detections that pass
  `_label_matches_target(...)` get published on
  `/perception/detections_2d` and `/perception/masks`. Flip to `False`
  to publish the full YOLO class set for debugging.
- **Alias matching.** New parameter `target_class_aliases`
  (default `["chair", "couch", "bench", "sofa", "armchair"]`).
  `_label_matches_target(label)` returns True if the label exactly
  equals the target, contains it as a substring (`"dining chair"`), or
  appears in the alias list. This covers the observed case where
  YOLO11-seg on COCO classifies the `EastRural_Chair` asset as `bench`.
- **Canonical label normalization.** When a raw YOLO detection passes
  the alias filter, `_process` now:
  - copies the detection,
  - moves the original YOLO label into a new `raw_label` field,
  - overwrites `class_label` with `self._current_target_class` (i.e.
    `chair`),
  - sets `is_target_candidate = True`.

  `_to_msgs(...)` then populates `Detection2D.class_label` and
  `InstanceMask.class_label` from the canonical value. Downstream
  nodes therefore see `class_label == "chair"` on the bus regardless
  of what YOLO actually called the object.
- **Debug path preserves raw output.** The debug image is drawn from
  `raw_detections` (pre-filter, pre-normalization), so the bounding box
  label text remains the YOLO original (e.g. `bench:0.72`). The
  heartbeat log's `yolo_raw=[...]` field is populated from the same
  unmodified `raw_detections`.
- **Heartbeat and diagnostic logging.**
  - `log_period_sec` parameter (default `1.0`).
  - Once per `log_period_sec`, the node prints
    `[chair-perception] frames=... target='chair' aliases=[...] raw_detections=R published=P target_found=... best_score=... yolo_raw=[...]`.
  - When `raw_detections > 0` but `published == 0`, an additional WARN
    guides the operator to extend `target_class_aliases`.
  - When no `/camera/color/image_raw` has arrived, the heartbeat prints
    `waiting for /camera/color/image_raw (...)`, so "no output" is
    never silent.
  - If the YOLO backend failed to initialise, the node logs a single
    high-visibility ERROR containing `YoloBackend.unavailable_reason`
    plus concrete remediation hints.

### 3.2 `go2_perception/yolo_backend.py`

- Exposes `available` and a new `unavailable_reason` property.
- Both the `import ultralytics` step and the `YOLO(model_name)`
  construction step are wrapped in specific `except` blocks that record
  the exception class and message on `unavailable_reason` and print a
  traceback to stderr. Previous behavior silently set `self._model = None`
  and gave the operator no diagnostic.

### 3.3 `go2_object_localization/object_localizer_3d_node.py`

- **Frames are parameterized.** New ROS parameters:
  - `depth_info_topic` (default `/camera/color/camera_info`) — matches
    the Phase 0 sim, where RGB and depth share a render product and
    therefore share intrinsics.
  - `global_frame` (default `"odom"`) — Phase 0 TF tree is
    `odom → base_link`, so `"map"` is not assumed.
  - `base_frame` (default `"base_link"`).
- **TF fallback.** If the camera-to-`global_frame` transform is
  unavailable, the node no longer drops the observation. It reuses the
  base-link centroid for `centroid_map` and logs a single WARN. The
  observation is still dropped only when the camera-to-`base_frame`
  transform itself is missing, which would make the data meaningless.
- **Heartbeat log.** Once per `log_period_sec` (default `1.0`) prints
  `[chair-localizer] det_cb=... obs_pub_total=... detections=D observations=O`
  or, while inputs are missing,
  `[chair-localizer] ... waiting for inputs depth_msg=... depth_info=...`.

### 3.4 `go2_bringup_sim/launch/chair_perception.launch.py`

A new Phase-1-only launch file. It starts exactly three processes:

1. `static_transform_publisher`: `base_link → camera_link` with the
   extrinsic matching the camera prim in
   `sim/run_go2_warehouse_ros2.py`
   (translation `(0.30, 0.00, 0.12)`,
   orientation WXYZ `(0.7071068, 0, -0.7071068, 0)`).
2. `go2_perception::perception_node` with the Phase 1 parameters.
3. `go2_object_localization::object_localizer_3d_node` with
   `depth_info_topic='/camera/color/camera_info'`, `global_frame='odom'`.

Launch arguments exposed: `target_class`, `only_target`, `yolo_model`,
`global_frame`.

It deliberately does **not** start `go2_semantic_memory`,
`go2_navigation`, `go2_task_coordinator`, or `go2_safety` — those are not
Phase 1.

### 3.5 `go2_bringup_sim/setup.py`

Registers `chair_perception.launch.py` under
`share/go2_bringup_sim/launch/` so that
`ros2 launch go2_bringup_sim chair_perception.launch.py` resolves
correctly after `colcon build`.

### 3.6 `docs/phase1_status.md`

This document. No other docs file was created.

### 3.7 Messages

`go2_msgs` was **not changed**. Phase 1 fits entirely inside the
existing `Detection2D`, `Detection2DArray`, `InstanceMask`,
`InstanceMaskArray`, `ObjectObservation`, and `ObjectObservationArray`
schemas.

---

## 4. Test / validation results

Validation was performed with:

```
Terminal A:  bash scripts/run_warehouse_ros2.sh               # Phase 0 sim
Terminal B:  ros2 launch go2_bringup_sim chair_perception.launch.py
Terminal C:  ros2 topic ... (inspection)
```

Observed outcomes:

- **Startup cleanliness.**
  `Perception ready. target_class='chair' aliases=[...] only_publish_target=True YOLO available=True GroundingSAM available=False`
  and
  `Object localizer ready. depth_info_topic='/camera/color/camera_info' global_frame='odom' base_frame='base_link'`
  appear within a few seconds. No NumPy ABI errors and no segfault once
  `numpy<2` is pinned (see "Environment requirements" below).
- **`/perception/detections_2d`.**
  `timeout 10 ros2 topic hz /perception/detections_2d` shows a steady
  rate in line with the perception node's timer (~10 Hz).
  `ros2 topic echo /perception/detections_2d --once` is **non-empty**
  and every entry carries `class_label: chair`,
  `is_target_candidate: true`, and a usable bounding box.
- **`/perception/masks`.**
  Advertised, and carries one entry per chair detection whenever YOLO
  produced a segmentation mask for that box. `width`/`height` match
  the color image.
- **`/perception/objects_3d`.**
  Non-empty whenever the chair is in view. Each observation has
  `class_label: chair`, a non-zero `depth_valid_ratio`, and a
  `centroid_base_link` consistent with the sim geometry (chair at
  `(2.7, 1.0)`, Go2 spawn `(-4.0, -4.0, 0.55)` yaw 45°; centroid is
  a few meters ahead of the robot in base_link).
- **Canonical vs. raw label.** On this warehouse, YOLO calls the
  `EastRural_Chair` asset `bench`. Validation confirmed:
  - `yolo_raw=[bench:0.xx]` in the `[chair-perception]` heartbeat
    (the raw detector output is preserved),
  - `/perception/debug_image` draws the bounding box with the text
    `bench:0.xx` (the raw detector output is preserved),
  - `/perception/detections_2d` and `/perception/objects_3d` both
    publish `class_label: chair` (the canonical task label is what
    leaves the node on the ROS bus).

This is the intended separation between debug view and downstream bus.

- **TF tree.** `ros2 run tf2_tools view_frames` shows
  `odom → base_link → camera_link` after the launch's static TF is up.

Negative-path coverage also observed and explicitly handled:

- When `/camera/color/image_raw` is not arriving, the perception node
  prints `[chair-perception] waiting for /camera/color/image_raw (...)`
  once per heartbeat period, so the state is never silent.
- When the YOLO backend cannot initialise (e.g. `ultralytics` missing),
  the node prints an ERROR with `YoloBackend.unavailable_reason` plus
  a remediation hint instead of silently publishing empty arrays.

---

## 5. Known limitations of Phase 1

- **Chair-only.** Only a single target class is supported end-to-end at
  a time, defaulting to `chair`. Other classes can be enabled by
  changing `target_class` / `target_class_aliases`, but this has not
  been validated.
- **Label normalization is a workaround, not a correct classifier.**
  The published `class_label` is `chair` whenever the raw YOLO label
  falls in the alias set. This is acceptable for the MVP because the
  warehouse contains exactly one chair-like object, but it is **not** a
  substitute for a classifier that actually gets the semantic class
  right. If a `couch` and a `chair` are both in the scene, both will be
  published as `chair` today.
- **No semantic memory.** Every frame is independent; there is no
  persistence, deduplication, tracking, or fusion of chair observations
  across frames. That is Phase 2.
- **No target selection.** There is no "select one chair as the goal"
  logic. Phase 2.
- **No navigation loop.** Nothing consumes `/perception/objects_3d`
  today; no planner, executor, arrival verification, or task
  coordinator is running in the Phase 1 launch. Phase 3.
- **Global frame is `odom`, not `map`.** Phase 0 does not publish a
  `map` frame. When Phase 2+ introduces SLAM / a map-level semantic
  store, `global_frame` should be switched via launch arg.
- **Detector semantics drift.** Because the detector physically outputs
  `bench`, any consumer that wants to distinguish chairs from benches
  has to call the object something different from what's displayed in
  `/perception/debug_image`. This is by design for Phase 1 but worth
  tracking.
- **Dependency pin required.** The ROS 2 Jazzy `cv_bridge` C extension
  is compiled against the NumPy 1.x ABI. Installing `ultralytics`
  pulls `numpy >= 2`, which causes a `cv_bridge` SIGSEGV on import.
  Phase 1 requires `numpy<2` pinned in the same Python that runs the
  ROS nodes (see "Environment requirements").

---

## 6. How to run Phase 1

### Environment requirements

One-time setup on a fresh machine (needed only for the Phase 1 Python
side — `sim/` still uses Isaac Sim's own bundled Python):

```bash
# pip itself (Ubuntu 24.04 ships no pip for system Python by default)
sudo apt install -y python3-pip

# Ultralytics + a NumPy 1.x pin so cv_bridge doesn't segfault
/usr/bin/python3 -m pip install --user --break-system-packages ultralytics
/usr/bin/python3 -m pip install --user --break-system-packages "numpy<2"
```

Quick sanity check:

```bash
/usr/bin/python3 -c "
import numpy, cv_bridge, ultralytics, sys
print('numpy   ', numpy.__version__)
print('ultralyt', ultralytics.__version__)
print('python  ', sys.executable)
"
# Expect: numpy 1.26.x, ultralytics 8.x.x, /usr/bin/python3
```

### Build

```bash
cd /path/to/GO2-semantic-navigation
colcon build --symlink-install \
  --packages-select go2_msgs go2_perception go2_object_localization go2_bringup_sim
source install/setup.bash
```

### Launch (two terminals)

**Terminal A — Phase 0 sim (must be up first):**

```bash
bash scripts/run_warehouse_ros2.sh
# Wait for: [run_ros2] Ready. Publishing ROS 2 topics.
```

**Terminal B — Phase 1 perception:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch go2_bringup_sim chair_perception.launch.py
```

Optional overrides:

```bash
ros2 launch go2_bringup_sim chair_perception.launch.py \
  target_class:=chair \
  only_target:=true \
  yolo_model:=yolo11l-seg.pt \
  global_frame:=odom
```

### Topic inspection

In a third terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Existence and rate
ros2 topic info /perception/detections_2d -v
ros2 topic info /perception/masks         -v
ros2 topic info /perception/objects_3d    -v

timeout 10 ros2 topic hz /perception/detections_2d
timeout 10 ros2 topic hz /perception/objects_3d

# Content (must show class_label: chair)
ros2 topic echo /perception/detections_2d --once
ros2 topic echo /perception/objects_3d    --once

# TF tree
ros2 run tf2_tools view_frames
```

### Debug image

```bash
ros2 run rqt_image_view rqt_image_view /perception/debug_image
```

Expected on the debug image: a rectangle around the chair, labelled
with the **raw detector output** (e.g. `bench:0.72`), not the canonical
label. This is intentional — the debug image's job is to show what
YOLO actually returns.

### Chair in FOV

The Go2 spawns at `(-4.0, -4.0, 0.55)` yaw `+45°` and the chair is at
`(2.7, 1.0)`. Yaw 45° in the Phase 0 warehouse layout puts the chair
inside the camera FOV immediately in most runs. If the heartbeat shows
`yolo_raw=[<empty>]` for more than a few seconds, rotate the robot:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{angular: {z: 0.3}}"
```

Stop with Ctrl-C once the heartbeat reports `target_found=True`.

### What success looks like (log markers)

```
Perception ready. target_class='chair' aliases=['chair','couch','bench','sofa','armchair']
  only_publish_target=True YOLO available=True GroundingSAM available=False
Object localizer ready. depth_info_topic='/camera/color/camera_info'
  global_frame='odom' base_frame='base_link'

[chair-perception] frames=N target='chair' aliases=[...]
  raw_detections=R published=P target_found=True best_score=0.xx
  yolo_raw=[bench:0.xx]
[chair-localizer] det_cb=... obs_pub_total=... detections=... observations=...
```

---

## 7. Acceptance criteria

Phase 1 is considered complete when **all** of the following hold on a
live run:

1. `/perception/detections_2d` publishes at a steady rate (≈10 Hz), and
   `ros2 topic echo /perception/detections_2d --once` returns a
   non-empty `detections` array where every entry has
   `class_label == "chair"` and `is_target_candidate: true`.
2. `/perception/masks` is advertised at the same rate and publishes
   one mask per chair detection whenever YOLO produced a segmentation
   mask.
3. `/perception/objects_3d` publishes at ≈10 Hz whenever the chair is
   in view, and each entry has `class_label == "chair"`, a
   `depth_valid_ratio >= 0.25`, and a geometrically reasonable
   `centroid_base_link`.
4. `/perception/debug_image` renders a bounding box around the chair
   labelled with the raw detector output, proving that the published
   canonical label is a deliberate normalization and not an accidental
   relabel by the model.
5. The `[chair-perception]` heartbeat shows `yolo_raw=[bench:0.xx]`
   (or similar raw label) while `published >= 1`, confirming the
   raw vs. canonical split is exercised end-to-end.
6. `ros2 run tf2_tools view_frames` shows
   `odom → base_link → camera_link`.

---

## 8. Next step

The next phase is **Phase 2: semantic memory.** Phase 2 will consume the
Phase 1 `/perception/objects_3d` stream, deduplicate / track / fuse
repeated observations of the chair, and expose a stable semantic entity
representation for higher-level phases.

Phase 2 is **not implemented**. It is only named here as the planned
next step.
