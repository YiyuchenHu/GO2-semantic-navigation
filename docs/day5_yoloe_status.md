# Day 5 — YOLOE open-vocabulary detection

**Status: ✓ verified end-to-end on 2026-04-30. All 8 hard checks pass.**

First run results (single chair in view, default `conf_threshold:=0.4`):

* `/detections` flowing at **14.37 Hz** (~RGB rate, no inference backlog)
* `class_id='chair'`, `score=0.63`, `bbox=(111×193)` pixels
* RViz overlay: green bbox + translucent red instance mask on the
  chair, label visible above the box
* `header.frame_id='camera_link'`, stamp preserved → Day 6 reprojection
  can read this directly
* GPU memory: ~1.3 GB on `cuda:0` (RTX with 8 GB headroom)
* Inference FPS heartbeat: 8-10 FPS (CPU-bound on cv_bridge encode +
  GPU-bound on YOLOE; well above the ≥5 Hz target)

This document covers the perception layer overhaul: replace the
chair-only YOLOv11l-seg detector that has driven Phases 1-4 with the
open-vocabulary **YOLOE** model. Day 5 is **strictly 2D** — bbox /
score / class on the input image. 3D backprojection lives in Day 6.

The legacy chair-only `perception_node` is **left in place**. The new
YOLOE node ships on a separate topic namespace (`/detections` and
`/detections/image`) so the two pipelines can run side by side
during the Day 6 transition.

---

## What Day 5 delivers

### Topology

```
/camera/color/image_raw  (sensor_msgs/Image, BEST_EFFORT, depth=1)
                  │
                  ▼
       ┌──────────────────────┐
       │  yoloe_detector_node │
       │  (go2_perception)    │
       │                      │
       │  YoloeBackend        │
       │  ├── set_classes()   │ ← prompt switch (Day 10 will trigger)
       │  └── infer()         │ ← per-frame YOLOE.predict
       └──────────┬───────────┘
                  │
        ┌─────────┴───────────┐
        ▼                     ▼
  /detections           /detections/image
(vision_msgs/             (sensor_msgs/Image,
 Detection2DArray)         RGB+bbox+mask overlay,
                           BEST_EFFORT, depth=1)
```

### Deliverables

| File | Purpose |
|------|---------|
| `src/go2_perception/go2_perception/yoloe_backend.py` | YOLOE wrapper with graceful fallback (mirrors YoloBackend's pattern). Exposes `available`, `unavailable_reason`, `set_classes()`, `infer()`. |
| `src/go2_perception/go2_perception/yoloe_detector_node.py` | ROS 2 node. Subscribes RGB BEST_EFFORT, publishes `vision_msgs/Detection2DArray`, optional overlay. |
| `src/go2_perception/setup.py` | Registers `yoloe_detector_node` console script. |
| `src/go2_perception/package.xml` | Adds `vision_msgs` runtime depend. |
| `src/go2_bringup_sim/launch/yoloe.launch.py` | Standalone Day 5 launch with model_path / classes / device launch args. |
| `src/go2_bringup_sim/setup.py` | Registers the new launch into `share/`. |
| `src/go2_bringup_sim/rviz/go2_semantic_nav.rviz` | Adds `Image (YOLOE detections)` panel (disabled by default). |
| `scripts/check_day5.sh` | Automated 5-section acceptance script. |
| `docs/day5_yoloe_status.md` | This document. |

---

## Pre-flight: install YOLOE

YOLOE landed in `ultralytics >= 8.3.0`. Install in the same Python
the existing `perception_node` uses (Phase 1's YOLOv11 path
already pulls ultralytics, so this is usually a no-op upgrade):

```bash
pip install -U "ultralytics>=8.3.0"
python3 -c "from ultralytics import YOLOE; print(YOLOE)"   # should print the class
```

Pre-download the weights so the first run isn't blocked on the
Ultralytics CDN:

```bash
mkdir -p ~/.config/Ultralytics
cd ~/.config/Ultralytics
# yoloe-11s-seg.pt is the MVP recommendation (~25M params,
# instance segmentation, fast on RTX 4060+ class GPUs).
wget https://github.com/THU-MIG/yoloe/releases/download/v0.1/yoloe-11s-seg.pt \
  || python3 -c "from ultralytics import YOLOE; YOLOE('yoloe-11s-seg.pt')"
```

The Python fallback above auto-downloads to the right path.

---

## How to run

```bash
# Terminal 1: Isaac Sim publishing /camera/color/image_raw
bash scripts/run_warehouse_ros2.sh

# Terminal 2: static TFs (camera_link, optical frames). The legacy
# perception_node also starts here, but on a different topic
# namespace (/perception/...), so it doesn't conflict with the
# YOLOE node.
ros2 launch go2_bringup_sim chair_perception.launch.py

# Terminal 3: YOLOE detector
ros2 launch go2_bringup_sim yoloe.launch.py
# Or with custom prompts:
ros2 launch go2_bringup_sim yoloe.launch.py \
    classes:="['box','crate','pallet']" \
    conf_threshold:=0.3

# Terminal 4: RViz — flip on the "Image (YOLOE detections)" display
bash scripts/run_rviz.sh
```

---

## Acceptance criteria

### Hard checks (`scripts/check_day5.sh`)

| # | Check | Pass criterion |
|---|-------|----------------|
| 1 | `/yoloe_detector` node alive | `ros2 node list` shows it |
| 2 | `/detections` advertised | `ros2 topic list` shows it; publisher is `/yoloe_detector` |
| 3 | `/detections/image` advertised | `ros2 topic list` shows it (or WARN if `publish_overlay:=false`) |
| 4 | `/detections` flowing | ≥ 1 message in 10 s |
| 5 | `/detections` rate ≥ 5 Hz | Soft target; below this Day 6 reprojection lags |
| 6 | Header preserved | `frame_id` non-empty, stamp non-zero (Day 6 requirement) |
| 7 | Detection2D content correct | bbox size > 0, `class_id` is a non-empty string, `0 ≤ score ≤ 1` |
| 8 | GPU in use | `nvidia-smi` shows a python process holding > 256 MB (WARN if not) |

### Manual checks (RViz / eyeballs)

| Check | Where to look |
|-------|---------------|
| Green bboxes around chairs | RViz `Image (YOLOE detections)` panel |
| `<class> <score>` label above each bbox | Same panel |
| Translucent red instance mask (`-seg` weights only) | Same panel |
| Bbox tracks the chair when Go2 rotates | Drive Go2 with teleop, watch the panel |
| `score > 0.4` on a centered, well-lit chair | Echo `/detections` |
| Multi-target scene (3+ chairs in view) gives 3+ detections | Place 3+ chairs in the warehouse first |
| Far-distance detection (~6 m) still fires | Park Go2 at the edge of the room |
| Open-vocabulary actually open: re-launch with `classes:="['box','crate']"`, verify the overlay tracks boxes instead of chairs | Manual prompt swap |

---

## Pitfalls (read before bring-up)

These are the failure modes we expect to hit on first run, ordered
by likelihood. Phases 1-4 burned ~6 layers of similar pitfalls
deep — Day 5 has fewer because we're not adding lifecycle / TF /
costmap concerns, but the model-loading layer has its own.

### Pitfall 1: `set_classes()` not called → prompt-free flood

**Symptom**: console heartbeat logs `last frame had 47 detections`,
overlay is full of random labels (`person`, `bottle`, `clock`, ...).

**Cause**: YOLOE in prompt-free mode runs against its full open
vocabulary and reports any salient object. The backend calls
`set_classes()` automatically when constructed with a non-empty
`classes` list — make sure the launch arg or the param wasn't
silently set to `[]`.

**Fix**: confirm the bring-up log line `Detecting classes: [...]`
shows your prompts; if the list is empty, check the launch
argument and rebuild.

### Pitfall 2: cv_bridge encoding mismatch

**Symptom**: chair detection precision is suspiciously low (`0.2`-
`0.3`) on otherwise-good frames, far worse than running the same
weights against the same JPEG offline.

**Cause**: Isaac Sim's RGB encoding is sometimes `rgb8`, the
backend's preprocessing assumes BGR. We pass
`desired_encoding="bgr8"` to `cv_bridge`, which performs the
swap; confirm the encoding actually arrived as `rgb8` first:

```bash
ros2 topic echo /camera/color/image_raw --field encoding --once
```

If it's `rgb8`, our cv_bridge call already converts. If it's
something exotic (e.g. `mono8`, `bayer_*`), the bridge raises and
the node logs `cv_bridge failed on input frame`.

### Pitfall 3: GPU OOM under Isaac Sim contention

**Symptom**: node dies with `RuntimeError: CUDA out of memory`
seconds after the first inference, OR the Isaac Sim window goes
sluggish / blank simultaneously.

**Cause**: Isaac Sim's RTX renderer + the cameras already use
6-7 GB on an 8 GB card; loading YOLOE on top of it tips the GPU
over. RTX 4060 Mobile / 4060 Ti are the common offenders.

**Fix (in priority order)**:

1. Launch with `half:=True` to halve YOLOE's GPU memory
   (FP16 is supported on Ampere+ GPUs).
2. Drop to `model_path:=yoloe-11n-seg.pt` (~6M params, ~half the
   VRAM of `-11s-seg`).
3. Lower Isaac Sim's render resolution in
   `sim/run_go2_warehouse_ros2.py` (camera prim's `width` /
   `height` attrs), e.g. 1280x720 → 640x480.

### Pitfall 4: ultralytics version too old

**Symptom**: backend initialisation logs
`Failed to "from ultralytics import YOLOE"`. Node still spins but
publishes only empty `Detection2DArray`s.

**Cause**: `ultralytics < 8.3.0` doesn't have the YOLOE class.

**Fix**: `pip install -U "ultralytics>=8.3.0"`. Note: the existing
chair-only `perception_node` keeps working with any `ultralytics`
that has `YOLO` (Phase 1 era), so an upgrade should be backwards-
compatible.

### Pitfall 5: low-poly chair USDs in the sim → low confidence

**Symptom**: detector runs, `class_id` is `chair`, but `score` is
`0.25-0.35` and frequently below the default `conf_threshold` of
`0.4`, so the chair appears in some frames and not others.

**Cause**: Isaac Sim's stock chair USDs are render-optimised low-
poly placeholders with simplified textures. YOLOE's training data
contains real-world chair photos; the appearance gap is real.

**Fix (in priority order)**:

1. Drop `conf_threshold` to `0.25`. MVP-acceptable; downstream
   filters in Day 6+ can re-tighten on `score`.
2. Replace the chair USD in the warehouse scene with a more
   realistic asset (ShapeNet / Sketchfab). Higher one-time cost,
   but it lifts every later phase too.
3. Expand the prompt list with synonyms — YOLOE's text encoder
   often pushes a borderline match into the keep zone if the
   prompt vocabulary is close to the textual class:
   `classes:="['chair','office chair','stool','folding chair','armchair']"`.

### Pitfall 6: rclpy single-thread blocking

**Symptom**: the FPS heartbeat reports 8 FPS even though the
RGB stream is at 22 Hz; topics that should react quickly (`/tf`,
`/scan`) lag behind.

**Cause**: the YOLOE inference runs *in* the image callback, on
the rclpy executor's single thread. Inference time exceeds the
RGB period.

**Why we accept this for MVP**: the node uses
`reliability=BEST_EFFORT, depth=1` on the input subscription, so
the executor only ever holds the *most recent* frame. Older
frames are dropped at the DDS layer rather than queued. The
result is a clean "process the latest, skip the in-between"
pattern — exactly what perception nodes want. **Don't change the
QoS to `RELIABLE`**; that would queue every frame and the lag
would compound.

If you genuinely need higher detection FPS on a slow GPU, switch
the node to a `MultiThreadedExecutor` and a separate inference
thread. Out of scope for Day 5.

---

## Known limitations / future work

- **No 3D**. `vision_msgs/Detection2D.results.pose` is left
  default-constructed. Day 6 reprojects bboxes against the depth
  image and fills it.
- **No mask topic**. The `-seg` masks are drawn into the overlay
  but not republished as `/detections/masks`. Day 6 may want
  per-pixel masks; if so, define a small ROS msg there or pickle
  into a `vision_msgs/Detection2D` extension. For MVP the bbox-
  median depth is enough.
- **Param callbacks not wired**. `ros2 param set /yoloe_detector
  classes "['box']"` will update the parameter but does **not**
  retroactively call `set_classes()`. Day 10 (command parser)
  will trigger prompt swaps via a service or topic — at that
  point we add a `set_parameters_callback` that detects the
  `classes` change and forwards it to the backend.
- **No automated multi-target / far-distance test**. Both depend
  on Isaac Sim having multiple chairs at varied distances; the
  acceptance script can't simulate that.

---

## Day 5 closure → Day 6 entry

Once `check_day5.sh` runs green and the manual RViz checks all
look right, Day 5 is closed. Day 6 (3D backprojection + semantic
memory) reads `/detections` directly:

- Sim & sensors: unchanged from Day 1-2.
- Mapping / localization: unchanged from Day 3-4 (slam_toolbox).
- New in Day 6: a node that subscribes
  `/detections` + `/camera/depth/image_rect_raw` +
  `/camera/color/camera_info` + tf2, and emits 3D object
  observations in the `map` frame. The legacy
  `object_localizer_3d_node` already does this for the chair-only
  pipeline; Day 6 generalises it to consume the YOLOE topic.
