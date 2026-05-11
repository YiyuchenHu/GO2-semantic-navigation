# Day 6 — depth reprojection + semantic memory aggregator

**Status: scaffolded, awaiting first end-to-end run.**

Day 6 turns Day 5's per-frame 2D detections into a persistent
3D object registry in the `map` frame. Two new ROS nodes, one
new ROS package (`go2_semantic_perception`), zero changes to the
Day 5 YOLOE node.

---

## What Day 6 delivers

### Topology

```
/camera/color/image_raw  ──► yoloe_detector_node  ──► /detections
                                                       (vision_msgs/
                                                        Detection2DArray)
                                                              │
/camera/depth/image_rect_raw  ───┐                            │
/camera/color/camera_info     ───┤   message_filters          │
                                 │   ApproximateTimeSync ─────┤
                                 ▼                            │
                       depth_projector_node ◄─────────────────┘
                          │   tf2 ► map frame
                          ▼
                       /detections_3d
                       (vision_msgs/Detection3DArray, frame_id=map)
                          │
                          ▼
                  semantic_memory_aggregator_node
                          │   spatial NMS by class (0.3 m radius)
                          │   exponential confidence decay
                          │   1 Hz housekeeping pruning
                          ▼
                  /semantic_map/objects   /semantic_map/markers
                  (go2_msgs/SemanticEntityArray)  (RViz markers)
```

### New files

| File | Lines | Purpose |
|------|------:|---------|
| `src/go2_semantic_perception/` | new pkg | colcon ament_python package |
| `src/go2_semantic_perception/go2_semantic_perception/depth_projector_node.py` | ~280 | Sync inputs, bbox-median depth, K⁻¹ unproject, tf2 to map |
| `src/go2_semantic_perception/go2_semantic_perception/semantic_memory_aggregator_node.py` | ~300 | Class-aware NMS, EMA position, confidence decay, RViz markers |
| `src/go2_semantic_perception/{package.xml,setup.py,setup.cfg,resource/...}` | misc | Package boilerplate |
| `src/go2_bringup_sim/launch/day6.launch.py` | ~150 | YOLOE + depth_projector + semantic_memory in one launch |
| `src/go2_bringup_sim/setup.py` | +1 | Register day6.launch.py in share/ |
| `src/go2_bringup_sim/rviz/go2_semantic_nav.rviz` | +20 | `/semantic_map/markers` MarkerArray display |
| `scripts/check_day6.sh` | ~250 | 4-section automated acceptance |
| `docs/day6_semantic_memory_status.md` | this | Status doc |

---

## How to run

```bash
# T1: sim
bash scripts/run_warehouse_ros2.sh

# T2: static TFs (camera_link, optical, lidar_link). The legacy
# perception_node and object_localizer_3d_node in this launch
# crash on numpy ABI; they're harmless because Day 6 runs on
# disjoint topics.  If you have run install_ml_deps.sh, they
# survive too.
ros2 launch go2_bringup_sim chair_perception.launch.py

# T3: nav2 (provides map → odom TF that depth_projector needs)
ros2 launch go2_bringup_sim nav2.launch.py

# T4: Day 6
ros2 launch go2_bringup_sim day6.launch.py
# Or with a richer prompt list:
ros2 launch go2_bringup_sim day6.launch.py \
    classes:="['chair','table','desk','box','crate','pallet']"

# T5: rviz
bash scripts/run_rviz.sh
# Enable: Image (YOLOE detections), Semantic memory (Day 6)
```

---

## Acceptance criteria

### Hard checks (`scripts/check_day6.sh`)

| # | Check | Pass criterion |
|---|-------|----------------|
| 1 | All three nodes alive | `/yoloe_detector` + `/depth_projector` + `/semantic_memory_aggregator` |
| 2 | Topic graph correct | `/detections`, `/detections_3d`, `/semantic_map/objects`, `/semantic_map/markers` all advertised |
| 3 | Publisher → topic binding | `/detections_3d` published by `/depth_projector`; `/semantic_map/objects` by `/semantic_memory_aggregator` |
| 4 | `/detections_3d` data flow ≥ 5 Hz | Real-rate measurement over 10 s window |
| 5 | `/detections_3d` header.frame_id = "map" | Reprojection target frame correct |
| 6 | Detection3D content valid | Finite (x,y,z), non-empty class_id, populated `results` |
| 7 | `/semantic_map/objects` flowing | At least 1 message in 8 s (housekeeping timer guarantees ≥ 1 Hz) |
| 8 | SemanticEntity content valid | Non-empty `entity_id`, `class_label`, finite pose, monotonic `observations_count` |

### Manual checks (RViz / eyeballs)

| Check | Where to look |
|-------|---------------|
| Coloured cylinder + label per object | `Semantic memory (Day 6)` MarkerArray display |
| Cylinder colour stable across frames for the same chair | Same display, watch over 10 s |
| Walking around the chair: cylinder stays put | Entity position averages via EMA |
| Confidence rises to ~1.0 then plateaus | Label text shows `<class> <conf:.2f> (n=<obs>)` |
| Looking away: cylinder fades + disappears | Alpha tied to confidence; ~13 s half-life by default, prunes after 30 s of no obs |
| Two chairs in frame → two cylinders | Distinct entity_ids, two different colours if they're different classes |

---

## Pitfalls (read before bring-up)

### Pitfall 1: depth + RGB resolution mismatch

`depth_projector_node` assumes the depth image is rectified to
the SAME intrinsics as the colour image, so `K` from
`/camera/color/camera_info` applies to both. Isaac Sim's RGB-D
Camera prim guarantees this (single render product, single
focal-length set). On real Go2 you might have separate depth
intrinsics (`/camera/depth/camera_info`); update the launch arg
`camera_info_topic` accordingly OR add a depth-vs-color rectifier.

**Symptom**: detections project to weird (x, y, z) — usually
shifted by a few cm because of the principal-point offset, or
scaled if focal lengths differ.

### Pitfall 2: TF chain breaks when nav2 is not running

`depth_projector_node` calls `tf2_buffer.lookup_transform("map",
"camera_color_optical_frame", stamp, timeout=1.5s)`. The `map`
frame only exists when slam_toolbox or AMCL (Day 4) is publishing.
Without nav2 running you get:

```
TF lookup camera_color_optical_frame -> map ... failed:
LookupException: ... frame "map" does not exist
```

throttled at 2 Hz. The projector silently drops every detection.
**Always start nav2 before Day 6** unless you change
`target_frame:=odom` in the launch (which works for short demos
but loses the map-frame benefit).

### Pitfall 3: depth NaN at object boundaries

Sim depth occasionally returns 0 / inf at object edges. We
filter these via `min_depth_m` (0.2) and `max_depth_m` (12.0),
then take median over the remaining ROI pixels. If your bbox is
mostly background-bleed (the inset shrink helps), the median can
still land on the wall behind the object. Symptom: chair
detections projecting 1-2 m too far.

**Mitigation**: tighten `bbox_shrink` (default 0.10 → try 0.20)
or wait for Day 6.5 to wire mask-aware depth sampling (the code
path is in place; just needs `/detections` to publish masks).

### Pitfall 4: semantic_memory keeps two entities for the same chair

If the chair detection's depth jitters by more than `nms_radius_m`
(0.3 m default) between consecutive frames, the aggregator will
register a second entity rather than merging. Symptom: two
cylinders at slightly different positions, each with low
`observations_count` and growing slowly.

**Mitigation**: increase `nms_radius_m` to 0.5 (we do it via
launch arg). Don't go beyond ~0.7 or distinct-but-close objects
(e.g. two stacked boxes) start merging.

### Pitfall 5: confidence decay too aggressive on slow detector

Day 5 measured 14 Hz on `/detections`. depth_projector emits
~14 Hz on `/detections_3d`. semantic_memory's confidence step
is +0.15 per match; with a 1.0 cap, ~7 frames bring an entity
from 0 to saturated. Decay is age-aware exponential
(``confidence *= exp(-decay_rate * age_since_last_seen)``); with
the default `decay_rate=0.05` an entity that hasn't been seen for
13.9 s is at half its peak confidence, fully decayed below the
prune threshold (0.05) around 60 s. An entity that was last seen
1 s ago barely decays per tick.

If you change YOLOE to `yoloe-11n-seg.pt` (Day 5 pitfall 3
mitigation) and detection rate drops to 5 Hz, the aggregator may
see an entity decay faster than it accumulates: confidence
oscillates. Tune `confidence_step_up` UP (try 0.25) or
`confidence_decay_rate` DOWN (try 0.025) when running on a
slower detector.

### Pitfall 6: chair_perception.launch.py's perception_node still crashes

Same numpy ABI issue from Day 5 pitfalls. Day 6 doesn't depend
on the legacy chair-only `perception_node` — we route through
Day 5's `/detections` namespace — so the crash is harmless
noise. If it bothers you, run `bash scripts/install_ml_deps.sh`
to install numpy<2 in user-site, which fixes the legacy nodes
too.

---

## Known limitations / Day 7+ work

- **No mask-based depth sampling**. The Day 6 code has the path
  in place but currently uses bbox-median (because /detections
  doesn't carry mask data; YOLOE-seg overlays its mask on
  /detections/image only). Day 7 may add an InstanceMask topic.
- **No Kalman filter**. Position is EMA, not properly fused with
  uncertainty. Adequate for static objects in a 10 m room;
  revisit for dynamic obstacles (humans).
- **`is_dynamic` always False**. We don't classify objects as
  static / dynamic. Day 7+ heuristic: small position variance
  → static, large → dynamic.
- **`size_xyz` always (0,0,0)**. We don't estimate 3D extents.
  Downstream NMS uses centre-distance, not box overlap.
- **No persistent storage**. Restarting `semantic_memory_aggregator`
  loses all entities. Day 9+ may add disk persistence.
- **Class-only NMS**. A `chair` and an `office chair` 0.2 m apart
  are TWO entities (different class strings), not merged. Day 10
  command parser can normalise prompts before the aggregator
  sees them.

---

## Day 6 closure → Day 7 entry

After Day 6 acceptance:
* `/semantic_map/objects` is the canonical "what objects do I
  know about, where, and how confident am I?" topic.
* Day 7 = **target selection + approach goal generation** on top
  of /semantic_map/objects, replacing the legacy Phase 3A
  `target_selector_node` + `goal_planner_node` to consume the
  new SemanticEntityArray with multi-class support.
* Day 8 = wire goal_planner output into Nav2's
  `/navigate_to_pose` action client. End-to-end "go to chair"
  comes online here.
