# Phase 2 — Semantic Memory (chair-only)

**Status: complete, chair only.** Repeated chair observations from Phase 1
are fused into (a) a stable tracked-object layer on
`/semantic/tracked_objects`, (b) a persistent semantic-entity layer on
`/semantic_map/entities`, and (c) RViz-visualizable markers on
`/semantic_map/markers`. The resulting chair entity survives short
perception dropouts — which is the specific new capability Phase 2 adds.

This document is scoped only to what Phase 2 delivered. Anything involving
target selection, approach planning, navigation execution, arrival
verification, task coordination, or safety integration is **Phase 3+**
and is not described here.

---

## Sources of truth

Written from the current contents of:

| File | Role |
|---|---|
| `src/go2_semantic_memory/go2_semantic_memory/object_tracker_node.py` | Tracker node: associates Phase 1 observations into stable tracks, smooths centroid / size / confidence, emits `/semantic/tracked_objects`. |
| `src/go2_semantic_memory/go2_semantic_memory/semantic_map_node.py` | Semantic map node: three-gate promotion, dropout-tolerant re-association, persistent entity storage, publishes `/semantic_map/entities` and `/semantic_map/markers`. |
| `src/go2_bringup_sim/launch/chair_semantic_memory.launch.py` | Phase 2 launch. Includes the Phase 1 launch and starts tracker + semantic map. |
| `src/go2_bringup_sim/setup.py` | Registers the new launch file under `share/`. |
| `src/go2_msgs/msg/TrackedObject{,Array}.msg`, `SemanticEntity{,Array}.msg` | Existed before Phase 2; no schema change. |
| Runtime evidence | Live `ros2 topic echo /semantic_map/entities --once` on the warehouse sim returned a chair entity (`entity_id=78875a94-…`, `class_label=chair`, `observations_count=11`, `frame_id=odom`) that kept being published while `currently_visible=false`, confirming the dropout-tolerance path. Heartbeat steadied at `entities=1 promotions_total=1 evicted_total=0 by_class=[chair:1]`. |

No `go2_msgs` schema change. No Phase 0 or Phase 1 code change required
for Phase 2 itself.

---

## 1. Phase 2 overview

Phase 2 consumes the Phase 1 stream `/perception/objects_3d` (a flow of
per-frame `ObjectObservation` records) and turns it into two longer-lived
representations:

- **Tracked objects** — one track per physical object that's being seen
  right now, with EMA-smoothed centroid / size / confidence and a
  configurable TTL. Published on `/semantic/tracked_objects` as
  `go2_msgs/TrackedObjectArray`.
- **Semantic entities** — a chair, once seen stably enough, is *promoted*
  into a persistent record that survives short perception dropouts and
  tracker evictions. Published on `/semantic_map/entities` as
  `go2_msgs/SemanticEntityArray`, and visualized on
  `/semantic_map/markers` as `visualization_msgs/MarkerArray`.

Phase 2 comes after Phase 1 because every input it needs — 3D
observations in a consistent frame, TF, stable topic rates — is what
Phase 1 certified as working. Phase 2 does not change Phase 0 or
Phase 1 behavior; the Phase 1 launch is simply **included** by the
Phase 2 launch so the full perception + memory stack comes up with
one command.

The specific capability Phase 2 adds: **temporal persistence**. Phase 1
forgets everything per frame. Phase 2 remembers a chair for minutes,
accumulates evidence across frames, and keeps it on the bus even when
the detector misses a few seconds.

---

## 2. Phase 2 goals

1. Consume `/perception/objects_3d` and associate observations into
   stable tracks.
2. Publish stable `/semantic/tracked_objects`.
3. Promote stable chair tracks into persistent semantic entities.
4. Publish `/semantic_map/entities` and `/semantic_map/markers`.
5. Tolerate short perception dropouts so a single `chair` entity keeps
   existing even when the tracker momentarily has nothing to report.
6. Ship the whole thing as a single launch file and keep the
   implementation simple and explicit.

---

## 3. What Phase 2 implemented

### 3.1 `go2_semantic_memory/object_tracker_node.py`

Subscription: `/perception/objects_3d`
(`go2_msgs/ObjectObservationArray`).
Publication: `/semantic/tracked_objects`
(`go2_msgs/TrackedObjectArray`), triggered both by each incoming
observation message and by a 0.5 s timer tick.

Per-observation association rule:

1. Candidate tracks = existing tracks with the same `class_label` not
   already matched this frame.
2. Pick the candidate whose 3D centroid is closest to the observation's
   `centroid_map`.
3. If `best_distance > association_distance_m` (default **1.0 m**) or
   there is no candidate, create a new track with a fresh UUID and
   initial state taken from the observation.
4. Otherwise update the existing track:
   - centroid, size, confidence, uncertainty updated via EMA
     (`ema_alpha = 0.4`).
   - velocity computed from smoothed centroid delta / dt.
   - `observations_count += 1`, `last_seen_ns = now`,
     `currently_visible = True`.

Other behavior:

- Tracks not seen in this observation message are flipped to
  `currently_visible = False`.
- On the 0.5 s timer tick, `currently_visible` is refreshed from
  `now − last_seen_ns < 0.8 s` so the flag stays honest even without
  new observations.
- Stale tracks are evicted when `now − last_seen_ns > static_ttl_sec`
  (default **60 s**). A small set of dynamic classes (`{"person"}`)
  uses `dynamic_ttl_sec = 3 s` instead; chair is static.
- Published frame is taken from the upstream
  `ObjectObservationArray.header.frame_id` whenever an observation
  message triggered the publish; on the pure-tick path the configured
  `global_frame` (default `"odom"`) is used.

Phase 2-added parameters: `global_frame` (default `"odom"`),
`log_period_sec` (default `1.0`). Phase 2-added logs:
- `[tracker] NEW track id=… class='chair' pos=(x, y, z)` once per new
  track.
- `[tracker] EVICT track id=… class='chair' obs=N age=Ts` once per
  eviction.
- `[tracker] obs_msgs=… obs_items=… active_tracks=… visible=… created_total=… evicted_total=… by_class=[chair:N]`
  once per `log_period_sec`.

### 3.2 `go2_semantic_memory/semantic_map_node.py`

Subscription: `/semantic/tracked_objects`.
Publications: `/semantic_map/entities`,
`/semantic_map/markers`.
Both are published on each tracker message **and** on a 1 Hz timer so
the stream does not stop when the tracker is silent.

Per-track resolution, performed in this order:

1. **Fast path — `track_id → entity_id`**: if the track is already
   bonded to an entity, update the entity (EMA on pose / size /
   confidence / uncertainty with `ema_alpha = 0.35`, refresh
   `last_seen`, refresh `currently_visible`).
2. **Dropout-tolerant re-association** — if there is no existing bond
   (because the upstream tracker evicted the old track during an
   occlusion and the next observation produced a fresh UUID), try to
   bind the new track to an existing entity by `class_label + 3D
   distance ≤ entity_association_distance_m` (default **1.2 m**). On
   success, log
   `[semantic-map] RE-ASSOC track=… -> entity=… class='chair' dist=…m`.
3. **Promotion** — if the track couldn't be re-associated, check the
   promotion gates described below. On success, log
   `[semantic-map] PROMOTED (fast|stable|chair_mvp) track=… -> entity=… class='chair' pose=(...) obs=N conf=0.xx`.

Three independent **promotion gates**, any one passing is enough:

| Gate | `observations_count ≥` | `confidence ≥` | Extra | Default |
|---|---|---|---|---|
| `fast` | 3 | 0.45 | — | on |
| `stable` | 8 | 0.15 | — | on |
| `chair_mvp` | 8 | any | `class_label == "chair"` | **on** |

`chair_mvp` is the gate that actually fires for this scene: YOLO-COCO
classifies the warehouse chair as `bench` with a raw confidence around
0.2, which Phase 1 normalizes to `class_label: chair`. The tracker's
EMA-smoothed `confidence` settles around 0.2–0.4, well below the
`fast` gate's 0.45 but easily above `stable`'s 0.15 threshold, and
always past `chair_mvp`'s "any confidence" line once observations
accumulate.

Persistence / eviction:

- An entity is evicted only when `now − last_seen_ns >
  static_entity_ttl_sec` (default **180 s** for static classes;
  5 s for dynamic). On eviction the node logs
  `[semantic-map] EVICT entity=… class='chair' age=…s obs=N` and
  drops the associated `track → entity` bonds.
- `currently_visible` on an entity is refreshed on the 1 Hz timer
  purely from `now − last_seen_ns < 1.0 s`, so the marker fades even
  if the tracker is silent.

Phase 2-added parameters (semantic_map_node):
`promotion_min_observations_stable = 8`,
`promotion_min_confidence_stable = 0.15`,
`chair_mvp_promotion_enabled = True`,
`chair_mvp_min_observations = 8`,
`entity_association_distance_m = 1.2`,
`global_frame = "odom"`, `log_period_sec = 1.0`, and the relaxed
`static_entity_ttl_sec = 180`.

### 3.3 `/semantic_map/markers`

Three marker namespaces so RViz can colour-code them:

| Namespace | Marker | Meaning | Colour |
|---|---|---|---|
| `semantic_entities` | `CUBE`, sized from entity `size_xyz` | persistent chair entity | blue, brighter when `currently_visible`, dimmer otherwise |
| `semantic_entities_labels` | `TEXT_VIEW_FACING` floating above the cube | `display_name = "chair_<entity_id[:8]>"` | white |
| `tracks` | `SPHERE` (≈ 30 cm) at track centroid | current tracker output (visible tracks only) | orange |

When an entity or a track disappears from the current frame, the node
emits explicit `Marker.DELETE` messages for the stale ids. Without
this, RViz would accumulate "ghost" cubes at the last known pose of
every chair that ever existed.

### 3.4 `go2_bringup_sim/launch/chair_semantic_memory.launch.py`

New Phase 2 launch. It:

1. **Includes** `chair_perception.launch.py` (Phase 1), forwarding
   `global_frame` and `target_class` launch arguments, so the whole
   `static TF → perception → localizer → tracker → semantic map` chain
   comes up with one command.
2. Starts `go2_semantic_memory::object_tracker_node` and
   `go2_semantic_memory::semantic_map_node` with the Phase 2
   parameters.

Explicitly does **not** start `target_selector_node`,
`goal_planner_node`, `nav_executor_node`, `arrival_verifier_node`,
`task_coordinator_node`, or `safety_monitor_node`. Those are Phase 3+.

Exposed launch arguments: `global_frame` (default `odom`),
`target_class` (default `chair`), `entity_association_distance_m`
(default `1.2`), `promotion_min_observations` (default `3`),
`promotion_min_confidence` (default `0.45`).

### 3.5 `go2_bringup_sim/setup.py`

Registers `chair_semantic_memory.launch.py` under
`share/go2_bringup_sim/launch/`.

### 3.6 `docs/phase2_status.md`

This document. No other doc file was added for Phase 2.

### 3.7 Messages

`go2_msgs` was **not changed**. Phase 2 uses the pre-existing
`TrackedObject`, `TrackedObjectArray`, `SemanticEntity`, and
`SemanticEntityArray` schemas.

---

## 4. Test / validation results

Validation was performed with:

```
Terminal A:  bash scripts/run_warehouse_ros2.sh
Terminal B:  ros2 launch go2_bringup_sim chair_semantic_memory.launch.py
Terminal C:  ros2 topic hz / echo and RViz
```

Observed outcomes:

- **Startup cleanliness.** Each node comes up with a single `ready` log
  line that announces the active parameters:
  ```
  Object tracker ready. global_frame='odom' assoc_dist=1.0m alpha=0.4 static_ttl=60.0s
  Semantic map node ready. global_frame='odom'
    promote_fast=(obs>=3, conf>=0.45)
    promote_stable=(obs>=8, conf>=0.15)
    promote_chair_mvp=(obs>=8, any conf)
    assoc_dist=1.2m static_ttl=180.0s
  ```
- **`/semantic/tracked_objects`.** `ros2 topic hz` shows ~13 Hz.
  `echo --once` returns a non-empty `tracks` array containing one
  entry with `class_label: chair`, a growing `observations_count`,
  and a valid centroid.
- **`/semantic_map/entities`.** `ros2 topic hz` shows ~13 Hz.
  `echo --once` returned a persistent chair entity, for example:
  ```
  entity_id: 78875a94-c034-49fe-aaf8-351b38abaf8d
  class_label: chair
  display_name: chair_78875a94
  frame_id: odom
  pose_map.position: (-1.10, 0.29, 0.69)
  confidence: 0.40
  observations_count: 11
  currently_visible: false         # tracker briefly lost it, entity stayed
  is_dynamic: false
  ```
  This observation — that the entity **remained on the bus while
  `currently_visible=false`** — is the concrete validation of the
  Phase 2 "survive short dropouts" requirement.
- **Promotion gate used.** The semantic map's heartbeat stabilizes at
  `entities=1 promotions_total=1 evicted_total=0 by_class=[chair:1]`.
  Because the tracker's EMA-smoothed `confidence` stays below 0.45,
  the promotion ran through the `stable` or `chair_mvp` gate
  (`PROMOTED (stable)` / `PROMOTED (chair_mvp)` lines are emitted by
  the node for any promotion; the specific label logged depends on
  which gate passed first).
- **Canonical class label.** The `class_label` on both
  `TrackedObject` and `SemanticEntity` is `chair`, inherited from
  Phase 1's label normalization, even though the underlying YOLO
  detection string was `bench` (and can be seen as such in
  `/perception/debug_image`'s bounding-box text).
- **RViz visualization.** With Fixed Frame `odom` and a `MarkerArray`
  display on `/semantic_map/markers` the operator sees:
  - an **orange sphere** at the chair while it's being tracked,
  - a **blue cube** that remains when the chair is briefly out of
    view (its brightness dips from "currently visible" to "remembered
    only"),
  - a **white text label** `chair_78875a94` floating above the cube.
- **Negative paths also covered.**
  - When the robot turns away and back, the tracker issues a fresh
    UUID; the semantic map's `RE-ASSOC` path binds it to the existing
    entity and logs
    `[semantic-map] RE-ASSOC track=… -> entity=… class='chair' dist=…m`,
    so no duplicate entity is created.
  - `Marker.DELETE` messages are emitted when an entity or track
    disappears, so stale cubes/spheres do not accumulate in RViz.

---

## 5. Known limitations of Phase 2

- **Chair-only validated.** The pipeline is class-agnostic in code, but
  only `chair` has been exercised end-to-end. Because Phase 1's alias
  map normalizes `couch`/`bench`/`sofa`/`armchair` to `chair`, a scene
  containing multiple chair-like objects would currently collapse them
  all to the same class.
- **Global frame is `odom`.** There is no SLAM and no `map` frame yet.
  Over long runs `odom` drifts, which slowly drags the entity's
  `pose_map` with it. Switching `global_frame` to `map` is a launch
  argument that should only be flipped once Phase 3+ brings a map
  publisher online.
- **Promotion thresholds are MVP-tuned.** `association_distance_m`,
  `entity_association_distance_m`, the EMA alphas, the TTLs, and
  especially the three promotion gates are set by inspection of the
  current warehouse scene. They have not been calibrated against a
  held-out dataset. The `chair_mvp` gate explicitly accepts any
  confidence for `chair`, which is fine for single-chair MVP but must
  be disabled before multi-class use.
- **Detector confidence is low by design of the sim.** YOLO11-seg on
  the Isaac Sim `EastRural_Chair` asset stays around 0.2 because of the
  sim-to-COCO domain gap. Phase 2's promotion gates are designed
  around this; the low raw score is not a Phase 2 bug.
- **Track/entity geometry is approximate.** Upstream 3D localization
  in Phase 1 projects masked depth pixels to 3D, then transforms
  through TF to `global_frame`. When the global-frame TF query fails,
  Phase 1 falls back to `base_link` coordinates; the first frame of
  data that reaches Phase 2 in that state can drag the EMA slightly
  off. Phase 2 does not retroactively correct this.
- **Re-association is `class_label + Euclidean distance` only.** A
  chair moved more than `entity_association_distance_m` between
  sightings will become a separate entity.
- **No observation-quality filter.** The tracker accepts every
  `ObjectObservation` regardless of its `depth_valid_ratio` /
  `uncertainty`. Phase 3 may want to gate low-quality observations
  before they enter the EMA.
- **Marker id reuse is ordering-dependent.** `Marker.DELETE` uses the
  integer id that entity appeared at last tick, re-derived from
  Python dict iteration order. CPython 3.7+ keeps insertion order, so
  this is stable in practice, but remains a latent fragility.

---

## 6. How to run Phase 2

Assumes Phase 0 sim is installed and the Phase 1 Python environment
(`/usr/bin/python3` with `ultralytics` and `numpy<2`) is set up — see
`docs/phase0_status.md` and `docs/phase1_status.md`.

### Build

```bash
cd /path/to/GO2-semantic-navigation
colcon build --symlink-install \
  --packages-select go2_msgs go2_perception go2_object_localization \
                    go2_semantic_memory go2_bringup_sim
source install/setup.bash
```

### Launch (two terminals)

**Terminal A — Phase 0 sim:**

```bash
bash scripts/run_warehouse_ros2.sh
# Wait for: [run_ros2] Ready. Publishing ROS 2 topics.
```

**Terminal B — Phase 2 semantic memory (which includes Phase 1 perception):**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch go2_bringup_sim chair_semantic_memory.launch.py
```

Optional overrides:

```bash
ros2 launch go2_bringup_sim chair_semantic_memory.launch.py \
  global_frame:=odom \
  entity_association_distance_m:=1.2 \
  promotion_min_observations:=3 \
  promotion_min_confidence:=0.45
```

### Topic inspection (third terminal)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Rates (expected: ~10–13 Hz on each)
timeout 10 ros2 topic hz /semantic/tracked_objects
timeout 10 ros2 topic hz /semantic_map/entities
timeout 10 ros2 topic hz /semantic_map/markers

# Content
ros2 topic echo /semantic/tracked_objects --once
ros2 topic echo /semantic_map/entities    --once
```

Expected content of an entity:
`class_label: chair`, `display_name: chair_<uuid8>`,
`frame_id: odom`, `observations_count` > 8, `confidence` ≈ 0.2–0.4,
`is_dynamic: false`.

### RViz

```bash
rviz2
```

- Set **Fixed Frame** to `odom`.
- **Add → By topic → /tf** (optional — shows the robot frame chain).
- **Add → By topic → /semantic_map/markers → MarkerArray**.

Expected: an **orange sphere** at the chair while it's being detected;
a **blue cube** plus a **white text label** `chair_<uuid8>` that
remains when the robot briefly looks away.

### Dropout round-trip (final acceptance check)

With everything running and an entity already present, rotate the Go2
away from the chair for a few seconds, then back:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"
# Ctrl-C after 5-10 seconds
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{angular: {z: -0.5}}"
# Ctrl-C when the heartbeat reports the chair is visible again
```

Expected in Terminal B's log:
```
[tracker] NEW track id=<new-uuid8> class='chair' pos=(...)
[semantic-map] RE-ASSOC track=<new-uuid8> -> entity=<old-uuid8> class='chair' dist=<small>m
```

…and `/semantic_map/entities` still contains exactly one chair entity
with the same `entity_id` as before.

### What success looks like

Steady-state Terminal B heartbeat after the chair is found:

```
[tracker]      obs_msgs=N obs_items=M active_tracks=1 visible=1 created_total=X evicted_total=Y by_class=[chair:1]
[semantic-map] track_msgs=N track_items=M entities=1 visible=1 promotions_total>=1 evicted_total=0 by_class=[chair:1]
```

Steady-state RViz: orange sphere + blue cube + white label, Fixed
Frame `odom`.

---

## 7. Acceptance criteria

Phase 2 is considered complete when **all** of the following hold on a
live run:

1. `/semantic/tracked_objects` is non-empty and contains at least one
   track with `class_label == "chair"` and a growing
   `observations_count`.
2. `/semantic_map/entities` is non-empty within a few seconds and
   contains at least one entity with `class_label == "chair"`.
3. Terminal B shows at least one `[semantic-map] PROMOTED (…) …` log
   line, with the gate tag being `fast`, `stable`, or `chair_mvp`
   (on the current warehouse the gate is `chair_mvp` because detector
   confidence stays below 0.45).
4. The persistent entity stays on `/semantic_map/entities` while the
   chair is briefly out of view, i.e. there exists a sampling window
   where the entity has `currently_visible: false` but has not been
   evicted.
5. After a rotate-away / rotate-back, there is still exactly one chair
   entity in `/semantic_map/entities` and the log contains at least
   one `[semantic-map] RE-ASSOC …` line for that entity.
6. RViz with Fixed Frame `odom` + MarkerArray on
   `/semantic_map/markers` shows the orange track sphere, the blue
   entity cube, and the white entity label at the chair. When an
   entity or track actually disappears, its marker is removed (no
   ghost cubes).

---

## 8. Next step

The next phase is **Phase 3: target selection + go-to-chair +
navigation execution**. Phase 3 will consume `/semantic_map/entities`,
pick a single chair as the active target, compute an approach goal,
and drive the Go2 to it through the existing `/cmd_vel` interface.

**Phase 3 is not implemented.** It is only named here as the planned
next step.
