# Day 8 acceptance — manual scene setup

This doc covers the **physical-scene-side** preparation for
`scripts/check_day8.sh` checks #3 (autonomous discovery) and #4
(exhausted-frontier → FAILED). The scripts themselves don't move
props — they only send `SemanticTask` and watch `/task/status`.

## Why the chair must be moved

The default warehouse (`sim/warehouse_scene.py`) spawns:

| object | XY | notes |
|---|---|---|
| Go2  | `(-4.0, -4.0)`, yaw `+45°` | facing the table/chair area |
| table | `(1.5, 1.0)` | within view from spawn |
| chair | `(2.7, 1.0)` | within view from spawn |

YOLOE sees both the table and the chair on the very first frame from
the spawn pose, so `target_class:=chair` triggers the **APPROACH** path
immediately and `EXPLORE` is never exercised. Day 8 check #3 tests
exactly the EXPLORE path, so the chair must start out of FOV.

## Two ways to hide the chair

### Option A — edit `sim/warehouse_scene.py` (persistent)

Pick coordinates behind one of the walls or in the far corner from
spawn. Recommended Day 8 location:

```python
# sim/warehouse_scene.py
CHAIR_XYZ = (3.5, -3.5, 0.0)   # far southeast corner
# or
CHAIR_XYZ = (-3.5, 3.5, 0.0)   # far northwest corner
```

With Go2 at `(-4, -4)` facing 45°, both corners are >90° off-axis at
spawn → no chance YOLOE picks the chair up before EXPLORE moves the
robot. Re-run `bash scripts/run_warehouse_ros2.sh` to apply.

### Option B — drag-and-drop in the Isaac Sim viewport (one-shot)

1. With the sim running, hit `Pause` (space).
2. In the stage browser, find `/World/Chair`.
3. Drag the gizmo to a far corner (e.g. `x=3.5, y=-3.5`) or behind a
   wall.
4. `Play` (space) again. The chair stays put for the rest of the
   session.

This avoids editing source but the scene resets next time you launch.

## Verifying "out of FOV" before sending the task

Before running `bash scripts/check_day8.sh`:

```bash
# Should show table_*** but NO chair_*** entity:
ros2 topic echo --once /semantic_map/objects | grep class_label
```

If `chair_xxx` already appears, the chair is still visible — move it
further or behind a more obstructing wall.

## Day 8 check #3 — what "PASS" looks like

Once the chair is hidden, the script publishes:

```yaml
SemanticTask{ task_id: "disc-001", target_class: "chair",
              requires_search: true, header: { frame_id: "map" } }
```

Expected `/task/status` sequence (script logs it for you):

```
PARSE_COMMAND       (or skipped if you used the SemanticTask path)
CHECK_MEMORY
TARGET_NOT_FOUND
EXPLORE             ← stays here while Nav2 drives to frontier #1
EXPLORE             ← may re-query frontiers a few times
CHECK_MEMORY        ← preempted: chair just appeared in /semantic_map/objects
TARGET_FOUND
PLAN_APPROACH_GOAL
NAVIGATE_TO_GOAL
VERIFY_TARGET
ARRIVED             ← gate #3 passes here
```

If the EXPLORE → CHECK_MEMORY edge never fires, the YOLOE detection
of the chair is failing — check `/detections` while Go2 stares at the
chair to confirm.

## Day 8 check #4 — what "PASS" looks like

The script sends `target_class:=microwave` (a class that does not
exist in the warehouse). Expected sequence:

```
CHECK_MEMORY
TARGET_NOT_FOUND
EXPLORE
EXPLORE
EXPLORE
...
FAILED:EXPLORE: environment fully explored, target 'microwave' not found
```

The `FAILED:` prefix and the substring `explored` / `not found` in the
reason are what the script grep's for.

If you instead see `ARRIVED`, perception falsely matched something —
tighten the YOLOE class list or `min_detection_confidence`.

If the script times out (default 5 min) without FAILED firing,
`frontier_explorer` is probably still finding tiny noise frontiers
forever. Live-tune:

```bash
ros2 param set /frontier_explorer min_cluster_size 25
```

(The 10x10 m warehouse is small; 25 cells × 0.05 m resolution ≈ 1.25 m
of frontier strip — anything smaller is sensor speckle.)
