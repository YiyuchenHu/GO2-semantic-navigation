# Day 7 Status — Semantic Target Navigation

**Status:** Code-complete, awaiting on-robot acceptance. Builds with
`colcon build --packages-select go2_semantic_perception go2_bringup_sim`.
Day 6 stack must be running and Nav2 `/navigate_to_pose` must be active.

## Goal

Close the loop from the Day 6 persistent semantic memory to Nav2's
NavigateToPose action server:

```
/semantic_map/objects (Day 6, persistent SemanticEntities)
        │
        ▼
target_selector_node            (Day 7, new)
   pick the best entity matching parameter `target_class`
        │
        ▼
/target/selected (go2_msgs/SelectedTarget)
        │
        ▼
approach_goal_planner_node      (Day 7, new)
   ring-sample around the target, costmap-filter, pick best,
   send NavigateToPose action
        │
        ▼
Nav2 /navigate_to_pose (Day 4, existing)
        │
        ▼
/cmd_vel → Go2
```

End-to-end behavior: when the operator runs `ros2 launch
go2_bringup_sim day7.launch.py target_class:=chair` and Go2 sees a
chair, the chair is added to semantic memory, `target_selector` picks
it, `approach_goal_planner` computes a stand-off pose 0.9 m away, and
Nav2 drives the Go2 there.

## What is new

| File | Role |
| --- | --- |
| `src/go2_semantic_perception/go2_semantic_perception/target_selector_node.py` | Pick best `SemanticEntity` matching `target_class` param. Publishes `SelectedTarget`. |
| `src/go2_semantic_perception/go2_semantic_perception/approach_goal_planner_node.py` | Ring-sample around target, filter by Nav2 costmap, send `NavigateToPose` action goal. |
| `src/go2_bringup_sim/launch/day7.launch.py` | Orchestrates the full Day 5 + Day 6 + Day 7 stack with one launch file. |
| `scripts/check_day7.sh` | Acceptance script (8 hard checks). |
| RViz config | New displays: `Approach goal (Day 7)` (Pose arrow), `Approach candidates (Day 7)` (MarkerArray). |

## Architectural choices (and the alternatives)

### Two nodes, not one combined `semantic_goto`
**Chosen:** `target_selector` + `approach_goal_planner` running as
separate processes, talking via `/target/selected`.

**Why:** Selection (entity scoring) and goal generation (geometric ring
sampling + costmap math) have orthogonal failure modes. Splitting them
means `ros2 topic echo /target/selected` immediately tells the operator
"selection is fine, planning is what failed", or vice-versa. Folding
them would require a single `semantic_goto` debug topic that
multiplexes both layers' state.

### Reused `go2_msgs/SelectedTarget`, did not invent a new msg
**Chosen:** Reuse the Phase 3A `SelectedTarget` message verbatim
between the two Day 7 nodes.

**Why:** `SelectedTarget` is class-agnostic (entity_id + class_label +
target_pose_map + score + ranking_reasons) — its name is the only
Phase 3A artifact. Adding a new message type just to escape the
"Phase 3A" naming is bureaucratic.

### NavigateToPose action client, not `/goal_pose` topic
**Chosen:** `rclpy.action.ActionClient(NavigateToPose)`.

**Why:** The action gives us cancel-on-target-change, feedback
streaming (`distance_remaining`, `number_of_recoveries`), and explicit
result codes that distinguish "Nav2 reached the goal" from "Nav2 BT
fell back to recovery and gave up". The `/goal_pose` topic path also
works (Day 4 verified) and is fire-and-forget — fine for one-shot
demos, insufficient for Day 8+ task supervision. Cost: action client
boilerplate adds ~80 lines of futures + status callbacks.

### `target_class` is a ROS parameter, not a topic
**Chosen:** `ros2 param set /target_selector target_class table` to
switch class.

**Why:** Day 7's MVP is a single class at a time. Day 10's command
interface (LLM / voice) becomes a thin wrapper around `param set`.
Subscribing to a String topic would add another moving part for no
behavioural difference at Day 7.

### Selector publishes empty `SelectedTarget` when no target found
**Chosen:** Always publish at `select_period_sec` (0.5 s); empty
`entity_id` signals "no candidate".

**Why:** Lets the planner cancel an in-flight goal cleanly when the
operator points the Go2 elsewhere. Alternative — go silent — would
leave the planner with a stale goal forever.

### Per-class approach distance lives in the planner, not a config file
**Chosen:** ROS parameters: `approach_distance_chair=0.9`,
`approach_distance_table=1.0`, `approach_distance_box=0.7`, …
default 0.9 for unlisted classes.

**Why:** The set is small (5–10 classes) and all values are physical
constants tied to the Go2's footprint. A YAML file would be lonely
and add a load step. Day 10+ may revisit if class-specific arm-reach
data needs persisting.

## How to run

Three shells (Nav2 must come up before Day 7 to avoid action-server
warnings):

```bash
# Shell 1 — sim
bash scripts/run_warehouse_ros2.sh

# Shell 2 — static TFs (chair_perception_node will crash on import,
# this is harmless and known; see docs/known_issues.md numpy ABI)
ros2 launch go2_bringup_sim chair_perception.launch.py

# Shell 3 — Nav2 (slam_toolbox + bt_navigator + costmaps).
# Wait for "Managed nodes are active" before starting Day 7.
ros2 launch go2_bringup_sim nav2.launch.py

# Shell 4 — Day 7 stack
ros2 launch go2_bringup_sim day7.launch.py
```

Switch class at runtime:

```bash
ros2 param set /target_selector target_class table
ros2 param set /target_selector min_confidence 0.4
```

Tune approach distance live:

```bash
ros2 param set /approach_goal_planner approach_distance_chair 1.1
ros2 param set /approach_goal_planner cost_threshold 75
```

## Acceptance — `scripts/check_day7.sh`

The script runs 6 hard checks (PASS/FAIL) plus 4 soft eyeball
checks. Hard checks:

1. **Day 6 + Day 7 nodes alive** — `ros2 node list` includes
   `/yoloe_detector`, `/depth_projector`, `/semantic_memory_aggregator`,
   `/target_selector`, `/approach_goal_planner`.
2. **Day 7 topics advertised** — `/target/selected`,
   `/semantic_goal/goal_pose`, `/semantic_goal/goal_candidates`,
   plus the Day 6 prerequisite `/semantic_map/objects` and Nav2's
   `/global_costmap/costmap`.
3. **`/navigate_to_pose` action server reachable** — `ros2 action list`
   contains it AND `ros2 action info` reports `Action servers: [1+]`.
4. **`/target/selected` flow rate** — ≥ 1 Hz from `target_selector`
   (the selector's housekeeping timer guarantees this even if no
   target is found).
5. **`/semantic_goal/goal_pose` content** — when published, has
   `frame_id="map"`, finite XY, unit quaternion.
6. **`/semantic_goal/goal_candidates` content** — at least one
   `viable` or `rejected` ring-sample marker per replan tick when a
   target is selected.

Soft (operator-eyeball, depend on a chair being in FOV + Nav2 costmap
populated):

* `target_selector` actually picks a non-empty `entity_id`.
* `approach_goal_planner` actually sends a NavigateToPose goal that
  Nav2 accepts.

## Tunable parameters cheat sheet

| Parameter | Where | Default | When to change |
| --- | --- | --- | --- |
| `target_class` | target_selector | `chair` | The class the Go2 should drive to. Day 10 command interface rewrites this live. |
| `min_confidence` | target_selector | `0.30` | Raise to ignore decayed memory ghosts; lower for noisy detectors. |
| `score_weight_visible` | target_selector | `1.0` | Bump higher to *strictly* prefer currently-visible entities (avoids re-driving to ghosts). |
| `num_angle_samples` | approach_planner | `16` | Raise to `32` in narrow corridors where most ring directions are blocked. |
| `approach_distance_default` | approach_planner | `0.9` m | Per-class overrides take precedence; default applies to classes the planner has never seen. |
| `cost_threshold` | approach_planner | `60` | 0=free, 100=lethal. Raise to 75–80 for permissive costmap; lower to 40 for very cautious approach. |
| `replan_period_sec` | approach_planner | `1.0` s | How often the planner re-evaluates. Lower for moving targets. |
| `replan_distance_m` | approach_planner | `0.10` m | Skip resending the goal when target jittered less than this since last send. Raise to 0.20–0.30 if Nav2 thrashes between two ring positions. |
| `score_alignment_weight` | approach_planner | `0.5` | How much to penalise goals that need a >90° in-place rotation up-front. |

## Day 6 acceptance gating Day 7

Before Day 7 will work, `check_day6.sh` must report PASS on:

* `/semantic_map/objects` populated
* SemanticEntity `class_label` field matches one of the YOLOE prompts
  (Day 6 launch `classes` arg)
* SemanticEntity `pose_map` is in `map` frame, finite

If `class_label` doesn't match, fix at YOLOE (`classes:='[…]'`) — the
selector matches on string equality (case + space normalised).

## Pitfalls hit during development (and probably again)

### 1. Nav2 not up → `/navigate_to_pose` missing
The planner logs `action server '/navigate_to_pose' not available; not
sending goal` every replan tick. Symptoms: `/semantic_goal/goal_pose`
publishes (planner still computes the pose for RViz) but Go2 doesn't
move. Fix: `ros2 launch go2_bringup_sim nav2.launch.py`, wait for
"Managed nodes are active".

### 2. `class_label` mismatch
Operator sets `target_class:=chair` but YOLOE labels the object
`'chair'` vs `'office chair'`. Selector compares normalised strings
(`'office_chair' != 'chair'`). Two fixes:
- Match `target_class` to what YOLOE actually publishes:
  `ros2 topic echo /semantic_map/objects | grep class_label`
- Or expand YOLOE prompts to map all variants to `'chair'`. Day 5's
  default already includes `office chair`, `stool`, `armchair` —
  YOLOE labels them `chair` because that's the FIRST class-name in
  the list and `set_classes()` uses that index for label.

### 3. Costmap `transient_local` durability mismatch
Nav2's `/global_costmap/costmap` is published TRANSIENT_LOCAL latched
(once on startup, then on map growth). Day 7 planner subscribes with
matching durability — if you swap to a SLAM source that publishes
VOLATILE you'll see "no costmap" warnings forever. Fix: ensure costmap
publisher is TRANSIENT_LOCAL (Nav2 default) and confirm with
`ros2 topic info -v /global_costmap/costmap`.

### 4. EMA jitter → goal-resend churn
Day 6's semantic memory updates entity position with α=0.3 EMA on every
detection. With detector at 8 Hz and target at 5 m range, the position
oscillates ~5–10 cm/s. Planner's `replan_distance_m=0.10` mostly
absorbs it; if you see Nav2 receiving 10+ goals/second, raise to 0.20.

### 5. All ring samples rejected
Symptoms: `goal_candidates` MarkerArray has 16 red, 0 green; planner
warns "no costmap-clear approach pose". Causes:
- Target is touching a wall — every ring sample falls in inflation.
  Lower `cost_threshold` or `approach_distance_default`.
- Costmap inflation_radius too aggressive for the room. Tune Nav2,
  not Day 7.
- `num_angle_samples` too low (corridor with only one viable
  direction at 22.5° resolution). Raise to 32 or 64.

### 6. SelectedTarget "stale" between target switches
Operator runs `ros2 param set /target_selector target_class table`
mid-traverse. The selector picks the new target but the planner still
has the old one in flight. Planner correctly cancels the old goal
when the entity_id changes (`_last_sent_entity_id != sel.entity_id`),
verified in code. If you see the Go2 still pursuing the old target,
check the planner's logs for "selected target cleared; canceling
in-flight goal".

## Carry-over from Day 6 (still relevant)

* Isaac Sim LiDAR stalls (~30 s gaps) cause Nav2 to abort. Day 7's
  planner sees the abort via NavigateToPose result and logs it; on
  the next replan tick it sends a fresh goal. No fix at Day 7 layer.
* `cv_bridge` numpy ABI crash in `chair_perception_node` is unrelated
  and harmless to Day 7 (which doesn't import cv_bridge directly).
* `slam_toolbox` `transform_publish_period` is at 0.1 s; Day 7
  inherits Day 4's intermittent map→odom availability.

## What's next (Day 8 preview)

* `target/selected` and `semantic_goal/goal_pose` are now first-class
  topics; Day 8 introduces a thin "task supervisor" that consumes the
  NavigateToPose action's feedback / result and surfaces it as
  `/navigation/status`. The supervisor is also where retries on
  `STATUS_ABORTED` get a budget — Day 7's planner currently does not
  retry, just waits for the next replan tick.
