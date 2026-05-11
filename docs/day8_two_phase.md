# Day 8 — two-phase variant: autonomous mapping + NL command

A simpler alternative to the original `day8.launch.py`. The core idea is
to **decouple** "drive Go2 around to fill in the SLAM map" from "tell
Go2 where to go". Both run in one launch, but the data flow is
strictly two-phase:

```
┌──────────────────── Phase A (autonomous, hands-off) ─────────────────┐
│                                                                       │
│  /scan ──► slam_toolbox ──► /map                                      │
│                              │                                        │
│  /lidar/points ──► perception ──► /semantic_map/objects               │
│   (yoloe + depth_projector + semantic_memory)                         │
│                                                                       │
│  /map + costmap ──► frontier_explorer ──► /get_frontiers              │
│                                              │                        │
│                              mapping_explorer ──► /navigate_to_pose   │
│                                              │                        │
│                              /mapping/status: IDLE → NAVIGATING       │
│                                                  ... → DONE           │
└───────────────────────────────────────────────────────────────────────┘
                                  │ (entities persist on SLAM map)
                                  ▼
┌──────────────────── Phase B (NL command-driven) ─────────────────────┐
│                                                                       │
│  operator: ros2 topic pub /user_command "data: 'go to chair'"         │
│                              │                                        │
│  /user_command ──► nl_parser ──► /semantic_task/request               │
│                                              │                        │
│  /semantic_map/objects ──► task_coordinator (CHECK_MEMORY hits        │
│                              immediately because phase A populated    │
│                              the chair entity already)                │
│                                              │                        │
│                              ──► PLAN_APPROACH_GOAL                   │
│                              ──► NAVIGATE_TO_GOAL                     │
│                              ──► ARRIVED                              │
└───────────────────────────────────────────────────────────────────────┘
```

Why this design over the original `day8.launch.py`
--------------------------------------------------

The original couples exploration with target search inside `task_coordinator`'s
FSM:

* `EXPLORE` is entered when the target class isn't yet in semantic memory
* During `EXPLORE`, `task_coordinator` itself drives `NavigateToPose`
* If Nav2 ABORTs three goals in a row → whole task `FAILED`
* The chair must start outside Go2's spawn FOV or `EXPLORE` is never exercised

That works for a single-shot demo, but every subtle Nav2 issue
(corridor too tight, pose extrapolation lag, costmap not yet fully
inflated) translates into "Day 8 demo failed". We saw all of these
in the day8.sh runs.

The two-phase variant fixes it by **separating concerns**:

* `mapping_explorer_node` is a single-purpose ~250-line node that only
  knows how to drive frontiers. A single ABORT is logged and skipped,
  not propagated as a task failure. Once `/map` has no more frontiers,
  it locks `DONE` and stays out of Nav2's way.
* `task_coordinator` runs in its **target-driven** mode (no
  `default_target_class`, no PARSE_COMMAND fallback). It sits in
  IDLE until the operator sends a real semantic task — exactly the
  Day 7 path, no surprise EXPLORE round-trip.
* `nl_parser_node` is the new operator hook: a tiny regex + keyword
  fuzzy-matcher that turns `/user_command` strings into
  `/semantic_task/request` messages. No spaCy / transformers / ollama;
  pure Python `difflib` against a configurable class whitelist.
* The legacy `day8.launch.py` is preserved unchanged — pick whichever
  fits the demo of the day.

---

## Terminal layout

You will run **6 terminals**. Open them in order; each must reach a
"steady state" before the next is brought up.

| # | Command | When ready you see... |
|---|---------|----------------------|
| T1 | `bash scripts/run_warehouse_ros2.sh` | Sim window with the warehouse + Go2; terminal stops scrolling |
| T2 | `ros2 launch go2_bringup_sim tf_and_scan.launch.py` | "static_transform_publisher" lines, no errors |
| T3 | `ros2 launch go2_bringup_sim nav2.launch.py slam:=True` | `Managed nodes are active` from `lifecycle_manager` |
| T4 | `ros2 launch go2_bringup_sim day8_two_phase.launch.py` | `mapping_explorer ready` + `nl_parser ready` |
| T5 | `bash scripts/run_rviz.sh` | Map + lidar + frontier markers visible |
| T6 | (unused; leave free for `ros2 topic pub` / teleop) | n/a |

> All 6 terminals must source the workspace first:
> ```
> source /opt/ros/jazzy/setup.bash && source install/setup.bash
> ```
> The convenience script already does this for you in T1/T5; do it
> manually in T2/T3/T4/T6.

---

## Phase A — let Go2 map the warehouse

Once T4 is up, **do nothing**. `mapping_explorer` will:

1. Wait for `/get_frontiers` service + first `map → base_link` TF (a few seconds).
2. Send the highest-scoring frontier as a `NavigateToPose` goal.
3. On `SUCCEEDED`, query the next frontier; on `ABORTED`, log + try the next.
4. When `/get_frontiers` returns an empty list and stays empty for
   `done_confirm_sec` (default 5 s), publish `/mapping/status: DONE`.

What you should see in **T6** while it runs:

```
ros2 topic echo /mapping/status --field data
```

Output progression: `IDLE` → `NAVIGATING` → ... → `DONE`. Typical
duration in the 10×10 m warehouse: **2–4 minutes**.

What you should see in **RViz (T5)** during Phase A:

* `/map` (grey OccupancyGrid) keeps growing.
* `/frontier_markers` (green/red spheres) appear on the boundary; the
  green one is the next goal.
* `/semantic_map/markers` (text labels) appear on chair / table /
  desk / box as Go2 passes them. They **persist** even when Go2 turns
  away, thanks to `permanent_after_observations`.

If Phase A hangs:

```
ros2 topic echo /mapping/status        # any FAILED:<reason>?
ros2 topic echo /frontier_markers --once  # any frontiers left?
ros2 param set /frontier_explorer min_cluster_size 20  # drop noisy ones
ros2 param set /frontier_explorer safety_radius_m 0.55  # stay further from walls
ros2 topic pub --once /mapping/control std_msgs/msg/String "{data: 'restart'}"
```

To force-end Phase A early (e.g. you saw enough for the recording):

```
ros2 topic pub --once /mapping/control std_msgs/msg/String "{data: 'abort'}"
```

---

## Phase B — issue NL commands

Once `/mapping/status: DONE`, the warehouse is mapped and at least
the chair / table are in `/semantic_map/objects`. Now you drive Go2
with English.

In **T6** publish a command (single quotes inside the YAML matter):

```
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'go to chair'}"
```

What happens:

1. `nl_parser` echoes the parse on `/nl_parser/feedback`:
   `OK task_id=nl-0001 target_class='chair' conf=1.00 raw='go to chair'`
2. `nl_parser` publishes a `SemanticTask` on `/semantic_task/request`.
3. `task_coordinator` enters `CHECK_MEMORY` → `TARGET_FOUND` (because
   the chair entity is already in memory) → `PLAN_APPROACH_GOAL` →
   `NAVIGATE_TO_GOAL` → `ARRIVED`.
4. Go2 walks to the chair, RViz shows the goal pose marker.

Other commands the parser handles out of the box:

| Command | Parses to |
|---------|-----------|
| `go to chair` | `chair` (exact label) |
| `find the table` | `table` (exact label) |
| `please navigate to the desk` | `desk` (exact label, distinct from `table`) |
| `I want you to fetch the box` | `box` (exact label) |
| `go to the office chair` | `chair` (multi-word synonym) |
| `please find a seat` | `chair` (synonym) |
| `walk over to the desks` | `desk` (fuzzy, conf 0.89) |
| `chiar` | `chair` (fuzzy, typo, conf 0.80) |
| `look for crates` | `box` (synonym) |

What gets rejected (you'll see it on `/nl_parser/feedback`):

| Command | Outcome |
|---------|---------|
| `navigate over there` | `REJECT` — no content token matches a known class |
| `open the door` | `REJECT` if `min_match_confidence ≥ 0.85`; may match `microwave` at the default 0.65 floor (set `nl_known_classes` to exclude `microwave` and the issue disappears). |

To monitor what task_coordinator is doing in real time (T6):

```
ros2 topic echo /task/status --field data
```

To switch to a different class without restarting anything (T6):

```
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'go to table'}"
```

`task_coordinator`'s state machine resets cleanly at every new task
(thanks to the bug fix landed alongside the two-phase work).

---

## Acceptance — `scripts/check_day8_two_phase.sh`

Five gates total; all five must PASS before Day 8 (two-phase) is
"green":

1. **NLP UNIT** — synthetic command battery against the parser logic.
   Pure Python, no ROS.
2. **PHASE A MAPPING** — wait up to 600 s for `/mapping/status: DONE`
   AND `/semantic_map/objects` to contain ≥ 1 entity.
3. **NLP PARSE (live)** — publish `go to chair` on `/user_command`,
   confirm `/semantic_task/request.target_class` becomes `chair`
   within 8 s.
4. **END-TO-END NAV** — publish the same command, watch
   `/task/status` until `ARRIVED` (default timeout 180 s — much
   shorter than `day8.sh`'s 300 s because Phase A pre-populated
   semantic memory).
5. **MULTI-CLASS SWITCH** — repeat with the other class
   (chair → table or table → chair) to prove the FSM resets cleanly
   between commands.

```
bash scripts/check_day8_two_phase.sh                    # interactive prompts
bash scripts/check_day8_two_phase.sh --auto             # skip prompts
bash scripts/check_day8_two_phase.sh --phase-a-to 300   # tighter timeout
bash scripts/check_day8_two_phase.sh --target-class table  # demo with table first
```

---

## Tunable parameters cheat-sheet

All settable via `ros2 launch ... <arg>:=<value>` or live with
`ros2 param set`.

### `mapping_explorer`

| Param | Default | Notes |
|-------|---------|-------|
| `done_confirm_sec` | 5.0 | Hold zero-frontiers this long before locking DONE |
| `max_consecutive_aborts` | 4 | Skip a frontier on ABORT, FAIL after this many in a row |
| `done_fast` | False | If True and at least one frontier was reached, lock DONE on the very first empty response (skip confirm hold) |

### `nl_parser`

| Param | Default | Notes |
|-------|---------|-------|
| `known_classes` | `['chair','table','desk','box']` | Whitelist; tighten to scene contents |
| `min_match_confidence` | 0.65 | Lower → more accepted commands but more false positives |
| `synonyms` | `[]` (use defaults) | `["chair:seat,armchair", ...]` per-class overrides |

### `frontier_explorer` (same as `day8.launch.py`)

| Param | Default | Notes |
|-------|---------|-------|
| `safety_radius_m` | 0.4 | Distance the centroid must keep from any /map occupied cell |
| `costmap_safe_max_cost` | 75 | TB3-style cost rejection threshold |
| `min_cluster_size` | 10 | Drop frontier clusters smaller than this |

---

## Files added by this work

| Path | Purpose |
|------|---------|
| `src/go2_navigation/go2_navigation/mapping_explorer_node.py` | Phase A driver |
| `src/go2_navigation/setup.py` (entry_point) | Register `mapping_explorer_node` |
| `src/go2_nl_parser/` (new package) | NL command parser |
| `src/go2_bringup_sim/launch/day8_two_phase.launch.py` | One-shot launch for both phases |
| `src/go2_bringup_sim/setup.py` (data_files) | Install the new launch |
| `scripts/check_day8_two_phase.sh` | 5-gate acceptance script |
| `docs/day8_two_phase.md` | This file |

---

## Build + run summary

```bash
cd <workspace>
colcon build --symlink-install \
  --packages-select go2_msgs go2_navigation go2_nl_parser \
                    go2_bringup_sim go2_task_coordinator \
                    go2_semantic_perception
source install/setup.bash

# T1
bash scripts/run_warehouse_ros2.sh

# T2
ros2 launch go2_bringup_sim tf_and_scan.launch.py

# T3
ros2 launch go2_bringup_sim nav2.launch.py slam:=True
# wait for "Managed nodes are active"

# T4
ros2 launch go2_bringup_sim day8_two_phase.launch.py

# T5
bash scripts/run_rviz.sh

# T6 (operator console; do this AFTER /mapping/status reads DONE)
ros2 topic pub --once /user_command std_msgs/msg/String "{data: 'go to chair'}"

# Acceptance gate
bash scripts/check_day8_two_phase.sh
```
