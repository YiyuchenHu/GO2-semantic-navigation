# Phase 4 — Search / Reacquisition Enhancement

> **Evidence used for this document**
>
> - Source of truth: the current code at
>   `src/go2_navigation/go2_navigation/search_manager_node.py`
>   (rewritten from stub),
>   `src/go2_navigation/go2_navigation/backends/simple_p_controller_backend.py`
>   (new `controller.publish_zero_when_idle` parameter only),
>   `src/go2_bringup_sim/launch/chair_with_search.launch.py` (new),
>   `src/go2_bringup_sim/launch/chair_execute_goal.launch.py` (new
>   `controller_publish_zero_when_idle` launch arg, pass-through only),
>   `src/go2_bringup_sim/setup.py` (launch registration).
> - Phase 0 / 1 / 2 / 3A / 3B are **not modified**. No message schemas
>   change.

## 1. Phase 4 overview

Phase 4 adds a thin **search / reacquisition** layer on top of the
existing stack. Its purpose is simple and narrow:

> When the chair is not currently visible, rotate in place so the
> RGB-D camera sweeps the scene, resume pursuit the instant upstream
> semantic memory sees the chair again, and stop rotating if the
> sweep goes on too long.

Phase 4 is strictly an MVP reacquisition behaviour. It is **not**
frontier exploration, not SLAM viewpoint selection, not a behaviour
tree, not a gait replacement. Those belong to later phases.

## 2. Phase 4 goals

- Don't stall silently when the chair is missing — do a concrete
  sweep.
- Hand control back to Phase 3B the instant the chair reappears.
- Keep a single bounded time budget; declare `LOST` when exhausted so
  the robot stops rather than spinning forever.
- Preserve package and phase boundaries: Phase 4 consumes Phase 0 /
  1 / 2 / 3A / 3B outputs read-only and does not touch their code
  (the one exception is a purely-additive parameter on the Phase 3B
  controller; its default behaviour is unchanged).
- Chair-only MVP. Parameter surface already carries `target_class`
  for future extension, but only `chair` has been exercised.
- Publish a single new status topic `/search/status` + one optional
  marker topic `/search/markers` so the operator can see exactly
  what layer currently owns `/cmd_vel`.

## 3. What Phase 4 implemented

### 3.1 `search_manager_node` (rewritten)

Path: `src/go2_navigation/go2_navigation/search_manager_node.py`.

**Subscribes (read-only):**

| Topic | Type | Role |
|---|---|---|
| `/semantic_map/entities` | `go2_msgs/SemanticEntityArray` | `currently_visible` flag of the chair entity |
| `/perception/objects_3d` | `go2_msgs/ObjectObservationArray` | timestamp of the most recent raw chair detection |
| `/navigation/status` | `std_msgs/String` | Phase 3B state (ROTATING/MOVING/REACHED/…) |
| `/arrival/status` | `std_msgs/String` | Phase 3B arrival verdict |
| `/odom` | `nav_msgs/Odometry` | current robot pose for marker placement |

**Publishes:**

| Topic | Type | Rate |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 10 Hz **only while `SEARCHING`**; zero Twist once on leaving `SEARCHING` |
| `/search/status` | `std_msgs/String` | 10 Hz |
| `/exploration/enabled` | `std_msgs/Bool` | 10 Hz (true iff `SEARCHING`) — kept alive for any downstream consumer that was already listening |
| `/search/markers` | `visualization_msgs/MarkerArray` | 10 Hz |

**Parameters (launch defaults shown):**

| Parameter | Default | Purpose |
|---|---|---|
| `target_class` | `chair` | which class counts as "the target" |
| `recent_visible_sec` | 2.0 s | target must have been seen (entity `currently_visible` or raw 3D hit) within this window to count as present |
| `search_timeout_sec` | 30.0 s | max sweep duration before declaring `LOST` |
| `search_angular_rate` | 0.4 rad/s | yaw rate in `SEARCHING` (~ ½ of Phase 3B `max_angular`) |
| `loop_hz` | 10.0 | control-loop rate |
| `log_period_sec` | 2.0 | heartbeat period |
| `global_frame` | `odom` | frame for marker placement |

### 3.2 Phase 3B extension (single-parameter addition only)

`SimplePControllerBackend` gains one new parameter:

```
controller.publish_zero_when_idle   default True   bool
```

When **False**, the backend stops publishing zero `Twist` while in
`IDLE` / waiting-for-odom. `REACHED` still publishes zero (the
controller's own braking signal — do not interpret it as a search
hand-over). `MOVING` / `ROTATING` behaviour is identical to before.

`chair_execute_goal.launch.py` exposes this as the
`controller_publish_zero_when_idle` launch arg, **default `true`**,
so every existing launch (including running Phase 3B alone) behaves
exactly as it did before Phase 4 existed. `chair_with_search.launch.py`
overrides this arg to `false`.

This is the only cross-phase change. Nothing else in Phase 0 / 1 / 2 /
3A / 3B was touched.

### 3.3 New launch: `chair_with_search.launch.py`

Composition:

1. Includes `chair_execute_goal.launch.py` with
   `controller_publish_zero_when_idle:=false`. That transitively
   brings up Phases 1, 2, 3A, and 3B with nav_executor staying
   silent on `/cmd_vel` while IDLE.
2. Starts `search_manager_node` with the Phase 4 parameters above.

Explicitly does **not** start:
- `task_coordinator_node` — deferred to the full-MVP orchestration
  phase.
- any safety / shared-autonomy node — out of scope.
- any frontier exploration or SLAM component — out of scope.

Registered in `src/go2_bringup_sim/setup.py` alongside the existing
launch files.

## 4. State machine / transition logic

Six states, published verbatim on `/search/status`:

| State | Entry condition | `/cmd_vel` output |
|---|---|---|
| `IDLE` | initial fall-through while Phase 3B has not yet declared a status | none |
| `SEARCHING` | Phase 3B `nav_status ∈ {IDLE, CANCELED, …, GOAL_REJECTED…}` **and** chair has not been observed within `recent_visible_sec` | `Twist(angular.z = search_angular_rate)` at 10 Hz |
| `REACQUIRED` | chair observed within `recent_visible_sec` **and** Phase 3B is not yet pursuing | none (one-tick latch; flips to `PURSUING` as soon as nav_executor picks up the new goal) |
| `PURSUING` | `nav_status ∈ {ROTATING, MOVING, REACHED}` and `/arrival/status` is **not** `ARRIVED_CONFIRMED` | none (Phase 3B owns `/cmd_vel`) |
| `ARRIVED` | `nav_status ∈ {ROTATING, MOVING, REACHED}` and `/arrival/status == ARRIVED_CONFIRMED` | none |
| `LOST` | continuous time in `SEARCHING` exceeds `search_timeout_sec` | none (one zero Twist on entry, then silent) |

Priority: **nav_status dominates.** While Phase 3B is actively driving
(`ROTATING/MOVING/REACHED`), Phase 4 defers to `PURSUING` / `ARRIVED`
and never publishes `/cmd_vel`. Search is only allowed to own
`/cmd_vel` when Phase 3B is IDLE, and is exited immediately the
moment a fresh chair observation lands (on either the entity stream
or the raw 3D observation stream).

Transition logging: an INFO line on **every** edge, e.g.

```
[search] IDLE -> SEARCHING
[search] SEARCHING -> REACQUIRED
[search] REACQUIRED -> PURSUING
[search] PURSUING -> ARRIVED
```

A 2 s heartbeat always shows the live gate values:

```
[search/hb] state=SEARCHING nav_status=IDLE arrival=WAITING_FOR_TARGET
           chair_in_memory=False chair_seen=never sweep_age=7.4s
```

Leaving `SEARCHING` on any edge also publishes one zero Twist, so
Phase 0's `CmdVelDriver` does not keep integrating the last rotation
command after hand-off.

## 5. Logs, markers, status topics

### Status topic

```
ros2 topic echo /search/status
---
data: SEARCHING
---
data: SEARCHING
---
data: REACQUIRED
---
data: PURSUING
---
```

### Markers (`/search/markers`)

- `TEXT_VIEW_FACING` floating above the robot: `SEARCH: <state>`,
  coloured amber (searching), green (arrived), red (lost), white
  (other).
- Thin amber `CYLINDER` on the ground around the robot while
  `SEARCHING`, as a visual "this is the sweep radius" cue. It is
  purely cosmetic; there is no real sensor-radius computation
  behind it.

### Legacy flag

`/exploration/enabled` (bool) is still published at 10 Hz and is
`true` iff the current state is `SEARCHING`. This preserves the
interface the old stub exposed in case anything downstream was
already listening.

## 6. Acceptance criteria

All must hold in a normal run against the warehouse sim with a
chair in the scene:

1. **Cold start**: at launch, no chair entity exists yet. Phase 3B's
   `/cmd_vel` is silent (parameter override), `search_manager` emits
   `IDLE -> SEARCHING`, and the robot visibly rotates in place.
2. **Reacquisition**: as soon as perception promotes a chair (either
   `/perception/objects_3d` reports a hit or the entity becomes
   `currently_visible`), `/search/status` flips `SEARCHING ->
   REACQUIRED` within ≤ 1 control tick (100 ms).
3. **Hand-off**: within one or two Phase 3A ticks (≤ 1 s) after
   `REACQUIRED`, Phase 3A publishes a selected target + goal pose,
   Phase 3B's `/navigation/status` flips to `ROTATING`, and
   `/search/status` flips `REACQUIRED -> PURSUING`.
4. **Arrival pass-through**: when Phase 3B's arrival_verifier flips
   to `ARRIVED_CONFIRMED`, `/search/status` flips `PURSUING ->
   ARRIVED`. No `/cmd_vel` was published by `search_manager_node`
   during `PURSUING` or `ARRIVED`.
5. **Loss → reacquire loop**: if the chair is manually moved out of
   sight during `PURSUING` and Phase 3B's goal is cancelled (e.g. via
   `ros2 topic pub -1 /navigation/cancel std_msgs/Bool "data: true"`),
   `/search/status` returns to `SEARCHING` and the robot starts
   rotating again.
6. **Timeout**: if the chair is absent for the entire
   `search_timeout_sec` window (default 30 s), `/search/status` flips
   to `LOST`, `/cmd_vel` becomes silent (one final zero Twist), and
   the robot stops rotating. A subsequent real chair detection flips
   the state out of `LOST` back into `REACQUIRED`.
7. **No `/cmd_vel` contention**: at no point do both
   `nav_executor_node` and `search_manager_node` publish non-zero
   `Twist` on `/cmd_vel` within the same tick. (Verifiable by
   `ros2 topic hz /cmd_vel` showing a clean 10 Hz during `SEARCHING`
   and a clean 10 Hz during `MOVING`/`ROTATING`, with a clear break
   between the two regimes.)

## 7. How to run Phase 4

### Build

```bash
cd ~/<repo>/GO2-semantic-navigation
colcon build --symlink-install \
  --packages-select go2_navigation go2_bringup_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Prerequisite: Phase 0 sim running in another shell

```bash
bash scripts/run_warehouse_ros2.sh
```

### Launch Phase 4 (includes Phases 1 + 2 + 3A + 3B)

```bash
ros2 launch go2_bringup_sim chair_with_search.launch.py
```

Optional overrides:

```bash
ros2 launch go2_bringup_sim chair_with_search.launch.py \
     search_timeout_sec:=45.0 \
     search_angular_rate:=0.3
```

### Live topic checks

```bash
timeout 5 ros2 topic hz /search/status              # ~10 Hz, always
timeout 5 ros2 topic hz /cmd_vel                    # ~10 Hz while SEARCHING or MOVING
timeout 5 ros2 topic hz /navigation/status          # ~5 Hz (Phase 3B)
timeout 5 ros2 topic hz /arrival/status             # ~2 Hz (Phase 3B)
ros2 topic echo /search/status
```

Expected log pattern on first run:

```
[search] IDLE -> SEARCHING
[search/hb] state=SEARCHING nav_status=IDLE ... chair_seen=never sweep_age=1.2s
...
[search] SEARCHING -> REACQUIRED
[search] REACQUIRED -> PURSUING
[nav-exec] NEW goal frame='odom' pos=(-0.30, 0.07) yaw=3.14rad
[nav-exec] ROTATING ... MOVING ... REACHED
[arrival] ARRIVED_CONFIRMED target=chair dist=0.92m ...
[search] PURSUING -> ARRIVED
```

### Force search manually

Cancel Phase 3B's in-flight goal to fall back to search:

```bash
ros2 topic pub -1 /navigation/cancel std_msgs/Bool "data: true"
```

`/navigation/status` flips to `CANCELED`, the controller clears its
goal, `search_manager_node` decides the chair is not currently fresh
(Phase 3A stops sending goals when memory clears) and re-enters
`SEARCHING`.

### Running Phase 3B alone (regression)

Phase 3B on its own keeps the default behaviour:

```bash
ros2 launch go2_bringup_sim chair_execute_goal.launch.py
```

`controller_publish_zero_when_idle` is `true` by default; nothing in
Phase 3B's behaviour changes relative to the pre-Phase-4 state.

## 8. Known limitations of Phase 4

1. **Single behaviour: pure rotate-in-place.** The sweep is a
   constant angular rate in one direction. There is no left/right
   ratchet, no translation, no "look back where I came from" logic.
2. **Search stops where it started.** The robot does not translate
   during `SEARCHING`, so it can only reacquire chairs that become
   visible from its current base frame. If the chair is in another
   room, the robot will time out to `LOST` without finding it.
3. **No obstacle awareness during sweep.** The rotation is
   unconditional. The scene is empty enough for this to be fine, but
   there is no costmap gate.
4. **Recovery from `LOST` is passive.** The robot stops and waits. A
   fresh detection is the only exit. There is no policy to go
   somewhere else and try again.
5. **Phase 3B goal memory interacts with cancellation, not
   perception.** If the chair disappears during `PURSUING`,
   `search_manager` does **not** itself cancel Phase 3B — the
   controller will drive to the last-known goal and sit in `REACHED`.
   Only when the goal is explicitly cancelled (by the operator, or
   eventually by a future task coordinator) does control return to
   `SEARCHING`. This is deliberate: Phase 4 does not own cancellation
   policy.
6. **`/cmd_vel` contention is resolved by a parameter override.** The
   cleanliness relies on `controller.publish_zero_when_idle=false`
   during Phase 4. If anyone launches Phase 4 without that override,
   they will see oscillation between rotate and stop. The launch
   file hard-codes the override so this is a risk only if someone
   bypasses the launch.
7. **Heading tolerance on hand-off is implicit.** When `REACQUIRED`
   fires, the robot is mid-rotation. Phase 3B will absorb whatever
   yaw the robot has at that moment and the controller's usual
   `rotate_threshold_rad` takes over. No explicit "stop and settle"
   step.
8. **Chair-only.** `target_class` is a parameter, but the rest of the
   stack has only been validated for `chair`.
9. **LOST timer is wall-clock.** A slow detector or a stalled
   perception pipeline counts against the sweep budget.
10. **No explicit RViz sweep trajectory.** Markers show state and a
    static ring, not a trace of the rotation so far.

## 9. Next step (future enhancement, NOT part of Phase 4)

Deferred to later phases:

- **Full frontier exploration** — go visit unexplored spatial cells
  when the chair is not in the current room. Requires a map /
  costmap publisher and a frontier detector, neither of which the
  current sim brings up.
- **SLAM viewpoint selection** — "rotate toward the most informative
  yaw" instead of a uniform sweep.
- **Translation search** — step forward / sideways between sweeps so
  small occlusions don't force a `LOST` timeout.
- **Goal cancellation policy** — have `search_manager_node` (or a
  future task coordinator) automatically cancel a stale Phase 3B
  goal when the target has been invisible for `> N` seconds mid
  pursuit, so the system can return to `SEARCHING` without operator
  intervention.
- **Multi-class / multi-instance search** — reason about several
  target classes at once, or about picking between multiple visible
  instances of the same class.
- **Behaviour tree integration** — replace the ad-hoc state machine
  with `py_trees` / BehaviorTree.CPP once the number of states grows.
- **Safety integration** — vet `/cmd_vel` through `go2_safety` before
  publishing (e-stop listener, collision monitor, shared-autonomy
  veto).
- **Gait replacement** — Phase 0 still uses `CmdVelDriver`; a real
  quadruped locomotion policy (e.g. an Isaac Lab RL policy) is the
  expected long-horizon swap-in, and does not touch this layer.
