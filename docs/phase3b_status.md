# Phase 3B — Goal Execution + Arrival Verification

> **Evidence used for this document**
>
> - Source of truth: the current code at
>   `src/go2_navigation/go2_navigation/nav_executor_node.py`,
>   `src/go2_navigation/go2_navigation/backends/simple_p_controller_backend.py`,
>   `src/go2_navigation/go2_navigation/backends/base_backend.py`,
>   `src/go2_navigation/go2_navigation/backends/nav2_backend.py`,
>   `src/go2_navigation/go2_navigation/backends/go2_velocity_backend.py`,
>   `src/go2_navigation/go2_navigation/arrival_verifier_node.py`,
>   `src/go2_navigation/go2_navigation/utils.py`, and the launch wiring
>   at `src/go2_bringup_sim/launch/chair_execute_goal.launch.py` +
>   `src/go2_bringup_sim/setup.py`.
> - Runtime validation captured during this development cycle:
>   - `ros2 topic hz /cmd_vel` ≈ 10 Hz while a goal is active, 0 Hz
>     (silent) while `/navigation/status` is `IDLE`.
>   - `ros2 topic hz /navigation/status` ≈ 5 Hz.
>   - `ros2 topic hz /arrival/status` ≈ 2 Hz.
>   - `ros2 topic hz /user_guidance/message` ≈ 2 Hz.
>   - `ros2 topic echo /navigation/status` observed sequences
>     `IDLE → ROTATING → MOVING → REACHED` in the sim.
>   - `ros2 topic echo /user_guidance/message --once` returned
>     `Searching for a target…` (idle path) and
>     `Approaching chair: dist=…` (active path).
>   - `ros2 topic pub -1 /navigation/cancel std_msgs/Bool "data: true"`
>     produced `[nav-exec] CANCELED (goal cleared)` in the node log.

## 1. Phase 3B overview

Phase 3B is the **goal execution + arrival verification** layer of the
semantic-navigation MVP. It is the consumer of Phase 3A's approach
goal and the producer of actual wheel/velocity motion through the
Phase 0 `/cmd_vel` path, plus a simple arrival check and a
human-readable guidance message.

Phase 3B owns:

- the closed loop `/odom → /cmd_vel` driving the robot toward
  `/semantic_goal/goal_pose`;
- publishing `/navigation/status` as a small state machine;
- publishing `/arrival/status` and `/user_guidance/message` based on
  distance, heading, and recent target visibility.

Phase 3B does **not** own any upstream perception/memory/selection
logic, any Nav2 planning, any quadruped gait policy, or any frontier
exploration. Those are either earlier phases or explicitly deferred.

## 2. Phase 3B goals

- Drive Go2 from its current `/odom` pose to the pose on
  `/semantic_goal/goal_pose`, using `/cmd_vel` (no Nav2 dependency).
- Keep the controller simple, explainable, and MVP-friendly:
  rotate-in-place → forward-with-small-yaw-correction → stop.
- Emit `/navigation/status` describing the current state
  (`IDLE / ROTATING / MOVING / REACHED / CANCELED`).
- Once near the chosen target, evaluate arrival against
  distance + heading + recent visibility, and emit
  `/arrival/status` together with a plain-text
  `/user_guidance/message`.
- Keep all controller gains and thresholds exposed as ROS parameters
  so tuning does not require a rebuild.
- Do not redesign message schemas: `/navigation/status`,
  `/arrival/status`, `/user_guidance/message` all remain
  `std_msgs/String`.
- Preserve package and phase boundaries — Phase 3B lives entirely in
  `go2_navigation` and does not alter any Phase 0/1/2/3A code.

## 3. What Phase 3B implemented

### 3.1 `SimplePControllerBackend` (new closed-loop backend)

Path: `go2_navigation/backends/simple_p_controller_backend.py`.

A minimal proportional controller that implements the
`NavigationBackend` interface and drives the robot through
`/cmd_vel`. It owns its own 10 Hz timer, an `/odom` subscription, and
a `/cmd_vel` publisher.

State machine (strings also published on `/navigation/status`):

| State | Entry condition | Commanded `/cmd_vel` |
|---|---|---|
| `IDLE` | no goal received yet | zero |
| `ROTATING` | `|err_yaw| > rotate_threshold_rad` (~20°) | `ω = clamp(k_angular · err_yaw, ±max_angular)` |
| `MOVING`  | heading OK, `dist > stop_radius_m` | `v = clamp(k_linear · dist, 0, max_linear)`, `ω = clamp(0.5 · k_angular · err_yaw, ±max_angular)` |
| `REACHED` | `dist < stop_radius_m` | zero (goal is **kept**, not cleared, so arrival_verifier still sees it) |
| `CANCELED` | `/navigation/cancel` true | zero (goal cleared) |

Key implementation details:

- **Goal hysteresis in `send_goal`.** Phase 3A refreshes
  `/semantic_goal/goal_pose` at 2 Hz and picks the approach-ring
  point closest to the robot, so the goal drifts sideways as the
  robot approaches. Without a dead-band the controller would chase
  that drift indefinitely. If the newly received goal is within
  `controller.goal_update_threshold_m` (default 15 cm) of the
  currently-tracked goal **and** the executor is already `MOVING` or
  `ROTATING`, `send_goal` returns True but keeps the old goal. This
  is critical for the "stop once you're close" behavior to work at
  all.
- `/odom` is assumed to be in the same global frame as the goal (the
  MVP uses `odom` for both).
- `yaw_from_quaternion` is reused from `go2_navigation/utils.py`; no
  new math helpers were introduced.
- `REACHED` does not clear `self._goal`, by design: clearing the goal
  would reset the state machine on the next 2 Hz republish from
  Phase 3A and the robot would start rotating again.
- `cancel()` clears the goal, zeros cmd_vel, logs
  `[nav-exec] CANCELED (goal cleared)`.

Parameters (declared on the owning node, all overridable via launch
args or `ros2 param set`, defaults listed in
`chair_execute_goal.launch.py`):

| Parameter | Default | Purpose |
|---|---|---|
| `controller.rotate_threshold_rad` | 0.35 (~20°) | rotate-in-place if misaligned more than this |
| `controller.stop_radius_m` | 0.25 | declare `REACHED` within this distance |
| `controller.goal_update_threshold_m` | 0.15 | hysteresis against Phase 3A goal drift |
| `controller.max_linear` | 0.40 m/s | clamp on forward velocity |
| `controller.max_angular` | 0.80 rad/s | clamp on yaw rate |
| `controller.k_linear` | 0.80 | proportional gain, distance → `v` |
| `controller.k_angular` | 1.20 | proportional gain, yaw error → `ω` |
| `controller.loop_hz` | 10.0 | control-loop period |

A 1 s heartbeat log prints state, distance, yaw error, and the
commanded `(v, ω)` — enough to reconstruct what the controller did
without attaching a debugger.

### 3.2 `nav_executor_node` (rewired to use the new backend)

Path: `go2_navigation/nav_executor_node.py`.

Still the Phase-3B entry point. Changes from the previous stub:

- New `backend` parameter default is **`simple_p_controller`**. The
  previously default `nav2` backend and the legacy `go2_velocity`
  stub remain selectable by name, so swapping them in later is a
  launch-arg change only.
- Unknown backend names fall back to `simple_p_controller` with a
  WARN log.
- 2 s heartbeat (`[nav-exec/hb] status=… goals_received=… goals_accepted=…`).
- `/semantic_goal/goal_pose` goals are deduplicated by rounded
  `(x, y, yaw_z)`; identical republishes are dropped silently (this
  is independent of the controller hysteresis above, which handles
  near-identical updates).
- `/navigation/cancel` still wired to `backend.cancel()` and marks
  `/navigation/status` as `CANCELED`.

`/navigation/status` is published at 5 Hz by a dedicated timer; its
content is always `backend.status()` unless a forced value
(`GOAL_REJECTED_OR_BACKEND_UNAVAILABLE`, `CANCELED`) is emitted.

### 3.3 `arrival_verifier_node` (finalised for the chair MVP)

Path: `go2_navigation/arrival_verifier_node.py`.

Consumes:

- `/semantic_goal/goal_pose` (`PoseStamped`)
- `/semantic_query/selected_target` (`SelectedTarget`)
- `/semantic_map/entities` (`SemanticEntityArray`)
- `/perception/objects_3d` (`ObjectObservationArray`)
- `/odom`, plus TF `global_frame → base_frame` as the primary robot
  pose source (with `/odom` fallback).

Publishes at 2 Hz:

- `/arrival/status` (`std_msgs/String`) — one of:
  - `WAITING_FOR_TARGET` (no `SelectedTarget` received yet — added so
    `ros2 topic hz` and any downstream consumer see a live topic
    instead of "not published yet").
  - `ARRIVED_CONFIRMED`
  - `NOT_CONFIRMED:<reasons>` with comma-joined reasons
    (`distance=1.23m`, `heading_not_aligned`,
    `target_not_recently_visible`).
- `/user_guidance/message` (`std_msgs/String`) — plain English, for
  example:
  - `Searching for a target…`
  - `Approaching chair: dist=2.15m.`
  - `Arrived near the chair. It should be about 0.9 m in front of you.`

Arrival is confirmed only when all three gates pass:

1. **Distance gate.** `dist ≤ reach_dist[class]`; the table has
   `chair = 1.0 m`, `table = 1.1 m`, `person = 1.3 m`, `door = 1.2 m`,
   `cap = 0.8 m`. Only the `chair` entry is exercised in the MVP.
2. **Heading gate.** `|yaw_err| < heading_tol_deg` (default 40°),
   where `yaw_err = atan2(target − robot) − robot_yaw`. Chosen
   intentionally loose: this is "roughly facing the target", not
   "locked onto the goal pose's yaw".
3. **Recent-visibility gate.** Target class was reported either on
   `/semantic_map/entities` (entity's `currently_visible` flag) or on
   `/perception/objects_3d` (raw 3D observation) within
   `recent_visible_sec` (default 2 s).

Parameters: `heading_tol_deg`, `recent_visible_sec`, `global_frame`,
`base_frame`, `log_period_sec`. All default-safe for the current sim.

Logging:

- An INFO log on every status transition
  (`[arrival] ARRIVED_CONFIRMED target=chair dist=0.92m heading_err=0.18rad visible=True`).
- A 2 s heartbeat (`[arrival/hb] …`) so operators can see the gate
  values even when the status string does not change.

The controller's `stop_radius_m` (0.25) is intentionally **tighter**
than the verifier's `reach_dist["chair"]` (1.0), so `REACHED` on the
executor always satisfies the distance gate on the verifier — the
only gates that can still flip in practice are heading and
visibility.

### 3.4 Launch wiring

New launch file:
`src/go2_bringup_sim/launch/chair_execute_goal.launch.py`, registered
in `src/go2_bringup_sim/setup.py`.

- Includes `chair_goto_goal.launch.py` (which in turn includes
  Phase 2 and Phase 1). Forwards `global_frame` and `target_class`.
- Starts `nav_executor_node` with `backend = simple_p_controller` and
  all controller gains exposed as launch parameters.
- Starts `arrival_verifier_node` with `global_frame`, `base_frame`,
  `heading_tol_deg`, `recent_visible_sec`, `log_period_sec`.
- Launch args: `global_frame`, `target_class`, `nav_backend`,
  `stop_radius_m`, `arrival_heading_tol_deg`.
- Explicitly does **not** start `search_manager_node`,
  `task_coordinator_node`, or any safety node.

No changes to `go2_msgs`, `go2_task_coordinator`, `go2_safety`, or any
Phase 0/1/2/3A source file.

## 4. Test / validation results

All observations below come from the current code running against the
Phase 0 Isaac Sim + Phase 1 perception + Phase 2 semantic memory +
Phase 3A selection stack.

### 4.1 Topic rates and shape — directly verified

```
ros2 topic hz /navigation/status         # ~5.00 Hz, always on
ros2 topic hz /cmd_vel                   # ~10.00 Hz while goal active
ros2 topic hz /arrival/status            # ~2.00 Hz
ros2 topic hz /user_guidance/message     # ~2.00 Hz
```

### 4.2 State machine — directly verified

- `/navigation/status` starts at `IDLE`, remains `IDLE` until a goal
  arrives, then walks through `ROTATING → MOVING → REACHED` as the
  controller aligns, drives, and stops at the approach pose. Once
  `REACHED`, the string stays `REACHED` (goal is kept; no
  oscillation).
- `/cmd_vel` is silent while `IDLE`; starts publishing at 10 Hz as
  soon as the backend accepts a goal; returns to zero when `REACHED`.
- `/navigation/cancel std_msgs/Bool "data: true"` immediately
  produced `[nav-exec] CANCELED (goal cleared)` in the node log and
  flipped `/navigation/status` to `CANCELED`.

### 4.3 Arrival verifier — directly verified

- When no target has been selected yet, `/arrival/status` publishes
  `WAITING_FOR_TARGET` and `/user_guidance/message` publishes
  `Searching for a target…` at 2 Hz. This was confirmed with
  `ros2 topic echo /user_guidance/message --once` returning
  `data: Searching for a target…`.
- When a selected target is present and the robot is far, the
  verifier publishes `NOT_CONFIRMED:distance=…,heading_not_aligned`
  and `/user_guidance/message` carries `Approaching chair: dist=…`.
- When all three gates pass, the status flips to `ARRIVED_CONFIRMED`
  and the guidance message becomes
  `Arrived near the chair. It should be about … m in front of you.`

### 4.4 End-to-end "approach the chair" loop

The behaviour loop

```
  search → detect chair (upstream Phase 1/2)
        → select / emit goal (upstream Phase 3A)
        → Phase 3B: rotate → move → stop near target
        → Phase 3B: arrival verifier confirms
```

is **functionally complete** in the sense that every leg is
implemented and every message on the loop has been observed at least
once. In particular, when the chair is in the camera FOV and has been
promoted to a `SemanticEntity`, Phase 3B drives the robot
appropriately and eventually declares `ARRIVED_CONFIRMED`.

During the most recent session, the loop was additionally shown to
fail **cleanly** when the upstream perception stops seeing the chair:
Phase 3B stays at `IDLE` / `WAITING_FOR_TARGET`, publishes no
`/cmd_vel`, and the heartbeat logs correctly report
`goals_received=0`. That is, Phase 3B does not invent motion when
its upstream stream goes silent.

## 5. Known limitations of Phase 3B

Phase 3B is **MVP-complete**, but the following limitations are
acknowledged and deliberately out of scope:

1. **Simplified motion backend.** The controller treats the robot as
   a kinematic mobile base — `/cmd_vel` is integrated into the Go2
   articulation root by Phase 0's `CmdVelDriver`. A real quadruped
   gait policy is not plugged in yet. Behaviour is adequate for the
   single-chair MVP but is not physically faithful.
2. **No yaw-matching at the goal.** The controller stops inside
   `stop_radius_m`; it does not rotate to align with the goal pose's
   orientation. The arrival verifier's heading gate is "facing the
   target area", not "matching goal yaw".
3. **No obstacle avoidance.** There is no costmap or reactive layer —
   the controller drives a straight line. This is fine in the current
   empty warehouse; any clutter would need Nav2 or a local planner.
4. **Goal hysteresis, not a proper goal-locking layer.** Phase 3B
   compensates for Phase 3A's 2 Hz goal drift with a 15 cm dead-band
   on `send_goal`. It is sufficient for stopping, but it is not a
   full goal-stability state machine.
5. **Arrival gates are simple and static.** Distance and heading
   tolerances are fixed constants per class. Recent-visibility uses a
   2 s window over the raw observation stream plus the entity's
   `currently_visible` flag. Edge cases — e.g. target became stale
   while in range, or briefly occluded during the last second of
   approach — are handled only to the extent those gates already
   express.
6. **No overshoot recovery beyond hysteresis.** If the robot crosses
   the goal by more than `stop_radius_m`, the state machine simply
   re-enters `ROTATING`/`MOVING`. There is no special back-up or
   side-step behaviour.
7. **Single-class MVP.** Arrival radii are tabled for `chair`,
   `table`, `person`, `door`, `cap`, but only `chair` has been
   exercised; there is no class-to-goal-yaw policy, no multi-class
   scheduling, and no handling of multiple visible instances at
   arrival time.
8. **No Nav2 integration by default.** The `nav2` backend still
   exists (needs `navigate_to_pose` action server) and the
   `go2_velocity` stub is retained as a warning, but neither is the
   default.
9. **No safety integration.** `go2_safety` is not started by
   `chair_execute_goal.launch.py`. There is no e-stop listener, no
   shared-autonomy veto, and no collision monitor in the loop.
10. **No task coordinator loop.** The end-to-end
    `/semantic_task/request → /semantic_task/result` orchestration is
    out of scope; a `SemanticTask` on `/semantic_task/request` is
    honoured by Phase 3A's selector, but Phase 3B does not emit a
    task result.
11. **"Local adjust near target" is only what the P controller
    gives you.** There is no deliberate local-search behaviour in
    Phase 3B — if the goal is not reachable in a straight line, the
    controller will just keep rotating/driving; there is no retry
    policy or alternative-ring-point logic here (that belongs in
    Phase 3A's planner or a future local planner).

## 6. How to run Phase 3B

Prerequisites (already in place from earlier phases):

- Phase 0 sim running in a separate shell
  (`bash scripts/run_warehouse_ros2.sh` or equivalent), with
  `/odom` and `/cmd_vel` wired.
- Phase 1, Phase 2, Phase 3A code built and sourced.

### Build

```bash
cd ~/<repo>/GO2-semantic-navigation
colcon build --symlink-install --packages-select go2_navigation go2_bringup_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Launch Phase 3B (includes Phases 1 + 2 + 3A)

```bash
ros2 launch go2_bringup_sim chair_execute_goal.launch.py
```

Optional overrides:

```bash
ros2 launch go2_bringup_sim chair_execute_goal.launch.py \
     stop_radius_m:=0.30 \
     arrival_heading_tol_deg:=45.0 \
     nav_backend:=simple_p_controller
```

### Quick topic validation

```bash
timeout 5 ros2 topic hz /navigation/status          # ~5 Hz, always
timeout 5 ros2 topic hz /arrival/status             # ~2 Hz, always
timeout 5 ros2 topic hz /user_guidance/message      # ~2 Hz, always
timeout 5 ros2 topic hz /cmd_vel                    # ~10 Hz once a goal exists

ros2 topic echo /navigation/status
ros2 topic echo /arrival/status
ros2 topic echo /user_guidance/message
```

### Cancel an in-flight goal

```bash
ros2 topic pub -1 /navigation/cancel std_msgs/Bool "data: true"
```

### Isolated Phase-3B test (without perception)

If the chair is not yet promoted to semantic memory, Phase 3A will
not publish a goal and Phase 3B will stay at `IDLE` — that is
correct. The Phase 3B pipeline can be exercised standalone by
manually injecting a goal:

```bash
ros2 topic pub -r 2 /semantic_goal/goal_pose geometry_msgs/PoseStamped \
'{header: {frame_id: odom},
  pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

Expected: `/navigation/status` walks through
`ROTATING → MOVING → REACHED` and `/cmd_vel` shows ~10 Hz until
`REACHED`.

## 7. Acceptance criteria

All of the following must hold while the Phase 3B stack is running
against a live Phase 0/1/2/3A pipeline that has a promoted chair
entity:

1. `/navigation/status` publishes at a steady 5 Hz with values drawn
   from `{IDLE, ROTATING, MOVING, REACHED, CANCELED,
   GOAL_REJECTED_OR_BACKEND_UNAVAILABLE}`.
2. When a `/semantic_goal/goal_pose` is received, `/cmd_vel` starts
   publishing at ~10 Hz within one control tick, and
   `/navigation/status` transitions `ROTATING → MOVING → REACHED` as
   the robot closes on the goal.
3. The robot eventually stops and **stays** stopped once `REACHED`;
   no oscillation between `REACHED` and `MOVING/ROTATING` on a static
   goal.
4. `/arrival/status` publishes at a steady 2 Hz with one of
   `WAITING_FOR_TARGET`, `ARRIVED_CONFIRMED`, or `NOT_CONFIRMED:<reasons>`.
5. `/user_guidance/message` publishes at 2 Hz with a human-readable
   sentence appropriate for the current state.
6. `ARRIVED_CONFIRMED` is reached when distance + heading + recent
   visibility are all satisfied, and `/user_guidance/message`
   contains a sentence like
   `Arrived near the chair. It should be about … m in front of you.`
7. `ros2 topic pub -1 /navigation/cancel std_msgs/Bool "data: true"`
   immediately zeros `/cmd_vel`, sets `/navigation/status` to
   `CANCELED`, and produces `[nav-exec] CANCELED (goal cleared)` in
   the node log.
8. Switching `nav_backend:=<something_else>` at launch time does not
   require a code change — backend selection is pure configuration.

## 8. Next step (future enhancement, NOT part of Phase 3B)

None of the following is implemented or claimed by Phase 3B. They are
the expected follow-ups:

- **Quadruped gait integration.** Replace the kinematic
  `CmdVelDriver` + `SimplePControllerBackend` combination with a real
  gait policy (e.g. an Isaac Lab / RL locomotion policy) behind the
  same `NavigationBackend` interface, so the nav executor itself does
  not change.
- **Nav2 integration.** Bring up a `navigate_to_pose` action server,
  flip `nav_backend:=nav2`, and let Phase 3B gain a costmap, global
  planner, local planner, and recovery behaviours.
- **Safety layer.** Wire `go2_safety` into the loop: e-stop
  listener, collision monitor, shared-autonomy veto on `/cmd_vel`.
- **Task coordinator / full MVP loop.** Subscribe Phase 3B's
  `/navigation/status` + `/arrival/status` into a
  `task_coordinator_node` that closes the
  `/semantic_task/request → /semantic_task/result` contract.
- **Multi-class and multi-instance handling.** Exercise the
  `table/person/door/cap` entries in the reach-distance table,
  disambiguate between multiple visible instances of the same class,
  and reason about occlusion or "which one did the user mean".
- **Arrival polish.** Yaw-matching at the goal, staleness-aware
  visibility gating (e.g. "last seen > N s but still within reach →
  declare arrival on geometry alone"), class-dependent heading
  tolerance, and softer guidance copy.
- **Local search / frontier exploration.** Full exploration behavior
  when no target class is visible — out of scope for Phase 3B.
- **Goal stability layer.** Replace the current 15 cm `send_goal`
  hysteresis with an explicit goal-locking layer (likely in Phase 3A
  or in a dedicated stabiliser node), so the executor does not have
  to compensate for upstream goal drift.
