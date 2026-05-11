# Phase 5 — Locomotion Backend Upgrade (Scaffold)

> **Evidence used for this document**
>
> - Source of truth: the current code at
>   `sim/locomotion_backends.py` (new), `sim/run_go2_warehouse_ros2.py`
>   (CmdVelDriver refactored into a backend façade + three new CLI
>   args).
> - No ROS 2 package was modified. No message schemas changed. Phase
>   0 / 1 / 2 / 3A / 3B / 4 code is untouched.
> - **The policy backend is NOT claimed to be validated end-to-end.**
>   No Isaac Lab checkpoint is bundled with the repository, so every
>   runtime test captured here is on the kinematic backend. See §9 for
>   the external dependencies still required to close this gap.

## 1. Phase 5 overview

Phase 5 replaces only the **lowest motion-execution layer** in the
simulator. Up to Phase 4 the Go2's base was teleported via
`SingleArticulation.set_world_pose()` with articulation gravity
disabled — correct for bringing the ROS 2 topic pipeline online, but
not a real gait. Phase 5:

1. Introduces a `LocomotionBackend` abstraction with two concrete
   implementations: `KinematicLocomotionBackend` (the Phase 0 cheat,
   unchanged in behaviour) and `PolicyLocomotionBackend` (scaffold for
   an Isaac Lab Go2 flat-terrain velocity policy).
2. Refactors `sim/run_go2_warehouse_ros2.py`'s `CmdVelDriver` into a
   thin façade that reads the SubTwist OmniGraph node once per
   physics tick and forwards the body-frame command to the selected
   backend.
3. Adds three CLI switches (`--locomotion`, `--policy-checkpoint`,
   `--policy-decimation`).
4. Guarantees that the ROS 2 upper pipeline (Phases 1 – 4) is
   bit-for-bit identical — same topics, same schemas, same rates —
   regardless of which backend is active.

## 2. Phase 5 scope

In scope (strictly):

- Define `LocomotionBackend` protocol and the config dataclasses that
  parameterise it.
- Move the existing Phase 0 teleport logic into
  `KinematicLocomotionBackend`, 1-for-1.
- Scaffold `PolicyLocomotionBackend` (TorchScript load, observation
  packing, joint-order mapping, joint-target apply) with a clean
  fallback path.
- Ship a `make_backend()` factory that falls back to kinematic on
  any policy-load error and surfaces a warning.
- CLI selection, preserving defaults such that Phase 0 behaviour is
  still what you get without flags.

Out of scope (explicit):

- Training / fine-tuning any locomotion policy.
- Exporting a checkpoint from Isaac Lab (`play.py --export`).
- Domain-randomisation / sim2sim robustness experiments.
- Sim-to-real deployment, hardware DDS bridges, motor calibration.
- Any change to Phase 1 / 2 / 3A / 3B / 4 code or message schemas.
- Redesigning the Phase 3B `SimplePControllerBackend` — that is a
  **high-level local controller** (goal → Twist) and is orthogonal
  to the locomotion backend (Twist → joint motion).

## 3. Current motion-backend analysis

Pre-Phase-5 pipeline:

```
Phase 3B SimplePControllerBackend   (goal → Twist)
        │
        ▼
    /cmd_vel   geometry_msgs/Twist  (body frame, ~10 Hz)
        │ DDS
        ▼
Isaac Sim OmniGraph SubTwist   (linear + angular Vec3 output)
        │                               ← attribute read
        ▼
sim/run_go2_warehouse_ros2.py :: CmdVelDriver
  • SingleArticulation(prim).disable_gravity()
  • self._yaw += wz * dt
  • self._pos += R(yaw) · (vx, vy) · dt
  • SingleArticulation.set_world_pose(pos, quat_wxyz)
```

Implicit assumptions of the old `CmdVelDriver`:

- No leg actuation is ever required. Gravity must be disabled on the
  articulation or the legs collapse.
- Body-frame (vx, vy, wz) is a **position target rate** in world
  frame — i.e. the robot slides at exactly the commanded velocity,
  with zero inertia, zero foot slip, zero tip-over.
- `IsaacComputeOdometry` still reports the movement correctly
  because it reads the PhysX-owned articulation root pose, and
  `set_world_pose` writes through that same API.

What the upper pipeline assumes about this layer:

- Publishing a `Twist` on `/cmd_vel` changes `/odom` and `/tf` in the
  expected direction within a few hundred milliseconds.
- The robot eventually stops when `/cmd_vel` becomes zero (true for
  both backends — Phase 0 stops when the integrator input is zero,
  the policy backend stops via the default joint pose when
  `velocity_commands = 0`).

That is the entire contract. It is unchanged by Phase 5.

## 4. Replacement interface design

### 4.1 `LocomotionBackend` protocol

```python
class LocomotionBackend:
    name: str

    def on_sim_started(self) -> None: ...
    def step(self,
             dt: float,
             lin_cmd: Optional[Sequence[float]],   # Twist.linear  (vx,vy,vz)
             ang_cmd: Optional[Sequence[float]]    # Twist.angular (wx,wy,wz)
             ) -> None: ...
    def on_sim_stopping(self) -> None: ...
```

- **Frame of the command**: body. The OmniGraph SubTwist node
  preserves whatever the publisher sent; Phase 3B publishes in the
  robot's body frame, which is what any velocity-tracking gait
  policy (Isaac Lab or otherwise) expects.
- **Clamping**: `LocomotionBackendConfig.max_lin` /
  `max_ang`. Applied by the concrete backend; the façade does not
  pre-clamp.
- **Physics tick rate**: determined by `world.get_physics_dt()` in
  the outer loop. Backends that need to run at a different rate
  (e.g. a 50 Hz policy on 200 Hz sim) are expected to decimate
  internally.

### 4.2 Factory

```python
def make_backend(name: str,
                 cfg: LocomotionBackendConfig,
                 policy_cfg: Optional[PolicyConfig] = None,
                 ) -> tuple[LocomotionBackend, Optional[str]]: ...
```

Returns `(backend, warning_or_None)`. If `name == "policy"` and
construction raises for any reason (no torch, no checkpoint, joint
mismatch, …), the factory falls back to `KinematicLocomotionBackend`
and surfaces the reason as the second element so the outer loop can
log it. **This is deliberate**: the sim must always come up, even
with a bad policy config, because everything above Phase 0 already
assumes `/cmd_vel` moves the robot.

### 4.3 What is unchanged upstream

| Layer | Phase | Touched? |
|---|---|---|
| Isaac Sim scene & OmniGraph | 0 | no |
| ROS 2 bridge (/clock, /odom, /tf, /imu, cameras) | 0 | no |
| Perception (chair-only) | 1 | no |
| Semantic memory | 2 | no |
| Target selection + goal generation | 3A | no |
| Phase 3B: `nav_executor_node` + `SimplePControllerBackend` + `arrival_verifier_node` | 3B | no |
| Search / reacquisition | 4 | no |
| `/cmd_vel` message schema (`geometry_msgs/Twist`) | — | no |

Phase 5 is purely a refactor **inside** one file
(`sim/run_go2_warehouse_ros2.py`) plus one new file
(`sim/locomotion_backends.py`).

## 5. Isaac Lab locomotion policy integration plan

The `PolicyLocomotionBackend` is wired for an Isaac Lab-trained Go2
flat-terrain velocity policy in the textbook
`Isaac-Velocity-Flat-Unitree-Go2-v0` shape. Concretely:

**Observation layout (default, 48-dim)** — packed in this order by
`PolicyLocomotionBackend._build_observation`:

| Block | Shape | Source |
|---|---|---|
| base linear velocity (body frame) | 3 | `art.get_linear_velocity()` rotated into body frame |
| base angular velocity (body frame) | 3 | `art.get_angular_velocity()` rotated into body frame |
| projected gravity (body frame) | 3 | world `(0,0,-1)` rotated into body frame |
| velocity commands | 3 | `(vx, vy, wz)` from `/cmd_vel` |
| joint positions offset | 12 | `q − q_default`, reordered to policy joint order |
| joint velocities | 12 | `qdot`, reordered |
| last action | 12 | from previous policy step |

**Action layout**: 12-dim raw action, applied as
`q_target = q_default + action_scale · action`, then written through
`ArticulationAction(joint_positions=…)`. `action_scale` defaults to
`0.25` (Isaac Lab textbook value) and is a `PolicyConfig` field.

**Joint order mapping**: `PolicyConfig.joint_names_policy_order`
lists the 12 DoF names in the order the policy was trained with;
`PolicyLocomotionBackend.__init__` resolves that to the articulation
DoF indices (`art.dof_names`) at bind time, and raises if a policy
joint name is missing from the USD. This is the single most common
subtle breakage source when swapping checkpoints.

**Decimation**: `PolicyConfig.decimation` defaults to 4. With the
current ~60 Hz physics this gives ~15 Hz policy, not the textbook
50 Hz. Either use a higher `physics_dt` in Isaac Sim or accept the
lower rate when testing a stock checkpoint.

**What Phase 5 does NOT do** (these are the external steps a future
phase / the user has to complete):

1. Train (or obtain) an `Isaac-Velocity-Flat-Unitree-Go2` checkpoint.
2. Export it via Isaac Lab's `play.py --task ... --export_policy`
   flow to produce a TorchScript `.pt` file.
3. Drop that path into `--policy-checkpoint`.
4. If the observation / joint-order differs from the defaults, either
   extend `PolicyConfig` or subclass `PolicyLocomotionBackend`.

## 6. Implementation strategy (minimum safe path)

The changes in this phase are the smallest set that keeps the sim
running while opening a real policy seat.

1. **New file** `sim/locomotion_backends.py` — the protocol, the two
   backends, the factory. Nothing in the module imports torch at
   module scope; torch is imported lazily inside
   `PolicyLocomotionBackend.__init__`.
2. **Refactor** `sim/run_go2_warehouse_ros2.py :: CmdVelDriver` into
   a ~30-line façade that:
   - Re-uses the existing SubTwist OmniGraph attribute reads (no
     change to OmniGraph wiring).
   - Constructs a `LocomotionBackend` via `make_backend(...)`.
   - Delegates `step(dt)` to the backend.
3. **CLI**: `--locomotion {kinematic, policy}` (default `kinematic`),
   `--policy-checkpoint <path>`, `--policy-decimation <int>`.
4. **Fallback**: `make_backend()` returns a working kinematic
   backend even if the policy config is broken; a warning is logged
   to stdout (`[run_ros2] WARN PolicyLocomotionBackend unavailable
   (...); falling back to kinematic.`).

The kinematic path is bit-for-bit the same as before (disable
gravity, integrate, `set_world_pose`). Running without any flag
produces exactly the behaviour every earlier phase was validated
against.

## 7. Test plan

### 7.1 Default (kinematic) regression

```bash
bash scripts/run_warehouse_ros2.sh           # or equivalent launcher
# no extra flags — defaults to --locomotion kinematic
```

Expected:

- Log line: `CmdVelDriver → LocomotionBackend='kinematic' articulation='/World/Go2/base'`
- `[phase5/kinematic] bound to ... start_pos=[...] start_yaw=...rad`
- All Phase 1–4 topics keep their pre-Phase-5 rates.
- Publishing a Twist to `/cmd_vel` moves the robot exactly as in
  Phase 0 (same trajectory, same max speeds, `disable_gravity`
  still on).

### 7.2 Policy-backend sanity (no checkpoint)

```bash
./run_warehouse_ros2 --locomotion policy
# no --policy-checkpoint
```

Expected:

- Log line:
  `[run_ros2] WARN PolicyLocomotionBackend unavailable (PolicyLocomotionBackend requires --policy-checkpoint pointing at a TorchScript (.pt) Go2 locomotion model.); falling back to kinematic.`
- Subsequently identical to 7.1. Sim comes up; Phase 1–4 unchanged.

### 7.3 Policy-backend sanity (bad path)

```bash
./run_warehouse_ros2 --locomotion policy --policy-checkpoint /tmp/does-not-exist.pt
```

Expected: warning with `FileNotFoundError`, fallback to kinematic,
sim still runs.

### 7.4 Policy-backend real checkpoint (requires external asset)

```bash
./run_warehouse_ros2 --locomotion policy \
    --policy-checkpoint /path/to/go2_flat_ts.pt \
    --policy-decimation 4
```

Expected (if the external Isaac Lab export is compatible):

- `[phase5/policy] loaded checkpoint=/...pt joints=12 decimation=4`
- `[phase5/policy] bound to /World/Go2/base dof_count=12`
- Gravity is **not** disabled on the articulation.
- Legs articulate; robot maintains stance; sending `Twist`s on
  `/cmd_vel` produces walking rather than sliding.
- `/odom` and `/tf` continue to track the base.

This last scenario is **not** self-contained in the repository (see
§9) and is intentionally not part of the acceptance criteria below.

## 8. Acceptance criteria

Phase 5 is accepted when **all** of the following hold:

1. **No upper-pipeline regression.** With `--locomotion kinematic`
   (default) every Phase 0–4 topic rate and acceptance criterion is
   met unchanged. Bit-exact trajectories are not required (because
   trajectories depend on the Phase 3B controller), but the previous
   "topic shows up at expected Hz" list is identical.
2. **Backend abstraction exists.** `sim/locomotion_backends.py`
   defines `LocomotionBackend`, `KinematicLocomotionBackend`,
   `PolicyLocomotionBackend`, and a `make_backend()` factory. The
   two concrete backends share the protocol exactly.
3. **CLI selection works.** `--locomotion kinematic` and
   `--locomotion policy` both boot the simulator; the driver log
   line tells you which one is active.
4. **Graceful fallback.** Any failure to construct
   `PolicyLocomotionBackend` (torch missing, checkpoint missing,
   joint mismatch, …) falls back to kinematic, logs a human-readable
   warning, and keeps the sim running.
5. **Message / launch compatibility.** No ROS 2 launch file or
   message schema is altered. `scripts/run_warehouse_ros2.sh`,
   `chair_perception.launch.py`, `chair_semantic_memory.launch.py`,
   `chair_goto_goal.launch.py`, `chair_execute_goal.launch.py`,
   `chair_with_search.launch.py` all continue to work without
   modification.
6. **Contract documented.** The policy backend's observation layout,
   action layout, joint-order resolution, and decimation are
   described in code and in this document (§5) so that a future
   contributor can swap a specific checkpoint in without re-reverse-
   engineering the interface.

Explicit NON-criteria:

- A real walking gait in the warehouse is **not** required for Phase
  5 to be accepted. That depends on an external Isaac Lab
  checkpoint and is tracked under §9.
- `/odom` must still reflect motion when the policy backend walks —
  this is inherited from `IsaacComputeOdometry` + `apply_action`
  writing through PhysX; it is deferred to first-checkpoint testing.

## 9. What still depends on Isaac Lab (future work, NOT Phase 5)

All of the following are deliberately **outside** Phase 5:

1. **Obtaining a checkpoint.** Train (or download) an
   `Isaac-Velocity-Flat-Unitree-Go2` (or equivalent) policy. Isaac
   Lab's built-in PPO recipe with RSL-RL is the standard path.
2. **Exporting it as TorchScript.** `play.py --task ... --export_policy`
   (flag name varies between Isaac Lab releases) produces the
   `.pt` file that `PolicyLocomotionBackend` expects.
3. **Aligning observation / action layouts.** The defaults in
   `PolicyConfig` match the stock Isaac Lab Go2 flat-terrain env;
   a custom policy may need a subclass or a config override.
4. **Matching control rate.** Isaac Lab typically uses 200 Hz
   physics / 50 Hz policy (`decimation=4`). This repo's sim runs at
   ~60 Hz. Either bump the physics rate or accept the rate mismatch
   when first testing a stock checkpoint.
5. **Robustness / domain randomisation.** The stock flat-terrain
   policy may lose balance on the warehouse floor material, on
   spawn-point perturbations, on rotating-in-place commands that
   were outside training distribution, etc. All of these are
   expected to surface during §7.4 and are deferred to a
   dedicated "policy validation" phase.
6. **Sim-to-real.** Hardware DDS, motor calibration, safety
   interlocks, and any deployment work are not part of this MVP
   chain.

## 10. Known limitations of Phase 5

1. **Policy backend is a scaffold, not a validated gait.** Every
   runtime test in this phase was on the kinematic backend.
2. **Observation code is parameterised but the defaults are the
   only documented ones.** Non-standard checkpoints likely need a
   subclass. The code makes that easy but does not ship one.
3. **No control-rate mismatch handling.** If the Isaac Sim physics
   rate and the checkpoint's training rate differ, `--policy-
   decimation` is the only knob; there is no interpolation or
   smoothing layer.
4. **Policy backend assumes `SingleArticulation.get_linear_velocity`
   / `get_angular_velocity` / `get_joint_positions` /
   `get_joint_velocities` / `apply_action(ArticulationAction)` are
   available on the running Isaac Sim version.** These exist on
   5.1; they may move again in future Kit releases.
5. **No safety layer.** When walking fails the robot falls; there
   is no e-stop watchdog that would re-enable `disable_gravity`
   and freeze the base. Safety integration is tracked against a
   later phase.
6. **The kinematic fallback is dynamically identical to Phase 0.**
   If something in Phase 1–4 was subtly tuned against the
   kinematic slide (e.g. "the chair goes out of FOV when the robot
   yaws at exactly this rate"), that tuning will stop matching
   reality the moment the policy backend is active — those are
   expected to re-tune rather than be considered regressions.
7. **`CmdVelDriver` name was preserved for blame / log-grep
   continuity.** New code should refer to `LocomotionDriver`-style
   concepts via the backend protocol, not by that class name.
