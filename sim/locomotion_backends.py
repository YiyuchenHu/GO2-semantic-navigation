"""Phase 5 — pluggable locomotion backends for the Isaac Sim Go2.

MVP NOTE (Day 8+):
    The stable demo path is the **kinematic** backend (default; no
    `--policy` flag). It is the *MVP backend* — the entire semantic
    navigation pipeline (SLAM → YOLO → semantic memory → NL parser →
    Nav2 goal) is validated against it. Use it for every "does the
    pipeline work end-to-end" demo, including the day8_two_phase
    natural-language → goal smoke test.

    The **policy** backend (PolicyLocomotionBackend, opted in via
    `--policy <ckpt>.pt`) is *experimental* and not on the MVP path.
    It loads a learned RL controller and exposes the real PhysX
    contact dynamics, but is sensitive to Nav2 cmd_vel rate /
    angular-z oscillation; goals that the kinematic backend reaches
    in seconds may topple the policy backend. Treat policy-backend
    work as a stretch goal — debug it independently with the standalone
    cmd_vel scripts (scripts/diagnose_cmd_vel_rates.sh) and don't make
    pipeline-level features depend on it.

Abstraction:
    LocomotionBackend.step(dt, lin_cmd, ang_cmd) is called every
    physics tick with the most recent body-frame velocity command
    (read from the /cmd_vel → SubTwist OmniGraph node by
    LocomotionDriver). A backend decides how to turn that command
    into robot motion that PhysX (and therefore
    IsaacComputeOdometry → /odom, /tf) can see.

Two backends ship with Phase 5:

  * KinematicLocomotionBackend — STABLE / MVP. The Phase 0 cheat,
    preserved as the default. It disables articulation gravity and
    teleports the base via SingleArticulation.set_world_pose() at
    each step, integrating the commanded body-frame velocity in the
    world frame. No joint motion, no gait. It is the only backend
    used by the kinematic demo path.

  * PolicyLocomotionBackend — EXPERIMENTAL. Real Isaac Lab Go2
    flat-terrain velocity policy. It expects a torchscript checkpoint
    (`.pt`) + a small YAML describing the observation layout and the
    articulation joint order. At __init__ time it will either succeed
    (torch + checkpoint + config all present) or raise, letting the
    caller fall back to the kinematic backend. The step() method
    itself runs policy inference and applies joint position targets,
    but is currently a stretch goal — see docs/phase5_status.md for
    the gap.

Factory:
    make_backend(name, **kwargs) constructs the requested backend and,
    on any PolicyLocomotionBackend construction failure, returns a
    KinematicLocomotionBackend together with a warning string, so the
    sim keeps running.
"""

from __future__ import annotations

import math
import os
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np


# -----------------------------------------------------------------------------
# Backend protocol
# -----------------------------------------------------------------------------
@dataclass
class LocomotionBackendConfig:
    """Arguments common to every backend."""

    articulation_path: str
    # Optional clamps on the command coming from /cmd_vel.
    max_lin: float = 1.0
    max_ang: float = 1.5

    # World-frame AABB the integrator clamps every set_world_pose() call
    # to. Without this clamp the kinematic backend will happily drive
    # Go2 right through the warehouse walls (it has no physics
    # collisions). When that happens slam_toolbox tracks the base_link
    # off into infinity and Nav2's costmap keeps resizing (we saw it
    # grow from 290x290 to 308x616 cells in one run on May 8).
    #
    # Defaults match the warehouse envelope (10x10m centred on origin)
    # minus a 0.35 m robot-radius margin so the body never overlaps a
    # wall cell. Pass +/-inf to disable.
    world_xy_min: float = -4.65
    world_xy_max: float = 4.65


class LocomotionBackend:
    """Interface every concrete backend must implement.

    Lifecycle:
        b = SomeBackend(cfg)
        # physics is running, world has been primed
        b.on_sim_started()
        while sim_running:
            b.step(dt, lin_cmd, ang_cmd)
        b.on_sim_stopping()
    """

    name: str = "base"

    def on_sim_started(self) -> None:
        """Called once right before the main simulation loop."""

    def step(
        self,
        dt: float,
        lin_cmd: Optional[Sequence[float]],
        ang_cmd: Optional[Sequence[float]],
    ) -> None:
        raise NotImplementedError

    def on_sim_stopping(self) -> None:
        """Called once when the main simulation loop is about to exit."""


# -----------------------------------------------------------------------------
# Kinematic backend (Phase 0 behaviour, preserved)
# -----------------------------------------------------------------------------
class KinematicLocomotionBackend(LocomotionBackend):
    """Drop-in replacement for the original CmdVelDriver integration
    path. Integrates the body-frame /cmd_vel into world pose and calls
    SingleArticulation.set_world_pose().
    """

    name = "kinematic"

    def __init__(self, cfg: LocomotionBackendConfig) -> None:
        self._cfg = cfg
        # Import Isaac Sim classes lazily so this module is importable
        # outside Isaac Sim (e.g. for lint/tooling).
        try:
            from isaacsim.core.prims import SingleArticulation  # type: ignore
        except ImportError:
            from isaacsim.core.api.articulations import (  # type: ignore
                Articulation as SingleArticulation,
            )
        self._SingleArticulation = SingleArticulation

        self._art: Any = SingleArticulation(
            prim_path=cfg.articulation_path, name="go2_kinematic_loco"
        )
        self._art.initialize()

        # Identical rationale to Phase 0: without a walking policy the
        # legs collapse under gravity. Disable articulation gravity
        # since we drive the base pose directly.
        #
        # IMPORTANT: in Isaac Sim 5.1 the articulation-level
        # disable_gravity() doesn't always propagate to the root
        # rigid body — leg-link gravity gets disabled but the base
        # link continues to accelerate downward. The kinematic
        # set_world_pose clamps position each tick, but PhysX still
        # accumulates BASE LINK VELOCITY between ticks. After ~100s
        # of running, the accumulated velocity overflows to NaN, and
        # the next IsaacComputeOdometry read produces (nan nan nan)
        # TF translations that take down SLAM.
        #
        # Defence: zero out the articulation's root velocity every
        # step (see step() below) and attempt to disable gravity at
        # both the articulation and individual rigid-body levels.
        try:
            self._art.disable_gravity()
        except Exception as exc:
            print(f"[phase5/kinematic] WARN disable_gravity() failed: {exc}")
            traceback.print_exc()
        # Belt + suspenders: walk every link and zero its rigid-body
        # gravity flag via PhysX directly. Some 5.1 builds need this.
        try:
            from pxr import UsdPhysics
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            arti_prim = stage.GetPrimAtPath(cfg.articulation_path)
            count = 0
            for prim in [arti_prim] + list(arti_prim.GetAllChildren()):
                rb_api = UsdPhysics.RigidBodyAPI.Get(stage, prim.GetPath())
                if rb_api:
                    # Disable gravity by setting the disableGravity attribute
                    attr = prim.GetAttribute("physxRigidBody:disableGravity")
                    if attr:
                        attr.Set(True)
                        count += 1
            if count > 0:
                print(f"[phase5/kinematic] disabled gravity on {count} rigid bodies")
        except Exception as exc:
            print(f"[phase5/kinematic] WARN per-link gravity disable failed: {exc}")

        pos, orn = self._art.get_world_pose()   # WXYZ
        # Defensive: if PhysX has accumulated NaN in the chassis pose
        # (long-running session, articulation joint solver edge cases),
        # initialising self._pos to that NaN would permanently poison
        # every subsequent integration step. Fall back to the
        # configured spawn pose used by warehouse_scene.py instead.
        pos_arr = np.array(pos, dtype=float)
        if not np.all(np.isfinite(pos_arr)):
            print(f"[phase5/kinematic] WARN get_world_pose() returned non-finite "
                  f"pos={pos_arr.tolist()}; falling back to spawn (-4, -4, 0.55)")
            pos_arr = np.array([-4.0, -4.0, 0.55], dtype=float)
            try:
                self._art.set_world_pose(
                    position=pos_arr,
                    orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
                )
            except Exception as exc:
                print(f"[phase5/kinematic] WARN reset set_world_pose failed: {exc}")
        self._pos = pos_arr
        w, x, y, z = float(orn[0]), float(orn[1]), float(orn[2]), float(orn[3])
        # Same defence on rotation.
        if not all(math.isfinite(v) for v in (w, x, y, z)):
            print(f"[phase5/kinematic] WARN get_world_pose() returned non-finite "
                  f"orn=({w}, {x}, {y}, {z}); resetting to identity")
            w, x, y, z = 1.0, 0.0, 0.0, 0.0
        self._yaw = math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        )
        print(
            f"[phase5/kinematic] bound to {cfg.articulation_path}  "
            f"start_pos={self._pos.tolist()}  start_yaw={self._yaw:.3f}rad"
        )

    def step(self, dt, lin_cmd, ang_cmd) -> None:
        if lin_cmd is None or ang_cmd is None:
            return

        # Defensive: SubTwist OmniGraph node occasionally returns
        # non-finite values during publisher transitions or after
        # long-running PhysX sessions. Skipping the whole step is
        # safer than feeding NaN into self._yaw / self._pos and
        # poisoning every downstream odom + tf publisher.
        if not (math.isfinite(dt)
                and math.isfinite(float(lin_cmd[0]))
                and math.isfinite(float(lin_cmd[1]))
                and math.isfinite(float(ang_cmd[2]))):
            print(f"[phase5/kinematic] WARN dropping step with non-finite "
                  f"input: dt={dt} lin=({float(lin_cmd[0])}, {float(lin_cmd[1])}) "
                  f"ang_z={float(ang_cmd[2])}")
            return

        vx = float(np.clip(lin_cmd[0], -self._cfg.max_lin, self._cfg.max_lin))
        vy = float(np.clip(lin_cmd[1], -self._cfg.max_lin, self._cfg.max_lin))
        wz = float(np.clip(ang_cmd[2], -self._cfg.max_ang, self._cfg.max_ang))

        new_yaw = self._yaw + wz * dt
        c, s = math.cos(new_yaw), math.sin(new_yaw)
        wx = c * vx - s * vy   # body-frame -> world
        wy = s * vx + c * vy
        new_x = self._pos[0] + wx * dt
        new_y = self._pos[1] + wy * dt

        # Final guard before write — if anything went sideways above,
        # don't push NaN into PhysX. Without this, a single bad step
        # corrupts the articulation pose forever (until restart) and
        # tf2 starts emitting TF_NAN_INPUT errors that take down SLAM.
        if not all(math.isfinite(v) for v in (new_yaw, new_x, new_y)):
            print(f"[phase5/kinematic] WARN integration produced non-finite "
                  f"state: new_yaw={new_yaw} new_x={new_x} new_y={new_y}; "
                  f"keeping previous pose")
            return

        # Soft "wall" — kinematic backend has no physics collisions, so
        # an unconstrained /cmd_vel (e.g. residual after Nav2 ABORTs
        # follow_path while velocity_smoother is still de-throttling)
        # would teleport Go2 right through the wall and into infinity.
        # Observed on 2026-05-08: base_link drifted from world (4.5,
        # 7.5) to (-2.4, 23.7) in ~30 s, costmap kept resizing to
        # follow it (290x290 -> 308x616), Nav2 planner repeatedly
        # failed and mapping_explorer eventually FAILED. Clamp to
        # cfg.world_xy_min..world_xy_max keeps Go2 inside the
        # warehouse and gives velocity_smoother time to drain.
        clamped_x = float(
            min(max(new_x, self._cfg.world_xy_min), self._cfg.world_xy_max)
        )
        clamped_y = float(
            min(max(new_y, self._cfg.world_xy_min), self._cfg.world_xy_max)
        )
        if clamped_x != new_x or clamped_y != new_y:
            # Throttled log — flooding terminals when Go2 sits against a
            # wall under residual cmd_vel doesn't help anyone.
            now_s = time.monotonic()
            if not hasattr(self, "_last_clamp_log_s"):
                self._last_clamp_log_s = 0.0
            if now_s - self._last_clamp_log_s > 5.0:
                print(
                    f"[phase5/kinematic] world-AABB clamp engaged "
                    f"(world_xy bounds=[{self._cfg.world_xy_min:.2f},"
                    f"{self._cfg.world_xy_max:.2f}]m): "
                    f"requested ({new_x:.2f},{new_y:.2f}) -> "
                    f"clamped ({clamped_x:.2f},{clamped_y:.2f}). "
                    f"This is normal when Go2 is pushed against a wall "
                    f"by residual /cmd_vel; suppress for 5 s."
                )
                self._last_clamp_log_s = now_s
        new_x, new_y = clamped_x, clamped_y

        self._yaw = new_yaw
        self._pos[0] = new_x
        self._pos[1] = new_y

        h = 0.5 * self._yaw
        quat_wxyz = np.array(
            [math.cos(h), 0.0, 0.0, math.sin(h)], dtype=float
        )
        self._art.set_world_pose(
            position=self._pos.astype(float), orientation=quat_wxyz
        )

        # CRITICAL: pin the articulation root velocity to zero every
        # tick. Without this, gravity accumulates velocity between
        # set_world_pose calls (because disable_gravity doesn't always
        # cover the base link in 5.1). After ~100s of running, the
        # velocity overflows to NaN, the next IsaacComputeOdometry
        # read returns (nan nan nan), and every TF consumer crashes.
        # We try both the bulk set_velocities API and the per-axis
        # APIs because Isaac Sim 5.1 build flavours differ on which
        # one exists.
        zero3 = np.zeros(3, dtype=float)
        try:
            self._art.set_linear_velocity(zero3)
            self._art.set_angular_velocity(zero3)
        except Exception:
            try:
                self._art.set_velocities(np.zeros(6, dtype=float))
            except Exception:
                pass
        # Also zero the joint velocities so unconstrained leg joints
        # don't swing freely under residual gravity.
        try:
            n_dof = int(getattr(self._art, "num_dof", 0))
            if n_dof > 0:
                self._art.set_joint_velocities(np.zeros(n_dof, dtype=float))
        except Exception:
            pass


# -----------------------------------------------------------------------------
# Policy backend (Isaac Lab Go2 flat-terrain — SCAFFOLD)
# -----------------------------------------------------------------------------
@dataclass
class PolicyConfig:
    """Describes how to pack observations and apply actions for a
    specific Isaac Lab Go2 locomotion checkpoint.

    The defaults are aligned with the **stock** Isaac Lab
    `Isaac-Velocity-Flat-Unitree-Go2-v0` configuration (verified on
    IsaacLab v2.3.0):

      * ActionsCfg: the BASE velocity_env_cfg sets
        `JointPositionActionCfg(scale=0.5)`, but
        `UnitreeGo2RoughEnvCfg.__post_init__` overrides this to
        **0.25** (see IsaacLab/.../config/go2/rough_env_cfg.py L30:
        `self.actions.joint_pos.scale = 0.25`). UnitreeGo2FlatEnvCfg
        inherits from RoughEnvCfg and does NOT re-override, so flat
        Go2 training also uses 0.25. We MUST match here or every
        joint target is ~2× larger than the policy expects, which
        manifests as a policy that walks for a few seconds but
        accumulates over-shoot drift and falls.
        → action_scale = 0.25, action order = articulation `dof_names`.
      * UnitreeGo2FlatEnvCfg drops `height_scan` so the policy obs
        is 48-dim: lin(3)+ang(3)+grav(3)+vel_cmd(3)+qpos_rel(12)+
        qvel(12)+last_action(12).
      * UNITREE_GO2_CFG.init_state.joint_pos sets each joint via
        regex (.*L_hip=0.1, .*R_hip=-0.1, F[LR]_thigh=0.8,
        R[LR]_thigh=1.0, .*_calf=-1.5). We replicate those rules at
        runtime instead of hard-coding a 12-vector, because the
        articulation's USD DoF order is not guaranteed to match any
        particular hand-written list (PhysX 5.1 sometimes reorders
        joints alphabetically per leg).

    If you trained with non-stock settings, either:
      * pass an explicit `joint_names_policy_order` + `default_joint_pos`
        with len == n_dof, or
      * subclass `PolicyLocomotionBackend._build_observation` /
        `_apply_last_action`.
    """

    checkpoint_path: str = ""
    # Ordered DoF names as they appear in the trained policy.
    # Empty → auto-fill from `articulation.dof_names` at __init__.
    # Empty is the right default for a stock Isaac Lab Go2 checkpoint
    # because Isaac Lab's `JointPositionActionCfg(joint_names=[".*"])`
    # iterates joints in articulation order.
    joint_names_policy_order: List[str] = field(default_factory=list)
    # Default joint angles at which the policy was trained; action
    # output is `default + scale * raw_action`.
    # Empty → derive from UNITREE_GO2_CFG init_state regex at __init__,
    # using the resolved joint_names_policy_order. Length must equal
    # len(joint_names_policy_order) once resolved.
    default_joint_pos: List[float] = field(default_factory=list)
    # IsaacLab Go2 flat task uses `action_scale=0.25` (set by
    # `UnitreeGo2RoughEnvCfg.__post_init__` and inherited by the flat
    # variant). Earlier this was incorrectly set to 0.5 here, which
    # silently doubled every commanded joint offset and caused the
    # policy to walk for a few seconds before accumulating overshoot
    # drift and falling. Keep this in sync with whatever the training
    # config sets — if you re-train with a different scale, override
    # via `PolicyConfig(action_scale=...)`.
    action_scale: float = 0.25
    # Policy runs at `physics_hz / decimation` in Isaac Lab; we keep
    # the option to slow it down inside the backend.
    decimation: int = 4


class PolicyLocomotionBackend(LocomotionBackend):
    """Go2 flat-terrain velocity locomotion policy backend.

    STATUS: scaffold. All the wiring is here — torchscript load,
    observation packing, policy call, joint-target application —
    but it has NOT been validated against a real checkpoint inside
    this repo, because no Isaac Lab checkpoint ships with the
    project. Use `--locomotion policy --policy-checkpoint <path>` to
    try it; on any import/load failure the driver falls back to
    KinematicLocomotionBackend.
    """

    name = "policy"

    def __init__(
        self,
        cfg: LocomotionBackendConfig,
        policy_cfg: PolicyConfig,
    ) -> None:
        self._cfg = cfg
        self._pcfg = policy_cfg

        if not policy_cfg.checkpoint_path:
            raise RuntimeError(
                "PolicyLocomotionBackend requires --policy-checkpoint "
                "pointing at a TorchScript (.pt) Go2 locomotion model."
            )
        if not os.path.isfile(policy_cfg.checkpoint_path):
            raise FileNotFoundError(
                f"Policy checkpoint not found: {policy_cfg.checkpoint_path}"
            )

        try:
            import torch  # noqa: F401  (runtime dependency for this backend only)
        except ImportError as exc:
            raise RuntimeError(
                "PolicyLocomotionBackend requires `torch` to be "
                "importable in the Isaac Sim Python environment."
            ) from exc
        self._torch = __import__("torch")

        # Load as TorchScript; this avoids pulling in the full
        # rsl_rl / skrl training machinery just to run inference.
        self._model = self._torch.jit.load(
            policy_cfg.checkpoint_path, map_location="cpu"
        )
        self._model.eval()
        # NOTE: joint_names_policy_order may still be empty here when the
        # caller relies on the auto-resolve path further down; the joint
        # count printed in the "bound to ..." line below is the truthful
        # one. We don't try to print it here.
        print(
            f"[phase5/policy] loaded checkpoint={policy_cfg.checkpoint_path} "
            f"decimation={policy_cfg.decimation}"
        )

        # Bind the articulation. We keep gravity ENABLED — a real
        # policy must hold the robot up against gravity by itself.
        try:
            from isaacsim.core.prims import SingleArticulation  # type: ignore
        except ImportError:
            from isaacsim.core.api.articulations import (  # type: ignore
                Articulation as SingleArticulation,
            )
        self._art: Any = SingleArticulation(
            prim_path=cfg.articulation_path, name="go2_policy_loco"
        )
        self._art.initialize()

        dof_names = list(self._art.dof_names)  # type: ignore[attr-defined]

        # Resolve `joint_names_policy_order`. Empty → use articulation
        # order verbatim (correct for stock Isaac Lab Go2 because
        # JointPositionActionCfg(joint_names=[".*"]) iterates joints
        # in articulation order). Non-empty → use as-is and let the
        # reorder map below align it to dof_names.
        policy_order = list(policy_cfg.joint_names_policy_order)
        if not policy_order:
            policy_order = list(dof_names)
            print(
                f"[phase5/policy] auto-resolved joint_names_policy_order "
                f"from articulation DoFs: {policy_order}"
            )

        # Build mapping policy → articulation. With empty config this
        # is the identity, but we still build it so an explicit
        # mismatched config (e.g. checkpoint trained on a fork that
        # reorders joints) keeps working.
        self._policy_to_art_idx = []
        for jname in policy_order:
            if jname not in dof_names:
                raise RuntimeError(
                    f"[phase5/policy] joint '{jname}' required by policy "
                    f"checkpoint not found in articulation DoFs: {dof_names}"
                )
            self._policy_to_art_idx.append(dof_names.index(jname))
        self._policy_to_art_idx = np.asarray(
            self._policy_to_art_idx, dtype=np.int64
        )

        # Resolve `default_joint_pos`. Empty → apply Isaac Lab's
        # UNITREE_GO2_CFG init_state regex to `policy_order`, so we
        # always feed the policy the same default it was trained
        # against. Explicit length must match.
        if not policy_cfg.default_joint_pos:
            default_qpos = _go2_default_joint_pos_from_regex(policy_order)
            print(
                f"[phase5/policy] auto-resolved default_joint_pos via "
                f"UNITREE_GO2_CFG regex: {default_qpos.tolist()}"
            )
        else:
            if len(policy_cfg.default_joint_pos) != len(policy_order):
                raise RuntimeError(
                    f"[phase5/policy] default_joint_pos length "
                    f"{len(policy_cfg.default_joint_pos)} != "
                    f"joint_names_policy_order length {len(policy_order)}"
                )
            default_qpos = np.asarray(
                policy_cfg.default_joint_pos, dtype=np.float32
            )

        self._policy_order = policy_order
        self._default_qpos_policy = default_qpos.astype(np.float32)
        # default_qpos in articulation order, used by reset/apply paths.
        default_qpos_art = np.zeros_like(default_qpos)
        default_qpos_art[self._policy_to_art_idx] = default_qpos
        self._default_qpos_art = default_qpos_art

        # We track TWO copies of the last action because IsaacLab's
        # `mdp.last_action` observation returns `env.action_manager.action`
        # which is the *raw policy output* — not the clipped, ramped, or
        # scale*offset-processed version that actually goes to the joint
        # PD targets. Conflating them is a real bug:
        #   - the ramped/clipped vector goes to PD targets (correct)
        #   - the raw vector must be the one fed back into obs[36:48]
        #     so the policy sees the same "what I asked for last tick"
        #     distribution it was trained on.
        # During warmup / cmd-vel deadband we set BOTH to zero (= "I am
        # holding still"), which matches IsaacLab's reset-to-zero of
        # `_action`/`_prev_action` in ActionManager.reset.
        self._last_raw_action = np.zeros(len(policy_order), dtype=np.float32)
        self._last_applied_action = np.zeros(len(policy_order), dtype=np.float32)
        self._decim_counter = 0

        # ----------------------------------------------------------------
        # Align PhysX joint drive gains with Isaac Lab's UNITREE_GO2_CFG.
        # ----------------------------------------------------------------
        # Training uses DCMotorCfg(stiffness=25.0, damping=0.5) on every
        # walking joint. The stock Go2 USD ships with PhysX joint drives
        # whose gains are NOT guaranteed to match those values, so the
        # PD controller responds to apply_action(joint_positions=...) at
        # the wrong stiffness. Symptom: policy commands a target pose but
        # joints barely move, robot collapses. We force the training
        # gains via TWO independent paths because either one alone has
        # been observed to silently fail on Isaac Sim 5.1:
        #
        #   1. ArticulationController.set_gains() — the runtime path,
        #      directly into PhysX implicit-PD state. Fast, but only
        #      works if the joint already has a PhysxJointDriveAPI
        #      applied at USD layer. Some versions of go2.usd ship
        #      without the drive API on every joint, in which case
        #      set_gains becomes a no-op.
        #   2. Direct USD edit — apply PhysxJointDriveAPI on every
        #      relevant joint prim and write stiffness/damping. This
        #      survives any PhysX state reload and guarantees the
        #      drive exists before set_gains is called.
        #
        # We do (2) first (so the drive APIs exist), then (1) (so the
        # already-running PhysX scene picks up the new values without
        # waiting for the next play/stop cycle).
        n_dof = len(dof_names)
        try:
            self._force_usd_joint_drives_for_locomotion(
                self._cfg.articulation_path, stiffness=25.0, damping=0.5
            )
        except Exception as exc:
            print(f"[phase5/policy] WARN USD joint drive override failed: {exc}")
            traceback.print_exc()
        try:
            ctrl = self._art.get_articulation_controller()
            kps = np.full(n_dof, 25.0, dtype=np.float32)
            kds = np.full(n_dof, 0.5, dtype=np.float32)
            try:
                cur_kps, cur_kds = ctrl.get_gains()
                print(
                    f"[phase5/policy] PRE-set gains: "
                    f"kps[0:3]={np.asarray(cur_kps).reshape(-1)[:3].tolist()}  "
                    f"kds[0:3]={np.asarray(cur_kds).reshape(-1)[:3].tolist()}"
                )
            except Exception:
                pass
            ctrl.set_gains(kps=kps, kds=kds)
            try:
                new_kps, new_kds = ctrl.get_gains()
                print(
                    f"[phase5/policy] POST-set gains: "
                    f"kps[0:3]={np.asarray(new_kps).reshape(-1)[:3].tolist()}  "
                    f"kds[0:3]={np.asarray(new_kds).reshape(-1)[:3].tolist()}  "
                    f"(target was 25.0 / 0.5)"
                )
            except Exception:
                print(
                    f"[phase5/policy] joint PD gains set to stiffness=25.0, "
                    f"damping=0.5 on {n_dof} DoFs (no get_gains readback)"
                )
        except Exception as exc:
            print(f"[phase5/policy] WARN set_gains failed: {exc}")
            traceback.print_exc()

        # ----------------------------------------------------------------
        # Reset to a training-distribution-aligned initial state.
        # ----------------------------------------------------------------
        # Training UNITREE_GO2_CFG.init_state has pos=(0,0,0.40) and the
        # standing joint pose default_qpos. We preserve the warehouse XY
        # spawn (so SLAM and waypoints stay valid) but force base z=0.40
        # and joint positions to default_qpos. Without this Go2 enters
        # the policy's first inference with an out-of-distribution obs
        # (free-falling base, splayed legs from USD default), the policy
        # cannot recover and the robot tips over within seconds.
        try:
            cur_pos, cur_orn = self._art.get_world_pose()
            cur_pos_arr = np.array(cur_pos, dtype=float)
            cur_pos_arr[2] = 0.40
            # Cache the training-aligned standing pose so we can re-assert
            # it every warmup tick (see step()). Stiffness=25 is the
            # training value but cannot fully cancel gravity on a 15kg
            # quadruped, so without re-asserting the robot sags from
            # 0.40m to ~0.32m by the end of the 1s warmup. That sagged
            # state is OOD for the policy and causes a "loading-phase"
            # over-extension on the first cmd_vel.
            self._init_base_xy = (float(cur_pos_arr[0]), float(cur_pos_arr[1]))
            self._init_base_z = 0.40
            self._init_base_orn = np.asarray(cur_orn, dtype=float).copy()
            self._art.set_world_pose(
                position=cur_pos_arr,
                orientation=np.asarray(cur_orn, dtype=float),
            )
            self._art.set_joint_positions(
                self._default_qpos_art.astype(np.float32)
            )
            try:
                self._art.set_joint_velocities(np.zeros(n_dof, dtype=np.float32))
            except Exception:
                pass
            try:
                self._art.set_linear_velocity(np.zeros(3, dtype=np.float32))
                self._art.set_angular_velocity(np.zeros(3, dtype=np.float32))
            except Exception:
                pass
            # Seed the position target so the first physics tick after
            # init doesn't yank joints toward whatever USD default the
            # PhysX drive was holding.
            try:
                from isaacsim.core.utils.types import ArticulationAction  # type: ignore
                self._art.apply_action(
                    ArticulationAction(
                        joint_positions=self._default_qpos_art.astype(
                            np.float32
                        )
                    )
                )
            except Exception:
                pass
            # Read back the actual articulation state so we can prove
            # set_world_pose / set_joint_positions actually committed
            # to PhysX (a silent failure here would leave the policy
            # to deal with a flat-on-floor robot at z≈0.05m, exactly
            # the symptom the user is seeing).
            verify_pos, _ = self._art.get_world_pose()
            verify_qpos = np.asarray(
                self._art.get_joint_positions(), dtype=np.float32
            )
            print(
                f"[phase5/policy] reset Go2 to standing pose: "
                f"base_xyz=({cur_pos_arr[0]:.2f},{cur_pos_arr[1]:.2f},"
                f"{cur_pos_arr[2]:.2f}) qpos=default"
            )
            print(
                f"[phase5/policy] post-reset verify: base_z="
                f"{float(verify_pos[2]):.3f}m  qpos[0:3]="
                f"{verify_qpos[:3].tolist()}  qpos[8:11]="
                f"{verify_qpos[8:11].tolist()}"
            )
        except Exception as exc:
            print(f"[phase5/policy] WARN initial-pose reset failed: {exc}")
            traceback.print_exc()

        # Stand-up grace period: how many physics ticks to spend
        # kinematically locking the robot at UNITREE_GO2_CFG.init_state
        # before letting the policy network drive joint targets. With
        # physics_dt=1/200s, 20 ticks = 0.1 s. Each tick we:
        #   - re-assert base pose to (init_xy, z=0.40, init_orn)
        #   - re-assert joint positions to default_qpos
        #   - zero base & joint velocities
        # so the warmup ends with the robot in exactly the state IsaacLab
        # uses for episode reset. The policy's first inference therefore
        # sees a *training-distribution* observation and does not need a
        # "loading phase" to push base back up to 0.40m. Twenty ticks is
        # enough to let RTX/Vulkan shader compile finish on the very
        # first main-loop frame (which always pays a one-time 1-5 min
        # compile cost regardless of warmup length).
        self._warmup_ticks_remaining = 20
        self._warmup_logged_done = False
        # After warmup, blend the policy's action in over `_ramp_ticks_total`
        # physics ticks. During this window the policy sees real obs and runs
        # inference at full rate, but the action *applied* to the joints is
        # alpha*policy_action + (1-alpha)*0, with alpha ramped linearly from
        # 0→1. Reason: training samples include observation noise (Unoise
        # typically ±0.1 lin / ±0.2 ang / ±1.5 joint_vel) and reset states
        # come with random base/joint perturbations, so a freshly handed-off
        # robot at *exactly* zero velocity is OOD enough that the policy can
        # emit a single large action that flips the body before the next
        # tick stabilizes it. The ramp gives the system one second to drift
        # into a perturbed-and-moving regime that is in distribution.
        self._ramp_ticks_total = 200    # 1.0s at 200Hz
        self._ramp_ticks_remaining = self._ramp_ticks_total
        # Per-joint action clipping. Go2 Velocity training uses
        # action_scale=0.25 and a Gaussian policy with init std≈1.0; in
        # practice >99% of actions land in [-2, 2]. We clip to ±2 because
        # at scale=0.25 a ±2 action only means a ±0.5 rad target offset
        # from default, which is well within actuator range. (Our earlier
        # ±1 clip was tuned for an incorrect action_scale=0.5.)
        self._action_abs_clip = 2.0
        self._first_policy_print_done = False
        # Deadband on vel_cmd. Stock IsaacLab Velocity training has
        # rel_standing_envs=0.02 — only 2% of training samples have
        # vel_cmd=[0, 0, 0]. The other 98% have non-zero commands. So
        # serving the policy a literal zero command is essentially OOD,
        # and the policy tends to emit small trot-like actions that
        # accumulate into a fall. When the user is not actively driving
        # the robot, hold the standing PD pose instead (functionally
        # identical to extending the warmup forever). Threshold values
        # are looser than typical IMU drift so a real cmd of any
        # noticeable magnitude immediately re-enables the policy.
        self._cmd_lin_deadband = 0.05   # m/s
        self._cmd_ang_deadband = 0.05   # rad/s
        # Periodic health log: every N policy ticks (50Hz → N=50 means
        # once a second), print base_z, tilt off-vertical, qvel/raw_action
        # magnitudes. Costs nothing and pinpoints the moment of failure.
        self._health_log_period = 50    # ticks
        self._policy_tick_counter = 0
        self._ramp_mid_logged = False
        self._ramp_end_logged = False

        print(
            f"[phase5/policy] bound to {cfg.articulation_path}  "
            f"dof_count={len(dof_names)}  action_scale={policy_cfg.action_scale}  "
            f"warmup_ticks={self._warmup_ticks_remaining}  "
            f"ramp_ticks={self._ramp_ticks_total}  "
            f"action_clip=±{self._action_abs_clip}"
        )

    def _kinematic_lock_to_init_state(self) -> None:
        """Hard-reset the articulation to UNITREE_GO2_CFG.init_state.

        Used by both the warmup grace period and the cmd_vel deadband
        branch. Without this, stiffness=25 (the IsaacLab training value)
        cannot fight gravity holding a 15kg quadruped at z=0.40 with
        the default qpos, so within ~0.5s base drifts to ~0.32m and
        joints sag a few centimeters. When the user finally publishes
        a cmd_vel, the policy's first active obs is OOD and it emits
        a raw action ~3.0 ("loading-phase / pop up to standing height")
        which is far outside the |a|<1 envelope it saw in training,
        creating a positive feedback loop with last_action obs that
        ends in the robot collapsing into the inverted-cone-and-fall
        failure mode. Hard-locking eliminates the drift completely:
        every tick the robot is teleported back to the exact pose
        IsaacLab uses for episode reset.
        """
        try:
            lock_pos = np.array(
                [
                    self._init_base_xy[0],
                    self._init_base_xy[1],
                    self._init_base_z,
                ],
                dtype=float,
            )
            self._art.set_world_pose(
                position=lock_pos,
                orientation=self._init_base_orn,
            )
            self._art.set_joint_positions(
                self._default_qpos_art.astype(np.float32)
            )
            n_dof = int(self._art.num_dof) if hasattr(self._art, "num_dof") else len(self._default_qpos_art)
            self._art.set_joint_velocities(
                np.zeros(n_dof, dtype=np.float32)
            )
            try:
                self._art.set_linear_velocity(np.zeros(3, dtype=np.float32))
                self._art.set_angular_velocity(np.zeros(3, dtype=np.float32))
            except Exception:
                pass
        except Exception:
            pass

    def step(self, dt, lin_cmd, ang_cmd) -> None:
        # Stand-up grace period: hold default_qpos via PD AND kinematic
        # lock so warmup ends with the robot in exactly init_state.
        if self._warmup_ticks_remaining > 0:
            self._warmup_ticks_remaining -= 1
            # Both raw (for obs) and applied (for PD) are zero during
            # warmup, matching IsaacLab's ActionManager.reset() which
            # zeroes `_action` and `_prev_action`.
            self._last_raw_action[:] = 0.0
            self._last_applied_action[:] = 0.0
            self._apply_last_action()
            self._kinematic_lock_to_init_state()
            if self._warmup_ticks_remaining == 0 and not self._warmup_logged_done:
                self._warmup_logged_done = True
                try:
                    pos, _ = self._art.get_world_pose()
                    qpos = np.asarray(
                        self._art.get_joint_positions(), dtype=np.float32
                    )
                    print(
                        f"[phase5/policy] warmup done: base_z="
                        f"{float(pos[2]):.3f}m  qpos_err_max="
                        f"{float(np.max(np.abs(qpos - self._default_qpos_art))):.3f}rad "
                        f"(handing off to policy network now)"
                    )
                except Exception:
                    print("[phase5/policy] warmup done (handing off to policy)")
            return

        # Decimation: run policy every `decimation` physics ticks,
        # hold last action in between. This mirrors Isaac Lab
        # training where physics runs at 200 Hz and the policy at
        # 50 Hz.
        self._decim_counter = (self._decim_counter + 1) % max(
            1, self._pcfg.decimation
        )
        if self._decim_counter != 0:
            self._apply_last_action()
            return

        if lin_cmd is None or ang_cmd is None:
            lin_cmd = [0.0, 0.0, 0.0]
            ang_cmd = [0.0, 0.0, 0.0]

        vx = float(np.clip(lin_cmd[0], -self._cfg.max_lin, self._cfg.max_lin))
        vy = float(np.clip(lin_cmd[1], -self._cfg.max_lin, self._cfg.max_lin))
        wz = float(np.clip(ang_cmd[2], -self._cfg.max_ang, self._cfg.max_ang))

        # vel_cmd deadband: skip policy inference when no meaningful
        # command is present. This protects against the OOD-on-zero-cmd
        # failure mode discussed in __init__. We also kinematic-lock
        # the base back to init_state every tick so that the moment
        # the user publishes a real cmd_vel, the policy's first obs
        # is *exactly* the IsaacLab episode-reset state. Without this
        # lock, the 4+ seconds the user typically takes to type
        # `ros2 topic pub /cmd_vel ...` are enough for stiffness=25
        # to lose the gravity battle and let base sag from 0.40m to
        # ~0.32m, which puts the policy back in the OOD regime that
        # causes the inverted-cone collapse. Confirmed in user logs:
        # warmup ended at base_z=0.400 but FIRST log showed 0.318
        # because the deadband branch held PD-only between them.
        cmd_norm = abs(vx) + abs(vy)
        ang_norm = abs(wz)
        if (
            cmd_norm < self._cmd_lin_deadband
            and ang_norm < self._cmd_ang_deadband
        ):
            self._last_raw_action[:] = 0.0
            self._last_applied_action[:] = 0.0
            self._apply_last_action()
            self._kinematic_lock_to_init_state()
            return

        obs = self._build_observation(vx, vy, wz)

        with self._torch.no_grad():
            action_tensor = self._model(
                self._torch.from_numpy(obs).unsqueeze(0)
            ).squeeze(0)
        raw_action = action_tensor.detach().cpu().numpy().astype(np.float32)

        # 1) Clip raw action to a sane envelope. Training never bounds it,
        #    but the policy's Gaussian head can emit large outliers on OOD
        #    obs that translate to physical joint targets way past the
        #    actuator's effective range and flip the robot.
        clipped = np.clip(
            raw_action, -self._action_abs_clip, self._action_abs_clip
        )

        # 2) Soft-start ramp: alpha grows 0→1 across _ramp_ticks_total
        #    policy ticks following warmup. We're already inside a
        #    decimation==4 branch, so this runs ~50 times per second,
        #    which is exactly the cadence we want for the ramp.
        if self._ramp_ticks_remaining > 0:
            alpha = 1.0 - (
                self._ramp_ticks_remaining / float(self._ramp_ticks_total)
            )
            self._ramp_ticks_remaining -= 1
        else:
            alpha = 1.0
        action = alpha * clipped

        # 3) Diagnostics. We log on three triggers:
        #      - very first policy step (alpha≈0 baseline)
        #      - mid-ramp     (alpha≈0.5)
        #      - end of ramp  (alpha=1, full policy authority)
        #    plus a periodic health beat at `_health_log_period` ticks.
        self._policy_tick_counter += 1

        def _log(tag: str) -> None:
            try:
                pos, _ = self._art.get_world_pose()
                base_z = float(pos[2])
            except Exception:
                base_z = float("nan")
            grav_b = obs[6:9]
            tilt_off_z = float(np.sqrt(grav_b[0] ** 2 + grav_b[1] ** 2))
            print(
                f"[phase5/policy] {tag}  "
                f"alpha={alpha:.3f}  base_z={base_z:.3f}m  "
                f"tilt_off_z={tilt_off_z:.3f}  "
                f"raw|max|={float(np.max(np.abs(raw_action))):.3f}  "
                f"clip|max|={float(np.max(np.abs(clipped))):.3f}  "
                f"applied|max|={float(np.max(np.abs(action))):.3f}  "
                f"qvel|max|={float(np.max(np.abs(obs[24:36]))):.3f}  "
                f"vel_cmd=[{vx:.2f},{vy:.2f},{wz:.2f}]"
            )

        if not self._first_policy_print_done:
            self._first_policy_print_done = True
            _log("FIRST")
            try:
                print(
                    f"[phase5/policy] FIRST obs split: "
                    f"lin_b={obs[0:3].tolist()}  "
                    f"ang_b={obs[3:6].tolist()}  "
                    f"grav_b={obs[6:9].tolist()}  "
                    f"vel_cmd={obs[9:12].tolist()}  "
                    f"qpos_rel|max|={float(np.max(np.abs(obs[12:24]))):.3f}  "
                    f"qvel|max|={float(np.max(np.abs(obs[24:36]))):.3f}  "
                    f"last_act|max|={float(np.max(np.abs(obs[36:48]))):.3f}"
                )
            except Exception:
                pass

        if (
            not self._ramp_mid_logged
            and self._ramp_ticks_remaining
            <= self._ramp_ticks_total // 2
        ):
            self._ramp_mid_logged = True
            _log("MID-RAMP")

        if (
            not self._ramp_end_logged
            and self._ramp_ticks_remaining == 0
        ):
            self._ramp_end_logged = True
            _log("END-RAMP")

        if self._policy_tick_counter % self._health_log_period == 0:
            _log("HEALTH")

        # Update both copies of "last action":
        #  * _last_raw_action  → fed back into obs[36:48] next tick.
        #    This MUST be the unclipped, un-ramped policy output to
        #    match IsaacLab's `mdp.last_action` (which returns
        #    `env.action_manager.action`, the policy's raw output).
        #    Earlier versions of this file fed `alpha * clipped` here,
        #    which during the ramp made obs[last_action] much smaller
        #    than what the policy had actually requested last tick →
        #    the policy compensated by emitting a larger raw action
        #    each tick (1.86 → 2.08 → 2.19 → 2.54 → 3.29 in user logs)
        #    until the body collapsed. Confirmed root cause for the
        #    "limbs retract into an inverted cone" failure on cmd_vel.
        #  * _last_applied_action → used by `_apply_last_action()` to
        #    compute the PD target (= default + scale * applied).
        #    During warmup/ramp this is intentionally smaller than raw
        #    so PD targets transition smoothly out of stand pose.
        self._last_raw_action = raw_action
        self._last_applied_action = action
        self._apply_last_action()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _build_observation(self, vx: float, vy: float, wz: float) -> np.ndarray:
        """Pack a textbook Isaac-Velocity-Flat-Unitree-Go2 observation.

        The exact layout must match the trained checkpoint. This
        default mirrors the stock Isaac Lab configuration; a custom
        checkpoint will require overriding this method or extending
        PolicyConfig.
        """
        art = self._art

        # Base body-frame linear / angular velocity.
        base_lin_w = np.asarray(art.get_linear_velocity(), dtype=np.float32)
        base_ang_w = np.asarray(art.get_angular_velocity(), dtype=np.float32)
        _, orn_wxyz = art.get_world_pose()
        R = _rot_from_wxyz(orn_wxyz)
        base_lin_b = R.T @ base_lin_w
        base_ang_b = R.T @ base_ang_w

        # Projected gravity in body frame.
        gravity_w = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        gravity_b = R.T @ gravity_w

        # Joint pos / vel, reordered to policy order, offset from
        # default pose.
        qpos_art = np.asarray(
            art.get_joint_positions(), dtype=np.float32
        )[self._policy_to_art_idx]
        qvel_art = np.asarray(
            art.get_joint_velocities(), dtype=np.float32
        )[self._policy_to_art_idx]
        qpos_rel = qpos_art - self._default_qpos_policy

        # NOTE: obs[36:48] must be the *raw* (unclipped, un-ramped)
        # policy output from the previous tick — see comment in step()
        # at `self._last_raw_action = raw_action`. Feeding the ramped
        # vector here causes a runaway feedback loop during the soft-
        # start window.
        obs = np.concatenate(
            [
                base_lin_b.astype(np.float32),
                base_ang_b.astype(np.float32),
                gravity_b.astype(np.float32),
                np.array([vx, vy, wz], dtype=np.float32),
                qpos_rel.astype(np.float32),
                qvel_art.astype(np.float32),
                self._last_raw_action.astype(np.float32),
            ]
        )
        return obs

    def _apply_last_action(self) -> None:
        """Apply `_last_applied_action` as joint position targets.

        `_last_applied_action` may be ramped/clipped/zeroed (during
        warmup, deadband, or the soft-start window); that's fine here
        because the PD target is what the joints actually track. The
        raw policy output goes to `_last_raw_action` and is fed back
        into the observation in the next tick — see comments in
        __init__ and step().
        """
        if self._last_applied_action is None:
            return
        target_qpos_policy = (
            self._default_qpos_policy
            + self._pcfg.action_scale * self._last_applied_action
        )
        target_qpos_art = np.zeros_like(target_qpos_policy)
        target_qpos_art[self._policy_to_art_idx] = target_qpos_policy
        try:
            # Isaac Sim 5.x SingleArticulation supports
            # ArticulationAction. Wrap target joint positions.
            from isaacsim.core.utils.types import ArticulationAction  # type: ignore

            self._art.apply_action(
                ArticulationAction(joint_positions=target_qpos_art)
            )
        except Exception as exc:   # pragma: no cover
            print(f"[phase5/policy] WARN apply_action failed: {exc}")

    @staticmethod
    def _force_usd_joint_drives_for_locomotion(
        articulation_root: str,
        stiffness: float,
        damping: float,
    ) -> None:
        """Apply UsdPhysics.DriveAPI("angular") with the given gains to
        every revolute joint under `articulation_root`. This is the
        layer-1 fix described in __init__: it guarantees every leg
        joint has a PhysX drive that holds the requested target — if
        the stock USD shipped without DriveAPI, the runtime
        ArticulationController.set_gains() call would silently no-op
        and Go2 would just collapse under gravity. After this call,
        set_gains() is the right path to update gains live.
        """
        # Lazy imports — these are expensive and unavailable outside Isaac.
        from pxr import UsdPhysics, PhysxSchema  # type: ignore
        import omni.usd  # type: ignore

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            print("[phase5/policy] _force_usd_joint_drives: no USD stage")
            return
        root_prim = stage.GetPrimAtPath(articulation_root)
        if not root_prim or not root_prim.IsValid():
            # Fall back to the parent /World/Go2 if /World/Go2/base
            # doesn't exist (USD composition variant).
            parent = articulation_root.rsplit("/", 1)[0]
            root_prim = stage.GetPrimAtPath(parent)
            if not root_prim or not root_prim.IsValid():
                print(
                    f"[phase5/policy] _force_usd_joint_drives: cannot resolve "
                    f"{articulation_root} or its parent"
                )
                return

        applied = 0
        for prim in iter(stage.Traverse()):
            # Only touch joints under the Go2 robot subtree.
            path_str = str(prim.GetPath())
            if not (
                path_str.startswith(articulation_root + "/")
                or path_str.startswith(
                    articulation_root.rsplit("/", 1)[0] + "/"
                )
            ):
                continue
            if not (prim.IsA(UsdPhysics.RevoluteJoint)
                    or prim.IsA(UsdPhysics.PrismaticJoint)):
                continue
            joint = UsdPhysics.RevoluteJoint(prim)
            if not joint:
                continue
            # The "angular" drive type is conventional for revolute
            # joints in PhysX (linear is for prismatic).
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            try:
                drive.CreateTypeAttr("force").Set("force")
            except Exception:
                pass
            try:
                drive.CreateStiffnessAttr().Set(float(stiffness))
                drive.CreateDampingAttr().Set(float(damping))
            except Exception:
                # Fallback for stale schema names.
                try:
                    prim.CreateAttribute(
                        "drive:angular:physics:stiffness", float
                    ).Set(float(stiffness))
                    prim.CreateAttribute(
                        "drive:angular:physics:damping", float
                    ).Set(float(damping))
                except Exception:
                    continue
            try:
                # PhysX-extended joint properties: ensure the joint is
                # actually controlled (not free-spinning) and use a
                # large enough max force that 25Nm/rad PD has authority.
                physx_joint = PhysxSchema.PhysxJointAPI.Apply(prim)
                # A 23.5 N·m saturation matches DCMotorCfg.effort_limit.
                # Set max force on the drive itself.
                try:
                    drive.CreateMaxForceAttr().Set(23.5)
                except Exception:
                    pass
            except Exception:
                pass
            applied += 1

        print(
            f"[phase5/policy] applied UsdPhysics.DriveAPI(angular, "
            f"stiffness={stiffness}, damping={damping}, maxForce=23.5) "
            f"to {applied} joints under {articulation_root}"
        )


_GO2_INIT_STATE_RULES: List[Tuple[str, float]] = [
    # Order matters: first match wins. Patterns mirror
    # IsaacLab v2.3.0 source/isaaclab_assets/.../unitree.py
    # UNITREE_GO2_CFG.init_state.joint_pos.
    (r".*L_hip_joint$", 0.1),
    (r".*R_hip_joint$", -0.1),
    (r"F[LR]_thigh_joint$", 0.8),
    (r"R[LR]_thigh_joint$", 1.0),
    (r".*_calf_joint$", -1.5),
]


def _go2_default_joint_pos_from_regex(joint_names: Sequence[str]) -> np.ndarray:
    """Replicate UNITREE_GO2_CFG.init_state.joint_pos resolution for
    an arbitrary joint-name ordering.

    Raises if any joint cannot be matched, because that would mean
    the articulation in this sim is not the standard Isaac Lab Go2
    and the policy obs/action layout is undefined.
    """
    import re

    out = np.zeros(len(joint_names), dtype=np.float32)
    for i, jn in enumerate(joint_names):
        matched = False
        for pat, val in _GO2_INIT_STATE_RULES:
            if re.match(pat, jn):
                out[i] = val
                matched = True
                break
        if not matched:
            raise RuntimeError(
                f"[phase5/policy] joint '{jn}' did not match any "
                f"UNITREE_GO2_CFG init_state regex; cannot derive "
                f"default_joint_pos. Expected names like "
                f"'FL_hip_joint', 'FR_thigh_joint', 'RR_calf_joint'."
            )
    return out


def _rot_from_wxyz(orn: Sequence[float]) -> np.ndarray:
    """WXYZ quaternion → 3×3 rotation matrix."""
    w, x, y, z = float(orn[0]), float(orn[1]), float(orn[2]), float(orn[3])
    return np.array(
        [
            [1 - 2 * (y * y + z * z),     2 * (x * y - w * z),     2 * (x * z + w * y)],
            [    2 * (x * y + w * z), 1 - 2 * (x * x + z * z),     2 * (y * z - w * x)],
            [    2 * (x * z - w * y),     2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


# -----------------------------------------------------------------------------
# Factory
# -----------------------------------------------------------------------------
def make_backend(
    name: str,
    cfg: LocomotionBackendConfig,
    policy_cfg: Optional[PolicyConfig] = None,
) -> Tuple[LocomotionBackend, Optional[str]]:
    """Construct a backend by name. Returns (backend, warning_or_None).

    If `name == "policy"` and construction fails (no checkpoint, no
    torch, joint mismatch, …), the function falls back to
    KinematicLocomotionBackend and surfaces the reason as the second
    return value so the caller can log it.
    """
    name = (name or "kinematic").lower()
    if name == "kinematic":
        return KinematicLocomotionBackend(cfg), None
    if name == "policy":
        try:
            pcfg = policy_cfg or PolicyConfig()
            return PolicyLocomotionBackend(cfg, pcfg), None
        except Exception as exc:
            warn = (
                f"PolicyLocomotionBackend unavailable ({exc}); "
                f"falling back to kinematic."
            )
            return KinematicLocomotionBackend(cfg), warn
    # Unknown.
    return (
        KinematicLocomotionBackend(cfg),
        f"Unknown locomotion backend '{name}', falling back to kinematic.",
    )
