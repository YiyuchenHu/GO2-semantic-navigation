"""Phase 5 — pluggable locomotion backends for the Isaac Sim Go2.

Abstraction:
    LocomotionBackend.step(dt, lin_cmd, ang_cmd) is called every
    physics tick with the most recent body-frame velocity command
    (read from the /cmd_vel → SubTwist OmniGraph node by
    LocomotionDriver). A backend decides how to turn that command
    into robot motion that PhysX (and therefore
    IsaacComputeOdometry → /odom, /tf) can see.

Two backends ship with Phase 5:

  * KinematicLocomotionBackend — the Phase 0 cheat, preserved as the
    default and as a debug fallback. It disables articulation gravity
    and teleports the base via SingleArticulation.set_world_pose() at
    each step, integrating the commanded body-frame velocity in the
    world frame. No joint motion, no gait.

  * PolicyLocomotionBackend — a SCAFFOLD for a real Isaac Lab Go2
    flat-terrain velocity policy. It expects a torchscript checkpoint
    (`.pt`) + a small YAML describing the observation layout and the
    articulation joint order. At __init__ time it will either succeed
    (torch + checkpoint + config all present) or raise, letting the
    caller fall back to the kinematic backend. The step() method
    itself runs policy inference and applies joint position targets,
    but this has NOT been validated end-to-end in this repo — see
    docs/phase5_status.md for the gap.

Factory:
    make_backend(name, **kwargs) constructs the requested backend and,
    on any PolicyLocomotionBackend construction failure, returns a
    KinematicLocomotionBackend together with a warning string, so the
    sim keeps running.
"""

from __future__ import annotations

import math
import os
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

    Fields are intentionally explicit because Isaac Lab configs vary
    subtly between task variants; the defaults below reflect a
    textbook `Isaac-Velocity-Flat-Unitree-Go2` setup (48-dim obs,
    12-dim action).
    """

    checkpoint_path: str = ""
    # Ordered DoF names as they appear in the trained policy.
    joint_names_policy_order: List[str] = field(
        default_factory=lambda: [
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
        ]
    )
    # Default joint angles at which the policy was trained; action
    # output is `default + scale * raw_action`.
    default_joint_pos: List[float] = field(
        default_factory=lambda: [
             0.1,  -0.1,   0.1,  -0.1,
             0.8,   0.8,   1.0,   1.0,
            -1.5,  -1.5,  -1.5,  -1.5,
        ]
    )
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
        print(
            f"[phase5/policy] loaded checkpoint={policy_cfg.checkpoint_path} "
            f"joints={len(policy_cfg.joint_names_policy_order)} "
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

        # Build a mapping from the policy's joint order to the
        # articulation's DoF order (which is determined by USD
        # authoring). This is required because the two are not
        # guaranteed to match.
        dof_names = list(self._art.dof_names)  # type: ignore[attr-defined]
        self._policy_to_art_idx = []
        for jname in policy_cfg.joint_names_policy_order:
            if jname not in dof_names:
                raise RuntimeError(
                    f"[phase5/policy] joint '{jname}' required by policy "
                    f"checkpoint not found in articulation DoFs: {dof_names}"
                )
            self._policy_to_art_idx.append(dof_names.index(jname))
        self._policy_to_art_idx = np.asarray(
            self._policy_to_art_idx, dtype=np.int64
        )
        self._default_qpos_policy = np.asarray(
            policy_cfg.default_joint_pos, dtype=np.float32
        )

        self._last_action = np.zeros(
            len(policy_cfg.joint_names_policy_order), dtype=np.float32
        )
        self._decim_counter = 0

        print(
            f"[phase5/policy] bound to {cfg.articulation_path}  "
            f"dof_count={len(dof_names)}"
        )

    def step(self, dt, lin_cmd, ang_cmd) -> None:
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

        obs = self._build_observation(vx, vy, wz)

        with self._torch.no_grad():
            action_tensor = self._model(
                self._torch.from_numpy(obs).unsqueeze(0)
            ).squeeze(0)
        action = action_tensor.detach().cpu().numpy().astype(np.float32)
        self._last_action = action
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

        obs = np.concatenate(
            [
                base_lin_b.astype(np.float32),
                base_ang_b.astype(np.float32),
                gravity_b.astype(np.float32),
                np.array([vx, vy, wz], dtype=np.float32),
                qpos_rel.astype(np.float32),
                qvel_art.astype(np.float32),
                self._last_action.astype(np.float32),
            ]
        )
        return obs

    def _apply_last_action(self) -> None:
        """Apply `_last_action` as joint position targets."""
        if self._last_action is None:
            return
        target_qpos_policy = (
            self._default_qpos_policy
            + self._pcfg.action_scale * self._last_action
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
