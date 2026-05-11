"""Dump the IsaacLab joint name order for the Go2 flat-velocity task.

Run it via IsaacLab's bundled python (which boots Kit/omni so isaaclab is
fully importable):

    cd /home/yiyuchenhu/IsaacLab_clean
    ./isaaclab.sh -p /home/yiyuchenhu/Desktop/2026spring/.../scripts/dump_isaaclab_joint_order.py

It prints two things we need to compare against the sim runtime:

  IsaacLab joint_names (training-time policy I/O order)
  IsaacLab default_joint_pos in that order

If they match what `[phase5/policy] auto-resolved joint_names_policy_order`
prints in the sim run, the auto-resolve was correct. If they DON'T match,
we've just found the bug — the action/obs vectors are being mapped to the
wrong physical joints.
"""
from __future__ import annotations

# Boot Kit headlessly via IsaacLab's AppLauncher so omni / isaaclab are usable.
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402

import isaaclab_tasks  # noqa: E402,F401  (registers the task)


def main() -> None:
    task = "Isaac-Velocity-Flat-Unitree-Go2-v0"
    env = gym.make(task, num_envs=1)

    robot = env.unwrapped.scene["robot"]
    joint_names = list(robot.joint_names)
    default_qpos_t = robot.data.default_joint_pos
    default_qpos = [float(x) for x in default_qpos_t.cpu().numpy().reshape(-1)]

    print("=" * 78)
    print(f"task: {task}")
    print(f"num_joints = {len(joint_names)}")
    print()
    print("IsaacLab joint_names (training-time order):")
    for i, n in enumerate(joint_names):
        print(f"  [{i:2d}] {n}    default={default_qpos[i]:+.3f}")
    print()
    print("As Python list (paste this into PolicyConfig.joint_names_policy_order):")
    print(repr(joint_names))
    print()
    print("As Python list (paste this into PolicyConfig.default_joint_pos):")
    print(repr(default_qpos))
    print("=" * 78)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
