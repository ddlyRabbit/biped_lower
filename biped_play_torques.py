"""Play biped policy and record joint torques to CSV."""

import argparse
import sys
import os

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--steps", type=int, default=1000)
parser.add_argument("--rough", action="store_true")
parser.add_argument("--output", type=str, default="/results/torques.csv")
parser.add_argument("--student", action="store_true", help="Play student (distilled) policy")
parser.add_argument("--urdf", type=str, default="heavy", choices=["heavy", "light"], help="URDF variant")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
def apply_urdf_selection(env_cfg, urdf_choice):
    """Override URDF path based on --urdf flag."""
    from biped_env_cfg import URDF_HEAVY, URDF_LIGHT
    if urdf_choice == "light":
        env_cfg.scene.robot.spawn.asset_path = URDF_LIGHT
        print(f"[INFO] Using light URDF: {URDF_LIGHT}")
    else:
        env_cfg.scene.robot.spawn.asset_path = URDF_HEAVY
        print(f"[INFO] Using heavy URDF: {URDF_HEAVY}")

import torch
import torch.nn as nn
import csv

from isaaclab_rl.skrl import SkrlVecEnvWrapper
from biped_env_cfg import BipedFlatEnvCfg_PLAY

if args_cli.rough:
    from biped_rough_env_cfg import BipedRoughEnvCfg_PLAY

gym.register(
    id="Biped-Flat-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg_PLAY"},
)
gym.register(
    id="Biped-Rough-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_rough_env_cfg:BipedRoughEnvCfg_PLAY"},
)

JOINT_NAMES = [
    "left_hip_pitch_04", "right_hip_pitch_04",
    "left_hip_roll_03", "right_hip_roll_03",
    "left_hip_yaw_03", "right_hip_yaw_03",
    "left_knee_04", "right_knee_04",
    "left_foot_pitch_02", "right_foot_pitch_02",
    "left_foot_roll_02", "right_foot_roll_02",
]


def main():
    if args_cli.rough:
        env_cfg = BipedRoughEnvCfg_PLAY()
        env_id = "Biped-Rough-Play-v0"
    else:
        env_cfg = BipedFlatEnvCfg_PLAY()
        env_id = "Biped-Flat-Play-v0"

    env_cfg.scene.num_envs = args_cli.num_envs
    apply_urdf_selection(env_cfg, args_cli.urdf)
    print("[DBG] Creating env", flush=True)
    env = gym.make(env_id, cfg=env_cfg)
    env = SkrlVecEnvWrapper(env, ml_framework="torch")
    obs, _ = env.reset()
    print(f"[DBG] obs shape={obs.shape}", flush=True)

    # Load actor
    ckpt = torch.load(args_cli.checkpoint, map_location="cuda:0")
    num_obs = obs.shape[-1]
    num_actions = env.action_space.shape[-1]

    actor = nn.Sequential(
        nn.Linear(num_obs, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, num_actions),).to("cuda:0")

    model_sd = ckpt["model_state_dict"]
    if hasattr(args_cli, "student") and args_cli.student:
        actor_sd = {k.replace("student.", ""): v for k, v in model_sd.items() if k.startswith("student.")}
        if not actor_sd:
            raise ValueError("No student.* keys in checkpoint")
    else:
        actor_sd = {k.replace("actor.0.", ""): v for k, v in model_sd.items() if k.startswith("actor.0.")}
    actor.load_state_dict(actor_sd)
    actor.eval()
    print("[DBG] Actor loaded, starting loop", flush=True)

    # Get reference to the underlying Isaac env for torque data
    isaac_env = env.unwrapped.unwrapped
    robot = isaac_env.scene["robot"]

    # CSV header
    os.makedirs(os.path.dirname(args_cli.output), exist_ok=True)
    f = open(args_cli.output, "w", newline="")
    writer = csv.writer(f)

    header = ["step", "time_s"]
    for name in JOINT_NAMES:
        header.extend([
            name + "_torque",
            name + "_pos",
            name + "_vel",
            name + "_action",
        ])
    header.extend(["base_lin_vel_x", "base_lin_vel_y", "base_lin_vel_z",
                    "base_ang_vel_x", "base_ang_vel_y", "base_ang_vel_z"])
    writer.writerow(header)

    dt = isaac_env.step_dt

    for step in range(args_cli.steps):
        with torch.inference_mode():
            actions = actor(obs)
        obs, _, _, _, _ = env.step(actions)

        # Extract data for env 0
        torques = robot.data.applied_torque[0].cpu().numpy()
        joint_pos = robot.data.joint_pos[0].cpu().numpy()
        joint_vel = robot.data.joint_vel[0].cpu().numpy()
        act = actions[0].cpu().numpy()
        base_lin = robot.data.root_lin_vel_w[0].cpu().numpy()
        base_ang = robot.data.root_ang_vel_w[0].cpu().numpy()

        row = [step, round(step * dt, 4)]
        for j in range(len(JOINT_NAMES)):
            row.extend([
                round(float(torques[j]), 4),
                round(float(joint_pos[j]), 4),
                round(float(joint_vel[j]), 4),
                round(float(act[j]), 4),
            ])
        row.extend([round(float(v), 4) for v in base_lin])
        row.extend([round(float(v), 4) for v in base_ang])
        writer.writerow(row)

        if step % 100 == 0:
            f.flush()

    f.close()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
