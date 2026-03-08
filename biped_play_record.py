"""Play biped: record video + joint torques simultaneously."""

import argparse
import sys
import os

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--video_length", type=int, default=500)
parser.add_argument("--rough", action="store_true")
parser.add_argument("--output_dir", type=str, default="/results/record")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if True:  # always need cameras for video
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import torch.nn as nn
import csv
import json

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

def main():
    out_dir = args_cli.output_dir
    os.makedirs(out_dir, exist_ok=True)

    if args_cli.rough:
        env_cfg = BipedRoughEnvCfg_PLAY()
        env_id = "Biped-Rough-Play-v0"
    else:
        env_cfg = BipedFlatEnvCfg_PLAY()
        env_id = "Biped-Flat-Play-v0"

    env_cfg.scene.num_envs = args_cli.num_envs

    # Camera follows robot 0
    env_cfg.viewer.origin_type = "asset_root"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.env_index = 0
    env_cfg.viewer.eye = (3.0, 3.0, 2.0)
    env_cfg.viewer.lookat = (0.0, 0.0, 0.5)

    env = gym.make(env_id, cfg=env_cfg, render_mode="rgb_array")

    video_dir = os.path.join(out_dir, "video")
    os.makedirs(video_dir, exist_ok=True)
    env = gym.wrappers.RecordVideo(
        env,
        video_folder=video_dir,
        step_trigger=lambda step: step == 0,
        video_length=args_cli.video_length,
        disable_logger=True,
    )

    env = SkrlVecEnvWrapper(env, ml_framework="torch")
    obs, _ = env.reset()

    # Load actor
    ckpt = torch.load(args_cli.checkpoint, map_location="cuda:0")
    num_obs = obs.shape[-1]
    num_actions = env.action_space.shape[-1]

    actor = nn.Sequential(
        nn.Linear(num_obs, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, num_actions),
    ).to("cuda:0")

    actor_sd = {}
    for k, v in ckpt["model_state_dict"].items():
        if k.startswith("actor."):
            actor_sd[k.replace("actor.", "")] = v
    actor.load_state_dict(actor_sd)
    actor.eval()

    # Get robot reference for torque data
    isaac_env = env.unwrapped.unwrapped.unwrapped  # SkrlVecEnv -> RecordVideo -> IsaacEnv
    robot = isaac_env.scene["robot"]
    dt = isaac_env.step_dt

    # Read actual joint names from simulation (correct order!)
    JOINT_NAMES = list(robot.joint_names)
    num_joints = len(JOINT_NAMES)

    # Save joint order for reference
    with open(os.path.join(out_dir, "joint_order.txt"), "w") as jf:
        for i, name in enumerate(JOINT_NAMES):
            jf.write("%d: %s\n" % (i, name))

    # CSV for torques
    csv_path = os.path.join(out_dir, "torques.csv")
    f = open(csv_path, "w", newline="")
    writer = csv.writer(f)

    header = ["step", "time_s"]
    for name in JOINT_NAMES:
        header.extend([name + "_torque", name + "_pos", name + "_vel", name + "_action"])
    header.extend(["base_vel_x", "base_vel_y", "base_vel_z",
                    "base_angvel_x", "base_angvel_y", "base_angvel_z"])
    writer.writerow(header)

    for step in range(args_cli.video_length):
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

        if step % 50 == 0:
            f.flush()

    f.close()

    # Save metadata
    meta = {
        "joint_names": JOINT_NAMES,
        "dt": dt,
        "steps": args_cli.video_length,
        "fps": int(1.0 / dt) if dt > 0 else 50,
        "checkpoint": args_cli.checkpoint,
        "rough": args_cli.rough,
    }
    with open(os.path.join(out_dir, "meta.json"), "w") as mf:
        json.dump(meta, mf, indent=2)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
