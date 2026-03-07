"""Play biped V52 rsl_rl policy — uses RecordVideo."""

import argparse
import sys
import os

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--video", action="store_true")
parser.add_argument("--video_length", type=int, default=300)
parser.add_argument("--rough", action="store_true", help="Use rough terrain config")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.video:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import torch.nn as nn

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
    if args_cli.rough:
        env_cfg = BipedRoughEnvCfg_PLAY()
        env_id = "Biped-Rough-Play-v0"
    else:
        env_cfg = BipedFlatEnvCfg_PLAY()
        env_id = "Biped-Flat-Play-v0"

    env_cfg.scene.num_envs = args_cli.num_envs

    env = gym.make(
        env_id,
        cfg=env_cfg,
        render_mode="rgb_array" if args_cli.video else None,
    )

    if args_cli.video:
        video_dir = "/results/videos/v52"
        os.makedirs(video_dir, exist_ok=True)
        env = gym.wrappers.RecordVideo(
            env,
            video_folder=video_dir,
            step_trigger=lambda step: step == 0,
            video_length=args_cli.video_length,
            disable_logger=True,
        )
        print(f"[INFO] Recording video to {video_dir}")

    env = SkrlVecEnvWrapper(env, ml_framework="torch")
    obs, _ = env.reset()

    print(f"[INFO] Loading checkpoint: {args_cli.checkpoint}")
    ckpt = torch.load(args_cli.checkpoint, map_location="cuda:0")

    num_obs = obs.shape[-1]
    num_actions = env.action_space.shape[-1]
    print(f"[INFO] obs_dim={num_obs}, actions={num_actions}")

    # Berkeley flat: [128, 128, 128] actor
    actor = nn.Sequential(
        nn.Linear(num_obs, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, num_actions),
    ).to("cuda:0")

    model_sd = ckpt["model_state_dict"]
    actor_sd = {}
    for k, v in model_sd.items():
        if k.startswith("actor."):
            actor_sd[k.replace("actor.", "")] = v
    actor.load_state_dict(actor_sd)
    actor.eval()
    print("[INFO] Actor loaded successfully")

    timestep = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = actor(obs)
        obs, _, _, _, _ = env.step(actions)
        timestep += 1

        if timestep % 50 == 0:
            print(f"[INFO] Step {timestep}")

        if args_cli.video and timestep >= args_cli.video_length:
            print(f"[INFO] Video recording complete ({timestep} steps)")
            break

    env.close()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
    simulation_app.close()
