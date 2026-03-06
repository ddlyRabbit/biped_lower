"""Play a trained biped policy checkpoint with skrl."""

import argparse
import sys
import os
import time

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play trained biped policy.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments.")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to model checkpoint (.pt).")
parser.add_argument("--seed", type=int, default=42, help="Random seed.")
parser.add_argument("--video", action="store_true", default=False, help="Record video.")
parser.add_argument("--video_length", type=int, default=300, help="Video length in steps.")
parser.add_argument("--real_time", action="store_true", default=False, help="Run in real-time.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.video:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows after sim launch."""

import gymnasium as gym
import torch
import yaml

import skrl
from skrl.utils.runner.torch import Runner

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab_rl.skrl import SkrlVecEnvWrapper

from biped_env_cfg import BipedFlatEnvCfg, BipedFlatEnvCfg_PLAY

# Register the play environment
gym.register(
    id="Isaac-Velocity-Flat-Biped-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg_PLAY",
    },
)


def main():
    # Use the PLAY config (no perturbations, constant forward velocity command)
    env_cfg = BipedFlatEnvCfg_PLAY()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    # Load agent config
    agent_cfg_path = "/workspace/biped_locomotion/agents/skrl_flat_ppo_cfg.yaml"
    with open(agent_cfg_path) as f:
        agent_cfg = yaml.safe_load(f)

    # Play-specific overrides
    agent_cfg["trainer"]["close_environment_at_exit"] = False
    agent_cfg["trainer"]["timesteps"] = 0  # no training
    agent_cfg["agent"]["experiment"]["write_interval"] = 0  # no TensorBoard
    agent_cfg["agent"]["experiment"]["checkpoint_interval"] = 0  # no checkpoints
    agent_cfg["seed"] = args_cli.seed

    # Create environment
    env = gym.make(
        "Isaac-Velocity-Flat-Biped-Play-v0",
        cfg=env_cfg,
        render_mode="rgb_array" if args_cli.video else None,
    )

    # Get step dt for real-time playback
    try:
        dt = env.step_dt
    except AttributeError:
        dt = env.unwrapped.step_dt

    # Video recording wrapper
    if args_cli.video:
        log_dir = os.path.dirname(os.path.abspath(args_cli.checkpoint))
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "..", "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print(f"[INFO] Recording video: {video_kwargs}")
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # Wrap for skrl
    env = SkrlVecEnvWrapper(env, ml_framework="torch")

    # Create runner & load checkpoint
    runner = Runner(env, agent_cfg)
    resume_path = os.path.abspath(args_cli.checkpoint)
    print(f"[INFO] Loading checkpoint: {resume_path}")
    runner.agent.load(resume_path)
    runner.agent.set_running_mode("eval")

    # Reset and run inference loop
    obs, _ = env.reset()
    timestep = 0
    print("[INFO] Playing policy... Close the window or Ctrl+C to stop.")

    while simulation_app.is_running():
        start_time = time.time()

        with torch.inference_mode():
            # Get deterministic actions (mean_actions) from the policy
            outputs = runner.agent.act(obs, timestep=0, timesteps=0)
            actions = outputs[-1].get("mean_actions", outputs[0])
            obs, _, _, _, _ = env.step(actions)

        if args_cli.video:
            timestep += 1
            if timestep == args_cli.video_length:
                break

        # Real-time pacing
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
