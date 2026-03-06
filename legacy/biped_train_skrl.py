"""Train biped flat terrain velocity tracking with skrl PPO."""

import argparse
import sys
import os

# Force headless + set PYTHONPATH
sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train biped with skrl PPO.")
parser.add_argument("--num_envs", type=int, default=4096, help="Number of environments.")
parser.add_argument("--max_iterations", type=int, default=2000, help="Max training iterations.")
parser.add_argument("--seed", type=int, default=42, help="Random seed.")
parser.add_argument("--checkpoint", type=str, default=None, help="Resume from checkpoint.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows after sim launch."""

import gymnasium as gym
from datetime import datetime

import skrl
from skrl.utils.runner.torch import Runner

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils.dict import print_dict
from isaaclab.utils.io import dump_yaml

from isaaclab_rl.skrl import SkrlVecEnvWrapper

# Register our custom environment
from biped_env_cfg import BipedFlatEnvCfg, BipedFlatEnvCfg_PLAY  # noqa: F401

gym.register(
    id="Isaac-Velocity-Flat-Biped-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg",
    },
)


def main():
    # Load env config
    env_cfg = BipedFlatEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    # Load skrl agent config
    import yaml
    agent_cfg_path = "/workspace/biped_locomotion/agents/skrl_flat_ppo_cfg.yaml"
    with open(agent_cfg_path) as f:
        agent_cfg = yaml.safe_load(f)

    # Override max iterations
    if args_cli.max_iterations:
        agent_cfg["trainer"]["timesteps"] = args_cli.max_iterations * agent_cfg["agent"]["rollouts"]
    agent_cfg["trainer"]["close_environment_at_exit"] = False
    agent_cfg["seed"] = args_cli.seed

    # Logging
    log_root_path = os.path.join("/results", "logs", "skrl", "biped_flat")
    os.makedirs(log_root_path, exist_ok=True)
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_ppo_torch"
    print(f"[INFO] Logging experiment in directory: {log_root_path}/{log_dir}")

    agent_cfg["agent"]["experiment"]["directory"] = log_root_path
    agent_cfg["agent"]["experiment"]["experiment_name"] = log_dir
    full_log_dir = os.path.join(log_root_path, log_dir)

    # Dump configs
    os.makedirs(os.path.join(full_log_dir, "params"), exist_ok=True)
    dump_yaml(os.path.join(full_log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(full_log_dir, "params", "agent.yaml"), agent_cfg)

    # Create environment
    env = gym.make("Isaac-Velocity-Flat-Biped-v0", cfg=env_cfg)

    # Wrap for skrl
    env = SkrlVecEnvWrapper(env, ml_framework="torch")

    # Create runner
    runner = Runner(env, agent_cfg)

    # Resume from checkpoint if specified
    if args_cli.checkpoint:
        print(f"[INFO] Loading checkpoint: {args_cli.checkpoint}")
        runner.agent.load(args_cli.checkpoint)
        # Reset ALL timestep counters so training runs for full max_iterations from here
        target_steps = args_cli.max_iterations * agent_cfg["agent"]["rollouts"]
        runner.trainer.initial_timestep = 0
        runner.trainer.timestep = 0
        runner.trainer.timesteps = target_steps
        print(f"[INFO] Reset timestep counters. initial=0, target={target_steps}")

    # Train
    runner.run()

    # Cleanup
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
