"""Train biped V52 — EXACT Berkeley Humanoid PPO config.

PPO differences from V51b:
  - actor/critic: [128, 128, 128] (was [256, 128, 128])
  - init_noise_std: 1.0 (was 1.5)
  - entropy_coef: 0.005 (was 0.02)
  - max_iterations: 15000 (was 6000)
  - save_interval: 200 (was 100)
"""

import argparse
import sys
import os

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=15000)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--resume", type=str, default=None)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner

from biped_env_cfg import BipedFlatEnvCfg

# Register environment
gym.register(
    id="Biped-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg"},
)

# EXACT Berkeley Flat PPO config
TRAIN_CFG = {
    "seed": 42,
    "runner_class_name": "OnPolicyRunner",
    "num_steps_per_env": 24,
    "max_iterations": 15000,
    "save_interval": 200,
    "experiment_name": "biped_flat_v52",
    "empirical_normalization": False,
    "obs_groups": {
        "policy": ["policy"],
        "critic": ["critic"],
    },
    "policy": {
        "class_name": "ActorCritic",
        "init_noise_std": 1.0,
        "actor_hidden_dims": [128, 128, 128],
        "critic_hidden_dims": [128, 128, 128],
        "activation": "elu",
    },
    "algorithm": {
        "class_name": "PPO",
        "value_loss_coef": 1.0,
        "use_clipped_value_loss": True,
        "clip_param": 0.2,
        "entropy_coef": 0.005,
        "num_learning_epochs": 5,
        "num_mini_batches": 4,
        "learning_rate": 1.0e-3,
        "schedule": "adaptive",
        "gamma": 0.99,
        "lam": 0.95,
        "desired_kl": 0.01,
        "max_grad_norm": 1.0,
    },
}


def main():
    env_cfg = BipedFlatEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    env = gym.make("Biped-Flat-v0", cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    train_cfg = TRAIN_CFG.copy()
    train_cfg["max_iterations"] = args_cli.max_iterations
    train_cfg["seed"] = args_cli.seed

    log_dir = os.path.join("/results/logs/rsl_rl", train_cfg["experiment_name"])
    os.makedirs(log_dir, exist_ok=True)

    runner = OnPolicyRunner(env, train_cfg, log_dir=log_dir, device="cuda:0")

    if args_cli.resume:
        print(f"[INFO] Resuming from: {args_cli.resume}")
        runner.load(args_cli.resume)

    runner.learn(num_learning_iterations=args_cli.max_iterations, init_at_random_ep_len=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
