"""Train biped V58 — +X forward, dual URDF (heavy/light).

PPO config (Berkeley-matched):
  - actor/critic: [512, 256, 128], ELU
  - init_noise_std: 1.0
  - entropy_coef: 0.005
  - LR: 1e-3 (adaptive)
  - max_iterations: 15000
  - save_interval: 200
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
parser.add_argument("--rough", action="store_true", help="Use rough terrain config")
parser.add_argument("--urdf", type=str, default="heavy", choices=["heavy", "light"], help="URDF variant")
parser.add_argument("--tanh", action="store_true", help="Add tanh output layer to actor (bounds actions to [-1, +1])")
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

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import OnPolicyRunner

from biped_env_cfg import BipedFlatEnvCfg

if args_cli.rough:
    from biped_rough_env_cfg import BipedRoughEnvCfg

# Register environments
gym.register(
    id="Biped-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg"},
)
gym.register(
    id="Biped-Rough-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_rough_env_cfg:BipedRoughEnvCfg"},
)

# EXACT Berkeley Flat PPO config
TRAIN_CFG = {
    "seed": 42,
    "runner_class_name": "OnPolicyRunner",
    "num_steps_per_env": 24,
    "max_iterations": 15000,
    "save_interval": 200,
    "experiment_name": "biped_flat_v57_implicit",
    "empirical_normalization": False,
    "obs_groups": {
        "policy": ["policy"],
        "critic": ["critic"],
    },
    "policy": {
        "class_name": "ActorCritic",
        "init_noise_std": 1.0,
        "actor_hidden_dims": [512, 256, 128],
        "critic_hidden_dims": [512, 256, 128],
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
        "symmetry_cfg": {
            "use_data_augmentation": True,
            "use_mirror_loss": False,
            "data_augmentation_func": "biped_symmetry:biped_symmetry_augmentation",
        },
    },
}


def main():
    if args_cli.rough:
        env_cfg = BipedRoughEnvCfg()
        env_id = "Biped-Rough-v0"
        experiment = "biped_rough_v57"
    else:
        env_cfg = BipedFlatEnvCfg()
        env_id = "Biped-Flat-v0"
        if args_cli.config == "v143":
            experiment = "biped_flat_v143"
        else:
            experiment = "biped_flat_v142"

    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    apply_urdf_selection(env_cfg, args_cli.urdf)

    env = gym.make(env_id, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    train_cfg = TRAIN_CFG.copy()
    train_cfg["max_iterations"] = args_cli.max_iterations
    train_cfg["seed"] = args_cli.seed
    train_cfg["experiment_name"] = experiment

    log_dir = os.path.join("/results/logs/rsl_rl", train_cfg["experiment_name"])
    os.makedirs(log_dir, exist_ok=True)


    runner = OnPolicyRunner(env, train_cfg, log_dir=log_dir, device="cuda:0")

    if args_cli.tanh:
        import torch.nn as nn
        original_actor = runner.alg.policy.actor
        runner.alg.policy.actor = nn.Sequential(original_actor, nn.Tanh())
        print("[INFO] Tanh output layer added to actor — actions bounded to [-1, +1]")

    if args_cli.resume:
        print(f"[INFO] Resuming from: {args_cli.resume}")
        runner.load(args_cli.resume)

    runner.learn(num_learning_iterations=args_cli.max_iterations, init_at_random_ep_len=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
