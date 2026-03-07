"""Biped Teacher-Student Distillation — G1-style.

Two modes:
  1. Distillation (default):
     - Loads teacher checkpoint (PPO-trained flat policy with base_lin_vel)
     - Trains student MLP via MSE loss on teacher actions
     - Student obs: 45-dim (no base_lin_vel)
     - Teacher obs: 48-dim (includes base_lin_vel)
     - Uses rsl_rl DistillationRunner natively

  2. Fine-tune (--finetune):
     - Standard PPO training using student obs only (no base_lin_vel)
     - Initialize from distilled student checkpoint
     - NOT YET IMPLEMENTED — placeholder for Phase 3

Usage:
  # Phase 2: Distill student from teacher
  python biped_distill_rsl.py --teacher_checkpoint /results/winners/v57_model_2899.pt \\
      --num_envs 16384 --max_iterations 3000

  # Resume distillation
  python biped_distill_rsl.py --teacher_checkpoint /results/winners/v57_model_2899.pt \\
      --resume /results/logs/rsl_rl/biped_distill_v57/model_1000.pt --max_iterations 3000
"""

import argparse
import sys
import os

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16384)
parser.add_argument("--max_iterations", type=int, default=3000)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--teacher_checkpoint", type=str, default=None,
                    help="Path to teacher model checkpoint (PPO-trained). Required for fresh distillation.")
parser.add_argument("--resume", type=str, default=None,
                    help="Path to distillation checkpoint to resume from")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import DistillationRunner

from biped_student_env_cfg import BipedStudentFlatEnvCfg

# Register environment
gym.register(
    id="Biped-Student-Flat-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_student_env_cfg:BipedStudentFlatEnvCfg"},
)

# Distillation config — matches rsl_rl DistillationRunner expectations
DISTILL_CFG = {
    "seed": 42,
    "runner_class_name": "DistillationRunner",
    "num_steps_per_env": 24,
    "max_iterations": 3000,
    "save_interval": 200,
    "experiment_name": "biped_distill_v57",
    "empirical_normalization": False,
    "obs_groups": {
        "policy": ["policy"],     # student obs (45-dim, no base_lin_vel)
        "teacher": ["teacher"],   # teacher obs (48-dim, with base_lin_vel)
        "critic": ["critic"],     # not used in distillation but required by env
    },
    "policy": {
        "class_name": "StudentTeacher",
        "student_hidden_dims": [128, 128, 128],
        "teacher_hidden_dims": [128, 128, 128],
        "activation": "elu",
        "init_noise_std": 0.1,
        "student_obs_normalization": False,
        "teacher_obs_normalization": False,
    },
    "algorithm": {
        "class_name": "Distillation",
        "learning_rate": 1.0e-3,
        "num_learning_epochs": 5,
        "gradient_length": 15,
        "loss_type": "mse",
        "max_grad_norm": 1.0,
    },
}


def main():
    env_cfg = BipedStudentFlatEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    env = gym.make("Biped-Student-Flat-v0", cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    train_cfg = DISTILL_CFG.copy()
    train_cfg["max_iterations"] = args_cli.max_iterations
    train_cfg["seed"] = args_cli.seed

    log_dir = os.path.join("/results/logs/rsl_rl", train_cfg["experiment_name"])
    os.makedirs(log_dir, exist_ok=True)

    runner = DistillationRunner(env, train_cfg, log_dir=log_dir, device="cuda:0")

    if args_cli.resume:
        # Resume from distillation checkpoint — contains both student + teacher weights
        print(f"[INFO] Resuming distillation from: {args_cli.resume}")
        runner.load(args_cli.resume)
    elif args_cli.teacher_checkpoint:
        # Fresh distillation — load teacher from PPO checkpoint
        # StudentTeacher.load_state_dict detects actor.* keys → maps to teacher weights
        print(f"[INFO] Loading teacher from: {args_cli.teacher_checkpoint}")
        runner.load(args_cli.teacher_checkpoint, load_optimizer=False)
    else:
        raise ValueError("Must provide --teacher_checkpoint for fresh distillation or --resume to continue")

    runner.learn(num_learning_iterations=args_cli.max_iterations, init_at_random_ep_len=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
