"""Biped Teacher-Student Distillation — G1-style.

Supports both flat and rough terrain distillation.

Usage:
  # Flat distillation
  python biped_distill_rsl.py --teacher_checkpoint /results/winners/flat_teacher.pt

  # Rough distillation
  python biped_distill_rsl.py --rough --teacher_checkpoint /results/logs/rsl_rl/biped_rough_v57/model_19200.pt

  # Resume
  python biped_distill_rsl.py --rough --resume /results/logs/rsl_rl/biped_distill_rough/model_1000.pt \\
      --teacher_checkpoint /results/logs/rsl_rl/biped_rough_v57/model_19200.pt
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
parser.add_argument("--rough", action="store_true", help="Use rough terrain env")
parser.add_argument("--teacher_checkpoint", type=str, default=None,
                    help="Path to teacher model checkpoint (PPO-trained)")
parser.add_argument("--resume", type=str, default=None,
                    help="Path to distillation checkpoint to resume from")
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

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from rsl_rl.runners import DistillationRunner

if args_cli.rough:
    from biped_student_env_cfg import BipedStudentRoughEnvCfg
    env_cfg_class = BipedStudentRoughEnvCfg
    env_id = "Biped-Student-Rough-v0"
    experiment_name = "biped_distill_rough"
else:
    from biped_student_env_cfg import BipedStudentFlatEnvCfg
    env_cfg_class = BipedStudentFlatEnvCfg
    env_id = "Biped-Student-Flat-v0"
    experiment_name = "biped_distill_flat"

# Register environment
gym.register(
    id=env_id,
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"biped_student_env_cfg:{env_cfg_class.__name__}"},
)

# Distillation config
DISTILL_CFG = {
    "seed": 42,
    "runner_class_name": "DistillationRunner",
    "num_steps_per_env": 24,
    "max_iterations": 3000,
    "save_interval": 200,
    "experiment_name": experiment_name,
    "empirical_normalization": False,
    "obs_groups": {
        "policy": ["policy"],     # student obs (45-dim, no base_lin_vel/height_scan)
        "teacher": ["teacher"],   # teacher obs (48-dim flat / 235-dim rough)
        "critic": ["critic"],
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
    env_cfg = env_cfg_class()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed

    # Reduce envs for rough terrain (VRAM)
    if args_cli.rough and args_cli.num_envs > 8192:
        env_cfg.scene.num_envs = 8192
        print(f"[INFO] Reduced num_envs to 8192 for rough terrain")

    env = gym.make(env_id, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    train_cfg = DISTILL_CFG.copy()
    train_cfg["max_iterations"] = args_cli.max_iterations
    train_cfg["seed"] = args_cli.seed

    log_dir = os.path.join("/results/logs/rsl_rl", train_cfg["experiment_name"])
    os.makedirs(log_dir, exist_ok=True)

    runner = DistillationRunner(env, train_cfg, log_dir=log_dir, device="cuda:0")

    if args_cli.resume:
        print(f"[INFO] Resuming distillation from: {args_cli.resume}")
        runner.load(args_cli.resume)
    elif args_cli.teacher_checkpoint:
        print(f"[INFO] Loading teacher from: {args_cli.teacher_checkpoint}")
        runner.load(args_cli.teacher_checkpoint, load_optimizer=False)
    else:
        raise ValueError("Must provide --teacher_checkpoint or --resume")

    runner.learn(num_learning_iterations=args_cli.max_iterations, init_at_random_ep_len=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
