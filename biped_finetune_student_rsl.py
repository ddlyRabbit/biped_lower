"""Phase 3: Fine-tune distilled student with PPO.

Loads a distilled student checkpoint (from biped_distill_rsl.py) and runs
standard PPO training using student observations only (no base_lin_vel).

The distilled checkpoint contains student.* keys which are remapped to
actor.* for rsl_rl's ActorCritic. Critic is initialized fresh.

Usage:
  # Flat fine-tune
  python biped_finetune_student_rsl.py \
      --distilled /results/logs/rsl_rl/biped_distill_flat/model_3000.pt \
      --max_iterations 5000

  # Rough fine-tune
  python biped_finetune_student_rsl.py --rough \
      --distilled /results/logs/rsl_rl/biped_distill_rough/model_3000.pt \
      --max_iterations 5000

  # Resume PPO fine-tuning
  python biped_finetune_student_rsl.py --rough \
      --resume /results/logs/rsl_rl/biped_student_rough/model_2000.pt \
      --max_iterations 5000
"""

import argparse
import sys
import os

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=5000)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--rough", action="store_true", help="Use rough terrain env")
parser.add_argument("--distilled", type=str, default=None,
                    help="Path to distilled student checkpoint (Phase 2 output)")
parser.add_argument("--resume", type=str, default=None,
                    help="Path to PPO fine-tune checkpoint to resume from")
parser.add_argument("--urdf", type=str, default="heavy", choices=["heavy", "light"], help="URDF variant")
parser.add_argument("--tanh", action="store_true", help="Add tanh output to actor (must match distillation)")
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

if args_cli.rough:
    from biped_student_env_cfg import BipedStudentRoughEnvCfg
    env_cfg_class = BipedStudentRoughEnvCfg
    env_id = "Biped-Student-Rough-v0"
    experiment = "biped_student_rough"
else:
    from biped_student_env_cfg import BipedStudentFlatEnvCfg
    env_cfg_class = BipedStudentFlatEnvCfg
    env_id = "Biped-Student-Flat-v0"
    experiment = "biped_student_flat"

gym.register(
    id=env_id,
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": f"biped_student_env_cfg:{env_cfg_class.__name__}"},
)

# Same PPO config as teacher training — lower init_noise_std since
# student is already pre-trained via distillation
TRAIN_CFG = {
    "seed": 42,
    "runner_class_name": "OnPolicyRunner",
    "num_steps_per_env": 24,
    "max_iterations": 5000,
    "save_interval": 200,
    "experiment_name": experiment,
    "empirical_normalization": False,
    "obs_groups": {
        "policy": ["policy"],
        "critic": ["critic"],
    },
    "policy": {
        "class_name": "ActorCritic",
        "init_noise_std": 0.5,   # overwritten by distilled std (~0.1)
        "actor_hidden_dims": [128, 128, 128],
        "critic_hidden_dims": [128, 128, 128],
        "activation": "elu",
    },
    "algorithm": {
        "class_name": "PPO",
        "value_loss_coef": 1.0,
        "use_clipped_value_loss": True,
        "clip_param": 0.1,        # conservative — protect pre-trained actor
        "entropy_coef": 0.001,     # low — student already knows what to do
        "num_learning_epochs": 5,
        "num_mini_batches": 4,
        "learning_rate": 3.0e-4,   # 3x lower than teacher — gentle fine-tuning
        "schedule": "adaptive",
        "gamma": 0.99,
        "lam": 0.95,
        "desired_kl": 0.01,
        "max_grad_norm": 1.0,
    },
}


def remap_distilled_to_actor_critic(distilled_path, runner):
    """Load distilled student weights into OnPolicyRunner's ActorCritic.

    Distilled checkpoint has:
      - student.0.weight, student.0.bias, ... (MLP layers)
      - std (action noise)
    ActorCritic (rsl_rl) has:
      - actor.0.weight, actor.0.bias, ...
      - critic.0.weight, ...
      - std

    Student → actor. Critic left at random init. std is copied.
    """
    ckpt = torch.load(distilled_path, map_location="cuda:0")
    model_sd = ckpt["model_state_dict"]

    # Remap student.* → actor.*
    # With --tanh: both student and actor are Sequential(MLP, Tanh)
    #   student.0.X → actor.0.X (straight replace, keep "0." prefix)
    # Without --tanh: both are plain MLP
    #   student.X → actor.X (straight replace)
    remap_sd = {}
    for k, v in model_sd.items():
        if k.startswith("student."):
            remap_sd[k.replace("student.", "actor.", 1)] = v
        elif k == "std":
            remap_sd["std"] = v

    if not any(k.startswith("actor.") for k in remap_sd):
        raise ValueError(
            f"No student.* keys in checkpoint. "
            f"Available prefixes: {set(k.split('.')[0] for k in model_sd.keys())}"
        )

    # Get current model state dict and update actor + std
    policy = runner.alg.policy
    current_sd = policy.state_dict()
    matched = 0
    for k, v in remap_sd.items():
        if k in current_sd:
            if current_sd[k].shape == v.shape:
                current_sd[k] = v
                matched += 1
            else:
                print(f"[WARN] Shape mismatch for {k}: expected {current_sd[k].shape}, got {v.shape}")
        else:
            print(f"[WARN] Key {k} not found in ActorCritic")

    policy.load_state_dict(current_sd)
    print(f"[INFO] Loaded {matched} weight tensors from distilled checkpoint")
    print(f"[INFO] Critic initialized fresh (random)")


def main():
    env_cfg = env_cfg_class()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    apply_urdf_selection(env_cfg, args_cli.urdf)

    if args_cli.rough and args_cli.num_envs > 8192:
        env_cfg.scene.num_envs = 8192
        print(f"[INFO] Reduced num_envs to 8192 for rough terrain")

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
        # Resume from PPO fine-tune checkpoint (has actor.* + critic.*)
        print(f"[INFO] Resuming PPO fine-tune from: {args_cli.resume}")
        runner.load(args_cli.resume)
    elif args_cli.distilled:
        # Load distilled student weights → actor, critic stays random
        print(f"[INFO] Loading distilled student from: {args_cli.distilled}")
        remap_distilled_to_actor_critic(args_cli.distilled, runner)
    else:
        print("[INFO] Training student from scratch (no distilled checkpoint)")

    runner.learn(num_learning_iterations=args_cli.max_iterations, init_at_random_ep_len=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
