"""Play biped rsl_rl policy — supports teacher and student models."""

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
parser.add_argument("--student", action="store_true", help="Play student (distilled) policy")
parser.add_argument("--env_index", type=int, default=0, help="Which env to follow with camera")
parser.add_argument("--global_camera", action="store_true", help="Fixed global camera view (for multi-env recording)")
parser.add_argument("--video_dir", type=str, default="/results/videos", help="Video output directory")
parser.add_argument("--urdf", type=str, default="heavy", choices=["heavy", "light"], help="URDF variant")
parser.add_argument("--tanh", action="store_true", help="Actor has tanh output layer")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.video:
    args_cli.enable_cameras = True

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

from isaaclab_rl.skrl import SkrlVecEnvWrapper

# Import configs based on mode
if args_cli.student:
    if args_cli.rough:
        from biped_student_env_cfg import BipedStudentRoughEnvCfg_PLAY
    else:
        from biped_student_env_cfg import BipedStudentFlatEnvCfg_PLAY
else:
    from biped_env_cfg import BipedFlatEnvCfg_PLAY
    if args_cli.rough:
        from biped_rough_env_cfg import BipedRoughEnvCfg_PLAY

# Register environments
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
gym.register(
    id="Biped-Student-Flat-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_student_env_cfg:BipedStudentFlatEnvCfg_PLAY"},
)
gym.register(
    id="Biped-Student-Rough-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_student_env_cfg:BipedStudentRoughEnvCfg_PLAY"},
)


def main():
    # Select env config
    if args_cli.student:
        if args_cli.rough:
            env_cfg = BipedStudentRoughEnvCfg_PLAY()
            env_id = "Biped-Student-Rough-Play-v0"
        else:
            env_cfg = BipedStudentFlatEnvCfg_PLAY()
            env_id = "Biped-Student-Flat-Play-v0"
    else:
        if args_cli.rough:
            env_cfg = BipedRoughEnvCfg_PLAY()
            env_id = "Biped-Rough-Play-v0"
        else:
            env_cfg = BipedFlatEnvCfg_PLAY()
            env_id = "Biped-Flat-Play-v0"

    env_cfg.scene.num_envs = args_cli.num_envs
    apply_urdf_selection(env_cfg, args_cli.urdf)

    # Camera setup
    if args_cli.global_camera:
        # Fixed global view — see all envs at once
        env_cfg.viewer.origin_type = "world"
        n = args_cli.num_envs
        spacing = env_cfg.scene.env_spacing if hasattr(env_cfg.scene, 'env_spacing') else 2.5
        grid_size = int(n ** 0.5 + 0.5) * spacing
        env_cfg.viewer.eye = (grid_size * 1.2, grid_size * 1.2, grid_size * 0.8)
        env_cfg.viewer.lookat = (grid_size * 0.3, grid_size * 0.3, 0.0)
    else:
        # Follow single robot
        env_cfg.viewer.origin_type = "asset_root"
        env_cfg.viewer.asset_name = "robot"
        env_cfg.viewer.env_index = args_cli.env_index
        env_cfg.viewer.eye = (3.0, 3.0, 2.0)
        env_cfg.viewer.lookat = (0.0, 0.0, 0.5)

    env = gym.make(
        env_id,
        cfg=env_cfg,
        render_mode="rgb_array" if args_cli.video else None,
    )

    if args_cli.video:
        video_dir = args_cli.video_dir
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
    print(f"[INFO] obs_dim={num_obs}, actions={num_actions}, student={args_cli.student}")

    # Build actor MLP: [128, 128, 128]
    actor_layers = [
        nn.Linear(num_obs, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, num_actions),
    ]
    if args_cli.tanh:
        actor_layers.append(nn.Tanh())
        print("[INFO] Tanh output layer added to actor")
    actor = nn.Sequential(*actor_layers).to("cuda:0")

    # Load weights — different key prefix for student vs teacher
    model_sd = ckpt["model_state_dict"]
    if args_cli.student:
        # Try student.* (distilled) first, fall back to actor.* (fine-tuned)
        actor_sd = {}
        for k, v in model_sd.items():
            if k.startswith("student."):
                actor_sd[k.replace("student.", "")] = v
        if not actor_sd:
            for k, v in model_sd.items():
                if k.startswith("actor."):
                    actor_sd[k.replace("actor.", "")] = v
            print("[INFO] No student.* keys, using actor.* (Phase 3 fine-tuned checkpoint)")
        if not actor_sd:
            raise ValueError(
                f"No student.* or actor.* keys in checkpoint. "
                f"Available prefixes: {set(k.split('.')[0] for k in model_sd.keys())}"
            )
    else:
        # PPO checkpoint: actor.* keys
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
