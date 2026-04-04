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
parser.add_argument("--cmd_vel_x", type=float, default=None, help="Override forward velocity command (m/s)")
parser.add_argument("--cmd_vel_y", type=float, default=None, help="Override lateral velocity command (m/s)")
parser.add_argument("--cmd_vel_yaw", type=float, default=None, help="Override yaw velocity command (rad/s)")
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

import numpy as np
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

    # Build actor MLP: [512, 256, 128]
    actor_layers = [
        nn.Linear(num_obs, 512), nn.ELU(),
        nn.Linear(512, 256), nn.ELU(),
        nn.Linear(256, 128), nn.ELU(),
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
                clean_k = k.replace("student.", "")
                if False:
                    clean_k = clean_k[2:]
                actor_sd[clean_k] = v
        if not actor_sd:
            for k, v in model_sd.items():
                if k.startswith("actor.0."):
                    clean_k = k.replace("actor.0.", "")
                    if False:
                        clean_k = clean_k[2:]
                    actor_sd[clean_k] = v
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
            if k.startswith("actor.0."):
                clean_k = k.replace("actor.0.", "")
                # Tanh wrapper adds "0." prefix to inner MLP keys (actor.0.X → 0.X)
                if False:
                    clean_k = clean_k[2:]  # strip "0." → flat sequential keys
                actor_sd[clean_k] = v
    actor.load_state_dict(actor_sd)
    actor.eval()
    print("[INFO] Actor loaded successfully")

    # Action logging + joint recording
    all_actions = []
    all_cmd_positions = []  # joint command targets (after action scale)
    all_joint_positions = []  # actual joint positions from sim
    JOINT_NAMES = [
        "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_foot_pitch", "R_foot_roll",
        "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_foot_pitch", "L_foot_roll",
    ]

    # Ankle roll: R_foot_roll=5, L_foot_roll=11 in ALL_JOINTS order
    ANKLE_ROLL_IDX = [5, 11]

    # Get robot asset for joint position readout
    isaac_env = env.unwrapped
    robot = isaac_env.scene["robot"]

    # Get default positions and action scale for computing command targets
    default_pos = robot.data.default_joint_pos[0].cpu().numpy()  # (num_joints,)
    action_scale = 0.5  # base scale
    action_scales = np.full(12, action_scale)
    action_scales[ANKLE_ROLL_IDX] = 0.25  # after the 0.5× multiply below

    # Override velocity commands if specified
    cmd_override = None
    if args_cli.cmd_vel_x is not None or args_cli.cmd_vel_y is not None or args_cli.cmd_vel_yaw is not None:
        vx = args_cli.cmd_vel_x if args_cli.cmd_vel_x is not None else 0.0
        vy = args_cli.cmd_vel_y if args_cli.cmd_vel_y is not None else 0.0
        vyaw = args_cli.cmd_vel_yaw if args_cli.cmd_vel_yaw is not None else 0.0
        cmd_override = torch.tensor([[vx, vy, vyaw]], device="cuda:0").expand(isaac_env.num_envs, -1)
        print(f"[INFO] Command override: vx={vx}, vy={vy}, vyaw={vyaw}")

    timestep = 0
    while simulation_app.is_running():
        # Override commands before policy inference
        if cmd_override is not None:
            isaac_env.command_manager.get_term("base_velocity").vel_command_b[:] = cmd_override

        with torch.no_grad():
            actions = actor(obs).clone()
        # actions[:, ANKLE_ROLL_IDX] *= 0.5  # effective 0.25 (env scale 0.5 × 0.5)
        obs, _, _, _, _ = env.step(actions)

        act_np = actions[0].cpu().numpy()
        all_actions.append(act_np)

        # Command targets: default + action * scale (env applies scale internally,
        # but we record what the env computes)
        cmd_pos = robot.data.joint_pos_target[0].cpu().numpy()
        all_cmd_positions.append(cmd_pos.copy())

        # Actual joint positions
        actual_pos = robot.data.joint_pos[0].cpu().numpy()
        all_joint_positions.append(actual_pos.copy())

        timestep += 1

        if timestep % 50 == 0:
            print(f"[INFO] Step {timestep}")

        if args_cli.video and timestep >= args_cli.video_length:
            print(f"[INFO] Video recording complete ({timestep} steps)")
            break

    # Print action statistics from actual rollout
    if all_actions:
        a = np.array(all_actions)
        print(f"\n[ACT] Action statistics over {len(a)} steps (actual rollout):")
        print(f"{'Joint':<15} {'abs':>6} {'rms':>6} {'min':>6} {'max':>6} {'off(rad)':>8}")
        print("-" * 52)
        for i, name in enumerate(JOINT_NAMES):
            c = a[:, i]
            scale = 0.5  # action_scale
            print(f"{name:<15} {np.mean(np.abs(c)):6.3f} {np.sqrt(np.mean(c**2)):6.3f} "
                  f"{np.min(c):6.3f} {np.max(c):6.3f} {c.mean()*scale:8.3f}")

        # Save to CSV if video dir exists
        if args_cli.video:
            csv_path = os.path.join(args_cli.video_dir, "action_stats.csv")
            with open(csv_path, "w") as f:
                f.write("joint,abs_mean,rms,min,max,mean_offset_rad\n")
                for i, name in enumerate(JOINT_NAMES):
                    c = a[:, i]
                    f.write(f"{name},{np.mean(np.abs(c)):.4f},{np.sqrt(np.mean(c**2)):.4f},"
                            f"{np.min(c):.4f},{np.max(c):.4f},{c.mean()*0.5:.4f}\n")
            print(f"[ACT] Saved to {csv_path}")

    # Save joint commands and actual positions CSV
    if all_cmd_positions and args_cli.video:
        cmd_arr = np.array(all_cmd_positions)   # (T, num_joints)
        pos_arr = np.array(all_joint_positions)  # (T, num_joints)
        joint_names = list(robot.joint_names)
        num_joints = len(joint_names)

        # Joint commands CSV
        cmd_csv = os.path.join(args_cli.video_dir, "joint_commands.csv")
        with open(cmd_csv, "w") as f:
            f.write("step," + ",".join(f"cmd_{n}" for n in joint_names) + "\n")
            for t in range(len(cmd_arr)):
                f.write(f"{t}," + ",".join(f"{cmd_arr[t,j]:.6f}" for j in range(num_joints)) + "\n")
        print(f"[CSV] Joint commands saved to {cmd_csv} ({len(cmd_arr)} steps × {num_joints} joints)")

        # Actual joint positions CSV
        pos_csv = os.path.join(args_cli.video_dir, "joint_positions.csv")
        with open(pos_csv, "w") as f:
            f.write("step," + ",".join(f"pos_{n}" for n in joint_names) + "\n")
            for t in range(len(pos_arr)):
                f.write(f"{t}," + ",".join(f"{pos_arr[t,j]:.6f}" for j in range(num_joints)) + "\n")
        print(f"[CSV] Joint positions saved to {pos_csv} ({len(pos_arr)} steps × {num_joints} joints)")

    env.close()
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
    simulation_app.close()
