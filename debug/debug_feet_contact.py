"""Debug feet_air_time for V52 — run with trained policy, print contact info."""
import argparse
import sys
sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--checkpoint", type=str, required=True)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import torch.nn as nn
import gymnasium as gym
from isaaclab_rl.skrl import SkrlVecEnvWrapper
from biped_env_cfg import BipedFlatEnvCfg_PLAY

gym.register(
    id="Biped-Flat-Debug-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg_PLAY"},
)

env_cfg = BipedFlatEnvCfg_PLAY()
env_cfg.scene.num_envs = args_cli.num_envs
raw_env = gym.make("Biped-Flat-Debug-v0", cfg=env_cfg)

# Get the inner ManagerBasedRLEnv
inner_env = raw_env.unwrapped

env = SkrlVecEnvWrapper(raw_env, ml_framework="torch")
obs, _ = env.reset()

# Load actor
ckpt = torch.load(args_cli.checkpoint, map_location="cuda:0")
num_obs = obs.shape[-1]
num_actions = env.action_space.shape[-1]

actor = nn.Sequential(
    nn.Linear(num_obs, 128), nn.ELU(),
    nn.Linear(128, 128), nn.ELU(),
    nn.Linear(128, 128), nn.ELU(),
    nn.Linear(128, num_actions),
).to("cuda:0")
sd = {k.replace("actor.", ""): v for k, v in ckpt["model_state_dict"].items() if k.startswith("actor.")}
actor.load_state_dict(sd)
actor.eval()

# Get contact sensor
contact_sensor = inner_env.scene.sensors["contact_forces"]

# Find foot body IDs
all_bodies = inner_env.scene["robot"].body_names
print(f"\nAll body names: {all_bodies}")
print(f"Num bodies: {len(all_bodies)}")

# Check sensor body names
print(f"\nContact sensor body_names: {contact_sensor.body_names}")
print(f"Contact sensor body_ids: {contact_sensor.cfg.body_ids if hasattr(contact_sensor.cfg, 'body_ids') else 'N/A'}")
print(f"Contact sensor num_bodies: {contact_sensor.data.net_forces_w.shape}")

# Get the body_ids that "foot_6061.*" resolves to
from isaaclab.managers import SceneEntityCfg
foot_cfg = SceneEntityCfg("contact_forces", body_names="foot_6061.*")
foot_cfg.resolve(inner_env.scene)
print(f"\nFoot body_ids (resolved): {foot_cfg.body_ids}")
foot_body_names = [contact_sensor.body_names[i] for i in foot_cfg.body_ids]
print(f"Foot body names: {foot_body_names}")

print(f"\nstep_dt: {inner_env.step_dt}")
print(f"physics_dt: {inner_env.physics_dt}")

# Run for 500 steps, collecting debug info
total_first_contacts = 0
total_air_time_sum = 0
total_steps = 0

for step in range(500):
    with torch.inference_mode():
        actions = actor(obs)
    obs, _, _, _, _ = env.step(actions)
    
    # Contact debug
    first_contact = contact_sensor.compute_first_contact(inner_env.step_dt)[:, foot_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, foot_cfg.body_ids]
    net_forces = contact_sensor.data.net_forces_w[:, foot_cfg.body_ids, :]
    force_norms = net_forces.norm(dim=-1)
    
    # Is in contact?
    in_contact = (force_norms > 1.0).float()
    
    n_first_contact = first_contact.sum().item()
    total_first_contacts += n_first_contact
    total_steps += 1
    
    if step % 50 == 0:
        # Root height
        root_pos = inner_env.scene["robot"].data.root_pos_w
        root_vel = inner_env.scene["robot"].data.root_lin_vel_w
        
        # Joint torques
        joint_torques = inner_env.scene["robot"].data.applied_torque
        torque_norms = joint_torques.abs().mean(dim=0)
        
        # Foot velocities
        foot_asset_cfg = SceneEntityCfg("robot", body_names="foot_6061.*")
        foot_asset_cfg.resolve(inner_env.scene)
        body_vel = inner_env.scene["robot"].data.body_lin_vel_w[:, foot_asset_cfg.body_ids, :]
        foot_speed = body_vel.norm(dim=-1).mean(dim=0)
        
        print(f"\n--- Step {step} ---")
        print(f"Root height: {root_pos[:, 2].mean().item():.4f}")
        print(f"Root vel XY: [{root_vel[:, 0].mean().item():.4f}, {root_vel[:, 1].mean().item():.4f}]")
        print(f"First contacts this step: {n_first_contact:.0f} / {first_contact.numel()} possible")
        print(f"Cumulative first contacts: {total_first_contacts:.0f} in {total_steps} steps")
        print(f"In contact (feet): {in_contact.mean(dim=0).cpu().numpy()}")
        print(f"Contact forces: {force_norms.mean(dim=0).cpu().numpy()}")
        print(f"Last air time: {last_air_time.mean(dim=0).cpu().numpy()}")
        print(f"Foot speeds: {foot_speed.cpu().numpy()}")
        print(f"Mean joint torques: {torque_norms.cpu().numpy()}")

        # Air time reward calc
        air_time = (last_air_time - 0.2) * first_contact
        air_time_clamped = torch.clamp(air_time, max=0.3)  # 0.5-0.2
        reward = torch.sum(air_time_clamped, dim=1)
        cmd = inner_env.command_manager.get_command("base_velocity")[:, :2]
        cmd_norm = torch.norm(cmd, dim=1)
        reward *= (cmd_norm > 0.1).float()
        print(f"feet_air_time reward: {reward.mean().item():.6f}")
        print(f"Command norm: {cmd_norm.mean().item():.4f}, non-zero: {(cmd_norm > 0.1).float().mean().item():.2f}")

print(f"\n=== SUMMARY ===")
print(f"Total first contacts: {total_first_contacts:.0f} in {total_steps} steps")
print(f"Rate: {total_first_contacts / total_steps:.2f} per step")

raw_env.close()
