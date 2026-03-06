"""Debug script to check feet_air_time_positive_biped reward components."""
import argparse
import torch
import sys
import os

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

sys.path.insert(0, '/workspace/biped_locomotion')
from biped_env_cfg import BipedFlatEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg

env_cfg = BipedFlatEnvCfg()
env_cfg.scene.num_envs = 4

env = ManagerBasedRLEnv(cfg=env_cfg)

debug_lines = []

# 1. Check command manager
cmd = env.command_manager.get_command('base_velocity')
debug_lines.append(f'Command shape: {cmd.shape}')
debug_lines.append(f'Command[0]: {cmd[0].cpu().tolist()}')
debug_lines.append(f'Command[:,:2] norm: {torch.norm(cmd[:, :2], dim=1).cpu().tolist()}')
debug_lines.append(f'Command norm > 0.1: {(torch.norm(cmd[:, :2], dim=1) > 0.1).cpu().tolist()}')

# 2. Check contact sensor body_ids for feet
contact_sensor = env.scene.sensors['contact_forces']
debug_lines.append(f'\nContact sensor body_names (all): {contact_sensor.body_names}')
debug_lines.append(f'Contact sensor num_bodies: {contact_sensor.data.net_forces_w.shape}')

foot_cfg = SceneEntityCfg('contact_forces', body_names='foot_6061.*')
foot_cfg.resolve(env.scene)
debug_lines.append(f'\nFoot body_ids resolved: {foot_cfg.body_ids}')
debug_lines.append(f'Foot body_ids type: {type(foot_cfg.body_ids)}')

# Get the matching body names
if hasattr(foot_cfg, 'body_ids') and foot_cfg.body_ids is not None:
    matching_names = [contact_sensor.body_names[i] for i in foot_cfg.body_ids]
    debug_lines.append(f'Matching body names: {matching_names}')

# 3. Run some steps with zero action (robot should fall/stand)
debug_lines.append(f'\n=== Running 100 steps with zero actions ===')
for step in range(100):
    actions = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    obs, rew, term, trunc, info = env.step(actions)
    
    if step % 20 == 0:
        air_time = contact_sensor.data.current_air_time[:, foot_cfg.body_ids]
        contact_time = contact_sensor.data.current_contact_time[:, foot_cfg.body_ids]
        in_contact = contact_time > 0.0
        single_stance = torch.sum(in_contact.int(), dim=1) == 1
        
        in_mode_time = torch.where(in_contact, contact_time, air_time)
        reward_raw = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
        reward_clamp = torch.clamp(reward_raw, max=0.4)
        
        debug_lines.append(f'\nStep {step}:')
        debug_lines.append(f'  air_time[0]: {air_time[0].cpu().tolist()}')
        debug_lines.append(f'  contact_time[0]: {contact_time[0].cpu().tolist()}')
        debug_lines.append(f'  in_contact[0]: {in_contact[0].cpu().tolist()}')
        debug_lines.append(f'  single_stance[0]: {single_stance[0].item()}')
        debug_lines.append(f'  in_mode_time[0]: {in_mode_time[0].cpu().tolist()}')
        debug_lines.append(f'  reward_raw[0]: {reward_raw[0].item():.4f}')
        debug_lines.append(f'  reward_clamp[0]: {reward_clamp[0].item():.4f}')
        debug_lines.append(f'  total_reward: {rew[0].item():.4f}')

# 4. Check if robot is actually standing
root_pos = env.scene['robot'].data.root_pos_w
root_height = root_pos[:, 2]
debug_lines.append(f'\n=== Final state ===')
debug_lines.append(f'Root height: {root_height.cpu().tolist()}')

# 5. Check reward manager individual terms
debug_lines.append(f'\n=== Reward term names ===')
for name in env.reward_manager.active_terms:
    debug_lines.append(f'  {name}')

env.close()

output = '\n'.join(debug_lines)
with open('/results/debug_v37_air_time.txt', 'w') as f:
    f.write(output)
print(output)

simulation_app.close()
