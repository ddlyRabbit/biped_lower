import torch
import sys
sys.path.insert(0, "/home/ubuntu/workspace/biped_locomotion")

from biped_env_cfg import BipedFlatEnvCfg
from isaaclab.envs import ManagerBasedRLEnv

env_cfg = BipedFlatEnvCfg()
env_cfg.scene.num_envs = 2

env = ManagerBasedRLEnv(cfg=env_cfg)

debug_out = []

# Check command manager output
cmd = env.command_manager.get_command("base_velocity")
debug_out.append(f"Command shape: {cmd.shape}")
debug_out.append(f"Command[0]: {cmd[0].cpu().tolist()}")
debug_out.append(f"Command[:, :2] norm: {torch.norm(cmd[:, :2], dim=1).cpu().tolist()}")

# Check contact sensor body_ids
contact_sensor = env.scene.sensors["contact_forces"]
from isaaclab.managers import SceneEntityCfg
foot_cfg = SceneEntityCfg("contact_forces", body_names="foot_6061.*")
foot_cfg.resolve(env)
debug_out.append(f"\nContact sensor foot body_ids: {foot_cfg.body_ids}")
debug_out.append(f"Contact sensor all body names: {contact_sensor.body_names}")

# Check air_time and contact_time
air_time = contact_sensor.data.current_air_time[:, foot_cfg.body_ids]
contact_time = contact_sensor.data.current_contact_time[:, foot_cfg.body_ids]
debug_out.append(f"\nAir time shape: {air_time.shape}")
debug_out.append(f"Air time[0]: {air_time[0].cpu().tolist()}")
debug_out.append(f"Contact time[0]: {contact_time[0].cpu().tolist()}")

# Run a few steps and check
for i in range(50):
    actions = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device)
    env.step(actions)

air_time2 = contact_sensor.data.current_air_time[:, foot_cfg.body_ids]
contact_time2 = contact_sensor.data.current_contact_time[:, foot_cfg.body_ids]
in_contact = contact_time2 > 0.0
in_mode_time = torch.where(in_contact, contact_time2, air_time2)
single_stance = torch.sum(in_contact.int(), dim=1) == 1

debug_out.append(f"\nAfter 50 steps:")
debug_out.append(f"Air time[0]: {air_time2[0].cpu().tolist()}")
debug_out.append(f"Contact time[0]: {contact_time2[0].cpu().tolist()}")
debug_out.append(f"In contact[0]: {in_contact[0].cpu().tolist()}")
debug_out.append(f"Single stance[0]: {single_stance[0].item()}")
debug_out.append(f"In mode time[0]: {in_mode_time[0].cpu().tolist()}")

# Check what the reward function would output
reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
reward_clamped = torch.clamp(reward, max=0.4)
cmd_gate = torch.norm(env.command_manager.get_command("base_velocity")[:, :2], dim=1) > 0.1
debug_out.append(f"\nReward before gate: {reward_clamped.cpu().tolist()}")
debug_out.append(f"Command gate: {cmd_gate.cpu().tolist()}")
debug_out.append(f"Final reward: {(reward_clamped * cmd_gate).cpu().tolist()}")

# Also check how many feet we have
debug_out.append(f"\nNum foot bodies in contact sensor: {len(foot_cfg.body_ids)}")

env.close()

with open("/results/debug_v37.txt", "w") as f:
    f.write("\n".join(debug_out))
print("\n".join(debug_out))
