"""Debug: write foot ordering to file."""
import sys, os
sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args(["--headless"])
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from isaaclab.managers import SceneEntityCfg
from biped_env_cfg import BipedFlatEnvCfg

gym.register(
    id="Isaac-Velocity-Flat-Biped-Debug4-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg"},
)

env_cfg = BipedFlatEnvCfg()
env_cfg.scene.num_envs = 2
env = gym.make("Isaac-Velocity-Flat-Biped-Debug4-v0", cfg=env_cfg)

robot = env.unwrapped.scene["robot"]
contact = env.unwrapped.scene.sensors["contact_forces"]

out = []

# Resolve each foot separately
right_robot = SceneEntityCfg("robot", body_names=["foot_6061"])
right_robot.resolve(env.unwrapped.scene)
left_robot = SceneEntityCfg("robot", body_names=["foot_6061_2"])
left_robot.resolve(env.unwrapped.scene)
both_robot = SceneEntityCfg("robot", body_names="foot_6061.*")
both_robot.resolve(env.unwrapped.scene)

right_contact = SceneEntityCfg("contact_forces", body_names=["foot_6061"])
right_contact.resolve(env.unwrapped.scene)
left_contact = SceneEntityCfg("contact_forces", body_names=["foot_6061_2"])
left_contact.resolve(env.unwrapped.scene)
both_contact = SceneEntityCfg("contact_forces", body_names="foot_6061.*")
both_contact.resolve(env.unwrapped.scene)

out.append("=== ROBOT BODY IDS ===")
out.append(f"RIGHT foot_6061: {right_robot.body_ids}")
out.append(f"LEFT foot_6061_2: {left_robot.body_ids}")
out.append(f"BOTH foot_6061.*: {both_robot.body_ids}")
for i, bid in enumerate(both_robot.body_ids):
    name = robot.data.body_names[bid]
    out.append(f"  both[{i}] = body_id {bid} = {name}")

out.append("\n=== CONTACT SENSOR BODY IDS ===")
out.append(f"RIGHT foot_6061: {right_contact.body_ids}")
out.append(f"LEFT foot_6061_2: {left_contact.body_ids}")
out.append(f"BOTH foot_6061.*: {both_contact.body_ids}")

out.append("\n=== COMBINED CONTACT MAPPING ===")
for i, bid in enumerate(both_contact.body_ids):
    if bid in right_contact.body_ids:
        out.append(f"  both_contact[{i}] = body_id {bid} = RIGHT foot")
    elif bid in left_contact.body_ids:
        out.append(f"  both_contact[{i}] = body_id {bid} = LEFT foot")
    else:
        out.append(f"  both_contact[{i}] = body_id {bid} = UNKNOWN")

# Run steps and check forces
obs, _ = env.reset()
for step in range(10):
    action = torch.zeros(2, robot.num_joints, device=robot.device)
    obs, rew, term, trunc, info = env.step(action)

forces = contact.data.net_forces_w
out.append(f"\n=== CONTACT FORCES (env 0) ===")
out.append(f"Shape: {forces.shape}")
for name, cfg in [("RIGHT", right_contact), ("LEFT", left_contact)]:
    f = forces[0, cfg.body_ids[0], :]
    out.append(f"{name} body_id={cfg.body_ids[0]}: [{f[0]:.2f}, {f[1]:.2f}, {f[2]:.2f}] norm={f.norm():.2f}")

# Joint defaults and limits for all joints
out.append(f"\n=== JOINT DEFAULTS + LIMITS ===")
defaults = robot.data.default_joint_pos[0]
limits = robot.data.joint_limits[0] if hasattr(robot.data, 'joint_limits') else robot.data.joint_pos_limits[0]
for i, name in enumerate(robot.data.joint_names):
    lo, hi = limits[i, 0].item(), limits[i, 1].item()
    out.append(f"  {name}: default={defaults[i].item():.4f}, limits=[{lo:.4f}, {hi:.4f}]")

# Write to file
with open("/results/debug_output.txt", "w") as f:
    f.write("\n".join(out))

env.close()
simulation_app.close()
