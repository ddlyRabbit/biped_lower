"""Debug script — print body/joint ordering for foot_6061.* and knee joints."""
import sys
sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args(["--headless"])
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
from isaaclab.managers import SceneEntityCfg
from biped_env_cfg import BipedFlatEnvCfg

gym.register(
    id="Isaac-Velocity-Flat-Biped-Debug-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg"},
)

env_cfg = BipedFlatEnvCfg()
env_cfg.scene.num_envs = 2
env = gym.make("Isaac-Velocity-Flat-Biped-Debug-v0", cfg=env_cfg)

robot = env.unwrapped.scene["robot"]
contact = env.unwrapped.scene.sensors["contact_forces"]

# Print ALL body names
print("\n=== ALL BODY NAMES ===")
for i, name in enumerate(robot.data.body_names):
    print(f"  body[{i}] = {name}")

# Print ALL joint names
print("\n=== ALL JOINT NAMES ===")
for i, name in enumerate(robot.data.joint_names):
    print(f"  joint[{i}] = {name}")

# Check foot body resolution
foot_cfg = SceneEntityCfg("robot", body_names="foot_6061.*")
foot_cfg.resolve(env.unwrapped.scene)
print(f"\n=== foot_6061.* body_ids ===")
print(f"  body_ids = {foot_cfg.body_ids}")
for idx in foot_cfg.body_ids:
    print(f"  [{idx}] = {robot.data.body_names[idx]}")

# Check contact sensor body resolution
contact_foot_cfg = SceneEntityCfg("contact_forces", body_names="foot_6061.*")
contact_foot_cfg.resolve(env.unwrapped.scene)
print(f"\n=== contact_forces foot_6061.* body_ids ===")
print(f"  body_ids = {contact_foot_cfg.body_ids}")
for idx in contact_foot_cfg.body_ids:
    bnames = contact.data.body_names if hasattr(contact.data, 'body_names') else None
    if bnames:
        print(f"  [{idx}] = {bnames[idx]}")
    else:
        print(f"  [{idx}] = (no body_names on contact sensor)")

# Check knee joint resolution
knee_cfg = SceneEntityCfg("robot", joint_names=[".*knee.*"])
knee_cfg.resolve(env.unwrapped.scene)
print(f"\n=== .*knee.* joint_ids ===")
print(f"  joint_ids = {knee_cfg.joint_ids}")
for idx in knee_cfg.joint_ids:
    print(f"  [{idx}] = {robot.data.joint_names[idx]}")

# Check hip joint resolution
hip_cfg = SceneEntityCfg("robot", joint_names=[".*hip.*"])
hip_cfg.resolve(env.unwrapped.scene)
print(f"\n=== .*hip.* joint_ids ===")
print(f"  joint_ids = {hip_cfg.joint_ids}")
for idx in hip_cfg.joint_ids:
    print(f"  [{idx}] = {robot.data.joint_names[idx]}")

# Print joint limits for knees and hips
import torch
print(f"\n=== JOINT LIMITS ===")
limits = robot.data.joint_limits  # [num_envs, num_joints, 2]
for idx in list(knee_cfg.joint_ids) + list(hip_cfg.joint_ids):
    lo = limits[0, idx, 0].item()
    hi = limits[0, idx, 1].item()
    print(f"  {robot.data.joint_names[idx]}: [{lo:.4f}, {hi:.4f}]")

# Print default joint positions
print(f"\n=== DEFAULT JOINT POS (env 0) ===")
default = robot.data.default_joint_pos[0]
for i, name in enumerate(robot.data.joint_names):
    print(f"  {name}: {default[i].item():.4f}")

env.close()
simulation_app.close()
