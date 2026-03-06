"""Debug: resolve left/right feet SEPARATELY in contact sensor."""
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
    id="Isaac-Velocity-Flat-Biped-Debug2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg"},
)

env_cfg = BipedFlatEnvCfg()
env_cfg.scene.num_envs = 2
env = gym.make("Isaac-Velocity-Flat-Biped-Debug2-v0", cfg=env_cfg)

robot = env.unwrapped.scene["robot"]
contact = env.unwrapped.scene.sensors["contact_forces"]

# Resolve each foot separately in BOTH robot and contact sensor
print("\n=== SEPARATE FOOT RESOLUTION ===")

# Robot body resolution
right_foot_robot = SceneEntityCfg("robot", body_names=["foot_6061"])
right_foot_robot.resolve(env.unwrapped.scene)
print(f"Robot foot_6061 (RIGHT): body_ids = {right_foot_robot.body_ids}")

left_foot_robot = SceneEntityCfg("robot", body_names=["foot_6061_2"])
left_foot_robot.resolve(env.unwrapped.scene)
print(f"Robot foot_6061_2 (LEFT): body_ids = {left_foot_robot.body_ids}")

both_robot = SceneEntityCfg("robot", body_names="foot_6061.*")
both_robot.resolve(env.unwrapped.scene)
print(f"Robot foot_6061.* (BOTH): body_ids = {both_robot.body_ids}")

# Contact sensor resolution
right_foot_contact = SceneEntityCfg("contact_forces", body_names=["foot_6061"])
right_foot_contact.resolve(env.unwrapped.scene)
print(f"\nContact foot_6061 (RIGHT): body_ids = {right_foot_contact.body_ids}")

left_foot_contact = SceneEntityCfg("contact_forces", body_names=["foot_6061_2"])
left_foot_contact.resolve(env.unwrapped.scene)
print(f"Contact foot_6061_2 (LEFT): body_ids = {left_foot_contact.body_ids}")

both_contact = SceneEntityCfg("contact_forces", body_names="foot_6061.*")
both_contact.resolve(env.unwrapped.scene)
print(f"Contact foot_6061.* (BOTH): body_ids = {both_contact.body_ids}")

# Also check: does the contact sensor have body_names we can list?
print(f"\n=== CONTACT SENSOR INFO ===")
print(f"Contact sensor type: {type(contact)}")
print(f"Contact data type: {type(contact.data)}")
# Try various attributes
for attr in ['body_names', 'body_physx_view', '_body_names', 'num_bodies']:
    val = getattr(contact.data, attr, getattr(contact, attr, 'NOT_FOUND'))
    if val != 'NOT_FOUND':
        print(f"  {attr} = {val}")

# Check if contact body_names are accessible via cfg
print(f"\n  contact.cfg.prim_path = {contact.cfg.prim_path}")

# Run 10 steps and print contact forces per foot
import torch
print(f"\n=== CONTACT FORCES AFTER 10 STEPS ===")
obs, _ = env.reset()
for step in range(10):
    action = torch.zeros(2, robot.num_joints, device=robot.device)
    obs, rew, term, trunc, info = env.step(action)

forces = contact.data.net_forces_w  # [num_envs, num_bodies, 3]
print(f"Contact forces shape: {forces.shape}")
# Show forces at the specific body_ids we resolved
for name, cfg in [("RIGHT (foot_6061)", right_foot_contact), ("LEFT (foot_6061_2)", left_foot_contact)]:
    f = forces[0, cfg.body_ids[0], :]
    print(f"  {name} body_id={cfg.body_ids[0]}: force = [{f[0]:.2f}, {f[1]:.2f}, {f[2]:.2f}], norm = {f.norm():.2f}")

# Now check: in the combined body_ids [6, 12], which index is which?
print(f"\n=== COMBINED body_ids MAPPING ===")
combined_ids = both_contact.body_ids
for i, bid in enumerate(combined_ids):
    if bid == right_foot_contact.body_ids[0]:
        print(f"  combined[{i}] = body_id {bid} = RIGHT foot")
    elif bid == left_foot_contact.body_ids[0]:
        print(f"  combined[{i}] = body_id {bid} = LEFT foot")
    else:
        print(f"  combined[{i}] = body_id {bid} = UNKNOWN")

env.close()
simulation_app.close()
