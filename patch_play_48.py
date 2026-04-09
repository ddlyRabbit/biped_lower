import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

# Change build_observation definition to accept obs_dim
old_def = "def build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action):"
new_def = "def build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action, obs_dim=45):"
text = text.replace(old_def, new_def)

# Update array initialization and offset
old_build = '    obs = np.zeros(45, dtype=np.float32)'
new_build = """    obs = np.zeros(obs_dim, dtype=np.float32)
    offset = 0
    if obs_dim == 48:
        # [0-2] base linear velocity (body frame)
        obs[0:3] = data.qvel[0:3].copy()
        offset = 3"""
text = text.replace(old_build, new_build)

# Shift indices
text = text.replace('obs[0:3] = ang_vel', 'obs[offset:offset+3] = ang_vel')
text = text.replace('obs[3:6] = proj_gravity', 'obs[offset+3:offset+6] = proj_gravity')
text = text.replace('obs[6:9] = cmd_vel', 'obs[offset+6:offset+9] = cmd_vel')
text = text.replace('obs[9 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]', 'obs[offset+9 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]')
text = text.replace('obs[15 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]', 'obs[offset+15 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]')
text = text.replace('obs[17 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]', 'obs[offset+17 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]')
text = text.replace('obs[19 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]', 'obs[offset+19 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]')
text = text.replace('obs[21 + i] = vel_dict[name]', 'obs[offset+21 + i] = vel_dict[name]')
text = text.replace('obs[33:45] = last_action', 'obs[offset+33:offset+45] = last_action')

# Fix the call to build_observation
text = text.replace("obs = build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action)",
                    "obs = build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action, obs_dim=obs_dim)")

# Get obs_dim from model in main
old_main = """    input_name = policy.get_inputs()[0].name
    print(f"Policy: {args.checkpoint}")
    print(f"  Input: {policy.get_inputs()[0].shape}, Output: {policy.get_outputs()[0].shape}")"""

new_main = """    input_name = policy.get_inputs()[0].name
    obs_dim = policy.get_inputs()[0].shape[1]
    print(f"Policy: {args.checkpoint}")
    print(f"  Input: {policy.get_inputs()[0].shape}, Output: {policy.get_outputs()[0].shape}")"""
text = text.replace(old_main, new_main)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)
print("patched")
