import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

# Add ACTION_ORDER array after ISAAC_JOINTS
action_order_str = """
# ─── Action output order from ONNX (must match training ALL_JOINTS) ─────────
ACTION_ORDER = [
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_foot_pitch", "R_foot_roll",
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_foot_pitch", "L_foot_roll",
]
"""

text = text.replace(']\n\n# ─── MuJoCo → Isaac index mapping ────────────────────────────────────────────', 
                    ']\n' + action_order_str + '\n# ─── MuJoCo → Isaac index mapping ────────────────────────────────────────────')

# Fix action extraction to use ACTION_ORDER
old_targets = """            # Convert to joint targets (Isaac order)
            targets_isaac = np.array([
                DEFAULT_POS_ISAAC[name] + actions_isaac[i] * ACTION_SCALE[i]
                for i, name in enumerate(ISAAC_JOINTS)
            ], dtype=np.float32)"""

new_targets = """            # Convert to joint targets (Isaac order)
            # The network outputs actions in ACTION_ORDER, we need to map them back to ISAAC_JOINTS order
            # so the subsequent MuJoCo mapping targets_mj = targets_isaac[MJ_TO_ISAAC_IDX] works correctly.
            
            # 1. First extract the action value for each joint by name from the network output
            action_by_name = {name: actions_isaac[i] for i, name in enumerate(ACTION_ORDER)}
            
            # 2. Then build the targets_isaac array in the ISAAC_JOINTS order
            targets_isaac = np.array([
                DEFAULT_POS_ISAAC[name] + action_by_name[name] * ACTION_SCALE[i]
                for i, name in enumerate(ISAAC_JOINTS)
            ], dtype=np.float32)"""

text = text.replace(old_targets, new_targets)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)

print("patched")
