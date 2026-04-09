import re
with open('sim2sim/stand_mujoco.py', 'r') as f:
    text = f.read()

old_kp = """JOINT_KP = np.array([
    10.0, 30.0, 30.0, 40.0, 80.0, 14.0,
    10.0, 30.0, 30.0, 40.0, 80.0, 14.0,
], dtype=np.float32)"""

new_kp = """JOINT_KP = np.array([
    180.0, 180.0, 180.0, 180.0, 30.0, 30.0,
    180.0, 180.0, 180.0, 180.0, 30.0, 30.0,
], dtype=np.float32)"""

old_kd = """JOINT_KD = np.array([
    2.0, 3.0, 3.0, 3.0, 3.2, 0.8,
    2.0, 3.0, 3.0, 3.0, 3.2, 0.8,
], dtype=np.float32)"""

new_kd = """JOINT_KD = np.array([
    6.5, 6.5, 3.0, 6.5, 1.0, 1.0,
    6.5, 6.5, 3.0, 6.5, 1.0, 1.0,
], dtype=np.float32)"""

text = text.replace(old_kp, new_kp)
text = text.replace(old_kd, new_kd)

with open('sim2sim/stand_mujoco.py', 'w') as f:
    f.write(text)
print("Gains patched.")
