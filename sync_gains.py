import re

with open("/home/abhinavroy/biped_lower/biped_env_cfg.py", "r") as f:
    content = f.read()

# Hip Roll
content = re.sub(r'("hip_roll": DelayedPDActuatorCfg\([\s\S]*?stiffness=)180\.0(.*?damping=)6\.5', r'\g<1>1000.0\g<2>40.0', content)
# Hip Yaw
content = re.sub(r'("hip_yaw": DelayedPDActuatorCfg\([\s\S]*?stiffness=)180\.0(.*?damping=)3\.0', r'\g<1>144.0\g<2>3.0', content)
# Hip Pitch
content = re.sub(r'("hip_pitch": DelayedPDActuatorCfg\([\s\S]*?stiffness=)180\.0(.*?damping=)6\.5', r'\g<1>250.0\g<2>6.5', content)
# Knee
content = re.sub(r'("knee": DelayedPDActuatorCfg\([\s\S]*?stiffness=)180\.0(.*?damping=)3\.0', r'\g<1>144.0\g<2>5.0', content)
# Foot Pitch
content = re.sub(r'("foot_pitch": DelayedPDActuatorCfg\([\s\S]*?stiffness=)100\.0(.*?damping=)2\.0', r'\g<1>100.0\g<2>3.0', content)
# Foot Roll
content = re.sub(r'("foot_roll": DelayedPDActuatorCfg\([\s\S]*?stiffness=)100\.0(.*?damping=)2\.0', r'\g<1>100.0\g<2>3.0', content)

with open("/home/abhinavroy/biped_lower/biped_env_cfg.py", "w") as f:
    f.write(content)

# Update MuJoCo script as well
with open("/home/abhinavroy/biped_lower/sim2sim/play_csv_physics_mujoco.py", "r") as f:
    mj = f.read()

kp_new = """def get_kp_mj():
    return np.array([250.0 if "hip_pitch" in name else (1000.0 if "hip_roll" in name else (144.0 if "hip_yaw" in name or "knee" in name else 100.0)) for name in mj_actuator_names], dtype=np.float32)"""
kd_new = """def get_kd_mj():
    return np.array([6.5 if "hip_pitch" in name else (40.0 if "hip_roll" in name else (5.0 if "knee" in name else 3.0)) for name in mj_actuator_names], dtype=np.float32)"""

mj = re.sub(r'def get_kp_mj\(\):.*?dtype=np\.float32\)', kp_new, mj, flags=re.DOTALL)
mj = re.sub(r'def get_kd_mj\(\):.*?dtype=np\.float32\)', kd_new, mj, flags=re.DOTALL)

with open("/home/abhinavroy/biped_lower/sim2sim/play_csv_physics_mujoco.py", "w") as f:
    f.write(mj)

print("Synced gains to control_params.yaml!")
