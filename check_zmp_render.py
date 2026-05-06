import numpy as np
import mujoco
import collections

model = mujoco.MjModel.from_xml_path("mjcf/sim2sim/robot_light.mjcf")
data = mujoco.MjData(model)

mj_actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]

MASTER_JOINT_ORDER = [
    "left_hip_pitch", "right_hip_pitch", "left_hip_roll", "right_hip_roll",
    "left_hip_yaw", "right_hip_yaw", "left_knee", "right_knee",
    "left_foot_pitch", "right_foot_pitch", "left_foot_roll", "right_foot_roll"
]

ISAAC_TO_MJ_IDX = []
for name in MASTER_JOINT_ORDER:
    for i, mj_name in enumerate(mj_actuator_names):
        if mj_name.startswith(name):
            ISAAC_TO_MJ_IDX.append(i)
            break

qp_idx = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in mj_actuator_names])
qv_idx = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in mj_actuator_names])
actuator_idx = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in mj_actuator_names])

def get_kp_mj():
    return np.array([250.0 if "hip_pitch" in name else (300.0 if "hip_roll" in name else (144.0 if "hip_yaw" in name or "knee" in name else 250.0)) for name in mj_actuator_names], dtype=np.float32)

def get_kd_mj():
    return np.array([6.5 if "hip_pitch" in name else (5.0 if "hip_roll" in name else (5.0 if "knee" in name else 3.0)) for name in mj_actuator_names], dtype=np.float32)

def get_friction_mj():
    return np.array([0.5 if "pitch" in name or "knee" in name else (0.375 if "roll" in name or "yaw" in name else 0.25) for name in mj_actuator_names], dtype=np.float32)

raw = np.genfromtxt('deploy/biped_ws/src/biped_bringup/config/trajectory.csv', delimiter=',')
timestamps = raw[0]
joint_angles = raw[1:]

CSV_JOINT_ORDER = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_foot_pitch", "left_foot_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_foot_pitch", "right_foot_roll"
]

reordered = np.zeros_like(joint_angles)
for i, name in enumerate(MASTER_JOINT_ORDER):
    csv_idx = CSV_JOINT_ORDER.index(name)
    reordered[i] = joint_angles[csv_idx]

mujoco.mj_resetData(model, data)
data.qpos[2] = 0.80

skip_steps = int(8.5 / 0.02)
render_steps = int(5.0 / 0.02)
target_buffer = collections.deque(maxlen=2)
for _ in range(2): target_buffer.append(np.zeros(12))

for step in range(skip_steps):
    t = step * 0.02
    idx = np.searchsorted(timestamps, t)
    if idx >= len(timestamps): idx = len(timestamps) - 1
    targets_master = reordered[:, idx]
    targets_mj = np.zeros(12, dtype=np.float32)
    for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):
        targets_mj[mj_i] = targets_master[master_i]

    for _ in range(40):
        target_buffer.append(targets_mj.copy())
        delayed_targets_mj = target_buffer[0]
        jp = data.qpos[qp_idx]
        jv = data.qvel[qv_idx]
        torques = get_kp_mj() * (delayed_targets_mj - jp) + get_kd_mj() * (0.0 - jv)
        friction_torque = -get_friction_mj() * np.sign(jv)
        data.ctrl[actuator_idx] = np.clip(torques + friction_torque, -100.0, 100.0)
        mujoco.mj_step(model, data)

# Now collect actual positions during render phase
recorded_pos = []
for step in range(skip_steps, skip_steps + render_steps):
    t = step * 0.02
    idx = np.searchsorted(timestamps, t)
    if idx >= len(timestamps): idx = len(timestamps) - 1
    targets_master = reordered[:, idx]
    targets_mj = np.zeros(12, dtype=np.float32)
    for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):
        targets_mj[mj_i] = targets_master[master_i]

    for _ in range(40):
        target_buffer.append(targets_mj.copy())
        delayed_targets_mj = target_buffer[0]
        jp = data.qpos[qp_idx]
        jv = data.qvel[qv_idx]
        torques = get_kp_mj() * (delayed_targets_mj - jp) + get_kd_mj() * (0.0 - jv)
        friction_torque = -get_friction_mj() * np.sign(jv)
        data.ctrl[actuator_idx] = np.clip(torques + friction_torque, -100.0, 100.0)
        mujoco.mj_step(model, data)
        
    recorded_pos.append(data.qpos[qp_idx].copy())
    
recorded_pos = np.array(recorded_pos)
print("Standard deviation of ACTUAL physics joint positions during render window:")
print(np.std(recorded_pos, axis=0))
