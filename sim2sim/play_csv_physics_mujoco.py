import argparse
import os
import collections

import numpy as np
import mujoco
import mediapy as media

mj_actuator_names = []

MASTER_JOINT_ORDER = [
    "left_hip_pitch", "right_hip_pitch", "left_hip_roll", "right_hip_roll",
    "left_hip_yaw", "right_hip_yaw", "left_knee", "right_knee",
    "left_foot_pitch", "right_foot_pitch", "left_foot_roll", "right_foot_roll"
]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MJCF_PATH = os.path.join(REPO_ROOT, "mjcf", "sim2sim", "robot.mjcf")

POLICY_DT = 0.02           # 50 Hz control rate from CSV
SUBSTEPS = int(POLICY_DT / 0.0005)  # 40 substeps at 2000Hz physics

def get_kp_mj():
    return np.array([250.0 if "hip_pitch" in name else (1000.0 if "hip_roll" in name else (144.0 if "hip_yaw" in name or "knee" in name else 500.0)) for name in mj_actuator_names], dtype=np.float32)

def get_kd_mj():
    return np.array([6.5 if "hip_pitch" in name else (10.0 if "hip_roll" in name else (5.0 if "knee" in name else 3.0)) for name in mj_actuator_names], dtype=np.float32)

def get_friction_mj():
    return np.array([0.5 if "pitch" in name or "knee" in name else (0.375 if "roll" in name or "yaw" in name else 0.25) for name in mj_actuator_names], dtype=np.float32)

CSV_JOINT_ORDER = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_foot_pitch", "left_foot_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_foot_pitch", "right_foot_roll"
]

def load_trajectory(csv_path: str):
    raw = np.genfromtxt(csv_path, delimiter=',')
    timestamps = raw[0]
    joint_angles = raw[1:]  # (12, N)
    
    reordered = np.zeros_like(joint_angles)
    for i, name in enumerate(MASTER_JOINT_ORDER):
        csv_idx = CSV_JOINT_ORDER.index(name)
        reordered[i] = joint_angles[csv_idx]
        
    return timestamps, reordered

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=str, required=True, help="Path to trajectory.csv")
    parser.add_argument("--model", type=str, default=MJCF_PATH)
    parser.add_argument("--video", type=str, default="trajectory_physics.mp4")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    global mj_actuator_names
    mj_actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    data = mujoco.MjData(model)

    timestamps, traj_master_order = load_trajectory(args.csv)
    
    ISAAC_TO_MJ_IDX = []
    for name in MASTER_JOINT_ORDER:
        for i, mj_name in enumerate(mj_actuator_names):
            if mj_name.startswith(name):
                ISAAC_TO_MJ_IDX.append(i)
                break

    qp_idx = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in mj_actuator_names])
    qv_idx = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in mj_actuator_names])
    actuator_idx = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in mj_actuator_names])

    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.80  # Base height
    
    targets_master = traj_master_order[:, 0]
    targets_mj = np.zeros(12, dtype=np.float32)
    for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):
        targets_mj[mj_i] = targets_master[master_i]
    data.qpos[qp_idx] = targets_mj
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, 480, 640)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.distance = 2.0
    camera.elevation = -20
    camera.azimuth = 90
    camera.lookat[:] = [0.0, 0.0, 0.5]

    frames = []
    target_buffer = collections.deque(maxlen=2)
    for _ in range(2):
        target_buffer.append(targets_mj.copy())

    # Fast forward physics WITHOUT rendering for the first 8.5 seconds
    skip_time = 8.5
    render_time = 5.0
    
    print(f"Fast-forwarding physics to {skip_time}s...")
    skip_steps = int(skip_time / POLICY_DT)
    
    for step in range(skip_steps):
        t = step * POLICY_DT
        idx = np.searchsorted(timestamps, t)
        if idx >= len(timestamps): idx = len(timestamps) - 1
            
        targets_master = traj_master_order[:, idx]
        targets_mj = np.zeros(12, dtype=np.float32)
        for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):
            targets_mj[mj_i] = targets_master[master_i]

        for _ in range(SUBSTEPS):
            target_buffer.append(targets_mj.copy())
            delayed_targets_mj = target_buffer[0]
            jp = data.qpos[qp_idx]
            jv = data.qvel[qv_idx]
            torques = get_kp_mj() * (delayed_targets_mj - jp) + get_kd_mj() * (0.0 - jv)
            friction_torque = -get_friction_mj() * np.sign(jv)
            torques = torques + friction_torque
            data.ctrl[actuator_idx] = np.clip(torques, -100.0, 100.0)
            mujoco.mj_step(model, data)
            
    print(f"Rendering physics for {render_time}s...")
    render_steps = int(render_time / POLICY_DT)
    
    for step in range(skip_steps, skip_steps + render_steps):
        t = step * POLICY_DT
        idx = np.searchsorted(timestamps, t)
        if idx >= len(timestamps): idx = len(timestamps) - 1
            
        targets_master = traj_master_order[:, idx]
        targets_mj = np.zeros(12, dtype=np.float32)
        for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):
            targets_mj[mj_i] = targets_master[master_i]

        for _ in range(SUBSTEPS):
            target_buffer.append(targets_mj.copy())
            delayed_targets_mj = target_buffer[0]
            jp = data.qpos[qp_idx]
            jv = data.qvel[qv_idx]
            torques = get_kp_mj() * (delayed_targets_mj - jp) + get_kd_mj() * (0.0 - jv)
            friction_torque = -get_friction_mj() * np.sign(jv)
            torques = torques + friction_torque
            data.ctrl[actuator_idx] = np.clip(torques, -100.0, 100.0)
            mujoco.mj_step(model, data)

        renderer.update_scene(data, camera)
        frames.append(renderer.render().copy())

    if frames:
        fps = int(1.0 / POLICY_DT)
        print(f"Saving {len(frames)} frames to {args.video} at {fps} FPS...")
        media.write_video(args.video, frames, fps=fps)

if __name__ == "__main__":
    main()
