import mujoco.viewer
import mediapy as media
#!/usr/bin/env python3
"""Sim2sim: Run ONNX student policy in MuJoCo.

Usage:
    python sim2sim/play_mujoco.py --checkpoint deploy/student_flat.onnx [--headless] [--duration 10]

Observation spec (45d) matches biped_env_cfg.py / obs_builder.py exactly.
PD control in Python, torques sent to MuJoCo motor actuators.
"""

import argparse
import os
import time
import collections

import numpy as np
import onnxruntime as ort
import mujoco
import random
mj_actuator_names = []

MASTER_JOINT_ORDER = [
    'L_hip_pitch', 'R_hip_pitch', 'L_hip_roll', 'R_hip_roll',
    'L_hip_yaw', 'R_hip_yaw', 'L_knee', 'R_knee',
    'L_foot_pitch', 'R_foot_pitch', 'L_foot_roll', 'R_foot_roll'
]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MJCF_PATH = os.path.join(REPO_ROOT, "mjcf", "sim2sim", "robot.mjcf")

# ─── Physics ───────────────────────────────────────────────────────────────────
POLICY_DT = 0.02           # 50 Hz

# ─── Default positions (V58 deployed, matching training) ─────────────────────
DEFAULT_POS_ISAAC = {
    "L_hip_pitch": 0.2, "R_hip_pitch": -0.2,
    "L_hip_roll": 0.0, "R_hip_roll": 0.0,
    "L_hip_yaw": 0.0, "R_hip_yaw": 0.0,
    "L_knee": 0.4, "R_knee": 0.4,
    "L_foot_pitch": -0.2, "R_foot_pitch": -0.2,
    "L_foot_roll": 0.0, "R_foot_roll": 0.0,
}
def get_default_pos_mj():
    return np.array([DEFAULT_POS_ISAAC.get(mj_name.rsplit('_', 1)[0], 0.0) for mj_name in mj_actuator_names], dtype=np.float32)

# ─── Action scaling ──────────────────────────────────────────────────────────
ACTION_SCALE = np.array([
    0.25 if name.endswith("foot_roll") else (0.3 if name.endswith("foot_pitch") else 0.5)
    for name in MASTER_JOINT_ORDER
], dtype=np.float32)

# ─── PD gains (from training config) ────────────────────────────────────────
def get_kp_mj():
    return np.array([180.0 if "foot" not in name else 60.0 for name in mj_actuator_names], dtype=np.float32)
def get_kd_mj():
    return np.array([6.5 if "hip_pitch" in name or "hip_roll" in name else (3.0 if "hip_yaw" in name or "knee" in name else 2.0) for name in mj_actuator_names], dtype=np.float32)
def get_friction_mj():
    return np.array([0.5 if "pitch" in name or "knee" in name else (0.375 if "roll" in name or "yaw" in name else 0.25) for name in mj_actuator_names], dtype=np.float32)

BASE_HEIGHT = 0.802


def quat_rotate_inverse(q, v):
    """Rotate vector v by inverse of quaternion q (w, x, y, z)."""
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


def build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action, obs_dim=45):
    """Build 45d observation vector matching Isaac training order."""
    obs = np.zeros(obs_dim, dtype=np.float32)
    offset = 0
    # Projected gravity from orientation sensor
    quat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
    quat_adr = model.sensor_adr[quat_id]
    base_quat = data.sensordata[quat_adr:quat_adr + 4].copy()  # w, x, y, z

    if obs_dim == 48:
        # [0-2] base linear velocity (body frame)
        # data.qvel[0:3] is in world frame. We must rotate it to body frame using inverse base_quat
        lin_vel_world = data.qvel[0:3].copy()
        lin_vel_body = quat_rotate_inverse(base_quat, lin_vel_world)
        obs[0:3] = lin_vel_body
        offset = 3

    # Base angular velocity from gyro sensor
    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
    gyro_adr = model.sensor_adr[gyro_id]
    ang_vel = data.sensordata[gyro_adr:gyro_adr + 3].copy()

    # Projected gravity
    gravity_world = np.array([0.0, 0.0, -1.0])
    proj_gravity = quat_rotate_inverse(base_quat, gravity_world)

    # [0-2] base angular velocity
    obs[offset:offset+3] = ang_vel

    # [3-5] projected gravity
    obs[offset+3:offset+6] = proj_gravity

    # [6-8] velocity commands
    obs[offset+6:offset+9] = cmd_vel

    # Joint positions (relative to default) — need Isaac name mapping
    joint_pos_mj = data.qpos[qp_idx]
    joint_vel_mj = data.qvel[qv_idx]

    # Build name→value dicts
    # Build pos and vel mapping by dropping Mujoco CAD suffixes
    pos_dict = {mj_name.rsplit('_', 1)[0]: pos for mj_name, pos in zip(mj_actuator_names, joint_pos_mj)}
    vel_dict = {mj_name.rsplit('_', 1)[0]: vel for mj_name, vel in zip(mj_actuator_names, joint_vel_mj)}

    # [9-20] Joint pos
    for i, name in enumerate(MASTER_JOINT_ORDER):
        obs[offset + 9 + i] = pos_dict.get(name, 0.0) - DEFAULT_POS_ISAAC.get(name, 0.0)
        
    # [21-32] Joint vel
    for i, name in enumerate(MASTER_JOINT_ORDER):
        obs[offset + 21 + i] = vel_dict.get(name, 0.0)

    # [33-44] last_action (Isaac order)
    obs[offset+33:offset+45] = last_action

    return obs


def main():
    playing = True
    parser = argparse.ArgumentParser(description="Play ONNX policy in MuJoCo")
    parser.add_argument("--checkpoint", type=str, default=os.path.join(REPO_ROOT, "deploy", "student_flat.onnx"))
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=None, help="Playback duration in seconds (default: 10s for video, inf for interactive)")
    parser.add_argument("--video", type=str, default=None, help="Save video to path")
    parser.add_argument("--cmd_vx", type=float, default=0.0, help="Forward velocity command")
    parser.add_argument("--cmd_vy", type=float, default=0.0)
    parser.add_argument("--cmd_wz", type=float, default=0.0)
    parser.add_argument("--urdf", type=str, default="heavy", choices=["heavy", "light"])
    parser.add_argument("--latency_ms", type=float, default=0.0, help="Artificial hardware latency in milliseconds")
    args = parser.parse_args()

    # Load ONNX model
    policy = ort.InferenceSession(args.checkpoint)
    input_name = policy.get_inputs()[0].name
    obs_dim = policy.get_inputs()[0].shape[1]
    print(f"Policy: {args.checkpoint}")
    print(f"  Input: {policy.get_inputs()[0].shape}, Output: {policy.get_outputs()[0].shape}")

    # Load MuJoCo model
    mjcf = MJCF_PATH.replace("robot.mjcf", f"robot_{args.urdf}.mjcf")
    print(f"[INFO] Using MJCF: {mjcf}")
    model = mujoco.MjModel.from_xml_path(mjcf)
    data = mujoco.MjData(model)

    PHYSICS_DT = model.opt.timestep
    SUBSTEPS = int(round(POLICY_DT / PHYSICS_DT))
    qp_idx = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in mj_actuator_names])
    qv_idx = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in mj_actuator_names])
    actuator_idx = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in mj_actuator_names])

    # Check sensors exist
    for sname in ["angular-velocity", "orientation"]:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
        assert sid >= 0, f"Sensor '{sname}' not found in MJCF"

    # Reset
    mujoco.mj_resetData(model, data)
    data.qpos[2] = BASE_HEIGHT
    data.qpos[qp_idx] = DEFAULT_POS_MJ
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    cmd_vel = np.array([args.cmd_vx, args.cmd_vy, args.cmd_wz], dtype=np.float32)
    last_action = np.zeros(12, dtype=np.float32)

    def key_callback(keycode):
        nonlocal playing
        if keycode == 265:  # Up arrow
            cmd_vel[0] += 0.1
        elif keycode == 264:  # Down arrow
            cmd_vel[0] -= 0.1
        elif keycode == 263:  # Left arrow
            cmd_vel[1] += 0.1
        elif keycode == 262:  # Right arrow
            cmd_vel[1] -= 0.1
        elif keycode == 65:  # A (yaw left)
            cmd_vel[2] += 0.2
        elif keycode == 68:  # D (yaw right)
            cmd_vel[2] -= 0.2
        elif keycode == 32:  # Space
            cmd_vel[:] = 0.0
        elif keycode in (81, 256):  # Q or ESC
            playing = False
        
        # Clamp commands to training ranges
        cmd_vel[0] = np.clip(cmd_vel[0], -0.5, 1.5)
        cmd_vel[1] = np.clip(cmd_vel[1], -0.5, 0.5)
        cmd_vel[2] = np.clip(cmd_vel[2], -1.0, 1.0)
        print(f"Command update: vx={cmd_vel[0]:.2f} vy={cmd_vel[1]:.2f} wz={cmd_vel[2]:.2f}")

    print(f"Commands: vx={cmd_vel[0]:.2f} vy={cmd_vel[1]:.2f} wz={cmd_vel[2]:.2f}")
    print(f"Physics: {1/PHYSICS_DT:.0f}Hz ({SUBSTEPS} substeps), Policy: {1/POLICY_DT:.0f}Hz")

    renderer = None
    frames = []
    if args.video:
        pass
        print("Starting renderer"); renderer = mujoco.Renderer(model, 480, 640); print("Renderer started")
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        camera.trackbodyid = 0
        camera.distance = 1.5
        camera.azimuth = 90
        camera.elevation = -10
        camera.lookat[:] = [0, 0, 0.4]
    
    if not args.headless and not args.video:
        pass
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=key_callback)
    else:
        viewer = None

    if args.duration is None:
        args.duration = 10.0 if args.video else float('inf')

    # Action delay: match training DelayedPDActuator min_delay=0, max_delay=6 steps (at 50Hz)
    # 6 policy steps = 6 * 40 physics steps = 240 physics steps max
    # Use default 3 policy steps of delay (midpoint)
    if args.latency_ms > 0:
        latency_steps = int(args.latency_ms / 1000.0 / PHYSICS_DT)
    else:
        # Default: 3 policy steps of delay (matches midpoint of training 0-6)
        latency_steps = 3 * SUBSTEPS
    target_buffer = collections.deque([DEFAULT_POS_MJ.copy() for _ in range(latency_steps + 1)], maxlen=latency_steps + 1)

    csv_writer = None
    if args.video:
        import csv
        csv_path = args.video.replace('.mp4', '.csv')
        log_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(log_file)
        header = ['time'] + [f'cmd_{j}' for j in MASTER_JOINT_ORDER] + [f'pos_{j}' for j in MASTER_JOINT_ORDER] + [f'act_{j}' for j in MASTER_JOINT_ORDER]
        csv_writer.writerow(header)

    step = 0
    try:
        while playing and (args.duration == float('inf') or step < int(args.duration / POLICY_DT)):
            t0 = time.perf_counter()

            # Build observation
            obs = build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action, obs_dim=obs_dim)

            # Run policy
            actions_isaac = policy.run(None, {input_name: obs.reshape(1, -1)})[0][0]
            actions_isaac = np.clip(actions_isaac, -10.0, 10.0)
            last_action = actions_isaac.copy()

            # Convert to joint targets (Isaac order)
            # Network outputs action directly in MASTER_JOINT_ORDER
            actions_isaac = action
            
            # Map action directly to MuJoCo targets using default position and scale
            targets_isaac = np.zeros(12, dtype=np.float32)
            for i, name in enumerate(MASTER_JOINT_ORDER):
                targets_isaac[i] = DEFAULT_POS_ISAAC[name] + actions_isaac[i] * ACTION_SCALE[i]
                
            # Reorder to MuJoCo order dynamically
            ISAAC_TO_MJ_IDX = []
            for name in MASTER_JOINT_ORDER:
                for i, mj_name in enumerate(mj_actuator_names):
                    if mj_name.startswith(name):
                        ISAAC_TO_MJ_IDX.append(i)
                        break
                        
            # Map the 12 targets to their respective MuJoCo actuator indices
            # Targets is size 12. ISAAC_TO_MJ_IDX maps from master order idx to mujoco actuator index.
            # Wait, targets_mj needs to be in MuJoCo actuator order.
            targets_mj = np.zeros(12, dtype=np.float32)
            for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):
                targets_mj[mj_i] = targets_isaac[master_i]
            
            # Step physics at 2000Hz
            for _ in range(SUBSTEPS):
                target_buffer.append(targets_mj.copy())
                delayed_targets_mj = target_buffer[0]
                
                jp = data.qpos[qp_idx]
                jv = data.qvel[qv_idx]
                torques = get_kp_mj() * (delayed_targets_mj - jp) + get_kd_mj() * (0.0 - jv)
                friction_torque = -get_friction_mj() * np.sign(jv)
                torques = torques + friction_torque
                torques = np.clip(torques, -100.0, 100.0)
                data.ctrl[actuator_idx] = torques
                mujoco.mj_step(model, data)

            if csv_writer is not None:
                row = [step * POLICY_DT] + targets_isaac.tolist() + [data.qpos[qp_idx][i] for i in ISAAC_TO_MJ_IDX] + actions_isaac.tolist()
                csv_writer.writerow(row)

            if renderer is not None:
                renderer.update_scene(data, camera)
                frames.append(renderer.render().copy())

            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    playing = False

            dt = time.perf_counter() - t0
            step += 1
            if POLICY_DT - dt > 0:
                time.sleep(POLICY_DT - dt)
            time.sleep(random.uniform(0,40)/10000.0)

            if step % 50 == 0:
                print(f"[{step * POLICY_DT:.1f}s] z={data.qpos[2]:.3f} qw={data.qpos[3]:.3f} "
                      f"con={data.ncon} act_rms={np.sqrt(np.mean(actions_isaac**2)):.2f}")
                print(f"Gravity: {obs}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if viewer is not None:
            viewer.close()
        if renderer is not None and frames:
            pass
            fps = int(1.0 / POLICY_DT)
            print(f"Saving {len(frames)} frames to {args.video} at {fps} FPS...")
            media.write_video(args.video, frames, fps=fps)

if __name__ == "__main__":
    main()
