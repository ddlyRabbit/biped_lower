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

import numpy as np
import onnxruntime as ort
import mujoco

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MJCF_PATH = os.path.join(REPO_ROOT, "mjcf", "sim2sim", "robot.mjcf")

# ─── Physics ───────────────────────────────────────────────────────────────────
POLICY_DT = 0.02           # 50 Hz

# ─── MuJoCo joint names (actuator order) ──────────────────────────────────────
MJ_JOINTS = [
    "right_hip_pitch_04", "right_hip_roll_03", "right_hip_yaw_03",
    "right_knee_04", "right_foot_pitch_02", "right_foot_roll_02",
    "left_hip_pitch_04", "left_hip_roll_03", "left_hip_yaw_03",
    "left_knee_04", "left_foot_pitch_02", "left_foot_roll_02",
]

# ─── Isaac runtime joint order (policy input/output order) ───────────────────
ISAAC_JOINTS = [
    "L_hip_pitch", "R_hip_pitch", "L_hip_roll", "R_hip_roll",
    "L_hip_yaw", "R_hip_yaw", "L_knee", "R_knee",
    "L_foot_pitch", "R_foot_pitch", "L_foot_roll", "R_foot_roll",
]

# ─── Action output order from ONNX (must match training ALL_JOINTS) ─────────
ACTION_ORDER = [
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch", "R_knee", "R_foot_pitch", "R_foot_roll",
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch", "L_knee", "L_foot_pitch", "L_foot_roll",
]

# ─── MuJoCo → Isaac index mapping ────────────────────────────────────────────
# MJ[i] corresponds to which Isaac joint?
MJ_TO_ISAAC_NAME = {
    "right_hip_pitch_04": "R_hip_pitch",
    "right_hip_roll_03":  "R_hip_roll",
    "right_hip_yaw_03":   "R_hip_yaw",
    "right_knee_04":      "R_knee",
    "right_foot_pitch_02":"R_foot_pitch",
    "right_foot_roll_02": "R_foot_roll",
    "left_hip_pitch_04":  "L_hip_pitch",
    "left_hip_roll_03":   "L_hip_roll",
    "left_hip_yaw_03":    "L_hip_yaw",
    "left_knee_04":       "L_knee",
    "left_foot_pitch_02": "L_foot_pitch",
    "left_foot_roll_02":  "L_foot_roll",
}

# Isaac index for each MuJoCo actuator
MJ_TO_ISAAC_IDX = [ISAAC_JOINTS.index(MJ_TO_ISAAC_NAME[mj]) for mj in MJ_JOINTS]
# Isaac index → MuJoCo actuator index (for converting policy output to ctrl)
ISAAC_TO_MJ_IDX = np.array([MJ_JOINTS.index(n.replace('R_', 'right_').replace('L_', 'left_') + ('_04' if 'knee' in n or ('hip' in n and 'pitch' in n) else '_03' if 'hip' in n else '_02')) for n in ISAAC_JOINTS])

# ─── Observation joint ordering ───────────────────────────────────────────────
# obs[9:21] = joint positions in groups: hip(6), knee(2), foot_pitch(2), foot_roll(2)
OBS_HIP_ORDER = ["L_hip_roll", "R_hip_roll", "L_hip_yaw", "R_hip_yaw", "L_hip_pitch", "R_hip_pitch"]
OBS_KNEE_ORDER = ["L_knee", "R_knee"]
OBS_FOOT_PITCH_ORDER = ["L_foot_pitch", "R_foot_pitch"]
OBS_FOOT_ROLL_ORDER = ["L_foot_roll", "R_foot_roll"]

# ─── Default positions (V58 deployed, matching training) ─────────────────────
DEFAULT_POS_ISAAC = {
    "L_hip_pitch": -0.08, "R_hip_pitch":  0.08,
    "L_hip_roll":   0.0,  "R_hip_roll":   0.0,
    "L_hip_yaw":    0.0,  "R_hip_yaw":    0.0,
    "L_knee":       0.25, "R_knee":       0.25,
    "L_foot_pitch": -0.17,"R_foot_pitch": -0.17,
    "L_foot_roll":  0.0,  "R_foot_roll":  0.0,
}

# Default as array in MuJoCo order
DEFAULT_POS_MJ = np.array([DEFAULT_POS_ISAAC[MJ_TO_ISAAC_NAME[mj]] for mj in MJ_JOINTS], dtype=np.float32)

# ─── Action scaling ──────────────────────────────────────────────────────────
ACTION_SCALE = np.array([
    0.5 if ij != "L_foot_roll" and ij != "R_foot_roll" else 0.25
    for ij in ISAAC_JOINTS
], dtype=np.float32)

# ─── PD gains (from training config) ────────────────────────────────────────
KP_ISAAC = {"L_hip_pitch": 180, "R_hip_pitch": 180, "L_hip_roll": 180, "R_hip_roll": 180,
            "L_hip_yaw": 180, "R_hip_yaw": 180, "L_knee": 180, "R_knee": 180,
            "L_foot_pitch": 30, "R_foot_pitch": 30, "L_foot_roll": 30, "R_foot_roll": 30}
KD_ISAAC = {"L_hip_pitch": 6.5, "R_hip_pitch": 6.5, "L_hip_roll": 6.5, "R_hip_roll": 6.5,
            "L_hip_yaw": 3.0, "R_hip_yaw": 3.0, "L_knee": 6.5, "R_knee": 6.5,
            "L_foot_pitch": 1.0, "R_foot_pitch": 1.0, "L_foot_roll": 1.0, "R_foot_roll": 1.0}

KP_MJ = np.array([KP_ISAAC[MJ_TO_ISAAC_NAME[mj]] for mj in MJ_JOINTS], dtype=np.float32)
KD_MJ = np.array([KD_ISAAC[MJ_TO_ISAAC_NAME[mj]] for mj in MJ_JOINTS], dtype=np.float32)
EFFORT_MJ = np.array([100, 50, 50, 100, 30, 30, 100, 50, 50, 100, 30, 30], dtype=np.float32)

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
    pos_dict = {MJ_TO_ISAAC_NAME[MJ_JOINTS[i]]: joint_pos_mj[i] for i in range(12)}
    vel_dict = {MJ_TO_ISAAC_NAME[MJ_JOINTS[i]]: joint_vel_mj[i] for i in range(12)}

    # [9-14] hip_pos (relative to default)
    for i, name in enumerate(OBS_HIP_ORDER):
        obs[offset+9 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]

    # [15-16] knee_pos
    for i, name in enumerate(OBS_KNEE_ORDER):
        obs[offset+15 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]

    # [17-18] foot_pitch_pos
    for i, name in enumerate(OBS_FOOT_PITCH_ORDER):
        obs[offset+17 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]

    # [19-20] foot_roll_pos
    for i, name in enumerate(OBS_FOOT_ROLL_ORDER):
        obs[offset+19 + i] = pos_dict[name] - DEFAULT_POS_ISAAC[name]

    # [21-32] joint_vel (Isaac runtime order)
    for i, name in enumerate(ISAAC_JOINTS):
        obs[offset+21 + i] = vel_dict[name]

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
    parser.add_argument("--cmd_vx", type=float, default=0.5, help="Forward velocity command")
    parser.add_argument("--cmd_vy", type=float, default=0.0)
    parser.add_argument("--cmd_wz", type=float, default=0.0)
    parser.add_argument("--urdf", type=str, default="heavy", choices=["heavy", "light"])
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
    qp_idx = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in MJ_JOINTS])
    qv_idx = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in MJ_JOINTS])

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
        renderer = mujoco.Renderer(model, 480, 640)
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

    csv_writer = None
    if args.video:
        import csv
        csv_path = args.video.replace('.mp4', '.csv')
        log_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(log_file)
        header = ['time'] + [f'cmd_{j}' for j in ISAAC_JOINTS] + [f'pos_{j}' for j in ISAAC_JOINTS] + [f'act_{j}' for j in ACTION_ORDER]
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
            # The network outputs actions in ACTION_ORDER, we need to map them back to ISAAC_JOINTS order
            # so the subsequent MuJoCo mapping targets_mj = targets_isaac[MJ_TO_ISAAC_IDX] works correctly.
            
            # 1. First extract the action value for each joint by name from the network output
            action_by_name = {name: actions_isaac[i] for i, name in enumerate(ACTION_ORDER)}
            
            # 2. Then build the targets_isaac array in the ISAAC_JOINTS order
            targets_isaac = np.array([
                DEFAULT_POS_ISAAC[name] + action_by_name[name] * ACTION_SCALE[i]
                for i, name in enumerate(ISAAC_JOINTS)
            ], dtype=np.float32)

            # Reorder to MuJoCo order
            targets_mj = targets_isaac[MJ_TO_ISAAC_IDX]

            # Step physics at 2000Hz, computing PD torque every step
            for _ in range(SUBSTEPS):
                jp = data.qpos[qp_idx]
                jv = data.qvel[qv_idx]
                torques = KP_MJ * (targets_mj - jp) + KD_MJ * (0.0 - jv)
                torques = np.clip(torques, -EFFORT_MJ, EFFORT_MJ)
                data.ctrl[:] = torques
                mujoco.mj_step(model, data)

            if csv_writer is not None:
                row = [step * POLICY_DT] + targets_isaac.tolist() + data.qpos[qp_idx][ISAAC_TO_MJ_IDX].tolist() + actions_isaac.tolist()
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

            if step % 50 == 0:
                print(f"[{step * POLICY_DT:.1f}s] z={data.qpos[2]:.3f} qw={data.qpos[3]:.3f} "
                      f"con={data.ncon} act_rms={np.sqrt(np.mean(actions_isaac**2)):.2f}")

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
