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

# Mapping from Isaac joint names (L_/R_) to MuJoCo actuator names (left_/right_)
# Built dynamically after model load. Key: Isaac name, Value: MuJoCo actuator index.
ISAAC_TO_MJ_MAP = {}  # populated in main() after mj_actuator_names is set

def build_isaac_to_mj_mapping():
    """Build bidirectional mapping between Isaac and MuJoCo joint names."""
    mapping = {}
    for isaac_name in MASTER_JOINT_ORDER:
        # L_hip_pitch -> left_hip_pitch_04
        side = 'left' if isaac_name.startswith('L_') else 'right'
        joint = isaac_name[2:]  # hip_pitch, hip_roll, etc.
        for mj_i, mj_name in enumerate(mj_actuator_names):
            # Match: left_hip_pitch_04 contains 'left' and 'hip_pitch'
            if mj_name.startswith(side) and joint in mj_name:
                mapping[isaac_name] = mj_i
                break
    return mapping

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MJCF_PATH = os.path.join(REPO_ROOT, "mjcf", "sim2sim", "robot.mjcf")

# ─── Physics ───────────────────────────────────────────────────────────────────
POLICY_DT = 0.02           # 50 Hz

# ─── Default positions (V58 deployed, matching training) ─────────────────────
DEFAULT_POS_ISAAC = {
    "L_hip_pitch": -0.08, "R_hip_pitch": 0.08,
    "L_hip_roll": 0.0, "R_hip_roll": 0.0,
    "L_hip_yaw": 0.0, "R_hip_yaw": 0.0,
    "L_knee": 0.25, "R_knee": 0.25,
    "L_foot_pitch": -0.17, "R_foot_pitch": -0.17,
    "L_foot_roll": 0.0, "R_foot_roll": 0.0,
}
def get_default_pos_mj():
    """Return default positions in MuJoCo actuator order."""
    result = np.zeros(len(mj_actuator_names), dtype=np.float32)
    for isaac_name, mj_i in ISAAC_TO_MJ_MAP.items():
        result[mj_i] = DEFAULT_POS_ISAAC[isaac_name]
    return result

# ─── Action scaling ──────────────────────────────────────────────────────────
ACTION_SCALE = np.array([
    0.25 if name.endswith("foot_roll") else 0.5
    for name in MASTER_JOINT_ORDER
], dtype=np.float32)

# ─── PD gains (from training config) ────────────────────────────────────────
def get_kp_mj():
    return np.array([180.0 if "foot" not in name else 30.0 for name in mj_actuator_names], dtype=np.float32)

def get_kd_mj():
    kd = []
    for name in mj_actuator_names:
        if "foot" in name:
            kd.append(2.8)
        elif "knee" in name:
            kd.append(10.0)
        elif "hip_pitch" in name:
            kd.append(20.0)
        elif "hip_roll" in name or "hip_yaw" in name:
            kd.append(15.0)
        else:
            kd.append(3.0)
    return np.array(kd, dtype=np.float32)

def get_friction_mj():
    return np.array([0.1 for _ in mj_actuator_names], dtype=np.float32)

def get_effort_mj():
    eff = []
    for name in mj_actuator_names:
        if "foot" in name:
            eff.append(30.0)
        elif "hip_roll" in name or "hip_yaw" in name:
            eff.append(50.0)
        else:
            eff.append(100.0)
    return np.array(eff, dtype=np.float32)


BASE_HEIGHT = 0.802


def quat_rotate_inverse(q, v):
    """Rotate vector v by inverse of quaternion q (w, x, y, z)."""
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


def build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action, imu_data=None, proprio_data=None, obs_dim=45):
    """Build observation vector matching Isaac training order."""
    obs = np.zeros(obs_dim, dtype=np.float32)
    offset = 0

    if imu_data is not None:
        base_quat, ang_vel, lin_vel_world = imu_data
    else:
        quat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
        quat_adr = model.sensor_adr[quat_id]
        base_quat = data.sensordata[quat_adr:quat_adr + 4].copy()

        gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
        gyro_adr = model.sensor_adr[gyro_id]
        ang_vel = data.sensordata[gyro_adr:gyro_adr + 3].copy()

        lin_vel_world = data.qvel[0:3].copy()

    if obs_dim == 48:
        # [0-2] base linear velocity (body frame)
        lin_vel_body = quat_rotate_inverse(base_quat, lin_vel_world)
        obs[0:3] = lin_vel_body
        offset = 3

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
    if proprio_data is not None:
        joint_pos_mj, joint_vel_mj = proprio_data
    else:
        joint_pos_mj = data.qpos[qp_idx].copy()
        joint_vel_mj = data.qvel[qv_idx].copy()

    # Build name→value dicts
    # Build pos and vel mapping by dropping Mujoco CAD suffixes
    pos_dict = {isaac_name: joint_pos_mj[mj_i] for isaac_name, mj_i in ISAAC_TO_MJ_MAP.items()}
    vel_dict = {isaac_name: joint_vel_mj[mj_i] for isaac_name, mj_i in ISAAC_TO_MJ_MAP.items()}

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
    parser.add_argument("--imu_latency_ms", type=float, default=0.0, help="IMU latency in ms")
    parser.add_argument("--push_time", type=float, default=-1.0, help="Time to apply push (s)")
    parser.add_argument("--push_duration", type=float, default=0.2, help="Duration of push (s)")
    parser.add_argument("--push_force", type=float, nargs=3, default=[0.0, 0.0, 0.0], help="Push force vector [Fx, Fy, Fz] in Newtons")
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

    global mj_actuator_names, ISAAC_TO_MJ_MAP
    mj_actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    print(f"[INFO] Actuators: {mj_actuator_names}")

    # Set accurate armature (rotor inertia) matching training config
    for i, name in enumerate(mj_actuator_names):
        dof_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if "foot" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.1
        elif "knee" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.3
        elif "hip_pitch" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.025
        elif "hip_roll" in name or "hip_yaw" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.01

    # Set accurate armature (rotor inertia) matching training config
    for i, name in enumerate(mj_actuator_names):
        dof_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if "foot" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.1
        elif "knee" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.3
        elif "hip_pitch" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.025
        elif "hip_roll" in name or "hip_yaw" in name:
            model.dof_armature[model.jnt_dofadr[dof_id]] = 0.01

    print("\n[INFO] Validating applied properties (Actuator Order):")
    kd_vals = get_kd_mj()
    kp_vals = get_kp_mj()
    fric_vals = get_friction_mj()
    eff_vals = get_effort_mj()
    for i, name in enumerate(mj_actuator_names):
        dof_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        arm_val = model.dof_armature[model.jnt_dofadr[dof_id]]
        print(f"{name:20s} | Arm: {arm_val:.3f} | KP: {kp_vals[i]:.0f} | KD: {kd_vals[i]:.1f} | Fric: {fric_vals[i]:.2f} | MaxTorque: {eff_vals[i]:.0f}")
    print("\n")

    ISAAC_TO_MJ_MAP = build_isaac_to_mj_mapping()
    print(f"[INFO] Isaac->MuJoCo mapping:")
    for isaac_name, mj_i in sorted(ISAAC_TO_MJ_MAP.items(), key=lambda x: x[1]):
        print(f"  {isaac_name:16s} -> [{mj_i:2d}] {mj_actuator_names[mj_i]}")
    assert len(ISAAC_TO_MJ_MAP) == 12, f"Mapping incomplete: only {len(ISAAC_TO_MJ_MAP)}/12 joints mapped"

    DEFAULT_POS_MJ = get_default_pos_mj()

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
        action_latency_steps = int((args.latency_ms / 2.0) / 1000.0 / PHYSICS_DT)
        proprio_latency_steps = int((args.latency_ms / 2.0) / 1000.0 / PHYSICS_DT)
    else:
        action_latency_steps = 3 * SUBSTEPS // 2
        proprio_latency_steps = 3 * SUBSTEPS // 2

    target_buffer = collections.deque([DEFAULT_POS_MJ.copy() for _ in range(action_latency_steps + 1)], maxlen=action_latency_steps + 1)
    proprio_buffer = collections.deque(maxlen=proprio_latency_steps + 1)

    if args.imu_latency_ms > 0:
        imu_latency_steps = int(args.imu_latency_ms / 1000.0 / PHYSICS_DT)
    else:
        imu_latency_steps = 0
    # Store tuples of (quat, ang_vel, lin_vel_world)
    imu_buffer = collections.deque(maxlen=imu_latency_steps + 1)


    print(f"\n[INFO] Delay Configuration:")
    print(f"  Physics DT   : {PHYSICS_DT*1000:.2f} ms")
    print(f"  Policy DT    : {POLICY_DT*1000:.2f} ms ({SUBSTEPS} substeps)")
    print(f"  Action Lat   : {args.latency_ms/2.0} ms -> {action_latency_steps} physics steps (Buffer len: {target_buffer.maxlen})")
    print(f"  Proprio Lat  : {args.latency_ms/2.0} ms -> {proprio_latency_steps} physics steps (Buffer len: {proprio_buffer.maxlen})")
    print(f"  IMU Latency  : {args.imu_latency_ms} ms -> {imu_latency_steps} physics steps (Buffer len: {imu_buffer.maxlen})\n")

    csv_writer = None
    if args.video:
        import csv
        csv_path = args.video.replace('.mp4', '.csv')
        log_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(log_file)
        header = ['time'] + [f'cmd_{j}' for j in MASTER_JOINT_ORDER] + [f'pos_{j}' for j in MASTER_JOINT_ORDER] + [f'act_{j}' for j in MASTER_JOINT_ORDER]
        csv_writer.writerow(header)

    # Initialize IMU buffer
    mujoco.mj_step(model, data)  # one step to populate sensors
    quat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
    quat_adr = model.sensor_adr[quat_id]
    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
    gyro_adr = model.sensor_adr[gyro_id]
    
    init_imu = (
        data.sensordata[quat_adr:quat_adr + 4].copy(),
        data.sensordata[gyro_adr:gyro_adr + 3].copy(),
        data.qvel[0:3].copy()
    )
    for _ in range(imu_buffer.maxlen):
        imu_buffer.append(init_imu)
    delayed_imu = imu_buffer[0]

    init_proprio = (data.qpos[qp_idx].copy(), data.qvel[qv_idx].copy())
    for _ in range(proprio_buffer.maxlen):
        proprio_buffer.append(init_proprio)
    delayed_proprio = proprio_buffer[0]

    step = 0
    try:
        while playing and (args.duration == float('inf') or step < int(args.duration / POLICY_DT)):
            t0 = time.perf_counter()

            # Build observation
            obs = build_observation(data, model, qp_idx, qv_idx, cmd_vel, last_action, imu_data=delayed_imu, proprio_data=delayed_proprio, obs_dim=obs_dim)

            # Run policy
            actions_isaac = policy.run(None, {input_name: obs.reshape(1, -1)})[0][0]
            actions_isaac = np.clip(actions_isaac, -10.0, 10.0)
            last_action = actions_isaac.copy()

            # Convert to joint targets (Isaac order)
            # Network outputs action directly in MASTER_JOINT_ORDER
            
            # Map action directly to MuJoCo targets using default position and scale
            targets_isaac = np.zeros(12, dtype=np.float32)
            for i, name in enumerate(MASTER_JOINT_ORDER):
                targets_isaac[i] = DEFAULT_POS_ISAAC[name] + actions_isaac[i] * ACTION_SCALE[i]
                
            # Reorder to MuJoCo actuator order using mapping
            targets_mj = np.zeros(len(mj_actuator_names), dtype=np.float32)
            for isaac_name, mj_i in ISAAC_TO_MJ_MAP.items():
                isaac_i = MASTER_JOINT_ORDER.index(isaac_name)
                targets_mj[mj_i] = targets_isaac[isaac_i]
            
            # Update IMU buffer at physics rate
            quat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
            quat_adr = model.sensor_adr[quat_id]
            current_quat = data.sensordata[quat_adr:quat_adr + 4].copy()
            gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
            gyro_adr = model.sensor_adr[gyro_id]
            current_gyro = data.sensordata[gyro_adr:gyro_adr + 3].copy()
            current_lin_vel = data.qvel[0:3].copy()
            

            # Find root body ID for push
            root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "root")
            push_active = False

            # Step physics at 2000Hz
            for _ in range(SUBSTEPS):
                # Handle Push
                current_time = data.time
                if args.push_time > 0 and args.push_time <= current_time <= (args.push_time + args.push_duration):
                    data.xfrc_applied[root_id][:3] = args.push_force
                    if not push_active:
                        print(f"[{current_time:.2f}s] [INFO] Applying push force: {args.push_force} N")
                        push_active = True
                else:
                    data.xfrc_applied[root_id][:3] = [0.0, 0.0, 0.0]
                    if push_active:
                        print(f"[{current_time:.2f}s] [INFO] Push ended.")
                        push_active = False

                target_buffer.append(targets_mj.copy())
                delayed_targets_mj = target_buffer[0]
                
                # Push newest IMU
                imu_buffer.append((
                    data.sensordata[quat_adr:quat_adr + 4].copy(),
                    data.sensordata[gyro_adr:gyro_adr + 3].copy(),
                    data.qvel[0:3].copy()
                ))

                # Push newest Proprioception
                proprio_buffer.append((data.qpos[qp_idx].copy(), data.qvel[qv_idx].copy()))

                jp = data.qpos[qp_idx]
                jv = data.qvel[qv_idx]
                torques = get_kp_mj() * (delayed_targets_mj - jp) + get_kd_mj() * (0.0 - jv)
                friction_torque = -get_friction_mj() * np.sign(jv)
                torques = torques + friction_torque
                torques = np.clip(torques, -get_effort_mj(), get_effort_mj())
                data.ctrl[actuator_idx] = torques
                mujoco.mj_step(model, data)
                
            delayed_imu = imu_buffer[0]
            delayed_proprio = proprio_buffer[0]


            if csv_writer is not None:
                mj_pos_by_isaac = [data.qpos[qp_idx][ISAAC_TO_MJ_MAP[name]] for name in MASTER_JOINT_ORDER]
                row = [step * POLICY_DT] + targets_isaac.tolist() + mj_pos_by_isaac + actions_isaac.tolist()
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
            # No sleep for headless recording

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