import mujoco.viewer
import mujoco
import argparse
import os
import numpy as np

def record_trajectory(csv_path, output_video="/tmp/zmp_playback.mp4", duration=26.0):
    try:
        import mediapy as media
    except ImportError:
        import sys
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "mediapy"])
        import mediapy as media

    REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    MJCF_PATH = os.path.join(REPO_ROOT, "mjcf", "sim2sim", "robot.mjcf")

    print(f"Loading MuJoCo model from {MJCF_PATH}")
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    # Enable gravity — let it actually walk with physics
    # model.opt.gravity[:] = [0, 0, -9.81]  # default

    with open(csv_path, 'r') as f:
        lines = f.readlines()

    t_arr = np.array([float(x) for x in lines[0].strip().split(',')])
    n_frames = len(t_arr)
    dt = 0.01

    csv_joint_order = [
        "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_foot_pitch", "L_foot_roll",
        "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_foot_pitch", "R_foot_roll",
    ]

    trajectory = {}
    for i, name in enumerate(csv_joint_order):
        trajectory[name] = np.array([float(x) for x in lines[i + 1].strip().split(',')])

    # Map to MuJoCo actuator IDs (position-controlled)
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    mapping = {
        "L_hip_pitch": "left_hip_pitch_04", "L_hip_roll": "left_hip_roll_03", "L_hip_yaw": "left_hip_yaw_03",
        "L_knee": "left_knee_04", "L_foot_pitch": "left_foot_pitch_02", "L_foot_roll": "left_foot_roll_02",
        "R_hip_pitch": "right_hip_pitch_04", "R_hip_roll": "right_hip_roll_03", "R_hip_yaw": "right_hip_yaw_03",
        "R_knee": "right_knee_04", "R_foot_pitch": "right_foot_pitch_02", "R_foot_roll": "right_foot_roll_02",
    }

    mj_act_idx = {}
    for short_name, mapped_name in mapping.items():
        for act_name in actuator_names:
            if act_name and mapped_name in act_name:
                mj_act_idx[short_name] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
                break

    print(f"Mapped {len(mj_act_idx)} actuators")

    mujoco.mj_resetData(model, data)
    model.opt.timestep = dt

    # Set initial standing pose
    data.qpos[2] = 0.55  # z height
    data.qpos[3:7] = [1, 0, 0, 0]  # upright quaternion

    # Set initial joint positions from first frame
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    mj_qpos_idx = {}
    for short_name, mapped_name in mapping.items():
        for j_name in joint_names:
            if j_name and mapped_name in j_name:
                mj_qpos_idx[short_name] = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)]
                break

    for short_name, qpos_idx in mj_qpos_idx.items():
        data.qpos[qpos_idx] = trajectory[short_name][0]

    mujoco.mj_forward(model, data)

    print("Setting up renderer...")
    renderer = mujoco.Renderer(model, 480, 640)

    # Create a camera that tracks the robot — zoomed in, side view
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = 0  # track the torso (body 0 or 1)
    camera.distance = 1.5   # zoomed in
    camera.azimuth = 90     # side view
    camera.elevation = -15  # slightly above
    camera.lookat[:] = [0, 0, 0.4]

    frames = []

    max_steps = min(n_frames, int(duration / dt))
    print(f"Rendering {max_steps} frames with physics...")

    subsample = 4  # 100Hz sim → 25fps video

    for step in range(max_steps):
        # Send joint position commands to actuators
        for short_name, act_idx in mj_act_idx.items():
            data.ctrl[act_idx] = trajectory[short_name][step]

        # Step physics (robot walks freely)
        mujoco.mj_step(model, data)

        if step % subsample == 0:
            renderer.update_scene(data, camera)
            frames.append(renderer.render())

    print(f"Rendered {len(frames)} frames. Saving video to {output_video}...")
    media.write_video(output_video, frames, fps=int((1.0 / dt) / subsample))
    print("Done!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=str, default="/tmp/trajectory.csv")
    args = parser.parse_args()
    record_trajectory(args.csv)
