#!/usr/bin/env python3
"""Play a CSV joint trajectory in MuJoCo — kinematic playback with walking base.

Usage:
    python sim2sim/play_traj_mujoco.py --csv deploy/biped_ws/src/biped_bringup/config/trajectory.csv
    python sim2sim/play_traj_mujoco.py --csv trajectory.csv --headless --video output.mp4

CSV format:
    Row 0: timestamps (seconds)
    Rows 1-12: joint angles (rad) in order:
        L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee, L_foot_pitch, L_foot_roll,
        R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee, R_foot_pitch, R_foot_roll

The base (floating joint) is translated forward based on --step_length and --num_steps
to simulate walking motion. The base height is set by --init_height.
"""

import argparse
import os
import time

import numpy as np
import mujoco
import mujoco.viewer

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MJCF_PATH = os.path.join(REPO_ROOT, "mjcf", "sim2sim", "robot.mjcf")

# CSV joint order → MuJoCo joint names
CSV_TO_MJ = [
    "left_hip_pitch_04", "left_hip_roll_03", "left_hip_yaw_03",
    "left_knee_04", "left_foot_pitch_02", "left_foot_roll_02",
    "right_hip_pitch_04", "right_hip_roll_03", "right_hip_yaw_03",
    "right_knee_04", "right_foot_pitch_02", "right_foot_roll_02",
]


def load_trajectory(csv_path: str):
    """Load CSV trajectory. Returns (timestamps, joint_angles[12, N])."""
    raw = np.genfromtxt(csv_path, delimiter=',')
    timestamps = raw[0]
    joint_angles = raw[1:]  # (12, N)
    assert joint_angles.shape[0] == 12, f"Expected 12 joints, got {joint_angles.shape[0]}"
    return timestamps, joint_angles


def compute_base_x(timestamps, step_length, num_steps, step_period):
    """Compute forward base displacement over time.
    
    Layout: idle(1 step_period) + walk(num_steps * step_period) + idle(1 step_period)
    Total forward: num_steps * step_length (first/last steps are half length).
    """
    n = len(timestamps)
    base_x = np.zeros(n)
    total_distance = num_steps * step_length
    idle_duration = step_period
    walk_start = idle_duration
    walk_end = idle_duration + num_steps * step_period

    for i, t in enumerate(timestamps):
        if t <= walk_start:
            base_x[i] = 0.0
        elif t >= walk_end:
            base_x[i] = total_distance
        else:
            progress = (t - walk_start) / (walk_end - walk_start)
            # Smooth cosine interpolation
            base_x[i] = total_distance * (0.5 * (1 - np.cos(np.pi * progress)))

    return base_x


def main():
    parser = argparse.ArgumentParser(description="Play CSV trajectory in MuJoCo")
    parser.add_argument("--csv", required=True, help="Path to trajectory CSV")
    parser.add_argument("--mjcf", default=MJCF_PATH, help="Path to MJCF model")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--loop", action="store_true", help="Loop trajectory")
    parser.add_argument("--headless", action="store_true", help="No viewer (for video)")
    parser.add_argument("--video", type=str, default=None, help="Save video to path")
    parser.add_argument("--video_fps", type=int, default=30, help="Video FPS")
    parser.add_argument("--init_height", type=float, default=0.78, help="Initial base height (m)")
    parser.add_argument("--step_length", type=float, default=0.05, help="Step length (m)")
    parser.add_argument("--num_steps", type=int, default=6, help="Number of steps")
    parser.add_argument("--step_period", type=float, default=3.0, help="Step period (s)")
    args = parser.parse_args()

    # Load model
    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)

    # Build joint index map: CSV order → MuJoCo qpos indices
    qpos_indices = []
    for csv_joint in CSV_TO_MJ:
        jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, csv_joint)
        if jnt_id < 0:
            raise ValueError(f"Joint '{csv_joint}' not found in MJCF")
        qpos_idx = model.jnt_qposadr[jnt_id]
        qpos_indices.append(qpos_idx)
    qpos_indices = np.array(qpos_indices)

    # Load trajectory
    timestamps, joint_angles = load_trajectory(args.csv)
    n_frames = len(timestamps)
    dt = np.median(np.diff(timestamps))
    duration = timestamps[-1] - timestamps[0]
    print(f"Trajectory: {n_frames} frames, {duration:.2f}s, dt={dt*1000:.1f}ms")

    # Compute base forward motion
    base_x = compute_base_x(timestamps, args.step_length, args.num_steps, args.step_period)
    print(f"Base displacement: {base_x[-1]:.3f}m over {duration:.1f}s")

    # Video / camera setup
    renderer = None
    video_frames = []
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = 0
    camera.distance = 1.2
    camera.azimuth = 90
    camera.elevation = -10
    camera.lookat[:] = [0, 0, 0.4]

    if args.video:
        renderer = mujoco.Renderer(model, height=480, width=640)
        print(f"Recording video to {args.video} @ {args.video_fps}fps")

    if not args.headless and args.video is None:
        # Interactive viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("Playing trajectory... (close window to stop)")
            playing = True
            while playing:
                for frame_idx in range(n_frames):
                    if not viewer.is_running():
                        playing = False
                        break

                    # Set base position (free joint: qpos[0:3] = xyz, qpos[3:7] = quat)
                    data.qpos[0] = base_x[frame_idx]
                    data.qpos[1] = 0.0
                    data.qpos[2] = args.init_height
                    data.qpos[3:7] = [1, 0, 0, 0]

                    # Set joint positions
                    data.qpos[qpos_indices] = joint_angles[:, frame_idx]
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    time.sleep(dt / args.speed)

                if not args.loop:
                    break

            print("Done.")
    else:
        # Headless / video mode
        frame_skip = max(1, int(1.0 / (dt * args.video_fps)))
        for frame_idx in range(n_frames):
            # Set base position
            data.qpos[0] = base_x[frame_idx]
            data.qpos[1] = 0.0
            data.qpos[2] = args.init_height
            data.qpos[3:7] = [1, 0, 0, 0]

            # Set joint positions
            data.qpos[qpos_indices] = joint_angles[:, frame_idx]
            mujoco.mj_forward(model, data)

            if renderer and frame_idx % frame_skip == 0:
                renderer.update_scene(data, camera)
                video_frames.append(renderer.render().copy())

            if frame_idx % 1000 == 0:
                t = timestamps[frame_idx]
                print(f"  Frame {frame_idx}/{n_frames} ({t:.2f}s)")

        print(f"Processed {n_frames} frames.")

    # Save video
    if args.video and video_frames:
        try:
            import imageio
            writer = imageio.get_writer(args.video, fps=args.video_fps)
            for frame in video_frames:
                writer.append_data(frame)
            writer.close()
            print(f"Video saved: {args.video} ({len(video_frames)} frames)")
        except ImportError:
            try:
                import mediapy as media
                media.write_video(args.video, video_frames, fps=args.video_fps)
                print(f"Video saved: {args.video} ({len(video_frames)} frames)")
            except ImportError:
                print("Install imageio or mediapy for video export")


if __name__ == "__main__":
    main()
