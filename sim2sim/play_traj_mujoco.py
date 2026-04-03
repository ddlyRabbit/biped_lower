#!/usr/bin/env python3
"""Play a CSV joint trajectory in MuJoCo.

Usage:
    python sim2sim/play_traj_mujoco.py --csv deploy/biped_ws/src/biped_bringup/config/trajectory.csv
    python sim2sim/play_traj_mujoco.py --csv trajectory.csv --headless --video output.mp4

CSV format:
    Row 0: timestamps (seconds)
    Rows 1-12: joint angles (rad) in order:
        L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee, L_foot_pitch, L_foot_roll,
        R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee, R_foot_pitch, R_foot_roll
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


def main():
    parser = argparse.ArgumentParser(description="Play CSV trajectory in MuJoCo")
    parser.add_argument("--csv", required=True, help="Path to trajectory CSV")
    parser.add_argument("--mjcf", default=MJCF_PATH, help="Path to MJCF model")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--loop", action="store_true", help="Loop trajectory")
    parser.add_argument("--headless", action="store_true", help="No viewer (for video)")
    parser.add_argument("--video", type=str, default=None, help="Save video to path")
    parser.add_argument("--video_fps", type=int, default=30, help="Video FPS")
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
    print(f"Playback speed: {args.speed}x")

    # Video setup
    renderer = None
    video_frames = []
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

                    # Set joint positions (FK only)
                    data.qpos[qpos_indices] = joint_angles[:, frame_idx]
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    time.sleep(dt / args.speed)

                if not args.loop:
                    break

            print("Done.")
    else:
        # Headless / video mode
        for frame_idx in range(n_frames):
            data.qpos[qpos_indices] = joint_angles[:, frame_idx]
            mujoco.mj_forward(model, data)

            if renderer and frame_idx % max(1, int(1.0 / (dt * args.video_fps))) == 0:
                renderer.update_scene(data)
                video_frames.append(renderer.render().copy())

            if frame_idx % 1000 == 0:
                t = timestamps[frame_idx]
                print(f"  Frame {frame_idx}/{n_frames} ({t:.2f}s)")

        if args.loop:
            print("Loop mode not supported in headless")

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
            # Fallback: save as raw frames
            out_dir = args.video + "_frames"
            os.makedirs(out_dir, exist_ok=True)
            for i, frame in enumerate(video_frames):
                import PIL.Image
                PIL.Image.fromarray(frame).save(os.path.join(out_dir, f"{i:05d}.png"))
            print(f"Frames saved to {out_dir}/ ({len(video_frames)} frames)")
            print("Install imageio for MP4: pip install imageio[ffmpeg]")


if __name__ == "__main__":
    main()
