#!/usr/bin/env python3
"""Combine agent video with torque plots.

Reads the video mp4 + torques.csv from a record dir,
renders matplotlib plot frames, stacks video on top + plot below.

Usage:
    python combine_torque_video.py --input_dir /results/record --output /results/record/combined.mp4
"""

import argparse
import csv
import json
import os
import subprocess
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from glob import glob


def build_joint_groups(joint_names):
    """Build joint groups from actual sim joint names."""
    groups = {
        "Right Hip": [],
        "Right Leg": [],
        "Left Hip": [],
        "Left Leg": [],
    }
    for jn in joint_names:
        if "right" in jn and ("hip" in jn):
            groups["Right Hip"].append(jn)
        elif "right" in jn:
            groups["Right Leg"].append(jn)
        elif "left" in jn and ("hip" in jn):
            groups["Left Hip"].append(jn)
        elif "left" in jn:
            groups["Left Leg"].append(jn)
    return groups


# 3 colors per group, cycled
GROUP_COLORS = ["#e41a1c", "#ff7f00", "#984ea3"]


def load_torques(csv_path):
    """Load torques CSV into dict of numpy arrays."""
    data = {}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    for key in rows[0].keys():
        data[key] = np.array([float(r[key]) for r in rows])

    return data


def render_plot_frame(data, frame_idx, total_frames, dt, fig, axes, joint_groups):
    """Render a single plot frame showing torques up to frame_idx."""
    time = data["time_s"]
    t_max = time[-1]

    for ax, (group_name, joints) in zip(axes.flat, joint_groups.items()):
        ax.clear()
        for ci, jname in enumerate(joints):
            key = jname + "_torque"
            color = GROUP_COLORS[ci % len(GROUP_COLORS)]
            if key in data:
                # Full trace in light gray
                ax.plot(time, data[key], color="#dddddd", linewidth=0.5)
                # Active trace up to current frame
                # Clean label: strip side prefix and _0x suffix
                label = jname.replace("left_", "").replace("right_", "")
                label = "_".join(label.split("_")[:-1]) if label[-1].isdigit() else label
                ax.plot(time[:frame_idx+1], data[key][:frame_idx+1],
                        color=color, linewidth=1.2,
                        label=label)

        # Timeline bar
        if frame_idx < len(time):
            ax.axvline(x=time[frame_idx], color="red", linewidth=1.5, alpha=0.7)

        ax.set_xlim(0, t_max)
        ax.set_title(group_name, fontsize=9, fontweight="bold", pad=2)
        ax.set_ylabel("Nm", fontsize=7)
        ax.tick_params(labelsize=6)
        ax.legend(fontsize=6, loc="upper right", ncol=1, framealpha=0.5)

        # Auto y-limits from full data
        all_vals = []
        for jname in joints:
            key = jname + "_torque"
            if key in data:
                all_vals.append(data[key])
        if all_vals:
            all_vals = np.concatenate(all_vals)
            ymin, ymax = np.percentile(all_vals, [1, 99])
            margin = max(abs(ymax - ymin) * 0.15, 1.0)
            ax.set_ylim(ymin - margin, ymax + margin)

    axes.flat[-1].set_xlabel("Time (s)", fontsize=7)
    axes.flat[-2].set_xlabel("Time (s)", fontsize=7)

    fig.tight_layout(pad=0.5)
    fig.canvas.draw()

    # Convert to numpy array
    buf = fig.canvas.buffer_rgba()
    img = np.asarray(buf)[:, :, :3]  # drop alpha
    return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", required=True)
    parser.add_argument("--output", default=None)
    parser.add_argument("--plot_height", type=int, default=360, help="Plot panel height in pixels")
    args = parser.parse_args()

    input_dir = args.input_dir
    csv_path = os.path.join(input_dir, "torques.csv")
    meta_path = os.path.join(input_dir, "meta.json")
    video_files = glob(os.path.join(input_dir, "video", "*.mp4"))

    if not video_files:
        print("ERROR: No video found in", os.path.join(input_dir, "video"))
        return

    video_path = video_files[0]
    output_path = args.output or os.path.join(input_dir, "combined.mp4")

    with open(meta_path) as f:
        meta = json.load(f)

    fps = meta.get("fps", 50)
    total_frames = meta["steps"]

    print(f"Video: {video_path}")
    print(f"Torques: {csv_path}")
    print(f"FPS: {fps}, Frames: {total_frames}")

    # Load data
    data = load_torques(csv_path)
    joint_names = meta.get("joint_names", [])
    joint_groups = build_joint_groups(joint_names)

    # Get video dimensions
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=width,height", "-of", "csv=p=0", video_path],
        capture_output=True, text=True
    )
    vid_w, vid_h = map(int, probe.stdout.strip().split(","))
    print(f"Video size: {vid_w}x{vid_h}")

    # Plot dimensions to match video width
    plot_h = args.plot_height
    dpi = 100
    fig_w = vid_w / dpi
    fig_h = plot_h / dpi
    fig, axes = plt.subplots(2, 2, figsize=(fig_w, fig_h), dpi=dpi)
    fig.patch.set_facecolor("#1a1a2e")
    for ax in axes.flat:
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#cccccc")
        ax.xaxis.label.set_color("#cccccc")
        ax.yaxis.label.set_color("#cccccc")
        ax.title.set_color("#ffffff")
        for spine in ax.spines.values():
            spine.set_color("#444444")

    # Render plot frames to temp dir
    tmpdir = tempfile.mkdtemp()
    plot_frames_dir = os.path.join(tmpdir, "plots")
    os.makedirs(plot_frames_dir)

    print(f"Rendering {total_frames} plot frames...")
    for i in range(total_frames):
        img = render_plot_frame(data, i, total_frames, meta["dt"], fig, axes, joint_groups)
        # Save as png
        plt.savefig(os.path.join(plot_frames_dir, f"frame_{i:05d}.png"),
                    dpi=dpi, facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.1)
        if i % 100 == 0:
            print(f"  Frame {i}/{total_frames}")

    plt.close(fig)

    # Encode plot frames to video
    plot_video = os.path.join(tmpdir, "plots.mp4")
    subprocess.run([
        "ffmpeg", "-y", "-framerate", str(fps),
        "-i", os.path.join(plot_frames_dir, "frame_%05d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
        plot_video
    ], check=True, capture_output=True)

    # Stack: agent video on top, plot on bottom
    subprocess.run([
        "ffmpeg", "-y",
        "-i", video_path,
        "-i", plot_video,
        "-filter_complex",
        f"[0:v]scale={vid_w}:{vid_h}[top];"
        f"[1:v]scale={vid_w}:{plot_h}[bottom];"
        f"[top][bottom]vstack=inputs=2[out]",
        "-map", "[out]",
        "-c:v", "libx264", "-crf", "23", "-preset", "fast",
        output_path
    ], check=True, capture_output=True)

    print(f"Combined video: {output_path}")

    # Cleanup
    import shutil
    shutil.rmtree(tmpdir)


if __name__ == "__main__":
    main()
