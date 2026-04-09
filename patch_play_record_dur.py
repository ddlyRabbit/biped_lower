import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

# Make default duration dependent on args.video
old_argparse = """    parser.add_argument("--duration", type=float, default=float("inf"))
    parser.add_argument("--video", type=str, default=None, help="Save video to path")"""

new_argparse = """    parser.add_argument("--duration", type=float, default=None, help="Playback duration in seconds (default: 10s for video, inf for interactive)")
    parser.add_argument("--video", type=str, default=None, help="Save video to path")"""
text = text.replace(old_argparse, new_argparse)

old_duration = """    csv_writer = None
    if args.video:"""

new_duration = """    if args.duration is None:
        args.duration = 10.0 if args.video else float('inf')

    csv_writer = None
    if args.video:"""
text = text.replace(old_duration, new_duration)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)

print("patched")
