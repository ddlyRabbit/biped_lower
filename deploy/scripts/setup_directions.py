#!/usr/bin/env python3
"""Interactive motor direction setup — run alongside bringup.launch.py.

For each joint, lets you toggle direction (+1/-1) while watching
the robot in Foxglove. Saves results to calibration.yaml.

Usage:
    # Terminal 1: bringup running
    ros2 launch biped_bringup bringup.launch.py ...

    # Terminal 2: this script
    source ~/biped_lower/deploy/biped_ws/setup_biped.bash
    python3 ~/biped_lower/deploy/scripts/setup_directions.py

    # Or specify calibration file:
    python3 ~/biped_lower/deploy/scripts/setup_directions.py --cal calibration.yaml
"""

import argparse
import sys
import yaml

CAL_PATH = "/home/roy/biped_lower/deploy/biped_ws/calibration.yaml"

JOINTS = [
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw",
    "R_knee", "R_foot_pitch", "R_foot_roll",
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw",
    "L_knee", "L_foot_pitch", "L_foot_roll",
]


def main():
    parser = argparse.ArgumentParser(description="Interactive motor direction setup")
    parser.add_argument("--cal", default=CAL_PATH, help="Path to calibration.yaml")
    args = parser.parse_args()

    # Load calibration
    try:
        with open(args.cal) as f:
            cal = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"Failed to load {args.cal}: {e}")
        return

    print("=" * 60)
    print("  Motor Direction Setup")
    print("=" * 60)
    print()
    print("For each joint, the current direction will be applied.")
    print("Move the joint by hand and check Foxglove:")
    print("  - If it moves the RIGHT way → press Enter (keep)")
    print("  - If it moves the WRONG way → type 'f' + Enter (flip)")
    print("  - Type 's' to skip a joint")
    print("  - Type 'q' to quit and save")
    print()

    # Current directions
    for name in JOINTS:
        if name not in cal:
            cal[name] = {}
        if "direction" not in cal[name]:
            cal[name]["direction"] = 1

    # Show current state
    print("Current directions:")
    for name in JOINTS:
        d = cal[name].get("direction", 1)
        print(f"  {name:20s}  direction={d:+d}")
    print()

    changed = False
    for name in JOINTS:
        current_dir = cal[name].get("direction", 1)
        print(f"\n--- {name} (current: direction={current_dir:+d}) ---")
        print(f"  Move this joint and check Foxglove.")
        print(f"  [Enter]=correct  [f]=flip  [s]=skip  [q]=quit+save")

        while True:
            try:
                resp = input(f"  {name} > ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                resp = "q"

            if resp == "" or resp == "y":
                # Keep current direction
                print(f"  ✅ {name} → direction={current_dir:+d} (kept)")
                break
            elif resp == "f" or resp == "flip":
                # Flip direction
                new_dir = -current_dir
                cal[name]["direction"] = new_dir
                changed = True
                print(f"  🔄 {name} → direction={new_dir:+d} (flipped)")
                print(f"  Move joint again to verify. [Enter]=confirm  [f]=flip back")
                continue
            elif resp == "s" or resp == "skip":
                print(f"  ⏭  {name} skipped")
                break
            elif resp == "q" or resp == "quit":
                print("\nStopping early.")
                break
            else:
                print(f"  Unknown: '{resp}'. Use Enter/f/s/q")
                continue

        if resp == "q" or resp == "quit":
            break

    # Summary
    print("\n" + "=" * 60)
    print("  Final Directions")
    print("=" * 60)
    for name in JOINTS:
        d = cal[name].get("direction", 1)
        marker = " ← changed" if d != 1 else ""
        print(f"  {name:20s}  direction={d:+d}{marker}")

    # Save
    if changed:
        print(f"\nSaving to {args.cal}...")
        with open(args.cal, "w") as f:
            yaml.dump(cal, f, default_flow_style=False, sort_keys=False)
        print(f"✅ Saved! Restart bringup.launch.py to apply.")
    else:
        print("\nNo changes made.")


if __name__ == "__main__":
    main()
