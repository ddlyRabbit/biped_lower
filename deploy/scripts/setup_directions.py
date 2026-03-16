#!/usr/bin/env python3
"""Interactive motor direction setup — run alongside bringup.launch.py.

For each joint, toggle direction (+1/-1) while watching Foxglove.
For ankle motors (foot_top/foot_bottom = upper/lower motor), shows
guidance since they are coupled through the parallel linkage.

Saves direction per joint in calibration.yaml.

Usage:
    # Terminal 1: bringup running + Foxglove connected
    # Terminal 2:
    source ~/biped_lower/deploy/biped_ws/setup_biped.bash
    python3 ~/biped_lower/deploy/scripts/setup_directions.py
"""

import argparse
import sys
import yaml

CAL_PATH = "/home/roy/biped_lower/deploy/biped_ws/calibration.yaml"

# Normal joints first, then ankle pairs together
NORMAL_JOINTS = [
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee",
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee",
]

ANKLE_PAIRS = [
    ("R_foot_top", "R_foot_bottom"),  # R upper, R lower
    ("L_foot_top", "L_foot_bottom"),  # L upper, L lower
]

ALL_JOINTS = NORMAL_JOINTS + [j for pair in ANKLE_PAIRS for j in pair]


def setup_normal_joint(name, cal):
    """Interactive direction setup for a normal (non-ankle) joint."""
    current_dir = cal[name].get("direction", 1)
    print(f"\n{'='*50}")
    print(f"  {name}  (current: direction={current_dir:+d})")
    print(f"{'='*50}")
    print(f"  Move this joint and check Foxglove.")
    print(f"  [Enter]=correct  [f]=flip  [s]=skip  [q]=quit")

    while True:
        try:
            resp = input(f"  {name} > ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            return "q"

        if resp == "" or resp == "y":
            print(f"  ✅ {name} → direction={cal[name].get('direction', 1):+d}")
            return "ok"
        elif resp in ("f", "flip"):
            new_dir = -cal[name].get("direction", 1)
            cal[name]["direction"] = new_dir
            print(f"  🔄 {name} → direction={new_dir:+d}")
            print(f"  Move joint again to verify...")
            continue
        elif resp in ("s", "skip"):
            print(f"  ⏭  skipped")
            return "ok"
        elif resp in ("q", "quit"):
            return "q"
        else:
            print(f"  Use: Enter/f/s/q")


def setup_ankle_pair(upper_name, lower_name, cal):
    """Interactive direction setup for an ankle motor pair.
    
    Since both motors are coupled through the parallel linkage:
      pitch = INV_P × (upper − lower)
      roll  = −INV_R × (upper + lower)
    
    Changing one motor's direction affects both pitch and roll.
    We test them together.
    """
    side = upper_name[0]  # R or L
    u_dir = cal[upper_name].get("direction", 1)
    l_dir = cal[lower_name].get("direction", 1)

    print(f"\n{'='*60}")
    print(f"  {side} ANKLE — upper motor ({upper_name}) + lower motor ({lower_name})")
    print(f"  Current: upper={u_dir:+d}  lower={l_dir:+d}")
    print(f"{'='*60}")
    print()
    print(f"  These motors are COUPLED through the ankle linkage.")
    print(f"  Flipping one motor affects both pitch AND roll display.")
    print()
    print(f"  Test procedure:")
    print(f"  1. Move ONLY the foot in pure pitch (tilt front/back)")
    print(f"     → Foxglove should show pitch changing, roll near zero")
    print(f"  2. Move ONLY the foot in pure roll (tilt left/right)")
    print(f"     → Foxglove should show roll changing, pitch near zero")
    print()
    print(f"  If pitch/roll are swapped or inverted, flip motors:")
    print(f"  [u] = flip upper motor ({upper_name})")
    print(f"  [l] = flip lower motor ({lower_name})")
    print(f"  [b] = flip both")
    print(f"  [Enter] = both correct")
    print(f"  [s] = skip  [q] = quit")

    while True:
        u_dir = cal[upper_name].get("direction", 1)
        l_dir = cal[lower_name].get("direction", 1)
        print(f"\n  State: upper={u_dir:+d}  lower={l_dir:+d}")

        try:
            resp = input(f"  {side}_ankle > ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            return "q"

        if resp == "" or resp == "y":
            print(f"  ✅ {upper_name}={u_dir:+d}  {lower_name}={l_dir:+d}")
            return "ok"
        elif resp == "u":
            new = -cal[upper_name].get("direction", 1)
            cal[upper_name]["direction"] = new
            print(f"  🔄 upper ({upper_name}) → {new:+d}")
            print(f"  Move foot again to verify...")
        elif resp == "l":
            new = -cal[lower_name].get("direction", 1)
            cal[lower_name]["direction"] = new
            print(f"  🔄 lower ({lower_name}) → {new:+d}")
            print(f"  Move foot again to verify...")
        elif resp == "b":
            new_u = -cal[upper_name].get("direction", 1)
            new_l = -cal[lower_name].get("direction", 1)
            cal[upper_name]["direction"] = new_u
            cal[lower_name]["direction"] = new_l
            print(f"  🔄 both flipped: upper={new_u:+d}  lower={new_l:+d}")
            print(f"  Move foot again to verify...")
        elif resp in ("s", "skip"):
            print(f"  ⏭  skipped")
            return "ok"
        elif resp in ("q", "quit"):
            return "q"
        else:
            print(f"  Use: u/l/b/Enter/s/q")


def main():
    parser = argparse.ArgumentParser(description="Interactive motor direction setup")
    parser.add_argument("--cal", default=CAL_PATH, help="Path to calibration.yaml")
    args = parser.parse_args()

    try:
        with open(args.cal) as f:
            cal = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"Failed to load {args.cal}: {e}")
        return

    # Ensure all joints have direction
    for name in ALL_JOINTS:
        if name not in cal:
            cal[name] = {}
        if "direction" not in cal[name]:
            cal[name]["direction"] = 1

    # Original state for change detection
    original = {name: cal[name].get("direction", 1) for name in ALL_JOINTS}

    print("=" * 60)
    print("  Motor Direction Setup")
    print("=" * 60)
    print()
    print("  Make sure bringup.launch.py is running and Foxglove")
    print("  is connected. Move joints and verify direction.")
    print()
    print("  Directions are saved to calibration.yaml.")
    print("  Restart bringup after saving to apply changes.")
    print()

    # Normal joints
    for name in NORMAL_JOINTS:
        result = setup_normal_joint(name, cal)
        if result == "q":
            break

    # Ankle pairs
    if result != "q":
        for upper, lower in ANKLE_PAIRS:
            result = setup_ankle_pair(upper, lower, cal)
            if result == "q":
                break

    # Summary
    changed = [n for n in ALL_JOINTS if cal[n].get("direction", 1) != original[n]]

    print(f"\n{'='*60}")
    print(f"  Summary")
    print(f"{'='*60}")
    for name in ALL_JOINTS:
        d = cal[name].get("direction", 1)
        marker = " ← CHANGED" if name in changed else ""
        print(f"  {name:20s}  direction={d:+d}{marker}")

    if changed:
        print(f"\nSaving to {args.cal}...")
        with open(args.cal, "w") as f:
            yaml.dump(cal, f, default_flow_style=False, sort_keys=False)
        print(f"✅ Saved! Restart bringup.launch.py to apply.")
    else:
        print("\nNo changes.")


if __name__ == "__main__":
    main()
