#!/usr/bin/env python3
"""Test hip yaw motors — slow jog with soft stop verification.

Uses BipedMotorManager (same as can_bus_node) so soft stops are
applied identically to the real system, using calibration limits.

Jogs R_hip_yaw and L_hip_yaw from 0 → +limit → 0 → -limit → 0
at a slow rate, printing position and torque feedback.

Usage (on RPi, with CAN up, no other nodes running):
    python3 test_hip_yaw.py
    python3 test_hip_yaw.py --kp 2.0 --speed 0.3
    python3 test_hip_yaw.py --motor R_hip_yaw
    python3 test_hip_yaw.py --max-angle 30
"""

import argparse
import math
import sys
import time
import yaml

sys.path.insert(0, "/home/roy/biped_lower/deploy/biped_ws/src/biped_driver")
from biped_driver.robstride_can import BipedMotorManager, SOFTSTOP_BUFFER_RAD

CAL_PATH = "/home/roy/biped_lower/deploy/biped_ws/calibration.yaml"
ROBOT_YAML_PATH = "/home/roy/biped_lower/deploy/biped_ws/src/biped_bringup/config/robot.yaml"

HIP_YAW_JOINTS = ["R_hip_yaw", "L_hip_yaw"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motor", default="both", help="R_hip_yaw, L_hip_yaw, or both")
    parser.add_argument("--kp", type=float, default=3.0, help="Position gain")
    parser.add_argument("--kd", type=float, default=1.0, help="Velocity gain")
    parser.add_argument("--speed", type=float, default=0.2, help="Jog speed (rad/s)")
    parser.add_argument("--max-angle", type=float, default=None,
                        help="Max angle in degrees (default: soft stop limit)")
    parser.add_argument("--hold", type=float, default=1.0, help="Hold time at limits (s)")
    args = parser.parse_args()

    # Select motors
    if args.motor == "both":
        motor_names = list(HIP_YAW_JOINTS)
    elif args.motor in HIP_YAW_JOINTS:
        motor_names = [args.motor]
    else:
        print(f"Unknown motor: {args.motor}. Choose from: {HIP_YAW_JOINTS}")
        return

    # Load robot config + calibration → build manager (same as can_bus_node)
    try:
        with open(ROBOT_YAML_PATH) as f:
            robot_config = yaml.safe_load(f)
    except Exception as e:
        print(f"Failed to load robot.yaml: {e}")
        return

    try:
        with open(CAL_PATH) as f:
            offsets = yaml.safe_load(f)
        print(f"Loaded calibration: {CAL_PATH}")
    except Exception as e:
        print(f"No calibration loaded: {e}")
        offsets = None

    mgr = BipedMotorManager.from_robot_yaml(robot_config, offsets)

    # Print limits for selected motors
    for name in motor_names:
        j = mgr.joints[name]
        print(f"\n{name} (CAN id={j.can_id}, offset={j.offset:.4f}):")
        print(f"  Hard limits:  [{j.motor_cmd_lo:+.4f}, {j.motor_cmd_hi:+.4f}] "
              f"([{math.degrees(j.motor_cmd_lo):+.1f}°, {math.degrees(j.motor_cmd_hi):+.1f}°])")
        print(f"  Soft stops:   [{j.motor_softstop_lo:+.4f}, {j.motor_softstop_hi:+.4f}] "
              f"([{math.degrees(j.motor_softstop_lo):+.1f}°, {math.degrees(j.motor_softstop_hi):+.1f}°])")

    # Compute jog range
    if args.max_angle is not None:
        max_rad = math.radians(args.max_angle)
    else:
        # Use the tightest soft stop limit across selected motors
        max_rad = min(
            min(abs(mgr.joints[n].motor_softstop_lo), abs(mgr.joints[n].motor_softstop_hi))
            for n in motor_names
        )

    print(f"\nJog: kp={args.kp}, kd={args.kd}, speed={args.speed} rad/s")
    print(f"Range: ±{math.degrees(max_rad):.1f}° ({max_rad:.4f} rad)")
    print(f"Sequence: 0 → +{math.degrees(max_rad):.0f}° → 0 → -{math.degrees(max_rad):.0f}° → 0")

    # Connect and enable only the selected motors
    print("\nConnecting...")
    mgr.connect_all()
    mgr.flush_all()

    # Enable only selected motors (not all 12)
    for name in motor_names:
        bus = mgr._bus_for(name)
        bus.enable(name)
        time.sleep(0.02)
        bus.set_mode(name, 0)
        time.sleep(0.02)
    mgr.flush_all()

    # Read initial positions
    print("Initial positions:")
    last_pos = {}
    for name in motor_names:
        # Zero-torque read
        mgr.send_mit_command(name, 0.0, 0.0, 0.0)
        fb = mgr.read_feedback(name, timeout=0.05)
        if fb:
            last_pos[name] = fb.position
            print(f"  {name}: pos={fb.position:+.4f} ({math.degrees(fb.position):+.1f}°) "
                  f"τ={fb.torque:+.3f} Nm  temp={fb.temperature:.1f}°C")
        else:
            last_pos[name] = 0.0
            print(f"  {name}: NO RESPONSE")

    print(f"\nPress Ctrl+C to stop\n")

    # Waypoints: (target_rad, label)
    waypoints = [
        (+max_rad, f"+{math.degrees(max_rad):.0f}°"),
        (0.0, "0°"),
        (-max_rad, f"-{math.degrees(max_rad):.0f}°"),
        (0.0, "0°"),
    ]

    dt = 0.02  # 50Hz
    targets = {n: 0.0 for n in motor_names}
    t_start = time.monotonic()

    try:
        for wp_target, wp_label in waypoints:
            print(f"\n--- Moving to {wp_label} ---")

            # Compute duration from current target to waypoint
            max_distance = max(abs(wp_target - targets[n]) for n in motor_names)
            duration = max_distance / args.speed if args.speed > 0 else 0
            start_targets = dict(targets)
            t_wp_start = time.monotonic()

            while True:
                t_now = time.monotonic()
                elapsed = t_now - t_wp_start
                alpha = min(elapsed / duration, 1.0) if duration > 0 else 1.0

                line_parts = [f"{t_now - t_start:6.2f}"]

                for name in motor_names:
                    target = start_targets[name] + (wp_target - start_targets[name]) * alpha

                    # send_mit_command applies soft stop clamp + restoring torque
                    mgr.send_mit_command(
                        name, target, args.kp, args.kd,
                        actual_pos=last_pos.get(name))
                    fb = mgr.read_feedback(name, timeout=0.01)

                    if fb:
                        last_pos[name] = fb.position
                        j = mgr.joints[name]
                        zone = ""
                        if fb.position < j.motor_softstop_lo or fb.position > j.motor_softstop_hi:
                            zone = " ⚠️SS"
                        if fb.position < j.motor_cmd_lo or fb.position > j.motor_cmd_hi:
                            zone = " 🛑HARD"
                        err = target - fb.position
                        line_parts.append(
                            f"{name}: tgt={target:+7.3f} pos={fb.position:+7.3f} "
                            f"τ={fb.torque:+6.3f} err={err:+5.3f}{zone}")

                    targets[name] = target

                sys.stdout.write(f"\r\033[2K{'  '.join(line_parts)}")
                sys.stdout.flush()

                if alpha >= 1.0:
                    for n in motor_names:
                        targets[n] = wp_target
                    break
                time.sleep(dt)

            # Hold at waypoint
            print(f"\n  Hold {wp_label} for {args.hold}s")
            t_hold = time.monotonic()
            while time.monotonic() - t_hold < args.hold:
                line_parts = [f"{time.monotonic() - t_start:6.2f}"]
                for name in motor_names:
                    mgr.send_mit_command(
                        name, targets[name], args.kp, args.kd,
                        actual_pos=last_pos.get(name))
                    fb = mgr.read_feedback(name, timeout=0.01)
                    if fb:
                        last_pos[name] = fb.position
                        line_parts.append(
                            f"{name}: pos={fb.position:+7.3f} τ={fb.torque:+6.3f}")
                sys.stdout.write(f"\r\033[2K{'  '.join(line_parts)}")
                sys.stdout.flush()
                time.sleep(dt)

        print("\n\n✅ Jog complete.")

    except KeyboardInterrupt:
        print("\n\nStopped.")

    # Ramp to zero
    print("Ramping to zero...")
    for i in range(25):
        for name in motor_names:
            targets[name] *= 0.9
            mgr.send_mit_command(name, targets[name], args.kp, args.kd,
                                 actual_pos=last_pos.get(name))
            fb = mgr.read_feedback(name, timeout=0.01)
            if fb:
                last_pos[name] = fb.position
        time.sleep(dt)

    # Zero torque
    for name in motor_names:
        mgr.send_mit_command(name, 0.0, 0.0, 0.0)
        mgr.read_feedback(name, timeout=0.01)
    time.sleep(0.05)

    # Disable selected motors
    print("Disabling...")
    for name in motor_names:
        bus = mgr._bus_for(name)
        bus.disable(name)
    mgr.disconnect_all()
    print("Done.")


if __name__ == "__main__":
    main()
