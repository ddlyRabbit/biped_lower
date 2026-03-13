#!/usr/bin/env python3
"""Test hip yaw motors — slow jog with soft stop verification.

Jogs R_hip_yaw and L_hip_yaw from 0 → +limit → 0 → -limit → 0
at a slow rate, printing position and torque feedback.

Usage (on RPi, with CAN up):
    python3 test_hip_yaw.py
    python3 test_hip_yaw.py --kp 2.0 --speed 0.3
    python3 test_hip_yaw.py --motor R_hip_yaw   # single motor
"""

import argparse
import math
import sys
import time
import yaml

sys.path.insert(0, "/home/roy/biped_lower/deploy/biped_ws/src/biped_driver")
from biped_driver.robstride_dynamics import RobstrideBus, Motor

MOTORS = {
    "R_hip_yaw": {"id": 3, "model": "rs-03"},
    "L_hip_yaw": {"id": 9, "model": "rs-03"},
}

# From calibration.yaml
CALIBRATION = {
    "R_hip_yaw": {"offset": 3.9984, "motor_min": 2.4276, "motor_max": 5.5831},
    "L_hip_yaw": {"offset": 4.5725, "motor_min": 3.0017, "motor_max": 6.1622},
}

SOFTSTOP_BUFFER = math.radians(2.0)  # 0.0349 rad
SOFTSTOP_KP = 20.0


def load_calibration():
    """Try to load calibration.yaml, fall back to hardcoded values."""
    try:
        with open("/home/roy/biped_lower/deploy/biped_ws/calibration.yaml") as f:
            cal = yaml.safe_load(f)
        for name in MOTORS:
            if name in cal:
                CALIBRATION[name] = {
                    "offset": cal[name].get("offset", CALIBRATION[name]["offset"]),
                    "motor_min": cal[name].get("motor_min", CALIBRATION[name]["motor_min"]),
                    "motor_max": cal[name].get("motor_max", CALIBRATION[name]["motor_max"]),
                }
        print("Loaded calibration.yaml")
    except Exception as e:
        print(f"Using hardcoded calibration: {e}")


def compute_limits(name):
    """Compute command-space limits from calibration."""
    cal = CALIBRATION[name]
    offset = cal["offset"]
    cmd_lo = cal["motor_min"] - offset
    cmd_hi = cal["motor_max"] - offset
    ss_lo = cmd_lo + SOFTSTOP_BUFFER
    ss_hi = cmd_hi - SOFTSTOP_BUFFER
    return cmd_lo, cmd_hi, ss_lo, ss_hi


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

    load_calibration()

    # Select motors
    if args.motor == "both":
        motor_names = list(MOTORS.keys())
    elif args.motor in MOTORS:
        motor_names = [args.motor]
    else:
        print(f"Unknown motor: {args.motor}")
        return

    # Build bus
    motors = {}
    calibration = {}
    for name in motor_names:
        m = MOTORS[name]
        cal = CALIBRATION[name]
        motors[name] = Motor(id=m["id"], model=m["model"])
        calibration[name] = {"direction": 1, "homing_offset": cal["offset"]}

    bus = RobstrideBus(channel="can0", motors=motors, calibration=calibration)
    bus.connect()

    # Print limits
    for name in motor_names:
        cmd_lo, cmd_hi, ss_lo, ss_hi = compute_limits(name)
        print(f"\n{name}:")
        print(f"  Hard limits: [{cmd_lo:+.4f}, {cmd_hi:+.4f}] "
              f"([{math.degrees(cmd_lo):+.1f}°, {math.degrees(cmd_hi):+.1f}°])")
        print(f"  Soft stops:  [{ss_lo:+.4f}, {ss_hi:+.4f}] "
              f"([{math.degrees(ss_lo):+.1f}°, {math.degrees(ss_hi):+.1f}°])")

    # Enable motors
    print("\nEnabling motors...")
    for name in motor_names:
        bus.enable(name)
        time.sleep(0.02)
        bus.set_mode(name, 0)
        time.sleep(0.02)
    bus.flush_rx()

    # Read initial positions
    print("\nReading initial positions...")
    for name in motor_names:
        bus.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
        fb = bus.read_operation_frame(name, timeout=0.05)
        if fb:
            print(f"  {name}: pos={fb.position:+.4f} ({math.degrees(fb.position):+.1f}°) "
                  f"torque={fb.torque:+.3f} Nm  temp={fb.temperature:.1f}°C")
        else:
            print(f"  {name}: NO RESPONSE")

    # Compute jog target
    if args.max_angle is not None:
        max_rad = math.radians(args.max_angle)
    else:
        # Use the smaller of the two soft stop limits
        max_rad = min(abs(compute_limits(n)[2]) for n in motor_names)
        max_rad = min(max_rad, abs(compute_limits(motor_names[0])[3]))

    print(f"\nJog parameters: kp={args.kp}, kd={args.kd}, speed={args.speed} rad/s")
    print(f"Max angle: ±{math.degrees(max_rad):.1f}° ({max_rad:.4f} rad)")
    print(f"Sequence: 0 → +{math.degrees(max_rad):.0f}° → 0 → -{math.degrees(max_rad):.0f}° → 0")
    print(f"\nPress Ctrl+C to stop (motors will be disabled)\n")

    # Jog sequence: 0 → +max → 0 → -max → 0
    dt = 0.02  # 50Hz
    targets = {n: 0.0 for n in motor_names}

    # Build waypoints: (target_rad, label)
    waypoints = [
        (+max_rad, f"+{math.degrees(max_rad):.0f}°"),
        (0.0, "0°"),
        (-max_rad, f"-{math.degrees(max_rad):.0f}°"),
        (0.0, "0°"),
    ]

    header = f"{'Time':>6}  "
    for name in motor_names:
        header += f"{'target':>8} {'actual':>8} {'torque':>7} {'err':>6}  "
    print(header)
    print("-" * len(header))

    try:
        t_start = time.monotonic()
        for wp_target, wp_label in waypoints:
            print(f"\n--- Moving to {wp_label} ---")
            for name in motor_names:
                start_pos = targets[name]
                distance = wp_target - start_pos
                duration = abs(distance) / args.speed if args.speed > 0 else 0
                t_wp_start = time.monotonic()

                while True:
                    t_now = time.monotonic()
                    elapsed = t_now - t_wp_start

                    if duration > 0:
                        alpha = min(elapsed / duration, 1.0)
                    else:
                        alpha = 1.0
                    target = start_pos + distance * alpha

                    # Send command
                    bus.write_operation_frame(name, target, args.kp, args.kd, 0.0, 0.0)
                    fb = bus.read_operation_frame(name, timeout=0.01)

                    if fb:
                        err = target - fb.position
                        t_rel = t_now - t_start
                        line = f"{t_rel:6.2f}  "
                        line += (f"{target:+8.4f} {fb.position:+8.4f} "
                                 f"{fb.torque:+7.3f} {err:+6.3f}  ")
                        # Show soft stop zone indicator
                        cmd_lo, cmd_hi, ss_lo, ss_hi = compute_limits(name)
                        if fb.position < ss_lo or fb.position > ss_hi:
                            line += " ⚠️ SOFTSTOP"
                        if fb.position < cmd_lo or fb.position > cmd_hi:
                            line += " 🛑 HARD LIMIT"
                        sys.stdout.write(f"\r\033[2K{line}")
                        sys.stdout.flush()

                    if alpha >= 1.0:
                        targets[name] = wp_target
                        break

                    time.sleep(dt)

            # Hold at waypoint
            print(f"\n  Holding at {wp_label} for {args.hold}s...")
            t_hold_start = time.monotonic()
            while time.monotonic() - t_hold_start < args.hold:
                for name in motor_names:
                    bus.write_operation_frame(name, targets[name], args.kp, args.kd, 0.0, 0.0)
                    fb = bus.read_operation_frame(name, timeout=0.01)
                    if fb:
                        cmd_lo, cmd_hi, ss_lo, ss_hi = compute_limits(name)
                        zone = ""
                        if fb.position < ss_lo or fb.position > ss_hi:
                            zone = " ⚠️ SOFTSTOP"
                        t_rel = time.monotonic() - t_start
                        line = (f"{t_rel:6.2f}  {name}: "
                                f"tgt={targets[name]:+.4f} "
                                f"pos={fb.position:+.4f} "
                                f"τ={fb.torque:+.3f}{zone}")
                        sys.stdout.write(f"\r\033[2K{line}")
                        sys.stdout.flush()
                time.sleep(dt)

        print("\n\n✅ Jog sequence complete.")

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    # Ramp to zero then disable
    print("Ramping to zero...")
    for _ in range(25):  # 0.5s ramp
        for name in motor_names:
            targets[name] *= 0.9
            bus.write_operation_frame(name, targets[name], args.kp, args.kd, 0.0, 0.0)
            bus.read_operation_frame(name, timeout=0.01)
        time.sleep(dt)

    # Zero torque then disable
    for name in motor_names:
        bus.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
        bus.read_operation_frame(name, timeout=0.01)
    time.sleep(0.05)

    print("Disabling motors...")
    for name in motor_names:
        bus.disable(name)
    bus.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
