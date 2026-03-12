#!/usr/bin/env python3
"""Scan CAN bus for RobStride motors — read-only, no actuation.

Pings each expected motor ID, reads position/velocity/voltage.
Use to verify wiring and CAN comms before enabling any motor.

Usage:
    python3 scan_motors.py              # scan can0 (single bus)
    python3 scan_motors.py can0 can1    # scan both buses
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    '..', 'biped_ws', 'src', 'biped_driver'))

from biped_driver.robstride_dynamics import RobstrideBus, Motor, CommunicationType, ParameterType

# Expected motor map
MOTOR_MAP = {
    "can0": [
        ("R_hip_pitch",   1, "rs-04"),
        ("R_hip_roll",    2, "rs-03"),
        ("R_hip_yaw",     3, "rs-03"),
        ("R_knee",        4, "rs-04"),
        ("R_foot_pitch",  5, "rs-02"),
        ("R_foot_roll",   6, "rs-02"),
        ("L_hip_pitch",   7, "rs-04"),
        ("L_hip_roll",    8, "rs-03"),
        ("L_hip_yaw",     9, "rs-03"),
        ("L_knee",       10, "rs-04"),
        ("L_foot_pitch", 11, "rs-02"),
        ("L_foot_roll",  12, "rs-02"),
    ],
    "can1": [
        ("L_hip_pitch",   7, "rs-04"),
        ("L_hip_roll",    8, "rs-03"),
        ("L_hip_yaw",     9, "rs-03"),
        ("L_knee",       10, "rs-04"),
        ("L_foot_pitch", 11, "rs-02"),
        ("L_foot_roll",  12, "rs-02"),
    ],
}

# Single bus mode: all 12 on can0
MOTOR_MAP_SINGLE = {
    "can0": MOTOR_MAP["can0"],
}


def scan_bus(channel: str, motors: list):
    """Ping and read from each motor on one CAN bus. Read-only — no enable, no MIT commands."""
    print(f"\n{'='*60}")
    print(f"  Scanning {channel} — {len(motors)} expected motors")
    print(f"{'='*60}")

    bus = RobstrideBus(channel=channel)
    bus.connect()

    found = 0
    failed = 0

    for name, motor_id, model in motors:
        # Ping: send GET_DEVICE_ID, check for response
        bus.transmit(CommunicationType.GET_DEVICE_ID, bus.host_id, motor_id)
        resp = bus.receive(timeout=0.1)

        if resp is None:
            print(f"  ❌ {name:15s}  ID={motor_id:2d}  ({model})  — NO RESPONSE")
            failed += 1
            continue

        found += 1

        # Read position via parameter read (no actuation)
        bus.motors[name] = Motor(id=motor_id, model=model)
        try:
            pos = bus.read_parameter(name, ParameterType.MECHANICAL_POSITION)
            vel = bus.read_parameter(name, ParameterType.MECHANICAL_VELOCITY)
            vbus = bus.read_parameter(name, ParameterType.VBUS)

            pos_str = f"{pos:+7.3f} rad" if pos is not None else "read fail"
            vel_str = f"{vel:+6.3f} rad/s" if vel is not None else "read fail"
            vbus_str = f"{vbus:.1f}V" if vbus is not None else "?"

            print(f"  ✅ {name:15s}  ID={motor_id:2d}  ({model})  "
                  f"pos={pos_str}  vel={vel_str}  vbus={vbus_str}")
        except Exception as e:
            print(f"  ⚠️  {name:15s}  ID={motor_id:2d}  ({model})  "
                  f"ping OK but read failed: {e}")

    bus.disconnect()

    print(f"\n  Result: {found}/{len(motors)} motors found, {failed} missing")
    return found, failed


def main():
    channels = sys.argv[1:] if len(sys.argv) > 1 else ["can0"]

    print("RobStride Motor Scanner — READ ONLY (no actuation)")
    print("Motors will NOT be enabled. Safe to run anytime.")

    total_found = 0
    total_failed = 0

    for ch in channels:
        # Use single-bus map for can0 if only scanning can0
        if len(channels) == 1 and ch == "can0":
            motors = MOTOR_MAP_SINGLE["can0"]
        elif ch in MOTOR_MAP:
            motors = MOTOR_MAP[ch]
        else:
            # Unknown bus — scan IDs 1-12
            motors = [(f"motor_{i}", i, "rs-04") for i in range(1, 13)]

        f, fail = scan_bus(ch, motors)
        total_found += f
        total_failed += fail

    print(f"\n{'='*60}")
    print(f"  TOTAL: {total_found} found, {total_failed} missing")
    if total_failed == 0 and total_found > 0:
        print("  ✅ All motors responding — ready for hardware.launch.py")
    elif total_found > 0:
        print("  ⚠️  Some motors missing — check wiring and CAN IDs")
    else:
        print("  ❌ No motors found — check power, CAN adapter, and wiring")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
