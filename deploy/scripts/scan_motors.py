#!/usr/bin/env python3
"""Scan CAN buses for RobStride motors — read-only, no actuation.

Pings each expected motor ID, reads position/velocity/voltage.
Use to verify wiring and CAN comms before enabling any motor.

Usage:
    python3 scan_motors.py                          # scan all buses from robot.yaml
    python3 scan_motors.py --config path/to/robot.yaml
    python3 scan_motors.py can0                     # scan can0 only
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    "..", "biped_ws", "src", "biped_driver"))

import yaml
from biped_driver.robstride_dynamics import RobstrideBus, Motor, CommunicationType, ParameterType

DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__),
    "..", "biped_ws", "src", "biped_bringup", "config", "robot.yaml")


def load_bus_map(config_path: str) -> dict[str, list[tuple[str, int, str]]]:
    """Parse robot.yaml into {bus_name: [(name, id, model), ...]}."""
    with open(config_path) as f:
        config = yaml.safe_load(f)

    bus_map = {}
    for bus_key, bus_cfg in config.items():
        interface = bus_cfg.get("interface", bus_key)
        motors = []
        for name, mcfg in bus_cfg.get("motors", {}).items():
            model = mcfg["type"].lower()
            if not model.startswith("rs-"):
                model = f"rs-{model[2:]}"  # RS04 -> rs-04
            motors.append((name, mcfg["id"], model))
        motors.sort(key=lambda m: m[1])  # sort by ID
        bus_map[interface] = motors

    return bus_map


def scan_bus(channel: str, motors: list):
    """Ping and read from each motor on one CAN bus. Read-only."""
    print(f"\n{=*60}")
    print(f"  Scanning {channel} — {len(motors)} expected motors")
    print(f"{=*60}")

    bus = RobstrideBus(channel=channel)
    bus.connect()

    found = 0
    failed = 0

    for name, motor_id, model in motors:
        bus.transmit(CommunicationType.GET_DEVICE_ID, bus.host_id, motor_id)
        resp = bus.receive(timeout=0.1)

        if resp is None:
            print(f"  ❌ {name:15s}  ID={motor_id:2d}  ({model})  — NO RESPONSE")
            failed += 1
            continue

        found += 1

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
    parser = argparse.ArgumentParser(description="Scan CAN buses for RobStride motors")
    parser.add_argument("buses", nargs="*", help="Bus(es) to scan (e.g. can0 can1). Default: all from config.")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="Path to robot.yaml")
    args = parser.parse_args()

    bus_map = load_bus_map(args.config)

    channels = args.buses if args.buses else sorted(bus_map.keys())

    print("RobStride Motor Scanner — READ ONLY (no actuation)")
    print("Motors will NOT be enabled. Safe to run anytime.")
    print(f"Config: {args.config}")

    total_found = 0
    total_failed = 0

    for ch in channels:
        motors = bus_map.get(ch)
        if motors is None:
            print(f"\n  ⚠️  Bus {ch} not in config — available: {, .join(bus_map.keys())}")
            continue
        found, failed = scan_bus(ch, motors)
        total_found += found
        total_failed += failed

    total = total_found + total_failed
    print(f"\n{=*60}")
    print(f"  TOTAL: {total_found}/{total} found, {total_failed} missing")
    if total_failed == 0 and total_found > 0:
        print("  ✅ All motors responding — ready for bringup")
    elif total_found > 0:
        print("  ⚠️  Some motors missing — check wiring and CAN IDs")
    else:
        print("  ❌ No motors found — check power, CAN adapter, and wiring")
    print(f"{=*60}")


if __name__ == "__main__":
    main()
