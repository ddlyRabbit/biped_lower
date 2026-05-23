#!/usr/bin/env python3
"""Sniff and decode RobStride MIT control commands transmitted on the CAN buses."""

import argparse
import sys
import os
import yaml
import can
import time
import struct
import numpy as np

# Standalone tables for decoding (no ROS dependencies)
MODEL_MIT_POSITION_TABLE = {
    "rs-00": 4 * np.pi, "rs-01": 4 * np.pi, "rs-02": 4 * np.pi,
    "rs-03": 4 * np.pi, "rs-04": 4 * np.pi, "rs-05": 4 * np.pi,
    "rs-06": 4 * np.pi,
}

MODEL_MIT_VELOCITY_TABLE = {
    "rs-00": 50, "rs-01": 44, "rs-02": 44,
    "rs-03": 50, "rs-04": 15, "rs-05": 33,
    "rs-06": 20,
}

MODEL_MIT_TORQUE_TABLE = {
    "rs-00": 17, "rs-01": 17, "rs-02": 17,
    "rs-03": 60, "rs-04": 120, "rs-05": 17,
    "rs-06": 60,
}

MODEL_MIT_KP_TABLE = {
    "rs-00": 500.0, "rs-01": 500.0, "rs-02": 500.0,
    "rs-03": 5000.0, "rs-04": 5000.0, "rs-05": 500.0,
    "rs-06": 5000.0,
}

MODEL_MIT_KD_TABLE = {
    "rs-00": 5.0, "rs-01": 5.0, "rs-02": 5.0,
    "rs-03": 100.0, "rs-04": 100.0, "rs-05": 5.0,
    "rs-06": 100.0,
}

def load_bus_map(config_path: str):
    with open(config_path) as f:
        config = yaml.safe_load(f)

    bus_map = {}
    id_to_motor = {}
    for bus_key, bus_cfg in config.items():
        interface = bus_cfg.get("interface", bus_key)
        motors = []
        for name, mcfg in bus_cfg.get("motors", {}).items():
            model = mcfg["type"].lower()
            if not model.startswith("rs-"):
                model = "rs-" + model[2:]
            motors.append((name, mcfg["id"], model))
            id_to_motor[mcfg["id"]] = (name, model)
        bus_map[interface] = motors

    return bus_map, id_to_motor

def parse_operation_frame(ext_id, data, motor_info):
    motor_name, model = motor_info
    
    comm_type = (ext_id >> 24) & 0x1F
    trq_u16 = (ext_id >> 8) & 0xFFFF
    
    if comm_type != 1:  # OPERATION_CONTROL (Host -> Motor)
        return None
        
    pos_u16, vel_u16, kp_u16, kd_u16 = struct.unpack(">HHHH", data)
    
    pos_max = MODEL_MIT_POSITION_TABLE[model]
    vel_max = MODEL_MIT_VELOCITY_TABLE[model]
    kp_max = MODEL_MIT_KP_TABLE[model]
    kd_max = MODEL_MIT_KD_TABLE[model]
    trq_max = MODEL_MIT_TORQUE_TABLE[model]
    
    pos = (pos_u16 / 0x7FFF - 1.0) * pos_max
    vel = (vel_u16 / 0x7FFF - 1.0) * vel_max
    kp = (kp_u16 / 0xFFFF) * kp_max
    kd = (kd_u16 / 0xFFFF) * kd_max
    trq = (trq_u16 / 0x7FFF - 1.0) * trq_max
    
    return {
        "motor": motor_name,
        "pos": pos,
        "vel": vel,
        "kp": kp,
        "kd": kd,
        "tau_ff": trq
    }

def main():
    parser = argparse.ArgumentParser(description="Sniff MIT control commands on CAN buses")
    parser.add_argument("--config", default="../biped_ws/src/biped_bringup/config/robot.yaml")
    parser.add_argument("--hz", type=float, default=5.0, help="Print frequency per motor (Hz)")
    args = parser.parse_args()
    
    config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), args.config))
    if not os.path.exists(config_path):
        print(f"Error: Config file not found at {config_path}")
        return
        
    bus_map, id_to_motor = load_bus_map(config_path)
    
    buses = []
    for interface in bus_map.keys():
        try:
            buses.append(can.interface.Bus(channel=interface, bustype='socketcan'))
            print(f"Listening on {interface}")
        except Exception as e:
            print(f"Failed to open {interface}: {e}")
            
    if not buses:
        print("No buses available. Exiting.")
        return

    print(f"\n{'Motor':<15} | {'Pos (rad)':<10} | {'Vel':<8} | {'Kp':<6} | {'Kd':<6} | {'Tau_FF':<8}")
    print("-" * 65)
    
    last_print = {}
    print_interval = 1.0 / args.hz

    try:
        while True:
            for bus in buses:
                msg = bus.recv(timeout=0.001)
                if msg is None or not msg.is_extended_id:
                    continue
                    
                device_id = msg.arbitration_id & 0xFF
                comm_type = (msg.arbitration_id >> 24) & 0x1F
                
                # Check for Host->Motor Operation Control Frame
                if comm_type == 1 and device_id in id_to_motor:
                    parsed = parse_operation_frame(msg.arbitration_id, msg.data, id_to_motor[device_id])
                    if parsed:
                        motor_name = parsed['motor']
                        now = time.time()
                        if now - last_print.get(motor_name, 0) > print_interval:
                            print(f"{motor_name:<15} | {parsed['pos']:>10.4f} | {parsed['vel']:>8.2f} | {parsed['kp']:>6.1f} | {parsed['kd']:>6.2f} | {parsed['tau_ff']:>8.3f}")
                            last_print[motor_name] = now

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        for bus in buses:
            bus.shutdown()

if __name__ == "__main__":
    main()
