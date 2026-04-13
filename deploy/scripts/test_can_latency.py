#!/usr/bin/env python3
"""
CAN Bus Latency Tester for RobStride Motors (Full Python Stack)

This script uses the exact same `biped_driver` Python libraries that the
production ROS2 `can_bus_node` uses. This means it measures not just the
raw SPI/CAN hardware latency, but also the Python-side packing, unpacking,
calibration application, and object allocation overhead.

If this script shows > 1.5ms per motor, our 20ms control budget (50Hz)
for all 12 motors is severely compromised before the ONNX policy even runs.

Prerequisites:
- Ensure the CAN interfaces (can0, can1) are up and running.
"""

import sys
import os
import time
import yaml
import statistics

# Dynamically add the biped_driver package to Python path so we can import it
# just like the ROS2 node does.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BIPED_DRIVER_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, '../biped_ws/src/biped_driver'))
CONFIG_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, '../biped_ws/src/biped_bringup/config/robot.yaml'))

sys.path.insert(0, BIPED_DRIVER_PATH)

try:
    from biped_driver.robstride_can import BipedMotorManager
except ImportError as e:
    print(f"Failed to import BipedMotorManager: {e}")
    print(f"Ensure {BIPED_DRIVER_PATH} is correct and contains biped_driver module.")
    sys.exit(1)

# --- CONFIGURATION ---
TARGET_JOINT = "R_hip_pitch"  # Pick one normal joint to test
NUM_PINGS = 1000              # Number of round trips to measure
DELAY_BETWEEN_PINGS = 0.002   # 2ms wait between pings to mimic control loop breathing room

def main():
    print("==================================================")
    print("   RobStride Python Stack Latency Tester (CAN)    ")
    print("==================================================")

    # 1. Load the exact production motor configuration
    if not os.path.exists(CONFIG_PATH):
        print(f"Config file not found: {CONFIG_PATH}")
        sys.exit(1)

    print(f"Loading configuration from {CONFIG_PATH}...")
    with open(CONFIG_PATH, 'r') as f:
        config = yaml.safe_load(f)
    
    # Check if 'biped_hardware' root key exists, otherwise assume root is the config
    hardware_cfg = config.get("biped_hardware", config)
    
    # 2. Initialize the Manager
    # We pass empty offsets here so it defaults to theoretical limits.
    # The packing/unpacking math latency remains identical.
    manager = BipedMotorManager.from_robot_yaml(hardware_cfg, offsets={})
    
    if TARGET_JOINT not in manager.joints:
        print(f"Error: Target joint '{TARGET_JOINT}' not found in robot.yaml!")
        sys.exit(1)
        
    print(f"Connecting to CAN buses...")
    try:
        manager.connect_all()
        # Enable MIT mode on all motors to prepare for commands
        manager.enable_all()
    except Exception as e:
        print(f"Failed to connect or enable motors: {e}")
        print("Check if can0/can1 are up and motors are powered.")
        sys.exit(1)

    # Allow motors a moment to transition states
    time.sleep(0.5)
    
    # Flush any lingering startup messages from the bus
    manager.flush_all()

    print(f"\nTesting Latency on Joint: {TARGET_JOINT}")
    print(f"Executing {NUM_PINGS} zero-torque MIT commands...\n")

    latencies = []
    timeouts = 0

    for i in range(NUM_PINGS):
        # Start clock right before packing and sending the command
        t_start = time.perf_counter()
        
        try:
            # Send a pure 0-torque command (Kp=0, Kd=0, Pos=0, Vel=0, FF=0)
            # This forces the motor to reply with its state without moving.
            manager.send_mit_command(
                joint_name=TARGET_JOINT,
                position=0.0,
                kp=0.0,
                kd=0.0,
                velocity=0.0,
                torque_ff=0.0,
                actual_pos=0.0 # Bypasses soft-stop math to isolate protocol latency
            )
            
            # Blocking read to wait for the motor's state feedback frame
            # Timeout is 10ms (plenty of time for a single motor)
            feedback = manager.read_feedback(TARGET_JOINT, timeout=0.010)
            
            # Stop clock the moment the Python Feedback object is fully unpacked
            t_end = time.perf_counter()
            
            if feedback is None:
                timeouts += 1
            else:
                rtt_ms = (t_end - t_start) * 1000.0
                latencies.append(rtt_ms)
                
        except Exception as e:
            print(f"Error on ping {i}: {e}")

        # Sleep to simulate the gap between motor polls in a real control loop
        time.sleep(DELAY_BETWEEN_PINGS)

    # Cleanup
    manager.disable_all()
    manager.disconnect_all()

    if not latencies:
        print(f"\n[ERROR] All {NUM_PINGS} pings timed out. No data received.")
        return

    # --- CALCULATE STATISTICS ---
    min_lat = min(latencies)
    max_lat = max(latencies)
    avg_lat = statistics.mean(latencies)
    
    latencies.sort()
    p99_idx = int(len(latencies) * 0.99)
    p99_lat = latencies[p99_idx]

    print("\n==================================================")
    print("      Python/CAN Latency Results (Round-Trip)     ")
    print("==================================================")
    print(f"Total Pings: {len(latencies)} successful, {timeouts} timeouts")
    print(f"Minimum:     {min_lat:.3f} ms")
    print(f"Average:     {avg_lat:.3f} ms")
    print(f"Maximum:     {max_lat:.3f} ms")
    print(f"99th %ile:   {p99_lat:.3f} ms")
    print("==================================================")
    
    if avg_lat > 1.5:
        print("\n[CRITICAL WARNING] Average RTT is > 1.5ms per motor.")
        print(f"Querying all 12 motors sequentially will take ~{avg_lat * 12:.2f}ms.")
        print("This destroys the 20ms control budget before the policy even runs.")
    elif avg_lat > 0.8:
        print("\n[WARNING] Average RTT is somewhat high (0.8 - 1.5ms).")
        print("You have very little headroom left for IMU I2C reads and ONNX inference.")
    else:
        print("\n[OK] Latency is well within budget for a 50Hz control loop.")


if __name__ == "__main__":
    main()
