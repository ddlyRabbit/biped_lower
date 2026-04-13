#!/usr/bin/env python3
"""
IMU Latency Tester for BNO085 (I2C)

This script bypasses ROS2 entirely and directly uses the Adafruit CircuitPython
library (standard for BNO085 on a Raspberry Pi) to measure the raw hardware I2C 
read time for the sensor. 

The primary goal is to determine how much of our 20ms control loop budget is 
being eaten simply by asking the IMU for the current gravity vector.

Prerequisites on the Pi:
- I2C enabled (sudo raspi-config)
- pip install adafruit-circuitpython-bno08x
"""

import time
import statistics
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

# --- CONFIGURATION ---
NUM_READS = 1000            # Total number of times to poll the IMU
REPORT_INTERVAL_S = 0.0033  # Tell the IMU to prepare a new report every ~3.3ms (300Hz)
DELAY_BETWEEN_READS = 0.0033  # Simulate the actual 300Hz IMU node loop


def main():
    print("--- Initializing I2C bus ---")
    try:
        # board.I2C() uses the default SCL/SDA pins on the Raspberry Pi (I2C bus 1)
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
        
        print("--- Connecting to BNO085 at 0x4B ---")
        # The BNO085 default I2C address is usually 0x4A, but wired to 0x4B (75) on this robot
        bno = BNO08X_I2C(i2c, address=0x4B)
    except Exception as e:
        print(f"Failed to initialize I2C or find BNO085: {e}")
        print("Check if the sensor is wired correctly and I2C is enabled.")
        return

    print(f"--- Enabling Rotation Vector report at {1/REPORT_INTERVAL_S}Hz ---")
    # Enable the specific quaternion report we use to project gravity in the observation builder
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    
    # Give the sensor half a second to settle and begin populating its FIFO buffer
    time.sleep(0.5)

    latencies = []
    print(f"\nMeasuring I2C read time over {NUM_READS} iterations (simulating 50Hz loop)...")

    for i in range(NUM_READS):
        # We sleep 20ms to roughly simulate the time between control loop ticks
        time.sleep(DELAY_BETWEEN_READS)
        
        # Start the clock immediately before asking for data
        t_start = time.perf_counter()
        
        try:
            # BLOCKING I2C READ: This pulls the 4 quaternion floats over the I2C wires.
            # BNO085 is known for "clock stretching" (holding the clock line low until it's ready),
            # which forces the Pi's CPU to wait. We are measuring exactly how long that wait is.
            quat = bno.quaternion
            
            # Stop the clock the moment the data is fully in Python memory
            t_end = time.perf_counter()
            
            if quat is not None:
                # Convert from seconds to milliseconds
                read_time_ms = (t_end - t_start) * 1000.0
                latencies.append(read_time_ms)
            else:
                print(f"Warning on read {i}: Data not ready in FIFO buffer.")
                
        except Exception as e:
            print(f"I2C read error on iteration {i}: {e}")

    if not latencies:
        print("No successful reads recorded. Check wiring and sensor status.")
        return

    # --- CALCULATE STATISTICS ---
    min_lat = min(latencies)
    max_lat = max(latencies)
    avg_lat = statistics.mean(latencies)
    
    # 99th percentile (sort and take the value at the 99% index)
    # This shows us the worst-case jitter, filtering out extreme 1-off outliers
    latencies.sort()
    p99_idx = int(len(latencies) * 0.99)
    p99_lat = latencies[p99_idx]

    print("\n==================================================")
    print("      IMU (BNO085) I2C Read Latency Results       ")
    print("==================================================")
    print(f"Total Reads: {len(latencies)} / {NUM_READS}")
    print(f"Minimum:     {min_lat:.3f} ms")
    print(f"Average:     {avg_lat:.3f} ms")
    print(f"Maximum:     {max_lat:.3f} ms")
    print(f"99th %ile:   {p99_lat:.3f} ms")
    print("==================================================")

    # Contextual warnings based on our 20ms total budget
    if avg_lat > 3.0:
        print("\n[WARNING] Average read time > 3.0ms.")
        print("This is eating > 15% of your 20ms control budget just to get gravity.")
    if p99_lat > 6.0:
        print("\n[CRITICAL WARNING] 99th percentile jitter is > 6.0ms.")
        print("The BNO085 is clock-stretching significantly. This will cause unpredictable")
        print("spikes in observation age and severely degrade policy performance.")


if __name__ == "__main__":
    main()
