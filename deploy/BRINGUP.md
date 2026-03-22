# Biped Robot — Bringup Guide

## Prerequisites

- RPi 5 with Ubuntu 24.04 + ROS2 Jazzy
- Waveshare RS485 CAN HAT (B) — MCP2515 on SPI0
- BNO085 IMU on I2C bus 1 (addr 0x4B, RST on GPIO 4)
- All 12 motors on single CAN bus (can0), IDs 1–12
- Student policy exported as ONNX (`student_flat.onnx`)

## CAN HAT Setup (one-time)

Add to `/boot/firmware/config.txt`:
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
```
Reboot. Verify: `ls /sys/class/net/ | grep can` → should show `can0`.

## Step 1: Bring Up CAN (every boot)

```bash
cd ~/biped_lower/deploy
./scripts/setup_can.sh
```

Verify: `candump can0` — shows nothing if no motors powered, frames if powered.

## Step 2: Scan Motors

Power on motors, then:
```bash
python3 scripts/scan_motors.py
```

All 12 motors should respond with position/velocity/voltage readings.

## Step 3: Calibrate (once per assembly)

```bash
ros2 launch biped_bringup calibrate.launch.py
```

To recalibrate a single joint (merges into existing calibration):
```bash
ros2 launch biped_bringup calibrate.launch.py joint:=R_knee
ros2 launch biped_bringup calibrate.launch.py joint:=L_foot_top    # left ankle upper motor
ros2 launch biped_bringup calibrate.launch.py joint:=L_foot_bottom # left ankle lower motor
```

Move each joint to both mechanical limits. Joints auto-mark ✅ when range matches
URDF limits ±20%. Press Ctrl+C to save `calibration.yaml`.

## Step 4: Test Hardware (no policy)

```bash
ros2 launch biped_bringup hardware.launch.py \
  calibration_file:=calibration.yaml

# In another terminal:
ros2 topic echo /joint_states     # motor positions + velocities
ros2 topic echo /imu/data         # IMU quaternion + gyro
ros2 topic echo /safety/status    # should be True
```

## Step 5: Suspended Test (robot hanging, feet off ground)

⚠️ **Robot must be suspended!**

```bash
# Terminal 1: full robot with low gains
ros2 launch biped_bringup bringup.launch.py \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/student_flat.onnx \
  gain_scale:=0.3

# Terminal 2: monitoring
ros2 launch biped_bringup record.launch.py

# Terminal 3: start standing
ros2 topic pub --once /state_command std_msgs/String "data: START"

# Connect Foxglove Studio on desktop to ws://<pi-ip>:8765
```

Robot should slowly move to standing pose over 2 seconds (soft start).

## Step 6: Ground Test

After suspended test looks good:
```bash
# Same as Step 5 but increase gains
ros2 launch biped_bringup bringup.launch.py \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/student_flat.onnx \
  gain_scale:=0.5

# Start standing, then walking:
ros2 topic pub --once /state_command std_msgs/String "data: START"
# Wait for stable stand...
ros2 topic pub --once /state_command std_msgs/String "data: WALK"

# Keyboard teleop:
ros2 run biped_teleop keyboard_teleop
# Press 1 for 0.1 m/s, then w to go forward
```

## Step 7: Emergency Stop

```bash
ros2 topic pub --once /state_command std_msgs/String "data: ESTOP"
```

Also triggers automatically on pitch/roll violation or motor fault.
Reset: `ros2 topic pub --once /state_command std_msgs/String "data: RESET"`

## Gain Tuning

Use `gain_scale` launch argument to scale all PD gains uniformly:

| Scale | Use case |
|-------|----------|
| 0.3 | First suspended test |
| 0.5 | Suspended walking test |
| 0.7 | Ground standing test |
| 1.0 | Full sim gains |

Safety pitch/roll limits default to 85° (adjustable via `max_pitch_deg` / `max_roll_deg` launch args).

## Launch Files

| Launch | Purpose |
|--------|---------|
| `hardware.launch.py` | IMU + CAN + safety (no policy) |
| `calibrate.launch.py` | Interactive joint calibration |
| `bringup.launch.py` | Full robot (all nodes) |
| `record.launch.py` | Foxglove bridge + rosbag recording |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No CAN device | Run `setup_can.sh`, check dtoverlay in `/boot/firmware/config.txt` |
| Motor no response | Check 48V power, run `scan_motors.py`, verify CAN ID |
| IMU init fails | Power cycle BNO085, check I2C: `i2cdetect -y 1` (expect 0x4B) |
| Robot falls immediately | Reduce `gain_scale`, verify calibration offsets |
| ESTOP triggered | Check: `ros2 topic echo /safety/fault` |
| Foxglove won't connect | Check Pi IP, ensure port 8765 not blocked |
| TX buffer full errors | Normal at startup (MCP2515 has 3 slots), retries handle it |
| Ankle motors track poorly | Re-calibrate ankles, check linkage rod lengths match CAD |

## CAN Driver Options

Three CAN driver implementations available:

| Driver | Language | Loop Rate | Launch Param |
|--------|----------|-----------|--------------|
| `can_bus_node` | Python (sync) | ~50Hz | `can_driver:=can_bus_node` (default) |
| `can_bus_node_async` | Python (async) | ~200Hz | `can_driver:=can_bus_node_async` |
| `can_bus_node_cpp` | C++ | ~300Hz | `can_driver:=can_bus_node_cpp` |

```bash
# Default (Python sync):
ros2 launch biped_bringup bringup.launch.py \
  calibration_file:=calibration.yaml

# C++ driver (recommended):
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  calibration_file:=calibration.yaml

# Python async:
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_async \
  calibration_file:=calibration.yaml
```

All drivers share the same ROS2 interface:
- Sub: `/joint_commands` (biped_msgs/MITCommandArray)
- Pub: `/joint_states` (sensor_msgs/JointState)
- Pub: `/motor_states` (biped_msgs/MotorStateArray)

The C++ driver (`biped_driver_cpp` package) uses direct SocketCAN syscalls
with two worker threads (one per CAN bus) for lowest latency.
Stats printed every 5s showing actual loop Hz per bus.

### Build

```bash
cd ~/biped_lower/deploy/biped_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select biped_driver_cpp biped_bringup
source setup_biped.bash
```

## Wiggle Test

Test individual or all joints with sine wave sweeps. Robot must be in STAND state first.

### Config

Edit `deploy/biped_ws/src/biped_bringup/config/wiggle.yaml`:
```yaml
wiggle:
  duration_per_joint: 3.0    # seconds per joint in sequential mode

  joints:
    R_hip_pitch:   { pos: 0.087, neg: -0.087, freq: 1.0 }
    R_knee:        { pos: 0.15,  neg: -0.05,  freq: 0.5 }
    # pos: max offset above default (rad)
    # neg: max offset below default (rad, negative value)
    # freq: sine wave frequency (Hz)
```

Config is **re-read on each WIGGLE command** — edit and re-trigger without restart.

### Commands

```bash
# Start robot
ros2 topic pub --once /state_command std_msgs/String "data: START"
# Wait for STAND to stabilize, then:

# Sequential: one joint at a time (cycles through all 12)
ros2 topic pub --once /state_command std_msgs/String "data: WIGGLE_SEQ"

# All joints simultaneously
ros2 topic pub --once /state_command std_msgs/String "data: WIGGLE_ALL"

# Stop wiggling, return to STAND
ros2 topic pub --once /state_command std_msgs/String "data: STOP"
```

### Safety
- Joint targets clamped to URDF limits
- Config validated at load (warns if range exceeds limits)
- ESTOP always available
- gain_scale from launch applies to wiggle PD gains
