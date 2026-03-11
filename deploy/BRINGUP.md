# Biped Robot — Bringup Guide

## Prerequisites

- RPi 5 with Ubuntu 24.04 + ROS2 Jazzy
- 2-CH CAN HAT (MCP2515) configured in `/boot/firmware/config.txt`
- BNO085 IMU on I2C bus 1 (addr 0x4B, RST on GPIO 4)
- Motor CAN IDs set: Right 1-6 (can0), Left 7-12 (can1)
- Student policy exported as ONNX

## CAN HAT Setup (one-time)

Add to `/boot/firmware/config.txt`:
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
dtoverlay=mcp2515-can1,oscillator=12000000,interrupt=24
```
Reboot. Verify: `ls /sys/class/net/ | grep can`

## Launch Files

| Launch | Purpose | When to use |
|--------|---------|-------------|
| `hardware.launch.py` | IMU + CAN × 2 + safety | Test hardware without policy |
| `calibrate.launch.py` | Interactive joint calibration | Once, or after motor reassembly |
| `bringup.launch.py` | Full robot (all nodes) | Normal operation |
| `record.launch.py` | Foxglove bridge + rosbag | Monitoring + data collection |

## Step 1: Setup CAN (every boot)

```bash
cd ~/biped_lower/deploy
./scripts/setup_can.sh
```

Verify: `candump can0` (shows nothing if no motors powered, frames if powered)

## Step 2: Calibrate (once per assembly)

Power on motors. Run per side:
```bash
ros2 launch biped_bringup calibrate.launch.py side:=right
ros2 launch biped_bringup calibrate.launch.py side:=left
```

Follow interactive prompts — move each joint to both mechanical limits.
Saves `calibration_right.yaml` and `calibration_left.yaml`.

## Step 3: Test hardware (no policy)

```bash
ros2 launch biped_bringup hardware.launch.py

# In another terminal:
ros2 topic echo /joint_states     # motor positions + velocities
ros2 topic echo /imu/data         # IMU quaternion + gyro
ros2 topic echo /safety/status    # should be True
```

## Step 4: Test policy (robot suspended)

**⚠️ Robot must be hanging / feet off ground!**

```bash
# Terminal 1: full robot with low gains
ros2 launch biped_bringup bringup.launch.py \
  onnx_model:=~/biped_lower/deploy/student_flat.onnx \
  gain_scale:=0.3

# Terminal 2: monitoring
ros2 launch biped_bringup record.launch.py

# Terminal 3: start standing
ros2 topic pub --once /state_command std_msgs/String "data: START"

# Connect Foxglove Studio on desktop to ws://<pi-ip>:8765
```

Robot should slowly move to standing pose over 2 seconds (soft start).

## Step 5: Walking

```bash
# After standing is stable:
ros2 topic pub --once /state_command std_msgs/String "data: WALK"

# Keyboard teleop in another terminal:
ros2 run biped_teleop keyboard_teleop

# Press 1 for 0.1 m/s, then w to go forward
```

## Step 6: Emergency stop

```bash
ros2 topic pub --once /state_command std_msgs/String "data: ESTOP"
```
Also triggers automatically on pitch/roll violation or motor fault.

Reset: `ros2 topic pub --once /state_command std_msgs/String "data: RESET"`

## Gain Tuning

Start with `gain_scale:=0.3`. Increase gradually:
```
0.3  →  test standing stability
0.5  →  test slow walking
0.7  →  test normal walking
1.0  →  full sim gains
```

## Motor Config

```
CAN0 (Right)              CAN1 (Left)
R_hip_pitch   1  RS04     L_hip_pitch   7  RS04
R_hip_roll    2  RS03     L_hip_roll    8  RS03
R_hip_yaw     3  RS03     L_hip_yaw     9  RS03
R_knee        4  RS04     L_knee       10  RS04
R_foot_pitch  5  RS02     L_foot_pitch 11  RS02
R_foot_roll   6  RS02     L_foot_roll  12  RS02
```

## Node Summary

| Node | Package | Function |
|------|---------|----------|
| `imu_node` | biped_driver | BNO085 I2C → /imu/data, /imu/gravity |
| `can_bus_node` × 2 | biped_driver | RS02/03/04 CAN → /joint_states, /motor_states |
| `policy_node` | biped_control | ONNX inference → /joint_commands (50Hz) |
| `safety_node` | biped_control | Watchdog → /safety/status |
| `state_machine_node` | biped_control | FSM → /state_machine, /robot_state |
| `keyboard_teleop` | biped_teleop | Keyboard → /cmd_vel |
| `calibrate_node` | biped_tools | Interactive calibration → calibration.yaml |
| `foxglove_bridge` | foxglove_bridge | WebSocket bridge for desktop monitoring |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No CAN device | Run `setup_can.sh`, check dtoverlay in config.txt |
| IMU init fails | Power cycle BNO085, check I2C: `i2cdetect -y 1` |
| Motor no response | Check 48V power, verify CAN ID with RobStride tool |
| Wild policy outputs | Normal without motor feedback — closed loop fixes this |
| Robot falls immediately | Reduce `gain_scale`, check calibration offsets |
| ESTOP triggered | Check: `ros2 topic echo /safety/fault` |
| Foxglove won't connect | Check Pi IP, ensure port 8765 is not blocked |
