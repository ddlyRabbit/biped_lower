# Biped Robot — Bringup Guide

## Prerequisites

- RPi 5 with Ubuntu 24.04 + ROS2 Jazzy
- Waveshare RS485 CAN HAT (B) — MCP2515 on SPI0
- IMU: BNO085 (I2C) or IM10A (USB serial /dev/ttyUSB0)
- All 12 motors on dual CAN bus (can0=right, can1=left), IDs 1–12
- Student policy exported as ONNX (`student_flat.onnx`)

## CAN HAT Setup (one-time)

Add to `/boot/firmware/config.txt`:
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
dtoverlay=mcp2515-can1,oscillator=12000000,interrupt=24
```
Reboot. Verify: `ls /sys/class/net/ | grep can` → should show `can0` and `can1`.

## Step 1: Bring Up CAN (every boot)

```bash
cd ~/biped_lower/deploy
./scripts/setup_can.sh
```

Verify: `candump can0` / `candump can1` — shows frames if motors powered.

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
ros2 launch biped_bringup calibrate.launch.py joint:=L_foot_top
ros2 launch biped_bringup calibrate.launch.py joint:=L_foot_bottom
```

Move each joint to both mechanical limits. Joints auto-mark ✅ when range matches
URDF limits ±20%. Press Ctrl+C to save `calibration.yaml`.

## Step 4: Test Hardware (no policy)

```bash
ros2 launch biped_bringup hardware.launch.py \
  can_driver:=can_bus_node_cpp \
  imu_type:=bno085 \
  calibration_file:=calibration.yaml

# For IM10A IMU (USB):
#   imu_type:=im10a

# In another terminal:
ros2 topic echo /joint_states     # motor positions + velocities
ros2 topic echo /imu/data         # IMU quaternion + gyro
ros2 topic echo /safety/status    # should be True
```

## Step 5: Suspended Test (robot hanging, feet off ground)

⚠️ **Robot must be suspended!**

```bash
# Terminal 1: full robot with C++ CAN driver
# imu_type: bno085 (default, I2C) | im10a (USB serial)
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  imu_type:=im10a \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/v68b_student_flat.onnx \
  gain_scale:=0.3

# Terminal 2: keyboard teleop (state transitions + velocity)
ros2 run biped_teleop keyboard_teleop

# Terminal 3: monitoring (optional)
ros2 launch biped_bringup record.launch.py
```

Press SPACE to stand, then `v` for SIM_WALK (safe viz test) before `g` for real WALK.

## State Machine

```
              ┌──────────────────────────────────────┐
              │                                      │
 IDLE ──SPACE──▶ STAND ──g──▶ WALK ──b──▶ STAND     │
                  │                                  │
                  ├──v──▶ SIM_WALK ──b──▶ STAND      │
                  │                                  │
                  ├──t──▶ WIGGLE_SEQ ──(auto)──▶ STAND
                  │                                  │
                  └──y──▶ WIGGLE_ALL ──b──▶ STAND    │
                                                     │
              ESC ── any state ──▶ ESTOP ◀───────────┘
```

### States

| State | Description | Motors |
|-------|-------------|--------|
| **IDLE** | Motors disabled, waiting | Off |
| **STAND** | Soft start → hold default pose | PD hold at defaults |
| **WALK** | Policy active, responds to cmd_vel | Policy output → motors |
| **SIM_WALK** | Policy runs for visualization only | PD hold at defaults (STAND) |
| **WIGGLE_SEQ** | Sequential sine sweep, one joint at a time | Sine wave per joint |
| **WIGGLE_ALL** | All joints wiggle simultaneously | Sine wave all joints |
| **ESTOP** | Emergency stop, zero torque | Zero torque |

### SIM_WALK (Visualization Mode)

SIM_WALK keeps the motors safely in STAND position while running the ONNX policy
in the background. Policy output publishes to `/policy_viz` instead of `/joint_commands`.

Use this to:
- Verify policy behavior before enabling real walking
- Compare policy targets vs actual joint positions in Foxglove
- Test with real IMU/sensor data without risk

**Topics:**
- `/policy_viz` (MITCommandArray) — what the policy *would* command
- `/joint_commands` (MITCommandArray) — actual STAND commands to motors
- `/joint_states` (JointState) — real motor feedback

### Keyboard Teleop

```bash
ros2 run biped_teleop keyboard_teleop
```

#### Keys

| Key | Action |
|-----|--------|
| **SPACE** | IDLE → STAND (enable motors) |
| **g** | STAND → WALK (policy active) |
| **v** | STAND → SIM_WALK (viz only) |
| **t** | STAND → WIGGLE_SEQ |
| **y** | STAND → WIGGLE_ALL |
| **b** | any → STAND (stop) |
| **ESC** | any → ESTOP (emergency) |
| w/s | forward / backward velocity |
| a/d | left / right velocity |
| q/e | yaw left / yaw right |
| x | zero all velocities |
| 1-5 | speed presets (0.1–1.0 m/s) |
| +/- | adjust step size |
| Ctrl+C | quit teleop |

### Manual State Commands (without teleop)

```bash
ros2 topic pub --once /state_command std_msgs/String "data: START"
ros2 topic pub --once /state_command std_msgs/String "data: WALK"
ros2 topic pub --once /state_command std_msgs/String "data: SIM_WALK"
ros2 topic pub --once /state_command std_msgs/String "data: WIGGLE_SEQ"
ros2 topic pub --once /state_command std_msgs/String "data: WIGGLE_ALL"
ros2 topic pub --once /state_command std_msgs/String "data: STOP"
ros2 topic pub --once /state_command std_msgs/String "data: ESTOP"
```

## Step 6: Ground Test

After suspended test looks good:
```bash
# Terminal 1: full robot with higher gains
# imu_type: bno085 (default) | im10a
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  imu_type:=im10a \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/v68b_student_flat.onnx \
  gain_scale:=0.5

# Terminal 2: teleop
ros2 run biped_teleop keyboard_teleop
# SPACE → stand, v → SIM_WALK (verify policy), g → WALK (real walking)
# w/1-5 for velocity, ESC for emergency stop
```

## Emergency Stop

ESC key in teleop, or:
```bash
ros2 topic pub --once /state_command std_msgs/String "data: ESTOP"
```

Also triggers automatically on pitch/roll violation or motor fault.
Reset: power cycle or `ros2 topic pub --once /state_command std_msgs/String "data: RESET"`

## Gain Tuning

Use `gain_scale` launch argument to scale all PD gains uniformly:

| Scale | Use case |
|-------|----------|
| 0.3 | First suspended test |
| 0.5 | Suspended walking test |
| 0.7 | Ground standing test |
| 1.0 | Full sim gains |

Default PD gains (V72):
| Joint | Kp | Kd |
|-------|----|----|
| hip_pitch | 15 | 3.0 |
| hip_roll | 10 | 3.0 |
| hip_yaw | 10 | 3.0 |
| knee | 15 | 3.0 |
| foot_pitch | 8 | 0.2 |
| foot_roll | 8 | 0.2 |

Safety ESTOP triggers on base tilt: pitch > 45° or roll > 30° (adjustable via `max_pitch_deg` / `max_roll_deg` launch args). These are torso orientation limits, not joint limits.

## Launch Files

| Launch | Purpose |
|--------|---------|
| `hardware.launch.py` | IMU + CAN + safety (no policy) |
| `calibrate.launch.py` | Interactive joint calibration |
| `bringup.launch.py` | Full robot (all nodes) |
| `record.launch.py` | Foxglove bridge + rosbag recording |

## CAN Driver Options

| Driver | Language | Loop Rate | Launch Param |
|--------|----------|-----------|--------------|
| `can_bus_node_cpp` | C++ | ~300Hz | `can_driver:=can_bus_node_cpp` **(recommended)** |
| `can_bus_node_async` | Python (async) | ~200Hz | `can_driver:=can_bus_node_async` |
| `can_bus_node` | Python (sync) | ~50Hz | `can_driver:=can_bus_node` |

All drivers share the same ROS2 interface:
- Sub: `/joint_commands` (biped_msgs/MITCommandArray)
- Pub: `/joint_states` (sensor_msgs/JointState)
- Pub: `/motor_states` (biped_msgs/MotorStateArray)

## Wiggle Test

Test individual or all joints with sine wave sweeps. Press `t` (sequential) or `y` (all) in teleop from STAND state.

### Config

Edit `deploy/biped_ws/src/biped_bringup/config/wiggle.yaml` (no rebuild needed):
```yaml
wiggle:
  duration_per_joint: 3.0

  joints:
    R_hip_pitch:   { pos: 0.087, neg: -0.087, freq: 1.0 }
    R_knee:        { pos: 0.15,  neg: -0.05,  freq: 0.5 }
```

Config is **re-read on each WIGGLE command** — edit and re-trigger without restart.

### Safety
- Joint targets clamped to URDF limits
- Config validated at load (warns if range exceeds limits)
- ESTOP always available (ESC key)
- gain_scale applies to wiggle PD gains

## Build (on Pi)

```bash
cd ~/biped_lower/deploy/biped_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select biped_msgs biped_driver biped_driver_cpp biped_control biped_teleop biped_bringup
source setup_biped.bash
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No CAN device | Run `setup_can.sh`, check dtoverlay in `/boot/firmware/config.txt` |
| Motor no response | Check 48V power, run `scan_motors.py`, verify CAN ID |
| BNO085 IMU init fails | Power cycle BNO085, check I2C: `i2cdetect -y 1` (expect 0x4B) |
| IM10A IMU no data | Check `ls /dev/ttyUSB0`, add user to `dialout` group, verify 460800 baud |
| Robot falls immediately | Reduce `gain_scale`, verify calibration offsets |
| ESTOP triggered | Check: `ros2 topic echo /safety/fault` |
| Foxglove won't connect | Check Pi IP, ensure port 8765 not blocked |
| TX buffer full errors | Normal at startup (MCP2515 has 3 slots), retries handle it |
| Ankle motors track poorly | Re-calibrate ankles, check linkage rod lengths match CAD |
