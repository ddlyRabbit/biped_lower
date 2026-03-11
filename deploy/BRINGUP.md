# Biped Robot — Bringup Guide

## Prerequisites

- RPi 5 with Ubuntu 24.04 + ROS2 Jazzy
- 2-CH CAN HAT (MCP2515) configured in `/boot/firmware/config.txt`
- BNO085 IMU on I2C (addr 0x4B, RST on GPIO 4)
- Motor CAN IDs set: Right 1-6 (can0), Left 7-12 (can1)
- Student policy exported to ONNX

## CAN HAT Setup (one-time)

Add to `/boot/firmware/config.txt`:
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
dtoverlay=mcp2515-can1,oscillator=12000000,interrupt=24
```
Reboot. Check: `ls /sys/class/net/ | grep can`

## Step 1: Setup CAN (every boot)

```bash
cd ~/biped_lower/deploy
./scripts/setup_can.sh
```

Verify: `candump can0` (should show nothing if no motors powered)

## Step 2: Source workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/biped_lower/deploy/biped_ws/install/setup.bash
```

(Already in `~/.bashrc` if configured)

## Step 3: Calibrate (once, or after motor reassembly)

```bash
ros2 run biped_tools calibrate_node --ros-args \
  -p can_interface:=can0 \
  -p motor_config:="R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02" \
  -p output_file:=calibration_right.yaml

ros2 run biped_tools calibrate_node --ros-args \
  -p can_interface:=can1 \
  -p motor_config:="L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02" \
  -p output_file:=calibration_left.yaml
```

Follow interactive prompts — move each joint to both limits.

## Step 4: Test hardware (no policy)

```bash
ros2 launch biped_bringup hardware.launch.py

# In another terminal:
ros2 topic echo /joint_states     # verify motor feedback
ros2 topic echo /imu/data         # verify IMU
ros2 topic echo /safety/status    # should be True
```

## Step 5: Test policy (robot suspended)

**⚠️ Robot must be hanging / feet off ground for first test!**

```bash
ros2 launch biped_bringup bringup.launch.py \
  onnx_model:=student_flat.onnx \
  gain_scale:=0.3

# In another terminal:
ros2 topic pub /state_command std_msgs/String "data: START"
# → Robot should slowly move to standing pose (2s ramp)

# Watch joint tracking:
ros2 topic echo /joint_states
```

## Step 6: Walking

```bash
# After standing is stable:
ros2 topic pub /state_command std_msgs/String "data: WALK"

# Open keyboard teleop in another terminal:
ros2 run biped_teleop keyboard_teleop
# Press 1 for 0.1 m/s, then w to go forward
```

## Step 7: Emergency stop

```bash
ros2 topic pub /state_command std_msgs/String "data: ESTOP"
# Or: safety_node auto-triggers on pitch/roll violation
# Reset: ros2 topic pub /state_command std_msgs/String "data: RESET"
```

## Gain Tuning

Start with `gain_scale:=0.3` (30% of sim gains). Increase gradually:
- 0.3 → test standing stability
- 0.5 → test slow walking
- 0.7 → test normal walking
- 1.0 → full sim gains (may oscillate — tune per-joint if needed)

Per-joint gains file:
```yaml
# gains.yaml
L_hip_pitch: {kp: 15.0, kd: 3.0}
R_hip_pitch: {kp: 15.0, kd: 3.0}
# ... etc
```
Pass with: `-p gains_file:=gains.yaml`

## Motor Config Reference

```
CAN0 (Right)              CAN1 (Left)
R_hip_pitch   1  RS04     L_hip_pitch   7  RS04
R_hip_roll    2  RS03     L_hip_roll    8  RS03
R_hip_yaw     3  RS03     L_hip_yaw     9  RS03
R_knee        4  RS04     L_knee       10  RS04
R_foot_pitch  5  RS02     L_foot_pitch 11  RS02
R_foot_roll   6  RS02     L_foot_roll  12  RS02
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `No CAN device` | Run `setup_can.sh`, check CAN HAT dtoverlay |
| IMU init fails | Power cycle BNO085, check I2C: `i2cdetect -y 1` |
| Motor no response | Check 48V power, verify CAN ID with RobStride tool |
| Policy outputs NaN | Check ONNX model path, verify obs dimensions |
| Robot falls immediately | Reduce `gain_scale`, check calibration offsets |
| ESTOP triggered | Check `/safety/fault` for reason |
