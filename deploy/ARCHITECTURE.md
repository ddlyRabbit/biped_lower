# Biped Deployment Architecture

ROS2 Jazzy · RPi 5 · Dual SocketCAN (`can0` + `can1`) · 12 RobStride motors · IMU (BNO085 or IM10A) · ONNX policy

---

## Hardware

| Component | Detail |
|-----------|--------|
| Compute | RPi 5 (8 GB), Ubuntu 24.04 aarch64 |
| CAN | 2-CH CAN HAT — 2× MCP2515/SPI, `can0` (right) + `can1` (left), 1 Mbps |
| IMU | BNO085 (I2C, 50Hz) or IM10A (USB serial, 300Hz) — axes aligned to base_link |
| Motors | 12× RobStride (4× RS04, 4× RS03, 4× RS02), can0=right, can1=left |
| Policy | `student_flat.onnx` — MLP 45→128→128→128→12, ~0.5 ms |

## Motors

| Joint | ID | Model | Nm | Joint | ID | Model | Nm |
|-------|----|-------|----|-------|----|-------|----|
| R_hip_pitch | 1 | RS04 | 12 | L_hip_pitch | 7 | RS04 | 12 |
| R_hip_roll | 2 | RS03 | 7.5 | L_hip_roll | 8 | RS03 | 7.5 |
| R_hip_yaw | 3 | RS03 | 7.5 | L_hip_yaw | 9 | RS03 | 7.5 |
| R_knee | 4 | RS04 | 12 | L_knee | 10 | RS04 | 12 |
| R_foot_pitch | 5 | RS02 | 4 | L_foot_pitch | 11 | RS02 | 4 |
| R_foot_roll | 6 | RS02 | 4 | L_foot_roll | 12 | RS02 | 4 |

Host CAN ID: `0xFD`. Per-motor `direction` (±1) stored in `calibration.yaml`.

## Motor Calibration

Each motor has an arbitrary encoder zero position that doesn't correspond to the URDF joint zero.
Calibration maps between **encoder space** (raw motor position) and **joint space** (URDF radians).

### Conversion

```
Joint → Motor (sending commands):
    motor_cmd = joint_pos * direction + offset

Motor → Joint (reading feedback):
    joint_pos = (encoder_pos - offset) * direction
```

Where:
- `offset`: encoder position when joint is at URDF zero (or URDF lower limit)
- `direction`: +1 (encoder increases = joint increases) or -1 (inverted)

### Calibration File (`calibration.yaml`)

```yaml
R_hip_pitch:
  motor_min: -0.6397    # encoder value at physical min stop
  motor_max: 2.6328     # encoder value at physical max stop
  offset: 1.5769        # encoder value at URDF joint zero
  direction: 1           # +1 normal, -1 inverted
```

### Calibration Procedure

1. `ros2 launch biped_bringup calibrate.launch.py` — starts calibration node
2. Manually move each joint to both mechanical limits
3. Node records `motor_min` and `motor_max` from encoder feedback
4. Offset computed automatically:
   - **Normal joints**: `offset = motor_min - urdf_lower_limit`
   - **Ankle motors** (parallel linkage): `offset = motor_min - cmd_lo` (dir=+1) or `offset = motor_min + cmd_hi` (dir=-1)
5. `direction` preserved from existing calibration (set via `setup_directions.py`)
6. Ctrl+C saves to `calibration.yaml`

### Ankle Linkage

Ankle motors (foot_top, foot_bottom) operate in **motor command space**, not joint space:

```
motor_upper =  pitch_sign × K_P × pitch − K_R × roll
motor_lower = −pitch_sign × K_P × pitch − K_R × roll
```

Where `K_P=1.276`, `K_R=0.974`, `pitch_sign=+1 (R) / -1 (L)`.

The CAN node handles this transform:
- **Outbound**: policy sends foot_pitch/roll → CAN node converts to foot_top/foot_bottom motor commands
- **Inbound**: motor encoders report in motor space → CAN node converts back to foot_pitch/roll for `/joint_states`

Motor command limits for ankles are computed by evaluating all four URDF corner combinations
of (pitch_min/max, roll_min/max) through the linkage transform.

## Joint Limits & Defaults

| Joint | Lower | Upper | Default | | Joint | Lower | Upper | Default |
|-------|-------|-------|---------|---|-------|-------|-------|---------|
| R_hip_pitch | −2.217 | 1.047 | 0.2 | | L_hip_pitch | −1.047 | 2.217 | −0.2 |
| R_hip_roll | −2.269 | 0.209 | 0.0 | | L_hip_roll | −0.209 | 2.269 | 0.0 |
| R_hip_yaw | −1.571 | 1.571 | 0.0 | | L_hip_yaw | −1.571 | 1.571 | 0.0 |
| R_knee | 0.000 | 2.705 | 0.4 | | L_knee | 0.000 | 2.705 | 0.4 |
| R_foot_pitch | −0.873 | 0.524 | −0.2 | | L_foot_pitch | −0.873 | 0.524 | −0.2 |
| R_foot_roll | −0.262 | 0.262 | 0.0 | | L_foot_roll | −0.262 | 0.262 | 0.0 |

Hip pitch/roll limits are mirrored L↔R. Knee, foot limits are symmetric.

## PD Gains

| Group | Kp | Kd | | Group | Kp | Kd |
|-------|----|----|---|-------|----|----|
| hip_pitch | 15 | 3.0 | | knee | 15 | 3.0 |
| hip_roll | 10 | 3.0 | | foot_pitch | 2.0 | 0.2 |
| hip_yaw | 10 | 3.0 | | foot_roll | 2.0 | 0.2 |

Runtime-tunable via `gain_scale` parameter (0.3→1.0 ladder).

---

## Packages

```
biped_ws/src/
├── biped_msgs/         CMake — MITCommand, MotorState, RobotState
├── biped_description/  CMake — URDF (light) + 35 STL meshes
├── biped_driver/       Python
│   ├── robstride_dynamics/  Vendored Seeed CAN library (python-can)
│   │   └── bus.py           RobstrideBus: MIT R/W, motor ID–filtered reads
│   ├── robstride_can.py     BipedMotorManager, ankle linkage, soft stops
│   ├── can_bus_node.py      12-motor CAN loop @ 50 Hz
│   ├── imu_node.py          BNO085 I2C @ 50 Hz
│   ├── im10a_imu_node.py   IM10A USB serial @ 300 Hz
│   └── im10a_driver.py     IM10A protocol parser (WIT-motion B6)
├── biped_control/      Python
│   ├── obs_builder.py       Sensor → 45d observation vector
│   ├── policy_node.py       ONNX inference @ 50 Hz
│   ├── state_machine_node.py  IDLE→STAND→WALK→ESTOP
│   └── safety_node.py       Pitch/roll/temp/timeout watchdog
├── biped_teleop/       keyboard_teleop → /cmd_vel
├── biped_tools/        calibrate_node, export_onnx
└── biped_bringup/      Launch files + robot.yaml
```

---

## Data Flow

### WALK Mode (motors active)

```
 /cmd_vel ──────────────────────────────┐
 /imu/data ──┐                          │
 /imu/gravity┤  ┌────────────────┐      │
             ├─▶│  policy_node   │◀─────┘
 /joint_states┤  │  obs→ONNX      │
              │  │  →action(12d)  │
              │  └───────┬────────┘
              │          │ /joint_commands (MITCommandArray)
              │          ▼
              │  ┌────────────────┐    ┌──────────────┐
              └─▶│ can_bus_node   │◀──▶│  12× motors  │
                 │ (C++ recommended)   └──────────────┘
                 └────────────────┘
                         │
 /motor_states ──────────┤
                         ▼
                 ┌────────────────┐    ┌──────────────────┐
                 │  safety_node   │───▶│ state_machine_node│
                 │  pitch/roll    │    │ IDLE→STAND→WALK   │
                 │  temp/faults   │    │ →ESTOP            │
                 └────────────────┘    └──────────────────┘

 /tf (odom→base_link) ◀── imu_node (raw quaternion)
 /tf (base_link→joints) ◀── robot_state_publisher ◀── /joint_states
 /robot_description ◀── bringup.launch.py (URDF with meshes)
```

### SIM_WALK Mode (viz only, motors hold STAND)

```
 /cmd_vel ──────────────────────────────┐
 /imu/data ──┐                          │
 /imu/gravity┤  ┌────────────────┐      │
             ├─▶│  policy_node   │◀─────┘
 /joint_states┤  │  obs→ONNX      │
              │  │  →action(12d)  │
              │  └──┬─────────┬───┘
              │     │         │
              │     │         ├─▶ /policy_viz (MITCommandArray)
              │     │         └─▶ /policy_viz_joints (JointState)
              │     │                    │
              │     │           ┌────────▼─────────────┐
              │     │           │ policy_state_publisher│
              │     │           │ frame_prefix="policy/"│
              │     │           └────────┬─────────────┘
              │     │                    │
              │     │           /tf policy/* frames → Foxglove ghost
              │     │
              │     ▼ (NOT used — state_machine holds STAND)
              │  ┌────────────────┐
              └─▶│ state_machine  │──▶ /joint_commands (STAND defaults)
                 │ _handle_stand  │        │
                 └────────────────┘        ▼ motors
```

### Policy Pipeline

```
Sensor data → obs_builder.build() → 48d observation vector
    ↓
ONNX inference (MLP 48→128→128→128→12) → raw actions
    ↓
np.clip(actions, -1.0, 1.0) → clamped actions
    ↓
action_to_positions: target = default_pos + action × scale → joint targets (rad)
    ↓
MITCommand per joint: { position, velocity=0, kp, kd, torque_ff=0 }
    ↓
WALK  → /joint_commands → motors
SIM_WALK → /policy_viz + /policy_viz_joints → Foxglove
```

### Tanh Output Layer (V74+)

Training supports `--tanh` flag which adds `nn.Tanh()` after the actor MLP, bounding raw actions to [-1, +1] natively. This eliminates actuator saturation on real hardware.

```bash
# Training with tanh
/isaac-sim/python.sh biped_train_rsl.py --tanh --num_envs 8192 --headless

# Play with tanh (must match training)
/isaac-sim/python.sh biped_play_rsl.py --tanh --checkpoint model.pt --video --headless
```

Without `--tanh`: actions are unbounded (MLP output), clipped to ±1 in deploy code.
With `--tanh`: actions bounded by architecture. Checkpoint keys have `actor.0.X` prefix (wrapped Sequential).

### Topics

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/joint_commands` | MITCommandArray | policy_node (WALK) / state_machine (STAND) | can_bus_node |
| `/joint_states` | JointState | can_bus_node | policy_node, state_machine, robot_state_publisher |
| `/motor_states` | MotorStateArray | can_bus_node | safety_node |
| `/policy_viz` | MITCommandArray | policy_node (SIM_WALK) | Foxglove |
| `/policy_viz_joints` | JointState | policy_node (SIM_WALK) | policy_state_publisher → /tf policy/* |
| `/cmd_vel` | Twist | keyboard_teleop | policy_node |
| `/state_command` | String | keyboard_teleop | state_machine_node |
| `/state_machine` | String | state_machine_node | policy_node, keyboard_teleop |
| `/imu/data` | Imu | imu_node | policy_node |
| `/imu/gravity` | Vector3Stamped | imu_node | policy_node, safety_node |
| `/safety/status` | Bool | safety_node | state_machine_node |
| `/robot_description` | String | bringup.launch.py | Foxglove URDF display |
| `/tf` | TFMessage | robot_state_publisher, policy_state_publisher | Foxglove 3D |

### Nodes

| Node | Package | Description |
|------|---------|-------------|
| `can_bus_node_cpp` | biped_driver_cpp | C++ CAN driver, ~300Hz per bus, dual CAN |
| `can_bus_node` | biped_driver | Python sync CAN driver, ~50Hz |
| `can_bus_node_async` | biped_driver | Python async CAN driver, ~200Hz |
| `imu_node` | biped_driver | BNO085 IMU via I2C, 50Hz (default) |
| `im10a_imu_node` | biped_driver | IM10A IMU via USB serial, 300Hz |
| `policy_node` | biped_control | ONNX inference at 50Hz, publishes motor cmds or viz |
| `safety_node` | biped_control | Pitch/roll/temp monitoring, triggers ESTOP |
| `state_machine_node` | biped_control | FSM: IDLE→STAND→WALK/SIM_WALK/WIGGLE→ESTOP |
| `keyboard_teleop` | biped_teleop | Terminal UI: state transitions + velocity control |
| `robot_state_publisher` | (system) | URDF → /tf from /joint_states |
| `policy_state_publisher` | (system) | URDF → /tf policy/* from /policy_viz_joints |

All control loops run at **50 Hz**. Safety at 50 Hz, state publishing at 10 Hz.

---

## Ankle Parallel Linkage

Ref: [asimov-v0/mechanical/ankle_mechanism.md](https://github.com/asimovinc/asimov-v0/blob/main/mechanical/ankle_mechanism.md)

Two RS02 motors per ankle with inverted mounting (A up → linkage up, B up → linkage down).

```
K_P = d/r = 41.14/32.249 = 1.2757    (pivot-to-bar / crank radius)
K_R = (c/2)/r = 31.398/32.249 = 0.9736  (bar half-length / crank radius)
```

### Forward (joint → motors)

```
motor_A (upper) =  ps × K_P × pitch − K_R × roll
motor_B (lower) = −ps × K_P × pitch − K_R × roll
```

### Inverse (motors → joint)

```
pitch =  ps × (A − B) / (2 × K_P)
roll  = −(A + B) / (2 × K_R)
```

**`ps` = pitch_sign**: `+1` for R ankle, `−1` for L ankle (mirrored mounting).

### What moves what

| Motion | Motor A | Motor B | Result |
|--------|---------|---------|--------|
| Pure pitch | +ps×K_P | −ps×K_P | Both linkages rise/fall together |
| Pure roll | −K_R | −K_R | One up, one down (inverted mounting) |

Roll transform is identical for L and R. Pitch is mirrored.

---

## Calibration & Motor Direction

### calibration.yaml

Produced by `calibrate_node` + `setup_directions.py`. Per joint:

```yaml
R_knee:
  motor_min: 1.4247      # raw encoder at physical limit A
  motor_max: 4.2013      # raw encoder at physical limit B
  offset: 1.4247         # computed homing offset
  direction: -1          # +1 normal, -1 inverted encoder
  urdf_lower: 0.0
  urdf_upper: 2.7053
```

### Direction detection

Some motors have encoders that increase in the opposite direction to the URDF
joint convention. `direction` in `calibration.yaml` handles this:

| direction | offset | joint_pos formula |
|-----------|--------|-------------------|
| +1 | `motor_min − URDF_lower` | `(encoder − offset) × 1` |
| −1 | `motor_max` | `(encoder − offset) × −1` |

Set interactively via `setup_directions.py` (move joint, check Foxglove, flip if wrong).

### Offset computation (from_robot_yaml)

```
Normal joints:    offset = motor_min − URDF_lower  (dir=+1)
                  offset = motor_max               (dir=−1)
Ankle joints:     offset = motor_min − cmd_at_encoder_min  (linkage transform)
```

### Soft stops

2° buffer inside calibration limits. Restoring torque `τ = 20 × penetration`.
All in **motor command-space** (not joint-space). For ankles, per-motor independently.

---

## Observation Vector (45d)

```
[0–2]   base_ang_vel      ← IMU gyro (rad/s, body frame)
[3–5]   projected_gravity  ← −gravity/‖g‖ (Isaac convention)
[6–8]   velocity_commands  ← /cmd_vel (lin_x, lin_y, ang_z)
[9–20]  joint_pos_rel      ← pos − default (hip×6, knee×2, foot_pitch×2, foot_roll×2)
[21–32] joint_vel          ← 12d in Isaac runtime order
[33–44] last_action        ← previous policy output
```

Isaac runtime joint order:
```
L_hip_pitch, R_hip_pitch, L_hip_roll, R_hip_roll,
L_hip_yaw, R_hip_yaw, L_knee, R_knee,
L_foot_pitch, R_foot_pitch, L_foot_roll, R_foot_roll
```

## Action Pipeline

```
action[12] (±1) → target = default + action × 0.5 → MIT(pos, kp×scale, kd×scale)
```

---

## State Machine

```
IDLE ──START──▶ STAND ──(stable)──▶ WALK
                  ▲                    │
                  │◀───STOP────────────┘
ANY ───────────▶ ESTOP ──RESET──▶ IDLE
```

**STAND**: 2s position interpolation → 1s gain ramp (10%→100%) → zero velocity.
**ESTOP**: kp=0, kd=0, torque=0 → motors free. Auto-triggers on safety fault.

---

## Safety

| Check | Threshold | Action |
|-------|-----------|--------|
| IMU pitch | 85° (configurable) | → ESTOP |
| IMU roll | 85° (configurable) | → ESTOP |
| Motor temp | 80°C | → ESTOP |
| Motor fault | any fault code | → ESTOP |
| IMU timeout | 200 ms | → ESTOP |

---

## CAN Timing (50 Hz = 20 ms budget)

```
12× MIT write+read (ID-filtered):  ~4.8 ms
IMU I2C read:                       ~3–5 ms
ONNX inference:                     ~0.5 ms
Total:                              ~9 ms → 11 ms headroom
```

`bus.py` filters responses by motor ID (discards wrong-motor frames).
MCP2515 TX buffer: 5-attempt retry with 0.5 ms backoff.

---

## Frame Conventions

| System | Convention |
|--------|-----------|
| URDF / Isaac | Z-up, +X forward, +Y left |
| IMU (mounted) | +X forward, +Y left, +Z up — aligned to base_link, no corrections |
| IMU quaternion | Sensor→Earth rotation, published raw as odom→base_link TF |
| projected_gravity | Both IMUs output (0,0,+9.81) upright → obs_builder negates → Isaac (0,0,−1) |
| IM10A gravity | Derived from quaternion: `R.apply([0,0,-1], inverse=True)` × −9.81 |
| Motor positions | Radians, output shaft, absolute encoder |

---

## New Robot: Zero to Walk

### 1. Hardware Setup
```bash
# Flash RPi 5 with Ubuntu 24.04 + ROS2 Jazzy
# Wire: 2-CH CAN HAT on SPI, BNO085 on I2C1 (addr 0x4B, RST→GPIO4)
# can0 = right leg (motors 1-6), can1 = left leg (motors 7-12)

# Add to /boot/firmware/config.txt:
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25
dtoverlay=mcp2515-can1,oscillator=12000000,interrupt=24
# Reboot
```

### 2. Clone & Build
```bash
git clone git@github.com:ddlyRabbit/biped_lower.git ~/biped_lower
cd ~/biped_lower/deploy/biped_ws
sudo apt install ros-jazzy-foxglove-bridge ros-jazzy-robot-state-publisher
pip install adafruit-circuitpython-bno08x adafruit-blinka onnxruntime
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
# Create setup_biped.bash (see Workspace Setup below)
```

### 3. CAN + Motor Scan
```bash
cd ~/biped_lower/deploy
./scripts/setup_can.sh              # run every boot
python3 scripts/scan_motors.py      # verify all 12 respond
```

### 4. Calibrate
```bash
source ~/biped_lower/deploy/biped_ws/setup_biped.bash
ros2 launch biped_bringup calibrate.launch.py
# Move each joint to both mechanical stops
# ✅ ticks are indicative (5% threshold) — press Enter when done
# Saves calibration.yaml
```

### 5. Set Motor Directions
```bash
# Terminal 1: launch hardware (or full bringup) + Foxglove
ros2 launch biped_bringup bringup.launch.py \
  calibration_file:=$(pwd)/calibration.yaml \
  onnx_model:=~/biped_lower/deploy/student_flat.onnx \
  gain_scale:=0.3

# Terminal 2: Foxglove
source ~/biped_lower/deploy/biped_ws/setup_biped.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# Connect Foxglove desktop to ws://<pi-ip>:8765

# Terminal 3: direction setup
python3 ~/biped_lower/deploy/scripts/setup_directions.py
# For each joint: move by hand, check Foxglove, flip if inverted
# For ankles: tested as pairs (upper/lower coupled)
# Saves direction to calibration.yaml — restart bringup to apply
```

### 6. Verify Hardware
```bash
ros2 launch biped_bringup hardware.launch.py \
  calibration_file:=$(pwd)/calibration.yaml
# Check in another terminal:
ros2 topic echo /joint_states --qos-reliability best_effort --once
ros2 topic echo /imu/data --qos-reliability best_effort --once
ros2 topic echo /safety/status --qos-reliability best_effort --once
```

### 7. Suspended Test (robot hanging)
```bash
ros2 launch biped_bringup bringup.launch.py \
  calibration_file:=$(pwd)/calibration.yaml \
  onnx_model:=~/biped_lower/deploy/student_flat.onnx \
  gain_scale:=0.3

# Start standing (2s soft start):
ros2 topic pub --once /state_command std_msgs/String "data: START"
# Robot moves to default pose. Increase gains if needed:
ros2 param set /policy_node gain_scale 0.5
```

### 8. Walk (suspended)
```bash
ros2 topic pub --once /state_command std_msgs/String "data: WALK"
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}}'
# Legs should show alternating swing motions
```

### 9. Ground Test
```bash
# Same as above but: gain_scale:=0.7, then 1.0
# Place robot on ground, START → WALK
# E-stop anytime:
ros2 topic pub --once /state_command std_msgs/String "data: ESTOP"
```

### Gain Ladder

| Scale | Use |
|-------|-----|
| 0.3 | First suspended test |
| 0.5 | Suspended walking |
| 0.7 | Ground standing |
| 1.0 | Full sim-matched gains |

---

## Workspace Setup (RPi)

```bash
# colcon bug: biped_msgs (CMake) not added to AMENT_PREFIX_PATH
# Use setup_biped.bash instead of install/setup.bash:
source ~/biped_lower/deploy/biped_ws/setup_biped.bash
```

## Scripts

| Script | Purpose |
|--------|---------|
| `setup_can.sh` | Bring up can0 (run each boot) |
| `scan_motors.py` | Read-only motor scan |
| `test_hip_yaw.py` | Jog test with soft stop verification |
| `setup_directions.py` | Interactive motor direction setup (Foxglove) |

## C++ CAN Driver (`biped_driver_cpp`)

Drop-in replacement for Python `biped_driver` CAN node.

```
biped_driver_cpp/
├── include/biped_driver_cpp/
│   ├── robstride_bus.hpp      ← SocketCAN + MIT protocol (RS02/03/04)
│   ├── motor_manager.hpp      ← Multi-bus management + calibration
│   └── ankle_linkage.hpp      ← Parallel linkage transform
├── src/
│   ├── robstride_bus.cpp      ← Direct AF_CAN, MIT encode/decode
│   ├── motor_manager.cpp      ← YAML loading, soft-stop, clamping
│   └── can_bus_node.cpp       ← ROS2 node, 2 worker threads
├── CMakeLists.txt
└── package.xml
```

Based on [Seeed RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control) C++ library.
Extended with: multi-bus, per-model scaling, calibration, ankle linkage, ROS2 integration.

## State Machine

### State Diagram

```
              ┌──────────────────────────────────────┐
              │                                      │
 IDLE ─SPACE─▶ STAND ──g──▶ WALK ──b──▶ STAND       │
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

| State | Motors | Policy | Description |
|-------|--------|--------|-------------|
| **IDLE** | Off | Off | Waiting for START |
| **STAND** | PD hold defaults | Off | Soft start → hold pose |
| **WALK** | Policy output | Active → `/joint_commands` | Real locomotion |
| **SIM_WALK** | PD hold defaults | Active → `/policy_viz` | Viz-only, motors safe |
| **WIGGLE_SEQ** | Sine sweep (1 joint) | Off | Sequential joint test |
| **WIGGLE_ALL** | Sine sweep (all) | Off | Simultaneous joint test |
| **ESTOP** | Zero torque | Off | Emergency, requires reset |

### SIM_WALK

Motors hold STAND position while the ONNX policy runs inference at 50Hz
using real sensor data. Policy output publishes to `/policy_viz` (not `/joint_commands`).

Use for:
- Pre-flight check before real walking
- Foxglove visualization of policy targets vs actual positions
- Debugging policy behavior with real IMU data

### WIGGLE_SEQ
Cycles through each joint one at a time. Active joint follows sine wave
between `default + neg` and `default + pos`. All other joints hold default.
Automatically returns to STAND after all 12 joints complete.

### WIGGLE_ALL
All joints wiggle simultaneously at their configured frequency/range.
Runs until STOP command.

Config: `biped_bringup/config/wiggle.yaml` (read from source tree, no rebuild needed).

### Keyboard Teleop

| Key | Action |
|-----|--------|
| SPACE | IDLE → STAND |
| g | STAND → WALK |
| v | STAND → SIM_WALK |
| t | STAND → WIGGLE_SEQ |
| y | STAND → WIGGLE_ALL |
| b | → STAND (stop) |
| ESC | → ESTOP |
| w/s/a/d/q/e | velocity control |
| x | zero velocity |
| 1-5 | speed presets |
