# Biped Deployment Architecture — ROS2 (V58 Light, +X Forward)

## Student Policy Observation Vector (45d) — Sim to Real Mapping

The obs vector must be assembled in EXACTLY this order (matches training):

```
Index  Dim  Name               Sim Source                    Real Source                         Notes
─────  ───  ─────────────────  ────────────────────────────  ──────────────────────────────────  ─────────────────────────────────
 0-2    3   base_ang_vel       base_ang_vel (body frame)     BNO085 bno.gyro (calibrated)       rad/s, body frame
 3-5    3   projected_gravity  quat_rotate_inverse(q, [0,0,-1])  BNO085 bno.gravity / |g|       MUST NORMALIZE to unit vector
 6-8    3   velocity_commands  generated_commands            /cmd_vel (lin_x, lin_y, ang_z)      from gamepad/keyboard
 9-14   6   hip_pos            joint_pos_rel (hip_roll,      CAN encoder - default_pos           order: L_roll, R_roll, L_yaw,
                               hip_yaw, hip_pitch)                                               R_yaw, L_pitch, R_pitch
15-16   2   knee_pos           joint_pos_rel (knee)          CAN encoder - default_pos           order: L_knee, R_knee
17-18   2   foot_pitch_pos     joint_pos_rel (foot_pitch)    CAN encoder - default_pos           order: L, R
19-20   2   foot_roll_pos      joint_pos_rel (foot_roll)     CAN encoder - default_pos           order: L, R
21-32  12   joint_vel          joint_vel_rel (all joints)    CAN velocity feedback               Isaac runtime joint order
33-44  12   last_action        last policy output            ring buffer of previous actions      dimensionless (before scaling)
─────  ───
Total: 45d
```

### Critical Details

**projected_gravity**: In Isaac, `projected_gravity = quat_rotate_inverse(body_quat, [0, 0, -1])`.
This is a UNIT vector. BNO085's `SH2_GRAVITY` report gives acceleration in m/s² (~9.81).
MUST normalize: `proj_grav = gravity_report / norm(gravity_report)`.

**Joint position order**: Isaac runtime order ≠ URDF order. The obs builder must use the
same joint ordering as Isaac's `robot.joint_names`. Verify at startup by printing joint names.
From our training, the runtime order is:
```
 0: left_hip_pitch_04      6: left_knee_04
 1: right_hip_pitch_04     7: right_knee_04
 2: left_hip_roll_03       8: left_foot_pitch_02
 3: right_hip_roll_03      9: right_foot_pitch_02
 4: left_hip_yaw_03       10: left_foot_roll_02
 5: right_hip_yaw_03      11: right_foot_roll_02
```
But hip_pos obs uses regex `[".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*"]` which groups
by joint TYPE, not left/right. Isaac resolves this to:
`[L_roll, R_roll, L_yaw, R_yaw, L_pitch, R_pitch]` (6d).

**joint_vel_rel**: Returns velocity relative to default (which is 0 for all joints),
so effectively just raw joint velocity in rad/s. Maps directly to CAN velocity feedback.

**Observation noise**: Training adds uniform noise. On real hardware, DO NOT add artificial
noise — real sensor noise replaces it. Set `enable_corruption = False` equivalent.

**velocity_commands**: With `heading_command=True`, the command manager internally converts
heading error to ang_vel_z. On real robot, /cmd_vel directly provides (lin_x, lin_y, ang_z).
No heading mode needed — just pass through.

### Action → Motor Command

```
Policy output: action[12] (dimensionless, range ~ [-1, 1])
Position target: target[i] = default_pos[i] + action[i] × 0.5  (action_scale=0.5)
MIT command: {position=target, velocity=0, Kp=gains[i], Kd=gains[i], torque_ff=0}

Motor executes: τ = Kp × (target - actual) + Kd × (0 - vel)
This matches Isaac's ImplicitActuator PD control.
```

**Default positions** (from URDF init_state, +X forward):
```python
defaults = {
    "left_hip_pitch":  -0.2,  "right_hip_pitch":  0.2,
    "left_hip_roll":    0.0,  "right_hip_roll":   0.0,
    "left_hip_yaw":     0.0,  "right_hip_yaw":    0.0,
    "left_knee":        0.4,  "right_knee":       0.4,
    "left_foot_pitch": -0.2,  "right_foot_pitch": -0.2,
    "left_foot_roll":   0.0,  "right_foot_roll":   0.0,
}
```

**PD Gains** (start at sim values, tune on real hardware):
```
hip_roll/yaw:  Kp=10,  Kd=3.0   (halved from Berkeley)
hip_pitch:     Kp=15,  Kd=3.0
knee:          Kp=15,  Kd=3.0
foot_pitch:    Kp=2.0, Kd=0.2
foot_roll:     Kp=2.0, Kd=0.2
```
RS04 MIT mode Kp range: 0-5000, Kd range: 0-100. Our sim values are at the
very low end — start here and increase if tracking is poor. RS04 gear ratio is
9:1 but MIT mode operates at output shaft level (joint-level, post-gearbox),
so sim Kp maps roughly to real Kp.

### Soft Start Sequence (STAND state)

On transition IDLE → STAND:
1. Read current joint positions from encoders
2. Set MIT targets = current positions (hold in place), Kp=2, Kd=0.5 (very soft)
3. Over 2 seconds, linearly interpolate targets from current → default positions
4. Over 1 second, ramp Kp/Kd from soft → full policy gains
5. Begin policy inference with zero velocity command
6. Transition to WALK allowed after 2s stable standing

This prevents violent motion when enabling the policy.

### RS04 Motor-Side Safety (hardware level)

Enable on every motor at startup:
- `CAN_TIMEOUT` (index 0x702B): Set to 100ms. Motor enters reset if no CAN
  command received within timeout. Hardware-level e-stop independent of RPi.
- `limit_torque` (index 0x700B): Set per-joint max torque matching sim effort limits.
- `limit_cur` (index 0x7018): Set current limit per actuator type.

## Package Structure

```
biped_ws/src/
├── biped_bringup/              ← Launch files + config
│   ├── launch/
│   │   ├── bringup.launch.py          ← Full robot (all nodes)
│   │   ├── hardware.launch.py         ← IMU + CAN only (no policy)
│   │   ├── calibrate.launch.py        ← Manual homing
│   │   ├── record.launch.py           ← Foxglove bridge + rosbag recording
│   ├── config/
│   │   ├── robot.yaml                 ← Joint names, CAN IDs, limits
│   │   ├── policy.yaml                ← ONNX path, obs scaling, action scale
│   │   ├── safety.yaml                ← E-stop thresholds, torque limits
│   │   ├── calibration.yaml           ← Encoder offsets (generated by homing)
│   │   └── gains.yaml                 ← Per-joint Kp/Kd for MIT mode
│   └── package.xml
│
├── biped_driver/               ← Hardware interface nodes
│   ├── biped_driver/
│   │   ├── __init__.py
│   │   ├── robstride_dynamics/        ← Vendored Seeed shared library (python-can)
│   │   │   ├── __init__.py
│   │   │   ├── protocol.py            ← CAN comm types + parameter IDs
│   │   │   ├── table.py               ← Per-model MIT scaling tables
│   │   │   └── bus.py                 ← RobstrideBus: CAN connect, MIT R/W, param R/W
│   │   ├── robstride_can.py           ← Biped adapter: BipedMotorManager, ankle linkage
│   │   ├── can_bus_node.py            ← Single node for all 12 motors (can0 + can1)
│   │   └── imu_node.py               ← BNO085 I2C (SH2 protocol, gyro+quat fused)
│   ├── package.xml
│   └── setup.py
│
├── biped_control/              ← Policy + state machine
│   ├── biped_control/
│   │   ├── __init__.py
│   │   ├── policy_node.py             ← 50Hz ONNX inference loop
│   │   ├── obs_builder.py             ← Sensor data → 45d observation
│   │   ├── state_machine_node.py      ← IDLE→CALIBRATE→STAND→WALK→ESTOP
│   │   ├── safety_node.py             ← Watchdog, e-stop, limits
│   │   └── action_converter.py        ← Policy action → MIT command
│   ├── package.xml
│   └── setup.py
│
├── biped_teleop/               ← Velocity command interfaces
│   ├── biped_teleop/
│   │   ├── __init__.py
│   │   ├── keyboard_teleop.py         ← Keyboard velocity commander
│   │   └── gamepad_teleop.py          ← BT gamepad → /cmd_vel
│   ├── package.xml
│   └── setup.py
│
├── biped_tools/                ← Calibration + utilities
│   ├── biped_tools/
│   │   ├── __init__.py
│   │   ├── calibrate_node.py          ← Manual homing tool
│   │   ├── export_onnx.py             ← PyTorch → ONNX
│   │   └── joint_test_node.py         ← Single joint wiggle test
│   ├── package.xml
│   └── setup.py
│
├── biped_msgs/                 ← Custom message definitions
│   ├── msg/
│   │   ├── MITCommand.msg              ← Single joint MIT command
│   │   ├── MITCommandArray.msg         ← 12-joint MIT command array
│   │   ├── MotorState.msg              ← Single motor feedback
│   │   ├── MotorStateArray.msg         ← 12-motor feedback array
│   │   └── RobotState.msg              ← Full robot state (FSM + faults)
│   ├── package.xml
│   └── CMakeLists.txt
│
└── models/
    └── student_flat.onnx               ← Exported policy
```

## Custom Messages

### MITCommand.msg
```
string joint_name
float32 position        # target position (rad)
float32 velocity        # target velocity (rad/s), typically 0
float32 kp              # position gain
float32 kd              # velocity gain
float32 torque_ff       # feedforward torque (Nm), typically 0
```

### MITCommandArray.msg
```
std_msgs/Header header
biped_msgs/MITCommand[] commands    # 12 joints
```

### MotorState.msg
```
string joint_name
uint8 can_id
float32 position        # absolute encoder position (rad, calibrated)
float32 velocity        # current velocity (rad/s)
float32 torque          # current torque (Nm)
float32 temperature     # motor temp (°C)
uint8 fault_code        # RS04 fault flags
uint8 mode_status       # 0=Reset, 1=Calibration, 2=Run
```

### MotorStateArray.msg
```
std_msgs/Header header
biped_msgs/MotorState[] motors      # 12 motors
```

### RobotState.msg
```
std_msgs/Header header
string fsm_state                    # IDLE, CALIBRATE, STAND, WALK, ESTOP
bool safety_ok
bool all_motors_enabled
float32 battery_voltage
string[] active_faults
```

## ROS2 Nodes

### 1. can_bus_node (biped_driver)
**Single node managing all 12 motors across 2 CAN buses (can0=right, can1=left)**

Uses vendored `robstride_dynamics` library (from Seeed-Projects/RobStride_Control)
via `BipedMotorManager` which routes commands to the correct bus per joint.

```
Node: /can_bus_node
Params:
  robot_config: "config/robot.yaml"       # Motor map (name→id→type→bus)
  calibration_file: "config/calibration.yaml"
  loop_rate: 50.0                         # Hz
  # Fallback (if no robot_config):
  motor_config_can0: "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,..."
  motor_config_can1: "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,..."

Subscribes:
  /joint_commands    MITCommandArray      ← from policy_node

Publishes:
  /joint_states      JointState     @ 50Hz  (position, velocity, effort)
  /motor_states      MotorStateArray @ 50Hz (raw: temp, faults, mode)

Architecture:
  robstride_dynamics (vendored, no ROS)
    └→ robstride_can.py: BipedMotorManager (multi-bus routing, ankle linkage)
        └→ can_bus_node.py (ROS2 pub/sub/timer)

Ankle parallel linkage:
  Policy sees foot_pitch/foot_roll. Node transforms to motor_upper/motor_lower
  using linearized linkage gains from CAD (pitch_gain=1.276, roll_gain=0.974).
  Both ankle motors live on the same bus (same leg).

Timer callback (50Hz):
  For each CAN bus (grouped, not interleaved):
    1. For each joint on this bus:
       a. Send MIT command (write_operation_frame)
       b. Read motor feedback (read_operation_frame)
       c. Apply ankle inverse linkage if applicable
    2. Ankle joints: clamp in joint-space → linkage transform → motor commands
  3. Publish /joint_states (calibrated) and /motor_states (raw + temp + faults)

Lifecycle:
  startup: connect both buses, flush RX
  enable_all(): enable motors → set MIT mode → flush
  shutdown: disable all → disconnect
```

### 2. imu_node (biped_driver)
```
Node: /imu
Params:
  i2c_bus: 1                              # RPi 5 I2C bus 1 (GPIO 2/3)
  i2c_address: 0x4A                       # BNO085 default I2C address

Publishes:
  /imu/data          Imu            @ 100Hz
  /imu/gravity       Vector3Stamped @ 100Hz   (raw gravity for projected_gravity)

BNO085 over I2C (SH2 protocol):
  Uses Hillcrest SH2 protocol over I2C at 400kHz. Same sensor reports as
  UART-SHTP but at a safe, native RPi 5 bus speed. No 3Mbaud risk.

  Enabled reports (via adafruit-circuitpython-bno08x):
  - BNO_REPORT_ROTATION_VECTOR @ 100Hz — fused quaternion (i,j,k,real)
    Accessor: bno.quaternion → (i, j, k, real)
    Option: BNO_REPORT_GAME_ROTATION_VECTOR for no mag correction (less drift correction)
  - BNO_REPORT_GYROSCOPE @ 100Hz — calibrated angular velocity (rad/s, body frame)
    Accessor: bno.gyro → (x, y, z)
  - BNO_REPORT_GRAVITY @ 100Hz — gravity vector (m/s², body frame)
    Accessor: bno.gravity → (x, y, z)

  Why I2C over UART-RVC:
  - RVC (UART 115200) only gives yaw/pitch/roll + accel. NO angular velocity.
  - I2C uses full SH2 protocol with access to all individual sensor reports.

  Why I2C over UART-SHTP:
  - UART-SHTP is fixed at 3Mbaud — not configurable, unreliable on RPi 5.
  - I2C at 400kHz is natively supported, well-tested on RPi.
  - Bandwidth: ~3.2KB/s needed vs 400KB/s I2C capacity. Plenty of headroom.

  Parse → populate sensor_msgs/Imu:
  - orientation: quaternion directly from GYRO_INTEGRATED_RV
  - angular_velocity: x/y/z rad/s directly from GYRO_INTEGRATED_RV
  - linear_acceleration: not used (gravity report used for projected_gravity)

  Implementation: Python port of Adafruit_BNO08x (begin_I2C + SH2 HAL)
  using smbus2 or adafruit-circuitpython-bno08x package.
```

### 3. policy_node (biped_control)
```
Node: /policy
Params:
  onnx_model: "models/student_flat.onnx"
  loop_rate: 50.0
  action_scale: 0.5
  default_positions: [0.2, -0.2, 0.0, ...]   # 12 values from URDF
  obs_noise_scales: {...}                      # match training noise

Subscribes:
  /imu/data          Imu
  /joint_states      JointState
  /cmd_vel           Twist
  /state_machine     String          ← only runs when state=WALK

Publishes:
  /joint_commands    MITCommandArray  @ 50Hz
  /policy/obs        Float32MultiArray (debug, 45d)
  /policy/actions    Float32MultiArray (debug, 12d)

Timer callback (50Hz):
  1. Check FSM state — only compute if WALK or STAND
  2. Build observation (45d):
     - projected_gravity (3d) ← from SH2_GRAVITY report (or quat→gravity)
     - base_ang_vel (3d) ← directly from GYRO_INTEGRATED_RV angVel fields
     - velocity_commands (3d) ← from /cmd_vel
     - joint_pos_rel per group (12d) ← (current - default), per-group noise
     - joint_vel (12d) ← from /joint_states
     - last_action (12d) ← ring buffer
  3. Run ONNX inference → 12 actions
  4. Convert: position_target[i] = default[i] + action[i] * 0.5
  5. Build MITCommandArray with per-joint Kp/Kd from gains.yaml
  6. Safety clamp (position limits, max torque)
  7. Publish /joint_commands
```

### 4. state_machine_node (biped_control)
```
Node: /state_machine
Params:
  startup_state: "IDLE"

Subscribes:
  /motor_states      MotorStateArray
  /imu/data          Imu
  /safety/status     Bool
  /joy               Joy              ← button triggers

Publishes:
  /state_machine     String           @ 10Hz
  /robot_state       RobotState       @ 10Hz

States:
  IDLE       → all motors disabled, waiting for command
  CALIBRATE  → homing node active (manual calibration)
  STAND      → soft start sequence:
               1. Hold current position (soft gains)
               2. Interpolate to default pose over 2s
               3. Ramp to full gains over 1s
               4. Run policy with zero velocity command
               transition: stable for 2s → ready for WALK
  WALK       → policy runs with /cmd_vel input
  ESTOP      → immediate zero torque on all joints, motor CAN_TIMEOUT
               also triggers hardware e-stop
               requires manual reset (button press) to → IDLE

Transitions:
  IDLE → STAND:     gamepad START button
  STAND → WALK:     gamepad A button (after 2s stable stand)
  WALK → STAND:     gamepad B button or /cmd_vel = 0 for 3s
  ANY → ESTOP:      gamepad SELECT, or safety_node trigger
  ESTOP → IDLE:     gamepad START + SELECT together (deliberate)
```

### 5. safety_node (biped_control)
```
Node: /safety
Params:
  max_pitch_deg: 45.0
  max_roll_deg: 30.0
  max_motor_temp: 80.0
  can_timeout_ms: 100                 # no CAN response → e-stop
  watchdog_timeout_ms: 50             # no policy output → e-stop

Subscribes:
  /imu/data          Imu
  /motor_states      MotorStateArray
  /joint_commands    MITCommandArray    ← watchdog: must arrive at 50Hz

Publishes:
  /safety/status     Bool              @ 10Hz (true=OK, false=ESTOP)
  /safety/faults     String            @ on fault

Checks (every 20ms):
  1. IMU pitch/roll within limits
  2. All motor temps < threshold
  3. No motor fault codes
  4. Policy node alive (watchdog)
  5. CAN nodes alive
  If any fails → publish false → state_machine → ESTOP
```

### 6. keyboard_teleop (biped_teleop)
```
Node: /keyboard_teleop
Publishes:
  /cmd_vel    Twist    @ on keypress

Controls (turtlebot-style):
  ┌───────────────────────────────────┐
  │  Keyboard Velocity Commander      │
  │                                   │
  │     w: +forward   s: -forward     │
  │     a: +left      d: -right       │
  │     q: +yaw_left  e: +yaw_right   │
  │                                   │
  │     x: stop (zero all)            │
  │     1-5: speed presets             │
  │       1: 0.1 m/s (crawl)          │
  │       2: 0.3 m/s (slow walk)      │
  │       3: 0.5 m/s (normal)         │
  │       4: 0.8 m/s (fast walk)      │
  │       5: 1.0 m/s (jog)            │
  │                                   │
  │  Each w/s press: ±0.1 m/s step    │
  │  Each a/d press: ±0.1 m/s step    │
  │  Each q/e press: ±0.1 rad/s step  │
  │  Holds last command until changed  │
  └───────────────────────────────────┘

Publishes Twist:
  linear.x  = forward velocity (m/s)   ← maps to policy lin_vel_x
  linear.y  = lateral velocity (m/s)   ← maps to policy lin_vel_y
  angular.z = yaw rate (rad/s)         ← maps to policy ang_vel_z
```

### 7. gamepad_teleop (biped_teleop)
```
Node: /gamepad_teleop
Subscribes:
  /joy    Joy    ← from ros2 joy_node (BT gamepad)

Publishes:
  /cmd_vel    Twist    @ 30Hz

Mapping (PS4/PS5 style):
  Left stick Y  → linear.x (forward/back)
  Left stick X  → linear.y (lateral)
  Right stick X → angular.z (yaw)
  R2 trigger    → speed multiplier (0.3–1.0 m/s)
  
Buttons → /state_machine triggers via /joy topic
```

### 8. calibrate_node (biped_tools)
```
Node: /calibrate
Params:
  output_file: "config/calibration.yaml"

Subscribes:
  /motor_states    MotorStateArray

Interactive CLI:
  For each joint (12 total):
    "Move [L_hip_pitch] to LIMIT A, press Enter..."
    → records encoder position
    "Move [L_hip_pitch] to LIMIT B, press Enter..."
    → records encoder position
    
  After all joints:
    - Computes range = |limit_a - limit_b|
    - Computes offset = maps encoder zero to URDF zero
    - Validates: range matches URDF joint limits ±10%
    - Saves calibration.yaml:
        L_hip_pitch:
          encoder_limit_a: 2.145
          encoder_limit_b: -1.892
          urdf_limit_a: 2.228
          urdf_limit_b: -2.222
          offset: 0.034
          range_error_pct: 2.1
```

## Launch Files

### bringup.launch.py (full robot)
```python
# Nodes launched:
# 1. can_bus_node ×1 (manages can0=right + can1=left)
# 2. imu_node
# 3. safety_node
# 4. state_machine_node
# 5. policy_node
# 6. gamepad: joy_node + gamepad_teleop
# 7. foxglove_bridge
```

### hardware.launch.py (drivers only, no policy)
```python
# For testing hardware before policy integration
# 1. can_bus_node ×1 (both buses)
# 2. imu_node
# 3. safety_node
# Useful with: ros2 topic echo /joint_states
```


### calibrate.launch.py
```python
# 1. can_bus_node ×1 (no calibration loaded, raw encoder)
# 2. calibrate_node (interactive)
```

### record.launch.py
```python
# 1. foxglove_bridge (WebSocket on port 8765)
# 2. ros2 bag record (all topics)
```


## ROS2 Graph

All nodes, topics, and connections for the full bringup (`bringup.launch.py`):

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              HARDWARE LAYER                                 │
│                                                                             │
│  ┌─────────────────────────────────┐     ┌────────────────────────────┐    │
│  │         /imu_node               │     │      /can_bus_node         │    │
│  │        (biped_driver)           │     │      (biped_driver)        │    │
│  │                                 │     │                            │    │
│  │  BNO085 I2C (SH2 @ 400kHz)     │     │  robstride_dynamics lib    │    │
│  │  Bus 1, addr 0x4B, RST GPIO 4  │     │  BipedMotorManager         │    │
│  │                                 │     │                            │    │
│  │  Publishes:                     │     │  can0 → Right leg (6×)    │    │
│  │   /imu/data       Imu    100Hz │     │    R_hip_pitch:1:RS04     │    │
│  │   /imu/gravity    Vec3   100Hz │     │    R_hip_roll:2:RS03      │    │
│  │                                 │     │    R_hip_yaw:3:RS03       │    │
│  └────────┬────────────┬───────────┘     │    R_knee:4:RS04          │    │
│           │            │                 │    R_foot_pitch:5:RS02    │    │
│           │            │                 │    R_foot_roll:6:RS02     │    │
│           │            │                 │                            │    │
│           │            │                 │  can1 → Left leg (6×)     │    │
│           │            │                 │    L_hip_pitch:7:RS04     │    │
│           │            │                 │    L_hip_roll:8:RS03      │    │
│           │            │                 │    L_hip_yaw:9:RS03       │    │
│           │            │                 │    L_knee:10:RS04         │    │
│           │            │                 │    L_foot_pitch:11:RS02   │    │
│           │            │                 │    L_foot_roll:12:RS02    │    │
│           │            │                 │                            │    │
│           │            │                 │  + ankle linkage xform    │    │
│           │            │                 │                            │    │
│           │            │                 │  Subscribes:               │    │
│           │            │                 │   /joint_commands  MITArr  │    │
│           │            │                 │  Publishes:                │    │
│           │            │                 │   /joint_states  JS  50Hz │    │
│           │            │                 │   /motor_states  MS  50Hz │    │
│           │            │                 └──────┬──────────┬─────────┘    │
│           │            │                        │          │              │
└───────────┼────────────┼────────────────────────┼──────────┼──────────────┘
            │            │                        │          │
            │ /imu/data  │ /imu/gravity    /joint │   /motor │
            │            │                _states │   _states│
            │            │                        │          │
┌───────────┼────────────┼────────────────────────┼──────────┼──────────────┐
│           │        CONTROL LAYER                │          │              │
│           │            │                        │          │              │
│           ▼            │                        ▼          │              │
│  ┌─────────────────────┼──────────────────────────┐       │              │
│  │      /policy_node   │   (biped_control)        │       │              │
│  │                     │                          │       │              │
│  │  Subscribes:        │                          │       │              │
│  │   /imu/data ────────┘                          │       │              │
│  │   /imu/gravity                                 │       │              │
│  │   /joint_states ◀──────────────────────────────┘       │              │
│  │   /cmd_vel                                             │              │
│  │   /state_machine                                       │              │
│  │                                                        │              │
│  │  ONNX inference (50Hz):                                │              │
│  │   obs(45d) → student_flat.onnx → action(12d)          │              │
│  │   target = default_pos + action × 0.5                  │              │
│  │                                                        │              │
│  │  Publishes:                                            │              │
│  │   /joint_commands  MITCommandArray  50Hz ──────────────┼──▶ can_bus   │
│  │   /policy/obs      Float32MultiArray (debug)           │              │
│  │   /policy/actions  Float32MultiArray (debug)           │              │
│  └────────────────────────────────────────────────┘       │              │
│                                                           │              │
│  ┌────────────────────────────────────────────────┐       │              │
│  │      /safety_node   (biped_control)            │       │              │
│  │                                                │       │              │
│  │  Subscribes:                                   │       │              │
│  │   /imu/data                                    │       │              │
│  │   /motor_states ◀──────────────────────────────┼───────┘              │
│  │   /joint_commands (watchdog)                   │                      │
│  │                                                │                      │
│  │  Checks @ 50Hz:                                │                      │
│  │   pitch/roll limits, motor temp, fault codes,  │                      │
│  │   policy watchdog, CAN watchdog                │                      │
│  │                                                │                      │
│  │  Publishes:                                    │                      │
│  │   /safety/status   Bool          10Hz          │                      │
│  │   /safety/faults   String        on fault      │                      │
│  └────────────────────┬───────────────────────────┘                      │
│                       │                                                  │
│                       │ /safety/status                                   │
│                       ▼                                                  │
│  ┌────────────────────────────────────────────────┐                      │
│  │      /state_machine_node  (biped_control)      │                      │
│  │                                                │                      │
│  │  Subscribes:                                   │                      │
│  │   /motor_states, /imu/data                     │                      │
│  │   /safety/status, /joy                         │                      │
│  │                                                │                      │
│  │  FSM: IDLE → STAND → WALK → ESTOP             │                      │
│  │                                                │                      │
│  │  Publishes:                                    │                      │
│  │   /state_machine  String         10Hz          │                      │
│  │   /robot_state    RobotState     10Hz          │                      │
│  └────────────────────────────────────────────────┘                      │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────┐
│                            TELEOP LAYER                                  │
│                                                                          │
│  ┌────────────────────┐   ┌────────────────────┐   ┌──────────────────┐ │
│  │  /keyboard_teleop  │   │  /gamepad_teleop   │   │    /joy_node     │ │
│  │  (biped_teleop)    │   │  (biped_teleop)    │◀──│  (ros2 joy pkg)  │ │
│  │                    │   │                    │   │                  │ │
│  │  Terminal input    │   │  PS4/PS5 gamepad   │   │  BT HID → /joy  │ │
│  │  WASD + speed 1-5  │   │  Sticks → velocity │   │                  │ │
│  │                    │   │  Buttons → FSM     │   └──────────────────┘ │
│  └────────┬───────────┘   └────────┬───────────┘            /joy        │
│           │                        │                                     │
│           └──────────┬─────────────┘                                     │
│                      │                                                   │
│                 /cmd_vel (Twist)                                         │
│                      │                                                   │
│                      ▼  ──────────────────────▶  /policy_node            │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────┐
│                          MONITORING LAYER                                │
│                                                                          │
│  ┌────────────────────────────────────────────────────────┐              │
│  │  /foxglove_bridge                                      │              │
│  │  WebSocket on port 8765                                │              │
│  │                                                        │              │
│  │  Bridges ALL topics to Foxglove Studio over WiFi       │              │
│  │  /joint_states, /motor_states, /imu/data,              │              │
│  │  /policy/obs, /policy/actions, /robot_state,           │              │
│  │  /safety/status, /cmd_vel, /joint_commands             │              │
│  └────────────────────────────────────────────────────────┘              │
│                          │                                               │
│                     WiFi / WebSocket                                     │
│                          │                                               │
│                          ▼                                               │
│               ┌────────────────────┐                                     │
│               │  Desktop Machine   │                                     │
│               │  Foxglove Studio   │                                     │
│               └────────────────────┘                                     │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

TOPIC SUMMARY
─────────────────────────────────────────────────────────
Topic                  Type                  Hz   Publisher        Subscribers
/imu/data              Imu                   100  imu_node         policy, safety
/imu/gravity           Vector3Stamped        100  imu_node         policy
/joint_states          JointState            50   can_bus_node     policy, foxglove
/motor_states          MotorStateArray       50   can_bus_node     safety, state_machine, foxglove
/joint_commands        MITCommandArray       50   policy_node      can_bus_node, safety (watchdog)
/cmd_vel               Twist                 var  teleop nodes     policy
/state_machine         String                10   state_machine    policy
/robot_state           RobotState            10   state_machine    foxglove
/safety/status         Bool                  10   safety_node      state_machine
/safety/faults         String                var  safety_node      foxglove
/joy                   Joy                   var  joy_node         gamepad_teleop, state_machine
/policy/obs            Float32MultiArray     50   policy_node      foxglove (debug)
/policy/actions        Float32MultiArray     50   policy_node      foxglove (debug)

NODE SUMMARY (8 nodes in bringup.launch.py)
─────────────────────────────────────────────────────────
Node                  Package          Function
/imu_node             biped_driver     BNO085 I2C → IMU + gravity
/can_bus_node         biped_driver     12× RobStride MIT control (can0+can1)
/policy_node          biped_control    ONNX inference → joint commands (50Hz)
/safety_node          biped_control    Watchdog + e-stop triggers
/state_machine_node   biped_control    FSM: IDLE→STAND→WALK→ESTOP
/keyboard_teleop      biped_teleop     Terminal → /cmd_vel
/gamepad_teleop       biped_teleop     PS4/PS5 → /cmd_vel
/joy_node             joy (ros2 pkg)   BT gamepad HID → /joy
```

## Frame Conventions

```
Isaac Lab (training):
  - World: Z-up, gravity = (0, 0, -9.81)
  - Robot forward: +X
  - projected_gravity = quat_rotate_inverse(body_quat, [0, 0, -1])
    → unit vector, gravity direction in body frame

BNO085:
  - Default: Z-up, X-forward (matches Isaac when mounted correctly)
  - Interface: I2C at 400kHz (safe, reliable on RPi 5)
  - SH2_GRAVITY report: gravity acceleration in body frame (m/s²)
    → ~(0, 0, -9.81) when upright
  - MUST normalize: proj_grav = gravity / |gravity|
  - SH2_GYRO_INTEGRATED_RV angVel: body-frame angular velocity (rad/s)
    → maps directly to base_ang_vel

RS04 Motor:
  - Position: absolute encoder, radians (output shaft / joint-level)
  - Velocity: rad/s (output shaft)
  - Torque: Nm (output shaft, post-gearbox)
  - Positive direction: verify per joint against URDF convention
```

**BNO085 mounting**: The IMU X-axis must point in the robot's +X direction (forward).
If mounted differently, apply a fixed rotation to align frames.

## CAN Timing Budget (50Hz = 20ms)

```
CAN commands (12 joints, bus-grouped):
  can0: 6 joints × ~0.2ms write+read = ~1.2ms
  can1: 6 joints × ~0.2ms write+read = ~1.2ms
  Total (sequential, grouped by bus):    ~2.4ms
IMU I2C read (SH2):                      ~0.2ms
Obs vector assembly:                     ~0.1ms
ONNX inference (45→128→128→128→12):      ~0.5ms
Safety checks:                           ~0.1ms
────────────────────────────────────────────────────
Total:                                   ~3.3ms
Headroom:                                16.7ms  ✅

Note: Joints are grouped by bus in the control loop (all can0 joints
processed before can1) to keep each bus's traffic contiguous. The two
buses have independent sockets so no cross-bus contention. The ~0.2ms
per motor assumes typical response time; worst case with 5ms timeout
per motor is 60ms (only on hardware failure, triggers safety e-stop).
```

## Known Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Joint order mismatch (sim vs real) | Wrong actions to wrong joints → crash | Print joint order at startup, verify against URDF |
| projected_gravity not normalized | Obs magnitude wrong → policy confused | Normalize in obs_builder, add assertion |
| Action delay (CAN + compute ~3ms) | Slight phase lag vs sim | Training includes last_action (12d) which partially compensates. If severe, add 1-step action delay to training. |
| Sim Kp/Kd → real Kp/Kd mismatch | Poor tracking, oscillation or soft | Start at sim values, tune incrementally. RS04 MIT Kp operates at joint level (post-gearbox). |
| IMU frame misalignment | gravity/gyro axes swapped → falls | Verify IMU orientation at startup: upright robot should read gravity ≈ (0, 0, -1) normalized |
| CAN bus failure (loose connector) | Partial joint control → crash | CAN_TIMEOUT on each motor (100ms), safety_node CAN watchdog |
| WiFi latency for Foxglove | Choppy visualization | Foxglove is monitoring only, not in control loop. Acceptable. |
| BNO085 I2C bus contention | Shared I2C bus slows down if other devices present | BNO085 is the only I2C device. Dedicated bus 1. |
