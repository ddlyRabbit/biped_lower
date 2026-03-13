# Biped Deployment Architecture

ROS2 Jazzy on RPi 5 (Ubuntu 24.04). Single SocketCAN bus (`can0`), all 12 RobStride motors.
Student RL policy (ONNX, 45d obs → 12d actions). BNO085 IMU on I2C. +X forward frame convention.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Compute | Raspberry Pi 5 (8 GB), Ubuntu 24.04 aarch64 |
| CAN | Waveshare RS485 CAN HAT (B) — MCP2515 on SPI0, SocketCAN `can0`, 1 Mbps |
| IMU | BNO085 on I2C bus 1, addr 0x4B, RST GPIO 4, SH2 protocol @ 400 kHz |
| Motors | 12× RobStride (4× RS04, 4× RS03, 4× RS02) on single CAN bus |
| Policy | `student_flat.onnx` — MLP 45→128→128→128→12, ~0.5 ms inference |

## Motor CAN IDs

| Joint | CAN ID | Model | Torque (Nm) |
|-------|--------|-------|-------------|
| R_hip_pitch | 1 | RS04 | 12.0 |
| R_hip_roll | 2 | RS03 | 7.5 |
| R_hip_yaw | 3 | RS03 | 7.5 |
| R_knee | 4 | RS04 | 12.0 |
| R_foot_pitch | 5 | RS02 | 4.0 |
| R_foot_roll | 6 | RS02 | 4.0 |
| L_hip_pitch | 7 | RS04 | 12.0 |
| L_hip_roll | 8 | RS03 | 7.5 |
| L_hip_yaw | 9 | RS03 | 7.5 |
| L_knee | 10 | RS04 | 12.0 |
| L_foot_pitch | 11 | RS02 | 4.0 |
| L_foot_roll | 12 | RS02 | 4.0 |

Host CAN ID: `0xFD` (253) — higher than all motor IDs for optimal CAN arbitration.

## URDF Joint Limits

| Joint | Lower (rad) | Upper (rad) | Default (rad) |
|-------|-------------|-------------|---------------|
| R_hip_pitch | −2.217 | 1.047 | 0.2 |
| R_hip_roll | −2.269 | 0.209 | 0.0 |
| R_hip_yaw | −1.571 | 1.571 | 0.0 |
| R_knee | 0.000 | 2.705 | 0.4 |
| R_foot_pitch | −0.873 | 0.524 | −0.2 |
| R_foot_roll | −0.262 | 0.262 | 0.0 |
| L_hip_pitch | −1.047 | 2.217 | −0.2 |
| L_hip_roll | −0.209 | 2.269 | 0.0 |
| L_hip_yaw | −1.571 | 1.571 | 0.0 |
| L_knee | 0.000 | 2.705 | 0.4 |
| L_foot_pitch | −0.873 | 0.524 | −0.2 |
| L_foot_roll | −0.262 | 0.262 | 0.0 |

## PD Gains

| Joint group | Kp | Kd | Notes |
|-------------|-----|-----|-------|
| hip_pitch | 15.0 | 3.0 | RS04 — MIT mode, output-shaft level |
| hip_roll | 10.0 | 3.0 | RS03 |
| hip_yaw | 10.0 | 3.0 | RS03 |
| knee | 15.0 | 3.0 | RS04 |
| foot_pitch | 2.0 | 0.2 | RS02 — through ankle linkage |
| foot_roll | 2.0 | 0.2 | RS02 — through ankle linkage |

Start with `gain_scale:=0.3` and increase. RS04 MIT Kp range: 0–5000.

---

## Package Structure

```
deploy/
├── ARCHITECTURE.md              ← this file
├── BRINGUP.md                   ← setup & operations guide
├── student_flat.onnx            ← exported policy
├── scripts/
│   ├── setup_can.sh             ← bring up can0 (run each boot)
│   └── scan_motors.py           ← read-only motor scan
├── foxglove/
│   └── biped_hardware.json      ← Foxglove Studio layout
└── biped_ws/src/
    ├── biped_msgs/              ← Custom message definitions
    │   └── msg/
    │       ├── MITCommand.msg
    │       ├── MITCommandArray.msg
    │       ├── MotorState.msg
    │       ├── MotorStateArray.msg
    │       └── RobotState.msg
    ├── biped_driver/            ← Hardware interface
    │   └── biped_driver/
    │       ├── robstride_dynamics/   ← Vendored Seeed library (no ROS)
    │       │   ├── protocol.py       ← CAN comm types + parameter IDs
    │       │   ├── table.py          ← Per-model MIT scaling tables
    │       │   └── bus.py            ← RobstrideBus: connect, MIT R/W, param R/W
    │       ├── robstride_can.py      ← BipedMotorManager, ankle linkage, soft stops
    │       ├── can_bus_node.py       ← ROS2 node: 12 motors on can0
    │       └── imu_node.py           ← BNO085 I2C node
    ├── biped_control/           ← Policy + state machine
    │   └── biped_control/
    │       ├── obs_builder.py        ← Sensor → 45d observation
    │       ├── policy_node.py        ← 50Hz ONNX inference
    │       ├── state_machine_node.py ← IDLE→STAND→WALK→ESTOP
    │       └── safety_node.py        ← Watchdog, e-stop triggers
    ├── biped_teleop/            ← Velocity commands
    │   └── biped_teleop/
    │       └── keyboard_teleop.py    ← Terminal → /cmd_vel
    ├── biped_tools/             ← Calibration + utilities
    │   └── biped_tools/
    │       ├── calibrate_node.py     ← Interactive homing
    │       └── export_onnx.py        ← PyTorch → ONNX
    └── biped_bringup/           ← Launch files + config
        ├── launch/
        │   ├── bringup.launch.py     ← Full robot (all nodes)
        │   ├── hardware.launch.py    ← IMU + CAN + safety (no policy)
        │   ├── calibrate.launch.py   ← Interactive calibration
        │   └── record.launch.py      ← Foxglove bridge + rosbag
        └── config/
            └── robot.yaml            ← Motor map (12 motors on can0)
```

---

## ROS2 Graph

```
┌──────────────────────────────────────────────────────────────────────────┐
│                            HARDWARE LAYER                                │
│                                                                          │
│  ┌────────────────────────┐       ┌──────────────────────────────────┐  │
│  │      /imu_node         │       │        /can_bus_node             │  │
│  │     (biped_driver)     │       │       (biped_driver)             │  │
│  │                        │       │                                  │  │
│  │  BNO085 I2C SH2       │       │  12× RobStride on can0           │  │
│  │  Bus 1, 0x4B, GPIO 4  │       │  BipedMotorManager               │  │
│  │                        │       │  + ankle linkage transform       │  │
│  │  Pub:                  │       │                                  │  │
│  │   /imu/data     100Hz  │       │  Sub: /joint_commands            │  │
│  │   /imu/gravity  100Hz  │       │  Pub: /joint_states  50Hz        │  │
│  │                        │       │       /motor_states  50Hz        │  │
│  └──────┬───────┬─────────┘       └──────┬──────────┬────────────────┘  │
│         │       │                        │          │                    │
└─────────┼───────┼────────────────────────┼──────────┼────────────────────┘
          │       │                        │          │
┌─────────┼───────┼────────────────────────┼──────────┼────────────────────┐
│         │   CONTROL LAYER                │          │                    │
│         ▼       │                        ▼          │                    │
│  ┌──────────────┼──────────────────────────┐        │                    │
│  │  /policy_node│     (biped_control)      │        │                    │
│  │              │                          │        │                    │
│  │  Sub: /imu/data, /imu/gravity,          │        │                    │
│  │       /joint_states, /cmd_vel,          │        │                    │
│  │       /state_machine                    │        │                    │
│  │                                         │        │                    │
│  │  ONNX 50Hz: obs(45d) → action(12d)     │        │                    │
│  │  target = default + action × 0.5        │        │                    │
│  │                                         │        │                    │
│  │  Pub: /joint_commands  50Hz ────────────┼──▶ can │                    │
│  └─────────────────────────────────────────┘        │                    │
│                                                     │                    │
│  ┌─────────────────────────────────────────┐        │                    │
│  │  /safety_node       (biped_control)     │        │                    │
│  │                                         │        │                    │
│  │  Sub: /imu/data, /motor_states,         │◀───────┘                    │
│  │       /joint_commands (watchdog)        │                             │
│  │                                         │                             │
│  │  Checks: pitch/roll, temp, faults       │                             │
│  │  Pub: /safety/status  10Hz              │                             │
│  └──────────────┬──────────────────────────┘                             │
│                 │                                                        │
│                 ▼                                                        │
│  ┌─────────────────────────────────────────┐                             │
│  │  /state_machine_node  (biped_control)   │                             │
│  │                                         │                             │
│  │  Sub: /safety/status, /joint_states,    │                             │
│  │       /state_command                    │                             │
│  │                                         │                             │
│  │  FSM: IDLE → STAND → WALK → ESTOP      │                             │
│  │  STAND: soft start (2s interp + 1s     │                             │
│  │         gain ramp)                      │                             │
│  │  ESTOP: sends kp=0/kd=0 → motors free  │                             │
│  │                                         │                             │
│  │  Pub: /state_machine  10Hz              │                             │
│  │       /robot_state    10Hz              │                             │
│  │       /joint_commands (during STAND)    │                             │
│  └─────────────────────────────────────────┘                             │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────────────┐
│  TELEOP: /keyboard_teleop → /cmd_vel (Twist)                            │
│  MONITORING: /foxglove_bridge (ws://pi:8765) + rosbag                   │
└──────────────────────────────────────────────────────────────────────────┘
```

### Topic Summary

| Topic | Type | Hz | Publisher | Subscribers |
|-------|------|-----|----------|-------------|
| /imu/data | Imu | 100 | imu_node | policy, safety |
| /imu/gravity | Vector3Stamped | 100 | imu_node | policy |
| /joint_states | JointState | 50 | can_bus_node | policy, state_machine |
| /motor_states | MotorStateArray | 50 | can_bus_node | safety |
| /joint_commands | MITCommandArray | 50 | policy / state_machine | can_bus_node, safety |
| /cmd_vel | Twist | var | teleop | policy |
| /state_machine | String | 10 | state_machine | policy |
| /robot_state | RobotState | 10 | state_machine | foxglove |
| /safety/status | Bool | 10 | safety_node | state_machine |
| /safety/fault | String | var | safety_node | foxglove |
| /state_command | String | var | user | state_machine |

---

## Custom Messages

### MITCommand.msg
```
string joint_name
float32 position      # target position (rad)
float32 velocity      # target velocity (rad/s), typically 0
float32 kp            # position gain
float32 kd            # velocity gain
float32 torque_ff     # feedforward torque (Nm), typically 0
```

### MITCommandArray.msg
```
std_msgs/Header header
biped_msgs/MITCommand[] commands
```

### MotorState.msg
```
string joint_name
uint8 can_id
float32 position      # calibrated position (rad)
float32 velocity      # velocity (rad/s)
float32 torque        # torque (Nm)
float32 temperature   # motor temp (°C)
uint8 fault_code      # 6-bit fault flags
uint8 mode_status     # 0=Reset, 1=Calibration, 2=Run
```

### MotorStateArray.msg
```
std_msgs/Header header
biped_msgs/MotorState[] motors
```

### RobotState.msg
```
std_msgs/Header header
string fsm_state
bool safety_ok
bool all_motors_enabled
float32 battery_voltage
string[] active_faults
```

---

## Observation Vector (45d)

Must match training order exactly:

| Index | Dim | Name | Source |
|-------|-----|------|--------|
| 0–2 | 3 | base_ang_vel | BNO085 gyro (rad/s, body frame) |
| 3–5 | 3 | projected_gravity | −BNO085 gravity / ‖g‖ (unit vector, Isaac convention) |
| 6–8 | 3 | velocity_commands | /cmd_vel (lin_x, lin_y, ang_z) |
| 9–14 | 6 | hip_pos | joint_pos − default: L_roll, R_roll, L_yaw, R_yaw, L_pitch, R_pitch |
| 15–16 | 2 | knee_pos | joint_pos − default: L_knee, R_knee |
| 17–18 | 2 | foot_pitch_pos | joint_pos − default: L, R |
| 19–20 | 2 | foot_roll_pos | joint_pos − default: L, R |
| 21–32 | 12 | joint_vel | CAN velocity (Isaac runtime order) |
| 33–44 | 12 | last_action | Previous policy output (dimensionless) |

### Isaac Runtime Joint Order
```
 0: L_hip_pitch    6: L_knee
 1: R_hip_pitch    7: R_knee
 2: L_hip_roll     8: L_foot_pitch
 3: R_hip_roll     9: R_foot_pitch
 4: L_hip_yaw     10: L_foot_roll
 5: R_hip_yaw     11: R_foot_roll
```

### projected_gravity
BNO085 `SH2_GRAVITY` reports acceleration (~9.81 m/s²). **Must normalize** to unit vector
then **negate** to match Isaac convention: `proj_grav = −gravity / ‖gravity‖`.

---

## Action Pipeline

```
Policy output: action[12]  (dimensionless, ~ [-1, 1])
       │
       ▼
Position target: target[i] = default_pos[i] + action[i] × 0.5
       │
       ▼
MIT command: { position=target, velocity=0, kp=gain×scale, kd=gain×scale, torque_ff=0 }
       │
       ▼
Motor executes: τ = Kp × (target − actual) + Kd × (0 − vel)
```

Matches Isaac's `ImplicitActuator` PD control.

---

## Ankle Parallel Linkage

Two RS02 motors per ankle, asymmetric rod lengths, U-joint to foot plate.

**From Onshape CAD:**
- Crank radius: 32.249 mm
- Upper rod: 192 mm (knee-side motor = foot_pitch CAN ID)
- Lower rod: 98 mm (foot-side motor = foot_roll CAN ID)
- Foot attachment: ±31.398 mm lateral, 41.14 mm forward

**Forward transform** (joint → motors):
```
motor_upper = pitch_gain × pitch + roll_gain × roll    (pitch_gain = 1.2757)
motor_lower = pitch_gain × pitch − roll_gain × roll    (roll_gain  = 0.9736)
```

**Inverse transform** (motors → joint):
```
foot_pitch = inv_pitch × (upper + lower)    (inv_pitch = 0.3919)
foot_roll  = inv_roll  × (upper − lower)    (inv_roll  = 0.5136)
```

Policy sees `foot_pitch` / `foot_roll` in joint-space. `can_bus_node` transparently
transforms to/from motor-space via these linear gains.

---

## Soft Stops

Every joint has a 2° (0.035 rad) buffer inside URDF limits. When a joint enters
the buffer zone, a restoring spring torque is applied:

```
τ_restore = Kp_softstop × min(penetration, buffer)
Kp_softstop = 20 Nm/rad
```

For normal joints: applied in `send_mit_command()` via `torque_ff`.
For ankle joints: applied in joint-space (before linkage transform) by `can_bus_node._handle_ankle()`.

---

## Calibration

`calibrate_node` manages CAN directly (no `can_bus_node` needed). Enables all 12 motors
with zero torque in MIT mode and tracks raw encoder min/max as the user moves each joint.

- **Normal joints:** `offset = encoder_min − URDF_lower_limit`
- **Ankle joints:** `offset = encoder_min − motor_cmd_at_joint_min` (accounts for linkage)

Auto-marks joints ✅ when observed range matches expected range within 20%.
Saves `calibration.yaml` with offsets and limits.

---

## Safety

### safety_node checks (50 Hz)
1. IMU pitch/roll within `max_pitch_deg` / `max_roll_deg` (launch args, default 85°)
2. Motor temperatures below 80°C
3. No motor fault codes
4. IMU data alive (200 ms timeout)

On any violation → `/safety/status = false` → `state_machine_node` → ESTOP.

### ESTOP behaviour
`state_machine_node` sends `kp=0 / kd=0 / torque_ff=0` to all joints via `/joint_commands`.
`can_bus_node` forwards these — motors go free (zero stiffness). The control loop always
runs; there is no disabled state in `can_bus_node`.

### Hardware-level safety
Each motor's `CAN_TIMEOUT` (index 0x702B) should be set to 100 ms at startup.
If no CAN command arrives within timeout, motor enters reset mode independently of the RPi.

---

## State Machine

```
IDLE ──START──▶ STAND ──(2s stable)──▶ WALK
                  ▲                      │
                  │◀────STOP─────────────┘
                  │
ANY ──────────▶ ESTOP ──RESET──▶ IDLE
```

### STAND soft start
1. Capture current joint positions
2. Interpolate to default positions over 2 s
3. Ramp gains from 10% → 100% over 1 s
4. Run with zero velocity command

---

## CAN Timing Budget (50 Hz = 20 ms)

```
CAN commands (12 motors on can0):
  12 × ~0.4 ms write+read             ~4.8 ms
IMU I2C read (SH2):                   ~0.2 ms
Obs vector assembly:                   ~0.1 ms
ONNX inference (45→128→128→128→12):   ~0.5 ms
Safety checks:                         ~0.1 ms
──────────────────────────────────────────────
Total:                                 ~5.7 ms
Headroom:                              14.3 ms  ✅
```

The MCP2515 has 3 TX buffer slots. `bus.py` retries on `CanOperationError` with
0.5 ms backoff (up to 5 attempts) to handle TX buffer full conditions. A 0.2 ms
sleep after each `write_operation_frame` lets the SPI transfer complete.

---

## Frame Conventions

| System | Convention |
|--------|-----------|
| Isaac Lab (training) | Z-up, +X forward, gravity = (0, 0, −9.81) |
| BNO085 | Z-up, +X forward (when mounted correctly) |
| projected_gravity | `−gravity / ‖gravity‖` to match Isaac `quat_rotate_inverse(q, [0,0,−1])` |
| Motor positions | Radians, output shaft (post-gearbox), absolute encoder |
| Motor torques | Nm, output shaft |

**BNO085 mounting**: IMU X-axis must point in robot's +X direction (forward).
