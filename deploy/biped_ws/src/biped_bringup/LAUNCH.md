# Launch Files

All launch files live in `biped_bringup/launch/`. Motor config comes from `robot.yaml` — never hardcoded in launch files.

## bringup.launch.py — Full Stack

Everything needed to walk + automatic rosbag recording.

**Nodes:** robot_state_publisher → imu_node → can_bus_node → safety_node → state_machine_node → policy_node + rosbag recorder

With `unified:=true`, the imu / can / policy nodes are replaced by a single
`biped_unified_cpp/unified_node` process (lowest latency, CPU-pinned);
safety_node and state_machine_node still run standalone. Python vs C++
variants of the standalone nodes are selected via `can_driver`,
`control_driver`, and `imu_type`.

Records MCAP bags to `~/biped_lower/bags/<timestamp>/` on every run. Open `.mcap` files directly in Foxglove Studio for playback.

```bash
ros2 launch biped_bringup bringup.launch.py                    # full send + recording
ros2 launch biped_bringup bringup.launch.py gain_scale:=0.3    # safe start (30% gains)
ros2 launch biped_bringup bringup.launch.py record:=false       # disable recording
ros2 launch biped_bringup bringup.launch.py calibration_file:=calibration.yaml
```

| Param | Default | Description |
|---|---|---|
| `unified` | `false` | `true` = single-process C++ node for IMU+CAN+Policy |
| `can_driver` | `can_bus_node` | `can_bus_node_cpp` (C++, recommended) or `can_bus_node` (Python) |
| `control_driver` | `biped_control` | `biped_control_cpp` (C++) or `biped_control` (Python) |
| `imu_type` | `bno085` | `bno085` / `bno085_cpp` / `im10a` / `im10a_cpp` |
| `robot_config` | `config/robot.yaml` | Motor → CAN bus mapping |
| `calibration_file` | _(none)_ | Joint offset calibration |
| `control_params_file` | `config/control_params.yaml` | Default pos / PD gains / joint limits |
| `onnx_model` | `student_flat.onnx` | Policy model file |
| `gain_scale` | `1.0` | PD gain multiplier (use 0.3 for testing) |
| `max_pitch_deg` | `85.0` | Safety pitch limit |
| `max_roll_deg` | `85.0` | Safety roll limit |
| `record` | `true` | Enable MCAP rosbag recording |

### Recorded Topics

| Topic | Rate | Content |
|---|---|---|
| `/joint_states` | 50Hz | Joint-space positions, velocities, effort |
| `/motor_states` | 50Hz | Motor command-space pos + temp + faults |
| `/joint_commands` | 50Hz | Policy/state_machine commands (pos, kp, kd) |
| `/imu/data` | 50Hz | Gyro + orientation |
| `/imu/gravity` | 50Hz | Gravity vector |
| `/cmd_vel` | — | Velocity commands |
| `/state_machine` | 10Hz | FSM state (IDLE/STAND/WALK/ESTOP) |
| `/robot_state` | 10Hz | FSM + safety status |
| `/safety/status` | 50Hz | Safety OK bool |
| `/safety/fault` | event | Fault description |
| `/tf` | 50Hz | Joint transforms |

**Estimated size:** ~7 MB/min. Bags split every 5 minutes.

### Playback

1. Copy bags from Pi: `scp -r pi:~/biped_lower/bags/<timestamp> .`
2. Open `.mcap` file directly in Foxglove Studio (File → Open local file)
3. Or via Foxglove bridge: `ros2 bag play <bag_dir>` + connect Foxglove to `ws://<pi>:8765`

## hardware.launch.py — Hardware Test

IMU + motors + safety, no policy or state machine. Use for hardware checkout before running policy.

**Nodes:** robot_state_publisher → imu_node → can_bus_node → safety_node

```bash
ros2 launch biped_bringup hardware.launch.py
ros2 launch biped_bringup hardware.launch.py \
  can_driver:=can_bus_node_cpp control_driver:=biped_control_cpp \
  imu_type:=bno085_cpp calibration_file:=calibration.yaml
```

| Param | Default | Description |
|---|---|---|
| `can_driver` | `can_bus_node` | `can_bus_node_cpp` (C++, recommended) or `can_bus_node` (Python) |
| `control_driver` | `biped_control` | Selects safety node variant (`biped_control_cpp` = C++) |
| `imu_type` | `bno085` | `bno085` / `bno085_cpp` / `im10a` |
| `robot_config` | `config/robot.yaml` | Motor → CAN bus mapping |
| `calibration_file` | _(none)_ | Joint offset calibration |
| `control_params_file` | `config/control_params.yaml` | Default pos / PD gains / joint limits |
| `max_pitch_deg` | `85.0` | Safety pitch limit |
| `max_roll_deg` | `85.0` | Safety roll limit |

## calibrate.launch.py — Motor Calibration

Interactive calibration tool. Manages CAN directly — do **not** run alongside can_bus_node.

**Nodes:** calibrate_node

```bash
ros2 launch biped_bringup calibrate.launch.py                  # all joints
ros2 launch biped_bringup calibrate.launch.py joint:=R_knee    # single joint
ros2 launch biped_bringup calibrate.launch.py output_file:=calibration.yaml
```

## record.launch.py — Foxglove Bridge

Foxglove WebSocket bridge for live visualization. Run alongside any other launch file.

**Nodes:** foxglove_bridge

```bash
ros2 launch biped_bringup record.launch.py
# Connect Foxglove Studio to ws://<pi-ip>:8765
```

## Typical Workflow

```bash
# 1. Setup CAN buses
bash ~/biped_lower/deploy/scripts/setup_can.sh

# 2. Scan motors
python3 ~/biped_lower/deploy/scripts/scan_motors.py

# 3. Hardware test (no policy)
ros2 launch biped_bringup hardware.launch.py

# 4. Full bringup (recording starts automatically)
ros2 launch biped_bringup bringup.launch.py gain_scale:=2.0 \
    calibration_file:=/home/roy/biped_lower/deploy/biped_ws/calibration.yaml \
    onnx_model:=/home/roy/biped_lower/deploy/student_flat.onnx

# 5. (Optional) Live Foxglove in another terminal
ros2 launch biped_bringup record.launch.py

# 6. After session, bags are at ~/biped_lower/bags/
ls ~/biped_lower/bags/
```
