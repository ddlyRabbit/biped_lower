# Launch Files

All launch files live in `biped_bringup/launch/`. Motor config comes from `robot.yaml` — never hardcoded in launch files.

## bringup.launch.py — Full Stack

Everything needed to walk. Use this for deployment.

**Nodes:** robot_state_publisher → imu_node → can_bus_node → safety_node → state_machine_node → policy_node

```bash
ros2 launch biped_bringup bringup.launch.py                    # full send
ros2 launch biped_bringup bringup.launch.py gain_scale:=0.3    # safe start (30% gains)
ros2 launch biped_bringup bringup.launch.py calibration_file:=calibration.yaml
```

| Param | Default | Description |
|---|---|---|
| `robot_config` | `config/robot.yaml` | Motor → CAN bus mapping |
| `calibration_file` | _(none)_ | Joint offset calibration |
| `onnx_model` | `student_flat.onnx` | Policy model file |
| `gain_scale` | `1.0` | PD gain multiplier (use 0.3 for testing) |
| `max_pitch_deg` | `85.0` | Safety pitch limit |
| `max_roll_deg` | `85.0` | Safety roll limit |

## hardware.launch.py — Hardware Test

IMU + motors + safety, no policy or state machine. Use for hardware checkout before running policy.

**Nodes:** robot_state_publisher → imu_node → can_bus_node → safety_node

```bash
ros2 launch biped_bringup hardware.launch.py
ros2 launch biped_bringup hardware.launch.py calibration_file:=calibration.yaml
```

| Param | Default | Description |
|---|---|---|
| `robot_config` | `config/robot.yaml` | Motor → CAN bus mapping |
| `calibration_file` | _(none)_ | Joint offset calibration |
| `max_pitch_deg` | `85.0` | Safety pitch limit |
| `max_roll_deg` | `85.0` | Safety roll limit |

## calibrate.launch.py — Motor Calibration

Interactive calibration tool. Manages CAN directly — do **not** run alongside can_bus_node.

**Nodes:** calibrate_node

```bash
ros2 launch biped_bringup calibrate.launch.py
ros2 launch biped_bringup calibrate.launch.py output_file:=calibration.yaml
```

## record.launch.py — Visualization & Recording

Foxglove WebSocket bridge + rosbag. Run alongside any other launch file.

**Nodes:** foxglove_bridge + `ros2 bag record -a`

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

# 4. Full bringup (safe gains first)
ros2 launch biped_bringup bringup.launch.py gain_scale:=0.3

# 5. (Optional) Recording in another terminal
ros2 launch biped_bringup record.launch.py
```
