# Deploy

Runtime for the biped: ROS2 workspace (`biped_ws/`), policy ONNX models, and
setup scripts. Full setup and testing guide: [BRINGUP.md](BRINGUP.md) ·
architecture reference: [ARCHITECTURE.md](ARCHITECTURE.md).

## Launching

> The Python nodes are legacy — use the C++ variants. Launch defaults still
> point to Python for compatibility, so pass the args explicitly.

**Unified node (preferred — lowest latency).** Single C++ process for
IMU + CAN + policy; state machine + safety run standalone alongside it:

```bash
ros2 launch biped_bringup bringup.launch.py \
  unified:=true \
  control_driver:=biped_control_cpp \
  imu_type:=bno085_cpp \
  robot_config:=robot.yaml \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/v155_student_light_6800.onnx \
  gain_scale:=0.3
```

**C++ individual nodes** (same stack as separate processes — easier to debug):

```bash
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  control_driver:=biped_control_cpp \
  imu_type:=bno085_cpp \
  robot_config:=robot.yaml \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/v155_student_light_6800.onnx \
  gain_scale:=0.3
```

Then drive with `ros2 run biped_teleop keyboard_teleop`
(SPACE → STAND, `v` → SIM_WALK, `g` → WALK, ESC → ESTOP).

# Models

## V155 Light Student (v155_student_light_6800.onnx) — current
- **Deployed:** 2026-06-14 (commit 16f97f3)
- **Source:** V155 training run, 6800 iterations — locked hip_yaw (action scale 0.0), fixed CoM.
- **URDF:** light
- **Deploy notes:** hip_yaw action scale is 0.0 in deploy to match training (yaw PD-held at 0). `student_flat.onnx` (the default filename used by launch files) is an identical copy of this model.

## Heavy URDF Fine-tuned (v1_heavy_student_finetuned_20k.onnx) — historical, not in this folder
- **Exported:** 2026-05-22
- **Source:** Phase 3 fine-tuning (20k iterations) on GCP.
- **URDF:** heavy (21.7kg)
- **Metrics:** 24.97 Reward, 0.88 m/s XY vel tracking, 3.3% fall rate.
- **Validation**: MuJoCo rendering videos for 0.0, 0.3, and 0.8 m/s generated and backed up to GCP winners folder.
