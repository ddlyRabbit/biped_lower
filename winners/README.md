# Winner Checkpoints — Forward Axis: -Y

All models trained with **-Y forward axis**. Not compatible with +X forward URDFs without retraining.

## v55b/model_5997.pt — Flat Teacher (no self-collisions)
- **Reward**: 21.5 | TrackVel: 0.90 | ep_len: 1000 (max)
- **Obs**: 48d (teacher) | **Actions**: 12 joint position targets
- **Self-collisions**: OFF
- **Actuators**: ImplicitActuator, original Berkeley values
- **Notes**: Best flat terrain policy before self-collision support

## v57_model_2899.pt — Flat Teacher (self-collisions ON)
- **Reward**: 19.1 | TrackVel: 0.84 | ep_len: 955
- **Obs**: 48d (teacher) | **Actions**: 12 joint position targets
- **Self-collisions**: ON | Height-based contact detection (0.063m threshold)
- **Actuators**: ImplicitActuator, hip_roll/yaw 50Nm, hip_pitch/knee 100Nm, foot 15Nm
- **Training**: From scratch → impact-based feet_air refinement → 2899 iters
- **Notes**: Teacher for student distillation pipeline

## v57_rough_model_19200.pt — Rough Teacher
- **Reward**: 18.0 | TrackVel: 0.83 | crash: 13.7% | ep_len: ~915
- **Obs**: 235d (48 proprio + 187 height_scan) | **Actions**: 12
- **Self-collisions**: ON
- **Actuators**: ImplicitActuator, hip_roll/yaw 50Nm, hip_pitch/knee 100Nm, foot 30Nm
- **Terrain**: 7 sub-terrains (slopes, stairs, rough, stepping stones)
- **Commands**: lin_vel_y=(-1.5, 0.5) forward=-Y, lin_vel_x=(-0.5, 0.5) lateral
- **Training**: Resumed from flat v57 → 20,599 rough iters, peak at 19,200

## v57_student_flat_model_4200.pt — Flat Student (deployable)
- **Reward**: 19.1 (100% teacher) | peak 19.11
- **Obs**: 45d (no base_lin_vel) | **Actions**: 12
- **Self-collisions**: ON
- **Pipeline**: Teacher(v57_2899) → Distill(2400 iters, loss 0.20) → PPO fine-tune(5000 iters)
- **Actuators**: ImplicitActuator, hip_roll/yaw 50Nm, hip_pitch/knee 100Nm, foot 30Nm
- **Fine-tune config**: LR 3e-4, clip 0.1, entropy 0.001 (conservative — protects pre-trained actor)
- **Notes**: Removes only base_lin_vel (not available on real IMU). Deployable on hardware.

## Common Config
- **URDF**: `urdf/robot.urdf` (-Y forward, 12 DoF, ~30kg with 10kg battery)
- **Framework**: rsl_rl, PPO [128,128,128] ELU
- **Forward axis**: **-Y** (commands use lin_vel_y for forward motion)
- **Play command**:
  ```bash
  docker run --gpus all -e DISPLAY=:99 -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /workspace:/workspace -v /results:/results -v /uploads:/uploads \
    isaaclab:latest /isaac-sim/python.sh \
    /workspace/biped_locomotion/biped_play_rsl.py \
    [--student] [--rough] --checkpoint /results/winners/<checkpoint> \
    --video --video_length 300 --num_envs 8 --headless
  ```
