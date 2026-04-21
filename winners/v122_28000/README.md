# V122 Teacher @ 28000 (Apr 21, 2026)

## Overview
Best V122 checkpoint — light URDF, tanh output, DelayedPD actuator with action delay.
Trained from V116 checkpoint resume with tuned reward weights (4x foot contact force penalty).

## Final Metrics (iter 28000)
- **Reward**: 35.44
- **Timeout**: 99.87% (almost never falls)
- **Base contact**: 0.13%
- **Velocity tracking**: lin_vel 0.877 (88%), ang_vel 0.862 (86%)

## Reward Breakdown
| Term | Weight | Value |
|------|--------|-------|
| track_lin_vel_xy_exp | +1.0 | +0.877 |
| track_ang_vel_z_exp | +1.0 | +0.862 |
| feet_air_time | +40.0 | +0.308 |
| feet_slide | -0.25 | -0.009 |
| foot_contact_force | -0.02 | -0.098 |
| joint_deviation_hip | -0.1 | -0.058 |
| joint_deviation_knee | -0.05 | -0.017 |
| joint_deviation_foot | -0.1 | -0.032 |
| flat_orientation_l2 | -0.5 | -0.009 |
| lin_vel_z_l2 | -2.0 | — |
| ang_vel_xy_l2 | -0.01 | — |
| joint_torques_l2 | -1e-5 | — |
| action_rate_l2 | -0.01 | — |
| stand_still | -0.2 | — |
| dof_pos_limits | -1.0 | — |
| undesired_contacts | -1.0 | — |

## Config
- **URDF**: Light (15.6kg, no battery_chest)
- **Actuators**: DelayedPDActuator (0-6 step / 0-12ms delay)
  - Kp: hip_roll/yaw 180, hip_pitch 180, knee 180, foot_pitch/roll 30
  - Kd: hip_roll/yaw 6.5/3.0, hip_pitch 6.5, knee 3.0, foot 1.0
  - Friction: hip_roll/yaw 0.375, hip_pitch/knee 0.5, foot 0.25
- **PPO**: [512, 256, 128], ELU, tanh output, 16384 envs
- **Obs dim**: 48 (includes base_lin_vel), act_dim=12
- **Command ranges**: lin_vel_x=(-0.5, 0.8), lin_vel_y=(-0.5, 0.5)
- **Action scale**: 0.5 (hip/knee), 0.25 (foot)
- **Decimation**: 4 (50 Hz)

## Training Path
- V116 trained from scratch → iter 28000
- Resumed as V122 with: hip_dev -0.1, knee_dev -0.05, foot_dev -0.1, foot_contact_force -0.02 (4x), feet_air_time threshold_max 0.40

## Files
- `model_28000.pt` — PyTorch checkpoint (rsl_rl format)
- `mujoco_v122_28000_v03.mp4` — MuJoCo render at 0.3 m/s forward

## Play
```bash
# Isaac Sim
docker run --gpus all -v /home/ubuntu/workspace:/workspace -v /home/ubuntu/results:/results -v /home/ubuntu/uploads:/uploads isaaclab:latest \
  /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/winners/v122_28000/model_28000.pt \
  --num_envs 8 --video --video_length 300 --headless --tanh

# MuJoCo (needs ONNX export first)
python3 /home/ubuntu/workspace/biped_locomotion/play_mujoco.py \
  --checkpoint /tmp/v122_28000_tanh.onnx --urdf light --headless \
  --video /tmp/v122_28000_play.mp4 --duration 10 --cmd_vx 0.3
```
