# Winner Models

## v55b/model_5997.pt
- **Version**: V55b (best without self-collisions)
- **Reward**: 21.5 | track_lin: 0.899 (90%) | track_ang: 0.382 | ep_len: 1000 (max)
- **Base contact**: 0.6% (almost never falls)
- **Self-collisions**: OFF | Contact processing: OFF
- **Feet air**: 0.021 (impact-based, Berkeley original sensor detection)
- **Training**: From scratch → V54 (continuous) → V55 (impact) → V55b (8192 envs, 6000 iters)
- **Actuators**: ImplicitActuator, 2× Berkeley Humanoid (original) values
- **URDF**: robot.urdf with 10kg battery_chest
- **Framework**: rsl_rl, PPO [128,128,128], ELU
- **Play**:
  ```
  docker run --gpus all -v /home/ubuntu/workspace:/workspace -v /home/ubuntu/results:/results -v /home/ubuntu/uploads:/uploads isaaclab:latest \
    /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
    --checkpoint /results/winners/v55b/model_5997.pt \
    --num_envs 8 --video --video_length 300 --headless
  ```

## v57_model_2899.pt
- **Version**: V57 (best with self-collisions)
- **Reward**: 19.1 | track_lin: 0.843 (84%) | track_ang: 0.339 | ep_len: 955
- **Base contact**: 7.1% falls
- **Self-collisions**: ON | Contact processing: ON
- **Feet air**: Impact-only (height-based detection, threshold 0.063m)
- **Feet slide**: Height-based (threshold 0.063m)
- **Push curriculum**: Active (started iter 1500, adaptive ±0.5 → ±3.0 m/s)
- **Training**: From scratch (continuous feet_air, height-based) → iter 200 resume (+ impact) → iter 2400 resume (impact-only, 500 iters)
- **Actuators**: ImplicitActuator, 2× Berkeley Humanoid (original) values
- **URDF**: robot.urdf with 10kg battery_chest
- **Framework**: rsl_rl, PPO [128,128,128], ELU
- **Key innovations**:
  - Height-based contact detection (foot Z > 0.063m = airborne) instead of force-based
  - Solves self-collision force contamination that broke all prior attempts
  - Continuous reward bootstrapped stepping, impact reward refined gait
- **Play**:
  ```
  docker run --gpus all -v /home/ubuntu/workspace:/workspace -v /home/ubuntu/results:/results -v /home/ubuntu/uploads:/uploads isaaclab:latest \
    /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
    --checkpoint /results/winners/v57_model_2899.pt \
    --num_envs 8 --video --video_length 300 --headless
  ```

## Notes
- All models use `biped_env_cfg.py` (flat terrain config)
- Obs dim: 48 (no height scanner)
- Actor: [128, 128, 128] ELU, 48 → 12 actions
- URDF: `/home/ubuntu/uploads/robot.urdf`
- Videos output to `/home/ubuntu/results/videos/`

## v57_rough_model_19200.pt
- **Version**: V57 rough (best rough terrain teacher)
- **Reward**: 18.0 | TrackVel: 0.83 | crash rate: 13.7% | ep_len: ~915
- **Self-collisions**: ON
- **Obs dim**: 235 (48 proprio + 187 height_scan)
- **Terrain**: 7 sub-terrains (slopes, stairs, rough, stepping stones)
- **Actuators**: ImplicitActuator, hip_roll/yaw 50Nm, hip_pitch/knee 100Nm, foot_pitch/roll 30Nm
- **Training**: Resumed from flat v57_model_2899 → 20,599 rough iters, peak at 19,200
- **Command ranges**: lin_vel_y=(-1.5, 0.5) forward=-Y, lin_vel_x=(-0.5, 0.5) lateral
- **Curriculum**: Push force + command velocity expansion from iter 1000
- **URDF**: robot.urdf with 10kg battery_chest, ankle 30Nm (parallel linkage)
- **Framework**: rsl_rl, PPO [128,128,128], ELU, 8192 envs
- **Play**:
  

## v57_student_flat_model_4200.pt
- **Version**: V57 student (3-phase distilled + fine-tuned)
- **Phase**: Teacher→Distill→PPO fine-tune (G1-style pipeline)
- **Reward**: 19.1 (100% of teacher) | peak 19.11 at iter 4200
- **Self-collisions**: ON
- **Obs dim**: 45 (no base_lin_vel — deployable on real hardware)
- **Teacher**: v57_model_2899.pt (reward 19.1, 48d obs)
- **Distillation**: 2400 iters, behavior loss 0.20, reward 12.2 (64% teacher)
- **Fine-tune**: 5000 iters, conservative PPO (LR 3e-4, clip 0.1, entropy 0.001)
- **Actuators**: ImplicitActuator, hip_roll/yaw 50Nm, hip_pitch/knee 100Nm, foot_pitch/roll 30Nm
- **URDF**: robot.urdf with 10kg battery_chest
- **Framework**: rsl_rl, PPO [128,128,128], ELU
- **Key details**:
  - Student removes only base_lin_vel (3d) from obs — not available on real IMU
  - Phase 3 conservative PPO critical: high LR (1e-3) destroys pre-trained actor due to random critic
  - Student surpassed teacher (19.11 > 19.1) — extra ankle torque (30Nm vs teacher's 15Nm)
- **Play**:
  

## V72 Teacher (Mar 23, 2026)
- **v72_teacher_5999.pt**: 6000 iters from scratch, pure Berkeley contact detection
- **v72_teacher_8998.pt**: continued 3000 more with threshold_min=0.15s
- Config: V58 gains (hip 10/15, knee 15, foot 8) + DelayedPD (0-5ms) + friction
- Self-collisions OFF, symmetry loss ON, Berkeley impact air_time
- Defaults: hip ±0.08, knee 0.25, foot -0.17
- Reward 20.5, vel 0.89, falls 2.4%
- obs_dim=48 (includes base_lin_vel), action_dim=12
- **Note**: Unbounded actions (no tanh), policy saturates actuators. Needs student distillation or tanh retrain for deploy.
