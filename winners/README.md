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
- Videos output to `/home/ubuntu/results/videos/v52/`
