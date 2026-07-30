# V188 Student Light - Iteration 13000 (Wide Stance, 0.70 Push Max)

- **Date:** 2026-07-30
- **Base Checkpoint:** `model_13000.pt`
- **Mean Reward:** 20.66
- **Tracking Velocity (XY):** 0.8056
- **Fall Rate:** 5.63%
- **Push Forces:** Maxed at 0.70 m/s pushes.
- **URDF:** Light
- **Network:** Distilled student policy (`[512, 256, 128]`, ELU, `tanh` bounded).
- **Curriculum modifications applied:**
  - `lin_vel_x`: `[-0.6, 0.8]`
  - `lin_vel_y`: `[-0.5, 0.5]`
  - `ang_vel_z`: `[-0.6, 0.6]`
  - `push_force_levels`: `[0.7, 0.7]` (scaled back from 0.8)

## Included Files
- `student_13000.pt`: Raw PyTorch checkpoint.
- `biped_env_cfg.py`: Base environment configuration used.
- `biped_student_env_cfg.py`: Student observation configuration.
- `biped_finetune_student_rsl.py`: PPO script for Phase 3.
- `mujoco_vx00_ema06.mp4`: Standing simulation at 0.0 vel (EMA 0.6).
- `mujoco_vx03_ema06.mp4`: Walking simulation at 0.3 vel (EMA 0.6).
