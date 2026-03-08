# Biped Locomotion — RL Training (IsaacLab + rsl_rl)

Bipedal robot walking via PPO in Isaac Sim / IsaacLab. Berkeley Humanoid-inspired config, G1-style teacher-student distillation.

## Robot

- URDF: `/uploads/robot.urdf` (12 DoF, ~27.5 kg with 10kg battery_chest)
- Forward axis: **-Y** (not +X). Asymmetric hip roll/pitch limits.
- Parallel linkage ankle (G1-style): each PR joint = 2 × motor torque

## Files

| File | Purpose |
|------|---------|
| `biped_env_cfg.py` | Flat env config (V57) |
| `biped_rough_env_cfg.py` | Rough env (inherits flat + terrain + height scanner) |
| `biped_student_env_cfg.py` | Student env — removes `base_lin_vel` from policy obs |
| `biped_train_rsl.py` | Phase 1: PPO teacher training |
| `biped_distill_rsl.py` | Phase 2: Teacher→student distillation (MSE) |
| `biped_finetune_student_rsl.py` | Phase 3: PPO fine-tune of distilled student |
| `biped_play_rsl.py` | Inference + video (`--rough`, `--student`) |
| `biped_play_record.py` | Torque + video recording (`--rough`, `--student`) |
| `biped_play_torques.py` | Torque-only recording (`--rough`, `--student`) |

## 3-Phase Training Pipeline

```
Phase 1: Teacher PPO        Phase 2: Distillation        Phase 3: Student PPO
┌──────────────────┐        ┌──────────────────┐         ┌──────────────────┐
│ Full obs (48/235)│───────▶│ Teacher: frozen   │────────▶│ Student obs only │
│ Standard PPO     │        │ Student: MSE loss │         │ Standard PPO     │
│ biped_train_rsl  │        │ biped_distill_rsl │         │ biped_finetune   │
└──────────────────┘        └──────────────────┘         └──────────────────┘
```

### Observation Dimensions

| | Flat | Rough |
|---|---|---|
| **Teacher** | 48d (with base_lin_vel) | 235d (+ height_scan 187d) |
| **Student** | 45d (no base_lin_vel) | 232d (height_scan kept, no base_lin_vel) |

Student keeps height_scan (real sensor on hardware). Only `base_lin_vel` is privileged.

### Commands

```bash
# Phase 1: Train teacher
python biped_train_rsl.py [--rough] --num_envs 4096 --max_iterations 15000

# Phase 2: Distill
python biped_distill_rsl.py [--rough] \
    --teacher_checkpoint /results/winners/v57_model_2899.pt \
    --num_envs 16384 --max_iterations 3000

# Phase 3: Fine-tune student
python biped_finetune_student_rsl.py [--rough] \
    --distilled /results/logs/rsl_rl/biped_distill_flat/model_3000.pt \
    --max_iterations 5000

# Play / record
python biped_play_rsl.py [--rough] [--student] \
    --checkpoint <path> --video --video_length 500
```

## Actuator Config

| Joint | Kp | Kd | Effort (Nm) | Armature |
|-------|----|----|-------------|----------|
| hip_roll/yaw | 20 | 3.0 | 50 | 0.0112 |
| hip_pitch | 30 | 3.0 | 100 | 0.0152 |
| knee | 30 | 3.0 | 100 | 0.024 |
| foot_pitch | 2.0 | 0.2 | 30 | 0.0112 |
| foot_roll | 2.0 | 0.2 | 30 | 0.001 |

Foot joints: 30Nm each via parallel linkage (2 motors × 15Nm). ImplicitActuator for training.

## Reward Terms (13)

| Term | Weight | | Term | Weight |
|------|--------|-|------|--------|
| track_lin_vel_xy_exp | 1.0 | | feet_slide | -0.25 |
| track_ang_vel_z_exp | 0.5 | | undesired_contacts | -1.0 |
| lin_vel_z_l2 | -2.0 | | joint_deviation_hip | -0.1 |
| ang_vel_xy_l2 | -0.05 | | joint_deviation_knee | -0.01 |
| joint_torques_l2 | -1e-5 | | flat_orientation_l2 | -0.5 |
| action_rate_l2 | -0.01 | | dof_pos_limits | -1.0 |
| feet_air_time | 2.0 | | | |

Rough terrain: `flat_orientation_l2=0`, `dof_pos_limits=0`.

## Command Ranges

```python
lin_vel_x = (-0.5, 0.5)    # lateral (small)
lin_vel_y = (-1.5, 0.5)    # forward=-Y (biased)
ang_vel_z = (-1.0, 1.0)
```

Curriculum expands `lin_vel_y` toward (-3.0, 1.5) based on tracking reward.

## Winners

| Checkpoint | Terrain | Reward | TrackVel | Notes |
|------------|---------|--------|----------|-------|
| `v57_model_2899.pt` | Flat | 19.1 | 0.84 | Teacher for distillation |
| `v57_rough_model_6498.pt` | Rough | ~16.5 | ~0.75 | Pre torque-limit fix |
| `model_19200.pt` (rough_v57) | Rough | **18.0** | **0.83** | Peak with 30Nm ankle |

## Docker

All on GCP L4 (`ubuntu@34.93.168.76`), image `isaaclab:latest`.

```bash
docker run --gpus all -d --name <name> \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh /workspace/biped_locomotion/<script> <args> --headless
```

Video requires Xvfb: `Xvfb :99 -screen 0 1920x1080x24 &` + `-e DISPLAY=:99 -v /tmp/.X11-unix:/tmp/.X11-unix`.

| Config | Envs | VRAM | Iter Time |
|--------|------|------|-----------|
| Flat | 16384 | 9.5 GB | ~2.5s |
| Rough | 8192 | 8.0 GB | ~5.2s |

## Key Lessons

1. **Forward is -Y**: Commands, curriculum, and play scripts must use lin_vel_y for forward
2. **ImplicitActuator >> DCMotor**: DCMotor saturation kills exploration
3. **10kg battery mass**: Prevents sliding exploits, improves feet_air_time 5×
4. **Impact-based feet_air_time**: Prevents sliding exploits vs continuous reward
5. **Joint mapping**: Isaac runtime order ≠ URDF order — always verify via `robot.joint_names`
6. **Parallel ankle**: Set PR effort = 2 × motor torque, don't model coupling in sim (G1 approach)
7. **Reward over-engineering kills performance**: Keep terms minimal, use terminations over penalties
8. **Knees dominate power** (56% of 229W avg) — upright stance reduces load but knee_deviation too weak at -0.01

## Directories

```
/home/ubuntu/workspace/biped_locomotion/  — source
/home/ubuntu/results/logs/rsl_rl/         — logs + checkpoints
/home/ubuntu/results/winners/             — best checkpoints
/home/ubuntu/uploads/robot.urdf           — URDF
```
