# Biped Locomotion — RL Training (IsaacLab + rsl_rl)

Bipedal robot walking via PPO in Isaac Sim / IsaacLab, using rsl_rl framework with Berkeley Humanoid-inspired configuration.

## Files

| File | Purpose |
|------|---------|
| `biped_env_cfg.py` | Flat terrain env config (V57, self-collisions ON) |
| `biped_rough_env_cfg.py` | Rough terrain env config (inherits flat, 7 sub-terrains, height scanner) |
| `biped_student_env_cfg.py` | Student distillation env config (removes base_lin_vel from policy obs) |
| `biped_train_rsl.py` | PPO training script (rsl_rl OnPolicyRunner) |
| `biped_distill_rsl.py` | Teacher-student distillation script (rsl_rl DistillationRunner) |
| `biped_play_rsl.py` | Inference + video recording (camera follows agent) |
| `record_grid.sh` | Record 4 agents → 2×2 grid video |

### Versioned Configs (archive)
- `biped_env_cfg_v54_continuous.py` — continuous feet_air reward, threshold_min=0.05
- `biped_env_cfg_v55_impact_based.py` — impact-based feet_air reward, threshold_min=0.05

## Robot

- URDF: `/uploads/robot.urdf` (with 10kg `battery_chest`)
- STL meshes: `/uploads/assets/`
- Forward axis: **-Y** (not +X)
- Asymmetric hip roll/pitch limits (mirrored L/R)
- Total mass: ~27.5 kg
- 12 actuated joints (6 per leg: hip_roll, hip_yaw, hip_pitch, knee, foot_pitch, foot_roll)

## Config Inheritance

```
biped_env_cfg.py (BipedFlatEnvCfg)
├── biped_rough_env_cfg.py (BipedRoughEnvCfg) — adds terrain, height scanner, curriculum
└── biped_student_env_cfg.py (BipedStudentFlatEnvCfg) — adds teacher obs group, removes base_lin_vel from policy
```

## Observations (48-dim teacher / 45-dim student)

| Term | Dims | Student | Teacher | Noise |
|------|------|---------|---------|-------|
| base_lin_vel | 3 | ❌ | ✅ | ±0.1 |
| base_ang_vel | 3 | ✅ | ✅ | ±0.2 |
| projected_gravity | 3 | ✅ | ✅ | ±0.05 |
| velocity_commands | 3 | ✅ | ✅ | — |
| hip_pos (6 joints) | 6 | ✅ | ✅ | ±0.03 |
| knee_pos (2 joints) | 2 | ✅ | ✅ | ±0.05 |
| foot_pitch_pos (2) | 2 | ✅ | ✅ | ±0.08 |
| foot_roll_pos (2) | 2 | ✅ | ✅ | ±0.03 |
| joint_vel (all 12) | 12 | ✅ | ✅ | ±1.5 |
| actions (last) | 12 | ✅ | ✅ | — |

Rough terrain adds: height_scan (160 rays) → obs_dim = 208.

## Reward Terms (13, Berkeley flat)

| Term | Weight | Type |
|------|--------|------|
| track_lin_vel_xy_exp | 1.0 | positive |
| track_ang_vel_z_exp | 0.5 | positive |
| lin_vel_z_l2 | -2.0 | penalty |
| ang_vel_xy_l2 | -0.05 | penalty |
| joint_torques_l2 | -1e-5 | penalty |
| action_rate_l2 | -0.01 | penalty |
| feet_air_time (impact-based) | 2.0 | positive |
| feet_slide | -0.25 | penalty |
| undesired_contacts | -1.0 | penalty |
| joint_deviation_hip | -0.1 | penalty |
| joint_deviation_knee | -0.01 | penalty |
| flat_orientation_l2 | -0.5 | penalty |
| dof_pos_limits | -1.0 | penalty |

## Docker Commands

All commands run inside `isaaclab:latest` container on GCP (`ubuntu@34.93.168.76`).

### Container Launch Template

```bash
docker run --gpus all -d --name <name> \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh /workspace/biped_locomotion/<script> <args> --headless
```

### Train — Flat Terrain

```bash
# From scratch (16384 envs, ~9.5 GB VRAM on L4)
docker run --gpus all -d --name biped_train_flat \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --num_envs 16384 --max_iterations 3000 --headless

# Resume from checkpoint
docker run --gpus all -d --name biped_train_flat \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --num_envs 16384 --max_iterations 3000 \
  --resume /results/logs/rsl_rl/biped_flat_v52/model_2899.pt --headless
```

### Train — Rough Terrain

```bash
# 8192 envs is sweet spot for rough on L4 (~8 GB VRAM, 5.23s/iter)
docker run --gpus all -d --name biped_train_rough \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --rough --num_envs 8192 --max_iterations 6000 --headless
```

### Distill — Teacher-Student (G1-style)

```bash
# Phase 2: Distill student from teacher (flat terrain, 16384 envs)
docker run --gpus all -d --name biped_distill \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_distill_rsl.py \
  --teacher_checkpoint /results/winners/v57_model_2899.pt \
  --num_envs 16384 --max_iterations 3000 --headless
```

### Record Video

**IMPORTANT**: Video recording on L4 requires Xvfb (virtual framebuffer). Start it on the host before launching Docker:

```bash
# Start Xvfb on host (run once)
pkill Xvfb; Xvfb :99 -screen 0 1920x1080x24 &>/dev/null &

# Single agent, camera following
docker run --gpus all -d --name biped_play \
  -e DISPLAY=:99 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/winners/v57_rough_model_6498.pt \
  --rough --video --video_length 500 --num_envs 8 \
  --env_index 0 --video_dir /results/videos \
  --headless
```

**Note**: First run takes ~6 min for shader compilation. Subsequent runs ~3 min.

### Record 2×2 Grid Video (4 agents)

```bash
# Start Xvfb first
pkill Xvfb; Xvfb :99 -screen 0 1920x1080x24 &>/dev/null &

# Record all 4 agents (runs sequentially, ~20 min total)
docker run --gpus all -d --name biped_grid \
  -e DISPLAY=:99 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest bash /workspace/biped_locomotion/record_grid.sh \
  /results/winners/v57_rough_model_6498.pt 500 --rough

# ffmpeg merge runs on host (not in Docker — ffmpeg not installed in container)
# record_grid.sh will fail at the merge step; run manually:
ffmpeg -y \
  -i /home/ubuntu/results/videos/grid/agent_0/rl-video-step-0.mp4 \
  -i /home/ubuntu/results/videos/grid/agent_1/rl-video-step-0.mp4 \
  -i /home/ubuntu/results/videos/grid/agent_2/rl-video-step-0.mp4 \
  -i /home/ubuntu/results/videos/grid/agent_3/rl-video-step-0.mp4 \
  -filter_complex \
    "[0:v]scale=640:360[v0];[1:v]scale=640:360[v1];[2:v]scale=640:360[v2];[3:v]scale=640:360[v3]; \
     [v0][v1]hstack=inputs=2[top];[v2][v3]hstack=inputs=2[bottom]; \
     [top][bottom]vstack=inputs=2[out]" \
  -map "[out]" -c:v libx264 -crf 23 -preset fast \
  /home/ubuntu/results/videos/grid/v57_rough_grid.mp4
```

## GPU Benchmarks (NVIDIA L4, 23 GB)

| Config | Envs | VRAM | GPU% | Iter Time | Notes |
|--------|------|------|------|-----------|-------|
| Flat | 16384 | 9.5 GB (41%) | 88-89% | ~2.5s | Standard |
| Rough | 8192 | 8.0 GB (35%) | 67% | 5.23s | Sweet spot for L4 |
| Rough | 16384 | 12.6 GB (55%) | — | 9.82s | Works but slower |

## Winners (Checkpoints)

| File | Config | Reward | track_lin | ep_len | Notes |
|------|--------|--------|-----------|--------|-------|
| `winners/v55b/model_5997.pt` | Flat, no self-collisions | 21.5 | 0.899 | 1000 | Best pre-self-collision |
| `winners/v57_model_2899.pt` | Flat, self-collisions ON | 19.1 | 0.843 | 955 | Teacher for distillation |
| `winners/v57_rough_model_6498.pt` | Rough, self-collisions ON | ~16.5 | ~0.75 | ~920 | Best rough terrain |

## Distillation Pipeline (G1-style)

Three-phase approach following NVIDIA's G1 implementation:

1. **Teacher** (done): Standard PPO with `base_lin_vel` in obs → `v57_model_2899.pt`
2. **Distill**: rsl_rl `DistillationRunner` — MSE loss between student and teacher actions. Student obs = 45-dim (no `base_lin_vel`), teacher obs = 48-dim.
3. **Fine-tune**: Standard PPO using student obs only (not yet implemented)

Uses rsl_rl built-in: `DistillationRunner`, `Distillation` algorithm, `StudentTeacher` module.

## Directories

```
/home/ubuntu/workspace/biped_locomotion/  — source code
/home/ubuntu/results/logs/rsl_rl/         — training logs + checkpoints
/home/ubuntu/results/winners/             — best checkpoints
/home/ubuntu/results/videos/              — recorded videos
/home/ubuntu/uploads/robot.urdf           — robot URDF
```

## Key Lessons

- **Xvfb required for video on L4**: `enable_cameras` hangs without virtual framebuffer. Always start `Xvfb :99` before recording.
- **8192 envs for rough terrain**: 16384 works but ~2× slower per iter with marginal throughput gain.
- **rsl_rl `--resume` adds to max_iterations**: `--max_iterations 6000 --resume model_200.pt` runs to iter 6200.
- **Impact-based feet_air_time**: Key breakthrough over continuous — prevents sliding exploits.
- **10kg battery mass**: Prevents sliding exploits by raising center of mass.
- **Height-based contact detection**: Replaces force-based (force-based breaks with self-collisions ON).
- **ImplicitActuator >> DCMotor for training**: DCMotor saturation kills exploration.

See `CHECKPOINTS.md` for full version history.
