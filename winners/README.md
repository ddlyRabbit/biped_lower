# Winner Models

## V125 Teacher (Apr 22, 2026) — ACTIVE TRAINING

| File | Description |
|------|-------------|
| *(training in progress)* | Light URDF, tanh, no IMU delay, symmetry ON |

**Config:**
- **Physics**: 500 Hz (dt=0.002), **Policy**: 50 Hz (decimation=10)
- **URDF**: Light (15.6kg)
- **Actuator**: DelayedPDActuator, delay 0-6 steps, friction 0.25-0.5
  - Kp: hip_roll/yaw=180, hip_pitch=180, knee=180, foot_pitch/roll=30
  - Kd: hip_roll/pitch=6.5, hip_yaw/knee=3.0, foot=1.0
- **IMU delay**: None (removed)
- **IMU noise**: 2x (lin_vel ±0.2, ang_vel ±0.4, gravity ±0.1)
- **Symmetry**: Data augmentation enabled (adaptive 45/48 dim, correct URDF joint negation)
- **Action scale**: 0.5 (hip/knee), 0.25 (foot_roll), tanh output
- **Envs**: 4096, PPO [512, 256, 128] ELU
- **Obs dim**: 48 (includes base_lin_vel)

**Reward Terms (V125 — 16 terms):**

| # | Term | Function | Weight | Notes |
|---|------|----------|--------|-------|
| 1 | track_lin_vel_xy_exp | base_mdp | +1.0 | std=0.5 |
| 2 | track_ang_vel_z_exp | base_mdp | +0.5 | std=0.5 |
| 3 | lin_vel_z_l2 | base_mdp | -2.0 | |
| 4 | ang_vel_xy_l2 | base_mdp | -0.01 | |
| 5 | joint_torques_l2 | base_mdp | -1e-5 | |
| 6 | action_rate_l2 | base_mdp | **-0.3** | 30x from V122 |
| 7 | feet_air_time | adaptive_berkeley | +20.0 | immediate impact (switch_step=0), 0.15-0.35 |
| 8 | feet_slide | berkeley | -0.25 | contact-sensor based |
| 9 | foot_contact_force | custom l2 | -0.02 | threshold 200N (4x from V116) |
| 10 | undesired_contacts | base_mdp | -1.0 | torso/hip links, threshold 1.0 |
| 11 | joint_deviation_hip | base_mdp | -0.1 | hip_roll + hip_yaw + hip_pitch |
| 12 | joint_deviation_knee | base_mdp | -0.01 | knee only |
| 13 | joint_deviation_foot | base_mdp | -0.1 | foot_pitch + foot_roll |
| 14 | stand_still | custom | -0.2 | penalize foot shuffle at zero cmd |
| 15 | flat_orientation_l2 | base_mdp | -0.5 | |
| 16 | dof_pos_limits | base_mdp | -1.0 | |

**What changed from V122:**
- IMU observation delay removed (was 0-3 steps)
- IMU noise doubled
- action_rate penalty: -0.01 → -0.3
- Symmetry augmentation enabled (data aug only, no mirror loss)
- Adaptive feet_air_time: immediate impact (switch_step=0, was 500 iters)
- Resumed from V116 model_10400 checkpoint

---

## V122 Teacher @ 28000 (Apr 21, 2026)

| File | Description |
|------|-------------|
| `v122_28000/model_28000.pt` | 28K iters (resumed from V116) |
| `v122_28000/mujoco_v122_28000_v03.mp4` | MuJoCo render at 0.3 m/s |
| `v122_28000/README.md` | Full details |

**Metrics (iter 28000):**
- **Reward**: 35.44 | **Timeout**: 99.87% | **Falls**: 0.13%
- **Velocity**: lin 0.877 (88%) | ang 0.862 (86%)
- **feet_air_time**: +0.308 | **foot_contact_force**: -0.098

**Play:**
```bash
docker run --gpus all -v /home/ubuntu/workspace:/workspace -v /home/ubuntu/results:/results -v /home/ubuntu/uploads:/uploads isaaclab:latest \
  /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/winners/v122_28000/model_28000.pt \
  --num_envs 8 --video --video_length 300 --headless --tanh
```

---

---

## V76 Teacher — Soft Ankles (Apr 3, 2026)

| File | Description |
|------|-------------|
| `v76_teacher_soft_ankles_17800.pt` | 17.8K iters, soft ankle gains |

**Metrics:** reward 21.91, vel 0.87, falls 1.0%, feet_air +0.013

---

## V74 Teacher (Mar 24, 2026) — Push-Hardened

| File | Description |
|------|-------------|
| `v74_teacher_18400.pt` | Push curriculum at 3.0 m/s |
| `v74_teacher_16600.pt` | Pre-push peak (reward 21.7) |
| `v74_teacher_18400.mp4` | Video under push |
| `v74_student_1000.pt` | Best deployable student (45d obs) |
| `v74_student_1000.mp4` | Student video |

**V74 Student (deployable):** reward 19.2, vel 0.81, falls 3%, 45d obs → ONNX
**V74 16600:** reward 21.7, vel 0.92, falls 0.7% (pre-push peak)
**V74 18400:** reward 11.7, vel 0.80, falls 23% (push 3.0 m/s)

---

## V72 Teacher (Mar 23, 2026)

| File | Description |
|------|-------------|
| `v72_teacher_5999.pt` | 6K iters from scratch |
| `v72_teacher_8998.pt` | +3K continued |
| `v72_teacher_5999.mp4` | Video |

**Metrics:** reward 20.5, vel 0.89, falls 2.4%
**Note:** Unbounded actions (no tanh). Superseded by V74.

---

## Config Reference

### Actuator Gains (across versions)

| Version | hip_roll Kp | hip_pitch Kp | knee Kp | foot Kp | Delay | Tanh |
|---------|-------------|--------------|---------|---------|-------|------|
| V72 | 10 | 15 | 15 | 8 | 0-5ms | No |
| V74 | 120 | 180 | 180 | 96/48 | 0-5ms | Yes |
| V76 | 180 | 180 | 180 | 30 | 0-12ms | Yes |
| V122 | 180 | 180 | 180 | 30 | 0-12ms | Yes |
| V125 | 180 | 180 | 180 | 30 | 0-12ms | Yes |

### Obs Dimension

| Version | Obs | Notes |
|---------|-----|-------|
| V72-V76 | 48 | Includes base_lin_vel |
| V74 student | 45 | No base_lin_vel (deployable) |
| V122-V125 | 48 | Includes base_lin_vel |

### URDF

| Version | Mass | Battery | Forward Axis |
|---------|------|---------|-------------|
| V72-V74 | 30.7kg (heavy) | 0.5kg | +X |
| V76-V125 | 15.6kg (light) | none | +X |
