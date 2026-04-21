# Winner Models

## V124 Teacher (Apr 21, 2026) — ACTIVE TRAINING

| File | Description |
|------|-------------|
| *(training in progress)* | Light URDF, tanh, IMU delay, adaptive airtime |

**Config:**
- **Physics**: 500 Hz (dt=0.002), **Policy**: 50 Hz (decimation=10)
- **URDF**: Light (15.6kg)
- **Actuator**: DelayedPDActuator, delay 0-6 steps, friction 0.25-0.5
  - Kp: hip_roll/yaw=180, hip_pitch=180, knee=180, foot_pitch/roll=30
  - Kd: hip_roll/pitch=6.5, hip_yaw/knee=3.0, foot=1.0
- **IMU delay**: 0-3 steps (0-6ms) on base_lin_vel, base_ang_vel, projected_gravity
- **Action scale**: 0.5 (hip/knee), 0.25 (foot_roll), tanh output
- **Envs**: 16384, PPO [512, 256, 128] ELU
- **Obs dim**: 48 (includes base_lin_vel)

**Reward Terms (V124 — 16 terms):**

| # | Term | Function | Weight | Notes |
|---|------|----------|--------|-------|
| 1 | track_lin_vel_xy_exp | base_mdp | +1.0 | std=0.5 |
| 2 | track_ang_vel_z_exp | base_mdp | +0.5 | std=0.5 |
| 3 | lin_vel_z_l2 | base_mdp | -2.0 | |
| 4 | ang_vel_xy_l2 | base_mdp | -0.01 | |
| 5 | joint_torques_l2 | base_mdp | -1e-5 | |
| 6 | action_rate_l2 | base_mdp | -0.01 | |
| 7 | feet_air_time | adaptive_berkeley | +20.0 | 500 iters positive_biped → impact, 0.15-0.35 |
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
- IMU observation delay (0-3 steps) — sim-to-real robustness
- Adaptive feet_air_time: positive_biped (500 iters) → berkeley impact
- feet_air_time weight 10→20, thresholds 0.15-0.35
- joint_deviation_hip: added hip_pitch, weight -0.5→-0.1
- joint_deviation_foot: added foot_pitch, renamed, weight -0.3→-0.1
- Training from scratch (not resume)

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

## V116 Teacher (Apr 16-20, 2026)

| File | Description |
|------|-------------|
| `v116_44000/model_44000.pt` | 44K iters, light URDF, tanh |
| `v116_44000/v116_isaac_41600.mp4` | Isaac Sim render |
| `v116_44000/v116_mujoco_42200.mp4` | MuJoCo render |
| `v116_44000/README.md` | Full details |
| `v116_10200/model_10200.pt` | 10.2K iter checkpoint (for resume experiments) |

**Metrics (iter 44000):**
- **Reward**: 22.5 | **Timeout**: 100% | **Falls**: ~0%
- **Velocity**: 0.89 | **sliding gait** (air_time negative)
- Berkeley impact air_time, threshold 0.25-0.30, weight 10
- PPO [512,256,128] ELU + tanh, 16384 envs

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
| V116 | 180 | 180 | 180 | 30 | 0-12ms | Yes |
| V122 | 180 | 180 | 180 | 30 | 0-12ms | Yes |
| V124 | 180 | 180 | 180 | 30 | 0-12ms | Yes |

### Obs Dimension

| Version | Obs | Notes |
|---------|-----|-------|
| V72-V76 | 48 | Includes base_lin_vel |
| V74 student | 45 | No base_lin_vel (deployable) |
| V116-V124 | 48 | Includes base_lin_vel |

### URDF

| Version | Mass | Battery | Forward Axis |
|---------|------|---------|-------------|
| V72-V74 | 30.7kg (heavy) | 0.5kg | +X |
| V76-V124 | 15.6kg (light) | none | +X |
