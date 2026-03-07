# Biped Locomotion — RL Training (IsaacLab + rsl_rl)

Bipedal robot walking via PPO in Isaac Sim / IsaacLab, using the rsl_rl framework with a Berkeley Humanoid-inspired configuration.

---

## Robot

- **URDF**: `/uploads/robot.urdf` (with 10kg `battery_chest` mass on torso)
- **STL meshes**: `/uploads/assets/`
- **Forward axis**: **-Y** (not +X) — commands use `lin_vel_y` negative
- **Asymmetric hip limits**: Roll/pitch mirrored between L/R legs
- **Total mass**: ~27.5kg (17.5kg robot + 10kg battery)
- **12 DOF**: 6 per leg (hip_yaw, hip_roll, hip_pitch, knee, foot_pitch, foot_roll)

---

## File Structure & Inheritance

```
biped_env_cfg.py                    ← Flat terrain config (base)
    ├── BipedFlatEnvCfg             ← Main training config
    ├── BipedFlatEnvCfg_PLAY        ← Inference config (100 envs, no pushes)
    ├── Custom reward functions:
    │   ├── feet_air_time()         ← Continuous single-stance (height-based)
    │   ├── feet_air_time_impact()  ← Impact reward at landing (height-based)
    │   ├── feet_slide()            ← Sliding penalty (height-based)
    │   ├── randomize_joint_default_pos()
    │   ├── modify_push_force()     ← Push curriculum (Berkeley exact)
    │   └── modify_command_velocity() ← Velocity curriculum (Berkeley exact)
    └── BIPED_CFG                   ← ArticulationCfg (actuators, URDF, init state)

biped_rough_env_cfg.py              ← Rough terrain config (inherits flat)
    ├── BipedRoughEnvCfg(BipedFlatEnvCfg)
    │   Adds: terrain generator, height scanner, terrain curriculum
    │   Relaxes: flat_orientation (0.0), dof_pos_limits (0.0)
    ├── BipedRoughEnvCfg_PLAY       ← Rough inference (50 envs, 5×5 terrain)
    ├── BIPED_ROUGH_TERRAINS_CFG    ← Terrain generator config
    └── terrain_levels_vel()        ← Terrain curriculum function

biped_train_rsl.py                  ← Training script
    ├── --rough flag selects rough config
    ├── Registers Biped-Flat-v0 / Biped-Rough-v0
    └── rsl_rl OnPolicyRunner with dict-based config

biped_play_rsl.py                   ← Inference + video recording
    ├── --rough flag selects rough config
    ├── Registers Biped-Flat-Play-v0 / Biped-Rough-Play-v0
    └── Manual actor MLP extraction from rsl_rl checkpoint
```

**Key inheritance rule**: `BipedRoughEnvCfg` inherits everything from `BipedFlatEnvCfg` — all rewards, events, commands, actuators, terminations. It only adds terrain, height scanner, terrain curriculum, and relaxes two penalty weights. Custom reward functions (e.g., `feet_air_time_impact`) are defined in `biped_env_cfg.py` and referenced by string path from both configs.

**Observation dimensions**:
- Flat: **48** (3 lin_vel + 3 ang_vel + 3 gravity + 3 commands + 6 hip_pos + 2 knee_pos + 2 foot_pitch + 2 foot_roll + 12 joint_vel + 12 actions)
- Rough: **208** (48 base + 160 height scan rays)

⚠️ Flat checkpoints **cannot** resume on rough (obs dim mismatch). Train rough from scratch.

---

## Config Details

### Actuators (ImplicitActuator, 2× Berkeley Humanoid original)

| Joint | Kp | Kd | Effort (Nm) | Armature |
|-------|-----|------|-------------|----------|
| hip_roll | 20.0 | 3.0 | 78 | 0.0112 |
| hip_yaw | 20.0 | 3.0 | 78 | 0.0112 |
| hip_pitch | 30.0 | 3.0 | 117 | 0.0152 |
| knee | 30.0 | 3.0 | 117 | 0.024 |
| foot_pitch | 2.0 | 0.2 | 20 | 0.0112 |
| foot_roll | 2.0 | 0.2 | 26 | 0.001 |

Why 2×: Our robot is heavier than Berkeley's (~27.5kg vs ~12kg). ImplicitActuator (ideal PD) is used for training; DCMotor saturates and kills exploration.

### Rewards (13 terms, Berkeley flat exact)

| Term | Function | Weight | Description |
|------|----------|--------|-------------|
| `track_lin_vel_xy_exp` | `mdp.track_lin_vel_xy_exp` | +1.0 | XY velocity tracking (Gaussian, σ²=0.25) |
| `track_ang_vel_z_exp` | `mdp.track_ang_vel_z_exp` | +0.5 | Yaw rate tracking (Gaussian, σ²=0.25) |
| `lin_vel_z_l2` | `mdp.lin_vel_z_l2` | -2.0 | Penalize vertical bouncing |
| `ang_vel_xy_l2` | `mdp.ang_vel_xy_l2` | -0.05 | Penalize roll/pitch angular velocity |
| `joint_torques_l2` | `mdp.joint_torques_l2` | -1e-5 | Energy efficiency |
| `action_rate_l2` | `mdp.action_rate_l2` | -0.01 | Smooth actions |
| `feet_air_time` | `feet_air_time_impact` | +2.0 | Reward at foot landing proportional to air time (0.05-0.5s window, height-based at 0.063m) |
| `feet_slide` | `feet_slide` | -0.25 | Penalize foot sliding when in contact (height < 0.063m) |
| `undesired_contacts` | `mdp.undesired_contacts` | -1.0 | Penalize hip/thigh ground contact (>1N threshold) |
| `joint_deviation_hip` | `mdp.joint_deviation_l1` | -0.1 | Keep hip_roll + hip_yaw near default |
| `joint_deviation_knee` | `mdp.joint_deviation_l1` | -0.01 | Keep knees near default |
| `flat_orientation_l2` | `mdp.flat_orientation_l2` | -0.5 | Keep torso upright (flat: -0.5, rough: **0.0**) |
| `dof_pos_limits` | `mdp.joint_pos_limits` | -1.0 | Joint limit penalty (flat: -1.0, rough: **0.0**) |

#### Height-based contact detection (V57 innovation)

All foot contact detection uses **height-based** instead of force-based:
- `body_pos_w[:, foot_ids, 2] > 0.063m` → foot is airborne
- Foot center rests at ~0.053m when standing; threshold 0.063m = 1cm clearance
- **Why**: Self-collision forces (100-2400N) are indistinguishable from ground contact forces, making force-based detection impossible with `enabled_self_collisions=True`

#### Reward functions explained

- **`feet_air_time` (continuous)**: Fixed 0.1 reward per step when exactly 1 foot is airborne. Bootstraps stepping behavior from scratch. Not used in current active config (kept as reference).
- **`feet_air_time_impact`**: Rewards proportional to air time duration when a foot lands. Only fires at the landing moment. Air time clamped to [0.05s, 0.5s] window. Shapes gait timing and quality.
- **`feet_slide`**: Penalizes foot XY velocity when foot is on the ground (height < threshold). Uses height-based detection, immune to self-collision noise.

### Physics

| Parameter | Value |
|-----------|-------|
| Self-collisions | **ON** (`enabled_self_collisions=True`) |
| Contact processing | **ON** (`disable_contact_processing=False`) |
| Sim dt | 0.005s (200 Hz) |
| Control dt | 0.02s (50 Hz, decimation=4) |
| Episode length | 20s (1000 steps) |
| Action scale | 0.25 |

### Observations (48-dim, Berkeley exact)

| Observation | Dim | Noise |
|-------------|-----|-------|
| Base linear velocity | 3 | ±0.1 |
| Base angular velocity | 3 | ±0.2 |
| Projected gravity | 3 | ±0.05 |
| Velocity commands | 3 | — |
| Hip joint positions (roll, yaw, pitch) | 6 | ±0.03 |
| Knee joint positions | 2 | ±0.05 |
| Foot pitch positions | 2 | ±0.08 |
| Foot roll positions | 2 | ±0.03 |
| Joint velocities (all 12) | 12 | ±1.5 |
| Last actions | 12 | — |

Critic uses same structure but with corruption disabled.

### Commands (Berkeley exact)

- Uniform velocity: lin_vel_x/y ∈ [-1.0, 1.0], ang_vel_z ∈ [-1.0, 1.0]
- Heading command enabled (stiffness 0.5)
- 2% standing envs (zero command)
- Resample every 10s

### Events (domain randomization)

**Startup** (once per environment):
- Rigid body material randomization (friction 0.2-1.25, restitution 0.0-0.1)
- Mass scaling (0.9-1.1× all links)
- Base mass addition (-1.0 to +1.0 kg on torso)
- Joint default position perturbation (±0.05 rad)
- Actuator gain scaling (0.8-1.2× stiffness/damping)

**Reset** (each episode):
- Random root pose (±0.5m XY, full yaw range)
- Random root velocity (±0.5 m/s linear, ±0.5 rad/s angular)
- Joint position scaling (0.5-1.5× default)

**Interval**:
- Push robot: ±0.5 m/s velocity impulse every 10-15s (curriculum scales up to ±3.0 m/s)

### Curriculum

1. **Push force** (`modify_push_force`): After iter 1500, adaptively increases/decreases push velocity based on fall rate. Max ±3.0 m/s.
2. **Command velocity** (`modify_command_velocity`): After iter 5000, expands lin_vel_x range when tracking reward > 80% of max. Up to [-1.5, 3.0] m/s.
3. **Terrain levels** (rough only, `terrain_levels_vel`): Promotes robots to harder terrain when they walk > half terrain size. Demotes when walking < half commanded distance.

### Terminations

- **Base contact**: Torso touches ground (>1N force threshold) → episode ends
- **Time out**: Episode reaches 20s (1000 steps) → episode ends (not counted as failure)

### PPO Config (rsl_rl)

| Parameter | Value |
|-----------|-------|
| Actor MLP | [128, 128, 128], ELU |
| Critic MLP | [128, 128, 128], ELU |
| Learning rate | 1e-3 (adaptive) |
| Epochs | 5 |
| Mini-batches | 4 |
| Discount (γ) | 0.99 |
| GAE (λ) | 0.95 |
| Entropy coeff | 0.005 |
| Clip range | 0.2 |
| Init noise std | 1.0 |
| Max grad norm | 1.0 |
| Desired KL | 0.01 |
| Save interval | 200 iters |

---

## Rough Terrain Config (additions over flat)

### Terrain Generator

7 sub-terrains across 10 difficulty rows × 20 columns:

| Terrain | Proportion | Parameters |
|---------|-----------|------------|
| Flat | 30% | Plain mesh |
| Pyramid slopes | 10% | 0.0-0.4 slope, 2m platform |
| Inverted pyramid slopes | 10% | 0.0-0.4 slope, 2m platform |
| Pyramid stairs | 5% | 0.0-0.1m step height, 0.3m width |
| Inverted pyramid stairs | 5% | 0.0-0.1m step height, 0.3m width |
| Wave terrain | 20% | 0.0-0.2m amplitude, 4 waves |
| Random rough | 20% | 0.0-0.06m noise, 0.02m step |

Grid: 8m × 8m per sub-terrain, 20m border.

### Height Scanner (RayCaster)

- Mounted on torso (`assy_formfg___kd_b_102b_torso_btm`)
- 1.6m × 1.0m grid at 0.1m resolution → **160 rays**
- Offset: 20m above (rays cast downward)
- Yaw-aligned (no roll/pitch)
- Noise: ±0.1, clipped to [-1.0, 1.0]
- Update period: matches control frequency (0.02s)

### Reward Changes (vs flat)

| Term | Flat | Rough |
|------|------|-------|
| `flat_orientation_l2` | -0.5 | **0.0** (disabled) |
| `dof_pos_limits` | -1.0 | **0.0** (disabled) |

All other rewards inherited unchanged.

---

## Training Results

### V55b — Best without self-collisions
- **Reward**: 21.5 | **track_lin**: 0.899 (90%) | **track_ang**: 0.382
- **ep_len**: 1000 (max) | **base_contact**: 0.6%
- Self-collisions OFF, contact processing OFF
- Impact-based feet_air (force-based detection)
- Training path: V54 (continuous, threshold 0.05) → V55 (impact) → V55b (8192 envs, 6K iters)
- Checkpoint: `/results/winners/v55b/model_5997.pt`

### V57 — Best with self-collisions (flat)
- **Reward**: 19.1 | **track_lin**: 0.843 (84%) | **track_ang**: 0.339
- **ep_len**: 955 | **base_contact**: 7.1%
- Self-collisions ON, contact processing ON, push curriculum active
- Impact-only feet_air (height-based detection at 0.063m)
- Training path: From scratch (continuous height-based, 200 iters) → resume with impact (to iter 2400) → resume impact-only (500 more iters to 2899)
- Checkpoint: `/results/winners/v57_model_2899.pt`
- Key innovation: Height-based contact detection solved self-collision force contamination

### Version History

| Version | Key Change | Result |
|---------|-----------|--------|
| V47 | Berkeley exact config, from scratch | Stands still (feet_air ≈ 0) |
| V48 | noise_std 1.5, entropy 0.02 | Minimal improvement |
| V49 | Wider command range, 6K iters | track_vel 1.19 but sliding exploit |
| V50 | 2× Berkeley actuators, feet_air w=2.0 | Still sliding |
| V51 | 10kg battery_chest added to URDF | feet_air 5× improvement |
| V52 | Berkeley exact rewards (13 terms) | Sliding trap |
| V53 | Continuous reward, weaker penalties | Still sliding |
| V54 | threshold_min 0.2→0.05, continuous | **First foot lifting** ✅ |
| V55 | Impact-based, resumed from V54 | reward 15.55, track_lin 0.725 |
| V55b | 8192 envs, continued V55 | **reward 21.5, track_lin 0.899** |
| V56 | Self-collisions ON, resumed V55b | Immediate collapse |
| V57 | Height-based detection, from scratch | **reward 19.1, track_lin 0.843** (with self-collisions) |

---

## Training

### Docker launch (GCP: ubuntu@34.93.168.76)

```bash
# --- FLAT TERRAIN ---

# Train from scratch
docker run --gpus all -d --name biped_train \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --num_envs 16384 --max_iterations 6000 --headless

# Resume from checkpoint
docker run --gpus all -d --name biped_train \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --num_envs 16384 --max_iterations 6000 \
  --resume /results/logs/rsl_rl/biped_flat_v52/model_2899.pt --headless

# Record video (flat)
docker run --gpus all -d --name biped_play \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/winners/v57_model_2899.pt \
  --num_envs 8 --video --video_length 300 --headless

# --- ROUGH TERRAIN ---

# Train from scratch
docker run --gpus all -d --name biped_train_rough \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --num_envs 8192 --max_iterations 6000 --rough --headless

# Resume rough from checkpoint
docker run --gpus all -d --name biped_train_rough \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --num_envs 8192 --max_iterations 6000 --rough \
  --resume /results/logs/rsl_rl/biped_rough_v57/model_499.pt --headless

# Record video (rough)
docker run --gpus all -d --name biped_play_rough \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/logs/rsl_rl/biped_rough_v57/model_499.pt \
  --num_envs 8 --video --video_length 300 --rough --headless
```

### GPU performance (NVIDIA L4, 23GB)

| Config | Envs | VRAM | GPU Util | Iter time | Speed |
|--------|------|------|----------|-----------|-------|
| Flat | 16384 | 9.5 GB (41%) | 89% | 4.2s | 0.24 it/s |
| Rough | 8192 | 8.0 GB (35%) | 71% | 5.2s | 0.19 it/s |
| Rough | 16384 | 12.6 GB (55%) | 90% | 9.8s | 0.10 it/s |
| Rough | 32768 | 21.9 GB (95%) | — | ❌ PhysX collision stack overflow |

### rsl_rl resume behavior
- `--resume model_200.pt --max_iterations 6000` → runs from iter 200 to **6200** (adds to loaded iter)

### Output directories
- Flat checkpoints: `/results/logs/rsl_rl/biped_flat_v52/`
- Rough checkpoints: `/results/logs/rsl_rl/biped_rough_v57/`
- Videos: `/results/videos/v52/rl-video-step-0.mp4`
- Winners: `/results/winners/`

---

## Directories

- `debug/` — Diagnostic scripts (contact analysis, reward debugging)
- `legacy/` — Old skrl-based training code (pre rsl_rl migration)

## Key Lessons

1. **threshold_min 0.2→0.05 was the stepping breakthrough** (V54)
2. **Continuous reward bootstraps stepping, impact refines gait** — use continuous first, then switch to impact
3. **10kg battery mass prevents sliding exploits** — top-heavy robot can't shuffle
4. **Height-based > force-based contact detection** — self-collision forces make force-based impossible
5. **ImplicitActuator >> DCMotor** for training — DCMotor saturation kills exploration
6. **8192 envs is the sweet spot for rough terrain on L4** — 16384 works but 2× slower per iter
7. **Flat checkpoints can't resume on rough** — obs dim changes (48 → 208)

See `CHECKPOINTS.md` for detailed version history.
