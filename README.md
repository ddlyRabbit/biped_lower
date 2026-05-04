# Biped Locomotion — RL Training (IsaacLab + rsl_rl)

Bipedal walking via PPO + teacher-student distillation. Isaac Sim / IsaacLab, rsl_rl framework, Berkeley Humanoid-inspired config.

## File Map

```
biped_locomotion/
│
├── Config ─────────────────────────────────────────────────────────
│   ├── biped_env_cfg.py              ← Flat env (V58). Scene, rewards, obs, actuators
│   ├── biped_rough_env_cfg.py        ← Rough env (inherits flat + terrain + height scanner)
│   └── biped_student_env_cfg.py      ← Student env (removes base_lin_vel from policy obs)
│       ├── FlatStudentObservationsCfg    → policy(45d) + teacher(48d) + critic(48d)
│       └── RoughStudentObservationsCfg   → policy(232d) + teacher(235d) + critic(235d)
│
├── Training ───────────────────────────────────────────────────────
│   ├── biped_train_rsl.py            ← Phase 1: PPO teacher (flat/rough)
│   ├── biped_distill_rsl.py          ← Phase 2: Teacher→student MSE distillation
│   └── biped_finetune_student_rsl.py ← Phase 3: PPO fine-tune of distilled student
│
├── Inference / Recording ──────────────────────────────────────────
│   ├── biped_play_rsl.py             ← Play + video (--rough, --student)
│   ├── biped_play_record.py          ← Torque + video recording (--student)
│   ├── biped_play_torques.py         ← Torque-only recording (--student)
│   ├── record_grid.sh                ← 2×2 multi-agent grid video
│   └── combine_torque_video.py       ← Merge torque plot + video
│
├── legacy/ ────────────────────────────────────────────────────────
│   ├── biped_env_cfg_v54_continuous.py
│   └── biped_env_cfg_v55_impact_based.py
│
└── __init__.py

urdf/
├── heavy/                            ← Original material (~30.7kg with 10kg battery)
│   ├── robot.urdf
│   └── assets/                       ← 35 STL meshes
└── light/                            ← Lighter material (~15.6kg with 0.5kg battery)
    ├── robot.urdf
    └── assets/                       ← 35 STL meshes

winners/
└── README.md                         ← Checkpoint descriptions + naming convention

deploy/
├── ARCHITECTURE.md                   ← Full deployment plan, obs mapping, risks
├── BRINGUP.md                        ← Step-by-step bringup guide
├── scripts/setup_can.sh              ← CAN interface setup (run after boot)
├── student_flat.onnx                 ← Exported student policy
└── biped_ws/src/                     ← ROS2 Jazzy workspace (6 packages)
    ├── biped_msgs/                   ← Custom messages (MITCommand, MotorState, RobotState)
    ├── biped_driver/                 ← IMU node, CAN bus node, RobStride protocol
    ├── biped_control/                ← Policy node, safety node, state machine
    ├── biped_teleop/                 ← Keyboard velocity commander
    ├── biped_tools/                  ← Calibration tool, ONNX export
    └── biped_bringup/                ← Launch files + config

/results/ (GCP only)
├── logs/rsl_rl/                      ← Training logs + checkpoints
│   ├── biped_flat_v52/               ← Phase 1 flat teacher
│   ├── biped_rough_v57/              ← Phase 1 rough teacher
│   ├── biped_distill_flat/           ← Phase 2 flat distillation
│   └── biped_student_flat/           ← Phase 3 flat fine-tune
├── winners/                          ← Best checkpoints
└── videos/                           ← Recorded videos
```

### Config Inheritance

```
biped_env_cfg.py (BipedFlatEnvCfg)
├── biped_rough_env_cfg.py (BipedRoughEnvCfg)
│   └── biped_student_env_cfg.py (BipedStudentRoughEnvCfg)
└── biped_student_env_cfg.py (BipedStudentFlatEnvCfg)
```

### Script Dependencies

```
biped_train_rsl.py ──imports──► biped_env_cfg.py, biped_rough_env_cfg.py
biped_distill_rsl.py ──imports──► biped_student_env_cfg.py ──imports──► biped_env_cfg.py, biped_rough_env_cfg.py
biped_finetune_student_rsl.py ──imports──► biped_student_env_cfg.py
biped_play_rsl.py ──imports──► all env configs (based on --rough/--student flags)
```

## Robot

- URDFs: `urdf/heavy/robot.urdf` (~30.7 kg) and `urdf/light/robot.urdf` (~15.6 kg)
- Select variant with `--urdf heavy|light` in all training and inference scripts (default: heavy)
- Forward axis: **+X**. Asymmetric hip roll/pitch limits.
- Parallel linkage ankle (G1-style): each PR joint = 2 × motor torque

## 3-Phase Training Pipeline

```
Phase 1: Teacher PPO        Phase 2: Distillation        Phase 3: Student PPO
┌──────────────────┐        ┌──────────────────┐         ┌──────────────────┐
│ Full obs (48/235)│───────▶│ Teacher: frozen   │────────▶│ Student obs only │
│ Standard PPO     │        │ Student: MSE loss │         │ Conservative PPO │
│ biped_train_rsl  │        │ biped_distill_rsl │         │ biped_finetune   │
└──────────────────┘        └──────────────────┘         └──────────────────┘
    LR=1e-3                   ~3000 iters                   LR=3e-4
    clip=0.2                  Loss: 6.8→0.2                 clip=0.1
    ~3000 iters               Reward: ~64% teacher          entropy=0.001
    Reward: ~19               Student: 45d/232d             Reward: ≥teacher
```

### Observation Dimensions

| | Flat | Rough |
|---|---|---|
| **Teacher** | 48d (with base_lin_vel) | 235d (+ height_scan 187d) |
| **Student** | 45d (no base_lin_vel) | 232d (height_scan kept) |

Student keeps height_scan (real sensor). Only `base_lin_vel` is privileged.

## Full Training Run from Scratch

All commands inside `isaaclab:latest` on GCP. Docker template:

```bash
docker run --gpus all -d --name <NAME> \
  -v /home/ubuntu/workspace:/workspace \
  -v /home/ubuntu/results:/results \
  -v /home/ubuntu/uploads:/uploads \
  isaaclab:latest /isaac-sim/python.sh /workspace/biped_locomotion/<SCRIPT> <ARGS> --headless
```

### Step 1: Train Teacher (Phase 1)

```bash
# Flat teacher (~2.5s/iter, ~2h for 3000 iters)
# Add --tanh for bounded actions [-1, +1] (recommended for deploy)
docker run --gpus all -d --name biped_train_flat \
  ... isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --num_envs 8192 --max_iterations 6000 --urdf light --tanh --headless

# Monitor
docker logs -f biped_train_flat 2>&1 | grep "Mean reward"

# Checkpoints saved every 200 iters to /results/logs/rsl_rl/biped_flat_v52/
# Pick best checkpoint (highest reward, typically iter 2500-3000)
```

**Rough teacher** (optional, for rough terrain deployment):
```bash
docker run --gpus all -d --name biped_train_rough \
  ... isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_train_rsl.py \
  --rough --num_envs 8192 --max_iterations 6000 --urdf heavy --headless
# ~5.2s/iter, ~8.5h
```

Expected: reward ~19 (flat), ~17-18 (rough).

### Step 2: Distill Student (Phase 2)

```bash
docker run --gpus all -d --name biped_distill \
  ... isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_distill_rsl.py \
  --teacher_checkpoint /results/logs/rsl_rl/biped_flat_v52/model_2899.pt \
  --num_envs 16384 --max_iterations 3000 --urdf heavy --headless
# Add --rough for rough terrain distillation

# Monitor behavior loss (should drop from ~7 to ~0.2)
docker logs -f biped_distill 2>&1 | grep "behavior loss"

# Checkpoints: /results/logs/rsl_rl/biped_distill_flat/
# Pick checkpoint with lowest loss (not highest reward — MSE is the objective)
```

Expected: behavior loss ~0.2, reward ~60-70% of teacher. May need 3000-7000 iters for rough terrain.

### Step 3: Fine-tune Student (Phase 3)

```bash
docker run --gpus all -d --name biped_finetune \
  ... isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_finetune_student_rsl.py \
  --distilled /results/logs/rsl_rl/biped_distill_flat/model_2400.pt \
  --num_envs 16384 --max_iterations 5000 --urdf heavy --headless
# Add --rough for rough terrain

# Monitor reward (should match or exceed teacher)
docker logs -f biped_finetune 2>&1 | grep "Mean reward"

# Checkpoints: /results/logs/rsl_rl/biped_student_flat/
```

**Critical**: Phase 3 uses conservative PPO to protect pre-trained actor:
- LR 3e-4 (not 1e-3) — random critic would destroy actor at high LR
- clip 0.1 (not 0.2) — limits policy update magnitude
- entropy 0.001 (not 0.005) — student already knows what to do

Expected: reward starts near 0 (critic warmup), reaches teacher level by iter ~3000-4000.

### Step 4: Verify — Play / Record Video

```bash
# Start Xvfb (required for video on headless GPU)
pkill Xvfb; Xvfb :99 -screen 0 1920x1080x24 &>/dev/null &

# Play student policy
docker run --gpus all -d --name biped_play \
  -e DISPLAY=:99 -v /tmp/.X11-unix:/tmp/.X11-unix \
  ... isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/biped_play_rsl.py \
  --student --checkpoint /results/logs/rsl_rl/biped_student_flat/model_3400.pt \
  --video --video_length 500 --num_envs 8 --env_index 0 \
  --video_dir /results/videos --urdf light --headless

# For tanh-trained teacher (add --tanh to match training):
#   --tanh --checkpoint model.pt --num_envs 8 --global_camera
# For tanh student (Phase 3):
#   --tanh --student --checkpoint student_model.pt
# Action stats (abs/rms/min/max per joint) printed at end + saved as action_stats.csv
```

### Typical Training Timeline (L4 GPU)

| Phase | Envs | Iter Time | Iters | Wall Time |
|-------|------|-----------|-------|-----------|
| 1. Teacher flat | 16384 | 2.5s | 3000 | ~2h |
| 1. Teacher rough | 8192 | 5.2s | 6000 | ~8.5h |
| 2. Distill flat | 16384 | 5.7s | 3000 | ~4.5h |
| 3. Fine-tune flat | 16384 | 5.7s | 5000 | ~8h |
| **Total (flat)** | | | | **~14.5h** |

## Actuator Config (V138)

Using `DelayedPDActuator` for training to better map sim-to-real gap. Actions are scaled `[-1, 1]` via `--tanh` flag.

| Joint | Kp | Kd | Effort (Nm) | Armature | Action Scale |
|-------|----|----|-------------|----------|--------------|
| hip_roll/yaw | 100 | 3.0 | 50 | 0.01 | 0.5 |
| hip_pitch | 100 | 3.0 | 100 | 0.02 | 0.5 |
| knee | 100 | 3.0 | 100 | 0.02 | 0.5 |
| foot_pitch | 30 | 1.0 | 30 | 0.005 | 0.5 |
| foot_roll | 30 | 1.0 | 30 | 0.005 | 0.25 |

*Delay:* 1-7 steps (at 500Hz physics, ~2-14ms delay)

Foot: 30Nm each via parallel linkage (2 motors × 15Nm).

## ActuatorNet Pipeline (Sim2Real)

The ActuatorNet pipeline captures real-world actuator dynamics (especially the parallel linkage in the ankles) and trains an MLP to replace the `DelayedPDActuator` in Isaac Sim. This drastically closes the Sim2Real gap.

The pipeline consists of three stages:

### 1. Hardware Data Collection (`SYSID_CHIRP`)
Collects highly synchronized joint targets, positions, velocities, and raw torques from the real robot.

1. **Configure the chirp limits:** Edit `deploy/biped_ws/src/biped_bringup/config/chirp.yaml`. (These limits are mathematically scaled to safely test the mechanical range without hitting end-stops. Hips/Knees at 50%, Ankles at 70%).
2. **Run the ROS 2 Data Recorder:**
   ```bash
   python3 actuator_net/record_data.py
   ```
   *(This node perfectly syncs `/joint_commands`, `/joint_states`, and `/motor_states`, and computes virtual ankle torques using the inverse Jacobian).*
3. **Trigger the Chirp Sequence:**
   Using the teleop node or direct ROS 2 topics, transition the robot to the `SYSID_CHIRP` state. The robot will interpolate to the center of the chirp bounds, and then oscillate each joint from 0.1Hz to 10Hz over 10 seconds.
4. **Save Data:** Press `Ctrl+C` in the recorder terminal to dump `sysid_data.csv`.

### 2. Model Training (`actuator_net/train.py`)
Trains a lightweight MLP (`Linear(12, 32) -> ELU -> Linear(32, 32) -> ELU -> Linear(32, 1)`) to predict the actual motor torque based on a rolling history (`k_history=6`) of position errors and velocities.

```bash
python3 actuator_net/train.py --csv sysid_data_air.csv sysid_data_ground.csv --joint_type foot_pitch --epochs 100
```
- **Inputs:** Rolling window of 6 `q_err` + 6 `dq` (12 dims) perfectly matched to Isaac Lab's `ActuatorNetMLP` schema.
- **Outputs:** Best validation epoch is automatically exported as a Torch JIT script (e.g. `foot_pitch_net.pt`). Both left and right side data are automatically combined to double the dataset size.

### 3. Validation in Isaac Sim (`sysid/sysid_isaac.py`)
Verifies the trained ActuatorNet matches the real hardware response.

1. The standalone Isaac testing script (`sysid/sysid_isaac.py`) and hardware CAN tester (`deploy/scripts/motor_sysid.py`) both automatically parse `chirp.yaml` to ensure the simulated sine sweeps match the exact amplitudes tested on hardware.
2. In your Isaac environment config, replace `DelayedPDActuatorCfg` with `ActuatorNetMLPCfg` for the specific joint group, pointing it to the exported `.pt` model.
3. Run the validation:
   ```bash
   /isaac-sim/python.sh sysid/sysid_isaac.py --joint R_foot_pitch --headless
   ```

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

`feet_air_time` uses adaptive mode: continuous reward for iters 0–500 (bootstraps stepping), then impact-based from iter 500+ (refines gait quality).

Rough terrain: `flat_orientation_l2=0`, `dof_pos_limits=0`.

## Command Ranges

```python
lin_vel_x = (-0.5, 1.5)    # forward=+X (biased positive)
lin_vel_y = (-0.5, 0.5)    # lateral (small)
ang_vel_z = (-1.0, 1.0)
```

Curriculum expands `lin_vel_x` toward (-0.5, 3.0) based on tracking reward.

## Winners (in `winners/`)

No checkpoints yet — training with +X forward URDFs in progress. Old -Y checkpoints available at tag `v1.0-neg-y-forward`.

## Key Lessons

1. **Forward is +X**: Commands, curriculum use lin_vel_x for forward
2. **ImplicitActuator >> DCMotor**: DCMotor saturation kills exploration
3. **10kg battery mass**: Prevents sliding exploits, improves feet_air_time 5×
4. **Impact-based feet_air_time**: Prevents sliding exploits vs continuous reward
5. **Joint mapping**: Isaac runtime order ≠ URDF order — always verify via `robot.joint_names`
6. **Parallel ankle**: Set PR effort = 2 × motor torque, don't model coupling in sim
7. **Phase 3 needs conservative PPO**: Random critic + high LR destroys pre-trained actor. Use LR 3e-4, clip 0.1
8. **Distillation plateaus at ~60% teacher**: This is normal. Phase 3 closes the gap to 100%+
9. **Pick distill checkpoint by loss, not reward**: Lowest MSE loss = best initialization for Phase 3
