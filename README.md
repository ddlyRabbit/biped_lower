# Biped Locomotion — RL Training (IsaacLab + rsl_rl)

Bipedal walking via PPO + teacher-student distillation. Isaac Sim / IsaacLab, rsl_rl framework.

**Branch `unitree-g1-rewards`**: Migrated to Unitree G1 29dof training config (rewards, obs, model, curriculum). Our hardware (URDF, actuators) unchanged.

## Architecture

### G1-Style Training Pipeline

```
Phase 1: Teacher PPO (50K iters)
  Model: [512, 256, 128] ELU, linear output
  Obs: 45d/frame × 5 history = 225d (policy), 48d × 5 = 240d (critic)
  Actions: 12 joints, scale=0.25, joint_names=[".*"] (Isaac alphabetical)
  Rewards: 17 terms (G1 exact), curriculum velocity ramp

Phase 2: Distillation (3K iters)
  Teacher obs: 240d → Student obs: 225d (no base_lin_vel)
  MSE loss on action outputs

Phase 3: Student Fine-tune PPO (5K iters)
  Student obs: 225d, conservative PPO (clip=0.1, LR=3e-4)

Deploy: ONNX on RPi 5
  Input: 225d (5-frame history buffer)
  Output: 12 joint position targets
  50Hz control loop
```

### Isaac Alphabetical Joint Order (canonical everywhere)

```
L_foot_pitch, L_foot_roll, L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee,
R_foot_pitch, R_foot_roll, R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee
```

## File Map

```
biped_locomotion/
│
├── Config ─────────────────────────────────────────────────────────
│   ├── biped_env_cfg.py              ← Flat env. Scene, rewards, obs, actuators
│   ├── biped_mdp.py                  ← Custom MDP functions (G1 rewards/curriculum)
│   └── biped_student_env_cfg.py      ← Student env (removes base_lin_vel from obs)
│       └── FlatStudentObservationsCfg    → policy(225d) + teacher(240d) + critic(240d)
│
├── Training ───────────────────────────────────────────────────────
│   ├── biped_train_rsl.py            ← Phase 1: PPO teacher [512,256,128]
│   ├── biped_distill_rsl.py          ← Phase 2: Teacher→student MSE distillation
│   └── biped_finetune_student_rsl.py ← Phase 3: PPO fine-tune of distilled student
│
├── Inference / Recording ──────────────────────────────────────────
│   └── biped_play_rsl.py             ← Play + video recording
│
├── URDF ───────────────────────────────────────────────────────────
│   ├── urdf/heavy/robot.urdf         ← ~21.7kg (with battery)
│   └── urdf/light/robot.urdf         ← ~15.6kg (light battery)
│       └── robot/                    ← USD files with physics layer
│
├── Deploy ─────────────────────────────────────────────────────────
│   ├── deploy/ARCHITECTURE.md
│   ├── deploy/BRINGUP.md
│   └── deploy/biped_ws/              ← ROS2 workspace
│       └── src/
│           ├── biped_control/        ← policy_node, obs_builder, state_machine
│           ├── biped_driver/         ← CAN + IMU hardware
│           ├── biped_bringup/        ← Launch files + config
│           ├── biped_teleop/         ← Keyboard/gamepad teleop
│           └── biped_tools/          ← export_onnx (input: 225d)
│
└── winners/                          ← Saved best checkpoints
```

## Config Summary (G1-Matched)

### Rewards (17 terms)

| Term | Weight | Function |
|------|--------|----------|
| track_lin_vel_xy | +1.0 | yaw-frame exp, std=√0.25 |
| track_ang_vel_z | +0.5 | exp, std=√0.25 |
| alive | +0.15 | is_alive |
| lin_vel_z_l2 | -2.0 | vertical velocity |
| ang_vel_xy_l2 | -0.05 | roll/pitch rate |
| joint_vel_l2 | -0.001 | joint velocity |
| joint_acc_l2 | -2.5e-7 | joint acceleration |
| action_rate_l2 | -0.05 | action smoothness |
| joint_pos_limits | -5.0 | near joint limits |
| energy | -2e-5 | |torque × vel| |
| joint_deviation_legs | -1.0 | hip_roll + hip_yaw |
| flat_orientation_l2 | -5.0 | upright posture |
| base_height_l2 | -10.0 | target 0.73m |
| feet_gait | +0.5 | phase-based (0.8s period) |
| feet_slide | -0.2 | contact foot velocity |
| foot_clearance | +1.0 | swing foot height (0.1m) |
| undesired_contacts | -1.0 | non-foot body contacts |

### Observations

| Group | Per Frame | History | Total |
|-------|-----------|---------|-------|
| Policy | ang_vel×0.2(3) + gravity(3) + cmd(3) + pos_rel(12) + vel×0.05(12) + action(12) = 45d | 5 | 225d |
| Critic | + base_lin_vel(3) = 48d | 5 | 240d |

### PPO Config

```
actor/critic: [512, 256, 128] ELU
init_noise_std: 1.0
entropy_coef: 0.01
LR: 1e-3 (adaptive, desired_kl=0.01)
num_steps_per_env: 24
max_iterations: 50000
save_interval: 100
```

### Commands (with curriculum)

```
Initial: lin_vel_x ±0.1, lin_vel_y ±0.1, ang_vel_z ±0.1
Limit:   lin_vel_x (-0.5, 1.0), lin_vel_y ±0.3, ang_vel_z ±0.2
Ramp:    ±0.1 per episode boundary when tracking > 80%
```

### Actuators (OUR HARDWARE — not G1)

| Joint | Kp | Kd | Effort | Type |
|-------|----|----|--------|------|
| hip_pitch | 200 | 7.5 | 100Nm | DelayedPD |
| hip_roll | 150 | 5.5 | 50Nm | DelayedPD |
| hip_yaw | 150 | 5.0 | 50Nm | DelayedPD |
| knee | 200 | 5.0 | 100Nm | DelayedPD |
| foot_pitch | 30 | 2.0 | 30Nm | DelayedPD |
| foot_roll | 30 | 2.0 | 30Nm | DelayedPD |

All: armature=0.01, delay 0-1ms, friction 0.25-0.5 Nm

### Events / Domain Randomization

- Ground friction: (0.3, 1.0) static+dynamic
- Base mass: (-1, +3) kg on torso
- Push: ±0.5 m/s every 5s
- Reset: random pose ±0.5m, ±π yaw, zero velocity
- Reset joints: default position, velocity (-1, 1)

### Terminations

- time_out (20s)
- base_height < 0.2m
- bad_orientation > 0.8 rad (46°)

### Physics

- dt=0.005, decimation=4 (50Hz control)
- episode=20s, num_envs=4096

## Deploy

Target: RPi 5, ROS2 Jazzy, 12 RobStride motors, BNO085 IMU, 2× CAN bus.

- ONNX input: 225d (5-frame history, obs scaling applied in obs_builder)
- ONNX output: 12 joint targets (Isaac alphabetical order)
- Action: target = default_pos + action × 0.25
- Gains: same Kp/Kd as training (no 5× multiplier)
- Control rate: 50Hz
