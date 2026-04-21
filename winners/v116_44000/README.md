# V116 — Light URDF, Tanh, DelayedPD (Apr 18, 2026)

## Checkpoint
- **model_44000.pt** (latest, 44K iterations total)
- Also saved as `/results/winners/latest_light.pt`

## Results (at iter 44000)
| Metric | Value |
|--------|-------|
| Reward | 22.5 |
| track_lin_vel | 0.89 (89%) |
| track_ang_vel | 0.30 |
| feet_air_time | -0.009 (negative = sliding gait) |
| feet_slide | -0.016 |
| timeout% | 100% (never falls) |
| flat_orientation | -0.002 |
| hip_deviation | -0.062 |
| foot_contact_force | -0.027 |
| base_contact (falls) | ~0% |

## Training trajectory
- **Iter 0-10K**: From scratch, rapid learning. Reward 0→20, vel 0→0.89
- **Iter 10K-30K**: Slow improvement. Reward 20→22.5, feet_air peaked 0.05 at 12K then collapsed
- **Iter 30K-44K**: Plateau. Reward ~22.5, air_time converged negative (sliding)
- Resumed from 29.4K at iter ~10K in this run (total 44K = 29.4K + 14.6K new)

## Config
- **URDF**: Light (15.6kg, no battery_chest)
- **Actuators**: DelayedPDActuator with friction + action delay (0-6ms)
  - hip_roll/yaw: Kp=180, Kd=3-6.5, friction=0.375Nm, effort=50Nm
  - hip_pitch/knee: Kp=180, Kd=3-6.5, friction=0.5Nm, effort=100Nm
  - foot_pitch/roll: Kp=30, Kd=1.0, friction=0.25Nm, effort=30Nm
- **Actions**: scale=0.5 (hip/knee/foot_pitch), 0.25 (foot_roll), **tanh output**
- **PPO**: [512, 256, 128] ELU, init_noise_std=1.0, LR=1e-3 adaptive
  - entropy_coef=0.005, clip=0.2, 5 epochs, 4 mini-batches
  - gamma=0.99, lam=0.95, desired_kl=0.01
- **Envs**: 4096, episode 20s, decimation=10 (50Hz), physics 1000Hz
- **Obs**: 48d (includes base_lin_vel)
- **Actions**: 12d, tanh bounded [-1,+1]
- **Self-collisions**: OFF
- **Rewards** (13 terms, Berkeley flat):
  - Positive: track_lin_vel (1.0), track_ang_vel (0.5), feet_air_time (10.0)
  - Penalties: lin_vel_z (-2.0), ang_vel_xy (-0.01), torques (-1e-5), action_rate (-0.01),
    feet_slide (-0.25), foot_contact_force (-0.005), undesired_contacts (-1.0),
    hip_deviation (-0.5), knee_deviation (-0.01), foot_roll_deviation (-0.3),
    stand_still (-0.2), flat_orientation (-0.5), dof_pos_limits (-1.0)
- **Feet air**: Berkeley impact-based (compute_first_contact), threshold_min=0.25, threshold_max=0.30
- **Curriculum**: Push force (start iter 1000, adaptive ±0.5→0.8 m/s), Command vel (start iter 5000)
- **Events**: Mass rand (1.0-1.2x), joint default pos ±0.05, gain rand (0.8-1.2x), friction rand (0.9-1.1x)
- **Defaults**: hip_pitch ±0.08, knee 0.25, foot_pitch -0.17

## Known issues
- **Sliding gait**: feet_air_time negative throughout training — policy never learns to lift feet
- MuJoCo sim2sim confirms: stable at z≈0.78 but slow tumble (qw 0.998→0.312 over 10s)
- Reward plateaued at ~22.5 since iter 30K
- Previous V116 checkpoint (28400) showed similar sliding behavior

## Play command
```bash
DISPLAY=:0 /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/winners/v116_44000/model_44000.pt \
  --video --video_length 300 --num_envs 1 --headless --urdf light --tanh --global_camera
```

## Resume checkpoint for experiments
- `model_10200.pt` saved in log dir — early point before sliding gait locked in
