# Motor System Identification

Characterize actuator response in Isaac Sim and on real hardware.

## Files

| File | Location | Purpose |
|------|----------|---------|
| `sysid_isaac.py` | `sysid/` | Isaac Sim standalone (SimulationContext + Articulation) |
| `sysid_config.py` | `sysid/` | V73 actuator config, joint params, test targets |
| `motor_sysid.py` | `deploy/scripts/` | Real robot via ROS2/CAN |

## Isaac Sim Sysid

**Standalone approach** — no `ManagerBasedRLEnv`. Uses `SimulationContext` + `Articulation` directly.
No randomization events, no rewards, no terminations. Pure actuator response.

### Architecture
```
SimulationContext (200Hz physics)
    └── Articulation (SYSID_ROBOT_CFG)
            ├── fix_base=True (suspended)
            ├── gravity ON
            └── V73 DelayedPDActuator (same as training)

Control loop (50Hz = 4× decimation):
    1. set_joint_position_target(absolute_targets)
    2. write_data_to_sim()
    3. sim.step() × 4
    4. robot.update(dt)
    5. read pos, vel → CSV
```

### V74 Actuator Gains (current training)
| Joint Group | Kp | Kd | Effort | Friction |
|-------------|----|----|--------|----------|
| hip_roll/yaw | 10 | 3 | 50Nm | 0.375Nm |
| hip_pitch/knee | 15 | 3 | 100Nm | 0.5Nm |
| foot_pitch/roll | 16 | 2 | 30Nm | 0.25Nm |

### Usage
```bash
# Inside isaaclab Docker on GCP:
/isaac-sim/python.sh sysid/sysid_isaac.py --joint R_hip_pitch --headless
/isaac-sim/python.sh sysid/sysid_isaac.py --joint R_hip_pitch --video --headless
```

### Docker
```bash
docker run -d --gpus all --name sysid \
  -v /home/ubuntu/workspace/biped_locomotion:/workspace/biped_locomotion \
  -v /home/ubuntu/uploads:/uploads \
  -v /home/ubuntu/results:/results \
  isaaclab:latest \
  /isaac-sim/python.sh /workspace/biped_locomotion/sysid_isaac.py \
    --joint R_hip_pitch --output /results/motor_sysid --headless
```

### Output
```
motor_sysid/R_hip_pitch_RS04/
├── metadata.yaml
├── step_response.csv      ← time, cmd_pos, pos, vel, torque
├── sine_0.5hz.csv
├── sine_1.0hz.csv
├── sine_2.0hz.csv
├── sine_5.0hz.csv
└── sine_10.0hz.csv
```

### Tests per joint
1. **Step response**: 0.5s hold → 2s step to target → 1s return
2. **Sine sweeps**: 0.5, 1.0, 2.0, 5.0, 10.0 Hz (5 cycles each)

### Test joints
| Joint | Motor | Step Target | Sine Amp |
|-------|-------|-------------|----------|
| R_hip_pitch | RS04 | 0.4 rad | 0.3 rad |
| R_hip_roll | RS03 | -0.3 rad | 0.3 rad |
| R_hip_yaw | RS03 | 0.3 rad | 0.3 rad |
| R_knee | RS04 | 0.8 rad | 0.3 rad |
| R_foot_pitch | RS02 | 0.1 rad | 0.15 rad |
| R_foot_roll | RS02 | 0.1 rad | 0.1 rad |

## Real Robot Sysid

```bash
# On Pi (robot in STAND state):
python3 deploy/scripts/motor_sysid.py --all --output ~/sysid_data
python3 deploy/scripts/motor_sysid.py --joint R_hip_pitch --kp 1 --kd 0.1
```

## Findings

### URDF Velocity Limit (Critical Fix)
- URDF had `velocity="10"` on all joints — PhysX enforces this at physics level
- Caused ±10 rad/s chattering regardless of actuator config
- Fixed: RS04/RS03 → 21 rad/s, RS02 → 30 rad/s (matching hardware)

### PD Loop Rate
- `write_data_to_sim()` must be called every physics substep for true high-rate PD
- Calling it once per control step → PD runs at 50Hz regardless of physics rate
- Fix: move `set_joint_position_target` + `write_data_to_sim` inside substep loop

### Video Recording
- Camera sensor approach produces black frames (Xvfb 0Hz refresh)
- Fixed: use `omni.replicator.core` render product + RGB annotator (same as Isaac Lab env)

## Notes
- Isaac cold boot: ~15-45 min on L4 GPU (varies with parallel training load)
- Standalone approach bypasses env randomization events — clean results
- The env-based sysid failed due to `scale_all_actuator_torque_constant`
- `joint_drive.target_type` must be `"position"` (not `"none"`) for explicit actuators
