# Motor System Identification

Characterize actuator response in simulation (Isaac Sim) and on real hardware.
Compare the two to quantify the sim-to-real gap and tune actuator models.

## Overview

```
┌─────────────────┐         ┌──────────────────┐
│  sysid_isaac.py │         │  sysid_real.py   │
│  (Isaac Sim)    │         │  (Real Robot)    │
│                 │         │                  │
│  Same actuator  │         │  ROS2 /joint_cmd │
│  model as       │         │  → CAN → motor   │
│  training env   │         │  → encoder fbk   │
└────────┬────────┘         └────────┬─────────┘
         │                           │
         ▼                           ▼
    CSV + video                   CSV data
    per joint                     per joint
         │                           │
         └───────────┬───────────────┘
                     ▼
           sysid_compare.py (TODO)
           Bode plots, step metrics,
           hysteresis analysis
```

## Files

| File | Purpose |
|------|---------|
| `sysid_isaac.py` | Isaac Sim sysid — uses training env (same actuator model) |
| `sysid_real.py` | Real robot sysid — ROS2 commands via CAN bus |
| `sysid_compare.py` | Compare sim vs real (TODO) |

## Isaac Sim Sysid

Uses the **exact same `biped_env_cfg.py`** and actuator model as training.
Robot suspended (fixed base), gravity ON. Records video + CSV data.

### Setup
- Robot base fixed (`fix_root_link=True`)
- Robot raised 500mm (z=1.30) for clearance
- Base contact termination disabled
- All non-test joints held at default positions

### Tests per joint
1. **Step response**: hold → step to target → hold → return → hold
2. **Sine sweeps**: 0.5, 1.0, 2.0, 5.0, 10.0 Hz at specified amplitude

### Representative joints (one per motor type)
| Joint | Motor | Step Target | Sine Amp |
|-------|-------|-------------|----------|
| R_hip_pitch | RS04 | 0.4 rad | 0.3 rad |
| R_hip_roll | RS03 | -0.3 rad | 0.3 rad |
| R_foot_pitch | RS02 | 0.1 rad | 0.15 rad |

### Usage

```bash
# On GCP (inside isaaclab Docker):
/isaac-sim/python.sh sysid_isaac.py --all --output /results/motor_sysid --headless

# Single joint:
/isaac-sim/python.sh sysid_isaac.py --joint R_hip_pitch --output /results/motor_sysid --headless
```

### Docker command (from host):
```bash
docker run -d --gpus all \
  --name sysid \
  -e DISPLAY=:2 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ubuntu/workspace/biped_locomotion:/workspace/biped_locomotion \
  -v /home/ubuntu/uploads:/uploads \
  -v /home/ubuntu/results:/results \
  isaaclab:latest \
  /isaac-sim/python.sh /workspace/biped_locomotion/sysid_isaac.py \
    --all --output /results/motor_sysid --headless
```

### Output structure
```
motor_sysid/
├── video/
│   └── sysid-episode-0.mp4          ← full video of all tests
├── R_hip_pitch_RS04/
│   ├── metadata.yaml                ← test params, gains, dt
│   ├── step_response.csv            ← time, cmd_pos, pos, vel, torque
│   ├── sine_0.5hz.csv
│   ├── sine_1.0hz.csv
│   ├── sine_2.0hz.csv
│   ├── sine_5.0hz.csv
│   └── sine_10.0hz.csv
├── R_hip_roll_RS03/
│   └── ...
└── R_foot_pitch_RS02/
    └── ...
```

### CSV format
```csv
time,cmd_pos,pos,vel,torque
0.00000,0.080000,0.079834,0.001234,0.019456
0.02000,0.080000,0.079912,0.000567,0.008234
```

## Real Robot Sysid

Same test protocol on the actual hardware via ROS2.

### Prerequisites
- Robot powered, CAN up, motors calibrated
- Bringup stack running (`ros2 launch biped_bringup bringup.launch.py`)
- Robot in STAND state

### Usage
```bash
# On Pi:
python3 sysid_real.py --all --output ~/sysid_data

# Single joint:
python3 sysid_real.py --joint R_hip_pitch --kp 1 --kd 0.1 --output ~/sysid_data
```

### Default gains for real robot sysid
All motor types: Kp=1, Kd=0.1 (safe low gains for characterization)

## Comparison (TODO)

`sysid_compare.py` will:
1. Load Isaac + real CSVs for the same joint
2. Compute step response metrics: rise time, settling time, overshoot, steady-state error
3. Generate Bode plots from sine sweep data (gain + phase vs frequency)
4. Identify hysteresis from position-torque loops
5. Fit actuator model parameters to minimize sim-real gap

## Notes

- Isaac sysid uses the **training actuator model** (DelayedPDActuator with friction)
- The env's action pipeline applies: `target = default_pos + action × action_scale`
- To set absolute joint targets, compute: `action = (desired - default) / action_scale`
- Isaac cold boot takes ~10-15 min (shader compilation) on L4 GPU
- Cannot run rendering + training simultaneously on a single GPU
- Video recording requires `--enable_cameras` and Xvfb (`:2`) on headless servers
