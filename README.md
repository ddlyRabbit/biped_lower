# Biped Locomotion — RL Training (IsaacLab + rsl_rl)

Bipedal robot walking via PPO in Isaac Sim / IsaacLab.

## Active Files
- `biped_env_cfg.py` — current env config (loaded by training)
- `biped_train_rsl.py` — training script (rsl_rl OnPolicyRunner)
- `biped_play_rsl.py` — inference + video recording

## Versioned Configs
- `biped_env_cfg_v54_continuous.py` — continuous feet_air reward, threshold_min=0.05
- `biped_env_cfg_v55_impact_based.py` — impact-based feet_air reward, threshold_min=0.05

## Robot
- URDF: `/uploads/robot.urdf` (with 10kg battery_chest)
- STL meshes: `/uploads/assets/`
- Forward axis: **-Y** (not +X)
- Asymmetric hip roll/pitch limits (mirrored L/R)

## Training
```bash
# Train from scratch (inside isaaclab:latest container)
/isaac-sim/python.sh biped_train_rsl.py --num_envs 8192 --max_iterations 5000 --headless

# Resume from checkpoint
/isaac-sim/python.sh biped_train_rsl.py --num_envs 8192 --max_iterations 5000 \
  --resume /results/logs/rsl_rl/biped_flat_v52/model_1498.pt --headless

# Record video
/isaac-sim/python.sh biped_play_rsl.py \
  --checkpoint /results/logs/rsl_rl/biped_flat_v52/model_1498.pt \
  --num_envs 4 --video --video_length 300 --headless
```

## Directories
- `debug/` — diagnostic scripts (contact analysis, reward debugging)
- `legacy/` — old skrl-based training code (pre rsl_rl migration)

## Key Lessons
See CHECKPOINTS.md for version history and what worked/failed.
