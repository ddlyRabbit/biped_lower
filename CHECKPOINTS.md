# Checkpoint Manifest

All versions share: /results/logs/rsl_rl/biped_flat_v52/
Checkpoints may overwrite each other across versions.

## V52 — Berkeley exact rewards, impact-based feet_air w=0.5, threshold_min=0.2
- 2000 iters from scratch → sliding trap, no stepping
- model_1999.pt (overwritten by V53)

## V52b — resumed V52 for 10K more
- model_2000.pt → model_3200.pt (Mar 6 ~07:30)

## V53 — Skyentific: continuous feet_air w=2.0, flat_orient -0.5, torques -1e-5, threshold_min=0.2
- 2000 iters from scratch → still sliding despite positive feet_air
- model_0.pt → model_1999.pt (Mar 6 08:07-09:08, overwrote V52)

## V54 — threshold_min 0.2→0.05, continuous feet_air_time_positive_biped
- 500 iters from scratch → FIRST visible foot lifting
- **model_499.pt** (Mar 6 ~10:00)
- Config: biped_env_cfg_v54_continuous.py

## V55 — impact-based feet_air_time, threshold_min=0.05, resumed from V54 model_499
- 1000 iters (499→1498) → reward 15.55, track_lin 0.725, 87% survive
- **model_1498.pt** (Mar 6 ~11:00)
- Config: biped_env_cfg_v55_impact_based.py

## V55b — continued V55, 8192 envs, resumed from model_1498
- 4500 more iters (1498→5998)
- Push curriculum active. Command_vel curriculum at iter 5000.
- Checkpoints: model_1600+ (Mar 6 11:20+)
