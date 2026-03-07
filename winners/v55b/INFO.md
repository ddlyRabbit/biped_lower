# V55b — Best Model (as of 2026-03-06)

## Checkpoint
- **File:** model_5997.pt
- **Total iters:** 5997 (V54: 0→499, V55: 499→1498, V55b: 1498→5997)
- **Envs:** 8192
- **Framework:** rsl_rl (PPO)

## Final Metrics
- **Mean reward:** 21.5 (theoretical max ~42)
- **track_lin_vel:** 0.899 (89% of max)
- **track_ang_vel:** 0.382 (38% of max)
- **feet_air_time:** 0.021
- **feet_slide:** -0.028
- **base_contact (falls):** 0.6%
- **ep_len:** 1000 (max, 20s)
- **noise_std:** 0.36

## Config
- **Reward:** Impact-based feet_air_time (Berkeley), weight 2.0
- **threshold_min:** 0.05, threshold_max: 0.5
- **Actuators:** ImplicitActuator, 2× Berkeley Humanoid values
- **URDF:** robot.urdf with 10kg battery_chest (~27.5kg total)
- **Decimation:** 4 (50 Hz control)
- **action_scale:** 0.5
- **Self-collisions:** OFF
- **Contact processing:** DISABLED
- **Disturbance forces:** push_robot ±0.5 m/s only (no external force/torque)
- **Curriculum:** push_force_levels (after iter 1500) + command_vel (after iter 5000)

## Lineage
- V54 (0→499): threshold_min 0.2→0.05, continuous reward, from scratch — first foot lifting
- V55 (499→1498): switched to impact-based reward, inflection at ~750 iters
- V55b (1498→5997): 8192 envs, steady refinement, reward 3.7→21.5

## Known Limitations
- Self-collisions OFF — legs clip through each other
- No external disturbance forces (only velocity pushes)
- feet_air 0.021 stuck — likely still partially sliding
- track_ang only 38% — poor turning
- Torque utilization ~20% of limits (not fully exploiting actuators)

## Power Estimate
- RMS torque: ~14.7 Nm per joint
- Hip/knee utilization: 13-19%, foot_pitch: 73%
- Total mechanical power: ~175-350W (realistic for 27.5kg biped)
