# Winner Models

## V72 Teacher (Mar 23, 2026) — Best Walking Policy

| File | Description |
|------|-------------|
| `v72_teacher_5999.pt` | 6K iters from scratch |
| `v72_teacher_8998.pt` | +3K continued (threshold 0.15s) |
| `v72_teacher_5999.mp4` | Video of 5999 checkpoint |

**Config:**
- Actuator: DelayedPDActuator, delay 0-5ms, friction 0.75/1.0/0.5 Nm
- Kp: hip_roll/yaw=10, hip_pitch/knee=15, foot=8 | Kd: hip=3, foot=0.2
- action_scale=0.5, no tanh (unbounded actions, RMS 2-4, max ~10)
- Self-collisions OFF, symmetry loss ON
- Berkeley impact air_time, threshold_min=0.15s
- Defaults: hip_pitch ±0.08, knee 0.25, foot_pitch -0.17
- Light URDF (15.6kg), 8192 envs, rsl_rl PPO [128,128,128]

**Metrics (5999):** reward 20.5, vel 0.89, falls 2.4%

**Note:** Unbounded actions — policy saturates actuators. Needs student distillation with bounded actions for deploy.

**Play:**
```bash
docker run --gpus all \
  -e DISPLAY=:2 -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ubuntu/workspace/biped_locomotion:/workspace/biped_locomotion \
  -v /home/ubuntu/uploads:/uploads -v /home/ubuntu/results:/results \
  isaaclab:latest /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/winners/v72_teacher_5999.pt \
  --num_envs 8 --urdf light --video --global_camera --headless
```
