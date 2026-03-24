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

**Note:** Unbounded actions — policy saturates actuators. Superseded by V74 (tanh).

## V74 Teacher (active training, Mar 24, 2026)

**Config:**
- Tanh output layer (actions bounded [-1, +1])
- Actuator: DelayedPDActuator, delay 0-5ms
- Kp: hip_roll=120, hip_yaw=60, hip_pitch=180, knee=180, foot_pitch=96, foot_roll=48
- Kd: hip=3, foot=2 | action_scale=0.5
- Push curriculum: active, ramped to 3.0 m/s (max)
- Heavy URDF (0.5kg battery), 8192 envs
- Symmetry loss ON, Berkeley impact air_time

**Metrics (iter ~17800):** reward 9.8 (under 3.0 m/s push), vel 0.74, 70% survive
**Pre-push peak (iter ~16400):** reward 21.7, vel 0.92, 99% survive

**Full tanh pipeline verified:** Train → Distill → Fine-tune → Play → ONNX export

### V74 iter 16600 — saved checkpoint

| File | Description |
|------|-------------|
| `v74_teacher_16600.pt` | Trained with push curriculum (1.69 m/s at this point) |
| `v74_teacher_16600.mp4` | Video with push active |

**Metrics (16600):** reward 20.3, vel 0.89, falls 3.4%, push 1.69 m/s
**Pre-push peak (16400):** reward 21.7, vel 0.92, falls 0.7%

**Play:**
```bash
docker run --gpus all \
  -e DISPLAY=:2 -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ubuntu/workspace/biped_locomotion:/workspace/biped_locomotion \
  -v /home/ubuntu/uploads:/uploads -v /home/ubuntu/results:/results \
  isaaclab:latest /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
  --checkpoint /results/winners/v74_teacher_16600.pt \
  --num_envs 8 --urdf heavy --tanh --video --global_camera --headless
```

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
