# Winners Directory

This directory holds the best PyTorch checkpoints. All legacy checkpoints prior to V131 were wiped due to the global `MASTER_JOINT_ORDER` refactor.

## V188 Teacher (Light, Heavy Action Smoothing) (Jul 28, 2026)
- **Model**: `v188_teacher_light_21000/v188_teacher_light_21000.onnx` (exported from model_21000.pt)
- **Reward**: ~19.6 | **Vel Tracking**: ~0.84 | **Falls**: ~2.9%
- **URDF**: Light (15.6kg)
- **Curriculum Pushes**: Maxed out at **0.8 m/s** (roughly 80N lateral force).
- **Key Characteristics**:
  - **Heavy Smoothing:** `action_rate_l2` penalty weight drastically increased to `-0.3` (3x the baseline). This forced the neural network to act as its own low-pass filter.
  - **No Accel Penalty:** Explicit `joint_accel_l2` physical penalty was removed; the heavy action rate penalty alone was sufficient to eliminate physical joint jitter.
  - **Resumed Training:** Fine-tuned dynamically by resuming from the `v181` checkpoint (20200) rather than starting from scratch, preventing curriculum paralysis.
- **Network**: PPO [512,256,128] ELU + Tanh output, 48-dim obs (privileged).
- **MuJoCo Validation**: Verified in sim2sim at 0.0, 0.3, 0.4, and 0.5 m/s in both forward and lateral (Y) directions. Gait remains incredibly fluid and parallel even with the EMA software filter entirely disabled (`alpha=1.0`).
