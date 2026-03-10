# Winner Checkpoints

No checkpoints yet. Training with +X forward URDFs in progress.

## Format

Checkpoints are rsl_rl PPO `.pt` files containing `actor.*`, `critic.*`, and `std` keys.

## Naming Convention

```
v<version>_<terrain>_model_<iter>.pt
```

Example: `v58_flat_model_3000.pt`, `v58_rough_model_6000.pt`, `v58_student_flat_model_4000.pt`
