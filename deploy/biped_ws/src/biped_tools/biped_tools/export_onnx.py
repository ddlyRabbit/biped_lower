"""Export student policy from rsl_rl checkpoint to ONNX.

Model: [128, 128, 128] ELU + optional Tanh output.
Default obs_dim=45 (student flat), act_dim=12.

Usage:
    python3 export_onnx.py --checkpoint model_5000.pt --output student_flat.onnx
    python3 export_onnx.py --checkpoint model_5000.pt --output student_flat.onnx --tanh
    python3 export_onnx.py --checkpoint model_5000.pt --output student_flat.onnx --obs_dim 45
"""

import argparse
import torch
import torch.nn as nn


class ActorMLP(nn.Module):
    """Reconstruct the rsl_rl actor MLP from checkpoint weights."""

    def __init__(self, obs_dim: int = 45, act_dim: int = 12,
                 hidden_dims: list = None, tanh: bool = False):
        super().__init__()
        if hidden_dims is None:
            hidden_dims = [128, 128, 128]
        layers = []
        in_dim = obs_dim
        for h in hidden_dims:
            layers.append(nn.Linear(in_dim, h))
            layers.append(nn.ELU())
            in_dim = h
        layers.append(nn.Linear(in_dim, act_dim))
        if tanh:
            layers.append(nn.Tanh())
        self.net = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)


def load_actor_from_checkpoint(checkpoint_path: str, obs_dim: int = 45,
                                act_dim: int = 12, tanh: bool = False):
    """Load actor weights from rsl_rl checkpoint.

    Supports two checkpoint formats:

    Without tanh (rsl_rl vanilla):
        actor.0.weight, actor.2.weight, actor.4.weight, actor.6.weight
        → maps to net.0.weight, net.2.weight, ...

    With tanh (--tanh training wraps actor in Sequential(MLP, Tanh)):
        actor.0.0.weight, actor.0.2.weight, actor.0.4.weight, actor.0.6.weight
        → the extra "0." is the MLP inside Sequential(MLP, Tanh)
        → strip "actor.0." prefix to get 0.weight, 2.weight, ...
        → maps to net.0.weight, net.2.weight, ...

    Student distillation checkpoints use "student." prefix instead of "actor.".
    PPO fine-tuned student checkpoints use "actor.0." prefix (same as tanh teacher).
    """
    raw = torch.load(checkpoint_path, map_location='cpu', weights_only=False)

    # rsl_rl wraps weights in 'model_state_dict'
    if 'model_state_dict' in raw:
        ckpt = raw['model_state_dict']
        print(f"Unwrapped model_state_dict (iter={raw.get('iter', '?')})")
    else:
        ckpt = raw

    # Detect key prefix and tanh wrapping
    has_student = any(k.startswith('student.') for k in ckpt.keys())
    has_actor_0_0 = any(k.startswith('actor.0.0.') for k in ckpt.keys())
    has_actor_0 = any(k.startswith('actor.0.') for k in ckpt.keys())
    has_actor = any(k.startswith('actor.') for k in ckpt.keys())

    if has_student:
        # Distillation checkpoint: student.0.weight, student.2.weight, ...
        prefix = 'student.'
        tanh_wrapped = False
    elif has_actor_0_0:
        # Tanh-wrapped PPO checkpoint: actor.0.0.weight (MLP inside Sequential)
        prefix = 'actor.0.'
        tanh_wrapped = True
    elif has_actor_0:
        # Could be tanh-wrapped or regular — check if actor.0.0 exists
        prefix = 'actor.0.'
        tanh_wrapped = True
    elif has_actor:
        # Vanilla rsl_rl: actor.0.weight, actor.2.weight, ...
        prefix = 'actor.'
        tanh_wrapped = False
    else:
        raise ValueError(f"Cannot find actor/student keys. Keys: {list(ckpt.keys())[:10]}")

    print(f"Key prefix: '{prefix}', tanh_wrapped={tanh_wrapped}")

    # Extract weight keys and infer hidden dims
    actor_weight_keys = sorted([k for k in ckpt.keys()
                                 if k.startswith(prefix) and 'weight' in k])
    hidden_dims = []
    for k in actor_weight_keys[:-1]:  # all but last linear
        hidden_dims.append(ckpt[k].shape[0])

    # Verify dimensions
    first_weight = ckpt[actor_weight_keys[0]]
    last_weight = ckpt[actor_weight_keys[-1]]
    detected_obs = first_weight.shape[1]
    detected_act = last_weight.shape[0]

    print(f"Checkpoint: {checkpoint_path}")
    print(f"Detected: obs_dim={detected_obs}, act_dim={detected_act}, hidden={hidden_dims}")

    if detected_obs != obs_dim:
        print(f"WARNING: detected obs_dim={detected_obs} != requested {obs_dim}, using detected")
        obs_dim = detected_obs

    # Build model
    model = ActorMLP(obs_dim, detected_act, hidden_dims, tanh=tanh)

    # Map checkpoint keys to model keys
    # checkpoint: prefix + "0.weight" → model: "net.0.weight"
    state_dict = {}
    for k, v in ckpt.items():
        if k.startswith(prefix):
            suffix = k[len(prefix):]
            new_key = 'net.' + suffix
            state_dict[new_key] = v

    model.load_state_dict(state_dict, strict=False)
    model.eval()

    # Verify all MLP weights loaded
    missing = [k for k in model.state_dict().keys()
               if k.startswith('net.') and 'weight' in k and k not in state_dict]
    if missing:
        print(f"WARNING: missing weight keys: {missing}")
    else:
        print(f"All {len(actor_weight_keys)} weight tensors loaded successfully")

    return model, obs_dim, detected_act


def export_onnx(model: ActorMLP, obs_dim: int, output_path: str):
    """Export model to ONNX format."""
    dummy_input = torch.randn(1, obs_dim)

    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        export_params=True,
        opset_version=11,
        input_names=['obs'],
        output_names=['actions'],
        dynamic_axes={
            'obs': {0: 'batch'},
            'actions': {0: 'batch'},
        },
    )
    print(f"Exported ONNX: {output_path}")
    print(f"  Input:  obs [{obs_dim}]")
    print(f"  Output: actions [12]")

    # Verify
    import onnxruntime as ort
    sess = ort.InferenceSession(output_path)
    test_obs = dummy_input.numpy()
    result = sess.run(None, {'obs': test_obs})
    print(f"  Verify: input shape={test_obs.shape}, output shape={result[0].shape}")
    print(f"  Sample output: {result[0][0][:4]}...")

    # Check output range (tanh should bound to [-1, 1])
    import numpy as np
    out_abs_max = np.max(np.abs(result[0]))
    if out_abs_max <= 1.01:
        print(f"  Output range: [{result[0].min():.4f}, {result[0].max():.4f}] — tanh bounded ✓")
    else:
        print(f"  Output range: [{result[0].min():.4f}, {result[0].max():.4f}] — UNBOUNDED (no tanh?)")

    print("  ✓ ONNX verification passed")


def main():
    parser = argparse.ArgumentParser(description='Export student policy to ONNX')
    parser.add_argument('--checkpoint', type=str, required=True, help='Path to .pt checkpoint')
    parser.add_argument('--output', type=str, default='student_flat.onnx', help='Output ONNX path')
    parser.add_argument('--obs_dim', type=int, default=45, help='Observation dimension (45=student flat)')
    parser.add_argument('--tanh', action='store_true', help='Include Tanh output layer (must match training)')
    args = parser.parse_args()

    model, obs_dim, act_dim = load_actor_from_checkpoint(
        args.checkpoint, args.obs_dim, tanh=args.tanh)
    export_onnx(model, obs_dim, args.output)


if __name__ == '__main__':
    main()
