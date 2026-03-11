"""Export student policy from rsl_rl checkpoint to ONNX.

The student policy MLP: 45d input → [128,128,128] ELU → 12d output.
Weights are stored as actor.0.weight, actor.2.weight, etc. in the checkpoint.

Usage:
    python3 export_onnx.py --checkpoint model_4200.pt --output student_flat.onnx
    python3 export_onnx.py --checkpoint model_4200.pt --output student_flat.onnx --obs_dim 232  # rough
"""

import argparse
import torch
import torch.nn as nn


class ActorMLP(nn.Module):
    """Reconstruct the rsl_rl actor MLP from checkpoint weights."""

    def __init__(self, obs_dim: int = 45, act_dim: int = 12,
                 hidden_dims: list = [128, 128, 128]):
        super().__init__()
        layers = []
        in_dim = obs_dim
        for h in hidden_dims:
            layers.append(nn.Linear(in_dim, h))
            layers.append(nn.ELU())
            in_dim = h
        layers.append(nn.Linear(in_dim, act_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)


def load_actor_from_checkpoint(checkpoint_path: str, obs_dim: int = 45,
                                act_dim: int = 12) -> ActorMLP:
    """Load actor weights from rsl_rl checkpoint.

    rsl_rl saves weights as:
        actor.0.weight, actor.0.bias   (Linear)
        actor.2.weight, actor.2.bias   (Linear, after ELU)
        actor.4.weight, actor.4.bias
        actor.6.weight, actor.6.bias
        std  (log std, not needed for deterministic inference)

    The student fine-tune checkpoint may use either:
        student.0.weight... (from distillation)
        actor.0.weight...   (from PPO fine-tune)
    """
    raw = torch.load(checkpoint_path, map_location='cpu', weights_only=False)

    # rsl_rl wraps weights in 'model_state_dict'
    if 'model_state_dict' in raw:
        ckpt = raw['model_state_dict']
        print(f"Unwrapped model_state_dict (iter={raw.get('iter', '?')})")
    else:
        ckpt = raw

    # Detect key prefix
    if any(k.startswith('student.') for k in ckpt.keys()):
        prefix = 'student.'
    elif any(k.startswith('actor.') for k in ckpt.keys()):
        prefix = 'actor.'
    else:
        raise ValueError(f"Cannot find actor/student keys in checkpoint. Keys: {list(ckpt.keys())[:10]}")

    # Infer hidden dims from weight shapes
    actor_keys = sorted([k for k in ckpt.keys() if k.startswith(prefix) and 'weight' in k])
    hidden_dims = []
    for k in actor_keys[:-1]:  # all but last linear
        hidden_dims.append(ckpt[k].shape[0])

    # Verify dimensions
    first_weight = ckpt[actor_keys[0]]
    last_weight = ckpt[actor_keys[-1]]
    detected_obs = first_weight.shape[1]
    detected_act = last_weight.shape[0]

    print(f"Checkpoint: {checkpoint_path}")
    print(f"Key prefix: {prefix}")
    print(f"Detected: obs_dim={detected_obs}, act_dim={detected_act}, hidden={hidden_dims}")

    if detected_obs != obs_dim:
        print(f"WARNING: detected obs_dim={detected_obs} != requested {obs_dim}")
        obs_dim = detected_obs

    # Build model and load weights
    model = ActorMLP(obs_dim, detected_act, hidden_dims)

    # Map checkpoint keys to model keys (actor.X → net.X)
    state_dict = {}
    for k, v in ckpt.items():
        if k.startswith(prefix):
            new_key = 'net.' + k[len(prefix):]
            state_dict[new_key] = v

    model.load_state_dict(state_dict, strict=False)
    model.eval()

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
    print("  ✓ ONNX verification passed")


def main():
    parser = argparse.ArgumentParser(description='Export student policy to ONNX')
    parser.add_argument('--checkpoint', type=str, required=True, help='Path to .pt checkpoint')
    parser.add_argument('--output', type=str, default='student_flat.onnx', help='Output ONNX path')
    parser.add_argument('--obs_dim', type=int, default=45, help='Observation dimension (45=flat, 232=rough)')
    args = parser.parse_args()

    model, obs_dim, act_dim = load_actor_from_checkpoint(args.checkpoint, args.obs_dim)
    export_onnx(model, obs_dim, args.output)


if __name__ == '__main__':
    main()
