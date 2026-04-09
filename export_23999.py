import torch
import torch.nn as nn
import onnxruntime as ort
import numpy as np

class ActorMLP(nn.Module):
    def __init__(self, obs_dim: int = 48, act_dim: int = 12, hidden_dims: list = None, tanh: bool = False):
        super().__init__()
        if hidden_dims is None:
            hidden_dims = [512, 256, 128]
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

# Load checkpoint
ckpt_path = 'deploy/model_23999.pt'
ckpt = torch.load(ckpt_path, map_location='cpu')
sd = ckpt['model_state_dict']

# Create model
model = ActorMLP(obs_dim=48, act_dim=12, hidden_dims=[512, 256, 128], tanh=False)

# Create new state_dict with correct keys
new_sd = {}
for k, v in sd.items():
    if k.startswith('actor.'):
        new_key = 'net.' + k[len('actor.'):]
        new_sd[new_key] = v

# Load weights
model.load_state_dict(new_sd, strict=False)
model.eval()

# Export to ONNX
output_path = 'deploy/v82_teacher_light_23999.onnx'
dummy_input = torch.randn(1, 48)
torch.onnx.export(
    model,
    dummy_input,
    output_path,
    export_params=True,
    opset_version=11,
    input_names=['obs'],
    output_names=['actions'],
    dynamic_axes={'obs': {0: 'batch'}, 'actions': {0: 'batch'}},
)

print(f"Exported ONNX: {output_path}")

# Verify
sess = ort.InferenceSession(output_path)
test_obs = dummy_input.numpy()
result = sess.run(None, {'obs': test_obs})
print(f"  Verify: input shape={test_obs.shape}, output shape={result[0].shape}")

