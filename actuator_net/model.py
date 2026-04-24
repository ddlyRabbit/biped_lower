import torch
import torch.nn as nn

class ActuatorNet(nn.Module):
    def __init__(self, k_history=6, hidden_dim=32):
        super().__init__()
        input_dim = k_history * 2 # 6 pos_errors + 6 velocities
        
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ELU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ELU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, x):
        # x: [batch_size, 12]
        return self.net(x)
