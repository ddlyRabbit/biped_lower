import torch
import torch.nn as nn

class ActuatorNet(nn.Module):
    def __init__(self, k_history, hidden_dim=32):
        super().__init__()
        input_dim = k_history * 2 # pos_errors + velocities
        
        # Paper: 3 hidden layers of 32 units each, Softsign activation
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.Softsign(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Softsign(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Softsign(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, x):
        # x: [batch_size, k_history * 2]
        return self.net(x)
