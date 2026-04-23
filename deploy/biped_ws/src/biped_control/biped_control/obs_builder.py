"""Observation vector builder — maps real sensor data to 45d policy input.

Must match training observation order EXACTLY:
  [0-2]   base_ang_vel (3d)     ← IMU gyro (rad/s, body frame)
  [3-5]   projected_gravity (3d) ← IMU gravity normalized to unit vector
  [6-8]   velocity_commands (3d) ← cmd_vel (lin_x, lin_y, ang_z)
  [9-14]  hip_pos (6d)           ← joint_pos_rel: L_roll,R_roll,L_yaw,R_yaw,L_pitch,R_pitch
  [15-16] knee_pos (2d)          ← joint_pos_rel: L_knee, R_knee
  [17-18] foot_pitch_pos (2d)    ← joint_pos_rel: L_foot_pitch, R_foot_pitch
  [19-20] foot_roll_pos (2d)     ← joint_pos_rel: L_foot_roll, R_foot_roll
  [21-32] joint_vel (12d)        ← all joint velocities (Isaac runtime order)
  [33-44] last_action (12d)      ← previous policy output
"""

import numpy as np
from typing import Dict, Optional
import yaml
import os

# Isaac runtime joint order (from training — verified via joint_order.txt)
ISAAC_JOINT_ORDER = [
    "L_hip_pitch", "R_hip_pitch",
    "L_hip_roll", "R_hip_roll",
    "L_hip_yaw", "R_hip_yaw",
    "L_knee", "R_knee",
    "L_foot_pitch", "R_foot_pitch",
    "L_foot_roll", "R_foot_roll",
]

# Alias for backward compatibility (policy_node.py, state_machine_node.py)
JOINT_ORDER = ISAAC_JOINT_ORDER

# Observation group joint order (from biped_env_cfg.py obs config)
# hip_pos uses regex [".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*"]
# Isaac resolves to: L_roll, R_roll, L_yaw, R_yaw, L_pitch, R_pitch
HIP_POS_ORDER = ["L_hip_roll", "R_hip_roll", "L_hip_yaw", "R_hip_yaw", "L_hip_pitch", "R_hip_pitch"]
KNEE_POS_ORDER = ["L_knee", "R_knee"]
FOOT_PITCH_ORDER = ["L_foot_pitch", "R_foot_pitch"]
FOOT_ROLL_ORDER = ["L_foot_roll", "R_foot_roll"]

DEFAULT_POSITIONS = {}
DEFAULT_GAINS = {}
JOINT_LIMITS = {}

def load_control_params(path_in=""):
    path = path_in if path_in else "/home/roy/biped_lower/deploy/biped_ws/src/biped_bringup/config/control_params.yaml"
    if not os.path.exists(path):
        raise RuntimeError(f"Fatal: Could not find control_params.yaml at {path}")
    
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
        
    if 'control_params' not in config or 'joints' not in config['control_params']:
        raise RuntimeError("Fatal: Invalid format in control_params.yaml")
        
    for name, params in config['control_params']['joints'].items():
        DEFAULT_POSITIONS[name] = float(params['default_pos'])
        DEFAULT_GAINS[name] = (float(params['kp']), float(params['kd']))
        JOINT_LIMITS[name] = (float(params['limit_min']), float(params['limit_max']))

# Action scale (from training config, per-joint)
ACTION_SCALE = 0.5
ACTION_SCALE_OVERRIDE = {
    "R_foot_roll": 0.25,
    "L_foot_roll": 0.25,
}

# Action output order from ONNX (must match training ALL_JOINTS with preserve_order=True)
ACTION_ORDER = [
    "R_hip_yaw", "R_hip_roll", "R_hip_pitch",
    "R_knee", "R_foot_pitch", "R_foot_roll",
    "L_hip_yaw", "L_hip_roll", "L_hip_pitch",
    "L_knee", "L_foot_pitch", "L_foot_roll",
]

# Default PD gains (from training config, halved Berkeley values)
# Deploy PD gains — Kp from training (V74), Kd 5× for hardware damping


class ObsBuilder:
    """Build 45d observation vector from sensor data."""

    def __init__(self):
        self._last_action = np.zeros(12, dtype=np.float32)

    def build(self,
              gyro: np.ndarray,             # (3,) rad/s body frame
              gravity: np.ndarray,          # (3,) m/s² body frame
              cmd_vel: np.ndarray,          # (3,) lin_x, lin_y, ang_z
              joint_positions: Dict[str, float],  # {name: rad}
              joint_velocities: Dict[str, float],  # {name: rad/s}
              ) -> np.ndarray:
        """Build observation vector. Returns (45,) float32 array."""

        obs = np.zeros(45, dtype=np.float32)

        # [0-2] base_ang_vel
        obs[0:3] = gyro

        # [3-5] projected_gravity
        # IMU nodes now output standardized Isaac convention:
        # upright -> (0, 0, -1.0)
        obs[3:6] = gravity

        # [6-8] velocity_commands
        obs[6:9] = cmd_vel

        # [9-14] hip_pos (relative to default)
        for i, name in enumerate(HIP_POS_ORDER):
            obs[9 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]

        # [15-16] knee_pos
        for i, name in enumerate(KNEE_POS_ORDER):
            obs[15 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]

        # [17-18] foot_pitch_pos
        for i, name in enumerate(FOOT_PITCH_ORDER):
            obs[17 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]

        # [19-20] foot_roll_pos
        for i, name in enumerate(FOOT_ROLL_ORDER):
            obs[19 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]

        # [21-32] joint_vel (Isaac runtime order)
        for i, name in enumerate(ISAAC_JOINT_ORDER):
            obs[21 + i] = joint_velocities.get(name, 0.0)

        # [33-44] last_action
        obs[33:45] = self._last_action

        return obs

    def update_last_action(self, action: np.ndarray):
        """Store action for next observation (before scaling)."""
        self._last_action = action.copy()

    @staticmethod
    def action_to_positions(action: np.ndarray) -> Dict[str, float]:
        """Convert policy output to joint position targets.

        target[i] = default_pos[i] + action[i] * ACTION_SCALE
        Action order matches training ALL_JOINTS (ONNX output order).
        """
        targets = {}
        for i, name in enumerate(ACTION_ORDER):
            scale = ACTION_SCALE_OVERRIDE.get(name, ACTION_SCALE)
            targets[name] = DEFAULT_POSITIONS[name] + float(action[i]) * scale
        return targets
