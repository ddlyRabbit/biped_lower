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

# Default joint positions (+X forward, from biped_env_cfg.py)
DEFAULT_POSITIONS = {
    "L_hip_pitch": -0.08, "R_hip_pitch":  0.08,
    "L_hip_roll":   0.0,  "R_hip_roll":   0.0,
    "L_hip_yaw":    0.0,  "R_hip_yaw":    0.0,
    "L_knee":       0.25, "R_knee":       0.25,
    "L_foot_pitch": -0.17,"R_foot_pitch": -0.17,
    "L_foot_roll":  0.0,  "R_foot_roll":  0.0,
}

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
DEFAULT_GAINS = {
    "L_hip_pitch": (180.0, 3.0), "R_hip_pitch": (180.0, 3.0),
    "L_hip_roll":  (180.0, 3.0), "R_hip_roll":  (180.0, 3.0),
    "L_hip_yaw":   (180.0, 3.0), "R_hip_yaw":   (180.0, 3.0),
    "L_knee":      (180.0, 3.0), "R_knee":      (180.0, 3.0),
    "L_foot_pitch": (30.0, 1.0), "R_foot_pitch": (30.0, 1.0),
    "L_foot_roll":  (30.0, 1.0), "R_foot_roll":  (30.0, 1.0),
}


class ObsBuilder:
    """Build 45d observation vector from sensor data."""

    def __init__(self):
        self._last_action = np.zeros(12, dtype=np.float32)

    def build(self,
              gyro: np.ndarray,             # (3,) rad/s body frame
              gravity: np.ndarray,          # (3,) m/s² body frame
              cmd_vel: np.ndarray,          # (3,) lin_x, lin_y, ang_z
              joint_positions: dict,  # {name: rad}
              joint_velocities: dict,  # {name: rad/s}
              ) -> np.ndarray:
        """Build observation vector. Returns (obs_dim,) float32 array."""

        obs = np.zeros(self.obs_dim, dtype=np.float32)

        # [0-2] base_ang_vel
        obs[0:3] = gyro

        # [3-5] projected_gravity
        g_norm = np.linalg.norm(gravity)
        if g_norm > 0.1:
            g_unit = gravity / g_norm
            obs[3:6] = [g_unit[0], g_unit[1], -g_unit[2]]
        else:
            obs[3:6] = [0.0, 0.0, -1.0]

        # [6-8] velocity_commands
        obs[6:9] = cmd_vel

        # Flatten relative pos/vel for history buffers
        pos_rel = np.array([joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name] for name in ISAAC_JOINT_ORDER], dtype=np.float32)
        vel_abs = np.array([joint_velocities.get(name, 0.0) for name in ISAAC_JOINT_ORDER], dtype=np.float32)
        act_abs = self.last_actions.copy()

        if self.obs_dim == 729:
            # 20 frames history
            self.update_history(pos_rel, vel_abs, act_abs)
            
            # Indices for ISAAC_JOINT_ORDER
            hip_idx = [2, 3, 4, 5, 0, 1] # roll, yaw, pitch (L_roll, R_roll, L_yaw, R_yaw, L_pitch, R_pitch)
            knee_idx = [6, 7]
            fp_idx = [8, 9]
            fr_idx = [10, 11]
            
            idx = 9
            obs[idx:idx+120] = self.flatten_history(self.hist_pos, hip_idx); idx += 120
            obs[idx:idx+40] = self.flatten_history(self.hist_pos, knee_idx); idx += 40
            obs[idx:idx+40] = self.flatten_history(self.hist_pos, fp_idx); idx += 40
            obs[idx:idx+40] = self.flatten_history(self.hist_pos, fr_idx); idx += 40
            obs[idx:idx+240] = self.flatten_history(self.hist_vel, list(range(12))); idx += 240
            obs[idx:idx+240] = self.flatten_history(self.hist_act, list(range(12))); idx += 240
        else:
            # 45d legacy mode
            for i, name in enumerate(HIP_POS_ORDER):
                obs[9 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]
            for i, name in enumerate(KNEE_POS_ORDER):
                obs[15 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]
            for i, name in enumerate(FOOT_PITCH_ORDER):
                obs[17 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]
            for i, name in enumerate(FOOT_ROLL_ORDER):
                obs[19 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]
            for i, name in enumerate(ISAAC_JOINT_ORDER):
                obs[21 + i] = joint_velocities.get(name, 0.0)
            for i in range(12):
                obs[33 + i] = self.last_actions[i]

        return obs


