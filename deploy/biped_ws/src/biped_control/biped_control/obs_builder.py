"""Observation vector builder — G1-style with history stacking.

Obs per frame (45d):
  [0-2]   base_ang_vel * 0.2   (3d)  ← IMU gyro (rad/s, body frame)
  [3-5]   projected_gravity    (3d)  ← IMU gravity normalized
  [6-8]   velocity_commands    (3d)  ← cmd_vel (lin_x, lin_y, ang_z)
  [9-20]  joint_pos_rel        (12d) ← joint positions - defaults (Isaac alphabetical)
  [21-32] joint_vel_rel * 0.05 (12d) ← joint velocities (Isaac alphabetical)
  [33-44] last_action          (12d) ← previous policy output (raw, before scaling)

History: 5 frames, newest first → total obs = 5 × 45 = 225
Action: scale=0.25, uniform across all joints, no tanh
"""

import numpy as np
from collections import deque
from typing import Dict


# Isaac alphabetical joint order (matches training with joint_names=[".*"])
JOINT_ORDER = [
    "L_foot_pitch", "L_foot_roll", "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee",
    "R_foot_pitch", "R_foot_roll", "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee",
]

# Backward compatibility — state_machine_node and policy_node use these names

# Default joint positions (from biped_env_cfg.py init_state)
DEFAULT_POSITIONS = {
    "L_foot_pitch": -0.17, "L_foot_roll": 0.0,
    "L_hip_pitch": -0.08, "L_hip_roll": 0.0, "L_hip_yaw": 0.0, "L_knee": 0.25,
    "R_foot_pitch": -0.17, "R_foot_roll": 0.0,
    "R_hip_pitch": 0.08, "R_hip_roll": 0.0, "R_hip_yaw": 0.0, "R_knee": 0.25,
}

# Action scale (uniform for all joints)
ACTION_SCALE = 0.25

# Deploy PD gains — matched to training (same Kp/Kd as sim)
DEFAULT_GAINS = {
    "L_hip_pitch": (180.0, 3.0), "R_hip_pitch": (180.0, 3.0),
    "L_hip_roll":  (180.0, 3.0), "R_hip_roll":  (180.0, 3.0),
    "L_hip_yaw":   (180.0, 3.0), "R_hip_yaw":   (180.0, 3.0),
    "L_knee":      (180.0, 3.0), "R_knee":      (180.0, 3.0),
    "L_foot_pitch": (120.0, 3.0), "R_foot_pitch": (120.0, 3.0),
    "L_foot_roll":  (120.0, 3.0), "R_foot_roll":  (120.0, 3.0),
}

FRAME_DIM = 45
HISTORY_LENGTH = 5
OBS_DIM = FRAME_DIM * HISTORY_LENGTH  # 225


class ObsBuilder:
    """Build 225d observation vector (5 frames × 45d) from sensor data."""

    def __init__(self):
        self._last_action = np.zeros(12, dtype=np.float32)
        # History buffer: deque of frames, newest appended last
        self._history: deque = deque(maxlen=HISTORY_LENGTH)
        # Initialize with zero frames
        for _ in range(HISTORY_LENGTH):
            self._history.append(np.zeros(FRAME_DIM, dtype=np.float32))

    def build(self,
              gyro: np.ndarray,             # (3,) rad/s body frame
              gravity: np.ndarray,          # (3,) m/s² body frame
              cmd_vel: np.ndarray,          # (3,) lin_x, lin_y, ang_z
              joint_positions: Dict[str, float],  # {name: rad}
              joint_velocities: Dict[str, float],  # {name: rad/s}
              ) -> np.ndarray:
        """Build observation vector. Returns (225,) float32 array."""

        frame = np.zeros(FRAME_DIM, dtype=np.float32)

        # [0-2] base_ang_vel * 0.2
        frame[0:3] = gyro * 0.2

        # [3-5] projected_gravity
        # BNO085 SH2_GRAVITY: upright → (0, 0, +9.81) (points up)
        # Isaac projected_gravity: upright → (0, 0, -1) (points down)
        g_norm = np.linalg.norm(gravity)
        if g_norm > 0.1:
            g_unit = gravity / g_norm
            frame[3:6] = [g_unit[0], g_unit[1], -g_unit[2]]
        else:
            frame[3:6] = [0.0, 0.0, -1.0]

        # [6-8] velocity_commands
        frame[6:9] = cmd_vel

        # [9-20] joint_pos_rel (Isaac alphabetical order)
        for i, name in enumerate(JOINT_ORDER):
            frame[9 + i] = joint_positions.get(name, 0.0) - DEFAULT_POSITIONS[name]

        # [21-32] joint_vel_rel * 0.05 (Isaac alphabetical order)
        for i, name in enumerate(JOINT_ORDER):
            frame[21 + i] = joint_velocities.get(name, 0.0) * 0.05

        # [33-44] last_action
        frame[33:45] = self._last_action

        # Append current frame to history
        self._history.append(frame)

        # Stack: newest first, oldest last
        obs = np.zeros(OBS_DIM, dtype=np.float32)
        for i, hist_frame in enumerate(reversed(self._history)):
            obs[i * FRAME_DIM : (i + 1) * FRAME_DIM] = hist_frame

        return obs

    def update_last_action(self, action: np.ndarray):
        """Store raw action (before scaling) for next observation."""
        self._last_action = action.copy()

    @staticmethod
    def action_to_positions(action: np.ndarray) -> Dict[str, float]:
        """Convert policy output to joint position targets.

        target[i] = default_pos[i] + action[i] * ACTION_SCALE
        Action order matches Isaac alphabetical (JOINT_ORDER).
        """
        targets = {}
        for i, name in enumerate(JOINT_ORDER):
            targets[name] = DEFAULT_POSITIONS[name] + float(action[i]) * ACTION_SCALE
        return targets
