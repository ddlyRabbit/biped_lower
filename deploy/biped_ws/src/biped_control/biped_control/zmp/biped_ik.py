"""Inverse kinematics for biped using pinocchio — full 6-DOF.

Solves for all 6 joints per leg: position + flat foot orientation.
Uses world-aligned Jacobian (LOCAL_WORLD_ALIGNED) which converges
reliably for our URDF geometry.

Target orientation = foot orientation at default standing pose (flat on ground).
The full trajectory is pre-computed before execution (not real-time).
Requires: pip install pin
"""

import numpy as np
from pathlib import Path
from typing import Dict, Tuple

import pinocchio as pin


PIN_JOINT_TO_DEPLOY = {
    "left_hip_pitch_04": "L_hip_pitch",
    "left_hip_roll_03": "L_hip_roll",
    "left_hip_yaw_03": "L_hip_yaw",
    "left_knee_04": "L_knee",
    "left_foot_pitch_02": "L_foot_pitch",
    "left_foot_roll_02": "L_foot_roll",
    "right_hip_pitch_04": "R_hip_pitch",
    "right_hip_roll_03": "R_hip_roll",
    "right_hip_yaw_03": "R_hip_yaw",
    "right_knee_04": "R_knee",
    "right_foot_pitch_02": "R_foot_pitch",
    "right_foot_roll_02": "R_foot_roll",
}


class BipedIK:
    """6-DOF IK solver: foot position + flat orientation."""

    def __init__(self, urdf_path: str, dt: float = 0.1, max_iter: int = 150,
                 eps_pos: float = 5e-4, eps_ori: float = 1e-2, damping: float = 1e-3):
        self.urdf_path = str(Path(urdf_path).resolve())
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.dt = dt
        self.max_iter = max_iter
        self.eps_pos = eps_pos
        self.eps_ori = eps_ori
        self.damping = damping

        self.r_foot_id = self.model.getFrameId("foot_6061")
        self.l_foot_id = self.model.getFrameId("foot_6061_2")

        # Build index maps
        self._pin_to_deploy = {}
        self._deploy_to_pin_idx = {}
        self._r_indices = []
        self._l_indices = []

        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            idx = self.model.joints[i].idx_q
            deploy_name = PIN_JOINT_TO_DEPLOY.get(name)
            if deploy_name:
                self._pin_to_deploy[idx] = deploy_name
                self._deploy_to_pin_idx[deploy_name] = idx
                if name.startswith("right"):
                    self._r_indices.append(idx)
                else:
                    self._l_indices.append(idx)

        # Default config
        self._q_default = pin.neutral(self.model)
        defaults = {
            "R_hip_pitch": 0.08, "L_hip_pitch": -0.08,
            "R_knee": 0.25, "L_knee": 0.25,
            "R_foot_pitch": -0.17, "L_foot_pitch": -0.17,
        }
        for name, val in defaults.items():
            if name in self._deploy_to_pin_idx:
                self._q_default[self._deploy_to_pin_idx[name]] = val

        self._last_q = self._q_default.copy()

        # Compute standing foot positions and "flat" orientations
        pin.forwardKinematics(self.model, self.data, self._q_default)
        pin.updateFramePlacements(self.model, self.data)
        self.right_foot_standing = self.data.oMf[self.r_foot_id].translation.copy()
        self.left_foot_standing = self.data.oMf[self.l_foot_id].translation.copy()
        # Target orientation = standing flat on ground
        self._R_flat_right = self.data.oMf[self.r_foot_id].rotation.copy()
        self._R_flat_left = self.data.oMf[self.l_foot_id].rotation.copy()

    def _solve_one_foot(self, q: np.ndarray, foot_id: int,
                        target_pos: np.ndarray, R_flat: np.ndarray,
                        leg_indices: list) -> np.ndarray:
        """6-DOF CLIK: position + flat foot orientation.

        Uses world-aligned error formulation:
          pos_error = target_pos - current_pos
          ori_error = log3(R_target @ R_current.T)
        Jacobian: LOCAL_WORLD_ALIGNED (stable convergence).
        """
        q = q.copy()

        for it in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            # World-frame errors
            pos_err = target_pos - self.data.oMf[foot_id].translation
            ori_err = pin.log3(R_flat @ self.data.oMf[foot_id].rotation.T)
            err = np.concatenate([pos_err, ori_err])

            if np.linalg.norm(pos_err) < self.eps_pos and np.linalg.norm(ori_err) < self.eps_ori:
                break

            J_full = pin.computeFrameJacobian(
                self.model, self.data, q, foot_id, pin.LOCAL_WORLD_ALIGNED
            )
            J_leg = J_full[:, leg_indices]

            JJT = J_leg @ J_leg.T + self.damping * np.eye(6)
            dq_leg = J_leg.T @ np.linalg.solve(JJT, err)

            dq = np.zeros(self.model.nv)
            for i, idx in enumerate(leg_indices):
                dq[idx] = dq_leg[i]

            q = pin.integrate(self.model, q, dq * self.dt)

            # Clamp to joint limits
            for idx in leg_indices:
                lo = self.model.lowerPositionLimit[idx]
                hi = self.model.upperPositionLimit[idx]
                q[idx] = np.clip(q[idx], lo, hi)

        return q

    def solve(
        self,
        com_x: float,
        com_y: float,
        left_foot_pos: np.ndarray,
        right_foot_pos: np.ndarray,
    ) -> Dict[str, float]:
        """Solve IK for both legs. Foot positions in world frame."""
        # Foot targets relative to torso
        r_rel = np.array([
            right_foot_pos[0] - com_x,
            right_foot_pos[1] - com_y,
            self.right_foot_standing[2] + right_foot_pos[2],
        ])
        l_rel = np.array([
            left_foot_pos[0] - com_x,
            left_foot_pos[1] - com_y,
            self.left_foot_standing[2] + left_foot_pos[2],
        ])

        # Solve right leg (6-DOF: position + flat orientation)
        q = self._solve_one_foot(
            self._last_q, self.r_foot_id, r_rel, self._R_flat_right, self._r_indices
        )
        # Solve left leg
        q = self._solve_one_foot(
            q, self.l_foot_id, l_rel, self._R_flat_left, self._l_indices
        )

        self._last_q = q.copy()

        joints = {}
        for idx, name in self._pin_to_deploy.items():
            joints[name] = float(q[idx])
        return joints

    def forward_kinematics(self, joints: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        q = self._q_default.copy()
        for name, angle in joints.items():
            if name in self._deploy_to_pin_idx:
                q[self._deploy_to_pin_idx[name]] = angle
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return (self.data.oMf[self.r_foot_id].translation.copy(),
                self.data.oMf[self.l_foot_id].translation.copy())

    def reset(self):
        self._last_q = self._q_default.copy()
