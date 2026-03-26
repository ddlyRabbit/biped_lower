"""Inverse kinematics for biped using pinocchio CLIK.

Position-only IK using hip + knee (4 joints per leg).
Foot pitch set to compensate accumulated tilt (as flat as possible).
Foot roll set to 0.

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
    """IK solver using pinocchio CLIK.

    Strategy:
    1. Solve hip + knee (4 joints) for foot position
    2. Set foot_pitch to maximize foot flatness (compensate chain tilt)
    3. Set foot_roll to 0
    """

    def __init__(self, urdf_path: str, dt: float = 0.1, max_iter: int = 150,
                 eps: float = 5e-4, damping: float = 1e-3):
        self.urdf_path = str(Path(urdf_path).resolve())
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.dt = dt
        self.max_iter = max_iter
        self.eps = eps
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

        # Standing foot positions
        pin.forwardKinematics(self.model, self.data, self._q_default)
        pin.updateFramePlacements(self.model, self.data)
        self.right_foot_standing = self.data.oMf[self.r_foot_id].translation.copy()
        self.left_foot_standing = self.data.oMf[self.l_foot_id].translation.copy()

    def _solve_one_foot(self, q: np.ndarray, foot_id: int,
                        target_pos: np.ndarray, leg_indices: list) -> np.ndarray:
        """Position-only CLIK for hip + knee, then flatten foot.

        1. Solve position with hip_pitch, hip_roll, hip_yaw, knee (4 joints)
        2. Search foot_pitch for flattest foot orientation
        3. Set foot_roll = 0
        """
        q = q.copy()
        solve_indices = leg_indices[:4]  # hip_pitch, hip_roll, hip_yaw, knee

        # Step 1: Position IK with 4 joints
        for it in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pos = self.data.oMf[foot_id].translation
            err = target_pos - current_pos

            if np.linalg.norm(err) < self.eps:
                break

            J_full = pin.computeFrameJacobian(
                self.model, self.data, q, foot_id, pin.LOCAL_WORLD_ALIGNED
            )[:3, :]

            J_leg = J_full[:, solve_indices]

            JJT = J_leg @ J_leg.T + self.damping * np.eye(3)
            dq_leg = J_leg.T @ np.linalg.solve(JJT, err)

            dq = np.zeros(self.model.nv)
            for i, idx in enumerate(solve_indices):
                dq[idx] = dq_leg[i]

            q = pin.integrate(self.model, q, dq * self.dt)

            for idx in solve_indices:
                lo = self.model.lowerPositionLimit[idx]
                hi = self.model.upperPositionLimit[idx]
                q[idx] = np.clip(q[idx], lo, hi)

        # Step 2: Find foot_pitch that makes foot flattest
        fp_idx = leg_indices[4]  # foot_pitch index
        fr_idx = leg_indices[5]  # foot_roll index

        fp_lo = self.model.lowerPositionLimit[fp_idx]
        fp_hi = self.model.upperPositionLimit[fp_idx]

        best_fp = 0.0
        best_z_up = -1.0

        for fp in np.linspace(fp_lo, fp_hi, 50):
            q[fp_idx] = fp
            q[fr_idx] = 0.0
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            # Foot Z axis dot world Z — 1.0 = perfectly flat
            z_up = self.data.oMf[foot_id].rotation[2, 2]
            if z_up > best_z_up:
                best_z_up = z_up
                best_fp = fp

        q[fp_idx] = best_fp
        q[fr_idx] = 0.0

        return q

    def solve(
        self,
        com_x: float,
        com_y: float,
        left_foot_pos: np.ndarray,
        right_foot_pos: np.ndarray,
    ) -> Dict[str, float]:
        """Solve IK for both legs."""
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

        q = self._solve_one_foot(self._last_q, self.r_foot_id, r_rel, self._r_indices)
        q = self._solve_one_foot(q, self.l_foot_id, l_rel, self._l_indices)

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
