"""Inverse kinematics for biped using ikpy + URDF.

Uses ikpy with strict joint bounds to prevent flipped configurations.
Each leg is an independent 6-DOF serial chain from torso to foot.

The full trajectory is pre-computed before execution (not real-time).
"""

import numpy as np
import ikpy.chain
import ikpy.link
from pathlib import Path
from typing import Dict, Tuple


RIGHT_NAMES = ["R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_foot_pitch", "R_foot_roll"]
LEFT_NAMES = ["L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_foot_pitch", "L_foot_roll"]


class BipedIK:
    """IK solver using ikpy with strict joint bounds per leg."""

    def __init__(self, urdf_path: str):
        self.urdf_path = str(Path(urdf_path).resolve())

        # Build chains from URDF
        self._r_chain = ikpy.chain.Chain.from_urdf_file(
            self.urdf_path,
            base_elements=["assy_formfg___kd_b_102b_torso_btm", "right_hip_pitch_04"],
            name="right_leg",
        )
        self._l_chain = ikpy.chain.Chain.from_urdf_file(
            self.urdf_path,
            base_elements=["assy_formfg___kd_b_102b_torso_btm", "left_hip_pitch_04"],
            name="left_leg",
        )

        # Override ikpy's bounds with tight walking-range limits
        # These prevent the solver from finding flipped configurations.
        # Format: (lower, upper) for each link [base(fixed), hp, hr, hy, kn, fp, fr]
        # Right leg
        self._r_bounds = [
            None,                     # base (fixed)
            (-0.5, 0.8),             # R hip pitch: small range for walking
            (-0.3, 0.3),             # R hip roll: small lateral
            (-0.3, 0.3),             # R hip yaw: small rotation
            (0.0, 1.2),              # R knee: only flexion, moderate range
            (-0.5, 0.3),             # R foot pitch
            (-0.2, 0.2),             # R foot roll
        ]
        # Left leg (mirrored pitch)
        self._l_bounds = [
            None,
            (-0.8, 0.5),             # L hip pitch (mirrored)
            (-0.3, 0.3),
            (-0.3, 0.3),
            (0.0, 1.2),
            (-0.5, 0.3),
            (-0.2, 0.2),
        ]

        # Apply bounds to chains
        self._apply_bounds(self._r_chain, self._r_bounds)
        self._apply_bounds(self._l_chain, self._l_bounds)

        # Default angles: [base, hp, hr, hy, kn, fp, fr]
        self._r_default = np.array([0, 0.08, 0, 0, 0.25, -0.17, 0])
        self._l_default = np.array([0, -0.08, 0, 0, 0.25, -0.17, 0])

        # Warm-start
        self._last_r = self._r_default.copy()
        self._last_l = self._l_default.copy()

        # Standing foot positions
        r_fk = self._r_chain.forward_kinematics(self._r_default)
        l_fk = self._l_chain.forward_kinematics(self._l_default)
        self.right_foot_standing = r_fk[:3, 3].copy()
        self.left_foot_standing = l_fk[:3, 3].copy()

    def _apply_bounds(self, chain, bounds):
        """Override joint bounds on an ikpy chain."""
        for i, link in enumerate(chain.links):
            if bounds[i] is not None and hasattr(link, 'bounds'):
                link.bounds = bounds[i]

    def solve(
        self,
        com_x: float,
        com_y: float,
        left_foot_pos: np.ndarray,
        right_foot_pos: np.ndarray,
    ) -> Dict[str, float]:
        """Solve IK for both legs independently.

        Foot positions in world frame → torso-relative → IK.
        """
        # Right foot relative to torso (torso XY ≈ CoM XY)
        r_rel = right_foot_pos.copy()
        r_rel[0] -= com_x
        r_rel[1] -= com_y

        q_r = self._r_chain.inverse_kinematics(
            r_rel, initial_position=self._last_r,
        )
        self._last_r = q_r.copy()

        # Left foot relative to torso
        l_rel = left_foot_pos.copy()
        l_rel[0] -= com_x
        l_rel[1] -= com_y

        q_l = self._l_chain.inverse_kinematics(
            l_rel, initial_position=self._last_l,
        )
        self._last_l = q_l.copy()

        # Build output (skip base link at index 0)
        joints = {}
        for i, name in enumerate(RIGHT_NAMES):
            joints[name] = float(q_r[i + 1])
        for i, name in enumerate(LEFT_NAMES):
            joints[name] = float(q_l[i + 1])

        return joints

    def forward_kinematics(self, joints: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        """Compute foot positions (torso-relative)."""
        q_r = np.array([0] + [joints.get(n, 0) for n in RIGHT_NAMES])
        q_l = np.array([0] + [joints.get(n, 0) for n in LEFT_NAMES])
        r_fk = self._r_chain.forward_kinematics(q_r)[:3, 3]
        l_fk = self._l_chain.forward_kinematics(q_l)[:3, 3]
        return r_fk, l_fk

    def reset(self):
        """Reset warm-start caches."""
        self._last_r = self._r_default.copy()
        self._last_l = self._l_default.copy()
