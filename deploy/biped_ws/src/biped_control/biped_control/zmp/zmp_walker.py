"""ZMP-based walking pattern generator.

Implements Kajita's preview control for CoM trajectory generation.
Reference: "Introduction to Humanoid Robotics" by Shuuji Kajita.

Given desired footstep positions → generates ZMP reference → solves for
CoM trajectory via discrete LQR preview control.
"""

import numpy as np
from scipy.linalg import solve_discrete_are


class ZMPWalker:
    """ZMP preview controller for biped CoM trajectory generation."""

    def __init__(
        self,
        com_height: float = 0.43,
        dt: float = 0.02,
        preview_horizon: int = 320,
        q_weight: float = 1.0,
        r_weight: float = 1e-6,
        g: float = 9.81,
    ):
        """
        Args:
            com_height: Height of CoM above ground (meters). From URDF FK.
            dt: Control timestep (seconds). 0.02 = 50Hz.
            preview_horizon: Number of look-ahead samples (N).
            q_weight: ZMP tracking weight in cost function.
            r_weight: Control effort weight in cost function.
            g: Gravitational acceleration.
        """
        self.zc = com_height
        self.dt = dt
        self.N = preview_horizon
        self.q = q_weight
        self.r = r_weight
        self.g = g

        # Build state-space system and controller
        self.A, self.B, self.C = self._create_system()
        self.K, self.Fs = self._create_controller()

    def _create_system(self):
        """Discrete-time linear inverted pendulum model.

        State: [x, dx, ddx]  (position, velocity, acceleration)
        Output: ZMP = x - (zc/g) * ddx
        """
        dt = self.dt
        A = np.array([
            [1, dt, dt**2 / 2],
            [0, 1, dt],
            [0, 0, 1],
        ])
        B = np.array([[dt**3 / 6, dt**2 / 2, dt]]).T
        C = np.array([[1, 0, -self.zc / self.g]])
        return A, B, C

    def _create_controller(self):
        """Solve discrete Riccati equation for preview control gains."""
        R = self.r * np.eye(1)
        Q = self.q * self.C.T @ self.C
        P = solve_discrete_are(self.A, self.B, Q, R)

        tmp = np.linalg.inv(R + self.B.T @ P @ self.B) @ self.B.T
        K = tmp @ P @ self.A

        # Preview gains
        Fs = []
        pre = np.copy(tmp)
        AcT = (self.A - self.B @ K).T
        for _ in range(self.N):
            Fs.append(pre @ self.C.T * self.q)
            pre = pre @ AcT
        Fs = np.array(Fs).flatten()

        return K, Fs

    def solve(self, zmp_ref: np.ndarray, x0: float = 0.0,
              dx0: float = 0.0, ddx0: float = 0.0):
        """Solve for CoM trajectory given ZMP reference.

        Args:
            zmp_ref: (N_samples,) array of desired ZMP positions (1D, either x or y).
            x0, dx0, ddx0: Initial state.

        Returns:
            com_pos: (N_samples,) CoM positions.
            zmp_actual: (N_samples,) Realized ZMP positions.
        """
        X = np.array([x0, dx0, ddx0], dtype=np.float64)
        n = len(zmp_ref)

        # Pad reference for preview horizon
        padded = np.append(zmp_ref, [zmp_ref[-1]] * (self.N - 1))

        com_pos = np.zeros(n)
        zmp_actual = np.zeros(n)

        for i in range(n):
            U = -self.K @ X + self.Fs @ padded[i:i + self.N]
            X = self.A @ X + self.B.flatten() * U.item()
            com_pos[i] = X[0]
            zmp_actual[i] = (self.C @ X).item()

        return com_pos, zmp_actual

    def generate(self, zmp_ref_x: np.ndarray, zmp_ref_y: np.ndarray):
        """Generate 2D CoM trajectory from X and Y ZMP references.

        Args:
            zmp_ref_x: (N,) ZMP reference in X (forward).
            zmp_ref_y: (N,) ZMP reference in Y (lateral).

        Returns:
            com_x, com_y: CoM trajectories.
            zmp_x, zmp_y: Realized ZMP trajectories.
        """
        com_x, zmp_x = self.solve(zmp_ref_x)
        com_y, zmp_y = self.solve(zmp_ref_y)
        return com_x, com_y, zmp_x, zmp_y
