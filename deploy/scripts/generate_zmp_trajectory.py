#!/usr/bin/env python3
"""
Generate ZMP trajectory CSV from config yaml for playback.
Implements fixes: IK warm-start, ZMP integral action, smooth double-support ZMP shift.
"""
import os
import math
import yaml
import argparse
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, Tuple, List
from scipy.linalg import solve_discrete_are
import pinocchio as pin

PIN_JOINT_TO_DEPLOY = {
    "L_hip_pitch_04": "L_hip_pitch",
    "L_hip_roll_03": "L_hip_roll",
    "L_hip_yaw_03": "L_hip_yaw",
    "L_knee_04": "L_knee",
    "L_foot_pitch_02": "L_foot_pitch",
    "L_foot_roll_02": "L_foot_roll",
    "R_hip_pitch_04": "R_hip_pitch",
    "R_hip_roll_03": "R_hip_roll",
    "R_hip_yaw_03": "R_hip_yaw",
    "R_knee_04": "R_knee",
    "R_foot_pitch_02": "R_foot_pitch",
    "R_foot_roll_02": "R_foot_roll",
}

class BipedIK:
    def __init__(self, urdf_path: str, dt: float = 0.1, max_iter: int = 200,
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

        self._q_default = pin.neutral(self.model)
        defaults = {
            "R_hip_pitch": 0.08, "L_hip_pitch": -0.08,
            "R_knee": 0.25, "L_knee": 0.25,
            "R_foot_pitch": -0.17, "L_foot_pitch": -0.17,
        }
        for name, val in defaults.items():
            if name in self._deploy_to_pin_idx:
                self._q_default[self._deploy_to_pin_idx[name]] = val

        pin.forwardKinematics(self.model, self.data, self._q_default)
        pin.updateFramePlacements(self.model, self.data)
        self.right_foot_standing = self.data.oMf[self.r_foot_id].translation.copy()
        self.left_foot_standing = self.data.oMf[self.l_foot_id].translation.copy()
        self._R_flat_right = self.data.oMf[self.r_foot_id].rotation.copy()
        self._R_flat_left = self.data.oMf[self.l_foot_id].rotation.copy()

        self._q_last = self._q_default.copy()

    def _solve_one_foot(self, foot_id: int, target_pos: np.ndarray,
                        R_flat: np.ndarray, leg_indices: list, q_init: np.ndarray) -> np.ndarray:
        q = q_init.copy()
        for it in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            pos_err = target_pos - self.data.oMf[foot_id].translation
            ori_err = pin.log3(R_flat @ self.data.oMf[foot_id].rotation.T)
            err = np.concatenate([pos_err, ori_err])
            if np.linalg.norm(pos_err) < self.eps_pos and np.linalg.norm(ori_err) < self.eps_ori:
                break
            J_full = pin.computeFrameJacobian(self.model, self.data, q, foot_id, pin.LOCAL_WORLD_ALIGNED)
            J_leg = J_full[:, leg_indices]
            JJT = J_leg @ J_leg.T + self.damping * np.eye(6)
            dq_leg = J_leg.T @ np.linalg.solve(JJT, err)
            dq = np.zeros(self.model.nv)
            for i, idx in enumerate(leg_indices):
                dq[idx] = dq_leg[i]
            q = pin.integrate(self.model, q, dq * self.dt)
            for idx in leg_indices:
                lo, hi = self.model.lowerPositionLimit[idx], self.model.upperPositionLimit[idx]
                q[idx] = np.clip(q[idx], lo, hi)
        return q

    def solve(self, base_x: float, base_y: float,
              left_foot_pos: np.ndarray, right_foot_pos: np.ndarray) -> Dict[str, float]:
        r_rel = np.array([
            right_foot_pos[0] - base_x,
            right_foot_pos[1] - base_y,
            self.right_foot_standing[2] + right_foot_pos[2],
        ])
        l_rel = np.array([
            left_foot_pos[0] - base_x,
            left_foot_pos[1] - base_y,
            self.left_foot_standing[2] + left_foot_pos[2],
        ])

        q_r = self._solve_one_foot(self.r_foot_id, r_rel, self._R_flat_right, self._r_indices, self._q_last)
        q_l = self._solve_one_foot(self.l_foot_id, l_rel, self._R_flat_left, self._l_indices, self._q_last)

        for idx in self._r_indices:
            self._q_last[idx] = q_r[idx]
        for idx in self._l_indices:
            self._q_last[idx] = q_l[idx]

        joints = {}
        for idx, name in self._pin_to_deploy.items():
            joints[name] = float(self._q_last[idx])
        return joints

    def compute_com(self, joints: Dict[str, float]) -> np.ndarray:
        q = self._q_default.copy()
        for name, angle in joints.items():
            if name in self._deploy_to_pin_idx:
                q[self._deploy_to_pin_idx[name]] = angle
        pin.forwardKinematics(self.model, self.data, q)
        return pin.centerOfMass(self.model, self.data, q).copy()

    def compute_com_default(self) -> np.ndarray:
        return self.compute_com({})

@dataclass
class FootstepPlan:
    zmp_ref_x: np.ndarray
    zmp_ref_y: np.ndarray
    left_foot: np.ndarray
    right_foot: np.ndarray
    support_foot: np.ndarray
    dt: float
    n_samples: int

class FootstepPlanner:
    def __init__(self, step_length=0.10, step_width=0.25, step_height=0.05,
                 step_period=0.8, num_steps=10, double_support_ratio=0.1, dt=0.02):
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
        self.step_period = step_period
        self.num_steps = num_steps
        self.ds_ratio = double_support_ratio
        self.dt = dt
        self.samples_per_step = int(step_period / dt)
        self.ds_samples = int(self.samples_per_step * double_support_ratio)
        self.ss_samples = self.samples_per_step - self.ds_samples

    def plan(self) -> FootstepPlan:
        n_phases = self.num_steps + 2
        n_total = n_phases * self.samples_per_step

        zmp_x = np.zeros(n_total)
        zmp_y = np.zeros(n_total)
        left_foot = np.zeros((n_total, 3))
        right_foot = np.zeros((n_total, 3))
        support = np.zeros(n_total, dtype=int)

        half_width = self.step_width / 2.0
        l_pos = np.array([0.0, half_width, 0.0])
        r_pos = np.array([0.0, -half_width, 0.0])

        s = 0
        e = self.samples_per_step
        zmp_x[s:e] = 0.0
        zmp_y[s:e] = 0.0
        left_foot[s:e] = l_pos
        right_foot[s:e] = r_pos
        support[s:e] = 0

        for step_idx in range(1, self.num_steps + 1):
            s = step_idx * self.samples_per_step
            e = s + self.samples_per_step

            left_is_swing = (step_idx % 2 == 1)
            forward = step_idx * self.step_length
            if step_idx == 1:
                swing_target_x = self.step_length * 0.5
            elif step_idx == self.num_steps:
                swing_target_x = forward - self.step_length * 0.5
            else:
                swing_target_x = forward

            if left_is_swing:
                swing_target = np.array([swing_target_x, half_width, 0.0])
                support_pos = r_pos.copy()
                prev_support_pos = l_pos.copy()
                support[s:e] = 2
            else:
                swing_target = np.array([swing_target_x, -half_width, 0.0])
                support_pos = l_pos.copy()
                prev_support_pos = r_pos.copy()
                support[s:e] = 1

            ds_half = self.ds_samples // 2
            for i in range(self.samples_per_step):
                if i < ds_half:
                    t_ds = (ds_half + i) / max(self.ds_samples, 1)
                    zmp_x[s + i] = prev_support_pos[0] + t_ds * (support_pos[0] - prev_support_pos[0])
                    zmp_y[s + i] = prev_support_pos[1] + t_ds * (support_pos[1] - prev_support_pos[1])
                    support[s + i] = 0
                elif i >= self.samples_per_step - ds_half:
                    t_ds = (i - (self.samples_per_step - ds_half)) / max(self.ds_samples, 1)
                    zmp_x[s + i] = support_pos[0] + t_ds * (swing_target[0] - support_pos[0])
                    zmp_y[s + i] = support_pos[1] + t_ds * (swing_target[1] - support_pos[1])
                    support[s + i] = 0
                else:
                    zmp_x[s + i] = support_pos[0]
                    zmp_y[s + i] = support_pos[1]

            if left_is_swing:
                swing_start = l_pos.copy()
                right_foot[s:e] = r_pos
                for i in range(self.samples_per_step):
                    if i < ds_half or i >= self.samples_per_step - ds_half:
                        left_foot[s + i] = swing_start if i < ds_half else swing_target
                    else:
                        ss_t = (i - ds_half) / self.ss_samples
                        xy = swing_start[:2] + ss_t * (swing_target[:2] - swing_start[:2])
                        z = 4 * self.step_height * ss_t * (1 - ss_t)
                        left_foot[s + i] = [xy[0], xy[1], z]
                l_pos = swing_target.copy()
            else:
                swing_start = r_pos.copy()
                left_foot[s:e] = l_pos
                for i in range(self.samples_per_step):
                    if i < ds_half or i >= self.samples_per_step - ds_half:
                        right_foot[s + i] = swing_start if i < ds_half else swing_target
                    else:
                        ss_t = (i - ds_half) / self.ss_samples
                        xy = swing_start[:2] + ss_t * (swing_target[:2] - swing_start[:2])
                        z = 4 * self.step_height * ss_t * (1 - ss_t)
                        right_foot[s + i] = [xy[0], xy[1], z]
                r_pos = swing_target.copy()

        s = (self.num_steps + 1) * self.samples_per_step
        e = n_total
        for i in range(self.samples_per_step):
            if i < ds_half:
                t_ds = (ds_half + i) / max(self.ds_samples, 1)
                prev_sup = support_pos
                center_x = (l_pos[0] + r_pos[0]) / 2.0
                zmp_x[s + i] = prev_sup[0] + t_ds * (center_x - prev_sup[0])
                zmp_y[s + i] = prev_sup[1] + t_ds * (0.0 - prev_sup[1])
            else:
                zmp_x[s + i] = (l_pos[0] + r_pos[0]) / 2.0
                zmp_y[s + i] = 0.0
        left_foot[s:e] = l_pos
        right_foot[s:e] = r_pos
        support[s:e] = 0

        return FootstepPlan(
            zmp_ref_x=zmp_x,
            zmp_ref_y=zmp_y,
            left_foot=left_foot,
            right_foot=right_foot,
            support_foot=support,
            dt=self.dt,
            n_samples=n_total,
        )

class ZMPWalker:
    def __init__(self, com_height=0.43, dt=0.02, preview_horizon=320,
                 q_weight=1.0, r_weight=1e-6, g=9.81):
        self.zc = com_height
        self.dt = dt
        self.N = preview_horizon
        self.q = q_weight
        self.r = r_weight
        self.g = g

        A = np.array([
            [1, dt, dt**2 / 2],
            [0, 1, dt],
            [0, 0, 1],
        ])
        B = np.array([[dt**3 / 6, dt**2 / 2, dt]]).T
        C = np.array([[1, 0, -self.zc / self.g]])

        self.A_aug = np.zeros((4, 4))
        self.A_aug[0, 0] = 1.0
        self.A_aug[0, 1:4] = C @ A
        self.A_aug[1:4, 1:4] = A
        
        self.B_aug = np.zeros((4, 1))
        self.B_aug[0, 0] = (C @ B)[0, 0]
        self.B_aug[1:4, 0] = B[:, 0]

        Q = np.zeros((4, 4))
        Q[0, 0] = self.q
        R = self.r * np.eye(1)

        P = solve_discrete_are(self.A_aug, self.B_aug, Q, R)

        tmp = np.linalg.inv(R + self.B_aug.T @ P @ self.B_aug) @ self.B_aug.T
        self.K_aug = tmp @ P @ self.A_aug

        Fs = []
        pre = tmp
        AcT = (self.A_aug - self.B_aug @ self.K_aug).T
        for _ in range(self.N):
            Fs.append((pre @ P[:, 0:1]).item())
            pre = pre @ AcT
        self.Fs = np.array(Fs)
        
        self.A = A
        self.B = B
        self.C = C

    def solve(self, zmp_ref: np.ndarray, x0=0.0, dx0=0.0, ddx0=0.0):
        X = np.array([x0, dx0, ddx0], dtype=np.float64).reshape(3, 1)
        n = len(zmp_ref)

        padded = np.append(zmp_ref, [zmp_ref[-1]] * (self.N - 1))

        com_pos = np.zeros(n)
        zmp_actual = np.zeros(n)
        sum_err = 0.0

        for i in range(n):
            zmp_curr = (self.C @ X)[0, 0]
            err = zmp_curr - zmp_ref[i]
            sum_err += err
            
            X_aug = np.zeros((4, 1))
            X_aug[0, 0] = sum_err
            X_aug[1:4, 0] = X[:, 0]

            preview_term = 0.0
            for j in range(self.N):
                preview_term += self.Fs[j] * (padded[i + j] - zmp_ref[i])

            U = -self.K_aug @ X_aug + preview_term
            X = self.A @ X + self.B * U[0, 0]

            com_pos[i] = X[0, 0]
            zmp_actual[i] = (self.C @ X)[0, 0]

        return com_pos, zmp_actual

    def generate(self, zmp_ref_x: np.ndarray, zmp_ref_y: np.ndarray):
        com_x, zmp_x = self.solve(zmp_ref_x)
        com_y, zmp_y = self.solve(zmp_ref_y)
        return com_x, com_y, zmp_x, zmp_y

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default='deploy/biped_ws/src/biped_bringup/config/zmp_config.yaml')
    parser.add_argument('--output', type=str, default='deploy/biped_ws/src/biped_bringup/config/trajectory.csv')
    args = parser.parse_args()

    cfg_path = os.path.abspath(args.config)
    out_path = os.path.abspath(args.output)
    
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)
        
    step_length = cfg.get('step_length', 0.05)
    step_width = cfg.get('step_width', 0.25)
    step_height = cfg.get('step_height', 0.03)
    step_period = cfg.get('step_period', 3.0)
    num_steps = int(cfg.get('num_steps', 6))
    ds_ratio = cfg.get('double_support_ratio', 0.5)
    com_height = cfg.get('com_height', 0.49)
    preview_horizon = int(cfg.get('preview_horizon', 320))
    dt = cfg.get('dt', 0.01)
    urdf_path = cfg.get('urdf_path', '/home/roy/biped_lower/urdf/light/robot.urdf')

    print(f"Generating trajectory:")
    print(f"  Steps: {num_steps}, Length: {step_length}m, Width: {step_width}m")
    print(f"  Period: {step_period}s, DS Ratio: {ds_ratio}, dt: {dt}s")

    planner = FootstepPlanner(
        step_length=step_length,
        step_width=step_width,
        step_height=step_height,
        step_period=step_period,
        num_steps=num_steps,
        double_support_ratio=ds_ratio,
        dt=dt,
    )
    plan = planner.plan()

    walker = ZMPWalker(
        com_height=com_height,
        dt=dt,
        preview_horizon=preview_horizon,
    )
    com_x, com_y, zmp_x, zmp_y = walker.generate(plan.zmp_ref_x, plan.zmp_ref_y)

    ik = BipedIK(urdf_path=urdf_path, dt=dt)
    com_default = ik.compute_com_default()

    trajectory = []
    com_offset = com_default.copy()

    for i in range(plan.n_samples):
        base_x = com_x[i] - com_offset[0]
        base_y = com_y[i] - com_offset[1]
        joints = ik.solve(base_x, base_y, plan.left_foot[i], plan.right_foot[i])
        trajectory.append(joints)
        com_offset = ik.compute_com(joints)

    ramp_ik_frames = int(1.0 / dt)
    ramp_joint_frames = int(1.0 / dt)
    end_com_x = com_x[-1]
    end_com_y = com_y[-1]
    end_l_foot = plan.left_foot[-1].copy()
    end_r_foot = plan.right_foot[-1].copy()
    stand_l_foot = np.array([0.0, end_l_foot[1], 0.0])
    stand_r_foot = np.array([0.0, end_r_foot[1], 0.0])
    center_x = (end_l_foot[0] + end_r_foot[0]) / 2.0

    for i in range(1, ramp_ik_frames + 1):
        alpha = i / ramp_ik_frames
        alpha = 0.5 * (1 - np.cos(np.pi * alpha))
        ramp_com_x = end_com_x + alpha * (center_x - end_com_x)
        ramp_com_y = end_com_y + alpha * (0.0 - end_com_y)
        ramp_l = end_l_foot + alpha * (stand_l_foot - end_l_foot)
        ramp_r = end_r_foot + alpha * (stand_r_foot - end_r_foot)
        
        ramp_base_x = ramp_com_x - com_offset[0]
        ramp_base_y = ramp_com_y - com_offset[1]
        
        joints = ik.solve(ramp_base_x, ramp_base_y, ramp_l, ramp_r)
        trajectory.append(joints)
        com_offset = ik.compute_com(joints)

    last_ik_joints = trajectory[-1]
    default_joints = {
        "R_hip_pitch": 0.08, "R_hip_roll": 0.0, "R_hip_yaw": 0.0,
        "R_knee": 0.25, "R_foot_pitch": -0.17, "R_foot_roll": 0.0,
        "L_hip_pitch": -0.08, "L_hip_roll": 0.0, "L_hip_yaw": 0.0,
        "L_knee": 0.25, "L_foot_pitch": -0.17, "L_foot_roll": 0.0,
    }

    for i in range(1, ramp_joint_frames + 1):
        alpha = i / ramp_joint_frames
        alpha = 0.5 * (1 - np.cos(np.pi * alpha))
        frame = {}
        for name in last_ik_joints:
            frame[name] = last_ik_joints[name] + alpha * (default_joints[name] - last_ik_joints[name])
        trajectory.append(frame)

    csv_joint_order = [
        "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_foot_pitch", "L_foot_roll",
        "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_foot_pitch", "R_foot_roll",
    ]
    
    n_total = len(trajectory)
    t_arr = np.arange(n_total) * dt
    
    csv_data = [t_arr.tolist()]
    for j_name in csv_joint_order:
        row = [frame[j_name] for frame in trajectory]
        csv_data.append(row)

    with open(out_path, 'w') as f:
        for row in csv_data:
            f.write(','.join([f"{v:.6f}" for v in row]) + '\n')
            
    print(f"Saved {n_total} frames ({t_arr[-1]:.2f}s) to {out_path}")

if __name__ == "__main__":
    main()
