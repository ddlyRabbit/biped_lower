#!/usr/bin/env python3
"""Sim2sim: Stand the biped at default joint positions in MuJoCo.

Usage:
    python sim2sim/stand_mujoco.py [--urdf light] [--headless] [--duration 10]

PD control in Python, torques sent to MuJoCo motor actuators.
"""

import argparse
import os
import time

import numpy as np
import mujoco
import mujoco.viewer

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ─── Physics ───────────────────────────────────────────────────────────────────
POLICY_DT = 0.02           # 50 Hz

# ─── Joint maps ───────────────────────────────────────────────────────────────
MJ_JOINTS = [
    "left_hip_pitch_04", "left_hip_roll_03", "left_hip_yaw_03", "left_knee_04", "left_foot_pitch_02", "left_foot_roll_02",
    "right_hip_pitch_04", "right_hip_roll_03", "right_hip_yaw_03", "right_knee_04", "right_foot_pitch_02", "right_foot_roll_02"
]

MJ_TO_ISAAC_NAME = {
    "left_hip_pitch_04": "L_hip_pitch", "left_hip_roll_03": "L_hip_roll", "left_hip_yaw_03": "L_hip_yaw",
    "left_knee_04": "L_knee", "left_foot_pitch_02": "L_foot_pitch", "left_foot_roll_02": "L_foot_roll",
    "right_hip_pitch_04": "R_hip_pitch", "right_hip_roll_03": "R_hip_roll", "right_hip_yaw_03": "R_hip_yaw",
    "right_knee_04": "R_knee", "right_foot_pitch_02": "R_foot_pitch", "right_foot_roll_02": "R_foot_roll"
}

# ─── Default standing pose (from Isaac biped_env_cfg) ─────────────────────────
DEFAULT_POS_ISAAC = {
    "L_hip_pitch": -0.08, "L_hip_roll": 0.0, "L_hip_yaw": 0.0, "L_knee": 0.25, "L_foot_pitch": -0.17, "L_foot_roll": 0.0,
    "R_hip_pitch": -0.08, "R_hip_roll": 0.0, "R_hip_yaw": 0.0, "R_knee": 0.25, "R_foot_pitch": -0.17, "R_foot_roll": 0.0
}
DEFAULT_POS_MJ = np.array([DEFAULT_POS_ISAAC[MJ_TO_ISAAC_NAME[mj]] for mj in MJ_JOINTS], dtype=np.float32)

# ─── PD gains (from play_mujoco.py) ──────────────────────────────────────────
KP_ISAAC = {"L_hip_pitch": 180, "R_hip_pitch": 180, "L_hip_roll": 180, "R_hip_roll": 180,
            "L_hip_yaw": 180, "R_hip_yaw": 180, "L_knee": 180, "R_knee": 180,
            "L_foot_pitch": 30, "R_foot_pitch": 30, "L_foot_roll": 30, "R_foot_roll": 30}
KD_ISAAC = {"L_hip_pitch": 6.5, "R_hip_pitch": 6.5, "L_hip_roll": 6.5, "R_hip_roll": 6.5,
            "L_hip_yaw": 3.0, "R_hip_yaw": 3.0, "L_knee": 6.5, "R_knee": 6.5,
            "L_foot_pitch": 1.0, "R_foot_pitch": 1.0, "L_foot_roll": 1.0, "R_foot_roll": 1.0}

KP_MJ = np.array([KP_ISAAC[MJ_TO_ISAAC_NAME[mj]] for mj in MJ_JOINTS], dtype=np.float32)
KD_MJ = np.array([KD_ISAAC[MJ_TO_ISAAC_NAME[mj]] for mj in MJ_JOINTS], dtype=np.float32)
EFFORT_MJ = np.array([100, 50, 50, 100, 30, 30, 100, 50, 50, 100, 30, 30], dtype=np.float32)

BASE_HEIGHT = 0.802

def main():
    parser = argparse.ArgumentParser(description="Biped standing in MuJoCo")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--urdf", choices=["heavy", "light"], default="light")
    args = parser.parse_args()

    mjcf_path = os.path.join(REPO_ROOT, "mjcf", "sim2sim", f"robot_{args.urdf}.mjcf")
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    assert model.nu == 12

    PHYSICS_DT = model.opt.timestep
    SUBSTEPS = int(round(POLICY_DT / PHYSICS_DT))

    qp_idx = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in MJ_JOINTS])
    qv_idx = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in MJ_JOINTS])
    
    # In C API it's jnt_qposadr and jnt_dofadr
    qp_idx = np.array([model.jnt_qposadr[i] for i in qp_idx])
    qv_idx = np.array([model.jnt_dofadr[i] for i in qv_idx])
    
    actuator_idx = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in MJ_JOINTS])

    mujoco.mj_resetData(model, data)
    data.qpos[2] = BASE_HEIGHT
    data.qpos[qp_idx] = DEFAULT_POS_MJ
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    total_mass = sum(model.body_mass[i] for i in range(model.nbody))
    print(f"MJCF: {mjcf_path}")
    print(f"Mass: {total_mass:.1f}kg, Spawn z: {BASE_HEIGHT}")
    print(f"Physics: {1/PHYSICS_DT:.0f}Hz ({SUBSTEPS} substeps), PD: {1/POLICY_DT:.0f}Hz")

    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(model, data)

    n_steps = int(args.duration / POLICY_DT)
    try:
        for step in range(n_steps):
            t0 = time.perf_counter()
            
            # Step physics
            for _ in range(SUBSTEPS):
                jp = data.qpos[qp_idx]
                jv = data.qvel[qv_idx]
                torques = KP_MJ * (DEFAULT_POS_MJ - jp) + KD_MJ * (0.0 - jv)
                torques = np.clip(torques, -EFFORT_MJ, EFFORT_MJ)
                data.ctrl[actuator_idx] = torques
                mujoco.mj_step(model, data)

            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break

            dt = time.perf_counter() - t0
            if POLICY_DT - dt > 0:
                time.sleep(POLICY_DT - dt)

            if step % 50 == 0:
                print(f"[{step * POLICY_DT:.1f}s] z={data.qpos[2]:.3f} qw={data.qpos[3]:.3f} con={data.ncon}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if viewer is not None:
            viewer.close()

if __name__ == "__main__":
    main()
