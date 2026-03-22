#!/usr/bin/env python3
"""Motor System Identification — Isaac Sim version.

Runs the same step/sine tests as motor_sysid.py but in Isaac Sim.
Robot is suspended (gravity disabled), one joint actuated at a time.

Usage:
    # Test one motor type with current actuator config
    python motor_sysid_isaac.py --joint R_hip_pitch --kp 250 --kd 5

    # All 3 motor types
    python motor_sysid_isaac.py --all

    # Different actuator model
    python motor_sysid_isaac.py --joint R_hip_pitch --actuator implicit

Output matches motor_sysid.py format for direct comparison.
"""

import argparse
import csv
import math
import os
import sys

sys.path.insert(0, "/workspace/biped_locomotion")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Motor SysID — Isaac Sim")
parser.add_argument("--joint", type=str, help="Joint name (e.g. R_hip_pitch)")
parser.add_argument("--all", action="store_true")
parser.add_argument("--kp", type=float, default=None)
parser.add_argument("--kd", type=float, default=None)
parser.add_argument("--output", type=str, default="/results/motor_sysid_isaac")
parser.add_argument("--actuator", type=str, default="delayed",
                    choices=["implicit", "delayed"], help="Actuator model")
parser.add_argument("--urdf", type=str, default="light", choices=["heavy", "light"])
parser.add_argument("--headless", action="store_true", default=True)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ─── Post-launch imports ────────────────────────────────────────────────────

import torch
import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.actuators import ImplicitActuatorCfg, DelayedPDActuatorCfg

from biped_env_cfg import (
    BipedFlatEnvCfg, URDF_HEAVY, URDF_LIGHT,
    ALL_JOINTS,
)

# ─── Motor type config ──────────────────────────────────────────────────────

MOTOR_TYPE_GAINS = {
    "RS04": {"kp": 250, "kd": 5, "effort": 60},
    "RS03": {"kp": 150, "kd": 5, "effort": 40},
    "RS02": {"kp": 40,  "kd": 5, "effort": 14},
}

JOINT_TO_MOTOR_TYPE = {
    "right_hip_pitch_04": "RS04", "left_hip_pitch_04": "RS04",
    "right_hip_roll_03": "RS03",  "left_hip_roll_03": "RS03",
    "right_hip_yaw_03": "RS03",   "left_hip_yaw_03": "RS03",
    "right_knee_04": "RS04",      "left_knee_04": "RS04",
    "right_foot_pitch_02": "RS02","left_foot_pitch_02": "RS02",
    "right_foot_roll_02": "RS02", "left_foot_roll_02": "RS02",
}

# Map friendly names to URDF joint names
FRIENDLY_TO_URDF = {
    "R_hip_pitch": "right_hip_pitch_04",
    "R_hip_roll": "right_hip_roll_03",
    "R_hip_yaw": "right_hip_yaw_03",
    "R_knee": "right_knee_04",
    "R_foot_pitch": "right_foot_pitch_02",
    "R_foot_roll": "right_foot_roll_02",
    "L_hip_pitch": "left_hip_pitch_04",
    "L_hip_roll": "left_hip_roll_03",
    "L_hip_yaw": "left_hip_yaw_03",
    "L_knee": "left_knee_04",
    "L_foot_pitch": "left_foot_pitch_02",
    "L_foot_roll": "left_foot_roll_02",
}

DEFAULT_TEST_JOINTS = {
    "RS04": "R_hip_pitch",
    "RS03": "R_hip_roll",
    "RS02": "R_foot_pitch",
}

JOINT_LIMITS_70PCT = {
    "R_hip_pitch":  (-1.55, 0.73),
    "R_hip_roll":   (-1.59, 0.15),
    "R_hip_yaw":    (-1.10, 1.10),
    "R_knee":       (0.0,   1.89),
    "R_foot_pitch": (-0.61, 0.37),
    "R_foot_roll":  (-0.18, 0.18),
}

SIM_DT = 0.005  # 200Hz sim (decimation=1 for max resolution)
SINE_FREQUENCIES = [0.5, 1.0, 2.0, 5.0, 10.0]


def save_csv(data: list[dict], path: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "cmd_pos", "pos", "vel", "torque", "temp"])
        writer.writeheader()
        writer.writerows(data)
    print(f"  Saved {path} ({len(data)} rows)")


def create_sim_context():
    """Create simulation context with high-rate physics."""
    sim_cfg = SimulationCfg(
        dt=SIM_DT,
        render_interval=1,
        gravity=(0.0, 0.0, 0.0),  # Gravity disabled — robot suspended
        device="cuda:0",
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.5])
    return sim


def create_robot(sim: SimulationContext, urdf: str, actuator_type: str,
                 test_joint: str, kp: float, kd: float):
    """Create robot with specified actuator model. Only test joint has Kp/Kd, others = 0."""
    urdf_path = URDF_LIGHT if urdf == "light" else URDF_HEAVY
    urdf_joint = FRIENDLY_TO_URDF[test_joint]
    motor_type = JOINT_TO_MOTOR_TYPE[urdf_joint]
    gains = MOTOR_TYPE_GAINS[motor_type]

    # Build per-joint stiffness/damping dicts
    stiffness_dict = {}
    damping_dict = {}
    for jname in ALL_JOINTS:
        if jname == urdf_joint:
            stiffness_dict[jname] = kp if kp else gains["kp"]
            damping_dict[jname] = kd if kd else gains["kd"]
        else:
            stiffness_dict[jname] = 0.0
            damping_dict[jname] = 0.0

    if actuator_type == "implicit":
        actuator_cfg = {
            "all_joints": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit=100.0,
                velocity_limit=20.0,
                stiffness=stiffness_dict,
                damping=damping_dict,
                armature=0.01,
            ),
        }
    else:  # delayed
        actuator_cfg = {
            "all_joints": DelayedPDActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit=100.0,
                velocity_limit=20.0,
                stiffness=stiffness_dict,
                damping=damping_dict,
                armature=0.01,
                friction=0.5,
                min_delay=1,
                max_delay=3,
            ),
        }

    robot_cfg = ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=urdf_path,
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                fix_root_link=True,  # Fixed base — suspended robot
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),
            joint_pos={".*": 0.0},
            joint_vel={".*": 0.0},
        ),
        actuators=actuator_cfg,
    )

    robot = Articulation(robot_cfg)
    return robot


def get_joint_index(robot: Articulation, joint_name: str) -> int:
    """Get index of joint in robot's joint array."""
    urdf_name = FRIENDLY_TO_URDF[joint_name]
    names = robot.data.joint_names
    for i, name in enumerate(names):
        if name == urdf_name:
            return i
    raise ValueError(f"Joint {urdf_name} not found in {names}")


def run_step_response(robot: Articulation, sim: SimulationContext,
                      joint_idx: int, target: float, duration: float = 2.0,
                      ramp_time: float = 0.1) -> list[dict]:
    """Run step response and record."""
    data = []
    n_joints = robot.num_joints
    t = 0.0

    # Phase 1: Hold at 0 (0.5s)
    for _ in range(int(0.5 / SIM_DT)):
        actions = torch.zeros(1, n_joints, device="cuda:0")
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        torque = robot.data.applied_torque[0, joint_idx].item() if hasattr(robot.data, 'applied_torque') else 0.0
        data.append({"time": t, "cmd_pos": 0.0, "pos": pos, "vel": vel, "torque": torque, "temp": 0})
        t += SIM_DT

    # Phase 2: Ramp to target
    n_ramp = int(ramp_time / SIM_DT)
    for i in range(n_ramp):
        progress = (i + 1) / n_ramp
        cmd = target * progress
        actions = torch.zeros(1, n_joints, device="cuda:0")
        actions[0, joint_idx] = cmd
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        data.append({"time": t, "cmd_pos": cmd, "pos": pos, "vel": vel, "torque": 0, "temp": 0})
        t += SIM_DT

    # Phase 3: Hold at target
    for _ in range(int(duration / SIM_DT)):
        actions = torch.zeros(1, n_joints, device="cuda:0")
        actions[0, joint_idx] = target
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        data.append({"time": t, "cmd_pos": target, "pos": pos, "vel": vel, "torque": 0, "temp": 0})
        t += SIM_DT

    # Phase 4: Ramp back to 0
    for i in range(n_ramp):
        progress = (i + 1) / n_ramp
        cmd = target * (1.0 - progress)
        actions = torch.zeros(1, n_joints, device="cuda:0")
        actions[0, joint_idx] = cmd
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        data.append({"time": t, "cmd_pos": cmd, "pos": pos, "vel": vel, "torque": 0, "temp": 0})
        t += SIM_DT

    # Phase 5: Hold at 0
    for _ in range(int(duration / SIM_DT)):
        actions = torch.zeros(1, n_joints, device="cuda:0")
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        data.append({"time": t, "cmd_pos": 0.0, "pos": pos, "vel": vel, "torque": 0, "temp": 0})
        t += SIM_DT

    print(f"  Step response: {len(data)} samples at {1/SIM_DT:.0f}Hz")
    return data


def run_sine_sweep(robot: Articulation, sim: SimulationContext,
                   joint_idx: int, freq: float, amplitude: float,
                   n_cycles: int = 5) -> list[dict]:
    """Run sine sweep and record."""
    data = []
    n_joints = robot.num_joints
    duration = n_cycles / freq
    t = 0.0

    # Hold at 0 (0.5s)
    for _ in range(int(0.5 / SIM_DT)):
        actions = torch.zeros(1, n_joints, device="cuda:0")
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        data.append({"time": t, "cmd_pos": 0.0, "pos": pos, "vel": vel, "torque": 0, "temp": 0})
        t += SIM_DT

    # Sine phase
    sine_start = t
    for _ in range(int(duration / SIM_DT)):
        cmd = amplitude * math.sin(2 * math.pi * freq * (t - sine_start))
        actions = torch.zeros(1, n_joints, device="cuda:0")
        actions[0, joint_idx] = cmd
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        data.append({"time": t, "cmd_pos": cmd, "pos": pos, "vel": vel, "torque": 0, "temp": 0})
        t += SIM_DT

    # Hold at 0 (0.5s)
    for _ in range(int(0.5 / SIM_DT)):
        actions = torch.zeros(1, n_joints, device="cuda:0")
        robot.set_joint_position_target(actions)
        sim.step()
        robot.update(SIM_DT)

        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        data.append({"time": t, "cmd_pos": 0.0, "pos": pos, "vel": vel, "torque": 0, "temp": 0})
        t += SIM_DT

    print(f"  Sine {freq}Hz: {len(data)} samples at {1/SIM_DT:.0f}Hz")
    return data


def run_full_test(joint_name: str, output_dir: str, actuator_type: str,
                  urdf: str, kp: float = None, kd: float = None):
    """Run complete sysid for one joint in Isaac."""
    urdf_name = FRIENDLY_TO_URDF[joint_name]
    motor_type = JOINT_TO_MOTOR_TYPE[urdf_name]
    gains = MOTOR_TYPE_GAINS[motor_type]

    lo, hi = JOINT_LIMITS_70PCT.get(joint_name, (-0.5, 0.5))
    mid = (lo + hi) / 2.0
    half_range = (hi - lo) / 2.0
    step_target = mid + half_range * 0.5
    sine_amplitude = half_range * 0.5

    actual_kp = kp or gains["kp"]
    actual_kd = kd or gains["kd"]

    print(f"\n{'='*60}")
    print(f"Isaac SysID: {joint_name} ({motor_type}), {actuator_type}")
    print(f"  Kp={actual_kp}, Kd={actual_kd}")
    print(f"  Step target: {step_target:.3f}, Sine amp: {sine_amplitude:.3f}")
    print(f"{'='*60}")

    sim = create_sim_context()
    robot = create_robot(sim, urdf, actuator_type, joint_name, actual_kp, actual_kd)

    # Reset sim
    sim.reset()
    robot.reset()

    joint_idx = get_joint_index(robot, joint_name)
    joint_dir = os.path.join(output_dir, f"isaac_{actuator_type}", f"{joint_name}_{motor_type}")

    # Save metadata
    import yaml
    os.makedirs(joint_dir, exist_ok=True)
    meta = {
        "joint": joint_name,
        "motor_type": motor_type,
        "actuator_type": actuator_type,
        "kp": actual_kp,
        "kd": actual_kd,
        "sim_dt": SIM_DT,
        "step_target": step_target,
        "sine_amplitude": sine_amplitude,
        "urdf": urdf,
    }
    with open(os.path.join(joint_dir, "metadata.yaml"), "w") as f:
        yaml.dump(meta, f)

    # Step response
    data = run_step_response(robot, sim, joint_idx, step_target)
    save_csv(data, os.path.join(joint_dir, "step_response.csv"))

    # Sine sweeps
    for freq in SINE_FREQUENCIES:
        robot.reset()
        data = run_sine_sweep(robot, sim, joint_idx, freq, sine_amplitude)
        save_csv(data, os.path.join(joint_dir, f"sine_{freq}hz.csv"))

    print(f"\n✅ Complete: {joint_dir}")


def main():
    if args_cli.all:
        for mtype, joint_name in DEFAULT_TEST_JOINTS.items():
            run_full_test(joint_name, args_cli.output, args_cli.actuator,
                          args_cli.urdf, args_cli.kp, args_cli.kd)
    elif args_cli.joint:
        run_full_test(args_cli.joint, args_cli.output, args_cli.actuator,
                      args_cli.urdf, args_cli.kp, args_cli.kd)
    else:
        print("Specify --joint or --all")

    simulation_app.close()


if __name__ == "__main__":
    main()
