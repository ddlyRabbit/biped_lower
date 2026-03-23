"""Motor SysID — standalone Isaac Sim.

No ManagerBasedRLEnv. Direct SimulationContext + Articulation.
No randomization, no events, no rewards. Pure actuator response.

Usage:
    /isaac-sim/python.sh sysid_isaac.py --joint R_hip_pitch --headless
    /isaac-sim/python.sh sysid_isaac.py --joint R_hip_pitch --headless --video
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--joint", type=str, required=True, help="Joint name (e.g. R_hip_pitch)")
parser.add_argument("--output", type=str, default="/results/motor_sysid")
parser.add_argument("--video", action="store_true", help="Record video")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

if args.video:
    args.enable_cameras = True

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# --- Post-launch imports ---
import torch
import yaml
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationCfg, SimulationContext

if args.video:
    from isaaclab.sensors import CameraCfg, Camera
    import imageio

from sysid_config import (
    SYSID_ROBOT_CFG, SIM_DT, DECIMATION, CONTROL_DT,
    JOINT_INDEX, DEFAULT_POS, MOTOR_TYPES, TEST_PARAMS,
)

# --- Helpers ---

def save_csv(data, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["time", "cmd_pos", "pos", "vel", "torque"])
        w.writeheader()
        w.writerows(data)
    print("  Saved %s (%d rows)" % (path, len(data)))


def build_default_targets(robot):
    """Build a target tensor with all joints at their default positions."""
    targets = robot.data.default_joint_pos[0].clone()
    return targets


def step_control(robot, sim, targets, decimation=DECIMATION):
    """One control step: set targets, step physics decimation times, update."""
    robot.set_joint_position_target(targets.unsqueeze(0))
    robot.write_data_to_sim()
    for _ in range(decimation):
        sim.step(render=False)
    robot.update(SIM_DT * decimation)


def step_control_render(robot, sim, targets, decimation=DECIMATION, camera=None, video_writer=None):
    """Same as step_control but renders on last substep + captures frame."""
    robot.set_joint_position_target(targets.unsqueeze(0))
    robot.write_data_to_sim()
    for i in range(decimation):
        sim.step(render=(i == decimation - 1))
    robot.update(SIM_DT * decimation)
    capture_frame(camera, video_writer)


def read_joint(robot, jidx):
    """Read position and velocity for a single joint."""
    pos = robot.data.joint_pos[0, jidx].item()
    vel = robot.data.joint_vel[0, jidx].item()
    return pos, vel


def capture_frame(camera, video_writer):
    """Capture one frame from camera and write to video."""
    if camera is None or video_writer is None:
        return
    camera.update(dt=CONTROL_DT)
    rgb = camera.data.output["rgb"]
    if rgb is not None and rgb.numel() > 0:
        frame = rgb[0].cpu().numpy()
        if frame.shape[-1] == 4:  # RGBA → RGB
            frame = frame[:, :, :3]
        video_writer.append_data(frame)


# --- Main ---

def main():
    joint_name = args.joint
    if joint_name not in JOINT_INDEX:
        print("Unknown joint: %s" % joint_name)
        print("Available: %s" % list(JOINT_INDEX.keys()))
        simulation_app.close()
        return

    jidx = JOINT_INDEX[joint_name]
    mtype = MOTOR_TYPES.get(joint_name, "UNK")
    params = TEST_PARAMS.get(joint_name, {"step": 0.3, "amp": 0.2, "freqs": [1.0]})
    default = DEFAULT_POS.get(joint_name, 0.0)
    out_dir = os.path.join(args.output, "%s_%s" % (joint_name, mtype))

    print("=" * 60)
    print("SysID: %s (%s) idx=%d default=%.3f" % (joint_name, mtype, jidx, default))
    print("  step_target=%.3f sine_amp=%.3f" % (params["step"], params["amp"]))
    print("  Kp from config, DelayedPD with friction")
    print("=" * 60)

    # Create sim
    sim_cfg = SimulationCfg(
        dt=SIM_DT,
        render_interval=DECIMATION if args.video else 1,
        device="cuda:0",
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2.5, 2.5, 2.0], target=[0.0, 0.0, 0.8])

    # Spawn ground plane
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)

    # Spawn light
    light_cfg = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/light", light_cfg, translation=(0, 0, 10))

    # Spawn robot
    robot = Articulation(SYSID_ROBOT_CFG)

    # Spawn camera for video
    camera = None
    video_writer = None
    if args.video:
        camera_cfg = CameraCfg(
            prim_path="/World/Camera",
            update_period=CONTROL_DT,
            height=480,
            width=640,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 100.0),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(3.0, 3.0, 2.0),
                rot=(0.36, 0.11, -0.28, 0.88),
                convention="world",
            ),
        )
        camera = Camera(camera_cfg)
        video_path = os.path.join(out_dir, "sysid_video.mp4")
        os.makedirs(out_dir, exist_ok=True)
        video_writer = imageio.get_writer(video_path, fps=int(1.0 / CONTROL_DT), codec="h264")
        print("Video recording to: %s" % video_path)

    # Reset sim
    sim.reset()
    robot.reset()
    if camera is not None:
        camera.reset()

    # Print joint names for verification
    print("Joint names:", robot.data.joint_names)
    print("Num joints:", robot.num_joints)
    print("Test joint: [%d] = %s" % (jidx, robot.data.joint_names[jidx]))

    # Verify defaults loaded
    defaults = robot.data.default_joint_pos[0]
    print("Default positions:", defaults.cpu().numpy())

    # Choose step function based on video flag
    if args.video:
        def do_step(robot, sim, targets, decimation=DECIMATION):
            step_control_render(robot, sim, targets, decimation, camera, video_writer)
    else:
        do_step = step_control

    # --- Warmup: let robot settle at defaults for 1s ---
    print("Warmup: settling at defaults...")
    targets = build_default_targets(robot)
    for _ in range(int(1.0 / CONTROL_DT)):
        do_step(robot, sim, targets)

    pos0, vel0 = read_joint(robot, jidx)
    print("After warmup: pos=%.4f vel=%.4f" % (pos0, vel0))

    # --- Step Response ---
    print("\nStep response...")
    data = []
    t = 0.0
    step_target = params["step"]

    # Hold at default (0.5s)
    targets = build_default_targets(robot)
    for _ in range(int(0.5 / CONTROL_DT)):
        do_step(robot, sim, targets)
        pos, vel = read_joint(robot, jidx)
        data.append({"time": round(t, 5), "cmd_pos": round(default, 6),
                      "pos": round(pos, 6), "vel": round(vel, 6), "torque": 0})
        t += CONTROL_DT

    # Step to absolute target (2s)
    targets = build_default_targets(robot)
    targets[jidx] = step_target
    for _ in range(int(2.0 / CONTROL_DT)):
        do_step(robot, sim, targets)
        pos, vel = read_joint(robot, jidx)
        data.append({"time": round(t, 5), "cmd_pos": round(step_target, 6),
                      "pos": round(pos, 6), "vel": round(vel, 6), "torque": 0})
        t += CONTROL_DT

    # Return to default (1s)
    targets = build_default_targets(robot)
    for _ in range(int(1.0 / CONTROL_DT)):
        do_step(robot, sim, targets)
        pos, vel = read_joint(robot, jidx)
        data.append({"time": round(t, 5), "cmd_pos": round(default, 6),
                      "pos": round(pos, 6), "vel": round(vel, 6), "torque": 0})
        t += CONTROL_DT

    save_csv(data, os.path.join(out_dir, "step_response.csv"))

    # --- Sine Sweeps ---
    for freq in params["freqs"]:
        print("Sine sweep %.1f Hz..." % freq)
        data = []
        t = 0.0
        amp = params["amp"]
        dur = 5.0 / freq  # 5 cycles

        # Hold 0.5s
        targets = build_default_targets(robot)
        for _ in range(int(0.5 / CONTROL_DT)):
            do_step(robot, sim, targets)
            pos, vel = read_joint(robot, jidx)
            data.append({"time": round(t, 5), "cmd_pos": round(default, 6),
                          "pos": round(pos, 6), "vel": round(vel, 6), "torque": 0})
            t += CONTROL_DT

        # Sine around default
        t0 = t
        for _ in range(int(dur / CONTROL_DT)):
            desired = default + amp * math.sin(2 * math.pi * freq * (t - t0))
            targets = build_default_targets(robot)
            targets[jidx] = desired
            do_step(robot, sim, targets)
            pos, vel = read_joint(robot, jidx)
            data.append({"time": round(t, 5), "cmd_pos": round(desired, 6),
                          "pos": round(pos, 6), "vel": round(vel, 6), "torque": 0})
            t += CONTROL_DT

        # Hold 0.5s
        targets = build_default_targets(robot)
        for _ in range(int(0.5 / CONTROL_DT)):
            do_step(robot, sim, targets)
            pos, vel = read_joint(robot, jidx)
            data.append({"time": round(t, 5), "cmd_pos": round(default, 6),
                          "pos": round(pos, 6), "vel": round(vel, 6), "torque": 0})
            t += CONTROL_DT

        save_csv(data, os.path.join(out_dir, "sine_%.1fhz.csv" % freq))

    # Metadata
    meta = {
        "joint": joint_name,
        "motor_type": mtype,
        "default_pos": float(default),
        "step_target": float(params["step"]),
        "sine_amp": float(params["amp"]),
        "sine_freqs": params["freqs"],
        "sim_dt": SIM_DT,
        "control_dt": CONTROL_DT,
        "decimation": DECIMATION,
        "suspended": True,
        "gravity": True,
    }
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "metadata.yaml"), "w") as f:
        yaml.dump(meta, f)

    # Close video
    if video_writer is not None:
        video_writer.close()
        print("Video saved: %s" % os.path.join(out_dir, "sysid_video.mp4"))

    print("\nDone! Output: %s" % out_dir)
    simulation_app.close()


if __name__ == "__main__":
    main()
