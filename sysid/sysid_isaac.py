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
    import omni.replicator.core as rep
    import imageio

from sysid_config import (
    SYSID_ROBOT_CFG, SIM_DT, DECIMATION, CONTROL_DT,
    JOINT_INDEX, DEFAULT_POS, MOTOR_TYPES, TEST_PARAMS,
)

# File-based debug logger (stdout gets swallowed by Isaac Sim)
_DEBUG_LOG = None
def dprint(msg):
    print(msg, flush=True)
    if _DEBUG_LOG:
        _DEBUG_LOG.write(msg + '\n')
        _DEBUG_LOG.flush()

# --- Helpers ---

def save_csv(data, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["time"] + [f"{j}_target_sim" for j in JOINT_INDEX.keys()] + [f"{j}_pos_sim" for j in JOINT_INDEX.keys()] + [f"{j}_vel_sim" for j in JOINT_INDEX.keys()] + [f"{j}_tau_sim" for j in JOINT_INDEX.keys()])
        w.writeheader()
        w.writerows(data)
    print("  Saved %s (%d rows)" % (path, len(data)))


def build_default_targets(robot):
    """Build a target tensor with all joints at their default positions."""
    targets = robot.data.default_joint_pos[0].clone()
    return targets


def read_all_joints(robot, targets):
    """Read all joint positions, velocities, and torques matching real ROS recorder."""
    row = {}
    for name, jidx in JOINT_INDEX.items():
        row[f"{name}_target_sim"] = targets[jidx].item()
        row[f"{name}_pos_sim"] = robot.data.joint_pos[0, jidx].item()
        row[f"{name}_vel_sim"] = robot.data.joint_vel[0, jidx].item()
        
        # Handle Isaac Lab API naming (applied_effort or applied_torque)
        tau = 0.0
        if hasattr(robot.data, 'applied_effort') and robot.data.applied_effort is not None:
            tau = robot.data.applied_effort[0, jidx].item()
        elif hasattr(robot.data, 'applied_torque') and robot.data.applied_torque is not None:
            tau = robot.data.applied_torque[0, jidx].item()
        row[f"{name}_tau_sim"] = tau
        
    return row

def step_control(robot, sim, targets, t_start, data):
    """One control step: PD recomputed every substep (true 2kHz PD). Records at 200Hz."""
    robot.set_joint_position_target(targets.unsqueeze(0))
    t = t_start
    for i in range(DECIMATION):
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(SIM_DT)
        t += SIM_DT
        if (i + 1) % 10 == 0:  # 2kHz / 10 = 200Hz
            row = read_all_joints(robot, targets)
            row["time"] = round(t, 5)
            data.append(row)
    return t

def step_control_render(robot, sim, targets, t_start, data, rgb_annotator=None, video_writer=None):
    """Same as step_control but renders on last substep + captures frame."""
    robot.set_joint_position_target(targets.unsqueeze(0))
    t = t_start
    for i in range(DECIMATION):
        robot.write_data_to_sim()
        sim.step(render=(i == DECIMATION - 1))
        robot.update(SIM_DT)
        t += SIM_DT
        if (i + 1) % 10 == 0:
            row = read_all_joints(robot, targets)
            row["time"] = round(t, 5)
            data.append(row)
    capture_frame(rgb_annotator, video_writer)
    return t


def capture_frame(rgb_annotator, video_writer):
    """Capture one frame via replicator annotator and write to video."""
    if rgb_annotator is None or video_writer is None:
        return
    rgb_data = rgb_annotator.get_data()
    if rgb_data is not None and rgb_data.size > 0:
        frame = np.frombuffer(rgb_data, dtype=np.uint8).reshape(*rgb_data.shape)
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

    global _DEBUG_LOG
    os.makedirs(args.output, exist_ok=True)
    _DEBUG_LOG = open(os.path.join(args.output, "sysid_debug.log"), "w")

    dprint("=" * 60)
    dprint("SysID: %s (%s) idx=%d default=%.3f" % (joint_name, mtype, jidx, default))
    dprint("  step_target=%.3f sine_amp=%.3f" % (params["step"], params["amp"]))
    dprint("  Kp from config, DelayedPD with friction")
    dprint("=" * 60)

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

    # Video recording via replicator render product (same as Isaac Lab env)
    rgb_annotator = None
    video_writer = None
    if args.video:
        video_path = os.path.join(out_dir, "sysid_video.mp4")
        os.makedirs(out_dir, exist_ok=True)
        video_writer = imageio.get_writer(video_path, fps=int(1.0 / CONTROL_DT), codec="h264")
        print("Video recording to: %s" % video_path)
        # Render product + annotator created after sim.reset()

    # Reset sim
    sim.reset()
    robot.reset()

    # Create render product after sim reset (viewport must exist)
    if args.video:
        render_product = rep.create.render_product(
            "/OmniverseKit_Persp", (640, 480)
        )
        rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
        rgb_annotator.attach([render_product])
        print("Render product + RGB annotator created")

    # Print joint names for verification
    dprint("Joint names: %s" % robot.data.joint_names)
    dprint("Num joints: %d" % robot.num_joints)
    dprint("Test joint: [%d] = %s" % (jidx, robot.data.joint_names[jidx]))

    # Verify defaults loaded
    defaults = robot.data.default_joint_pos[0]
    dprint("Default positions: %s" % defaults.cpu().numpy())

    # Choose step function based on video flag
    if args.video:
        def do_step(robot, sim, targets, t, data):
            return step_control_render(robot, sim, targets, t, data, rgb_annotator, video_writer)
    else:
        do_step = step_control

    # --- Warmup: let robot settle at defaults for 1s ---
    dprint("Warmup: settling at defaults...")
    targets = build_default_targets(robot)
    t = 0.0
    for _ in range(int(1.0 / CONTROL_DT)):
        t = do_step(robot, sim, targets, t, [])

    row = read_all_joints(robot, targets)
    dprint("After warmup: pos=%.4f vel=%.4f" % (row[f"{joint_name}_pos_sim"], row[f"{joint_name}_vel_sim"]))

    # --- Step Response ---
    dprint("\nStep response...")
    data = []
    t = 0.0
    step_target = params["step"]

    # Hold at default (0.5s)
    targets = build_default_targets(robot)
    for _ in range(int(0.5 / CONTROL_DT)):
        t = do_step(robot, sim, targets, t, data)

    # Step to absolute target (2s)
    targets = build_default_targets(robot)
    targets[jidx] = step_target
    for _ in range(int(2.0 / CONTROL_DT)):
        t = do_step(robot, sim, targets, t, data)

    # Return to default (1s)
    targets = build_default_targets(robot)
    for _ in range(int(1.0 / CONTROL_DT)):
        t = do_step(robot, sim, targets, t, data)

    save_csv(data, os.path.join(out_dir, "step_response.csv"))

    # --- Chirp Sweep (Matches state_machine_node exactly) ---
    dprint("\nChirp sweep 1Hz to 10Hz (6s per step)...")
    data = []
    t = 0.0

    # Hold 0.5s at center (mid)
    targets = build_default_targets(robot)
    jcfg = params # has "step" (not needed), "amp" which is actually half_range * 0.5. 
    # Wait, state_machine_node uses: amp_base = (jcfg.max - jcfg.min) / 2.0.
    # In sysid_config.py, params["amp"] IS half_range * 0.5, but wait, the real node doesn't multiply by 0.5?
    # Let me check state_machine_node: double amp_base = (jcfg.max - jcfg.min) / 2.0;
    # So amp_base is the full half_range.
    amp_base = params["half_range"]
    mid = params["mid"]

    for _ in range(int(0.5 / CONTROL_DT)):
        targets[jidx] = mid
        t = do_step(robot, sim, targets, t, data)

    # Chirp
    t0 = t
    for step_idx in range(10): # 0 to 9 -> 1Hz to 10Hz
        current_f = 1.0 + step_idx
        phi_start = 2.0 * math.pi * 6.0 * (step_idx + (step_idx * (step_idx - 1)) / 2.0)
        amp = amp_base / current_f
        dprint("  Step %d: %.1f Hz, amp=%.3f rad" % (step_idx, current_f, amp))
        
        for _ in range(int(6.0 / CONTROL_DT)):
            time_in_step = t - t0 - (step_idx * 6.0)
            phi = phi_start + 2.0 * math.pi * current_f * time_in_step
            desired = mid + amp * math.sin(phi)
            
            targets = build_default_targets(robot)
            targets[jidx] = desired
            t = do_step(robot, sim, targets, t, data)

    # Hold 0.5s at default
    targets = build_default_targets(robot)
    for _ in range(int(0.5 / CONTROL_DT)):
        t = do_step(robot, sim, targets, t, data)

    save_csv(data, os.path.join(out_dir, "chirp_sweep.csv"))

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

    dprint("\nDone! Output: %s" % out_dir)
    if _DEBUG_LOG:
        _DEBUG_LOG.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
