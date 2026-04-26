#!/usr/bin/env python3
"""Motor System Identification — Record step and sine responses from real hardware.

Activates ONLY the test motor on its CAN bus. All other motors disabled.
Records at maximum CAN rate (single motor → ~500-1000Hz).

Usage:
    # Test one motor type
    python motor_sysid.py --joint R_hip_pitch --kp 250 --kd 5

    # All 3 motor types (RS04, RS03, RS02)
    python motor_sysid.py --all

    # Custom output dir
    python motor_sysid.py --joint R_hip_roll --output /tmp/sysid/

Requires: robot powered, CAN bus up (run setup_can.sh first).
Robot must be SUSPENDED (feet off ground, joints free to swing).
"""

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Add parent paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "biped_ws", "src", "biped_driver", "biped_driver"))
from robstride_can import (
    BipedMotorManager, JointConfig,
    ankle_command_to_motors, ankle_motors_to_feedback,
    ANKLE_PAIRS, ANKLE_MOTOR_TO_JOINT,
)

# ─── Default test configs per motor type ────────────────────────────────────

MOTOR_TYPE_GAINS = {
    "RS04": {"kp": 1, "kd": 0.1, "effort": 60},
    "RS03": {"kp": 1, "kd": 0.1, "effort": 40},
    "RS02": {"kp": 1, "kd": 0.1, "effort": 14},
}

# Representative joints per motor type
DEFAULT_TEST_JOINTS = {
    "RS04": "R_hip_pitch",
    "RS03": "R_hip_roll",
    "RS02": "R_foot_pitch",  # ankle joint-space (uses both foot_top + foot_bottom)
}

# Ankle joints need special handling (parallel linkage)
ANKLE_JOINTS = {"R_foot_pitch", "L_foot_pitch", "R_foot_roll", "L_foot_roll"}

# Map ankle joint-space names to motor pair names
ANKLE_JOINT_TO_PAIR = {
    "R_foot_pitch": ("R_foot_top", "R_foot_bottom"),
    "R_foot_roll":  ("R_foot_top", "R_foot_bottom"),
    "L_foot_pitch": ("L_foot_top", "L_foot_bottom"),
    "L_foot_roll":  ("L_foot_top", "L_foot_bottom"),
}

import os
import yaml
import math

_this_dir = os.path.dirname(os.path.abspath(__file__))
CHIRP_YAML = os.path.abspath(os.path.join(_this_dir, "../biped_ws/src/biped_bringup/config/chirp.yaml"))
with open(CHIRP_YAML, 'r') as f:
    _wcfg = yaml.safe_load(f)["chirp"]["joints"]

# Dynamically loaded limits for sysid
JOINT_LIMITS_CONFIG = {
    j: (math.radians(data["min"]), math.radians(data["max"]))
    for j, data in _wcfg.items()
}

# Hardware sysid needs motor-space mappings for ankle top/bottom:
for side in ["R", "L"]:
    if f"{side}_foot_pitch" in JOINT_LIMITS_CONFIG:
        JOINT_LIMITS_CONFIG[f"{side}_foot_top"] = JOINT_LIMITS_CONFIG[f"{side}_foot_pitch"]
    if f"{side}_foot_roll" in JOINT_LIMITS_CONFIG:
        JOINT_LIMITS_CONFIG[f"{side}_foot_bottom"] = JOINT_LIMITS_CONFIG[f"{side}_foot_roll"]


@dataclass
class SysIdConfig:
    joint_name: str
    motor_type: str
    kp: float
    kd: float
    effort: float
    step_target: float      # step response target (rad)
    sine_amplitude: float   # sine sweep amplitude (rad)
    sine_frequencies: list   # Hz
    sine_cycles: int = 5    # cycles per frequency
    ramp_time: float = 0.1  # seconds to ramp to target


def get_motor_type(joint_name: str, mgr: BipedMotorManager) -> str:
    """Get RS02/RS03/RS04 from joint config."""
    # Ankle joint-space names map to motor names
    if joint_name in ANKLE_JOINT_TO_PAIR:
        top_name = ANKLE_JOINT_TO_PAIR[joint_name][0]
        jcfg = mgr.joints.get(top_name)
    else:
        jcfg = mgr.joints.get(joint_name)
    if jcfg is None:
        raise ValueError(f"Joint {joint_name} not found")
    model = jcfg.model.upper().replace("-", "").replace("RS0", "RS0")
    if "04" in model:
        return "RS04"
    elif "03" in model:
        return "RS03"
    elif "02" in model:
        return "RS02"
    raise ValueError(f"Unknown motor model: {jcfg.model}")


def build_config(joint_name: str, mgr: BipedMotorManager, kp: float = None, kd: float = None) -> SysIdConfig:
    """Build test config for a joint."""
    mtype = get_motor_type(joint_name, mgr)
    gains = MOTOR_TYPE_GAINS[mtype]

    lo, hi = JOINT_LIMITS_CONFIG.get(joint_name, (-0.5, 0.5))
    mid = (lo + hi) / 2.0
    half_range = (hi - lo) / 2.0

    return SysIdConfig(
        joint_name=joint_name,
        motor_type=mtype,
        kp=kp or gains["kp"],
        kd=kd or gains["kd"],
        effort=gains["effort"],
        step_target=mid + half_range * 0.5,  # 50% into range from center
        sine_amplitude=half_range * 0.5,
        sine_frequencies=[0.5, 1.0, 2.0, 5.0, 10.0],
    )


class SysIdRecorder:
    """Records motor response data at max CAN rate."""

    def __init__(self, mgr: BipedMotorManager, config: SysIdConfig):
        self.mgr = mgr
        self.cfg = config
        self.data: list[dict] = []
        self._bus_name = mgr.joints[config.joint_name].can_bus

    def _get_bus_motors(self) -> list[str]:
        """Get all motor names on the same bus."""
        return [n for n, j in self.mgr.joints.items() if j.can_bus == self._bus_name]

    def _record_point(self, t: float, cmd_pos: float, fb):
        """Record one data point."""
        self.data.append({
            "time": t,
            "cmd_pos": cmd_pos,
            "pos": fb.position if fb else float("nan"),
            "vel": fb.velocity if fb else float("nan"),
            "torque": fb.torque if fb else float("nan"),
            "temp": fb.temperature if fb else 0,
        })

    def _is_ankle(self) -> bool:
        return self.cfg.joint_name in ANKLE_JOINTS

    def _get_pitch_sign(self) -> int:
        return -1 if self.cfg.joint_name.startswith("L") else 1

    def _send_and_record(self, cmd_pos: float, t0: float) -> float:
        """Send MIT command, record feedback. Handles ankle linkage transform."""
        t = time.monotonic() - t0
        fb = None
        try:
            if self._is_ankle():
                top_name, bottom_name = ANKLE_JOINT_TO_PAIR[self.cfg.joint_name]
                pitch_sign = self._get_pitch_sign()
                # Determine if testing pitch or roll
                is_pitch = "pitch" in self.cfg.joint_name
                if is_pitch:
                    motor_upper, motor_lower = ankle_command_to_motors(cmd_pos, 0.0, pitch_sign)
                else:
                    motor_upper, motor_lower = ankle_command_to_motors(0.0, cmd_pos, pitch_sign)
                # Send to both motors
                self.mgr.send_ankle_mit_command(
                    top_name, motor_upper, self.cfg.kp, self.cfg.kd, 0.0, 0.0)
                fb_upper = self.mgr.read_feedback(top_name)
                self.mgr.send_ankle_mit_command(
                    bottom_name, motor_lower, self.cfg.kp, self.cfg.kd, 0.0, 0.0)
                fb_lower = self.mgr.read_feedback(bottom_name)
                # Transform back to joint space
                if fb_upper and fb_lower:
                    joint_pitch, joint_roll = ankle_motors_to_feedback(
                        fb_upper.position, fb_lower.position, pitch_sign)
                    joint_pitch_vel, joint_roll_vel = ankle_motors_to_feedback(
                        fb_upper.velocity, fb_lower.velocity, pitch_sign)
                    # Create synthetic feedback in joint space
                    from robstride_dynamics.bus import MotorFeedback as MF
                    if is_pitch:
                        fb = MF(position=joint_pitch, velocity=joint_pitch_vel,
                                torque=(fb_upper.torque + fb_lower.torque) / 2,
                                temperature=max(fb_upper.temperature, fb_lower.temperature))
                    else:
                        fb = MF(position=joint_roll, velocity=joint_roll_vel,
                                torque=(fb_upper.torque + fb_lower.torque) / 2,
                                temperature=max(fb_upper.temperature, fb_lower.temperature))
            else:
                self.mgr.send_mit_command(
                    self.cfg.joint_name, cmd_pos,
                    self.cfg.kp, self.cfg.kd, 0.0, 0.0)
                fb = self.mgr.read_feedback(self.cfg.joint_name)
        except Exception as e:
            print(f"  CAN error: {e}")
        self._record_point(t, cmd_pos, fb)
        return t

    def setup(self):
        """Connect, enable test motor only, disable all others on same bus."""
        bus_motors = self._get_bus_motors()
        print(f"Bus {self._bus_name}: {bus_motors}")
        print(f"Test motor: {self.cfg.joint_name} ({self.cfg.motor_type})")
        print(f"  Kp={self.cfg.kp}, Kd={self.cfg.kd}, effort={self.cfg.effort}")

        # Connect and enable all on this bus first
        self.mgr.connect_all()
        self.mgr.flush_all()

        # Enable only the test motor
        bus = self.mgr._buses[self._bus_name]
        for name in bus_motors:
            bus.enable(name)
            time.sleep(0.01)

        # Send zero torque to all others (Kp=0, Kd=0)
        for name in bus_motors:
            if name != self.cfg.joint_name:
                bus.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
                bus._receive_feedback(name)

        # For ankle joints, also keep the paired motor active
        if self.cfg.joint_name in ANKLE_JOINTS:
            pair = ANKLE_JOINT_TO_PAIR[self.cfg.joint_name]
            print(f"  Ankle mode: activating pair {pair}")

        print("Setup complete — test motor(s) active")

    def run_step_response(self, target: float = None, duration: float = 2.0):
        """Step response: 0 → target, hold, target → 0."""
        if target is None:
            target = self.cfg.step_target

        print(f"\n=== Step response: 0 → {target:.3f} rad ===")
        self.data = []
        t0 = time.monotonic()

        # Phase 1: Hold at 0 for 0.5s
        while time.monotonic() - t0 < 0.5:
            self._send_and_record(0.0, t0)

        # Phase 2: Ramp to target
        ramp_start = time.monotonic() - t0
        while time.monotonic() - t0 < ramp_start + self.cfg.ramp_time:
            progress = (time.monotonic() - t0 - ramp_start) / self.cfg.ramp_time
            cmd = target * min(progress, 1.0)
            self._send_and_record(cmd, t0)

        # Phase 3: Hold at target
        while time.monotonic() - t0 < 0.5 + self.cfg.ramp_time + duration:
            self._send_and_record(target, t0)

        # Phase 4: Ramp back to 0
        ramp_start2 = time.monotonic() - t0
        while time.monotonic() - t0 < ramp_start2 + self.cfg.ramp_time:
            progress = (time.monotonic() - t0 - ramp_start2) / self.cfg.ramp_time
            cmd = target * (1.0 - min(progress, 1.0))
            self._send_and_record(cmd, t0)

        # Phase 5: Hold at 0
        while time.monotonic() - t0 < ramp_start2 + self.cfg.ramp_time + duration:
            self._send_and_record(0.0, t0)

        rate = len(self.data) / (time.monotonic() - t0)
        print(f"  Recorded {len(self.data)} samples in {time.monotonic()-t0:.1f}s ({rate:.0f} Hz)")
        return list(self.data)

    def run_sine_sweep(self, freq: float, amplitude: float = None, n_cycles: int = None):
        """Sine sweep at given frequency."""
        if amplitude is None:
            # Velocity envelope to prevent saturation at high frequencies
            import math
            amplitude = min(self.cfg.sine_amplitude, 8.0 / (2 * math.pi * freq))
        if n_cycles is None:
            n_cycles = self.cfg.sine_cycles

        duration = n_cycles / freq
        print(f"\n=== Sine {freq}Hz, amp={amplitude:.3f} rad, {n_cycles} cycles ({duration:.1f}s) ===")
        self.data = []
        t0 = time.monotonic()

        # Ramp in
        while time.monotonic() - t0 < 0.5:
            self._send_and_record(0.0, t0)

        # Sine phase
        sine_start = time.monotonic() - t0
        while time.monotonic() - t0 < sine_start + duration:
            t = time.monotonic() - t0 - sine_start
            cmd = amplitude * math.sin(2 * math.pi * freq * t)
            self._send_and_record(cmd, t0)

        # Ramp out
        ramp_start = time.monotonic() - t0
        while time.monotonic() - t0 < ramp_start + 0.5:
            self._send_and_record(0.0, t0)

        rate = len(self.data) / (time.monotonic() - t0)
        print(f"  Recorded {len(self.data)} samples in {time.monotonic()-t0:.1f}s ({rate:.0f} Hz)")
        return list(self.data)

    def teardown(self):
        """Disable all motors and disconnect."""
        try:
            self.mgr.disable_all()
        except Exception:
            pass
        self.mgr.disconnect_all()


def save_csv(data: list[dict], path: str):
    """Save data points to CSV."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "cmd_pos", "pos", "vel", "torque", "temp"])
        writer.writeheader()
        writer.writerows(data)
    print(f"  Saved {path} ({len(data)} rows)")


def run_full_test(mgr: BipedMotorManager, joint_name: str, output_dir: str,
                  kp: float = None, kd: float = None):
    """Run complete sysid test for one joint."""
    config = build_config(joint_name, mgr, kp, kd)
    recorder = SysIdRecorder(mgr, config)

    joint_dir = os.path.join(output_dir, f"{joint_name}_{config.motor_type}")
    os.makedirs(joint_dir, exist_ok=True)

    # Save metadata
    import yaml
    meta = {
        "joint": joint_name,
        "motor_type": config.motor_type,
        "kp": config.kp,
        "kd": config.kd,
        "effort": config.effort,
        "step_target": config.step_target,
        "sine_amplitude": config.sine_amplitude,
        "sine_frequencies": config.sine_frequencies,
        "ramp_time": config.ramp_time,
    }
    with open(os.path.join(joint_dir, "metadata.yaml"), "w") as f:
        yaml.dump(meta, f)

    try:
        recorder.setup()

        # Step response
        data = recorder.run_step_response()
        save_csv(data, os.path.join(joint_dir, "step_response.csv"))

        time.sleep(1.0)  # settle

        # Sine sweeps
        for freq in config.sine_frequencies:
            data = recorder.run_sine_sweep(freq)
            save_csv(data, os.path.join(joint_dir, f"sine_{freq}hz.csv"))
            time.sleep(0.5)

    finally:
        recorder.teardown()

    print(f"\n✅ Complete: {joint_dir}")


def main():
    parser = argparse.ArgumentParser(description="Motor System Identification")
    parser.add_argument("--joint", type=str, help="Joint name (e.g. R_hip_pitch)")
    parser.add_argument("--all", action="store_true", help="Test all 3 motor types")
    parser.add_argument("--kp", type=float, default=None, help="Override Kp")
    parser.add_argument("--kd", type=float, default=None, help="Override Kd")
    parser.add_argument("--output", type=str, default="motor_sysid_data", help="Output directory")
    parser.add_argument("--robot_config", type=str,
                        default=os.path.expanduser("~/biped_lower/deploy/biped_ws/src/biped_bringup/config/robot.yaml"))
    parser.add_argument("--calibration", type=str,
                        default=os.path.expanduser("~/biped_lower/deploy/biped_ws/calibration.yaml"))
    args = parser.parse_args()

    # Load robot config
    import yaml
    with open(args.robot_config) as f:
        robot_cfg = yaml.safe_load(f)

    cal = {}
    if os.path.exists(args.calibration):
        with open(args.calibration) as f:
            cal = yaml.safe_load(f) or {}

    mgr = BipedMotorManager.from_robot_yaml(robot_cfg, cal)

    if args.all:
        for mtype, joint_name in DEFAULT_TEST_JOINTS.items():
            print(f"\n{'='*60}")
            print(f"Testing {mtype}: {joint_name}")
            print(f"{'='*60}")
            run_full_test(mgr, joint_name, args.output, args.kp, args.kd)
    elif args.joint:
        run_full_test(mgr, args.joint, args.output, args.kp, args.kd)
    else:
        parser.error("Specify --joint or --all")


if __name__ == "__main__":
    main()
