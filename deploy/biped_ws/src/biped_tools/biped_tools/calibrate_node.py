"""Live joint calibration — LeRobot-style real-time display.

Continuously reads all motor positions and tracks min/max as you move them.
Joints auto-mark as ✅ DONE when their observed range matches URDF within 20%.
Press Ctrl+C when all joints are done to save calibration.yaml.

Usage:
    ros2 run biped_tools calibrate_node --ros-args \
        -p can_backend:=waveshare -p can_channel:=/dev/ttyUSB0
"""

import yaml
import time
import math
import sys
import os
import rclpy
from rclpy.node import Node

from biped_driver.robstride_dynamics import RobstrideBus, Motor

# URDF joint limits: (lower_rad, upper_rad, default_pos_rad)
URDF_LIMITS = {
    "R_hip_pitch":  (-1.047, 2.222,  0.2),
    "R_hip_roll":   (-2.269, 0.209,  0.0),
    "R_hip_yaw":    (-2.094, 2.094,  0.0),
    "R_knee":       ( 0.000, 2.618,  0.4),
    "R_foot_pitch": (-1.047, 0.524, -0.2),
    "R_foot_roll":  (-0.262, 0.262,  0.0),
    "L_hip_pitch":  (-2.222, 1.047, -0.2),
    "L_hip_roll":   (-0.209, 2.269,  0.0),
    "L_hip_yaw":    (-2.094, 2.094,  0.0),
    "L_knee":       ( 0.000, 2.618,  0.4),
    "L_foot_pitch": (-1.047, 0.524, -0.2),
    "L_foot_roll":  (-0.262, 0.262,  0.0),
}

MOTOR_LIST = [
    ("R_hip_pitch",   1, "rs-04"),
    ("R_hip_roll",    2, "rs-03"),
    ("R_hip_yaw",     3, "rs-03"),
    ("R_knee",        4, "rs-04"),
    ("R_foot_pitch",  5, "rs-02"),
    ("R_foot_roll",   6, "rs-02"),
    ("L_hip_pitch",   7, "rs-04"),
    ("L_hip_roll",    8, "rs-03"),
    ("L_hip_yaw",     9, "rs-03"),
    ("L_knee",       10, "rs-04"),
    ("L_foot_pitch", 11, "rs-02"),
    ("L_foot_roll",  12, "rs-02"),
]

ANKLE_JOINTS = {"L_foot_pitch", "L_foot_roll", "R_foot_pitch", "R_foot_roll"}
RANGE_MATCH_THRESHOLD = 0.20  # 20% tolerance


class CalibrateNode(Node):
    def __init__(self):
        super().__init__('calibrate_node')

        self.declare_parameter('can_backend', 'waveshare')
        self.declare_parameter('can_channel', '/dev/ttyUSB0')
        self.declare_parameter('output_file', 'calibration.yaml')

        self._backend = str(self.get_parameter('can_backend').value)
        self._channel = str(self.get_parameter('can_channel').value)
        self._output = str(self.get_parameter('output_file').value)

    def run_calibration(self):
        # Build bus
        motors = {}
        for name, mid, model in MOTOR_LIST:
            motors[name] = Motor(id=mid, model=model)

        bus = RobstrideBus(
            channel=self._channel,
            motors=motors,
            backend=self._backend,
        )
        bus.connect()

        # Enable all in zero-torque MIT mode
        print(f"\nEnabling {len(MOTOR_LIST)} motors (zero torque)...")
        for name, mid, model in MOTOR_LIST:
            try:
                bus.enable(name)
                time.sleep(0.02)
                bus.set_mode(name, 0)
                time.sleep(0.02)
            except Exception as e:
                print(f"  ✗ {name}: {e}")
                bus.disconnect()
                return
        bus.flush_rx()
        print("All motors enabled. Move each joint to both limits.\n")

        # Tracking state
        pos_min = {}
        pos_max = {}
        pos_cur = {}
        done = {}
        for name, _, _ in MOTOR_LIST:
            pos_min[name] = float('inf')
            pos_max[name] = float('-inf')
            pos_cur[name] = 0.0
            done[name] = False

        n_joints = len(MOTOR_LIST)
        # Header height: 3 lines header + n_joints lines + 2 lines footer
        display_lines = n_joints + 5

        # Print static header
        print("=" * 90)
        print(f"  {'Joint':<16} {'Cur':>7}  {'Min':>7}  {'Max':>7}  "
              f"{'Range':>7}  {'URDF':>7}  {'Err%':>5}  Status")
        print("-" * 90)
        # Reserve lines for each joint + footer
        for _ in range(n_joints + 2):
            print()

        try:
            while True:
                # Read all positions via zero-torque MIT
                for name, mid, model in MOTOR_LIST:
                    try:
                        bus.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
                        fb = bus.read_operation_frame(name, timeout=0.01)
                        if fb:
                            p = fb.position
                            pos_cur[name] = p
                            if p < pos_min[name]:
                                pos_min[name] = p
                            if p > pos_max[name]:
                                pos_max[name] = p
                    except Exception:
                        pass

                # Check done status
                n_done = 0
                for name, _, _ in MOTOR_LIST:
                    if done[name]:
                        n_done += 1
                        continue
                    urdf = URDF_LIMITS.get(name)
                    if not urdf:
                        continue
                    observed = pos_max[name] - pos_min[name]
                    if name in ANKLE_JOINTS:
                        # Ankle: just check that we've seen meaningful range (>0.3 rad)
                        if observed > 0.3:
                            done[name] = True
                            n_done += 1
                    else:
                        urdf_range = urdf[1] - urdf[0]
                        if urdf_range > 0:
                            err = abs(observed - urdf_range) / urdf_range
                            if err <= RANGE_MATCH_THRESHOLD:
                                done[name] = True
                                n_done += 1

                # Move cursor up to overwrite
                sys.stdout.write(f"\033[{n_joints + 2}A")

                for name, mid, model in MOTOR_LIST:
                    cur = pos_cur[name]
                    mn = pos_min[name] if pos_min[name] != float('inf') else 0.0
                    mx = pos_max[name] if pos_max[name] != float('-inf') else 0.0
                    observed = mx - mn

                    urdf = URDF_LIMITS.get(name)
                    if urdf:
                        urdf_range = urdf[1] - urdf[0]
                        if urdf_range > 0 and observed > 0.01:
                            err_pct = abs(observed - urdf_range) / urdf_range * 100
                        else:
                            err_pct = 999.9
                        urdf_str = f"{urdf_range:6.2f}"
                    else:
                        err_pct = 0.0
                        urdf_str = "  —   "

                    if done[name]:
                        status = "✅ DONE"
                    elif observed > 0.1:
                        status = "🔄 moving..."
                    else:
                        status = "⏳ waiting"

                    line = (f"  {name:<16} {cur:+7.3f}  {mn:+7.3f}  {mx:+7.3f}  "
                            f"{observed:6.3f}  {urdf_str}  {err_pct:5.1f}  {status}")
                    # Clear line and write
                    sys.stdout.write(f"\033[2K{line}\n")

                # Footer
                sys.stdout.write(f"\033[2K\n")
                sys.stdout.write(f"\033[2K  {n_done}/{n_joints} joints done. "
                                 f"{'Press Ctrl+C to save.' if n_done == n_joints else 'Keep moving joints...'}\n")
                sys.stdout.flush()

                if n_done == n_joints:
                    time.sleep(0.5)
                    break

                time.sleep(0.05)  # ~20Hz display update

        except KeyboardInterrupt:
            print("\n\nStopped by user.")

        # Disable motors
        print("\nDisabling motors...")
        for name, _, _ in MOTOR_LIST:
            try:
                bus.disable(name)
            except Exception:
                pass
        bus.disconnect()

        # Build and save calibration
        calibration = {}
        for name, mid, model in MOTOR_LIST:
            mn = pos_min[name] if pos_min[name] != float('inf') else 0.0
            mx = pos_max[name] if pos_max[name] != float('-inf') else 0.0

            cal = {
                'encoder_limit_a': round(float(mn), 4),
                'encoder_limit_b': round(float(mx), 4),
                'limit_lo': round(float(mn), 4),
                'limit_hi': round(float(mx), 4),
            }

            urdf = URDF_LIMITS.get(name)
            if urdf and name not in ANKLE_JOINTS:
                offset = mn - urdf[0]
                cal['offset'] = round(float(offset), 4)
                cal['urdf_lower'] = round(urdf[0], 4)
                cal['urdf_upper'] = round(urdf[1], 4)
                cal['default_pos'] = round(urdf[2], 4)
            else:
                cal['offset'] = round(float(mn), 4)

            calibration[name] = cal

        print(f"\nSaving to {self._output}...")
        with open(self._output, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False, sort_keys=False)

        done_count = sum(1 for v in done.values() if v)
        print(f"\n✅ Calibration saved! ({done_count}/{n_joints} joints fully calibrated)")
        print(f"   File: {self._output}")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateNode()
    try:
        node.run_calibration()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
