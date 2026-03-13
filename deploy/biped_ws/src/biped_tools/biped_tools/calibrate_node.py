"""Live joint calibration — LeRobot-style real-time display.

Continuously reads all motor positions and tracks min/max.
Joints auto-mark ✅ DONE when observed range matches expected within 20%.

Ankle motors (foot_pitch/foot_roll) are shown in raw motor-space since
joint-space conversion requires offsets we don't have yet. The expected
motor range is computed from URDF joint limits through the linkage forward.

Press Ctrl+C when all joints are done (or early) to save calibration.yaml.

Usage:
    ros2 run biped_tools calibrate_node --ros-args \
        -p can_channel:=can0
"""

import yaml
import time
import sys
import rclpy
from rclpy.node import Node

from biped_driver.robstride_dynamics import RobstrideBus, Motor
from biped_driver.robstride_can import ANKLE_PAIRS, _PITCH_GAIN, _ROLL_GAIN

# From URDF: urdf/heavy/robot.urdf joint limits (rad)
# Format: (lower_rad, upper_rad, default_pos_rad)
URDF_LIMITS = {
    "R_hip_pitch":  (-2.21657, 1.04720,  0.2),
    "R_hip_roll":   (-2.26893, 0.20944,  0.0),
    "R_hip_yaw":    (-1.57080, 1.57080,  0.0),
    "R_knee":       ( 0.00000, 2.70526,  0.4),
    "R_foot_pitch": (-0.87267, 0.52360, -0.2),
    "R_foot_roll":  (-0.26180, 0.26180,  0.0),
    "L_hip_pitch":  (-1.04720, 2.21657, -0.2),
    "L_hip_roll":   (-0.20944, 2.26893,  0.0),
    "L_hip_yaw":    (-1.57080, 1.57080,  0.0),
    "L_knee":       ( 0.00000, 2.70526,  0.4),
    "L_foot_pitch": (-0.87267, 0.52360, -0.2),
    "L_foot_roll":  (-0.26180, 0.26180,  0.0),
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

DISPLAY_JOINTS = [name for name, _, _ in MOTOR_LIST]

# Ankle motor names (these are in ANKLE_PAIRS.keys() or .values())
ANKLE_PITCH_MOTORS = set(ANKLE_PAIRS.keys())       # foot_pitch = upper motor
ANKLE_ROLL_MOTORS = set(ANKLE_PAIRS.values())       # foot_roll = lower motor
ANKLE_ALL = ANKLE_PITCH_MOTORS | ANKLE_ROLL_MOTORS

# Expected motor range for each ankle motor (Asimov convention):
# motor_A (upper) =  K_P × pitch − K_R × roll
# motor_B (lower) = −K_P × pitch − K_R × roll
# Both motors have the same total range = K_P × pitch_range + K_R × roll_range
# (the sign differences cancel when computing max − min)
def _ankle_expected_motor_range(side_prefix):
    """Compute expected motor range for ankle motors from URDF joint limits."""
    pitch_urdf = URDF_LIMITS[f"{side_prefix}_foot_pitch"]
    roll_urdf = URDF_LIMITS[f"{side_prefix}_foot_roll"]
    pitch_range = pitch_urdf[1] - pitch_urdf[0]
    roll_range = roll_urdf[1] - roll_urdf[0]
    return _PITCH_GAIN * pitch_range + _ROLL_GAIN * roll_range

ANKLE_EXPECTED_RANGE = {
    "R_foot_pitch": _ankle_expected_motor_range("R"),
    "R_foot_roll":  _ankle_expected_motor_range("R"),
    "L_foot_pitch": _ankle_expected_motor_range("L"),
    "L_foot_roll":  _ankle_expected_motor_range("L"),
}

RANGE_MATCH_THRESHOLD = 0.05


class CalibrateNode(Node):
    def __init__(self):
        super().__init__('calibrate_node')

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('output_file', 'calibration.yaml')

        self._channel = str(self.get_parameter('can_channel').value)
        self._output = str(self.get_parameter('output_file').value)

    def run_calibration(self):
        motors = {}
        for name, mid, model in MOTOR_LIST:
            motors[name] = Motor(id=mid, model=model)

        bus = RobstrideBus(
            channel=self._channel,
            motors=motors,
        )
        bus.connect()

        print(f"\nEnabling {len(MOTOR_LIST)} motors (zero torque)...")
        for name, mid, _ in MOTOR_LIST:
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

        # Track raw motor encoder positions for ALL joints
        motor_cur = {n: 0.0 for n in DISPLAY_JOINTS}
        motor_min = {n: float('inf') for n in DISPLAY_JOINTS}
        motor_max = {n: float('-inf') for n in DISPLAY_JOINTS}
        done = {n: False for n in DISPLAY_JOINTS}

        n_joints = len(DISPLAY_JOINTS)

        # Compute expected ranges
        expected_range = {}
        for name in DISPLAY_JOINTS:
            if name in ANKLE_ALL:
                expected_range[name] = ANKLE_EXPECTED_RANGE[name]
            else:
                urdf = URDF_LIMITS.get(name)
                expected_range[name] = (urdf[1] - urdf[0]) if urdf else 0.0

        # Header
        print("=" * 95)
        print(f"  {'Joint':<16} {'Cur':>7}  {'Min':>7}  {'Max':>7}  "
              f"{'Range':>7}  {'Expct':>7}  {'Err%':>5}  Status")
        print("-" * 95)
        for _ in range(n_joints + 2):
            print()

        try:
            while True:
                # Read all motor positions
                for name, _, _ in MOTOR_LIST:
                    try:
                        bus.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
                        fb = bus.read_operation_frame(name, timeout=0.01)
                        if fb:
                            motor_cur[name] = fb.position
                            if fb.position < motor_min[name]:
                                motor_min[name] = fb.position
                            if fb.position > motor_max[name]:
                                motor_max[name] = fb.position
                    except Exception:
                        pass

                # Check done
                n_done = 0
                for name in DISPLAY_JOINTS:
                    if done[name]:
                        n_done += 1
                        continue
                    mn = motor_min[name]
                    mx = motor_max[name]
                    if mn == float('inf') or mx == float('-inf'):
                        continue
                    observed = mx - mn
                    exp = expected_range[name]
                    if exp > 0 and observed > 0.05:
                        err = abs(observed - exp) / exp
                        if err <= RANGE_MATCH_THRESHOLD:
                            done[name] = True
                            n_done += 1

                # Render
                sys.stdout.write(f"\033[{n_joints + 2}A")

                for name in DISPLAY_JOINTS:
                    cur = motor_cur[name]
                    mn = motor_min[name] if motor_min[name] != float('inf') else 0.0
                    mx = motor_max[name] if motor_max[name] != float('-inf') else 0.0
                    observed = mx - mn
                    exp = expected_range[name]

                    if exp > 0 and observed > 0.01:
                        err_pct = abs(observed - exp) / exp * 100
                    else:
                        err_pct = 999.9

                    is_ankle = name in ANKLE_ALL
                    tag = " [M]" if is_ankle else "    "

                    if done[name]:
                        status = "✅ DONE"
                    elif observed > 0.05:
                        status = "🔄 moving..."
                    else:
                        status = "⏳ waiting"

                    line = (f"  {name:<16}{cur:+7.3f}  {mn:+7.3f}  {mx:+7.3f}  "
                            f"{observed:6.3f}  {exp:6.3f}  {err_pct:5.1f}  {status}{tag}")
                    sys.stdout.write(f"\033[2K{line}\n")

                sys.stdout.write(f"\033[2K\n")
                if n_done == n_joints:
                    sys.stdout.write(
                        f"\033[2K  ✅ {n_done}/{n_joints} done! "
                        f"Press ENTER to save calibration.\n")
                else:
                    sys.stdout.write(
                        f"\033[2K  {n_done}/{n_joints} done. "
                        f"[M]=motor-space (ankle linkage). Keep moving...\n")
                sys.stdout.flush()

                if n_done == n_joints:
                    # All joints calibrated — wait for user confirmation
                    input()
                    break

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\nStopped early by user.")

        # Disable
        print("\nDisabling motors...")
        for name, _, _ in MOTOR_LIST:
            try:
                bus.disable(name)
            except Exception:
                pass
        bus.disconnect()

        # Save calibration
        calibration = {}
        for name, _, _ in MOTOR_LIST:
            mn = motor_min[name] if motor_min[name] != float('inf') else 0.0
            mx = motor_max[name] if motor_max[name] != float('-inf') else 0.0

            cal = {
                'motor_min': round(float(mn), 4),
                'motor_max': round(float(mx), 4),
            }

            urdf = URDF_LIMITS.get(name)
            is_ankle = name in ANKLE_ALL

            if is_ankle:
                # Ankle: offset = motor_min (raw motor-space zero reference)
                # Joint-space limits computed at runtime via linkage inverse
                cal['offset'] = round(float(mn), 4)
                # Store raw motor limits for soft stops in motor-space
                cal['limit_lo'] = round(float(mn), 4)
                cal['limit_hi'] = round(float(mx), 4)
            else:
                # Normal joint: offset maps encoder → URDF frame
                if urdf:
                    offset = mn - urdf[0]
                    cal['offset'] = round(float(offset), 4)
                    # Joint-space limits from URDF (validated by range check)
                    cal['limit_lo'] = round(float(urdf[0]), 4)
                    cal['limit_hi'] = round(float(urdf[1]), 4)
                else:
                    cal['offset'] = round(float(mn), 4)
                    cal['limit_lo'] = round(float(mn), 4)
                    cal['limit_hi'] = round(float(mx), 4)

            if urdf:
                cal['urdf_lower'] = round(urdf[0], 4)
                cal['urdf_upper'] = round(urdf[1], 4)
                cal['default_pos'] = round(urdf[2], 4)

            calibration[name] = cal

        print(f"\nSaving to {self._output}...")
        with open(self._output, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False, sort_keys=False)

        done_count = sum(1 for v in done.values() if v)
        print(f"\n✅ Calibration saved! ({done_count}/{n_joints} joints calibrated)")
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
