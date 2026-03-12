"""Live joint calibration — LeRobot-style real-time display.

Continuously reads all motor positions and tracks min/max.
Ankle joints (foot_pitch/foot_roll) use the parallel linkage inverse
to display and validate in joint-space, matching what the policy sees.

Joints auto-mark ✅ DONE when observed range matches URDF within 20%.
Press Ctrl+C when all joints are done (or early) to save calibration.yaml.

Usage:
    ros2 run biped_tools calibrate_node --ros-args \
        -p can_backend:=waveshare -p can_channel:=/dev/ttyUSB0
"""

import yaml
import time
import sys
import rclpy
from rclpy.node import Node

from biped_driver.robstride_dynamics import RobstrideBus, Motor
from biped_driver.robstride_can import (
    ankle_motors_to_feedback,
    ANKLE_PAIRS,
)

# URDF joint limits: (lower_rad, upper_rad, default_pos_rad)
# These are all in JOINT-SPACE (what the policy sees)
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

# Motor list (CAN order)
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

# Display order: all joints the policy sees (12 joints)
# Same as MOTOR_LIST — ankle joints show joint-space values via linkage inverse
DISPLAY_JOINTS = [name for name, _, _ in MOTOR_LIST]

RANGE_MATCH_THRESHOLD = 0.20  # 20%


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

        # Raw motor encoder positions (always tracked)
        motor_cur = {n: 0.0 for n, _, _ in MOTOR_LIST}
        motor_min = {n: float('inf') for n, _, _ in MOTOR_LIST}
        motor_max = {n: float('-inf') for n, _, _ in MOTOR_LIST}

        # Joint-space tracking (for display + done check)
        # For normal joints: same as motor. For ankle: computed via linkage.
        joint_cur = {n: 0.0 for n in DISPLAY_JOINTS}
        joint_min = {n: float('inf') for n in DISPLAY_JOINTS}
        joint_max = {n: float('-inf') for n in DISPLAY_JOINTS}
        done = {n: False for n in DISPLAY_JOINTS}

        n_joints = len(DISPLAY_JOINTS)

        # Header
        print("=" * 95)
        print(f"  {'Joint':<16} {'Cur':>7}  {'Min':>7}  {'Max':>7}  "
              f"{'Range':>7}  {'URDF':>7}  {'Err%':>5}  Status")
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

                # Compute joint-space positions
                for name, _, _ in MOTOR_LIST:
                    pair = ANKLE_PAIRS.get(name)
                    if pair:
                        # This is a pitch joint — compute both pitch and roll
                        roll_name = pair
                        pitch_pos, roll_pos = ankle_motors_to_feedback(
                            motor_cur[name], motor_cur[roll_name])
                        joint_cur[name] = pitch_pos
                        joint_cur[roll_name] = roll_pos

                        # Track joint-space min/max for pitch
                        if pitch_pos < joint_min[name]:
                            joint_min[name] = pitch_pos
                        if pitch_pos > joint_max[name]:
                            joint_max[name] = pitch_pos
                        # Track joint-space min/max for roll
                        if roll_pos < joint_min[roll_name]:
                            joint_min[roll_name] = roll_pos
                        if roll_pos > joint_max[roll_name]:
                            joint_max[roll_name] = roll_pos
                    elif name not in ANKLE_PAIRS.values():
                        # Normal joint — motor = joint
                        joint_cur[name] = motor_cur[name]
                        joint_min[name] = motor_min[name]
                        joint_max[name] = motor_max[name]
                    # else: roll joint — already handled by its pitch pair above

                # Check done
                n_done = 0
                for name in DISPLAY_JOINTS:
                    if done[name]:
                        n_done += 1
                        continue
                    urdf = URDF_LIMITS.get(name)
                    if not urdf:
                        continue
                    jmin = joint_min[name]
                    jmax = joint_max[name]
                    if jmin == float('inf') or jmax == float('-inf'):
                        continue
                    observed = jmax - jmin
                    urdf_range = urdf[1] - urdf[0]
                    if urdf_range > 0 and observed > 0.05:
                        err = abs(observed - urdf_range) / urdf_range
                        if err <= RANGE_MATCH_THRESHOLD:
                            done[name] = True
                            n_done += 1

                # Render
                sys.stdout.write(f"\033[{n_joints + 2}A")

                for name in DISPLAY_JOINTS:
                    cur = joint_cur[name]
                    mn = joint_min[name] if joint_min[name] != float('inf') else 0.0
                    mx = joint_max[name] if joint_max[name] != float('-inf') else 0.0
                    observed = mx - mn

                    urdf = URDF_LIMITS.get(name)
                    if urdf:
                        urdf_range = urdf[1] - urdf[0]
                        err_pct = (abs(observed - urdf_range) / urdf_range * 100
                                   if urdf_range > 0 and observed > 0.01 else 999.9)
                        urdf_str = f"{urdf_range:6.3f}"
                    else:
                        err_pct = 0.0
                        urdf_str = "  —   "

                    is_ankle = name in ANKLE_PAIRS or name in ANKLE_PAIRS.values()
                    tag = " [L]" if is_ankle else "    "

                    if done[name]:
                        status = "✅ DONE"
                    elif observed > 0.05:
                        status = "🔄 moving..."
                    else:
                        status = "⏳ waiting"

                    line = (f"  {name:<16}{cur:+7.3f}  {mn:+7.3f}  {mx:+7.3f}  "
                            f"{observed:6.3f}  {urdf_str}  {err_pct:5.1f}  {status}{tag}")
                    sys.stdout.write(f"\033[2K{line}\n")

                sys.stdout.write(f"\033[2K\n")
                sys.stdout.write(
                    f"\033[2K  {n_done}/{n_joints} joints done. "
                    f"[L]=linkage joint-space. "
                    f"{'Ctrl+C to save.' if n_done == n_joints else 'Keep moving...'}\n")
                sys.stdout.flush()

                if n_done == n_joints:
                    time.sleep(0.5)
                    break

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\nStopped by user.")

        # Disable
        print("\nDisabling motors...")
        for name, _, _ in MOTOR_LIST:
            try:
                bus.disable(name)
            except Exception:
                pass
        bus.disconnect()

        # Save calibration
        # Motor-space offsets (what the bus actually uses)
        # Joint-space limits (for soft stops)
        calibration = {}
        for name, _, _ in MOTOR_LIST:
            mn_motor = motor_min[name] if motor_min[name] != float('inf') else 0.0
            mx_motor = motor_max[name] if motor_max[name] != float('-inf') else 0.0
            mn_joint = joint_min[name] if joint_min[name] != float('inf') else 0.0
            mx_joint = joint_max[name] if joint_max[name] != float('-inf') else 0.0

            urdf = URDF_LIMITS.get(name)
            is_ankle = name in ANKLE_PAIRS or name in ANKLE_PAIRS.values()

            cal = {
                'motor_min': round(float(mn_motor), 4),
                'motor_max': round(float(mx_motor), 4),
                'limit_lo': round(float(mn_joint), 4),
                'limit_hi': round(float(mx_joint), 4),
            }

            if urdf and not is_ankle:
                # Normal joint: offset maps motor encoder to URDF frame
                offset = mn_motor - urdf[0]
                cal['offset'] = round(float(offset), 4)
            else:
                # Ankle: offset is raw motor-space min (linkage applied at runtime)
                cal['offset'] = round(float(mn_motor), 4)

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
