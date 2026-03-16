"""Live joint calibration — LeRobot-style real-time display.

Continuously reads motor positions and tracks min/max.
Joints auto-mark ✅ DONE when observed range matches expected within 5%.

Ankle motors (foot_top/foot_bottom (upper/lower motors)) are shown in raw motor-space since
joint-space conversion requires offsets we don't have yet. The expected
motor range is computed from URDF joint limits through the linkage forward.

Press Enter (or Ctrl+C in non-TTY) when done to save.

Usage:
    # Calibrate all joints (both CAN buses):
    ros2 run biped_tools calibrate_node --ros-args \
        -p robot_config:=<path/to/robot.yaml>

    # Calibrate a single joint (merges into existing file):
    ros2 run biped_tools calibrate_node --ros-args \
        -p joint:=R_knee

    # Via launch file:
    ros2 launch biped_bringup calibrate.launch.py
    ros2 launch biped_bringup calibrate.launch.py joint:=L_hip_pitch
"""

import yaml
import time
import sys
import os
import rclpy
from rclpy.node import Node

from biped_driver.robstride_dynamics import RobstrideBus, Motor
from biped_driver.robstride_can import (
    ANKLE_PAIRS, ANKLE_ALL, ANKLE_TOP_MOTORS,
    _PITCH_GAIN, _ROLL_GAIN,
    ankle_motor_theoretical_limits,
)

# From URDF: urdf/heavy/robot.urdf joint limits (rad)
# Format: (lower_rad, upper_rad, default_pos_rad)
URDF_LIMITS = {
    "R_hip_pitch":  (-2.21657, 1.04720,  0.2),
    "R_hip_roll":   (-2.26893, 0.20944,  0.0),
    "R_hip_yaw":    (-1.57080, 1.57080,  0.0),
    "R_knee":       ( 0.00000, 2.70526,  0.4),
    "R_foot_top": (-0.87267, 0.52360, -0.2),
    "R_foot_bottom":  (-0.26180, 0.26180,  0.0),
    "L_hip_pitch":  (-1.04720, 2.21657, -0.2),
    "L_hip_roll":   (-0.20944, 2.26893,  0.0),
    "L_hip_yaw":    (-1.57080, 1.57080,  0.0),
    "L_knee":       ( 0.00000, 2.70526,  0.4),
    "L_foot_top": (-0.87267, 0.52360, -0.2),
    "L_foot_bottom":  (-0.26180, 0.26180,  0.0),
}

# Ankle motor names
# ANKLE sets imported from robstride_can


def _ankle_expected_motor_range(side_prefix):
    """Compute expected motor range for ankle motors from URDF joint limits."""
    pitch_urdf = URDF_LIMITS[f"{side_prefix}_foot_top"]
    roll_urdf = URDF_LIMITS[f"{side_prefix}_foot_bottom"]
    pitch_range = pitch_urdf[1] - pitch_urdf[0]
    roll_range = roll_urdf[1] - roll_urdf[0]
    return _PITCH_GAIN * pitch_range + _ROLL_GAIN * roll_range


ANKLE_EXPECTED_RANGE = {
    "R_foot_top": _ankle_expected_motor_range("R"),
    "R_foot_bottom":  _ankle_expected_motor_range("R"),
    "L_foot_top": _ankle_expected_motor_range("L"),
    "L_foot_bottom":  _ankle_expected_motor_range("L"),
}

RANGE_MATCH_THRESHOLD = 0.05


def load_robot_yaml(path):
    """Parse robot.yaml → list of (name, id, model, bus) tuples."""
    with open(path) as f:
        config = yaml.safe_load(f) or {}

    motors = []
    for bus_key, bus_cfg in config.items():
        interface = bus_cfg.get("interface", bus_key)
        for name, mcfg in bus_cfg.get("motors", {}).items():
            model = mcfg["type"].lower()
            if not model.startswith("rs-"):
                model = "rs-" + model[2:]
            motors.append((name, mcfg["id"], model, interface))
    motors.sort(key=lambda m: m[1])
    return motors


class CalibrateNode(Node):
    def __init__(self):
        super().__init__('calibrate_node')

        self.declare_parameter('robot_config', '')
        self.declare_parameter('output_file', 'calibration.yaml')
        self.declare_parameter('joint', '')  # empty = all joints

        self._robot_config = str(self.get_parameter('robot_config').value)
        self._output = str(self.get_parameter('output_file').value)
        self._joint_filter = str(self.get_parameter('joint').value).strip()

        # Find robot.yaml if not provided
        if not self._robot_config:
            candidates = [
                os.path.expanduser("~/biped_lower/deploy/biped_ws/src/biped_bringup/config/robot.yaml"),
            ]
            for c in candidates:
                if os.path.isfile(c):
                    self._robot_config = c
                    break
            else:
                self.get_logger().error("No robot_config found. Pass robot_config param.")
                raise RuntimeError("No robot_config")

    def run_calibration(self):
        # Load motor list from robot.yaml
        all_motors = load_robot_yaml(self._robot_config)
        all_names = [name for name, _, _, _ in all_motors]

        # Validate joint filter
        if self._joint_filter:
            if self._joint_filter not in all_names:
                print(f"\n❌ Unknown joint '{self._joint_filter}'")
                print(f"   Available: {', '.join(all_names)}")
                return
            target_joints = [self._joint_filter]
            # For ankle joints, must calibrate both in the pair together
            for top, bottom in ANKLE_PAIRS.items():
                if self._joint_filter in (top, bottom):
                    target_joints = [top, bottom]
                    print(f"  ℹ️  Ankle joints are coupled — calibrating both: {top}, {bottom}")
                    break
        else:
            target_joints = all_names

        # Determine which motors to enable (target joints only)
        target_motors = [(n, mid, mdl, bus) for n, mid, mdl, bus in all_motors
                         if n in target_joints]

        # Group by bus
        buses_needed = set(bus for _, _, _, bus in target_motors)

        # Open CAN buses
        bus_handles = {}
        for bus_name in sorted(buses_needed):
            bus_motors = {n: Motor(id=mid, model=mdl)
                          for n, mid, mdl, bus in target_motors if bus == bus_name}
            b = RobstrideBus(channel=bus_name, motors=bus_motors)
            b.connect()
            bus_handles[bus_name] = b

        # Motor → bus lookup
        motor_bus = {n: bus for n, _, _, bus in target_motors}

        mode_str = f"joint={self._joint_filter}" if self._joint_filter else "all joints"
        print(f"\nCalibrating: {mode_str}")
        print(f"Buses: {', '.join(sorted(buses_needed))}")
        print(f"Config: {self._robot_config}")
        print(f"\nEnabling {len(target_motors)} motors (zero torque)...")

        for name, mid, _, bus_name in target_motors:
            b = bus_handles[bus_name]
            try:
                b.enable(name)
                time.sleep(0.02)
                b.set_mode(name, 0)
                time.sleep(0.02)
            except Exception as e:
                print(f"  ✗ {name}: {e}")
                for bh in bus_handles.values():
                    bh.disconnect()
                return

        for b in bus_handles.values():
            b.flush_rx()
        print("Motors enabled. Move each joint to both limits.\n")

        # Track raw motor encoder positions
        motor_cur = {n: 0.0 for n in target_joints}
        motor_min = {n: float('inf') for n in target_joints}
        motor_max = {n: float('-inf') for n in target_joints}
        done = {n: False for n in target_joints}

        n_joints = len(target_joints)

        # Compute expected ranges
        expected_range = {}
        for name in target_joints:
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

        import select
        has_tty = sys.stdin.isatty()
        old_settings = None
        if has_tty:
            import tty
            import termios
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        try:
            user_done = False

            while not user_done:
                if has_tty and select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if ch in ('\n', '\r'):
                        user_done = True

                # Read motor positions
                for name, _, _, bus_name in target_motors:
                    if name not in target_joints:
                        continue
                    b = bus_handles[bus_name]
                    try:
                        b.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
                        fb = b.read_operation_frame(name, timeout=0.01)
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
                for name in target_joints:
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

                for name in target_joints:
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
                prompt = "Press ENTER to save." if has_tty else "Ctrl+C to save."
                sys.stdout.write(
                    f"\033[2K  {n_done}/{n_joints} ✅  —  {prompt}\n")
                sys.stdout.flush()

                time.sleep(0.05)

        except KeyboardInterrupt:
            if not has_tty:
                print("\n")
            else:
                print("\n\nAborted — calibration NOT saved.")
                if old_settings:
                    import termios
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                for name, _, _, bus_name in target_motors:
                    try:
                        bus_handles[bus_name].disable(name)
                    except Exception:
                        pass
                for b in bus_handles.values():
                    b.disconnect()
                return
        finally:
            if old_settings:
                import termios
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Disable
        print("\nDisabling motors...")
        for name, _, _, bus_name in target_motors:
            try:
                bus_handles[bus_name].disable(name)
            except Exception:
                pass
        for b in bus_handles.values():
            b.disconnect()

        # Load existing calibration (for merge in single-joint mode)
        existing_cal = {}
        if self._joint_filter and os.path.isfile(self._output):
            try:
                with open(self._output) as f:
                    existing_cal = yaml.safe_load(f) or {}
                print(f"  Merging into existing {self._output}")
            except Exception:
                pass

        # Build calibration for target joints
        new_cal = {}
        for name, _, _, _ in target_motors:
            if name not in target_joints:
                continue

            mn = motor_min[name] if motor_min[name] != float('inf') else 0.0
            mx = motor_max[name] if motor_max[name] != float('-inf') else 0.0

            cal = {
                'motor_min': round(float(mn), 4),
                'motor_max': round(float(mx), 4),
            }

            urdf = URDF_LIMITS.get(name)
            is_ankle = name in ANKLE_ALL

            if is_ankle:
                # Ankle offset: depends on motor direction.
                # dir=+1: pos = encoder - offset → at m_min, pos = cmd_lo
                #   offset = m_min - cmd_lo
                # dir=-1: pos = -(encoder - offset) → at m_min, pos = cmd_hi
                #   offset = m_min + cmd_hi
                is_upper = name in ANKLE_TOP_MOTORS
                cmd_lo, cmd_hi = ankle_motor_theoretical_limits(is_upper)
                existing_dir = existing_cal.get(name, {}).get('direction', 1)
                if existing_dir == -1:
                    cal['offset'] = round(float(mn + cmd_hi), 4)
                else:
                    cal['offset'] = round(float(mn - cmd_lo), 4)
            else:
                if urdf:
                    offset = mn - urdf[0]
                    cal['offset'] = round(float(offset), 4)
                else:
                    cal['offset'] = round(float(mn), 4)



            # Preserve direction from existing calibration
            if name in existing_cal and 'direction' in existing_cal[name]:
                cal['direction'] = existing_cal[name]['direction']

            new_cal[name] = cal

        # Merge: existing + new (new overwrites target joints only)
        calibration = dict(existing_cal)
        calibration.update(new_cal)

        print(f"\nSaving to {self._output}...")
        with open(self._output, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False, sort_keys=False)

        done_count = sum(1 for v in done.values() if v)
        if self._joint_filter:
            print(f"\n✅ Joint '{self._joint_filter}' calibration updated! ({done_count}/{n_joints} ✅)")
        else:
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
