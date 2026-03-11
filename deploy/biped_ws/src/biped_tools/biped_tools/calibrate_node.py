"""Manual joint calibration tool.

Interactive CLI: operator moves each joint to both limits while this
node records absolute encoder positions. Computes URDF-aligned zero
offsets and saves to calibration.yaml.

Only needs to run once (or after motor reassembly).

Usage:
    ros2 run biped_tools calibrate_node --ros-args \
        -p motor_config:="L_hip_pitch:1:RS04,L_hip_roll:2:RS03,..." \
        -p can_interface:=can0 \
        -p output_file:=calibration.yaml
"""

import yaml
import time
import sys
import rclpy
from rclpy.node import Node
from biped_driver.robstride_can import RobStrideMotor

# URDF joint limits (from biped_env_cfg.py init_state + URDF)
# These are the expected ranges from the URDF
URDF_LIMITS = {
    # joint_name: (lower_limit_rad, upper_limit_rad, default_pos_rad)
    "L_hip_pitch":  (-2.222, 1.047, -0.2),
    "R_hip_pitch":  (-1.047, 2.222,  0.2),
    "L_hip_roll":   (-0.209, 2.269,  0.0),
    "R_hip_roll":   (-2.269, 0.209,  0.0),
    "L_hip_yaw":    (-2.094, 2.094,  0.0),
    "R_hip_yaw":    (-2.094, 2.094,  0.0),
    "L_knee":       ( 0.000, 2.618,  0.4),
    "R_knee":       ( 0.000, 2.618,  0.4),
    "L_foot_pitch": (-1.047, 0.524, -0.2),
    "R_foot_pitch": (-1.047, 0.524, -0.2),
    "L_foot_roll":  (-0.262, 0.262,  0.0),
    "R_foot_roll":  (-0.262, 0.262,  0.0),
}


class CalibrateNode(Node):
    def __init__(self):
        super().__init__('calibrate_node')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('motor_config', '')
        self.declare_parameter('master_id', 253)
        self.declare_parameter('output_file', 'calibration.yaml')

        self._iface = str(self.get_parameter('can_interface').value)
        self._master_id = int(self.get_parameter('master_id').value)
        self._output = str(self.get_parameter('output_file').value)
        config_str = str(self.get_parameter('motor_config').value)

        # Parse motor config
        self._joint_names = []
        self._motor_ids = {}
        self._motor_types = {}

        for entry in config_str.split(','):
            entry = entry.strip()
            if not entry:
                continue
            parts = entry.split(':')
            if len(parts) == 3:
                name, mid, mtype = parts[0].strip(), int(parts[1].strip()), parts[2].strip()
                self._joint_names.append(name)
                self._motor_ids[name] = mid
                self._motor_types[name] = mtype

    def run_calibration(self):
        """Interactive calibration procedure."""
        print("\n" + "=" * 60)
        print("  BIPED JOINT CALIBRATION")
        print("  Move each joint to both limits when prompted.")
        print("  All motors are in zero-torque mode (free to move).")
        print("=" * 60)

        if not self._joint_names:
            print("\nERROR: No motors configured. Set motor_config parameter.")
            return

        calibration = {}
        motors = {}

        # Connect to all motors
        print(f"\nConnecting to {len(self._joint_names)} motors on {self._iface}...")
        for name in self._joint_names:
            try:
                motor = RobStrideMotor(
                    self._iface, self._motor_ids[name],
                    self._motor_types[name], self._master_id,
                )
                motor.enable()
                motors[name] = motor
                print(f"  ✓ {name} (ID={self._motor_ids[name]}, {self._motor_types[name]})")
            except Exception as e:
                print(f"  ✗ {name}: {e}")
                return

        print("\nAll motors enabled in zero-torque mode.\n")

        # Calibrate each joint
        for name in self._joint_names:
            motor = motors[name]
            urdf = URDF_LIMITS.get(name)

            print(f"--- {name} ---")
            if urdf:
                print(f"  URDF range: [{urdf[0]:.3f}, {urdf[1]:.3f}] rad, default: {urdf[2]:.3f}")

            # Read current position
            current = self._read_position_avg(motor)
            print(f"  Current encoder position: {current:.4f} rad")

            # Limit A
            input(f"  → Move {name} to LIMIT A (one end), then press Enter...")
            limit_a = self._read_position_avg(motor)
            print(f"    Recorded LIMIT A: {limit_a:.4f} rad")

            # Limit B
            input(f"  → Move {name} to LIMIT B (other end), then press Enter...")
            limit_b = self._read_position_avg(motor)
            print(f"    Recorded LIMIT B: {limit_b:.4f} rad")

            # Compute
            encoder_range = abs(limit_b - limit_a)
            encoder_min = min(limit_a, limit_b)
            encoder_max = max(limit_a, limit_b)

            if urdf:
                urdf_range = urdf[1] - urdf[0]
                range_error = abs(encoder_range - urdf_range) / urdf_range * 100

                # Offset: encoder_position_at_urdf_lower_limit
                # offset = encoder_min - urdf_lower_limit
                offset = encoder_min - urdf[0]

                print(f"    Encoder range: {encoder_range:.4f} rad")
                print(f"    URDF range:    {urdf_range:.4f} rad")
                print(f"    Range error:   {range_error:.1f}%")
                print(f"    Offset:        {offset:.4f} rad")

                if range_error > 15:
                    print(f"    ⚠️  Range error >15% — check joint limits!")
            else:
                offset = encoder_min
                range_error = 0.0
                print(f"    No URDF limits defined for {name}")
                print(f"    Offset set to encoder_min: {offset:.4f}")

            calibration[name] = {
                'encoder_limit_a': round(float(limit_a), 4),
                'encoder_limit_b': round(float(limit_b), 4),
                'encoder_min': round(float(encoder_min), 4),
                'encoder_max': round(float(encoder_max), 4),
                'offset': round(float(offset), 4),
                'range_error_pct': round(float(range_error), 1),
            }
            if urdf:
                calibration[name]['urdf_lower'] = round(urdf[0], 4)
                calibration[name]['urdf_upper'] = round(urdf[1], 4)
                calibration[name]['default_pos'] = round(urdf[2], 4)

            print()

        # Disable motors
        for motor in motors.values():
            motor.disable()
            motor.close()

        # Save calibration
        print(f"Saving calibration to {self._output}...")
        with open(self._output, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False, sort_keys=False)

        print("\n✓ Calibration complete!")
        print(f"  File: {self._output}")
        print(f"  Joints: {len(calibration)}")
        print("\n  Use with can_bus_node:")
        print(f"    -p calibration_file:={self._output}")

    def _read_position_avg(self, motor: RobStrideMotor, samples: int = 10) -> float:
        """Read position averaged over multiple samples for stability."""
        positions = []
        for _ in range(samples):
            fb = motor.send_mit_command(kp=0.0, kd=0.0, torque_ff=0.0)
            if fb:
                positions.append(fb.position)
            time.sleep(0.01)

        if not positions:
            return 0.0
        return sum(positions) / len(positions)


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
