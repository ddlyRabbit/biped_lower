"""Manual joint calibration tool.

Interactive CLI: move each joint to both mechanical limits,
records encoder positions, computes offsets, saves calibration.yaml.

Supports socketcan and waveshare backends.

Usage:
    ros2 run biped_tools calibrate_node --ros-args \
        -p can_backend:=waveshare -p can_channel:=/dev/ttyUSB0

    ros2 run biped_tools calibrate_node --ros-args \
        -p can_backend:=socketcan -p can_channel:=can0
"""

import yaml
import time
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

# Motor list in calibration order (right leg first, then left)
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
        print("\n" + "=" * 60)
        print("  BIPED JOINT CALIBRATION")
        print("  Move each joint to both limits when prompted.")
        print(f"  Backend: {self._backend}, Channel: {self._channel}")
        print("=" * 60)

        # Build motor dict for the bus
        motors = {}
        for name, mid, model in MOTOR_LIST:
            motors[name] = Motor(id=mid, model=model)

        bus = RobstrideBus(
            channel=self._channel,
            motors=motors,
            backend=self._backend,
        )
        bus.connect()

        # Enable all motors in zero-torque mode (free to move by hand)
        print(f"\nEnabling {len(MOTOR_LIST)} motors (zero torque — free to move)...")
        for name, mid, model in MOTOR_LIST:
            try:
                bus.enable(name)
                time.sleep(0.02)
                bus.set_mode(name, 0)  # MIT mode
                time.sleep(0.02)
                print(f"  ✓ {name} (ID={mid})")
            except Exception as e:
                print(f"  ✗ {name}: {e}")
                bus.disconnect()
                return

        bus.flush_rx()
        print("\nAll motors enabled. Joints are free to move.\n")

        calibration = {}

        for name, mid, model in MOTOR_LIST:
            urdf = URDF_LIMITS.get(name)

            print(f"--- {name} (ID={mid}, {model}) ---")
            if urdf:
                print(f"  URDF range: [{urdf[0]:.3f}, {urdf[1]:.3f}] rad")

            # Read current position
            current = self._read_pos(bus, name)
            print(f"  Current position: {current:.4f} rad")

            # Limit A
            input(f"  → Move {name} to LIMIT A (one end), press Enter...")
            limit_a = self._read_pos(bus, name)
            print(f"    Limit A: {limit_a:.4f} rad")

            # Limit B
            input(f"  → Move {name} to LIMIT B (other end), press Enter...")
            limit_b = self._read_pos(bus, name)
            print(f"    Limit B: {limit_b:.4f} rad")

            # Compute
            lo = min(limit_a, limit_b)
            hi = max(limit_a, limit_b)
            encoder_range = hi - lo

            cal_entry = {
                'encoder_limit_a': round(float(limit_a), 4),
                'encoder_limit_b': round(float(limit_b), 4),
                'limit_lo': round(float(lo), 4),
                'limit_hi': round(float(hi), 4),
            }

            if urdf and name not in ANKLE_JOINTS:
                urdf_range = urdf[1] - urdf[0]
                range_error = abs(encoder_range - urdf_range) / urdf_range * 100
                offset = lo - urdf[0]

                cal_entry['offset'] = round(float(offset), 4)
                cal_entry['range_error_pct'] = round(float(range_error), 1)
                cal_entry['urdf_lower'] = round(urdf[0], 4)
                cal_entry['urdf_upper'] = round(urdf[1], 4)
                cal_entry['default_pos'] = round(urdf[2], 4)

                print(f"    Range: {encoder_range:.3f} rad (URDF: {urdf_range:.3f}, err: {range_error:.1f}%)")
                print(f"    Offset: {offset:.4f} rad")
                if range_error > 15:
                    print(f"    ⚠️  Range error >15%!")
            elif name in ANKLE_JOINTS:
                # Ankle motors: offset is motor-space, linkage at runtime
                offset = lo
                cal_entry['offset'] = round(float(offset), 4)
                print(f"    [ANKLE] Motor range: {encoder_range:.3f} rad")
                print(f"    Offset: {offset:.4f} rad (motor-space)")
            else:
                offset = lo
                cal_entry['offset'] = round(float(offset), 4)
                print(f"    Range: {encoder_range:.3f} rad, offset: {offset:.4f}")

            calibration[name] = cal_entry
            print()

        # Disable all motors
        print("Disabling motors...")
        for name, _, _ in MOTOR_LIST:
            try:
                bus.disable(name)
            except Exception:
                pass
        bus.disconnect()

        # Save
        print(f"\nSaving to {self._output}...")
        with open(self._output, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False, sort_keys=False)

        print(f"\n✅ Calibration complete! ({len(calibration)} joints)")
        print(f"   File: {self._output}")
        print(f"   Use: ros2 launch biped_bringup hardware.launch.py "
              f"calibration_file:={self._output} bus_mode:=waveshare")

    def _read_pos(self, bus: RobstrideBus, name: str, samples: int = 10) -> float:
        """Read position averaged over samples using zero-torque MIT frames."""
        positions = []
        for _ in range(samples):
            try:
                bus.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
                fb = bus.read_operation_frame(name, timeout=0.02)
                if fb:
                    positions.append(fb.position)
            except Exception:
                pass
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
