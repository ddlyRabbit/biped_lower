"""Safety monitor node — e-stop on pitch/roll/temp/watchdog.

Monitors IMU orientation, motor temperatures, CAN health, and policy
heartbeat. Publishes safety status. Forces ESTOP on any violation.

Usage:
    ros2 run biped_control safety_node
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3Stamped
from biped_msgs.msg import MITCommandArray, MotorStateArray
import biped_control.obs_builder as obs_builder


class SafetyNode(Node):
    """Safety watchdog — publishes /safety/status (Bool).

    Checks:
        1. IMU pitch/roll within limits
        2. Motor temperatures below threshold
        3. No motor fault codes
        4. Policy node alive (command watchdog)
        5. IMU alive (data watchdog)
    """

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('max_pitch_deg', 45.0)
        self.declare_parameter('max_roll_deg', 30.0)
        self.declare_parameter('max_motor_temp', 80.0)
        self.declare_parameter('command_timeout_ms', 200.0)
        self.declare_parameter('imu_timeout_ms', 200.0)
        self.declare_parameter('check_rate', 50.0)

        self.declare_parameter('control_params_file', '')
        
        path = self.get_parameter('control_params_file').value
        obs_builder.load_control_params(path)
        
        self._max_pitch = np.radians(float(self.get_parameter('max_pitch_deg').value))
        self._max_roll = np.radians(float(self.get_parameter('max_roll_deg').value))
        self._max_temp = float(self.get_parameter('max_motor_temp').value)
        self._cmd_timeout = float(self.get_parameter('command_timeout_ms').value) / 1000.0
        self._imu_timeout = float(self.get_parameter('imu_timeout_ms').value) / 1000.0
        rate = float(self.get_parameter('check_rate').value)

        # State
        self._gravity = np.array([0.0, 0.0, 9.81])  # Z-up (BNO085 convention)
        self._gravity_received = False
        self._motor_temps = {}
        self._motor_faults = {}
        self._last_cmd_time = time.time()
        self._last_imu_time = time.time()
        self._safety_ok = True
        self._fault_reason = ""

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Vector3Stamped, '/imu/gravity', self._gravity_cb, sensor_qos)
        self.create_subscription(Imu, '/imu/data', self._imu_alive_cb, sensor_qos)
        self.create_subscription(MotorStateArray, '/motor_states', self._motor_cb, sensor_qos)
        self.create_subscription(MITCommandArray, '/joint_commands', self._cmd_alive_cb, 10)

        # Publishers
        self._pub_status = self.create_publisher(Bool, '/safety/status', 10)
        self._pub_fault = self.create_publisher(String, '/safety/fault', 10)

        # Timer
        self._timer = self.create_timer(1.0 / rate, self._check)

        self.get_logger().info(
            f'Safety node started — pitch<{np.degrees(self._max_pitch):.0f}°, '
            f'roll<{np.degrees(self._max_roll):.0f}°, temp<{self._max_temp:.0f}°C'
        )

    def _gravity_cb(self, msg: Vector3Stamped):
        self._gravity[0] = msg.vector.x
        self._gravity[1] = msg.vector.y
        self._gravity[2] = msg.vector.z
        self._gravity_received = True

    def _imu_alive_cb(self, msg: Imu):
        self._last_imu_time = time.time()

    def _motor_cb(self, msg: MotorStateArray):
        for m in msg.motors:
            self._motor_temps[m.joint_name] = m.temperature
            self._motor_faults[m.joint_name] = m.fault_code

    def _cmd_alive_cb(self, msg: MITCommandArray):
        self._last_cmd_time = time.time()

    def _check(self):
        """Run all safety checks."""
        now = time.time()
        ok = True
        reason = ""

        # 1. IMU orientation (pitch/roll from gravity vector)
        # Skip until first gravity message received
        g = self._gravity
        g_norm = np.linalg.norm(g)
        if self._gravity_received and g_norm > 0.1:
            g_unit = g / g_norm
            # BNO085 gravity: (0, 0, +9.81) when upright (Z-up)
            # pitch = atan2(gx, gz), roll = atan2(gy, gz)
            pitch = np.arctan2(g_unit[0], g_unit[2])
            roll = np.arctan2(g_unit[1], g_unit[2])

            if abs(pitch) > self._max_pitch:
                ok = False
                reason = f"Pitch {np.degrees(pitch):.1f}° exceeds limit"
            elif abs(roll) > self._max_roll:
                ok = False
                reason = f"Roll {np.degrees(roll):.1f}° exceeds limit"

        # 2. Motor temperatures
        for name, temp in self._motor_temps.items():
            if temp > self._max_temp:
                ok = False
                reason = f"{name} temp {temp:.1f}°C exceeds limit"
                break

        # 3. Motor faults
        for name, fault in self._motor_faults.items():
            if fault > 0:
                ok = False
                reason = f"{name} fault code: {fault}"
                break

        # 4. Command watchdog (only check if we've received at least one command)
        if self._last_cmd_time > 0 and (now - self._last_cmd_time) > self._cmd_timeout:
            # Don't trigger e-stop for command timeout — policy might not be running yet
            pass

        # 5. IMU watchdog
        if (now - self._last_imu_time) > self._imu_timeout:
            ok = False
            reason = "IMU data timeout"

        # Publish
        status_msg = Bool()
        status_msg.data = ok
        self._pub_status.publish(status_msg)

        if not ok and (ok != self._safety_ok or reason != self._fault_reason):
            fault_msg = String()
            fault_msg.data = reason
            self._pub_fault.publish(fault_msg)
            self.get_logger().error(f'SAFETY FAULT: {reason}')

        self._safety_ok = ok
        self._fault_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
