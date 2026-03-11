"""BNO085 IMU Node — I2C, SH2 protocol via adafruit-circuitpython-bno08x.

Publishes fused orientation (quaternion) from ROTATION_VECTOR,
calibrated angular velocity from GYROSCOPE, and gravity from GRAVITY.

Requires: adafruit-circuitpython-bno08x
  pip install adafruit-circuitpython-bno08x adafruit-blinka

Hardware: BNO085 on I2C bus 1, address 0x4A (default).
  RPi 5 GPIO 2 (SDA) / GPIO 3 (SCL), 400kHz.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


class BNO085Node(Node):
    """ROS2 node for BNO085 IMU over I2C.

    Publishes:
      /imu/data     sensor_msgs/Imu          @ rate_hz (default 100)
      /imu/gravity  geometry_msgs/Vector3Stamped  @ rate_hz

    /imu/data contains:
      - orientation: quaternion from SH2_ROTATION_VECTOR (fused, mag-corrected)
      - angular_velocity: from SH2_GYROSCOPE_CALIBRATED (rad/s, body frame)
      - linear_acceleration: zeros (gravity published separately)

    /imu/gravity contains the raw gravity vector (m/s²) from SH2_GRAVITY.
    Policy node normalizes this for projected_gravity observation.
    """

    def __init__(self):
        super().__init__('imu_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x4A)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('use_game_quaternion', False)  # True = no mag correction

        self._bus = int(self.get_parameter('i2c_bus').value)
        self._addr = int(self.get_parameter('i2c_address').value)
        self._rate = float(self.get_parameter('rate_hz').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._use_game_quat = bool(self.get_parameter('use_game_quaternion').value)

        # QoS: best-effort for real-time sensor data
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # Publishers
        self._pub_imu = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self._pub_gravity = self.create_publisher(
            Vector3Stamped, '/imu/gravity', sensor_qos
        )

        # Initialize BNO085
        self._bno = None
        self._init_imu()

        # State — latest readings
        self._last_quat = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w) ROS convention
        self._last_gyro = (0.0, 0.0, 0.0)         # rad/s body frame
        self._last_gravity = (0.0, 0.0, -9.81)     # m/s²

        # Diagnostics
        self._read_count = 0
        self._error_count = 0

        # Timer at configured rate
        period = 1.0 / self._rate
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'BNO085 IMU started — I2C bus {self._bus}, addr 0x{self._addr:02X}, '
            f'{self._rate}Hz, game_quat={self._use_game_quat}'
        )

    def _init_imu(self):
        """Initialize BNO085 over I2C and enable sensor reports."""
        try:
            import board
            import busio
            from adafruit_bno08x.i2c import BNO08X_I2C
            from adafruit_bno08x import (
                BNO_REPORT_ROTATION_VECTOR,
                BNO_REPORT_GAME_ROTATION_VECTOR,
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_GRAVITY,
            )

            i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self._bno = BNO08X_I2C(i2c, address=self._addr)

            # Enable reports at configured interval
            interval_us = int(1_000_000 / self._rate)

            # Quaternion: rotation_vector (mag-corrected) or game_rotation_vector (no mag)
            if self._use_game_quat:
                self._bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, interval_us)
                self.get_logger().info('Using GAME_ROTATION_VECTOR (no mag correction)')
            else:
                self._bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, interval_us)
                self.get_logger().info('Using ROTATION_VECTOR (mag-corrected)')

            # Calibrated gyroscope: direct angular velocity (rad/s)
            self._bno.enable_feature(BNO_REPORT_GYROSCOPE, interval_us)

            # Gravity vector
            self._bno.enable_feature(BNO_REPORT_GRAVITY, interval_us)

            self.get_logger().info(
                f'BNO085 initialized — reports enabled at {interval_us}μs interval'
            )

        except ImportError as e:
            self.get_logger().error(
                f'Missing dependency: {e}. '
                'Install: pip install adafruit-circuitpython-bno08x adafruit-blinka'
            )
            self._bno = None
        except Exception as e:
            self.get_logger().error(f'BNO085 init failed: {e}')
            self._bno = None

    def _read_sensor(self):
        """Read all enabled sensor reports. Non-blocking — uses latest available data."""
        if self._bno is None:
            return

        try:
            # Quaternion
            if self._use_game_quat:
                quat = self._bno.game_quaternion  # (i, j, k, real)
            else:
                quat = self._bno.quaternion  # (i, j, k, real)

            if quat is not None:
                qi, qj, qk, qreal = quat
                # SH2 convention: (i, j, k, real) → ROS convention: (x, y, z, w)
                self._last_quat = (qi, qj, qk, qreal)

            # Gyroscope (calibrated, rad/s)
            gyro = self._bno.gyro  # (x, y, z) rad/s
            if gyro is not None:
                self._last_gyro = gyro

            # Gravity (m/s²)
            gravity = self._bno.gravity  # (x, y, z) m/s²
            if gravity is not None:
                self._last_gravity = gravity

            self._read_count += 1

        except Exception as e:
            self._error_count += 1
            if self._error_count % 100 == 1:
                self.get_logger().warn(
                    f'IMU read error ({self._error_count} total): {e}'
                )

    def _timer_callback(self):
        """Read sensor and publish messages."""
        self._read_sensor()

        now = self.get_clock().now().to_msg()

        # --- Publish /imu/data ---
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self._frame_id

        # Orientation (quaternion, ROS convention: x, y, z, w)
        imu_msg.orientation.x = self._last_quat[0]
        imu_msg.orientation.y = self._last_quat[1]
        imu_msg.orientation.z = self._last_quat[2]
        imu_msg.orientation.w = self._last_quat[3]

        # Angular velocity (rad/s, body frame — direct from calibrated gyro)
        imu_msg.angular_velocity.x = self._last_gyro[0]
        imu_msg.angular_velocity.y = self._last_gyro[1]
        imu_msg.angular_velocity.z = self._last_gyro[2]

        # Linear acceleration: not populated (use /imu/gravity for projected_gravity)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        # Covariance: -1 in first element = unknown
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self._pub_imu.publish(imu_msg)

        # --- Publish /imu/gravity ---
        grav_msg = Vector3Stamped()
        grav_msg.header.stamp = now
        grav_msg.header.frame_id = self._frame_id
        grav_msg.vector.x = self._last_gravity[0]
        grav_msg.vector.y = self._last_gravity[1]
        grav_msg.vector.z = self._last_gravity[2]

        self._pub_gravity.publish(grav_msg)

        # Periodic diagnostics
        if self._read_count > 0 and self._read_count % 1000 == 0:
            self.get_logger().info(
                f'IMU stats: {self._read_count} reads, {self._error_count} errors, '
                f'gravity=({self._last_gravity[0]:.2f}, {self._last_gravity[1]:.2f}, '
                f'{self._last_gravity[2]:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BNO085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
