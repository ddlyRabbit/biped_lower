"""IM10A IMU Node — USB serial, Hiwonder/Hexmove HFI-A9.

Same ROS2 interface as BNO085 imu_node:
  /imu/data     sensor_msgs/Imu          @ 50 Hz
  /imu/gravity  geometry_msgs/Vector3Stamped  @ 50 Hz
  /tf           odom → base_link         @ 50 Hz

Gravity convention matches Isaac Sim: upright robot → gravity = (0, 0, -1).
Derived from quaternion, not raw accel (avoids dynamics contamination).

Hardware: IM10A on USB (/dev/ttyUSB0), CP210x bridge, 921600 baud.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from tf2_ros import TransformBroadcaster

from biped_driver.im10a_driver import IM10ADriver


class IM10ANode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('rate_hz', 300.0)
        self.declare_parameter('frame_id', 'imu_link')

        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baudrate').value)
        self._rate = float(self.get_parameter('rate_hz').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers (same topics as BNO085 node)
        self._pub_imu = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self._pub_gravity = self.create_publisher(Vector3Stamped, '/imu/gravity', sensor_qos)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Driver
        self._driver = IM10ADriver(port=port, baudrate=baud)
        try:
            self._driver.open()
            self.get_logger().info(
                f'IM10A IMU opened — {port} @ {baud}, {self._rate}Hz'
            )
        except Exception as e:
            self.get_logger().error(f'IM10A init failed: {e}')
            self._driver = None

        # State
        self._last_quat = (0.0, 0.0, 0.0, 1.0)  # (x, y, z, w)
        self._last_gyro = (0.0, 0.0, 0.0)
        self._last_gravity = (0.0, 0.0, -1.0)
        self._read_count = 0

        # Reconnect state
        self._port = port
        self._baud = baud
        self._reconnect_interval = 2.0  # seconds between reconnect attempts
        self._last_reconnect_attempt = 0.0

        # Timer
        self._timer = self.create_timer(1.0 / self._rate, self._timer_callback)

    def _try_reconnect(self):
        """Attempt to reopen serial port after I/O error."""
        import time
        now = time.time()
        if now - self._last_reconnect_attempt < self._reconnect_interval:
            return
        self._last_reconnect_attempt = now
        try:
            self._driver = IM10ADriver(port=self._port, baudrate=self._baud)
            self._driver.open()
            self.get_logger().info(f'IMU reconnected — {self._port} @ {self._baud}')
        except Exception as e:
            self.get_logger().warn(f'IMU reconnect failed: {e}')
            self._driver = None

    def _timer_callback(self):
        if self._driver is None:
            self._try_reconnect()
            return

        # Read all available frames (drain buffer)
        data = None
        try:
            while True:
                d = self._driver.read()
                if d is None:
                    break
                data = d  # keep latest
        except OSError as e:
            self.get_logger().error(f'IMU serial error: {e} — will reconnect')
            try:
                self._driver.close()
            except Exception:
                pass
            self._driver = None
            return

        if data is not None:
            # quaternion: driver returns (w, x, y, z), ROS wants (x, y, z, w)
            self._last_quat = (data.quaternion[1], data.quaternion[2],
                               data.quaternion[3], data.quaternion[0])
            self._last_gyro = tuple(data.gyro)
            # Gravity from driver: already in Isaac convention (upright → [0, 0, -1])
            self._last_gravity = (
                data.gravity[0],
                data.gravity[1],
                data.gravity[2],
            )
            self._read_count += 1

        now = self.get_clock().now().to_msg()

        # /imu/data
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self._frame_id
        imu_msg.orientation.x = self._last_quat[0]
        imu_msg.orientation.y = self._last_quat[1]
        imu_msg.orientation.z = self._last_quat[2]
        imu_msg.orientation.w = self._last_quat[3]
        imu_msg.angular_velocity.x = self._last_gyro[0]
        imu_msg.angular_velocity.y = self._last_gyro[1]
        imu_msg.angular_velocity.z = self._last_gyro[2]
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self._pub_imu.publish(imu_msg)

        # /imu/gravity
        grav_msg = Vector3Stamped()
        grav_msg.header.stamp = now
        grav_msg.header.frame_id = self._frame_id
        grav_msg.vector.x = self._last_gravity[0]
        grav_msg.vector.y = self._last_gravity[1]
        grav_msg.vector.z = self._last_gravity[2]
        self._pub_gravity.publish(grav_msg)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.x = self._last_quat[0]
        t.transform.rotation.y = self._last_quat[1]
        t.transform.rotation.z = self._last_quat[2]
        t.transform.rotation.w = self._last_quat[3]
        self._tf_broadcaster.sendTransform(t)

        if self._read_count > 0 and self._read_count % 10000 == 0:
            self.get_logger().info(f'IM10A: {self._read_count} reads')

    def destroy_node(self):
        if self._driver:
            self._driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IM10ANode()
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
