"""IM10A (Hiwonder/Hexmove HFI-A9) IMU driver — USB serial.

Protocol: Two-frame packets over serial (921600 baud, CP210x USB-UART).
  Frame 1 (49B): 0xAA 0x55 0x2C | 4B flags | 4B timestamp | 9×float32 (gyro, accel, mag)
  Frame 2 (25B): 0xAA 0x55 0x14 | 4B flags | 4B timestamp | 3×float32 (roll, pitch, yaw)

Reference: MRPT CTaoboticsIMU (parser_hfi_a9), K-Scale kos-kbot hexmove driver.
"""

import struct
import time
import numpy as np
from scipy.spatial.transform import Rotation


# Frame headers
FRAME1_HEADER = bytes([0xAA, 0x55, 0x2C])  # 49 bytes: gyro + accel + mag
FRAME2_HEADER = bytes([0xAA, 0x55, 0x14])  # 25 bytes: euler angles
FRAME1_LEN = 49
FRAME2_LEN = 25

# Gravity constant
G = 9.81


class IM10AData:
    """Parsed IMU data from one frame pair."""
    __slots__ = ['gyro', 'accel', 'mag', 'euler', 'quaternion', 'gravity', 'timestamp']

    def __init__(self):
        self.gyro = np.zeros(3)      # rad/s (wx, wy, wz)
        self.accel = np.zeros(3)     # m/s² (ax, ay, az)
        self.mag = np.zeros(3)       # magnetic field
        self.euler = np.zeros(3)     # rad (roll, pitch, yaw)
        self.quaternion = np.array([1.0, 0, 0, 0])  # (w, x, y, z)
        self.gravity = np.array([0.0, 0.0, -1.0])   # projected gravity (Isaac convention)
        self.timestamp = 0.0


class IM10ADriver:
    """Serial driver for IM10A IMU.

    Usage:
        driver = IM10ADriver('/dev/ttyUSB0')
        driver.open()
        while True:
            data = driver.read()
            if data:
                print(data.gyro, data.quaternion, data.gravity)
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 921600):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._buf = bytearray()
        self._pending_frame1 = None  # partial: frame1 parsed, waiting for frame2

    def open(self):
        """Open serial port."""
        import serial
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.01,  # 10ms read timeout
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._serial.reset_input_buffer()
        self._buf.clear()

    def close(self):
        """Close serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def read(self) -> IM10AData | None:
        """Read and parse one complete frame pair.

        Returns IM10AData if a complete pair was parsed, None otherwise.
        Call in a loop at >= 100Hz.
        """
        if not self._serial or not self._serial.is_open:
            return None

        # Read available bytes
        avail = self._serial.in_waiting
        if avail > 0:
            self._buf.extend(self._serial.read(min(avail, 1024)))

        # Try to parse frames
        return self._parse()

    def _parse(self) -> IM10AData | None:
        """Parse buffered bytes for frame pairs."""
        while True:
            if len(self._buf) < 3:
                return None

            # Look for frame headers
            if self._buf[:3] == FRAME1_HEADER:
                if len(self._buf) < FRAME1_LEN:
                    return None  # need more bytes
                frame = bytes(self._buf[:FRAME1_LEN])
                del self._buf[:FRAME1_LEN]
                self._pending_frame1 = self._parse_frame1(frame)

            elif self._buf[:3] == FRAME2_HEADER:
                if len(self._buf) < FRAME2_LEN:
                    return None
                frame = bytes(self._buf[:FRAME2_LEN])
                del self._buf[:FRAME2_LEN]

                if self._pending_frame1 is not None:
                    data = self._pending_frame1
                    self._parse_frame2(frame, data)
                    self._pending_frame1 = None
                    return data
                # else: orphan frame2, discard

            else:
                # Not a valid header — discard one byte and resync
                del self._buf[0]

    def _parse_frame1(self, frame: bytes) -> IM10AData:
        """Parse 49-byte frame: gyro + accel + mag."""
        # Bytes 3-6: flags, 7-10: timestamp, 11-46: 9 floats (little-endian)
        data = IM10AData()
        data.timestamp = time.time()

        floats = struct.unpack_from('<9f', frame, 11)
        # gyro: rad/s (raw from IMU)
        data.gyro[0] = floats[0]  # wx
        data.gyro[1] = floats[1]  # wy
        data.gyro[2] = floats[2]  # wz
        # accel: raw is in g-units, convert to m/s²
        data.accel[0] = floats[3] * G
        data.accel[1] = floats[4] * G
        data.accel[2] = floats[5] * G
        # mag
        data.mag[0] = floats[6]
        data.mag[1] = floats[7]
        data.mag[2] = floats[8]

        return data

    def _parse_frame2(self, frame: bytes, data: IM10AData):
        """Parse 25-byte frame: euler angles → quaternion + gravity."""
        # Bytes 3-6: flags, 7-10: timestamp, 11-22: 3 floats (roll, pitch, yaw) in degrees
        floats = struct.unpack_from('<3f', frame, 11)
        # Convert degrees to radians
        data.euler[0] = np.radians(floats[0])  # roll
        data.euler[1] = np.radians(floats[1])  # pitch
        data.euler[2] = np.radians(floats[2])  # yaw

        # Euler → quaternion (scipy uses scalar-last: [x, y, z, w])
        r = Rotation.from_euler('xyz', data.euler)
        quat_xyzw = r.as_quat()  # [x, y, z, w]
        data.quaternion[0] = quat_xyzw[3]  # w
        data.quaternion[1] = quat_xyzw[0]  # x
        data.quaternion[2] = quat_xyzw[1]  # y
        data.quaternion[3] = quat_xyzw[2]  # z

        # Projected gravity in body frame (Isaac convention: upright → [0, 0, -1])
        # Same as K-Scale: r.apply([0, 0, -1], inverse=True)
        data.gravity = r.apply(np.array([0.0, 0.0, -1.0]), inverse=True)
