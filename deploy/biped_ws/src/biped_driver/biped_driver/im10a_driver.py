"""IM10A (Hiwonder/Hexmove) IMU driver — USB serial, WIT-motion B6 protocol.

Protocol: 11-byte frames over serial.
  Header: 0x55 0x5X
  Types: 0x50=time, 0x51=accel, 0x52=gyro, 0x53=euler, 0x59=quaternion

Default baud: 9600. Driver auto-detects and upgrades to TARGET_BAUD.
Baud change persists in IMU flash (only done once).

Reference: MRPT CTaoboticsIMU (parser_hfi_b6), WIT-motion protocol docs.
"""

import struct
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation

# Frame constants
FRAME_LEN = 11
HEADER = 0x55
TYPE_ACCEL = 0x51
TYPE_GYRO = 0x52
TYPE_EULER = 0x53
TYPE_QUAT = 0x59

# Baud rate config
DEFAULT_BAUD = 9600
TARGET_BAUD = 460800
BAUD_CODES = {
    9600: 0x02, 19200: 0x03, 38400: 0x04, 57600: 0x05,
    115200: 0x06, 230400: 0x07, 460800: 0x08, 921600: 0x09,
}


class IM10AData:
    """Parsed IMU data."""
    __slots__ = ['gyro', 'accel', 'euler', 'quaternion', 'gravity', 'timestamp']

    def __init__(self):
        self.gyro = np.zeros(3, dtype=np.float64)       # rad/s (x, y, z)
        self.accel = np.zeros(3, dtype=np.float64)       # m/s²
        self.euler = np.zeros(3, dtype=np.float64)       # rad (roll, pitch, yaw)
        self.quaternion = np.array([1.0, 0, 0, 0])       # (w, x, y, z)
        self.gravity = np.array([0.0, 0.0, -1.0])        # projected gravity (Isaac convention)
        self.timestamp = 0.0


class IM10ADriver:
    """Serial driver for IM10A IMU (WIT-motion B6 protocol).

    Auto-detects 9600 baud default and upgrades to 460800.
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = TARGET_BAUD):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._buf = bytearray()
        self._data = IM10AData()
        self._has_quat = False
        self._has_gyro = False

    def open(self):
        """Open serial port. Auto-upgrade baud if at default 9600."""
        import serial

        # Try target baud first
        self._serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
        self._serial.reset_input_buffer()
        time.sleep(0.3)
        data = self._serial.read(100)
        frames = self._count_frames(data)

        if frames >= 3:
            # Already at target baud
            self._serial.reset_input_buffer()
            return

        # Try default 9600
        self._serial.close()
        self._serial = serial.Serial(port=self.port, baudrate=DEFAULT_BAUD, timeout=0.1)
        self._serial.reset_input_buffer()
        time.sleep(0.3)
        data = self._serial.read(100)
        frames = self._count_frames(data)

        if frames >= 3:
            # At 9600 — upgrade to target
            self._upgrade_baud()
            self._serial.close()
            time.sleep(0.3)
            self._serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.01)
            self._serial.reset_input_buffer()
            return

        raise RuntimeError(
            f"IM10A not responding on {self.port} at {self.baudrate} or {DEFAULT_BAUD}"
        )

    def _count_frames(self, data: bytes) -> int:
        """Count valid frame headers in data."""
        count = 0
        for i in range(len(data) - 1):
            if data[i] == HEADER and (data[i + 1] & 0x50) == 0x50:
                count += 1
        return count

    def _upgrade_baud(self):
        """Send WIT-motion commands to change baud rate."""
        code = BAUD_CODES.get(self.baudrate)
        if code is None:
            raise ValueError(f"Unsupported baud rate: {self.baudrate}")

        # Unlock config
        self._serial.write(bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
        time.sleep(0.1)
        # Set baud rate
        self._serial.write(bytes([0xFF, 0xAA, 0x04, code, 0x00]))
        time.sleep(0.1)
        # Save to flash
        self._serial.write(bytes([0xFF, 0xAA, 0x00, 0x00, 0x00]))
        time.sleep(0.3)

    def close(self):
        if self._serial and self._serial.is_open:
            self._serial.close()

    def read(self) -> IM10AData | None:
        """Read and parse frames. Returns data when a complete set is available."""
        if not self._serial or not self._serial.is_open:
            return None

        avail = self._serial.in_waiting
        if avail > 0:
            self._buf.extend(self._serial.read(min(avail, 1024)))

        return self._parse()

    def _parse(self) -> IM10AData | None:
        """Parse 11-byte frames from buffer."""
        result = None

        while len(self._buf) >= FRAME_LEN:
            # Sync to header
            if self._buf[0] != HEADER:
                del self._buf[0]
                continue

            if len(self._buf) < FRAME_LEN:
                break

            frame_type = self._buf[1]
            if (frame_type & 0x50) != 0x50:
                del self._buf[0]
                continue

            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]

            # Parse 4 int16 values (little-endian) from bytes 2-9
            d0 = struct.unpack_from('<hhhh', frame, 2)

            if frame_type == TYPE_ACCEL:
                # Accel: raw / 32768 * 16g * 9.81
                self._data.accel[0] = d0[0] / 32768.0 * 16.0 * 9.81
                self._data.accel[1] = d0[1] / 32768.0 * 16.0 * 9.81
                self._data.accel[2] = d0[2] / 32768.0 * 16.0 * 9.81

            elif frame_type == TYPE_GYRO:
                # Gyro: raw / 32768 * 2000 deg/s → rad/s
                self._data.gyro[0] = d0[0] / 32768.0 * 2000.0 * math.pi / 180.0
                self._data.gyro[1] = d0[1] / 32768.0 * 2000.0 * math.pi / 180.0
                self._data.gyro[2] = d0[2] / 32768.0 * 2000.0 * math.pi / 180.0
                self._has_gyro = True

            elif frame_type == TYPE_EULER:
                # Euler: raw / 32768 * 180 deg → rad
                self._data.euler[0] = d0[0] / 32768.0 * math.pi  # roll
                self._data.euler[1] = d0[1] / 32768.0 * math.pi  # pitch
                self._data.euler[2] = d0[2] / 32768.0 * math.pi  # yaw

            elif frame_type == TYPE_QUAT:
                # Quaternion: raw / 32768
                qw = d0[0] / 32768.0
                qx = d0[1] / 32768.0
                qy = d0[2] / 32768.0
                qz = d0[3] / 32768.0
                self._data.quaternion[:] = [qw, qx, qy, qz]
                self._has_quat = True

                # Derive gravity from quaternion (Isaac convention: upright → [0, 0, -1])
                r = Rotation.from_quat([qx, qy, qz, qw])  # scipy: scalar-last
                self._data.gravity = r.apply(np.array([0.0, 0.0, -1.0]), inverse=True)

            # Return data when we have both quat and gyro
            if self._has_quat and self._has_gyro:
                self._data.timestamp = time.time()
                self._has_quat = False
                self._has_gyro = False
                result = self._data  # return latest complete set

        return result
