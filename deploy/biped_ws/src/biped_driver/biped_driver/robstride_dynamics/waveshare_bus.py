"""Waveshare USB-CAN-A serial transport — drop-in for python-can Bus.

The Waveshare USB-CAN-A uses a proprietary serial protocol (NOT slcand/SocketCAN).
This module speaks that protocol directly over pyserial, exposing the same
send()/recv() interface as python-can so RobstrideBus can use it transparently.

Protocol (variable-length mode):
  Settings: AA 55 12 speed frame_type filter[4] mask[4] mode 01 00 00 00 00 checksum
  TX/RX:    AA [info] ID[2 or 4] DATA[0-8] 55

  info byte:
    bit 7: always 1
    bit 6: always 1
    bit 5: 0=standard (11-bit ID, 2 bytes), 1=extended (29-bit ID, 4 bytes)
    bit 4: 0=data frame, 1=remote frame
    bits 3-0: DLC (0-8)

  For extended frame: ID is 4 bytes little-endian (29-bit)
  For standard frame: ID is 2 bytes little-endian (11-bit)

Hardware: Waveshare USB-CAN-A, /dev/ttyUSBx, serial 2Mbps, CAN up to 1Mbps.
"""

import struct
import time
import threading
from typing import Optional
from dataclasses import dataclass

import serial


# CAN speed codes for the settings command
SPEED_MAP = {
    1000000: 0x01,
    800000:  0x02,
    500000:  0x03,
    400000:  0x04,
    250000:  0x05,
    200000:  0x06,
    125000:  0x07,
    100000:  0x08,
    50000:   0x09,
    20000:   0x0A,
    10000:   0x0B,
    5000:    0x0C,
}

MODE_NORMAL    = 0x00
MODE_LOOPBACK  = 0x01
MODE_SILENT    = 0x02

FRAME_STANDARD = 0x01
FRAME_EXTENDED = 0x02


@dataclass
class Message:
    """CAN message — compatible with python-can's can.Message interface."""
    arbitration_id: int = 0
    is_extended_id: bool = True
    dlc: int = 8
    data: bytes = b'\x00' * 8


class WaveshareCANBus:
    """Serial-based CAN bus for Waveshare USB-CAN-A.

    Drop-in replacement for python-can's can.interface.Bus with the same
    send(msg) and recv(timeout) API used by RobstrideBus.

    Usage:
        bus = WaveshareCANBus('/dev/ttyUSB0', bitrate=1000000)
        bus.send(Message(arbitration_id=0x0000FD01, is_extended_id=True, dlc=8, data=bytes(8)))
        msg = bus.recv(timeout=0.01)
    """

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        bitrate: int = 1000000,
        serial_baudrate: int = 2000000,
        mode: int = MODE_NORMAL,
    ):
        self.port = port
        self.bitrate = bitrate
        self._serial_baudrate = serial_baudrate
        self._mode = mode
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

        self._open()
        self._configure()

    def _open(self):
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self._serial_baudrate,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_TWO,
            parity=serial.PARITY_NONE,
            timeout=0.001,  # 1ms for non-blocking reads
        )
        # Flush any stale data
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def _configure(self):
        """Send the 20-byte settings command to configure CAN speed, mode, and frame type."""
        speed_code = SPEED_MAP.get(self.bitrate)
        if speed_code is None:
            raise ValueError(f"Unsupported CAN bitrate: {self.bitrate}")

        cmd = bytearray(20)
        cmd[0] = 0xAA
        cmd[1] = 0x55
        cmd[2] = 0x12
        cmd[3] = speed_code
        cmd[4] = FRAME_EXTENDED  # Always use extended frame mode
        # cmd[5:9] = filter ID (0 = accept all)
        # cmd[9:13] = mask ID (0 = accept all)
        cmd[13] = self._mode
        cmd[14] = 0x01  # Enable
        # cmd[15:19] = 0
        cmd[19] = sum(cmd[2:19]) & 0xFF  # Checksum

        with self._lock:
            self._ser.write(cmd)
            time.sleep(0.1)  # Wait for adapter to apply settings
            self._ser.reset_input_buffer()  # Flush config response

    def send(self, msg: Message):
        """Send a CAN frame."""
        frame = bytearray()
        frame.append(0xAA)

        # Info byte
        info = 0xC0  # bits 7,6 always 1
        if msg.is_extended_id:
            info |= 0x20  # bit 5 = extended
        dlc = min(msg.dlc, 8)
        info |= (dlc & 0x0F)
        frame.append(info)

        # ID (little-endian)
        if msg.is_extended_id:
            frame.append(msg.arbitration_id & 0xFF)
            frame.append((msg.arbitration_id >> 8) & 0xFF)
            frame.append((msg.arbitration_id >> 16) & 0xFF)
            frame.append((msg.arbitration_id >> 24) & 0xFF)
        else:
            frame.append(msg.arbitration_id & 0xFF)
            frame.append((msg.arbitration_id >> 8) & 0xFF)

        # Data
        data = msg.data[:dlc] if msg.data else bytes(dlc)
        frame.extend(data)

        # Tail
        frame.append(0x55)

        with self._lock:
            self._ser.write(frame)

    def recv(self, timeout: float = None) -> Optional[Message]:
        """Receive a CAN frame. Returns Message or None on timeout."""
        deadline = time.monotonic() + (timeout if timeout else 0.0)
        buf = bytearray()

        while True:
            remaining = deadline - time.monotonic() if timeout else 0
            if timeout and remaining <= 0:
                return None

            with self._lock:
                chunk = self._ser.read(max(1, self._ser.in_waiting))

            if chunk:
                buf.extend(chunk)
                msg = self._try_parse(buf)
                if msg is not None:
                    return msg
            else:
                if timeout is None:
                    return None
                time.sleep(0.0001)  # 0.1ms between polls

    def _try_parse(self, buf: bytearray) -> Optional[Message]:
        """Try to parse a complete frame from the buffer. Consumes parsed bytes."""
        while len(buf) > 0:
            # Sync to 0xAA header
            if buf[0] != 0xAA:
                buf.pop(0)
                continue

            if len(buf) < 2:
                return None

            # Check if this is a settings response (AA 55 ...)
            if buf[1] == 0x55:
                if len(buf) >= 20:
                    del buf[:20]  # Consume settings frame
                    continue
                return None  # Incomplete settings frame

            # Data frame: info byte has bits 7,6 set
            if (buf[1] >> 4) != 0xC and (buf[1] >> 4) != 0xE:
                buf.pop(0)  # Not a valid data frame
                continue

            info = buf[1]
            is_extended = bool(info & 0x20)
            dlc = info & 0x0F

            id_len = 4 if is_extended else 2
            frame_len = 2 + id_len + dlc + 1  # header + info + ID + data + tail

            if len(buf) < frame_len:
                return None  # Incomplete

            # Verify tail
            if buf[frame_len - 1] != 0x55:
                buf.pop(0)  # Bad frame, resync
                continue

            # Parse ID (little-endian)
            if is_extended:
                arb_id = buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24)
                data_start = 6
            else:
                arb_id = buf[2] | (buf[3] << 8)
                data_start = 4

            data = bytes(buf[data_start:data_start + dlc])

            # Consume frame
            del buf[:frame_len]

            return Message(
                arbitration_id=arb_id,
                is_extended_id=is_extended,
                dlc=dlc,
                data=data,
            )

        return None

    def shutdown(self):
        """Close the serial port."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None

    def __del__(self):
        self.shutdown()
