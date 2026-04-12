"""RobStride CAN bus driver — SocketCAN interface for MIT-mode motor control.

Upstream: https://github.com/Seeed-Projects/RobStride_Control/tree/master/python
Adapted for biped project: added MotorFeedback dataclass, fault decoding,
parameter R/W with type dispatch, and flush_rx.
"""

from dataclasses import dataclass
import struct
import time
from typing import Optional

import can
import numpy as np

from .table import (
    MODEL_MIT_POSITION_TABLE,
    MODEL_MIT_VELOCITY_TABLE,
    MODEL_MIT_TORQUE_TABLE,
    MODEL_MIT_KP_TABLE,
    MODEL_MIT_KD_TABLE,
)
from .protocol import CommunicationType, ParameterType


@dataclass
class Motor:
    id: int
    model: str  # "rs-02", "rs-03", "rs-04"


@dataclass
class MotorFeedback:
    """Decoded motor feedback from MIT status frame."""
    position: float = 0.0     # rad (output shaft)
    velocity: float = 0.0     # rad/s
    torque: float = 0.0       # Nm
    temperature: float = 0.0  # °C
    fault_code: int = 0       # 6-bit fault flags from ext ID
    mode_status: int = 0      # 0=Reset, 1=Calibration, 2=Run


class RobstrideBus:
    """Manages a CAN bus with one or more RobStride motors.

    All communication uses extended CAN IDs:
        bits 28-24: communication type
        bits 23-8:  extra data (host_id for TX, status flags for RX)
        bits 7-0:   device ID
    """

    def __init__(
        self,
        channel: str = "can0",
        motors: Optional[dict[str, Motor]] = None,
        calibration: Optional[dict[str, dict]] = None,
        bitrate: int = 1_000_000,
    ):
        self.channel = channel
        self.motors: dict[str, Motor] = motors or {}
        self.calibration = calibration or {}
        self.bitrate = bitrate
        self._bus = None  # can.Bus
        # Host ID > all motor IDs for optimal CAN arbitration
        self.host_id = 0xFD  # 253, matching ARCHITECTURE.md

    @property
    def is_connected(self) -> bool:
        return self._bus is not None

    # ── Connection ──────────────────────────────────────────────────

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError(f"Already connected to {self.channel}")
        self._bus = can.interface.Bus(
            interface="socketcan",
            channel=self.channel,
            bitrate=self.bitrate,
        )

    def disconnect(self) -> None:
        if self._bus:
            self._bus.shutdown()
            self._bus = None

    # ── Low-level CAN ───────────────────────────────────────────────

    def transmit(
        self,
        comm_type: int,
        extra_data: int,
        device_id: int,
        data: bytes = b"\x00" * 8,
    ) -> None:
        """Send a CAN extended frame."""
        assert 0 <= comm_type <= 0x1F
        assert 0 <= extra_data <= 0xFFFF
        assert 0 < device_id <= 0xFF
        ext_id = (comm_type << 24) | (extra_data << 8) | device_id
        frame = can.Message(
            arbitration_id=ext_id,
            is_extended_id=True,
            dlc=len(data),
            data=data,
        )
        for attempt in range(5):
            try:
                self._bus.send(frame)
                return
            except can.CanOperationError:
                # TX buffer full — MCP2515 has only 3 TX slots
                time.sleep(0.0005 * (attempt + 1))  # 0.5ms, 1ms, 1.5ms...
        # Last attempt without catch
        self._bus.send(frame)

    def receive(self, timeout: float = 0.05) -> Optional[tuple[int, int, int, bytes]]:
        """Returns (comm_type, extra_data, host_id, data) or None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            frame = self._bus.recv(timeout=remaining)
            if frame is None:
                return None
            if frame.is_extended_id:
                comm_type = (frame.arbitration_id >> 24) & 0x1F
                extra_data = (frame.arbitration_id >> 8) & 0xFFFF
                host_id = frame.arbitration_id & 0xFF
                return comm_type, extra_data, host_id, frame.data
        return None

    def flush_rx(self, timeout: float = 0.005) -> int:
        """Drain pending RX frames. Returns count flushed."""
        count = 0
        while True:
            frame = self._bus.recv(timeout=timeout)
            if frame is None:
                break
            count += 1
        return count

    def poll_all_feedback(self) -> dict[str, MotorFeedback]:
        """Non-blocking drain of all pending MIT status frames.
        
        Returns a dictionary of motor_name -> MotorFeedback for all
        motors that sent a status frame since the last poll.
        """
        feedbacks = {}
        while True:
            frame = self._bus.recv(timeout=0.001)
            if frame is None:
                break
            
            if not frame.is_extended_id:
                continue
                
            comm_type = (frame.arbitration_id >> 24) & 0x1F
            if comm_type != CommunicationType.OPERATION_CONTROL:
                continue
                
            motor_id = (frame.arbitration_id >> 8) & 0xFF
            data = frame.data
            
            # Find the motor name for this ID
            motor_name = None
            motor_model = None
            for name, m in self.motors.items():
                if m.id == motor_id:
                    motor_name = name
                    motor_model = m.model
                    break
                    
            if motor_name is None:
                continue

            try:
                if motor_model in ("rs-02", "rs-03", "rs-04"):
                    pos_u16, vel_u16, trq_u16, temp_u8, err_u8 = struct.unpack(
                        ">HHHBB", data
                    )
                    
                    # Apply reverse mapping
                    position = MODEL_MIT_POSITION_TABLE[motor_model].map_to_float(pos_u16)
                    velocity = MODEL_MIT_VELOCITY_TABLE[motor_model].map_to_float(vel_u16)
                    torque = MODEL_MIT_TORQUE_TABLE[motor_model].map_to_float(trq_u16)
                    temperature = float(temp_u8)
                    
                    # Error bytes: [mode 2 bits][fault 6 bits]
                    mode_status = (err_u8 >> 6) & 0x03
                    fault_code = err_u8 & 0x3F
                    
                    cal = self.calibration.get(motor_name, {"direction": 1, "homing_offset": 0.0})
                    position = (position - cal["homing_offset"]) * cal["direction"]
                    velocity = velocity * cal["direction"]
                    torque = torque * cal["direction"]
                    
                    feedbacks[motor_name] = MotorFeedback(
                        position=position,
                        velocity=velocity,
                        torque=torque,
                        temperature=temperature,
                        fault_code=fault_code,
                        mode_status=mode_status,
                    )
            except Exception:
                pass
                
        return feedbacks

    # ── Motor lifecycle ─────────────────────────────────────────────

    def enable(self, motor_name: str) -> Optional[MotorFeedback]:
        m = self.motors[motor_name]
        self.transmit(CommunicationType.ENABLE, self.host_id, m.id)
        return self._receive_feedback(motor_name)

    def disable(self, motor_name: str, clear_fault: bool = False) -> Optional[MotorFeedback]:
        m = self.motors[motor_name]
        data = bytes([1 if clear_fault else 0]) + bytes(7)
        self.transmit(CommunicationType.DISABLE, self.host_id, m.id, data)
        return self._receive_feedback(motor_name)

    def set_zero_position(self, motor_name: str) -> None:
        m = self.motors[motor_name]
        data = bytes([1]) + bytes(7)
        self.transmit(CommunicationType.SET_ZERO_POSITION, self.host_id, m.id, data)
        self._receive_feedback(motor_name)

    def set_mode(self, motor_name: str, mode: int) -> None:
        """Switch run mode: 0=MIT, 1=Position, 2=Velocity, 3=Current."""
        m = self.motors[motor_name]
        param_id = ParameterType.MODE[0]
        value_buf = struct.pack("<bBH", mode, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buf
        self.transmit(CommunicationType.WRITE_PARAMETER, self.host_id, m.id, data)
        self._receive_feedback(motor_name, timeout=0.05)  # consume response
        time.sleep(0.02)

    # ── MIT frame write/read ────────────────────────────────────────

    def write_operation_frame(
        self,
        motor_name: str,
        position: float,
        kp: float,
        kd: float,
        velocity: float = 0.0,
        torque: float = 0.0,
    ) -> None:
        """Send one MIT control frame.

        Motor executes: τ = Kp*(position - p) + Kd*(velocity - v) + torque
        """
        m = self.motors[motor_name]
        model = m.model

        # Apply calibration
        cal = self.calibration.get(motor_name, {"direction": 1, "homing_offset": 0.0})
        pos = position * cal["direction"] + cal["homing_offset"]
        vel = velocity * cal["direction"]
        trq = torque * cal["direction"]

        # Encode to uint16 per model scaling tables
        pos_max = MODEL_MIT_POSITION_TABLE[model]
        vel_max = MODEL_MIT_VELOCITY_TABLE[model]
        kp_max = MODEL_MIT_KP_TABLE[model]
        kd_max = MODEL_MIT_KD_TABLE[model]
        trq_max = MODEL_MIT_TORQUE_TABLE[model]

        pos = float(np.clip(pos, -pos_max, pos_max))
        vel = float(np.clip(vel, -vel_max, vel_max))
        kp = float(np.clip(kp, 0.0, kp_max))
        kd = float(np.clip(kd, 0.0, kd_max))
        trq = float(np.clip(trq, -trq_max, trq_max))

        pos_u16 = int(np.clip(((pos / pos_max) + 1.0) * 0x7FFF, 0, 0xFFFF))
        vel_u16 = int(np.clip(((vel / vel_max) + 1.0) * 0x7FFF, 0, 0xFFFF))
        kp_u16 = int((kp / kp_max) * 0xFFFF)
        kd_u16 = int((kd / kd_max) * 0xFFFF)
        trq_u16 = int(np.clip(((trq / trq_max) + 1.0) * 0x7FFF, 0, 0xFFFF))

        data = struct.pack(">HHHH", pos_u16, vel_u16, kp_u16, kd_u16)
        self.transmit(CommunicationType.OPERATION_CONTROL, trq_u16, m.id, data)
        time.sleep(0.0002)  # 0.2ms — let MCP2515 SPI transfer complete before next op

    def read_operation_frame(
        self, motor_name: str, timeout: float = 0.01
    ) -> Optional[MotorFeedback]:
        """Read one MIT status frame (call after write_operation_frame)."""
        return self._receive_feedback(motor_name, timeout)

    # ── Parameter access ────────────────────────────────────────────

    def read_parameter(
        self, motor_name: str, parameter_type: tuple[int, np.dtype, str]
    ) -> Optional[float]:
        """Read a parameter from the motor."""
        m = self.motors[motor_name]
        param_id, param_dtype, _ = parameter_type
        data = struct.pack("<HHL", param_id, 0x00, 0x00)
        self.transmit(CommunicationType.READ_PARAMETER, self.host_id, m.id, data)
        resp = self.receive(timeout=0.05)
        if resp is None:
            return None
        _, _, _, resp_data = resp
        value_bytes = resp_data[4:]
        if param_dtype == np.uint8:
            value, _, _ = struct.unpack("<BBH", value_bytes)
        elif param_dtype == np.int8:
            value, _, _ = struct.unpack("<bBH", value_bytes)
        elif param_dtype == np.uint16:
            value, _ = struct.unpack("<HH", value_bytes)
        elif param_dtype == np.int16:
            value, _ = struct.unpack("<hH", value_bytes)
        elif param_dtype == np.uint32:
            (value,) = struct.unpack("<L", value_bytes)
        elif param_dtype == np.int32:
            (value,) = struct.unpack("<l", value_bytes)
        elif param_dtype == np.float32:
            (value,) = struct.unpack("<f", value_bytes)
        else:
            return None
        return float(value)

    def write_parameter(
        self, motor_name: str, parameter_type: tuple[int, np.dtype, str], value
    ) -> None:
        """Write a parameter to the motor."""
        m = self.motors[motor_name]
        param_id, param_dtype, _ = parameter_type
        if param_dtype == np.uint8:
            vbuf = struct.pack("<BBH", int(value), 0, 0)
        elif param_dtype == np.int8:
            vbuf = struct.pack("<bBH", int(value), 0, 0)
        elif param_dtype == np.uint16:
            vbuf = struct.pack("<HH", int(value), 0)
        elif param_dtype == np.int16:
            vbuf = struct.pack("<hH", int(value), 0)
        elif param_dtype == np.uint32:
            vbuf = struct.pack("<L", int(value))
        elif param_dtype == np.int32:
            vbuf = struct.pack("<l", int(value))
        elif param_dtype == np.float32:
            vbuf = struct.pack("<f", float(value))
        else:
            return
        data = struct.pack("<HH", param_id, 0x00) + vbuf
        self.transmit(CommunicationType.WRITE_PARAMETER, self.host_id, m.id, data)
        self._receive_feedback(motor_name, timeout=0.05)

    # ── Feedback decoding ───────────────────────────────────────────

    def _receive_feedback(
        self, motor_name: str, timeout: float = 0.01
    ) -> Optional[MotorFeedback]:
        """Receive and decode a motor feedback frame.

        Filters by motor ID — discards frames from other motors
        and keeps reading until the expected motor responds or timeout.
        """
        m = self.motors[motor_name]
        expected_id = m.id
        model = m.model
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = max(0.001, deadline - time.monotonic())
            resp = self.receive(remaining)
            if resp is None:
                return None

            comm_type, extra_data, _, data = resp

            if comm_type == CommunicationType.FAULT_REPORT:
                fault_val, warn_val = struct.unpack("<LL", data)
                raise RuntimeError(
                    f"Motor {motor_name} fault: fault=0x{fault_val:08X} warn=0x{warn_val:08X}"
                )

            if comm_type != CommunicationType.OPERATION_STATUS:
                continue  # not an operation status frame, keep looking

            # Check motor ID
            device_id = extra_data & 0xFF
            if device_id != expected_id:
                continue  # wrong motor, discard and keep reading

            # Status flags
            fault_code = (extra_data >> 8) & 0x3F
            mode_status = (extra_data >> 14) & 0x03

            # Decode feedback (big-endian per spec)
            pos_u16, vel_u16, torque_u16, temp_u16 = struct.unpack(">HHHH", data)

            position = (float(pos_u16) / 0x7FFF - 1.0) * MODEL_MIT_POSITION_TABLE[model]
            velocity = (float(vel_u16) / 0x7FFF - 1.0) * MODEL_MIT_VELOCITY_TABLE[model]
            torque = (float(torque_u16) / 0x7FFF - 1.0) * MODEL_MIT_TORQUE_TABLE[model]
            temperature = float(temp_u16) * 0.1

            # Undo calibration
            cal = self.calibration.get(motor_name, {"direction": 1, "homing_offset": 0.0})
            position = (position - cal["homing_offset"]) * cal["direction"]
            velocity = velocity * cal["direction"]
            torque = torque * cal["direction"]

            return MotorFeedback(
                position=position,
                velocity=velocity,
                torque=torque,
                temperature=temperature,
                fault_code=fault_code,
                mode_status=mode_status,
            )

        return None  # timeout

    # ── Bulk operations ─────────────────────────────────────────────

    def enable_all(self) -> dict[str, Optional[MotorFeedback]]:
        states = {}
        for name in self.motors:
            self.flush_rx()
            states[name] = self.enable(name)
            time.sleep(0.02)
        return states

    def disable_all(self, clear_fault: bool = False) -> dict[str, Optional[MotorFeedback]]:
        states = {}
        for name in self.motors:
            self.flush_rx()
            states[name] = self.disable(name, clear_fault)
            time.sleep(0.02)
        return states

    def set_mode_all(self, mode: int) -> None:
        for name in self.motors:
            self.flush_rx()
            self.set_mode(name, mode)

    def enable_and_set_mit_all(self) -> None:
        """Enable all motors and set MIT mode with per-motor verification.

        Sequence per motor: disable → set MIT mode → verify mode → enable → verify MIT.
        Retries mode set up to 3 times if verification fails.
        """
        for name in self.motors:
            # 1. Disable (clean state, clear faults)
            self.flush_rx()
            self.disable(name, clear_fault=True)
            time.sleep(0.05)

            # 2. Set MIT mode with verification
            mode_ok = False
            for attempt in range(3):
                self.flush_rx()
                self.set_mode(name, 0)
                time.sleep(0.05)

                # Read back mode to verify
                self.flush_rx()
                mode_val = self.read_parameter(name, ParameterType.MODE)
                if mode_val is not None and int(mode_val) == 0:
                    mode_ok = True
                    break
                else:
                    print(f"  ⚠ {name}: mode set attempt {attempt+1} failed (read={mode_val}), retrying...")
                    time.sleep(0.05)

            if not mode_ok:
                print(f"  ✗ {name}: FAILED to set MIT mode after 3 attempts!")

            # 3. Enable
            self.flush_rx()
            self.enable(name)
            time.sleep(0.05)

            # 4. Verify with zero-torque MIT command
            self.flush_rx()
            self.write_operation_frame(name, 0.0, 0.0, 0.0, 0.0, 0.0)
            verify_fb = self.read_operation_frame(name, timeout=0.05)
            if verify_fb:
                print(f"  ✓ {name}: mode={'MIT' if mode_ok else '??'}, pos={verify_fb.position:.3f}")
            else:
                print(f"  ⚠ {name}: no verify response")

    @classmethod
    def scan_channel(cls, channel: str, start_id: int = 1, end_id: int = 127):
        """Probe a CAN channel and list responding motor IDs."""
        bus = cls(channel=channel)
        bus.connect()
        found = {}
        for dev_id in range(start_id, end_id + 1):
            bus.transmit(CommunicationType.GET_DEVICE_ID, bus.host_id, dev_id)
            resp = bus.receive(timeout=0.05)
            if resp is not None:
                found[dev_id] = resp
        bus.disconnect()
        return found
