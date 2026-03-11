"""RobStride RS02/RS03/RS04 CAN protocol — MIT mode (mode 0).

Low-level CAN frame encoding/decoding for RobStride actuators.
No ROS dependency — usable standalone.

Protocol: CAN 2.0 extended frame (29-bit ID), 1Mbps.
MIT control law: τ = Kp*(p_set - p) + Kd*(v_set - v) + τ_ff

Reference: RobStride RS04 User Manual, Chapter 5 (Private Protocol)
           and robstride_ros_sample motor_cfg.h

Usage:
    motor = RobStrideMotor("can0", motor_id=1, actuator_type="RS04")
    motor.enable()
    pos, vel, torque, temp = motor.send_mit_command(
        position=0.5, velocity=0.0, kp=15.0, kd=3.0, torque_ff=0.0)
    motor.disable()
"""

import socket
import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple

# CAN communication types (29-bit extended ID encoding)
COMM_GET_ID = 0x00
COMM_MOTION_CONTROL = 0x01      # MIT mode command
COMM_MOTOR_FEEDBACK = 0x02      # Motor state feedback
COMM_ENABLE = 0x03              # Enable motor
COMM_DISABLE = 0x04             # Disable motor (byte[0]=1 clears faults)
COMM_SET_ZERO = 0x06            # Set mechanical zero
COMM_SET_CAN_ID = 0x07          # Change CAN ID
COMM_READ_PARAM = 0x11          # Read single parameter
COMM_WRITE_PARAM = 0x12         # Write single parameter
COMM_FAULT_FEEDBACK = 0x15      # Fault feedback
COMM_SAVE_PARAMS = 0x16         # Save parameters to flash

# Parameter indices
PARAM_RUN_MODE = 0x7005
PARAM_IQ_REF = 0x7006
PARAM_SPD_REF = 0x700A
PARAM_LIMIT_TORQUE = 0x700B
PARAM_CUR_KP = 0x7010
PARAM_CUR_KI = 0x7011
PARAM_CUR_FILT = 0x7014
PARAM_LOC_REF = 0x7016
PARAM_LIMIT_SPD = 0x7017
PARAM_LIMIT_CUR = 0x7018
PARAM_MECH_POS = 0x7019        # Read-only: mechanical position
PARAM_IQF = 0x701A             # Read-only: filtered Iq
PARAM_MECH_VEL = 0x701B        # Read-only: mechanical velocity
PARAM_VBUS = 0x701C            # Read-only: bus voltage
PARAM_ROTATION = 0x701D        # Read-only: rotation count
PARAM_CAN_TIMEOUT = 0x702B     # CAN timeout (0=disabled)

# Run modes
MODE_MIT = 0            # Motion control (MIT/impedance)
MODE_POSITION_PP = 1    # Profile Position
MODE_VELOCITY = 2       # Velocity
MODE_CURRENT = 3        # Current/torque
MODE_SET_ZERO = 4       # Zero calibration
MODE_POSITION_CSP = 5   # Cyclic Synchronous Position


@dataclass
class ActuatorConfig:
    """Per-actuator-type parameter ranges for CAN encoding."""
    name: str
    max_torque: float       # Nm
    max_velocity: float     # rad/s (for MIT feedback encoding)
    max_current: float      # A
    gear_ratio: float
    # MIT command encoding ranges (private protocol, 16-bit)
    p_min: float = -12.57
    p_max: float = 12.57
    v_min: float = -44.0
    v_max: float = 44.0
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0
    t_min: float = -17.0
    t_max: float = 17.0


# Actuator type configs — ranges from datasheets
ACTUATOR_CONFIGS = {
    "RS02": ActuatorConfig(
        name="RS02", max_torque=17.0, max_velocity=44.0, max_current=23.0,
        gear_ratio=7.75,
        v_min=-44.0, v_max=44.0,
        kp_max=500.0, kd_max=5.0,
        t_min=-17.0, t_max=17.0,
    ),
    "RS03": ActuatorConfig(
        name="RS03", max_torque=60.0, max_velocity=50.0, max_current=23.0,
        gear_ratio=9.0,
        v_min=-50.0, v_max=50.0,
        kp_max=5000.0, kd_max=100.0,
        t_min=-60.0, t_max=60.0,
    ),
    "RS04": ActuatorConfig(
        name="RS04", max_torque=120.0, max_velocity=15.0, max_current=90.0,
        gear_ratio=9.0,
        v_min=-15.0, v_max=15.0,
        kp_max=5000.0, kd_max=100.0,
        t_min=-120.0, t_max=120.0,
    ),
}


@dataclass
class MotorFeedback:
    """Decoded motor feedback from CAN response."""
    position: float     # rad (output shaft)
    velocity: float     # rad/s
    torque: float       # Nm
    temperature: float  # °C
    fault_code: int     # 6-bit fault flags
    mode_status: int    # 0=Reset, 1=Calibration, 2=Run


class RobStrideMotor:
    """Single RobStride motor interface over SocketCAN.

    Args:
        can_interface: SocketCAN interface name (e.g. "can0")
        motor_id: Motor CAN ID (1-127)
        actuator_type: "RS02", "RS03", or "RS04"
        master_id: Host CAN ID (default 253)
    """

    def __init__(self, can_interface: str, motor_id: int,
                 actuator_type: str = "RS04", master_id: int = 253):
        self.interface = can_interface
        self.motor_id = motor_id
        self.master_id = master_id
        self.config = ACTUATOR_CONFIGS[actuator_type]
        self._sock = None
        self._connect()

    def _connect(self):
        """Open SocketCAN socket."""
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((self.interface,))
        self._sock.settimeout(0.01)  # 10ms timeout for reads

    def close(self):
        """Close CAN socket."""
        if self._sock:
            self._sock.close()
            self._sock = None

    # --- CAN frame helpers ---

    @staticmethod
    def _float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        """Map float to unsigned int for CAN encoding."""
        span = x_max - x_min
        x = max(x_min, min(x_max, x))
        return int(((x - x_min) / span) * ((1 << bits) - 1))

    @staticmethod
    def _uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
        """Map unsigned int from CAN to float."""
        span = x_max - x_min
        return float(x_int) * span / ((1 << bits) - 1) + x_min

    def _build_ext_id(self, comm_type: int, data16: int = 0) -> int:
        """Build 29-bit extended CAN ID.

        Bits 28-24: communication type (5 bits)
        Bits 23-8:  data area 2 (16 bits)
        Bits 7-0:   destination motor ID (8 bits)
        """
        return ((comm_type & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (self.motor_id & 0xFF)

    def _parse_ext_id(self, can_id: int) -> Tuple[int, int, int]:
        """Parse extended CAN ID → (comm_type, data16, motor_id)."""
        comm_type = (can_id >> 24) & 0x1F
        data16 = (can_id >> 8) & 0xFFFF
        source_id = can_id & 0xFF
        return comm_type, data16, source_id

    def _send_frame(self, can_id: int, data: bytes):
        """Send a CAN extended frame."""
        # struct can_frame: uint32 can_id, uint8 len, uint8[3] pad, uint8[8] data
        can_id_eff = can_id | 0x80000000  # Set EFF (extended frame) flag
        padded_data = data.ljust(8, b'\x00')
        frame = struct.pack("=IB3x8s", can_id_eff, min(len(data), 8), padded_data[:8])
        self._sock.send(frame)

    def _recv_frame(self, timeout: float = 0.01) -> Optional[Tuple[int, bytes]]:
        """Receive a CAN frame. Returns (can_id, data) or None on timeout."""
        self._sock.settimeout(timeout)
        try:
            frame = self._sock.recv(16)
            can_id, dlc = struct.unpack("=IB3x", frame[:8])
            can_id &= 0x1FFFFFFF  # Strip EFF flag
            data = frame[8:8 + dlc]
            return can_id, data
        except socket.timeout:
            return None

    # --- Motor commands ---

    def enable(self) -> Optional[MotorFeedback]:
        """Enable motor (comm type 3). Returns feedback or None."""
        can_id = self._build_ext_id(COMM_ENABLE, self.master_id)
        self._send_frame(can_id, bytes(8))
        return self._read_feedback()

    def disable(self, clear_fault: bool = False):
        """Disable motor (comm type 4). Set clear_fault=True to clear errors."""
        can_id = self._build_ext_id(COMM_DISABLE, self.master_id)
        data = bytes([1 if clear_fault else 0]) + bytes(7)
        self._send_frame(can_id, data)
        self._read_feedback()  # consume response

    def set_zero(self):
        """Set current position as mechanical zero (comm type 6)."""
        can_id = self._build_ext_id(COMM_SET_ZERO, self.master_id)
        data = bytes([1]) + bytes(7)
        self._send_frame(can_id, data)
        self._read_feedback()

    def send_mit_command(self, position: float = 0.0, velocity: float = 0.0,
                         kp: float = 0.0, kd: float = 0.0,
                         torque_ff: float = 0.0) -> Optional[MotorFeedback]:
        """Send MIT impedance control command (comm type 1).

        τ = Kp*(position - p_actual) + Kd*(velocity - v_actual) + torque_ff

        Returns motor feedback or None on timeout.
        """
        cfg = self.config

        # Encode torque into ID data16 field (16-bit)
        torque_uint = self._float_to_uint(torque_ff, cfg.t_min, cfg.t_max, 16)
        can_id = self._build_ext_id(COMM_MOTION_CONTROL, torque_uint)

        # Encode command data (8 bytes, big-endian per spec)
        pos_uint = self._float_to_uint(position, cfg.p_min, cfg.p_max, 16)
        vel_uint = self._float_to_uint(velocity, cfg.v_min, cfg.v_max, 16)
        kp_uint = self._float_to_uint(kp, cfg.kp_min, cfg.kp_max, 16)
        kd_uint = self._float_to_uint(kd, cfg.kd_min, cfg.kd_max, 16)

        data = struct.pack(">HHHH", pos_uint, vel_uint, kp_uint, kd_uint)
        self._send_frame(can_id, data)

        return self._read_feedback()

    def _read_feedback(self, timeout: float = 0.01) -> Optional[MotorFeedback]:
        """Read and decode motor feedback (comm type 2 response)."""
        result = self._recv_frame(timeout)
        if result is None:
            return None

        can_id, data = result
        comm_type, data16, source_id = self._parse_ext_id(can_id)

        if comm_type != COMM_MOTOR_FEEDBACK or len(data) < 8:
            return None

        # Decode ID field: motor_id (bits 8-15), fault (bits 16-21), mode (bits 22-23)
        fault_code = (data16 >> 8) & 0x3F
        mode_status = (data16 >> 14) & 0x03

        cfg = self.config

        # Decode data bytes (big-endian per spec)
        pos_uint = (data[0] << 8) | data[1]
        vel_uint = (data[2] << 8) | data[3]
        torque_uint = (data[4] << 8) | data[5]
        temp_raw = (data[6] << 8) | data[7]

        position = self._uint_to_float(pos_uint, cfg.p_min, cfg.p_max, 16)
        velocity = self._uint_to_float(vel_uint, cfg.v_min, cfg.v_max, 16)
        torque = self._uint_to_float(torque_uint, cfg.t_min, cfg.t_max, 16)
        temperature = temp_raw / 10.0  # Temp in °C × 10

        return MotorFeedback(
            position=position,
            velocity=velocity,
            torque=torque,
            temperature=temperature,
            fault_code=fault_code,
            mode_status=mode_status,
        )

    # --- Parameter access ---

    def read_parameter(self, index: int) -> Optional[float]:
        """Read a single parameter (comm type 0x11)."""
        can_id = self._build_ext_id(COMM_READ_PARAM, self.master_id)
        data = struct.pack("<H", index) + bytes(6)
        self._send_frame(can_id, data)

        result = self._recv_frame(timeout=0.05)
        if result is None:
            return None

        _, resp_data = result
        if len(resp_data) >= 8:
            # Response: bytes[0-1] = index, bytes[4-7] = float value (little-endian)
            value = struct.unpack("<f", resp_data[4:8])[0]
            return value
        return None

    def write_parameter(self, index: int, value: float):
        """Write a single parameter (comm type 0x12). Volatile — use save_params() to persist."""
        can_id = self._build_ext_id(COMM_WRITE_PARAM, self.master_id)
        data = struct.pack("<H2xf", index, value)
        self._send_frame(can_id, data)
        time.sleep(0.001)

    def save_params(self):
        """Save volatile parameters to flash (comm type 0x16)."""
        can_id = self._build_ext_id(COMM_SAVE_PARAMS, self.master_id)
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        self._send_frame(can_id, data)
        time.sleep(0.01)

    def set_can_timeout(self, timeout_ms: int):
        """Set CAN communication timeout. Motor enters reset if no command received.
        0 = disabled, 20000 = 1 second. Each unit = 50μs."""
        # Convert ms to register units (each unit ≈ 50μs)
        units = int(timeout_ms * 20) if timeout_ms > 0 else 0
        self.write_parameter(PARAM_CAN_TIMEOUT, float(units))

    def set_mode(self, mode: int):
        """Set run mode (0=MIT, 1=PP, 2=velocity, 3=current, 5=CSP)."""
        self.write_parameter(PARAM_RUN_MODE, float(mode))

    def read_position(self) -> Optional[float]:
        """Read current mechanical position (rad)."""
        return self.read_parameter(PARAM_MECH_POS)

    def read_velocity(self) -> Optional[float]:
        """Read current mechanical velocity (rad/s)."""
        return self.read_parameter(PARAM_MECH_VEL)

    def read_bus_voltage(self) -> Optional[float]:
        """Read bus voltage (V)."""
        return self.read_parameter(PARAM_VBUS)


class RobStrideMultiMotor:
    """Manage multiple RobStride motors on one or two CAN buses.

    Args:
        motors: dict of {joint_name: (can_interface, motor_id, actuator_type)}
    """

    def __init__(self, motors: dict):
        self.motors = {}
        for name, (iface, mid, atype) in motors.items():
            self.motors[name] = RobStrideMotor(iface, mid, atype)

    def enable_all(self):
        """Enable all motors."""
        for name, motor in self.motors.items():
            fb = motor.enable()
            if fb:
                print(f"  {name}: enabled, pos={fb.position:.3f}, mode={fb.mode_status}")
            else:
                print(f"  {name}: enable failed (no response)")

    def disable_all(self, clear_fault: bool = False):
        """Disable all motors."""
        for motor in self.motors.values():
            motor.disable(clear_fault)

    def send_mit_commands(self, commands: dict) -> dict:
        """Send MIT commands to multiple motors.

        Args:
            commands: {joint_name: (position, velocity, kp, kd, torque_ff)}

        Returns:
            {joint_name: MotorFeedback or None}
        """
        feedback = {}
        for name, (pos, vel, kp, kd, tff) in commands.items():
            if name in self.motors:
                feedback[name] = self.motors[name].send_mit_command(pos, vel, kp, kd, tff)
        return feedback

    def read_all_positions(self) -> dict:
        """Read positions from all motors via parameter read."""
        positions = {}
        for name, motor in self.motors.items():
            positions[name] = motor.read_position()
        return positions

    def close_all(self):
        """Close all CAN sockets."""
        for motor in self.motors.values():
            motor.close()


# --- Standalone test ---
if __name__ == "__main__":
    import sys

    iface = sys.argv[1] if len(sys.argv) > 1 else "can0"
    motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    atype = sys.argv[3] if len(sys.argv) > 3 else "RS04"

    print(f"Testing {atype} on {iface}, ID={motor_id}")
    motor = RobStrideMotor(iface, motor_id, atype)

    print("Enabling...")
    fb = motor.enable()
    print(f"  Feedback: {fb}")

    if fb:
        print(f"Reading position...")
        pos = motor.read_position()
        print(f"  Position: {pos}")

        print("Sending zero MIT command (hold position)...")
        fb = motor.send_mit_command(
            position=fb.position, velocity=0.0,
            kp=5.0, kd=0.5, torque_ff=0.0,
        )
        print(f"  Feedback: {fb}")

    print("Disabling...")
    motor.disable()
    motor.close()
    print("Done.")
