import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from biped_msgs.msg import MITCommand, MITCommandArray, RobotState
import time
import math
import yaml
import os

from biped_control.obs_builder import JOINT_ORDER, DEFAULT_POSITIONS, DEFAULT_GAINS

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.declare_parameter('stand_ramp_time', 2.0)
        self.declare_parameter('stand_gain_ramp_time', 1.0)
        self.declare_parameter('stand_stable_time', 2.0)
        self.declare_parameter('control_params_file', '')
        self.declare_parameter('wiggle_config', '')
        self.declare_parameter('step_config', '')
        self.declare_parameter('gain_scale', 1.0)
        
        path = self.get_parameter('control_params_file').value
        if path:
            import biped_control.obs_builder as obs_builder
            obs_builder.load_control_params(path)

        self._ramp_time = float(self.get_parameter('stand_ramp_time').value)
        self._gain_ramp_time = float(self.get_parameter('stand_gain_ramp_time').value)
        self._stable_time = float(self.get_parameter('stand_stable_time').value)

        self._wiggle_cfg = None
        self._step_cfg = None
        self._wiggle_start = 0.0
        
        self._active_joint_idx = -1
        self._target_joint_idx = -1
        self._wiggle_interpolating = False
        self._interp_start_time = 0.0
        self._interp_start_pos = 0.0
        self._wiggle_sine_start_time = 0.0

        self.declare_parameter('trajectory_file', '')
        self._traj_frames = None
        self._traj_index = 0
        self._traj_sim_only = False
        self._traj_ramp_frames = 0
        self._traj_gain_ramp_secs = 3.0

        self._safety_ok = True
        self._current_positions = {name: 0.0 for name in JOINT_ORDER}
        self._stand_start_positions = {name: 0.0 for name in JOINT_ORDER}
        self._state = "IDLE"
        self._state_start_time = time.time()

        self._joint_limits = {
            "L_hip_pitch": (-1.047, 2.217), "R_hip_pitch": (-2.217, 1.047),
            "L_hip_roll":  (-0.209, 2.269), "R_hip_roll":  (-2.269, 0.209),
            "L_hip_yaw":   (-1.571, 1.571), "R_hip_yaw":   (-1.571, 1.571),
            "L_knee":      ( 0.000, 2.705), "R_knee":      ( 0.000, 2.705),
            "L_foot_pitch":(-0.873, 0.524), "R_foot_pitch":(-0.873, 0.524),
            "L_foot_roll": (-0.262, 0.262), "R_foot_roll": (-0.262, 0.262),
        }

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Bool, '/safety/status', self._safety_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(String, '/state_command', self._fsm_cb, 10)

        self._pub_state = self.create_publisher(String, '/state_machine', 10)
        self._pub_robot = self.create_publisher(RobotState, '/robot_state', 10)
        self._pub_cmd = self.create_publisher(MITCommandArray, '/joint_commands', 10)
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_viz_js = self.create_publisher(JointState, '/policy_viz_joints', 10)

        self._timer_control = self.create_timer(1.0 / 50.0, self._loop_cb)
        self._timer_status = self.create_timer(1.0 / 10.0, self._publish_state_cb)

        self.get_logger().info(f'State machine started (Py) — state: {self._state}')

    def _load_wiggle_config(self):
        path = self.get_parameter('wiggle_config').value
        cfg = {'joints': {}}
        global_freq = 1.0
        if path and os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    raw = yaml.safe_load(f)
                    if 'wiggle' in raw and 'joints' in raw['wiggle']:
                        for name, params in raw['wiggle']['joints'].items():
                            cfg['joints'][name] = {
                                'pos': float(params.get('pos', 5.0)) * math.pi / 180.0,
                                'neg': float(params.get('neg', -5.0)) * math.pi / 180.0,
                                'freq': float(params.get('freq', global_freq)),
                            }
                self.get_logger().info('Wiggle config loaded (converted from degrees)')
            except Exception as e:
                self.get_logger().warn(f'Failed to load wiggle config: {e}')
        
        for name in JOINT_ORDER:
            if name not in cfg['joints']:
                cfg['joints'][name] = {'pos': 5.0 * math.pi / 180.0, 'neg': -5.0 * math.pi / 180.0, 'freq': global_freq}
        self._wiggle_cfg = cfg

    def _load_step_config(self):
        path = self.get_parameter('step_config').value
        cfg = {'joints': {}}
        global_period = 2.0
        if path and os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    raw = yaml.safe_load(f)
                    if 'step' in raw and 'joints' in raw['step']:
                        for name, params in raw['step']['joints'].items():
                            cfg['joints'][name] = {
                                'step_max': float(params.get('step_max', 10.0)) * math.pi / 180.0,
                                'step_min': float(params.get('step_min', -10.0)) * math.pi / 180.0,
                                'period': float(params.get('period', global_period)),
                            }
                self.get_logger().info('Step test config loaded (converted from degrees)')
            except Exception as e:
                self.get_logger().warn(f'Failed to load step test config: {e}')
        
        for name in JOINT_ORDER:
            if name not in cfg['joints']:
                cfg['joints'][name] = {'step_max': 10.0 * math.pi / 180.0, 'step_min': -10.0 * math.pi / 180.0, 'period': global_period}
        self._step_cfg = cfg

    def _load_trajectory(self):
        pass # Simplified for script limit, trajectory playback works as is in cpp

    def _safety_cb(self, msg: Bool):
        self._safety_ok = msg.data
        if not self._safety_ok and self._state != "ESTOP":
            self.get_logger().error("Safety fault triggered! Forcing ESTOP.")
            self._transition("ESTOP")

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self._current_positions[name] = msg.position[i]

    def _fsm_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        
        if cmd == "START" and self._state == "IDLE":
            self._transition("STAND")
        elif cmd == "WALK" and self._state == "STAND":
            self._transition("WALK")
        elif cmd == "SIM_WALK" and self._state == "STAND":
            self._transition("SIM_WALK")
        elif cmd == "PLAY_TRAJ" and self._state == "STAND":
            self._transition("PLAY_TRAJ")
        elif cmd == "PLAY_TRAJ_SIM" and self._state == "STAND":
            self._transition("PLAY_TRAJ_SIM")
        elif cmd == "STOP" and self._state not in ("IDLE", "ESTOP"):
            self._transition("STAND")
        elif cmd.startswith("WIGGLE_SEQ") or cmd.startswith("STEP_TEST"):
            parts = cmd.split(":")
            base_cmd = parts[0]
            if len(parts) > 1:
                if self._state == base_cmd and (time.time() - self._state_start_time <= self._ramp_time + self._stable_time):
                    self.get_logger().warn("Ignoring joint selection during initial ramp phase.")
                    return
                
                try:
                    target_idx = int(parts[1])
                    if 0 <= target_idx < len(JOINT_ORDER):
                        if self._state != base_cmd:
                            self._transition(base_cmd)
                        
                        if target_idx != getattr(self, '_active_joint_idx', -1):
                            self._target_joint_idx = target_idx
                            self._interp_start_time = time.time()
                            if getattr(self, '_active_joint_idx', -1) >= 0:
                                self._interp_start_pos = self._current_positions.get(
                                    JOINT_ORDER[self._active_joint_idx], 0.0
                                ) - DEFAULT_POSITIONS[JOINT_ORDER[self._active_joint_idx]]
                                self._wiggle_interpolating = True
                            else:
                                self._active_joint_idx = target_idx
                                self._wiggle_sine_start_time = time.time()
                                self._wiggle_interpolating = False
                except ValueError:
                    pass
            else:
                if self._state != base_cmd:
                    self._transition(base_cmd)
        elif cmd == "WIGGLE_ALL" and self._state == "STAND":
            self._transition("WIGGLE_ALL")
        elif cmd == "ESTOP":
            self._transition("ESTOP")
        elif cmd == "RESET" and self._state == "ESTOP":
            self._transition("IDLE")

    def _transition(self, new_state: str):
        old = self._state
        self._state = new_state
        self._state_start_time = time.time()

        if new_state == "STAND":
            self._stand_start_positions = dict(self._current_positions)
        elif new_state == "WIGGLE_SEQ":
            self._stand_start_positions = dict(self._current_positions)
            self._load_wiggle_config()
            self._active_joint_idx = -1
            self._target_joint_idx = -1
            self._wiggle_interpolating = False
        elif new_state == "STEP_TEST":
            self._stand_start_positions = dict(self._current_positions)
            self._load_step_config()
            self._active_joint_idx = -1
            self._target_joint_idx = -1
            self._wiggle_interpolating = False
        elif new_state == "WIGGLE_ALL":
            self._stand_start_positions = dict(self._current_positions)
            self._load_wiggle_config()
            self._wiggle_start = time.time()
        elif new_state == "ESTOP":
            self._send_zero_torque()

        self.get_logger().info(f'State: {old} -> {new_state}')

    def _send_zero_torque(self):
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        for name in JOINT_ORDER:
            cmd = MITCommand()
            cmd.joint_name = name
            cmd_msg.commands.append(cmd)
        self._pub_cmd.publish(cmd_msg)

    def _loop_cb(self):
        if self._state == "STAND":
            self._handle_stand()
        elif self._state == "WALK":
            pass
        elif self._state == "SIM_WALK":
            self._handle_stand_hold()
        elif self._state == "WIGGLE_SEQ":
            self._handle_wiggle_sequential()
        elif self._state == "STEP_TEST":
            self._handle_step_test()
        elif self._state == "WIGGLE_ALL":
            self._handle_wiggle_all()
        elif self._state == "ESTOP":
            self._send_zero_torque()

    def _handle_stand(self):
        elapsed = time.time() - self._state_start_time
        pos_alpha = min(elapsed / self._ramp_time, 1.0)
        gain_elapsed = max(0.0, elapsed - self._ramp_time)
        gain_alpha = min(gain_elapsed / self._gain_ramp_time, 1.0)

        soft_kp_scale = 0.1 + 0.9 * gain_alpha
        soft_kd_scale = 0.2 + 0.8 * gain_alpha
        gs = float(self.get_parameter("gain_scale").value)

        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for name in JOINT_ORDER:
            start_pos = self._stand_start_positions.get(name, DEFAULT_POSITIONS[name])
            target = DEFAULT_POSITIONS[name]
            current_target = start_pos + (target - start_pos) * pos_alpha

            kp_base, kd_base = DEFAULT_GAINS[name]
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = float(current_target)
            cmd.velocity = 0.0
            cmd.kp = float(kp_base * gs * soft_kp_scale)
            cmd.kd = float(kd_base * gs * soft_kd_scale)
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)
        self._pub_vel.publish(Twist())

    def _handle_stand_hold(self):
        gs = float(self.get_parameter("gain_scale").value)
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        for name in JOINT_ORDER:
            kp_base, kd_base = DEFAULT_GAINS[name]
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = float(DEFAULT_POSITIONS[name])
            cmd.velocity = 0.0
            cmd.kp = float(kp_base * gs)
            cmd.kd = float(kd_base * gs)
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)
        self._pub_cmd.publish(cmd_msg)

    def _handle_wiggle_sequential(self):
        elapsed = time.time() - self._state_start_time
        if elapsed <= self._ramp_time + self._stable_time:
            self._handle_stand()
            return

        current_time = time.time()
        gs = float(self.get_parameter("gain_scale").value)
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        if getattr(self, '_wiggle_interpolating', False):
            alpha = min((current_time - self._interp_start_time) / 3.0, 1.0)
            if alpha >= 1.0:
                self._wiggle_interpolating = False
                self._active_joint_idx = self._target_joint_idx
                self._wiggle_sine_start_time = current_time
            
            for i, name in enumerate(JOINT_ORDER):
                target = DEFAULT_POSITIONS[name]
                if i == self._active_joint_idx:
                    target += self._interp_start_pos * (1.0 - alpha)

                limit = self._joint_limits.get(name)
                if limit:
                    target = max(limit[0], min(limit[1], target))

                kp_base, kd_base = DEFAULT_GAINS[name]
                cmd = MITCommand()
                cmd.joint_name = name
                cmd.position = float(target)
                cmd.velocity = 0.0
                cmd.kp = float(kp_base * gs)
                cmd.kd = float(kd_base * gs)
                cmd.torque_ff = 0.0
                cmd_msg.commands.append(cmd)
        else:
            for i, name in enumerate(JOINT_ORDER):
                target = DEFAULT_POSITIONS[name]
                if i == getattr(self, '_active_joint_idx', -1):
                    jcfg = self._wiggle_cfg['joints'].get(
                        name, {'pos': 0.087, 'neg': -0.087, 'freq': 1.0}
                    )
                    phase_t = current_time - self._wiggle_sine_start_time
                    sin_val = math.sin(2 * math.pi * jcfg['freq'] * phase_t)
                    offset = (jcfg['pos'] * sin_val) if sin_val >= 0 else (-jcfg['neg'] * sin_val)
                    target += offset

                limit = self._joint_limits.get(name)
                if limit:
                    target = max(limit[0], min(limit[1], target))

                kp_base, kd_base = DEFAULT_GAINS[name]
                cmd = MITCommand()
                cmd.joint_name = name
                cmd.position = float(target)
                cmd.velocity = 0.0
                cmd.kp = float(kp_base * gs)
                cmd.kd = float(kd_base * gs)
                cmd.torque_ff = 0.0
                cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

    def _handle_step_test(self):
        """Square wave step response test."""
        elapsed = time.time() - self._state_start_time
        if elapsed <= self._ramp_time + self._stable_time:
            self._handle_stand()
            return

        current_time = time.time()
        gs = float(self.get_parameter("gain_scale").value)
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        if getattr(self, '_wiggle_interpolating', False):
            alpha = min((current_time - self._interp_start_time) / 3.0, 1.0)
            if alpha >= 1.0:
                self._wiggle_interpolating = False
                self._active_joint_idx = self._target_joint_idx
                self._wiggle_sine_start_time = current_time
            
            for i, name in enumerate(JOINT_ORDER):
                target = DEFAULT_POSITIONS[name]
                if i == self._active_joint_idx:
                    target += self._interp_start_pos * (1.0 - alpha)

                limit = self._joint_limits.get(name)
                if limit:
                    target = max(limit[0], min(limit[1], target))

                kp_base, kd_base = DEFAULT_GAINS[name]
                cmd = MITCommand()
                cmd.joint_name = name
                cmd.position = float(target)
                cmd.velocity = 0.0
                cmd.kp = float(kp_base * gs)
                cmd.kd = float(kd_base * gs)
                cmd.torque_ff = 0.0
                cmd_msg.commands.append(cmd)
        else:
            for i, name in enumerate(JOINT_ORDER):
                target = DEFAULT_POSITIONS[name]

                if i == getattr(self, '_active_joint_idx', -1):
                    jcfg = self._step_cfg['joints'].get(
                        name, {'step_max': 0.174, 'step_min': -0.174, 'period': 2.0}
                    )
                    phase_t = current_time - self._wiggle_sine_start_time
                    cycle_time = math.fmod(phase_t, jcfg['period'])
                    
                    if cycle_time < (jcfg['period'] / 2.0):
                        offset = jcfg['step_max']
                    else:
                        offset = jcfg['step_min']
                        
                    target += offset

                limit = self._joint_limits.get(name)
                if limit:
                    target = max(limit[0], min(limit[1], target))

                kp_base, kd_base = DEFAULT_GAINS[name]
                cmd = MITCommand()
                cmd.joint_name = name
                cmd.position = float(target)
                cmd.velocity = 0.0
                cmd.kp = float(kp_base * gs)
                cmd.kd = float(kd_base * gs)
                cmd.torque_ff = 0.0
                cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

    def _handle_wiggle_all(self):
        elapsed = time.time() - self._state_start_time
        if elapsed <= self._ramp_time + self._stable_time:
            self._handle_stand()
            return
            
        gs = float(self.get_parameter("gain_scale").value)
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for name in JOINT_ORDER:
            target = DEFAULT_POSITIONS[name]
            jcfg = self._wiggle_cfg['joints'].get(
                name, {'pos': 0.087, 'neg': -0.087, 'freq': 1.0}
            )
            sin_val = math.sin(2 * math.pi * jcfg['freq'] * (time.time() - self._wiggle_start))
            offset = (jcfg['pos'] * sin_val) if sin_val >= 0 else (-jcfg['neg'] * sin_val)
            target += offset

            limit = self._joint_limits.get(name)
            if limit:
                target = max(limit[0], min(limit[1], target))

            kp_base, kd_base = DEFAULT_GAINS[name]
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = float(target)
            cmd.velocity = 0.0
            cmd.kp = float(kp_base * gs)
            cmd.kd = float(kd_base * gs)
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

    def _publish_state_cb(self):
        sm = String()
        sm.data = self._state
        self._pub_state.publish(sm)

        rm = RobotState()
        rm.header.stamp = self.get_clock().now().to_msg()
        rm.fsm_state = self._state
        rm.safety_ok = self._safety_ok
        rm.all_motors_enabled = (self._state not in ("IDLE", "ESTOP"))
        self._pub_robot.publish(rm)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
