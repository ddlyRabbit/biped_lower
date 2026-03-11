"""Keyboard velocity commander for biped robot.

Turtlebot-style keyboard control. Publishes geometry_msgs/Twist to /cmd_vel.
Each keypress increments/decrements velocity in steps. Holds last command.

Controls:
    w/s : forward/backward  (±step m/s)
    a/d : left/right        (±step m/s)
    q/e : yaw left/right    (±step rad/s)
    x   : full stop (zero all)
    1-5 : speed presets (0.1, 0.3, 0.5, 0.8, 1.0 m/s)
    +/- : increase/decrease step size
    ESC/Ctrl+C : quit

Usage:
    ros2 run biped_teleop keyboard_teleop
"""

import sys
import time
import threading
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
┌─────────────────────────────────────────┐
│        Biped Keyboard Teleop            │
│                                         │
│   w/s : forward / backward              │
│   a/d : left / right                    │
│   q/e : yaw left / yaw right            │
│                                         │
│   x   : STOP (zero all velocities)      │
│   1-5 : speed presets                   │
│         1=0.1  2=0.3  3=0.5             │
│         4=0.8  5=1.0  m/s               │
│   +/- : increase/decrease step size     │
│                                         │
│   ESC / Ctrl+C : quit                   │
└─────────────────────────────────────────┘
"""

# Check if we have a real terminal
_HAS_TTY = False
try:
    termios.tcgetattr(sys.stdin.fileno())
    _HAS_TTY = True
except Exception:
    pass


def get_key(timeout=0.1):
    """Read a single keypress with timeout."""
    if not _HAS_TTY:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 1.0)

        self._lin_step = float(self.get_parameter('linear_step').value)
        self._ang_step = float(self.get_parameter('angular_step').value)
        self._max_lin = float(self.get_parameter('max_linear').value)
        self._max_ang = float(self.get_parameter('max_angular').value)

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._running = True

        self._presets = {'1': 0.1, '2': 0.3, '3': 0.5, '4': 0.8, '5': 1.0}

    def _clamp(self, val, limit):
        return max(-limit, min(limit, val))

    def _publish(self):
        try:
            msg = Twist()
            msg.linear.x = self._vx
            msg.linear.y = self._vy
            msg.angular.z = self._wz
            self._pub.publish(msg)
        except Exception:
            pass  # context may be invalid during shutdown

    def _print_status(self):
        sys.stdout.write(
            f'\r  vx={self._vx:+.2f}  vy={self._vy:+.2f}  '
            f'wz={self._wz:+.2f}  step={self._lin_step:.2f}    '
        )
        sys.stdout.flush()

    def run(self):
        print(BANNER)
        self._print_status()

        # Publish in a background thread at 10Hz
        def publish_loop():
            while self._running and rclpy.ok():
                self._publish()
                time.sleep(0.1)

        pub_thread = threading.Thread(target=publish_loop, daemon=True)
        pub_thread.start()

        try:
            while self._running and rclpy.ok():
                key = get_key(timeout=0.1)

                if key == '':
                    continue

                changed = True

                if key == 'w':
                    self._vx = self._clamp(self._vx + self._lin_step, self._max_lin)
                elif key == 's':
                    self._vx = self._clamp(self._vx - self._lin_step, self._max_lin)
                elif key == 'a':
                    self._vy = self._clamp(self._vy + self._lin_step, self._max_lin)
                elif key == 'd':
                    self._vy = self._clamp(self._vy - self._lin_step, self._max_lin)
                elif key == 'q':
                    self._wz = self._clamp(self._wz + self._ang_step, self._max_ang)
                elif key == 'e':
                    self._wz = self._clamp(self._wz - self._ang_step, self._max_ang)
                elif key == 'x':
                    self._vx = self._vy = self._wz = 0.0
                elif key in self._presets:
                    self._vx = self._presets[key]
                    self._vy = self._wz = 0.0
                elif key in ('+', '='):
                    self._lin_step = min(self._lin_step + 0.05, 0.5)
                    self._ang_step = min(self._ang_step + 0.05, 0.5)
                elif key == '-':
                    self._lin_step = max(self._lin_step - 0.05, 0.01)
                    self._ang_step = max(self._ang_step - 0.05, 0.01)
                elif key in ('\x1b', '\x03'):
                    break
                else:
                    changed = False

                if changed:
                    self._print_status()

        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            self._vx = self._vy = self._wz = 0.0
            self._publish()
            print('\n\nStopped.')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
