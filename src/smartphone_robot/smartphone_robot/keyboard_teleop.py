#!/usr/bin/env python3
"""Drive the robot from the keyboard with the arrow keys.

This is a teleop node: it reads keypresses and publishes velocity on /cmd_vel, the same
topic Nav2 and the object follower use. It exists so the robot can be driven by hand --
to position it, to test that it responds, or just to see the wheels turn.

    Up / W     forward          Down / S   backward
    Left / A   turn left        Right / D  turn right
    Space      stop             q          quit
    + / -      faster / slower

HOW ARROW KEYS ARE READ
=======================
A terminal delivers an arrow key not as one byte but as a three-byte escape sequence:
ESC '[' 'A' for Up, 'B' Down, 'C' Right, 'D' Left. To catch them the terminal is put in
"raw" mode (cbreak, no echo) so each keystroke arrives immediately instead of waiting for
Enter, and the escape sequence is decoded by hand. The original terminal settings are
saved and restored on exit, so the shell is not left broken.

SAFETY
======
Velocity is a latched command -- the base keeps the last speed until told otherwise -- so
this node stops the robot on quit and whenever no key has been pressed recently. Run it
INSTEAD of Nav2, not alongside it: two things writing /cmd_vel fight over the base.
"""

import sys
import select
import termios
import tty
import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

HELP = """
  Drive the robot:
    Up / W      forward         Down / S    backward
    Left / A    turn left       Right / D   turn right
    Space       stop            + / -       faster / slower
    q           quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed', 0.25)   # m/s
        self.declare_parameter('angular_speed', 0.8)   # rad/s
        self.declare_parameter('key_timeout', 0.5)     # s of no key -> stop

        self.v = self.get_parameter('linear_speed').value
        self.w = self.get_parameter('angular_speed').value
        self.key_timeout = self.get_parameter('key_timeout').value

        # When true, publish nothing until the first movement key is pressed. This lets
        # teleop share a launch with an auto-driver (wander_node during mapping) without
        # both writing /cmd_vel at once: teleop stays silent while the robot maps itself,
        # and takes over the moment you press an arrow. Once you have driven, the normal
        # "no key -> stop" safety applies.
        self.declare_parameter('silent_until_first_key', False)
        self.silent = self.get_parameter('silent_until_first_key').value
        self.activated = not self.silent

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lin = 0.0
        self.ang = 0.0

    def read_key(self):
        """Return the next key as a short string, or '' on timeout. Handles arrows."""
        r, _, _ = select.select([sys.stdin], [], [], self.key_timeout)
        if not r:
            return ''
        ch = sys.stdin.read(1)
        if ch == '\x1b':                     # ESC -- maybe an arrow sequence
            seq = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.01)[0] else ''
            return {'[A': 'up', '[B': 'down', '[C': 'right', '[D': 'left'}.get(seq, 'esc')
        return ch

    def step(self, key):
        """Update the commanded velocity from one key, then publish it."""
        if key in ('up', 'w'):
            self.lin, self.ang = self.v, 0.0
        elif key in ('down', 's'):
            self.lin, self.ang = -self.v, 0.0
        elif key in ('left', 'a'):
            self.lin, self.ang = 0.0, self.w
        elif key in ('right', 'd'):
            self.lin, self.ang = 0.0, -self.w
        elif key == ' ':
            self.lin, self.ang = 0.0, 0.0
        elif key in ('+', '='):
            self.v = min(1.0, self.v + 0.05)
            self.w = min(2.0, self.w + 0.1)
            self._status(f'speed up: {self.v:.2f} m/s, {self.w:.2f} rad/s')
        elif key in ('-', '_'):
            self.v = max(0.05, self.v - 0.05)
            self.w = max(0.1, self.w - 0.1)
            self._status(f'slow down: {self.v:.2f} m/s, {self.w:.2f} rad/s')
        elif key == '':
            # No key this cycle -> stop, so a released key does not leave the robot going.
            self.lin, self.ang = 0.0, 0.0

        # A real movement/stop key activates teleop. Until then, in shared-launch mode,
        # stay completely silent so we do not fight the auto-driver.
        if key in ('up', 'down', 'left', 'right', 'w', 'a', 's', 'd', ' '):
            if not self.activated:
                self.activated = True
                self._status('teleop active -- you are now driving')

        if not self.activated:
            return

        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)

    def _status(self, text):
        sys.stdout.write('\r' + text + ' ' * 10)
        sys.stdout.flush()

    def stop(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()

    # This node reads raw keystrokes, which only works from a real terminal. Run via
    # `ros2 run` in a terminal, or via teleop.launch.py which opens an xterm for it.
    if not sys.stdin.isatty():
        node.get_logger().error(
            'keyboard_teleop needs a terminal (a TTY) to read the arrow keys. '
            'Run it with `ros2 run smartphone_robot keyboard_teleop` in a terminal, '
            'or use `ros2 launch smartphone_robot teleop.launch.py`.'
        )
        node.destroy_node()
        rclpy.try_shutdown()
        return

    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    stop = False

    def request_stop(_s, _f):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not stop:
            key = node.read_key()
            if key == 'q':
                break
            node.step(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        # Restore the terminal FIRST, so the shell is usable even if the stop fails.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
        print('\nStopped.')


if __name__ == '__main__':
    main()
