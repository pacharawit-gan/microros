#!/usr/bin/env python3
"""Hold-to-drive keyboard teleop, publishing to ``cmd_vel_raw``.

``teleop_twist_keyboard`` does not work well against ``cmd_vel_safety``: its ``getKey()``
blocks on ``stdin.read(1)`` and publishes exactly once per keystroke, so while you hold a
key nothing is sent during the terminal's auto-repeat delay (~660 ms on Ubuntu). That is
longer than the 0.5 s dead-man, so the robot lurches, stops, then goes - and the console
fills with "cmd_vel_raw went stale" warnings.

This node instead:

* **publishes continuously at ``rate`` Hz, always**, even when the command is zero. The
  dead-man in ``cmd_vel_safety`` then means what it is supposed to mean - *teleop died or
  the link dropped* - rather than *you are between key repeats*.
* **holds the last key's command for ``key_timeout`` seconds**, refreshed by every repeat.
  Release the key and the command zeroes on its own.

``key_timeout`` must exceed the terminal's auto-repeat delay or holding a key stutters.
Check yours with ``xset q | grep -A1 'auto repeat delay'``; the 0.75 s default covers the
660 ms standard. If you shorten the delay (``xset r rate 250 30``) you can drop
``key_timeout`` to ~0.35 s and the robot will stop sooner after you let go.

**Releasing a key is not an emergency stop.** Worst case the robot keeps its command for
``key_timeout``, then ``cmd_vel_safety`` ramps it down at ``max_linear_accel``. Hit
**space** or **k** to request zero; the safety node still ramps down. Default speeds are deliberately low for that reason.
"""

import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (linear_x, angular_z) as fractions of the current speed/turn scale.
MOVE_BINDINGS = {
    'i': (1.0, 0.0),   ',': (-1.0, 0.0),
    'j': (0.0, 1.0),   'l': (0.0, -1.0),
    'u': (1.0, 1.0),   'o': (1.0, -1.0),
    'm': (-1.0, 1.0),  '.': (-1.0, -1.0),
    # WASD for anyone who does not have the ijkl layout in muscle memory.
    'w': (1.0, 0.0),   's': (-1.0, 0.0),
    'a': (0.0, 1.0),   'd': (0.0, -1.0),
    # Arrow keys arrive as escape sequences; normalised to these names below.
    'UP': (1.0, 0.0),  'DOWN': (-1.0, 0.0),
    'LEFT': (0.0, 1.0), 'RIGHT': (0.0, -1.0),
}

# key -> (speed factor, turn factor)
SPEED_BINDINGS = {
    'q': (1.1, 1.1), 'z': (0.9, 0.9),
    'e': (1.1, 1.0), 'c': (0.9, 1.0),
    'r': (1.0, 1.1), 'v': (1.0, 0.9),
}

BANNER = """
MicroROS-Pi5 keyboard teleop  ->  /cmd_vel_raw

        u  i  o          forward / turn
        j  k  l          k = stop
        m  ,  .          (w a s d and arrow keys also work)

  space/k : request stop          q/z : all speeds  +/- 10%
  e/c   : linear  +/- 10%   r/v : angular +/- 10%
  Ctrl-C or ESC : quit (publishes a zero on the way out)
"""


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('output_topic', 'cmd_vel_raw')
        self.declare_parameter('rate', 20.0)
        # Gentle by default: this is what a first drive on a real robot should feel like.
        # cmd_vel_safety clamps to max_linear_x anyway, so raising these cannot exceed it.
        self.declare_parameter('speed', 0.15)        # m/s
        self.declare_parameter('turn', 0.6)          # rad/s
        self.declare_parameter('key_timeout', 0.75)  # s; must exceed auto-repeat delay

        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        self.key_timeout = self.get_parameter('key_timeout').value
        rate = self.get_parameter('rate').value

        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

        self.x = 0.0
        self.th = 0.0
        self.last_key_time = None
        self.create_timer(1.0 / rate, self.tick)

    def apply_key(self, key: str) -> bool:
        """Returns False if the user asked to quit."""
        if key in ('\x03', '\x1b'):          # Ctrl-C, ESC
            return False
        if key == ' ':
            self.x = self.th = 0.0
            self.last_key_time = None
            return True
        if key in MOVE_BINDINGS:
            self.x, self.th = MOVE_BINDINGS[key]
            self.last_key_time = self.get_clock().now()
            return True
        if key in SPEED_BINDINGS:
            sf, tf = SPEED_BINDINGS[key]
            self.speed *= sf
            self.turn *= tf
            self.report()
            return True
        # Any other key (including 'k') is an explicit stop.
        self.x = self.th = 0.0
        self.last_key_time = None
        return True

    def tick(self):
        if self.last_key_time is not None:
            age = (self.get_clock().now() - self.last_key_time).nanoseconds * 1e-9
            if age > self.key_timeout:
                self.x = self.th = 0.0
                self.last_key_time = None

        msg = Twist()
        msg.linear.x = self.x * self.speed
        msg.angular.z = self.th * self.turn
        self.pub.publish(msg)

    def report(self):
        sys.stdout.write(
            '\r  speed %.2f m/s   turn %.2f rad/s        ' % (self.speed, self.turn))
        sys.stdout.flush()

    def stop(self):
        self.pub.publish(Twist())


def read_key(timeout: float):
    """Non-blocking single keypress. Arrow keys come back as 'UP'/'DOWN'/'LEFT'/'RIGHT'."""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch != '\x1b':
        return ch
    # Escape: either a bare ESC (quit) or the start of an arrow sequence.
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return '\x1b'
    rest = sys.stdin.read(2)
    return {'[A': 'UP', '[B': 'DOWN', '[C': 'RIGHT', '[D': 'LEFT'}.get(rest, '')


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_teleop needs a terminal. Run it with `ros2 run`, not `ros2 launch`.',
              file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = KeyboardTeleop()

    if not os.environ.get('ROS_DOMAIN_ID'):
        node.get_logger().warn(
            'ROS_DOMAIN_ID is unset. The board is on domain 20 - you will drive nothing.')

    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    node.report()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.02)
            if key is not None and not node.apply_key(key):
                break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # BaseException, not Exception: a second SIGINT lands as KeyboardInterrupt and would
        # otherwise skip the stopping publish. Never leave the board holding a command.
        try:
            node.stop()
        except BaseException:
            pass
        print()
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
