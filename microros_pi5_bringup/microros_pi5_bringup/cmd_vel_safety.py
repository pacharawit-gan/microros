#!/usr/bin/env python3
"""The only node allowed to publish /cmd_vel.

Everything that wants to drive the robot (teleop, Nav2, your own behaviour nodes)
publishes to ``cmd_vel_raw``; this node clamps, slew-rate-limits and dead-mans it before
forwarding to ``cmd_vel``, which the ESP32 board subscribes to.

Why bother:

* **Clamping** stops a typo'd Nav2 parameter or a stuck key from commanding 5 m/s into a
  wall.
* **Slew limiting** stops the step commands teleop sends from browning out the 7.4 V pack
  and resetting the Pi - four 370 motors starting simultaneously is a real current spike.
* **Timeout** ramps to zero when upstream commands stop, provided this node can still
  reach the board. Loss of the board link or this process requires a board-side watchdog.

Do not run a second publisher on ``cmd_vel``; two publishers on a motion topic means the
robot obeys whichever arrived last, which is not a control system.
"""

import math

import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from rclpy.node import Node


def _approach(current: float, target: float, max_delta: float) -> float:
    """Move `current` towards `target` by at most `max_delta`."""
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value)) if math.isfinite(value) else 0.0


class CmdVelSafety(Node):

    def __init__(self):
        super().__init__('cmd_vel_safety')

        self.declare_parameter('input_topic', 'cmd_vel_raw')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('rate', 20.0)
        # Stock MicroROS-Pi5 tops out near 1.0 m/s; 0.5 is a sane default for indoor work.
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.0)      # 0 => reject lateral commands
        self.declare_parameter('max_angular_z', 2.0)
        self.declare_parameter('max_linear_accel', 1.0)   # m/s^2
        self.declare_parameter('max_angular_accel', 3.0)  # rad/s^2
        self.declare_parameter('timeout', 0.5)            # s without input => ramp to stop

        for name in ('rate', 'timeout', 'max_linear_accel', 'max_angular_accel',
                     'max_linear_x', 'max_linear_y', 'max_angular_z'):
            value = self.get_parameter(name).value
            positive = name in ('rate', 'timeout', 'max_linear_accel', 'max_angular_accel')
            if not math.isfinite(value) or value < 0 or (positive and value == 0):
                raise ValueError(f'{name} must be finite and {"positive" if positive else "nonnegative"}')
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_linear_y = self.get_parameter('max_linear_y').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.timeout = self.get_parameter('timeout').value
        rate = self.get_parameter('rate').value
        self.dt = 1.0 / rate
        self.max_dv = self.get_parameter('max_linear_accel').value * self.dt
        self.max_dw = self.get_parameter('max_angular_accel').value * self.dt

        self.target = Twist()
        self.current = Twist()
        self.last_input_time = None
        self.timed_out_logged = False

        self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.on_cmd, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        self.create_timer(self.dt, self.tick, clock=self.safety_clock)

        self.get_logger().info(
            f"limits: vx<={self.max_linear_x} m/s, wz<={self.max_angular_z} rad/s, "
            f"timeout={self.timeout}s")

    def on_cmd(self, msg: Twist):
        if not all(math.isfinite(v) for v in
                   (msg.linear.x, msg.linear.y, msg.linear.z,
                    msg.angular.x, msg.angular.y, msg.angular.z)):
            self.target = Twist()
            self.last_input_time = None
            self.get_logger().warn('Rejected non-finite velocity; ramping to stop')
            return
        self.target.linear.x = _clamp(msg.linear.x, self.max_linear_x)
        self.target.linear.y = _clamp(msg.linear.y, self.max_linear_y)
        self.target.angular.z = _clamp(msg.angular.z, self.max_angular_z)
        self.last_input_time = self.safety_clock.now()
        self.timed_out_logged = False

    def tick(self):
        now = self.safety_clock.now()

        stale = (
            self.last_input_time is None
            or (now - self.last_input_time).nanoseconds * 1e-9 > self.timeout
        )
        if stale:
            if not self.timed_out_logged and self.last_input_time is not None:
                self.get_logger().warn('cmd_vel_raw went stale - ramping to stop')
                self.timed_out_logged = True
            self.target = Twist()

        self.current.linear.x = _approach(self.current.linear.x, self.target.linear.x, self.max_dv)
        self.current.linear.y = _approach(self.current.linear.y, self.target.linear.y, self.max_dv)
        self.current.angular.z = _approach(
            self.current.angular.z, self.target.angular.z, self.max_dw)

        # Continue zero heartbeats at rest, including startup and after reconnection.
        # A board-side watchdog is still required if this process/link disappears.
        self.pub.publish(self.current)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot stopped rather than holding the last command.
        #
        # These catch BaseException, not Exception, on purpose. A second SIGINT (launch
        # forwards one, and a supervising script may send its own) arrives as a
        # KeyboardInterrupt, which is a BaseException - `except Exception` would let it
        # through and skip the stopping publish, leaving the board driving.
        try:
            node.pub.publish(Twist())
        except BaseException:
            pass
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
