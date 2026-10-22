#!/usr/bin/env python3
"""Impersonate the ESP32-S3 microROS control board.

This node stands in for the real hardware at exactly the boundary the hardware occupies:

    subscribes  /cmd_vel    geometry_msgs/Twist     <- what the board obeys
    publishes   /odom_raw   nav_msgs/Odometry       -> what the board reports, ~20 Hz
    publishes   /imu        sensor_msgs/Imu         -> what the board reports, ~20 Hz

Nothing else in the stack changes. `odom_republisher`, `wheel_joint_publisher`,
`cmd_vel_safety` and `robot_state_publisher` all run exactly as they will on the real car,
so what you debug here is the code you ship. Swap this node for the micro-ROS agent and
the robot behaves the same way.

It deliberately imitates the board's *flaws*, not just its interface:

* publishes BEST_EFFORT, like micro-ROS does, so QoS mismatches show up here and not on the
  robot,
* leaves the covariance blocks zeroed, so `odom_republisher` has something to fix,
* stamps messages with a clock that can be offset from ROS time (``clock_offset``),
  reproducing the unsynchronised-ESP32 problem that breaks tf2,
* applies a first-order lag to velocity, because motors do not step instantly.

This is kinematic, not physical: no mass, no wheel slip beyond the optional noise term, no
collisions. It cannot tell you whether the robot tips over or whether a corridor is wide
enough - that needs Gazebo.
"""

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

# micro-ROS publishes best-effort. Matching it here means a QoS bug surfaces in sim.
BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def yaw_to_quaternion(yaw: float):
    """Yaw-only rotation -> (x, y, z, w). Avoids a tf_transformations dependency."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class FakeBoard(Node):

    def __init__(self):
        super().__init__('fake_board')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom_raw')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('publish_rate', 20.0)      # the real board runs at ~20 Hz
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # Plant model
        self.declare_parameter('velocity_tau', 0.15)      # s, first-order motor lag
        self.declare_parameter('max_linear_x', 1.0)       # m/s, hardware ceiling
        self.declare_parameter('max_angular_z', 3.0)      # rad/s
        self.declare_parameter('cmd_timeout', 1.0)        # s; the real board holds the last
                                                          # command - see the note below

        # Imperfections, all off by default so first runs are clean
        self.declare_parameter('linear_noise_std', 0.0)   # m/s
        self.declare_parameter('angular_noise_std', 0.0)  # rad/s
        self.declare_parameter('imu_noise_std', 0.0)      # rad/s
        self.declare_parameter('clock_offset', 0.0)       # s added to published stamps

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tau = self.get_parameter('velocity_tau').value
        self.max_v = self.get_parameter('max_linear_x').value
        self.max_w = self.get_parameter('max_angular_z').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.lin_noise = self.get_parameter('linear_noise_std').value
        self.ang_noise = self.get_parameter('angular_noise_std').value
        self.imu_noise = self.get_parameter('imu_noise_std').value
        self.clock_offset = self.get_parameter('clock_offset').value

        rate = self.get_parameter('publish_rate').value
        self.dt = 1.0 / rate

        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd, 10)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, BEST_EFFORT_QOS)
        self.imu_pub = self.create_publisher(
            Imu, self.get_parameter('imu_topic').value, BEST_EFFORT_QOS)

        # Pose in the odom frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Commanded vs achieved body velocity
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.v = 0.0
        self.w = 0.0
        self.prev_v = 0.0
        self.last_cmd_time = None

        self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"faking the microROS board: subscribing '{self.cmd_vel_topic}', publishing "
            f"'{self.get_parameter('odom_topic').value}' + "
            f"'{self.get_parameter('imu_topic').value}' at {rate} Hz")

    def on_cmd(self, msg: Twist):
        self.cmd_v = max(-self.max_v, min(self.max_v, msg.linear.x))
        self.cmd_w = max(-self.max_w, min(self.max_w, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def _stamp(self):
        """Board clock: ROS time plus whatever drift we were told to simulate."""
        now = self.get_clock().now()
        if self.clock_offset:
            now = rclpy.time.Time(
                nanoseconds=now.nanoseconds + int(self.clock_offset * 1e9))
        return now.to_msg()

    def tick(self):
        # The real board holds its last command indefinitely - that is precisely why
        # cmd_vel_safety exists. cmd_timeout only exists so a forgotten sim does not
        # drive off to infinity; set it to 0.0 to reproduce the board's real behaviour.
        if self.cmd_timeout > 0.0 and self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.cmd_v = 0.0
                self.cmd_w = 0.0

        # First-order lag towards the commanded velocity: motors have inertia.
        alpha = self.dt / max(self.tau, 1e-6) if self.tau > 0.0 else 1.0
        alpha = min(alpha, 1.0)
        self.v += (self.cmd_v - self.v) * alpha
        self.w += (self.cmd_w - self.w) * alpha

        v = self.v + (random.gauss(0.0, self.lin_noise) if self.lin_noise else 0.0)
        w = self.w + (random.gauss(0.0, self.ang_noise) if self.ang_noise else 0.0)

        # Exact integration over the step for a constant-curvature arc. The naive
        # Euler form drifts noticeably while turning, which would make the sim lie
        # about odometry quality.
        if abs(w) < 1e-9:
            self.x += v * math.cos(self.theta) * self.dt
            self.y += v * math.sin(self.theta) * self.dt
        else:
            theta_new = self.theta + w * self.dt
            self.x += (v / w) * (math.sin(theta_new) - math.sin(self.theta))
            self.y -= (v / w) * (math.cos(theta_new) - math.cos(self.theta))
            self.theta = math.atan2(math.sin(theta_new), math.cos(theta_new))

        stamp = self._stamp()
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Covariance left zeroed on purpose - the real board does the same, and
        # odom_republisher is the node whose job it is to fill it in.
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_frame'
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.z = w + (
            random.gauss(0.0, self.imu_noise) if self.imu_noise else 0.0)
        imu.linear_acceleration.x = (v - self.prev_v) / self.dt
        imu.linear_acceleration.y = v * w          # centripetal term while turning
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        self.prev_v = v


def main(args=None):
    rclpy.init(args=args)
    node = FakeBoard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
