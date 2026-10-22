#!/usr/bin/env python3
"""Publish /joint_states so robot_state_publisher can complete the TF tree.

The control board reports body velocity, not per-wheel encoder angles, so there is nothing
to forward directly. This node reconstructs plausible wheel angles from the body twist
using the differential-drive inverse kinematics:

    v_left  = v_x - w_z * (track / 2)
    v_right = v_x + w_z * (track / 2)
    d(theta_wheel)/dt = v_wheel / r

**These angles are for visualisation only.** They are derived from the same odometry they
would otherwise validate, so they can never disagree with it - do not use them as encoder
feedback. If you later expose real encoder ticks from the ESP32 firmware, publish those
here instead and delete the integration.

The gimbal joints are published too (default 0.0), because robot_state_publisher will not
emit TF for ``camera_link`` until every joint on the chain has a state.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

WHEEL_JOINTS = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]
GIMBAL_JOINTS = ['gimbal_yaw_joint', 'gimbal_pitch_joint']

TWO_PI = 2.0 * 3.14159265358979323846


class WheelJointPublisher(Node):

    def __init__(self):
        super().__init__('wheel_joint_publisher')

        self.declare_parameter('odom_topic', 'odom_raw')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('odom_timeout', 0.5)
        # Must match microros_pi5.urdf.xacro - keep the two in sync when you re-measure.
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.180)
        self.declare_parameter('publish_gimbal_joints', True)
        self.declare_parameter('best_effort_input', True)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_gimbal = self.get_parameter('publish_gimbal_joints').value
        rate = self.get_parameter('publish_rate').value

        qos = BEST_EFFORT_QOS if self.get_parameter('best_effort_input').value else 10
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, qos)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.last_odom_time = None
        self.vx = 0.0
        self.wz = 0.0
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.left_rate = 0.0
        self.right_rate = 0.0
        self.last_time = self.get_clock().now()

        self.create_timer(1.0 / rate, self.tick)

    def on_odom(self, msg: Odometry):
        if not all(math.isfinite(v) for v in (msg.twist.twist.linear.x, msg.twist.twist.angular.z)):
            self.vx = self.wz = 0.0
            return
        self.last_odom_time = self.get_clock().now()
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:      # first tick, or we were paused/suspended
            return

        age = None if self.last_odom_time is None else (now - self.last_odom_time).nanoseconds * 1e-9
        if age is None or age < 0.0 or age > self.odom_timeout:
            self.vx = self.wz = 0.0
        half_track = self.wheel_separation / 2.0
        v_left = self.vx - self.wz * half_track
        v_right = self.vx + self.wz * half_track

        self.left_rate = v_left / self.wheel_radius
        self.right_rate = v_right / self.wheel_radius

        # Wrapping keeps the accumulated float small over a long run.
        self.left_angle = (self.left_angle + self.left_rate * dt) % TWO_PI
        self.right_angle = (self.right_angle + self.right_rate * dt) % TWO_PI

        # front_left, front_right, rear_left, rear_right - the two wheels on a side are
        # rigidly coupled on this chassis, so they share an angle.
        names = list(WHEEL_JOINTS)
        positions = [self.left_angle, self.right_angle, self.left_angle, self.right_angle]
        velocities = [self.left_rate, self.right_rate, self.left_rate, self.right_rate]

        if self.publish_gimbal:
            names += list(GIMBAL_JOINTS)
            positions += [0.0, 0.0]
            velocities += [0.0, 0.0]

        # Assign once: rosidl turns float64[] fields into array.array on assignment, and
        # array.array will not accept `+= [list]` afterwards.
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = names
        msg.position = positions
        msg.velocity = velocities

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelJointPublisher()
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
