#!/usr/bin/env python3
"""Turn the control board's raw odometry into a well-formed ROS 2 odometry source.

The ESP32 microROS board publishes ``/odom_raw`` (nav_msgs/Odometry) at ~20 Hz, but:

* its header stamp comes from the board's own clock, which is not synchronised with the
  Pi/PC - TF and Nav2 both reject messages whose stamps are in the future,
* the frame ids it sets are whatever the firmware was compiled with,
* the covariance blocks are left as all-zeros, which robot_localization and AMCL read as
  "infinitely confident" and which quietly ruins any fusion you do later.

This node fixes all three and (optionally) broadcasts the ``odom -> base_footprint``
transform. Turn ``publish_tf`` off the moment you add an EKF, or you will have two
publishers fighting over the same transform.
"""

import copy
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster

# micro-ROS publishes best-effort; matching that here avoids a silent no-data subscription.
BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class OdomRepublisher(Node):

    def __init__(self):
        super().__init__('odom_republisher')

        self.declare_parameter('input_topic', 'odom_raw')
        self.declare_parameter('output_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('stamp_with_ros_time', True)
        self.declare_parameter('best_effort_input', True)
        # x, y, z, roll, pitch, yaw. Wheel odometry is blind in z/roll/pitch, hence the
        # large values there - that is the honest answer, not a bug.
        self.declare_parameter('pose_covariance_diagonal',
                               [0.001, 0.001, 1e6, 1e6, 1e6, 0.03])
        self.declare_parameter('twist_covariance_diagonal',
                               [0.001, 0.001, 1e6, 1e6, 1e6, 0.03])

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.stamp_with_ros_time = self.get_parameter('stamp_with_ros_time').value
        self.pose_cov = list(self.get_parameter('pose_covariance_diagonal').value)
        self.twist_cov = list(self.get_parameter('twist_covariance_diagonal').value)

        qos = BEST_EFFORT_QOS if self.get_parameter('best_effort_input').value else 10

        self.sub = self.create_subscription(
            Odometry, self.input_topic, self.on_odom, qos)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.msg_count = 0
        self.create_timer(5.0, self.warn_if_silent)

        self.get_logger().info(
            f"republishing '{self.input_topic}' -> '{self.output_topic}' "
            f"({self.odom_frame} -> {self.base_frame}), publish_tf={self.publish_tf}")

    def on_odom(self, msg: Odometry):
        pose = msg.pose.pose
        twist = msg.twist.twist
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                  twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z]
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn('Discarding non-finite odometry')
            return
        self.msg_count += 1
        now = self.get_clock().now().to_msg()

        out = Odometry()
        out.header.stamp = now if self.stamp_with_ros_time else msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose = copy.deepcopy(msg.pose.pose)
        out.twist.twist = copy.deepcopy(msg.twist.twist)

        # The board leaves covariance zeroed; only overwrite when that is the case, so a
        # future firmware that does fill it in keeps its own numbers.
        if not any(msg.pose.covariance):
            for i in range(6):
                out.pose.covariance[i * 6 + i] = self.pose_cov[i]
        else:
            out.pose.covariance = msg.pose.covariance

        if not any(msg.twist.covariance):
            for i in range(6):
                out.twist.covariance[i * 6 + i] = self.twist_cov[i]
        else:
            out.twist.covariance = msg.twist.covariance

        # An unnormalised quaternion makes tf2 throw; the board's integrator drifts.
        q = out.pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
        else:
            q.x, q.y, q.z, q.w = q.x / norm, q.y / norm, q.z / norm, q.w / norm

        self.pub.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf)

    def warn_if_silent(self):
        if self.msg_count == 0:
            self.get_logger().warn(
                f"no messages on '{self.input_topic}' yet - is the micro-ROS agent running "
                f"and the board connected? Check with: ros2 topic hz {self.input_topic}")
        self.msg_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
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
