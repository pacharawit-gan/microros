"""Provide locally stamped, reliable gyro data for the optional EKF."""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')
        self.declare_parameter('input_topic', 'imu')
        self.declare_parameter('output_topic', 'imu_sanitized')
        self.declare_parameter('gyro_variance', 0.03)
        self.gyro_variance = self.get_parameter('gyro_variance').value
        if not math.isfinite(self.gyro_variance) or self.gyro_variance <= 0.0:
            raise ValueError('gyro_variance must be finite and positive')
        self.pub = self.create_publisher(Imu, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Imu, self.get_parameter('input_topic').value,
                                 self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        gyro = msg.angular_velocity
        if msg.angular_velocity_covariance[0] < 0.0 or not all(
                math.isfinite(v) for v in (gyro.x, gyro.y, gyro.z)):
            self.get_logger().warn('Discarding unavailable or non-finite gyro data')
            return
        out = copy.deepcopy(msg)
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'imu_frame'
        # Only gyro is calibrated for this path. Do not advertise acceleration or
        # orientation as valid measurements to an accidental additional consumer.
        out.orientation_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        cov = out.angular_velocity_covariance
        if (not all(math.isfinite(v) for v in cov)
                or any(cov[i] <= 0.0 for i in (0, 4, 8))):
            out.angular_velocity_covariance = [
                self.gyro_variance, 0.0, 0.0,
                0.0, self.gyro_variance, 0.0,
                0.0, 0.0, self.gyro_variance]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
