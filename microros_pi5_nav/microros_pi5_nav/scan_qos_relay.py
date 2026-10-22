#!/usr/bin/env python3
"""Bridge the board's BEST_EFFORT /scan to a RELIABLE topic. Optional safety net.

The MS200 lidar is wired to the ESP32, so /scan arrives over micro-ROS with
BEST_EFFORT reliability (verified 2026-08-31). A RELIABLE subscriber will not
match a BEST_EFFORT publisher - DDS treats that as an incompatible QoS contract
and the subscription simply never fires. No error, no warning, just silence,
which looks identical to a dead lidar.

slam_toolbox and Nav2's costmap both *should* subscribe with sensor-data QoS
(BEST_EFFORT), in which case you do not need this node at all. Check first:

    ros2 topic info /scan --verbose        # look at each subscriber's QoS

Only run this if you find a RELIABLE subscriber sitting on a topic it can never
receive. Then point the consumer at the relayed topic instead:

    ros2 run microros_pi5_nav scan_qos_relay --ros-args -p output_topic:=scan_reliable

Relaying costs a copy and a few ms of latency, so prefer fixing the consumer's
QoS if it exposes a parameter for it.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class ScanQosRelay(Node):

    def __init__(self):
        super().__init__('scan_qos_relay')

        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_reliable')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        if self.resolve_topic_name(in_topic) == self.resolve_topic_name(out_topic):
            raise RuntimeError(
                f"input_topic and output_topic are both '{in_topic}' - that would "
                f"feed the relay its own output")

        self.pub = self.create_publisher(LaserScan, out_topic, RELIABLE_QOS)
        self.create_subscription(LaserScan, in_topic, self.on_scan, BEST_EFFORT_QOS)

        self.count = 0
        self.create_timer(5.0, self.warn_if_silent)

        self.get_logger().info(
            f"relaying '{in_topic}' (BEST_EFFORT) -> '{out_topic}' (RELIABLE)")

    def on_scan(self, msg: LaserScan):
        self.count += 1
        self.pub.publish(msg)

    def warn_if_silent(self):
        if self.count == 0:
            self.get_logger().warn(
                f"no scans received yet - is the micro-ROS agent up and the board "
                f"connected? Check: ros2 topic echo --once "
                f"--qos-reliability best_effort /scan")
        self.count = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanQosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
