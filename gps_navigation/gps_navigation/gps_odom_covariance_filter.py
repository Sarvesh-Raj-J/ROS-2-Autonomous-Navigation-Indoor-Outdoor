#!/usr/bin/env python3
"""
Inflate navsat GPS odometry covariance before it enters robot_localization.

Outdoor GPS often reports over-confident covariance. If the EKF trusts that
position too much, the map pose jumps sideways and Nav2 drives away from an
otherwise correct OSM route. This node keeps the GPS correction available, but
prevents it from overpowering wheel odometry and IMU between fixes.
"""

from copy import deepcopy
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GPSOdomCovarianceFilter(Node):
    def __init__(self):
        super().__init__('gps_odom_covariance_filter')

        self.declare_parameter('input_topic', '/odometry/gps_raw')
        self.declare_parameter('output_topic', '/odometry/gps')
        self.declare_parameter('status_topic', '/gps_odom_filter/status')
        self.declare_parameter('min_horizontal_variance_m2', 9.0)
        self.declare_parameter('min_vertical_variance_m2', 100.0)
        self.declare_parameter('publish_period_sec', 2.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.min_horizontal_variance_m2 = max(
            0.0,
            float(self.get_parameter('min_horizontal_variance_m2').value),
        )
        self.min_vertical_variance_m2 = max(
            0.0,
            float(self.get_parameter('min_vertical_variance_m2').value),
        )
        publish_period_sec = max(
            0.5,
            float(self.get_parameter('publish_period_sec').value),
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Odometry, self.input_topic, self.odom_callback, 10)
        self.create_timer(publish_period_sec, self.publish_status)

        self.received_count = 0
        self.last_rate_count = 0
        self.last_rate_time = self.get_clock().now()
        self.last_msg_time = None
        self.last_in_cov = (float('nan'), float('nan'), float('nan'))
        self.last_out_cov = (float('nan'), float('nan'), float('nan'))
        self.last_rate_hz = 0.0

        self.get_logger().info(
            'GPS odom covariance filter started: '
            f'{self.input_topic} -> {self.output_topic}, '
            f'min_horizontal_sigma={math.sqrt(self.min_horizontal_variance_m2):.2f}m'
        )

    def odom_callback(self, msg: Odometry):
        filtered = deepcopy(msg)
        covariance = filtered.pose.covariance

        in_x = covariance[0]
        in_y = covariance[7]
        in_z = covariance[14]
        covariance[0] = max(covariance[0], self.min_horizontal_variance_m2)
        covariance[7] = max(covariance[7], self.min_horizontal_variance_m2)
        covariance[14] = max(covariance[14], self.min_vertical_variance_m2)

        self.last_in_cov = (in_x, in_y, in_z)
        self.last_out_cov = (covariance[0], covariance[7], covariance[14])
        self.last_msg_time = self.get_clock().now()
        self.received_count += 1
        self.odom_pub.publish(filtered)

    def publish_status(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_rate_time).nanoseconds * 1e-9
        if elapsed > 0.0:
            count_delta = self.received_count - self.last_rate_count
            self.last_rate_hz = count_delta / elapsed
            self.last_rate_count = self.received_count
            self.last_rate_time = now

        age_sec = float('inf')
        if self.last_msg_time is not None:
            age_sec = (now - self.last_msg_time).nanoseconds * 1e-9

        msg = String()
        msg.data = (
            'gps_odom_filter | '
            f'rate_hz={self.last_rate_hz:.2f} | '
            f'age_sec={age_sec:.2f} | '
            f'in_cov_xy=({self.last_in_cov[0]:.2f}, {self.last_in_cov[1]:.2f}) | '
            f'out_cov_xy=({self.last_out_cov[0]:.2f}, {self.last_out_cov[1]:.2f})'
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = GPSOdomCovarianceFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
