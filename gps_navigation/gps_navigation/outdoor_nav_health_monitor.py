#!/usr/bin/env python3
"""Publish live outdoor navigation sensor/control update rates."""

from dataclasses import dataclass
import math
from typing import Dict

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String


@dataclass
class TopicStats:
    count: int = 0
    last_count: int = 0
    rate_hz: float = 0.0
    last_msg_ns: int = 0


def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class OutdoorNavHealthMonitor(Node):
    def __init__(self):
        super().__init__('outdoor_nav_health_monitor')

        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('gps_imu_compass_topic', '/gps/imu_compass')
        self.declare_parameter('gps_imu_heading_topic', '/gps/imu_heading')
        self.declare_parameter('base_imu_topic', '/imu')
        self.declare_parameter('raw_odom_topic', '/odom')
        self.declare_parameter('gps_odom_topic', '/odometry/gps')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('nav_cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('controller_cmd_vel_topic', '/cmd_vel_nav_raw')
        self.declare_parameter('report_period_sec', 1.0)

        self.report_period_sec = max(
            0.5,
            float(self.get_parameter('report_period_sec').value),
        )
        self.stats: Dict[str, TopicStats] = {
            'gps_fix': TopicStats(),
            'gps_imu_compass': TopicStats(),
            'gps_imu_heading': TopicStats(),
            'base_imu': TopicStats(),
            'raw_odom': TopicStats(),
            'gps_odom': TopicStats(),
            'filtered_odom': TopicStats(),
            'nav_cmd_vel': TopicStats(),
            'cmd_vel': TopicStats(),
            'controller_cmd_vel': TopicStats(),
        }
        self.latest_imus: Dict[str, Imu] = {}
        self.latest_twists: Dict[str, Twist] = {}
        self.latest_odoms: Dict[str, Odometry] = {}
        self.last_report_ns = self.get_clock().now().nanoseconds

        self.status_pub = self.create_publisher(
            String,
            '/outdoor_nav_health/status',
            10,
        )
        self.create_subscription(
            NavSatFix,
            self.get_parameter('gps_fix_topic').value,
            lambda _msg: self.record('gps_fix'),
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('gps_imu_compass_topic').value,
            lambda msg: self.record_imu('gps_imu_compass', msg),
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('gps_imu_heading_topic').value,
            lambda msg: self.record_imu('gps_imu_heading', msg),
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('base_imu_topic').value,
            lambda msg: self.record_imu('base_imu', msg),
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('raw_odom_topic').value,
            lambda msg: self.record_odom('raw_odom', msg),
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('gps_odom_topic').value,
            lambda _msg: self.record('gps_odom'),
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('filtered_odom_topic').value,
            lambda msg: self.record_odom('filtered_odom', msg),
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('nav_cmd_vel_topic').value,
            lambda msg: self.record_twist('nav_cmd_vel', msg),
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            lambda msg: self.record_twist('cmd_vel', msg),
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter('controller_cmd_vel_topic').value,
            lambda msg: self.record_twist('controller_cmd_vel', msg),
            10,
        )
        self.create_timer(self.report_period_sec, self.publish_status)

        self.get_logger().info('Outdoor navigation health monitor started')

    def record(self, key: str):
        stat = self.stats[key]
        stat.count += 1
        stat.last_msg_ns = self.get_clock().now().nanoseconds

    def record_twist(self, key: str, msg: Twist):
        self.record(key)
        self.latest_twists[key] = msg

    def record_imu(self, key: str, msg: Imu):
        self.record(key)
        self.latest_imus[key] = msg

    def record_odom(self, key: str, msg: Odometry):
        self.record(key)
        self.latest_odoms[key] = msg

    def format_stat(self, key: str, now_ns: int) -> str:
        stat = self.stats[key]
        if stat.last_msg_ns:
            age_sec = (now_ns - stat.last_msg_ns) / 1e9
            age_text = f'{age_sec:.2f}s'
        else:
            age_text = 'n/a'
        return f'{key}={stat.rate_hz:.1f}Hz/{age_text}'

    def format_twist(self, key: str) -> str:
        msg = self.latest_twists.get(key)
        if msg is None:
            return f'{key}:vx=n/a,wz=n/a'
        return (
            f'{key}:vx={msg.linear.x:.3f},'
            f'vy={msg.linear.y:.3f},'
            f'wz={msg.angular.z:.3f}'
        )

    def format_imu_yaw(self, key: str) -> str:
        msg = self.latest_imus.get(key)
        if msg is None:
            return f'{key}:yaw=n/a,wz=n/a'
        q = msg.orientation
        return (
            f'{key}:yaw={quaternion_to_yaw_deg(q.x, q.y, q.z, q.w):.1f},'
            f'wz={msg.angular_velocity.z:.3f}'
        )

    def format_odom_twist(self, key: str) -> str:
        msg = self.latest_odoms.get(key)
        if msg is None:
            return f'{key}:vx=n/a,wz=n/a,yaw=n/a'
        twist = msg.twist.twist
        pose = msg.pose.pose
        yaw_deg = quaternion_to_yaw_deg(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return (
            f'{key}:vx={twist.linear.x:.3f},'
            f'vy={twist.linear.y:.3f},'
            f'wz={twist.angular.z:.3f},'
            f'yaw={yaw_deg:.1f}'
        )

    def publish_status(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed = max((now_ns - self.last_report_ns) / 1e9, 1e-6)
        self.last_report_ns = now_ns

        for stat in self.stats.values():
            delta = stat.count - stat.last_count
            stat.rate_hz = delta / elapsed
            stat.last_count = stat.count

        status = (
            'outdoor_nav_health | '
            + self.format_odom_twist('raw_odom')
            + ' | '
            + self.format_odom_twist('filtered_odom')
            + ' | '
            + self.format_imu_yaw('gps_imu_compass')
            + ' | '
            + self.format_imu_yaw('gps_imu_heading')
            + ' | '
            + self.format_imu_yaw('base_imu')
            + ' | '
            + self.format_twist('nav_cmd_vel')
            + ' | '
            + self.format_twist('cmd_vel')
            + ' | '
            + self.format_twist('controller_cmd_vel')
            + ' | rates '
            + ' | '.join(
                self.format_stat(key, now_ns)
                for key in (
                    'gps_fix',
                    'gps_imu_compass',
                    'gps_imu_heading',
                    'base_imu',
                    'raw_odom',
                    'gps_odom',
                    'filtered_odom',
                    'nav_cmd_vel',
                    'cmd_vel',
                    'controller_cmd_vel',
                )
            )
        )
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = OutdoorNavHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
