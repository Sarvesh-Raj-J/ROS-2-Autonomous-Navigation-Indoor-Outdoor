#!/usr/bin/env python3
"""
Monitor yaw estimates from the GPS IMU, base IMU, and filtered odometry.

Published topics:
  /yaw_debug/yaws_deg   geometry_msgs/Vector3Stamped
    x = GPS IMU yaw in degrees
    y = base IMU yaw in degrees
    z = filtered odometry yaw in degrees

  /yaw_debug/delta_deg  geometry_msgs/Vector3Stamped
    x = GPS IMU yaw - base IMU yaw in degrees
    y = GPS IMU yaw - filtered odometry yaw in degrees
    z = base IMU yaw - filtered odometry yaw in degrees

  /yaw_debug/status     std_msgs/String
    Human-readable one-line status summary.
"""

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    """Convert a quaternion to planar yaw in degrees."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def wrap_delta_deg(delta_deg: float) -> float:
    """Wrap a yaw difference into [-180, 180] degrees."""
    return math.degrees(
        math.atan2(
            math.sin(math.radians(delta_deg)),
            math.cos(math.radians(delta_deg)),
        )
    )


class YawDebugMonitor(Node):
    def __init__(self):
        super().__init__('yaw_debug_monitor')

        self.declare_parameter('gps_imu_topic', '/gps/imu')
        self.declare_parameter('base_imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('report_period', 1.0)
        self.declare_parameter('log_to_console', True)

        self.gps_imu_topic = self.get_parameter('gps_imu_topic').value
        self.base_imu_topic = self.get_parameter('base_imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.report_period = float(self.get_parameter('report_period').value)
        self.log_to_console = bool(self.get_parameter('log_to_console').value)

        self.latest = {
            'gps_imu': {'yaw_deg': None, 'frame_id': '', 'received_ns': None},
            'base_imu': {'yaw_deg': None, 'frame_id': '', 'received_ns': None},
            'odom': {'yaw_deg': None, 'frame_id': '', 'received_ns': None},
        }

        self.yaws_pub = self.create_publisher(
            Vector3Stamped, '/yaw_debug/yaws_deg', 10
        )
        self.delta_pub = self.create_publisher(
            Vector3Stamped, '/yaw_debug/delta_deg', 10
        )
        self.status_pub = self.create_publisher(
            String, '/yaw_debug/status', 10
        )

        self.create_subscription(
            Imu,
            self.gps_imu_topic,
            self.gps_imu_callback,
            10,
        )
        self.create_subscription(
            Imu,
            self.base_imu_topic,
            self.base_imu_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.create_timer(self.report_period, self.report_status)

        self.get_logger().info(
            'Yaw debug monitor started: '
            f'gps_imu={self.gps_imu_topic}, '
            f'base_imu={self.base_imu_topic}, '
            f'odom={self.odom_topic}'
        )

    def update_source(self, key: str, yaw_deg: float, frame_id: str):
        self.latest[key]['yaw_deg'] = yaw_deg
        self.latest[key]['frame_id'] = frame_id
        self.latest[key]['received_ns'] = self.get_clock().now().nanoseconds

    def gps_imu_callback(self, msg: Imu):
        yaw_deg = quaternion_to_yaw_deg(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.update_source('gps_imu', yaw_deg, msg.header.frame_id)

    def base_imu_callback(self, msg: Imu):
        yaw_deg = quaternion_to_yaw_deg(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.update_source('base_imu', yaw_deg, msg.header.frame_id)

    def odom_callback(self, msg: Odometry):
        yaw_deg = quaternion_to_yaw_deg(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.update_source('odom', yaw_deg, msg.header.frame_id)

    def get_age_sec(self, key: str):
        received_ns = self.latest[key]['received_ns']
        if received_ns is None:
            return None
        return (self.get_clock().now().nanoseconds - received_ns) / 1e9

    @staticmethod
    def format_value(value, suffix=''):
        if value is None:
            return 'n/a'
        return f'{value:.1f}{suffix}'

    def publish_numeric_topics(
        self,
        gps_yaw_deg,
        base_yaw_deg,
        odom_yaw_deg,
        gps_minus_base_deg,
        gps_minus_odom_deg,
        base_minus_odom_deg,
    ):
        stamp = self.get_clock().now().to_msg()

        yaws_msg = Vector3Stamped()
        yaws_msg.header.stamp = stamp
        yaws_msg.header.frame_id = 'yaw_debug'
        yaws_msg.vector.x = float('nan') if gps_yaw_deg is None else gps_yaw_deg
        yaws_msg.vector.y = float('nan') if base_yaw_deg is None else base_yaw_deg
        yaws_msg.vector.z = float('nan') if odom_yaw_deg is None else odom_yaw_deg
        self.yaws_pub.publish(yaws_msg)

        delta_msg = Vector3Stamped()
        delta_msg.header.stamp = stamp
        delta_msg.header.frame_id = 'yaw_debug'
        delta_msg.vector.x = (
            float('nan') if gps_minus_base_deg is None else gps_minus_base_deg
        )
        delta_msg.vector.y = (
            float('nan') if gps_minus_odom_deg is None else gps_minus_odom_deg
        )
        delta_msg.vector.z = (
            float('nan') if base_minus_odom_deg is None else base_minus_odom_deg
        )
        self.delta_pub.publish(delta_msg)

    def report_status(self):
        gps_yaw_deg = self.latest['gps_imu']['yaw_deg']
        base_yaw_deg = self.latest['base_imu']['yaw_deg']
        odom_yaw_deg = self.latest['odom']['yaw_deg']

        gps_minus_base_deg = None
        gps_minus_odom_deg = None
        base_minus_odom_deg = None

        if gps_yaw_deg is not None and base_yaw_deg is not None:
            gps_minus_base_deg = wrap_delta_deg(gps_yaw_deg - base_yaw_deg)
        if gps_yaw_deg is not None and odom_yaw_deg is not None:
            gps_minus_odom_deg = wrap_delta_deg(gps_yaw_deg - odom_yaw_deg)
        if base_yaw_deg is not None and odom_yaw_deg is not None:
            base_minus_odom_deg = wrap_delta_deg(base_yaw_deg - odom_yaw_deg)

        self.publish_numeric_topics(
            gps_yaw_deg,
            base_yaw_deg,
            odom_yaw_deg,
            gps_minus_base_deg,
            gps_minus_odom_deg,
            base_minus_odom_deg,
        )

        gps_age = self.get_age_sec('gps_imu')
        base_age = self.get_age_sec('base_imu')
        odom_age = self.get_age_sec('odom')

        status = (
            'yaw_debug | '
            f'gps={self.format_value(gps_yaw_deg, " deg")} '
            f'(age={self.format_value(gps_age, " s")}) | '
            f'imu={self.format_value(base_yaw_deg, " deg")} '
            f'(age={self.format_value(base_age, " s")}) | '
            f'odom={self.format_value(odom_yaw_deg, " deg")} '
            f'(age={self.format_value(odom_age, " s")}) | '
            f'gps-imu={self.format_value(gps_minus_base_deg, " deg")} | '
            f'gps-odom={self.format_value(gps_minus_odom_deg, " deg")} | '
            f'imu-odom={self.format_value(base_minus_odom_deg, " deg")}'
        )

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        if self.log_to_console:
            self.get_logger().info(status)


def main():
    rclpy.init()
    node = YawDebugMonitor()
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
