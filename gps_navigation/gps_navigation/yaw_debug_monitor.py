#!/usr/bin/env python3
"""
Monitor yaw estimates from the GPS IMU, base IMU, and filtered odometry.

Published topics:
  /yaw_debug/yaws_deg   geometry_msgs/Vector3Stamped
    x = corrected GPS IMU robot-heading yaw in degrees
    y = base IMU yaw in degrees
    z = filtered odometry yaw in degrees

  /yaw_debug/delta_deg  geometry_msgs/Vector3Stamped
    x = corrected GPS IMU yaw - base IMU yaw in degrees
    y = corrected GPS IMU yaw - filtered odometry yaw in degrees
    z = base IMU yaw - filtered odometry yaw in degrees

  /yaw_debug/status     std_msgs/String
    Human-readable one-line status summary.
"""

import math

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf2_ros


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


def wrap_angle_deg(angle_deg: float) -> float:
    return wrap_delta_deg(angle_deg)


class YawDebugMonitor(Node):
    def __init__(self):
        super().__init__('yaw_debug_monitor')

        self.declare_parameter('gps_imu_raw_topic', '/gps/imu_compass')
        self.declare_parameter('gps_imu_topic', '/gps/imu_heading')
        self.declare_parameter('base_imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('report_period', 1.0)
        self.declare_parameter('log_to_console', True)

        self.gps_imu_raw_topic = self.get_parameter('gps_imu_raw_topic').value
        self.gps_imu_topic = self.get_parameter('gps_imu_topic').value
        self.base_imu_topic = self.get_parameter('base_imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.report_period = float(self.get_parameter('report_period').value)
        self.log_to_console = bool(self.get_parameter('log_to_console').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest = {
            'gps_raw': {'yaw_deg': None, 'frame_id': '', 'received_ns': None},
            'gps_heading': {'yaw_deg': None, 'frame_id': '', 'received_ns': None},
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
            self.gps_imu_raw_topic,
            self.gps_imu_raw_callback,
            10,
        )
        self.create_subscription(
            Imu,
            self.gps_imu_topic,
            self.gps_imu_heading_callback,
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
            f'gps_raw={self.gps_imu_raw_topic}, '
            f'gps_heading={self.gps_imu_topic}, '
            f'base_imu={self.base_imu_topic}, '
            f'odom={self.odom_topic}, '
            f'tf={self.map_frame}/{self.odom_frame}/{self.base_frame}/{self.utm_frame}'
        )

    def update_source(self, key: str, yaw_deg: float, frame_id: str):
        self.latest[key]['yaw_deg'] = yaw_deg
        self.latest[key]['frame_id'] = frame_id
        self.latest[key]['received_ns'] = self.get_clock().now().nanoseconds

    def gps_imu_raw_callback(self, msg: Imu):
        yaw_deg = quaternion_to_yaw_deg(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.update_source('gps_raw', yaw_deg, msg.header.frame_id)

    def gps_imu_heading_callback(self, msg: Imu):
        yaw_deg = quaternion_to_yaw_deg(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.update_source('gps_heading', yaw_deg, msg.header.frame_id)

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

    def lookup_tf_yaw_deg(self, target_frame: str, source_frame: str):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            rotation = transform.transform.rotation
            return quaternion_to_yaw_deg(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
        except Exception:
            return None

    def gps_yaw_in_base_frame(self, gps_yaw_deg, key='gps_heading'):
        if gps_yaw_deg is None:
            return None
        gps_frame = self.latest[key]['frame_id']
        if not gps_frame:
            return gps_yaw_deg

        # The static transform is base_frame -> gps_frame. If the GPS IMU
        # x-axis is mounted theta radians from robot forward, the robot yaw is
        # sensor yaw - theta.
        mount_yaw_deg = self.lookup_tf_yaw_deg(self.base_frame, gps_frame)
        if mount_yaw_deg is None:
            return gps_yaw_deg
        return wrap_angle_deg(gps_yaw_deg - mount_yaw_deg)

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
        gps_raw_yaw_deg = self.latest['gps_raw']['yaw_deg']
        gps_yaw_deg = self.latest['gps_heading']['yaw_deg']
        base_yaw_deg = self.latest['base_imu']['yaw_deg']
        odom_yaw_deg = self.latest['odom']['yaw_deg']
        gps_base_yaw_deg = self.gps_yaw_in_base_frame(gps_yaw_deg, 'gps_heading')

        gps_minus_base_deg = None
        gps_minus_odom_deg = None
        gps_base_minus_odom_deg = None
        base_minus_odom_deg = None

        if gps_yaw_deg is not None and base_yaw_deg is not None:
            gps_minus_base_deg = wrap_delta_deg(gps_yaw_deg - base_yaw_deg)
        if gps_yaw_deg is not None and odom_yaw_deg is not None:
            gps_minus_odom_deg = wrap_delta_deg(gps_yaw_deg - odom_yaw_deg)
        if gps_base_yaw_deg is not None and odom_yaw_deg is not None:
            gps_base_minus_odom_deg = wrap_delta_deg(gps_base_yaw_deg - odom_yaw_deg)
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

        gps_raw_age = self.get_age_sec('gps_raw')
        gps_age = self.get_age_sec('gps_heading')
        base_age = self.get_age_sec('base_imu')
        odom_age = self.get_age_sec('odom')
        map_base_yaw_deg = self.lookup_tf_yaw_deg(self.map_frame, self.base_frame)
        odom_base_yaw_deg = self.lookup_tf_yaw_deg(self.odom_frame, self.base_frame)
        utm_map_yaw_deg = self.lookup_tf_yaw_deg(self.utm_frame, self.map_frame)

        status = (
            'yaw_debug | '
            f'gps_raw={self.format_value(gps_raw_yaw_deg, " deg")} '
            f'(age={self.format_value(gps_raw_age, " s")}) | '
            f'gps_heading={self.format_value(gps_yaw_deg, " deg")} '
            f'(age={self.format_value(gps_age, " s")}) | '
            f'gps_base={self.format_value(gps_base_yaw_deg, " deg")} | '
            f'imu={self.format_value(base_yaw_deg, " deg")} '
            f'(age={self.format_value(base_age, " s")}) | '
            f'odom={self.format_value(odom_yaw_deg, " deg")} '
            f'(age={self.format_value(odom_age, " s")}) | '
            f'gps_heading-imu={self.format_value(gps_minus_base_deg, " deg")} | '
            f'gps_heading-odom={self.format_value(gps_minus_odom_deg, " deg")} | '
            f'gps_base-odom={self.format_value(gps_base_minus_odom_deg, " deg")} | '
            f'imu-odom={self.format_value(base_minus_odom_deg, " deg")} | '
            f'tf_map_base={self.format_value(map_base_yaw_deg, " deg")} | '
            f'tf_odom_base={self.format_value(odom_base_yaw_deg, " deg")} | '
            f'tf_utm_map={self.format_value(utm_map_yaw_deg, " deg")}'
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
