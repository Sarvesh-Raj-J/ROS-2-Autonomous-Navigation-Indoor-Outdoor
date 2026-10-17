#!/usr/bin/env python3
"""Filter forward velocity when the 2D LiDAR reports a close front obstacle."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class CmdVelLidarSafetyFilter(Node):
    def __init__(self):
        super().__init__('cmd_vel_lidar_safety_filter')

        self.declare_parameter('input_topic', '/cmd_vel_nav_raw')
        self.declare_parameter('output_topic', '/cmd_vel_nav')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('backup_scan_topic', '/scan_raw')
        self.declare_parameter('front_center_degrees', [0.0, 180.0])
        self.declare_parameter('front_angle_deg', 80.0)
        self.declare_parameter('slow_distance_m', 0.90)
        self.declare_parameter('stop_distance_m', 0.55)
        self.declare_parameter('scan_timeout_sec', 0.7)
        self.declare_parameter('max_forward_speed_when_slow', 0.05)
        self.declare_parameter('allow_rotation_when_blocked', False)
        self.declare_parameter('avoid_turn_speed', 0.25)
        self.declare_parameter('min_turn_cmd_when_blocked', 0.12)

        front_centers = self.get_parameter('front_center_degrees').value
        self.front_centers_rad = [math.radians(float(v)) for v in front_centers]
        self.front_angle_rad = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )
        self.slow_distance_m = float(self.get_parameter('slow_distance_m').value)
        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.scan_timeout_sec = float(self.get_parameter('scan_timeout_sec').value)
        self.max_forward_speed_when_slow = float(
            self.get_parameter('max_forward_speed_when_slow').value
        )
        self.allow_rotation_when_blocked = bool(
            self.get_parameter('allow_rotation_when_blocked').value
        )
        self.avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self.min_turn_cmd_when_blocked = float(
            self.get_parameter('min_turn_cmd_when_blocked').value
        )

        self.scan_stats = {}
        self.last_scan_time = None
        self.front_min_range = float('inf')
        self.left_min_range = float('inf')
        self.right_min_range = float('inf')
        self.all_min_range = float('inf')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        backup_scan_topic = self.get_parameter('backup_scan_topic').value

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.status_pub = self.create_publisher(String, '/cmd_vel_safety/status', 10)
        scan_topics = [scan_topic]
        if backup_scan_topic and backup_scan_topic not in scan_topics:
            scan_topics.append(backup_scan_topic)
        for topic in scan_topics:
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, source=topic: self.scan_callback(msg, source),
                qos_profile_sensor_data,
            )
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            'cmd_vel LiDAR safety filter active: '
            f'{input_topic} -> {output_topic}, scans={scan_topics}, '
            f'front_centers_deg={front_centers}, '
            f'stop={self.stop_distance_m:.2f}m, slow={self.slow_distance_m:.2f}m'
        )

    @staticmethod
    def finite_range(value: float, range_min: float, range_max: float) -> bool:
        return math.isfinite(value) and range_min <= value <= range_max

    @staticmethod
    def angular_distance(a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def scan_callback(self, msg: LaserScan, source: str):
        front_half_angle = 0.5 * self.front_angle_rad
        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')
        all_min = float('inf')

        angle = msg.angle_min
        for value in msg.ranges:
            if self.finite_range(value, msg.range_min, msg.range_max):
                all_min = min(all_min, value)
                wrapped = math.atan2(math.sin(angle), math.cos(angle))
                for center in self.front_centers_rad:
                    if self.angular_distance(wrapped, center) <= front_half_angle:
                        front_min = min(front_min, value)
                        side = math.atan2(
                            math.sin(wrapped - center),
                            math.cos(wrapped - center),
                        )
                        if side >= 0.0:
                            left_min = min(left_min, value)
                        else:
                            right_min = min(right_min, value)
                        break
            angle += msg.angle_increment

        now = time.monotonic()
        self.scan_stats[source] = {
            'time': now,
            'front': front_min,
            'left': left_min,
            'right': right_min,
            'all': all_min,
        }
        self.update_aggregate_scan(now)

    def update_aggregate_scan(self, now=None) -> bool:
        now = time.monotonic() if now is None else now
        fresh = [
            stats for stats in self.scan_stats.values()
            if now - stats['time'] <= self.scan_timeout_sec
        ]
        if not fresh:
            return False

        self.front_min_range = min(stats['front'] for stats in fresh)
        self.left_min_range = min(stats['left'] for stats in fresh)
        self.right_min_range = min(stats['right'] for stats in fresh)
        self.all_min_range = min(stats['all'] for stats in fresh)
        self.last_scan_time = max(stats['time'] for stats in fresh)
        return True

    def choose_avoid_turn(self) -> float:
        if self.left_min_range < self.right_min_range:
            return -abs(self.avoid_turn_speed)
        if self.right_min_range < self.left_min_range:
            return abs(self.avoid_turn_speed)
        return abs(self.avoid_turn_speed)

    def cmd_callback(self, msg: Twist):
        safe = Twist()
        safe.linear.x = msg.linear.x
        safe.linear.y = msg.linear.y
        safe.linear.z = msg.linear.z
        safe.angular.x = msg.angular.x
        safe.angular.y = msg.angular.y
        safe.angular.z = msg.angular.z

        if msg.linear.x > 0.0:
            if not self.update_aggregate_scan():
                safe.linear.x = 0.0
                self.get_logger().warn(
                    'Blocking forward cmd_vel because LiDAR scan is stale',
                    throttle_duration_sec=2.0,
                )
            elif self.front_min_range <= self.stop_distance_m:
                safe.linear.x = 0.0
                if not self.allow_rotation_when_blocked:
                    safe.angular.z = 0.0
                elif abs(safe.angular.z) < self.min_turn_cmd_when_blocked:
                    safe.angular.z = self.choose_avoid_turn()
                self.get_logger().warn(
                    'Blocking forward cmd_vel: front obstacle at '
                    f'{self.front_min_range:.2f}m',
                    throttle_duration_sec=1.0,
                )
            elif self.front_min_range <= self.slow_distance_m:
                safe.linear.x = min(safe.linear.x, self.max_forward_speed_when_slow)

        self.cmd_pub.publish(safe)

    def publish_status(self):
        msg = String()
        age = (
            time.monotonic() - self.last_scan_time
            if self.last_scan_time is not None
            else float('nan')
        )
        msg.data = (
            'cmd_vel_safety | '
            f'scan_age={age:.2f}s | '
            f'front_min={self.front_min_range:.2f}m | '
            f'left_min={self.left_min_range:.2f}m | '
            f'right_min={self.right_min_range:.2f}m | '
            f'all_min={self.all_min_range:.2f}m'
        )
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelLidarSafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
