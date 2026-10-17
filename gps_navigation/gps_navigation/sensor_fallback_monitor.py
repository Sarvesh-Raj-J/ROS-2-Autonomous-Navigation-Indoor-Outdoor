#!/usr/bin/env python3
"""
Report which obstacle sensor is alive: depth camera first, LiDAR as fallback.

Switches the reported primary between depth camera and LiDAR based on sensor
health (message age).

Priority: Depth Camera (indoor) → LiDAR (outdoor/sunlight)
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


class SensorFallbackMonitor(Node):
    def __init__(self):
        super().__init__('sensor_fallback_monitor')
        self.declare_parameter('depth_topic', '/depth_cam/depth/points')
        self.declare_parameter('scan_topic', '/scan')
        depth_topic = self.get_parameter('depth_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        # Sensor health tracking
        self.depth_timeout = 3.0  # seconds
        self.lidar_timeout = 3.0  # seconds

        self.last_depth_msg_time = 0
        self.last_lidar_msg_time = 0

        self.current_primary = 'depth_camera'  # Start with depth camera
        self.previous_primary = 'depth_camera'

        # Subscribe to both sensors
        self.create_subscription(
            PointCloud2,
            depth_topic,
            self.depth_callback,
            1
        )

        self.create_subscription(
            LaserScan,
            scan_topic,
            self.lidar_callback,
            1
        )

        # Health status publisher
        self.status_pub = self.create_publisher(
            String, '/sensor_fallback/status', 1
        )

        # Health check timer (1 Hz)
        self.create_timer(1.0, self.check_sensor_health)

        self.get_logger().info(
            '🔄 Sensor Fallback Monitor started\n'
            f'  Primary: Depth Camera ({depth_topic})\n'
            f'  Fallback: LiDAR ({scan_topic})\n'
            f'  Timeout: {self.depth_timeout}s'
        )

    def depth_callback(self, msg: PointCloud2):
        """Monitor depth camera heartbeat."""
        self.last_depth_msg_time = time.time()

    def lidar_callback(self, msg: LaserScan):
        """Monitor LiDAR heartbeat."""
        self.last_lidar_msg_time = time.time()

    def check_sensor_health(self):
        """Check sensor health and switch if needed."""
        current_time = time.time()

        # Calculate time since last message
        depth_age = current_time - self.last_depth_msg_time
        lidar_age = current_time - self.last_lidar_msg_time

        # Determine sensor status
        depth_alive = depth_age < self.depth_timeout
        lidar_alive = lidar_age < self.lidar_timeout

        # Decide primary sensor
        if depth_alive:
            self.current_primary = 'depth_camera'
        elif lidar_alive:
            self.current_primary = 'lidar'
        else:
            self.current_primary = 'none'

        # Log when switching
        if self.current_primary != self.previous_primary:
            self.get_logger().warn(
                f'⚠️  SENSOR SWITCH: {self.previous_primary} → {self.current_primary}'
            )
            self.previous_primary = self.current_primary

        # Publish status
        status_msg = String()
        status_msg.data = (
            f'Primary: {self.current_primary} | '
            f'Depth: {depth_age:.1f}s | '
            f'LiDAR: {lidar_age:.1f}s'
        )
        self.status_pub.publish(status_msg)

        # DEBUG - Only log every 5 seconds
        if int(current_time) % 5 == 0:
            self.get_logger().debug(
                f'Depth: {depth_age:.1f}s ({"🟢" if depth_alive else "🔴"}) | '
                f'LiDAR: {lidar_age:.1f}s ({"🟢" if lidar_alive else "🔴"}) | '
                f'Using: {self.current_primary}'
            )


def main():
    rclpy.init()
    node = SensorFallbackMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
