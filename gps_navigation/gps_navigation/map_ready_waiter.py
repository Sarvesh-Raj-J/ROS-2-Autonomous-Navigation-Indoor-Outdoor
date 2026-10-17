#!/usr/bin/env python3
"""Wait for the first usable OccupancyGrid before starting dependent nodes."""

import sys
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapReadyWaiter(Node):
    def __init__(self):
        super().__init__('map_ready_waiter')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('timeout_sec', 0.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.ready = False

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )
        self.get_logger().info(f'Waiting for first usable map on {self.map_topic}')

    def map_callback(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn('Ignoring empty OccupancyGrid while waiting for map')
            return
        if len(msg.data) != msg.info.width * msg.info.height:
            self.get_logger().warn(
                'Ignoring malformed OccupancyGrid while waiting for map: '
                f'{len(msg.data)} cells for {msg.info.width}x{msg.info.height}'
            )
            return

        self.ready = True
        self.get_logger().info(
            'Map is ready: '
            f'{msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m/cell, '
            f'frame={msg.header.frame_id or "map"}'
        )


def main():
    rclpy.init()
    node = MapReadyWaiter()
    start_time = time.monotonic()
    exit_code = 0

    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.25)
            if node.timeout_sec > 0.0 and time.monotonic() - start_time > node.timeout_sec:
                node.get_logger().error(
                    f'Timed out waiting {node.timeout_sec:.1f}s for {node.map_topic}'
                )
                exit_code = 1
                break
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
