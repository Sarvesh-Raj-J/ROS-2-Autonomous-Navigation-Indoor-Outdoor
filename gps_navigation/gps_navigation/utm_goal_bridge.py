#!/usr/bin/env python3
"""
Bridge RViz goals from an earth-aligned frame such as UTM into Nav2's map frame.

Use case:
  - RViz Fixed Frame = utm
  - RViz 2D Goal Pose topic = /goal_pose_utm
  - This node transforms the clicked goal into map and sends NavigateToPose
"""

import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import tf2_ros


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class UTMGoalBridge(Node):
    def __init__(self):
        super().__init__('utm_goal_bridge')

        self.declare_parameter('input_topic', '/goal_pose_utm')
        self.declare_parameter('output_topic', '/goal_pose')
        self.declare_parameter('input_frame', 'utm')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('send_to_nav2', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.input_frame = self.get_parameter('input_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.send_to_nav2 = bool(self.get_parameter('send_to_nav2').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.goal_callback,
            10,
        )

        self.get_logger().info(
            'UTM goal bridge started: '
            f'{self.input_topic} ({self.input_frame}) -> {self.output_topic} ({self.map_frame})'
        )

    def transform_goal_to_map(self, goal_in):
        source_frame = goal_in.header.frame_id or self.input_frame
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0),
        )

        tf_t = transform.transform.translation
        tf_q = transform.transform.rotation
        tf_yaw = quaternion_to_yaw(tf_q.x, tf_q.y, tf_q.z, tf_q.w)

        src_x = goal_in.pose.position.x
        src_y = goal_in.pose.position.y
        dst_x = tf_t.x + math.cos(tf_yaw) * src_x - math.sin(tf_yaw) * src_y
        dst_y = tf_t.y + math.sin(tf_yaw) * src_x + math.cos(tf_yaw) * src_y

        src_q = goal_in.pose.orientation
        src_yaw = quaternion_to_yaw(src_q.x, src_q.y, src_q.z, src_q.w)
        dst_yaw = tf_yaw + src_yaw
        qx, qy, qz, qw = yaw_to_quaternion(dst_yaw)

        goal_out = PoseStamped()
        goal_out.header.stamp = self.get_clock().now().to_msg()
        goal_out.header.frame_id = self.map_frame
        goal_out.pose.position.x = dst_x
        goal_out.pose.position.y = dst_y
        goal_out.pose.position.z = 0.0
        goal_out.pose.orientation.x = qx
        goal_out.pose.orientation.y = qy
        goal_out.pose.orientation.z = qz
        goal_out.pose.orientation.w = qw
        return goal_out

    def send_nav2_goal(self, goal_pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server /navigate_to_pose is not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self.nav_client.send_goal_async(goal_msg)

        def _done_cb(done_future):
            try:
                goal_handle = done_future.result()
                if goal_handle is None or not goal_handle.accepted:
                    self.get_logger().warn('Nav2 rejected transformed UTM goal')
                else:
                    self.get_logger().info('Nav2 accepted transformed UTM goal')
            except Exception as exc:
                self.get_logger().error(f'Failed to send transformed goal: {exc}')

        future.add_done_callback(_done_cb)

    def goal_callback(self, msg: PoseStamped):
        try:
            goal_map = self.transform_goal_to_map(msg)
            self.goal_pub.publish(goal_map)
            self.get_logger().info(
                'UTM goal -> map goal: '
                f'[{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}] '
                f'{msg.header.frame_id or self.input_frame} '
                f'-> [{goal_map.pose.position.x:.2f}, {goal_map.pose.position.y:.2f}] '
                f'{self.map_frame}'
            )
            if self.send_to_nav2:
                self.send_nav2_goal(goal_map)
        except Exception as exc:
            self.get_logger().error(f'Failed to transform UTM goal: {exc}')


def main():
    rclpy.init()
    node = UTMGoalBridge()
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
