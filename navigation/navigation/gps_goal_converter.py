#!/usr/bin/env python3
"""
GPS Goal Converter - Converts GPS (lat/lon) to map frame goal pose.
Subscribes to GPS goals and publishes in map frame for Nav2.
"""
import threading

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import math
import tf2_ros


class GPSGoalConverter(Node):
    def __init__(self):
        super().__init__('gps_goal_converter')

        self.latest_gps_fix = None
        self.map_frame = 'map'
        self.base_frame = 'base_footprint'
        self.utm_frame = 'utm'
        self._gps_fix_event = threading.Event()
        self._callback_group = ReentrantCallbackGroup()

        # TF buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Mirror goal poses on /goal_pose for RViz/debugging tools.
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 1
        )
        self.nav2_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self._callback_group
        )

        # Subscribe to GPS fix to cache latest reading
        self.create_subscription(
            NavSatFix, '/gps/fix',
            self.gps_fix_callback, 1,
            callback_group=self._callback_group
        )

        # Service to accept GPS coordinates directly
        from interfaces.srv import SetPose2D
        self.create_service(
            SetPose2D, '~/set_gps_goal',
            self.set_gps_goal_callback,
            callback_group=self._callback_group
        )

        self.get_logger().info(
            '🛰️  GPS Goal Converter started. '
            'Use /gps_goal_converter/set_gps_goal service.'
        )

    def gps_fix_callback(self, msg: NavSatFix):
        """Cache latest GPS fix"""
        self.latest_gps_fix = msg
        self._gps_fix_event.set()
        self.get_logger().debug(
            f'GPS fix received: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}'
        )

    def wait_for_gps_fix(self, timeout_sec: float = 2.0) -> bool:
        """Wait for the first GPS fix if one has not arrived yet."""
        if self.latest_gps_fix is not None:
            return True

        self.get_logger().warn('Waiting for GPS fix...')
        return self._gps_fix_event.wait(timeout=timeout_sec)

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Return planar yaw from a quaternion."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def rotate_enu_delta_into_map(self, delta_east: float, delta_north: float):
        """
        Rotate an east/north delta into the current map frame.

        navsat_transform may align map with the robot's initial odometry
        heading instead of true east/north. If we skip this rotation, GPS goals
        can be sent to the wrong bearing even when the position delta is right.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            q = transform.transform.rotation
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            delta_map_x = math.cos(yaw) * delta_east - math.sin(yaw) * delta_north
            delta_map_y = math.sin(yaw) * delta_east + math.cos(yaw) * delta_north
            return delta_map_x, delta_map_y, True
        except Exception as exc:
            self.get_logger().warn(
                f'UTM→map transform unavailable, assuming map is ENU-aligned: {exc}'
            )
            return delta_east, delta_north, False

    def send_nav2_goal(self, goal_pose: PoseStamped):
        """Forward a map-frame goal to the Nav2 NavigateToPose action server."""
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            return False, 'Nav2 action server /navigate_to_pose is not available'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        goal_response = {
            'accepted': False,
            'message': 'Timed out waiting for Nav2 goal response',
        }
        goal_event = threading.Event()

        def _goal_response_cb(future):
            try:
                goal_handle = future.result()
                if goal_handle is None:
                    goal_response['message'] = 'Nav2 returned no goal handle'
                elif not goal_handle.accepted:
                    goal_response['message'] = 'Nav2 rejected the GPS goal'
                else:
                    goal_response['accepted'] = True
                    goal_response['message'] = 'Nav2 accepted the GPS goal'
            except Exception as exc:
                goal_response['message'] = f'Failed to send Nav2 goal: {exc}'
            finally:
                goal_event.set()

        future = self.nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(_goal_response_cb)

        if not goal_event.wait(timeout=3.0):
            return False, goal_response['message']

        return goal_response['accepted'], goal_response['message']

    def set_gps_goal_callback(self, request, response):
        """
        Service callback: Accept GPS goal as SetPose2D
        data.x = latitude, data.y = longitude
        data.roll = altitude (optional), data.yaw = heading
        """
        try:
            lat = request.data.x
            lon = request.data.y
            yaw = request.data.yaw

            self.get_logger().info(
                f'📍 GPS Goal: lat={lat:.6f}, lon={lon:.6f}, '
                f'heading={math.degrees(yaw):.1f}°'
            )

            # Wait briefly for GPS data if not available yet
            if not self.wait_for_gps_fix():
                response.success = False
                response.message = 'No GPS fix received on /gps/fix yet'
                return response

            # Convert GPS to map frame
            goal_pose = self.gps_to_map_pose(lat, lon, yaw)

            if goal_pose:
                # Mirror the goal on /goal_pose for RViz/debugging.
                self.goal_pub.publish(goal_pose)
                goal_sent, nav2_message = self.send_nav2_goal(goal_pose)
                response.success = goal_sent
                response.message = (
                    f'{nav2_message}; map goal '
                    f'[{goal_pose.pose.position.x:.2f}, '
                    f'{goal_pose.pose.position.y:.2f}]'
                )
            else:
                response.success = False
                response.message = 'Failed to convert GPS to map frame'

        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(f'GPS conversion failed: {e}', exc_info=True)

        return response

    def gps_to_map_pose(self, lat: float, lon: float, yaw: float = 0.0):
        """
        Convert GPS (lat/lon) to map frame pose.
        Uses cached GPS fix as reference, then converts input lat/lon.
        """
        try:
            if self.latest_gps_fix is None:
                self.get_logger().error('No GPS fix received yet')
                return None

            # Get reference GPS fix (current robot position)
            ref_gps = self.latest_gps_fix

            # Compute difference in lat/lon from reference to target
            delta_lat = lat - ref_gps.latitude
            delta_lon = lon - ref_gps.longitude

            # Convert to meters (approximate at this latitude)
            M_per_deg_lat = 111320.0
            lat_rad = math.radians(ref_gps.latitude)
            M_per_deg_lon = 111320.0 * math.cos(lat_rad)

            delta_east = delta_lon * M_per_deg_lon
            delta_north = delta_lat * M_per_deg_lat

            # Get robot's current position in map frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y
            except Exception:
                self.get_logger().warn('TF lookup failed, using origin')
                robot_x = 0.0
                robot_y = 0.0

            delta_map_x, delta_map_y, used_utm_rotation = self.rotate_enu_delta_into_map(
                delta_east,
                delta_north,
            )

            # Goal position = robot position + delta offset
            goal = PoseStamped()
            goal.header.frame_id = self.map_frame
            goal.header.stamp = self.get_clock().now().to_msg()

            goal.pose.position.x = robot_x + delta_map_x
            goal.pose.position.y = robot_y + delta_map_y
            goal.pose.position.z = 0.0

            # Orientation - use provided yaw
            from tf_transformations import quaternion_from_euler
            q = quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

            self.get_logger().info(
                f'GPS→Map: lat={lat:.6f}, lon={lon:.6f} → '
                f'[{goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}] '
                f'(ENU delta: {delta_east:.2f}, {delta_north:.2f}; '
                f'map delta: {delta_map_x:.2f}, {delta_map_y:.2f}; '
                f'utm rotation: {"yes" if used_utm_rotation else "no"})'
            )

            return goal

        except Exception as e:
            self.get_logger().error(f'Conversion failed: {e}')
            return None


def main():
    rclpy.init()
    node = GPSGoalConverter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
