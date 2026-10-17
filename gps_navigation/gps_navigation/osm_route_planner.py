#!/usr/bin/env python3
"""
Plan long-range GPS routes on an OSM road/path graph and execute them with Nav2.

This gives the robot a "Google Maps"-style first layer:
  - OSM graph chooses a reasonable road/path route globally
  - Nav2 waypoint follower executes that route locally
  - Local lidar costmaps still handle dynamic obstacle avoidance

Goals:
  - RViz 2D Goal Pose on goal_topic: snapped onto the OSM graph and routed.
  - geographic_msgs/GeoPoseStamped on gps_goal_topic: latitude/longitude goal.
    With an OSM graph it is routed on the roads; without one it is sent to Nav2
    as a direct goal. Orientation is ENU yaw (0 = east, counter-clockwise).
"""

import heapq
import json
import math
import os
import threading

from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Point, PoseStamped
from gps_navigation.osm_loader import load_osm_extract
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def latlon_delta_m(ref_lat: float, ref_lon: float, lat: float, lon: float):
    """Approximate east/north delta in meters around a local reference."""
    meters_per_deg_lat = 111320.0
    lat_rad = math.radians(ref_lat)
    meters_per_deg_lon = 111320.0 * math.cos(lat_rad)
    delta_north = (lat - ref_lat) * meters_per_deg_lat
    delta_east = (lon - ref_lon) * meters_per_deg_lon
    return delta_east, delta_north


def distance_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    delta_east, delta_north = latlon_delta_m(lat_a, lon_a, lat_b, lon_b)
    return math.hypot(delta_east, delta_north)


def is_valid_navsat_fix(msg: NavSatFix) -> bool:
    if msg.status.status < 0:
        return False
    return math.isfinite(msg.latitude) and math.isfinite(msg.longitude)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class OSMRoutePlanner(Node):
    def __init__(self):
        super().__init__('osm_route_planner')

        self.declare_parameter('osm_file', '')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('georef_json', '')
        self.declare_parameter('waypoint_spacing_m', 4.0)
        self.declare_parameter('send_to_nav2', True)
        self.declare_parameter('route_topic', '/osm_route_plan')
        self.declare_parameter('route_marker_array_topic', '/osm_route_markers')
        self.declare_parameter('graph_topic', '/osm_graph')
        self.declare_parameter('graph_marker_array_topic', '/osm_graph_markers')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('gps_goal_topic', '/gps_goal')
        self.declare_parameter('accept_rviz_goals', True)
        self.declare_parameter('append_exact_goal', True)
        self.declare_parameter('prepend_snapped_start', True)
        self.declare_parameter('max_nav2_waypoints', 40)
        self.declare_parameter('nav2_simplify_tolerance_m', 1.0)
        self.declare_parameter('snap_start_to_graph', True)
        self.declare_parameter('snap_goal_to_graph', True)
        self.declare_parameter('max_snap_distance_m', 20.0)
        self.declare_parameter('snap_heading_weight_m', 8.0)
        self.declare_parameter('prune_start_behind_robot', True)
        self.declare_parameter('start_prune_distance_m', 3.0)
        self.declare_parameter('short_goal_direct_distance_m', 0.0)
        self.declare_parameter('respect_rviz_goal_orientation', True)
        self.declare_parameter('nav2_action_mode', 'navigate_through_poses')
        self.declare_parameter('follow_waypoints_wait_sec', 10.0)
        self.declare_parameter('graph_publish_period_sec', 2.0)
        self.declare_parameter('graph_display_radius_m', 150.0)
        self.declare_parameter(
            'allowed_highway_types',
            [
                'motorway', 'trunk', 'primary', 'secondary', 'tertiary',
                'unclassified', 'residential', 'service', 'living_street',
                'road', 'track', 'path', 'footway', 'pedestrian', 'cycleway',
            ],
        )

        self.osm_file = self.get_parameter('osm_file').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.map_topic = self.get_parameter('map_topic').value
        self.georef_json = self.get_parameter('georef_json').value
        self.waypoint_spacing_m = float(self.get_parameter('waypoint_spacing_m').value)
        self.send_to_nav2 = bool(self.get_parameter('send_to_nav2').value)
        self.route_topic = self.get_parameter('route_topic').value
        self.route_marker_array_topic = self.get_parameter('route_marker_array_topic').value
        self.graph_topic = self.get_parameter('graph_topic').value
        self.graph_marker_array_topic = self.get_parameter(
            'graph_marker_array_topic'
        ).value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.gps_goal_topic = self.get_parameter('gps_goal_topic').value
        self.accept_rviz_goals = bool(self.get_parameter('accept_rviz_goals').value)
        self.append_exact_goal = bool(self.get_parameter('append_exact_goal').value)
        self.prepend_snapped_start = bool(self.get_parameter('prepend_snapped_start').value)
        self.max_nav2_waypoints = max(
            2,
            int(self.get_parameter('max_nav2_waypoints').value),
        )
        self.nav2_simplify_tolerance_m = max(
            0.0,
            float(self.get_parameter('nav2_simplify_tolerance_m').value),
        )
        self.snap_start_to_graph = bool(self.get_parameter('snap_start_to_graph').value)
        self.snap_goal_to_graph = bool(self.get_parameter('snap_goal_to_graph').value)
        self.max_snap_distance_m = float(self.get_parameter('max_snap_distance_m').value)
        self.snap_heading_weight_m = max(
            0.0,
            float(self.get_parameter('snap_heading_weight_m').value),
        )
        self.prune_start_behind_robot = bool(
            self.get_parameter('prune_start_behind_robot').value
        )
        self.start_prune_distance_m = float(
            self.get_parameter('start_prune_distance_m').value
        )
        self.short_goal_direct_distance_m = float(
            self.get_parameter('short_goal_direct_distance_m').value
        )
        self.respect_rviz_goal_orientation = bool(
            self.get_parameter('respect_rviz_goal_orientation').value
        )
        self.nav2_action_mode = str(
            self.get_parameter('nav2_action_mode').value
        ).strip().lower()
        self.follow_waypoints_wait_sec = float(
            self.get_parameter('follow_waypoints_wait_sec').value
        )
        self.graph_publish_period_sec = max(
            0.5,
            float(self.get_parameter('graph_publish_period_sec').value),
        )
        self.graph_display_radius_m = float(
            self.get_parameter('graph_display_radius_m').value
        )
        self.allowed_highway_types = set(
            self.get_parameter('allowed_highway_types').value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._gps_fix_event = threading.Event()
        self.latest_gps_fix = None
        self.static_map_georef = None
        self.static_map_origin = None
        self.static_map_logged = False
        self.static_georef_logged = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints',
            callback_group=self._callback_group,
        )
        self.navigate_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
            callback_group=self._callback_group,
        )
        route_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.route_pub = self.create_publisher(Path, self.route_topic, route_qos)
        self.route_marker_array_pub = self.create_publisher(
            MarkerArray,
            self.route_marker_array_topic,
            route_qos,
        )
        self.graph_pub = self.create_publisher(Marker, self.graph_topic, 1)
        self.graph_array_pub = self.create_publisher(
            MarkerArray,
            self.graph_marker_array_topic,
            1,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, '/osm_route_status', status_qos)
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
            callback_group=self._callback_group,
        )

        self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_fix_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            GeoPoseStamped,
            self.gps_goal_topic,
            self.gps_goal_callback,
            10,
            callback_group=self._callback_group,
        )
        if self.accept_rviz_goals:
            self.create_subscription(
                PoseStamped,
                self.goal_topic,
                self.goal_callback,
                10,
                callback_group=self._callback_group,
            )
        self.create_timer(
            self.graph_publish_period_sec,
            self.publish_graph_marker,
            callback_group=self._callback_group,
        )

        self.nodes = {}
        self.graph = {}
        self.segments = []
        self.road_way_count = 0
        self.load_osm_graph()
        self.load_static_map_georef()

        self.get_logger().info(
            'OSM route planner started: '
            f'osm_file={self.osm_file or "n/a"}, '
            f'roads={self.road_way_count}, nodes={len(self.nodes)}, '
            f'nav2_action_mode={self.nav2_action_mode}, '
            f'snap_heading_weight_m={self.snap_heading_weight_m:.1f}, '
            f'goal_topic={self.goal_topic if self.accept_rviz_goals else "disabled"}, '
            f'georef_json={self.georef_json or "n/a"}'
        )
        self.publish_status(
            'osm_route_status | ready=True | '
            f'roads={self.road_way_count} | nodes={len(self.nodes)} | '
            f'goal_topic={self.goal_topic if self.accept_rviz_goals else "disabled"}'
        )

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_graph_visual(self, marker: Marker):
        self.graph_pub.publish(marker)
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.graph_array_pub.publish(marker_array)

    def gps_fix_callback(self, msg: NavSatFix):
        if not is_valid_navsat_fix(msg):
            self.get_logger().warn(
                'Ignoring GPS fix without a valid position',
                throttle_duration_sec=5.0,
            )
            return
        self.latest_gps_fix = msg
        self._gps_fix_event.set()

    def map_callback(self, msg: OccupancyGrid):
        origin = msg.info.origin
        yaw = quaternion_to_yaw(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        if not all(
            math.isfinite(value)
            for value in (origin.position.x, origin.position.y, yaw)
        ):
            self.get_logger().warn('Ignoring /map origin with non-finite values')
            return

        self.static_map_origin = (origin.position.x, origin.position.y, yaw)
        if not self.static_map_logged:
            self.static_map_logged = True
            self.get_logger().info(
                'Using georeferenced /map origin for OSM route conversion: '
                f'origin=({origin.position.x:.2f}, {origin.position.y:.2f}), '
                f'yaw={math.degrees(yaw):.1f} deg'
            )

    def load_static_map_georef(self, *, log_missing: bool = True):
        if self.static_map_georef is not None:
            return True
        if not self.georef_json:
            return False
        if not os.path.isfile(self.georef_json):
            if log_missing:
                self.get_logger().warn(
                    f'Waiting for static map georeference metadata: {self.georef_json}'
                )
            return False

        try:
            with open(self.georef_json, 'r', encoding='utf-8') as georef_file:
                georef = json.load(georef_file)
            self.static_map_georef = {
                'reference_lat': float(georef['reference_lat']),
                'reference_lon': float(georef['reference_lon']),
                'min_x_m': float(georef['min_x_m']),
                'min_y_m': float(georef['min_y_m']),
            }
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to load static map georeference metadata: {exc}'
            )
            self.static_map_georef = None
            return False

        if not self.static_georef_logged:
            self.static_georef_logged = True
            self.get_logger().info(
                'Loaded static map georeference metadata for OSM route conversion'
            )
        return True

    def has_static_map_alignment(self):
        if self.static_map_georef is None:
            self.load_static_map_georef(log_missing=False)
        return self.static_map_georef is not None and self.static_map_origin is not None

    def static_map_alignment_required(self):
        return bool(self.georef_json)

    def static_map_alignment_error(self):
        return (
            'Waiting for georeferenced /map origin before converting OSM routes. '
            'Check georeferenced_static_map and map_ready_waiter logs.'
        )

    def latlon_to_static_map_xy(self, lat: float, lon: float):
        georef = self.static_map_georef
        origin_x, origin_y, origin_yaw = self.static_map_origin
        local_x, local_y = latlon_delta_m(
            georef['reference_lat'],
            georef['reference_lon'],
            lat,
            lon,
        )
        rel_x = local_x - georef['min_x_m']
        rel_y = local_y - georef['min_y_m']
        map_x = origin_x + math.cos(origin_yaw) * rel_x - math.sin(origin_yaw) * rel_y
        map_y = origin_y + math.sin(origin_yaw) * rel_x + math.cos(origin_yaw) * rel_y
        return map_x, map_y

    def static_map_xy_to_latlon(self, map_x: float, map_y: float):
        georef = self.static_map_georef
        origin_x, origin_y, origin_yaw = self.static_map_origin
        delta_x = map_x - origin_x
        delta_y = map_y - origin_y
        rel_x = math.cos(origin_yaw) * delta_x + math.sin(origin_yaw) * delta_y
        rel_y = -math.sin(origin_yaw) * delta_x + math.cos(origin_yaw) * delta_y

        local_x = rel_x + georef['min_x_m']
        local_y = rel_y + georef['min_y_m']
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(georef['reference_lat']))
        lat = georef['reference_lat'] + local_y / meters_per_deg_lat
        lon = georef['reference_lon'] + local_x / max(meters_per_deg_lon, 1e-6)
        return lat, lon

    def wait_for_gps_fix(self, timeout_sec: float = 2.0) -> bool:
        if self.latest_gps_fix is not None:
            return True
        self.get_logger().warn('Waiting for GPS fix...')
        return self._gps_fix_event.wait(timeout=timeout_sec)

    def load_osm_graph(self):
        self.nodes = {}
        self.graph = {}
        self.segments = []
        self.road_way_count = 0

        if not self.osm_file:
            self.get_logger().warn(
                'No OSM file configured. Route service will stay idle until '
                'osm_file is set in the launch or node parameters.'
            )
            return
        if not os.path.isfile(self.osm_file):
            self.get_logger().error(f'OSM file not found: {self.osm_file}')
            return

        raw_nodes, ways = load_osm_extract(self.osm_file)

        graph = {}
        road_way_count = 0
        for way in ways:
            tags = way['tags']
            highway_type = tags.get('highway')
            if highway_type not in self.allowed_highway_types:
                continue
            if tags.get('access') == 'no':
                continue

            refs = [ref for ref in way['refs'] if ref in raw_nodes]
            if len(refs) < 2:
                continue

            oneway_value = tags.get('oneway', 'no').lower()
            forward_only = oneway_value in ('yes', '1', 'true')
            backward_only = oneway_value == '-1'

            for idx in range(len(refs) - 1):
                start_id = refs[idx]
                end_id = refs[idx + 1]
                start_lat, start_lon = raw_nodes[start_id]
                end_lat, end_lon = raw_nodes[end_id]
                edge_distance = distance_m(start_lat, start_lon, end_lat, end_lon)
                if edge_distance <= 0.01:
                    continue

                graph.setdefault(start_id, []).append((end_id, edge_distance))
                if not forward_only:
                    graph.setdefault(end_id, []).append((start_id, edge_distance))
                if backward_only:
                    graph.setdefault(end_id, []).append((start_id, edge_distance))
                    graph[start_id] = [
                        edge for edge in graph.get(start_id, []) if edge[0] != end_id
                    ]
                self.segments.append({
                    'start_id': start_id,
                    'end_id': end_id,
                    'start_lat': start_lat,
                    'start_lon': start_lon,
                    'end_lat': end_lat,
                    'end_lon': end_lon,
                    'length_m': edge_distance,
                })

            road_way_count += 1

        referenced_node_ids = set(graph.keys())
        for adjacency in graph.values():
            for node_id, _ in adjacency:
                referenced_node_ids.add(node_id)

        self.nodes = {node_id: raw_nodes[node_id] for node_id in referenced_node_ids}
        self.graph = graph
        self.road_way_count = road_way_count

    def nearest_graph_node(self, lat: float, lon: float):
        best_node_id = None
        best_distance = float('inf')
        for node_id, (node_lat, node_lon) in self.nodes.items():
            node_distance = distance_m(lat, lon, node_lat, node_lon)
            if node_distance < best_distance:
                best_distance = node_distance
                best_node_id = node_id
        return best_node_id, best_distance

    def lookup_utm_to_map_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            q = transform.transform.rotation
            return quaternion_to_yaw(q.x, q.y, q.z, q.w)
        except Exception as exc:
            self.get_logger().warn(
                'UTM->map transform unavailable for heading-aware snap, '
                f'assuming ENU-aligned map: {exc}',
                throttle_duration_sec=10.0,
            )
            return 0.0

    def segment_heading_in_map(self, segment, utm_to_map_yaw: float = 0.0):
        if self.has_static_map_alignment():
            start_x, start_y = self.latlon_to_static_map_xy(
                segment['start_lat'],
                segment['start_lon'],
            )
            end_x, end_y = self.latlon_to_static_map_xy(
                segment['end_lat'],
                segment['end_lon'],
            )
            return math.atan2(end_y - start_y, end_x - start_x)

        delta_east, delta_north = latlon_delta_m(
            segment['start_lat'],
            segment['start_lon'],
            segment['end_lat'],
            segment['end_lon'],
        )
        delta_map_x = (
            math.cos(utm_to_map_yaw) * delta_east
            - math.sin(utm_to_map_yaw) * delta_north
        )
        delta_map_y = (
            math.sin(utm_to_map_yaw) * delta_east
            + math.cos(utm_to_map_yaw) * delta_north
        )
        return math.atan2(delta_map_y, delta_map_x)

    def heading_axis_error(self, segment_yaw: float, preferred_yaw: float):
        heading_error = abs(wrap_to_pi(segment_yaw - preferred_yaw))
        return min(heading_error, abs(math.pi - heading_error))

    def nearest_graph_projection(
        self,
        lat: float,
        lon: float,
        preferred_yaw: float = None,
    ):
        best_projection = None
        best_score = float('inf')
        use_heading = (
            preferred_yaw is not None
            and math.isfinite(preferred_yaw)
            and self.snap_heading_weight_m > 0.0
        )
        utm_to_map_yaw = 0.0
        if use_heading and not self.has_static_map_alignment():
            utm_to_map_yaw = self.lookup_utm_to_map_yaw()

        for segment in self.segments:
            start_east, start_north = latlon_delta_m(
                lat,
                lon,
                segment['start_lat'],
                segment['start_lon'],
            )
            end_east, end_north = latlon_delta_m(
                lat,
                lon,
                segment['end_lat'],
                segment['end_lon'],
            )
            seg_dx = end_east - start_east
            seg_dy = end_north - start_north
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
            if seg_len_sq <= 1e-6:
                continue

            # Project the query point (the origin in this local frame) onto the segment.
            t = -((start_east * seg_dx) + (start_north * seg_dy)) / seg_len_sq
            t = min(1.0, max(0.0, t))

            proj_east = start_east + t * seg_dx
            proj_north = start_north + t * seg_dy
            projection_distance = math.hypot(proj_east, proj_north)
            heading_error = None
            heading_score = 0.0
            if use_heading:
                segment_yaw = self.segment_heading_in_map(segment, utm_to_map_yaw)
                heading_error = self.heading_axis_error(segment_yaw, preferred_yaw)
                # 0 m penalty when aligned; up to snap_heading_weight_m for a
                # perpendicular road branch. Distance remains the primary snap
                # constraint through max_snap_distance_m.
                heading_score = self.snap_heading_weight_m * (1.0 - math.cos(heading_error))

            projection_score = projection_distance + heading_score
            if projection_score < best_score:
                proj_lat = segment['start_lat'] + t * (
                    segment['end_lat'] - segment['start_lat']
                )
                proj_lon = segment['start_lon'] + t * (
                    segment['end_lon'] - segment['start_lon']
                )
                best_score = projection_score
                best_projection = {
                    'lat': proj_lat,
                    'lon': proj_lon,
                    'distance_m': projection_distance,
                    'score_m': projection_score,
                    'heading_error_rad': (
                        heading_error if heading_error is not None else float('nan')
                    ),
                    'segment': segment,
                    't': t,
                }

        return best_projection

    def graph_has_edge(self, from_id, to_id):
        return any(neighbor == to_id for neighbor, _ in self.graph.get(from_id, []))

    def node_latlon(self, node_id, extra_nodes=None):
        if extra_nodes is not None and node_id in extra_nodes:
            return extra_nodes[node_id]
        return self.nodes[node_id]

    def add_projection_node(self, node_id, projection, extra_nodes, extra_edges):
        extra_nodes[node_id] = (projection['lat'], projection['lon'])

        segment = projection['segment']
        start_id = segment['start_id']
        end_id = segment['end_id']
        length_m = max(segment['length_m'], 0.0)
        t = min(1.0, max(0.0, projection['t']))
        distance_from_start = t * length_m
        distance_from_end = (1.0 - t) * length_m

        if self.graph_has_edge(start_id, end_id):
            extra_edges.setdefault(node_id, []).append((end_id, distance_from_end))
            extra_edges.setdefault(start_id, []).append((node_id, distance_from_start))
        if self.graph_has_edge(end_id, start_id):
            extra_edges.setdefault(node_id, []).append((start_id, distance_from_start))
            extra_edges.setdefault(end_id, []).append((node_id, distance_from_end))

    def add_same_segment_projection_edge(
        self,
        start_node_id,
        start_projection,
        goal_node_id,
        goal_projection,
        extra_edges,
    ):
        if start_projection is None or goal_projection is None:
            return

        start_segment = start_projection['segment']
        goal_segment = goal_projection['segment']
        same_segment = (
            start_segment['start_id'] == goal_segment['start_id']
            and start_segment['end_id'] == goal_segment['end_id']
        )
        if not same_segment:
            return

        t_start = min(1.0, max(0.0, start_projection['t']))
        t_goal = min(1.0, max(0.0, goal_projection['t']))
        distance = abs(t_goal - t_start) * max(start_segment['length_m'], 0.0)
        if distance <= 1e-6:
            extra_edges.setdefault(start_node_id, []).append((goal_node_id, 0.0))
            return

        if t_goal > t_start and self.graph_has_edge(
            start_segment['start_id'],
            start_segment['end_id'],
        ):
            extra_edges.setdefault(start_node_id, []).append((goal_node_id, distance))
        elif t_goal < t_start and self.graph_has_edge(
            start_segment['end_id'],
            start_segment['start_id'],
        ):
            extra_edges.setdefault(start_node_id, []).append((goal_node_id, distance))

    def plan_route_node_ids(self, start_id, goal_id, extra_nodes=None, extra_edges=None):
        frontier = [(0.0, start_id)]
        came_from = {start_id: None}
        cost_so_far = {start_id: 0.0}
        extra_edges = extra_edges or {}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_id:
                break

            neighbors = list(self.graph.get(current, []))
            neighbors.extend(extra_edges.get(current, []))
            for neighbor, edge_cost in neighbors:
                new_cost = cost_so_far[current] + edge_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    goal_lat, goal_lon = self.node_latlon(goal_id, extra_nodes)
                    neigh_lat, neigh_lon = self.node_latlon(neighbor, extra_nodes)
                    heuristic = distance_m(neigh_lat, neigh_lon, goal_lat, goal_lon)
                    priority = new_cost + heuristic
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        if goal_id not in came_from:
            return None

        route = []
        current = goal_id
        while current is not None:
            route.append(current)
            current = came_from[current]
        route.reverse()
        return route

    def densify_route(self, route_node_ids, extra_nodes=None):
        if not route_node_ids:
            return []

        route_latlons = [
            self.node_latlon(node_id, extra_nodes) for node_id in route_node_ids
        ]
        if len(route_latlons) == 1:
            return route_latlons

        dense_points = [route_latlons[0]]
        for idx in range(len(route_latlons) - 1):
            start_lat, start_lon = route_latlons[idx]
            end_lat, end_lon = route_latlons[idx + 1]
            segment_length = distance_m(start_lat, start_lon, end_lat, end_lon)
            subdivisions = max(1, int(segment_length / max(self.waypoint_spacing_m, 0.5)))

            for sub_idx in range(1, subdivisions + 1):
                ratio = sub_idx / subdivisions
                interp_lat = start_lat + ratio * (end_lat - start_lat)
                interp_lon = start_lon + ratio * (end_lon - start_lon)
                dense_points.append((interp_lat, interp_lon))

        return dense_points

    def rotate_enu_delta_into_map(self, delta_east: float, delta_north: float):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            q = transform.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            delta_map_x = math.cos(yaw) * delta_east - math.sin(yaw) * delta_north
            delta_map_y = math.sin(yaw) * delta_east + math.cos(yaw) * delta_north
            return delta_map_x, delta_map_y
        except Exception as exc:
            self.get_logger().warn(
                f'UTM->map transform unavailable, assuming ENU-aligned map: {exc}',
                throttle_duration_sec=10.0,
            )
            return delta_east, delta_north

    def rotate_map_delta_into_enu(self, delta_map_x: float, delta_map_y: float):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            q = transform.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            delta_east = math.cos(yaw) * delta_map_x + math.sin(yaw) * delta_map_y
            delta_north = -math.sin(yaw) * delta_map_x + math.cos(yaw) * delta_map_y
            return delta_east, delta_north
        except Exception as exc:
            self.get_logger().warn(
                f'UTM->map transform unavailable, assuming ENU-aligned map: {exc}',
                throttle_duration_sec=10.0,
            )
            return delta_map_x, delta_map_y

    def current_robot_map_pose(self):
        robot_x, robot_y, _ = self.current_robot_map_pose_with_yaw()
        return robot_x, robot_y

    def current_robot_map_pose_with_yaw(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0),
        )
        q = transform.transform.rotation
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )

    def latlon_to_map_pose(self, lat: float, lon: float, yaw: float):
        if self.latest_gps_fix is None:
            raise RuntimeError('No GPS fix available')

        robot_x, robot_y = self.current_robot_map_pose()
        map_x, map_y = self.latlon_to_map_xy_with_ref(
            self.latest_gps_fix.latitude,
            self.latest_gps_fix.longitude,
            robot_x,
            robot_y,
            lat,
            lon,
        )

        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def latlon_to_map_xy_with_ref(
        self,
        ref_lat: float,
        ref_lon: float,
        robot_x: float,
        robot_y: float,
        lat: float,
        lon: float,
    ):
        if self.has_static_map_alignment():
            return self.latlon_to_static_map_xy(lat, lon)
        if self.static_map_alignment_required():
            raise RuntimeError(self.static_map_alignment_error())

        delta_east, delta_north = latlon_delta_m(ref_lat, ref_lon, lat, lon)
        delta_map_x, delta_map_y = self.rotate_enu_delta_into_map(
            delta_east,
            delta_north,
        )
        return robot_x + delta_map_x, robot_y + delta_map_y

    def project_latlon_to_map_xy(self, lat: float, lon: float):
        if self.latest_gps_fix is None:
            raise RuntimeError('No GPS fix available')

        robot_x, robot_y = self.current_robot_map_pose()
        return self.latlon_to_map_xy_with_ref(
            self.latest_gps_fix.latitude,
            self.latest_gps_fix.longitude,
            robot_x,
            robot_y,
            lat,
            lon,
        )

    def publish_graph_marker(self):
        if not self.segments:
            return

        robot_pose_available = True
        try:
            robot_x, robot_y = self.current_robot_map_pose()
        except Exception:
            robot_x = 0.0
            robot_y = 0.0
            robot_pose_available = False

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'osm_graph'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.30
        marker.color.r = 1.0
        marker.color.g = 0.76
        marker.color.b = 0.10
        marker.color.a = 0.85

        if self.has_static_map_alignment():
            for segment in self.segments:
                start_x, start_y = self.latlon_to_static_map_xy(
                    segment['start_lat'],
                    segment['start_lon'],
                )
                end_x, end_y = self.latlon_to_static_map_xy(
                    segment['end_lat'],
                    segment['end_lon'],
                )
                if (
                    robot_pose_available
                    and self.graph_display_radius_m > 0.0
                    and math.hypot(
                        0.5 * (start_x + end_x) - robot_x,
                        0.5 * (start_y + end_y) - robot_y,
                    ) > self.graph_display_radius_m
                ):
                    continue

                start_point = Point()
                start_point.x = start_x
                start_point.y = start_y
                start_point.z = 0.05
                end_point = Point()
                end_point.x = end_x
                end_point.y = end_y
                end_point.z = 0.05
                marker.points.append(start_point)
                marker.points.append(end_point)

            self.publish_graph_visual(marker)
            return
        if self.static_map_alignment_required():
            return

        if not robot_pose_available or self.latest_gps_fix is None:
            return

        ref_lat = self.latest_gps_fix.latitude
        ref_lon = self.latest_gps_fix.longitude

        for segment in self.segments:
            if (
                self.graph_display_radius_m > 0.0
                and distance_m(
                    ref_lat,
                    ref_lon,
                    0.5 * (segment['start_lat'] + segment['end_lat']),
                    0.5 * (segment['start_lon'] + segment['end_lon']),
                ) > self.graph_display_radius_m
            ):
                continue

            start_x, start_y = self.latlon_to_map_xy_with_ref(
                ref_lat,
                ref_lon,
                robot_x,
                robot_y,
                segment['start_lat'],
                segment['start_lon'],
            )
            end_x, end_y = self.latlon_to_map_xy_with_ref(
                ref_lat,
                ref_lon,
                robot_x,
                robot_y,
                segment['end_lat'],
                segment['end_lon'],
            )

            start_point = Point()
            start_point.x = start_x
            start_point.y = start_y
            start_point.z = 0.05
            end_point = Point()
            end_point.x = end_x
            end_point.y = end_y
            end_point.z = 0.05
            marker.points.append(start_point)
            marker.points.append(end_point)

        self.publish_graph_visual(marker)

    def map_goal_to_latlon(self, goal_pose_map: PoseStamped):
        if self.has_static_map_alignment():
            return self.static_map_xy_to_latlon(
                goal_pose_map.pose.position.x,
                goal_pose_map.pose.position.y,
            )
        if self.static_map_alignment_required():
            raise RuntimeError(self.static_map_alignment_error())

        if self.latest_gps_fix is None:
            raise RuntimeError('No GPS fix available')

        robot_x, robot_y = self.current_robot_map_pose()
        delta_map_x = goal_pose_map.pose.position.x - robot_x
        delta_map_y = goal_pose_map.pose.position.y - robot_y
        delta_east, delta_north = self.rotate_map_delta_into_enu(
            delta_map_x,
            delta_map_y,
        )

        ref_lat = self.latest_gps_fix.latitude
        ref_lon = self.latest_gps_fix.longitude
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(ref_lat))
        goal_lat = ref_lat + delta_north / meters_per_deg_lat
        goal_lon = ref_lon + delta_east / max(meters_per_deg_lon, 1e-6)
        return goal_lat, goal_lon

    def transform_goal_to_map(self, goal_pose: PoseStamped):
        source_frame = goal_pose.header.frame_id or self.map_frame
        if source_frame == self.map_frame:
            return goal_pose

        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0),
        )

        tf_t = transform.transform.translation
        tf_q = transform.transform.rotation
        tf_yaw = quaternion_to_yaw(tf_q.x, tf_q.y, tf_q.z, tf_q.w)

        src_x = goal_pose.pose.position.x
        src_y = goal_pose.pose.position.y
        dst_x = tf_t.x + math.cos(tf_yaw) * src_x - math.sin(tf_yaw) * src_y
        dst_y = tf_t.y + math.sin(tf_yaw) * src_x + math.cos(tf_yaw) * src_y

        src_q = goal_pose.pose.orientation
        src_yaw = quaternion_to_yaw(src_q.x, src_q.y, src_q.z, src_q.w)
        dst_yaw = wrap_to_pi(tf_yaw + src_yaw)
        qx, qy, qz, qw = quaternion_from_yaw(dst_yaw)

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

    def latlon_route_to_map_poses(self, route_latlons, final_yaw: float):
        if self.latest_gps_fix is None:
            raise RuntimeError('No GPS fix available')

        route_points_map = []
        if self.has_static_map_alignment():
            for lat, lon in route_latlons:
                route_points_map.append(self.latlon_to_static_map_xy(lat, lon))
        else:
            robot_x, robot_y = self.current_robot_map_pose()
            ref_lat = self.latest_gps_fix.latitude
            ref_lon = self.latest_gps_fix.longitude

            for lat, lon in route_latlons:
                delta_east, delta_north = latlon_delta_m(ref_lat, ref_lon, lat, lon)
                delta_map_x, delta_map_y = self.rotate_enu_delta_into_map(
                    delta_east, delta_north
                )
                route_points_map.append((robot_x + delta_map_x, robot_y + delta_map_y))

        poses = []
        for idx, (map_x, map_y) in enumerate(route_points_map):
            if idx < len(route_points_map) - 1:
                next_x, next_y = route_points_map[idx + 1]
                yaw = math.atan2(next_y - map_y, next_x - map_x)
            else:
                yaw = final_yaw

            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = map_x
            pose.pose.position.y = map_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)

        return poses

    def prune_initial_route_poses(self, poses):
        if not self.prune_start_behind_robot or len(poses) <= 1:
            return poses

        try:
            robot_x, robot_y, robot_yaw = self.current_robot_map_pose_with_yaw()
        except Exception as exc:
            self.get_logger().warn(
                f'Could not prune route start behind robot: {exc}',
                throttle_duration_sec=5.0,
            )
            return poses

        forward_x = math.cos(robot_yaw)
        forward_y = math.sin(robot_yaw)
        pruned_count = 0

        # Never prune the route down to a single pose. A one-point Path is not
        # drawable in RViz and gives Nav2 no usable route direction.
        while len(poses) > 2:
            first = poses[0].pose.position
            dx = first.x - robot_x
            dy = first.y - robot_y
            distance = math.hypot(dx, dy)
            forward_distance = dx * forward_x + dy * forward_y

            if distance <= self.start_prune_distance_m or forward_distance < -0.1:
                poses.pop(0)
                pruned_count += 1
                continue
            break

        if pruned_count:
            self.get_logger().info(
                f'Pruned {pruned_count} initial OSM waypoint(s) behind/near the robot'
            )
        return poses

    def publish_route_path(self, poses):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        display_poses = list(poses)
        if len(display_poses) == 1:
            try:
                robot_x, robot_y, robot_yaw = self.current_robot_map_pose_with_yaw()
                display_poses.insert(0, self.pose_from_xy_yaw(robot_x, robot_y, robot_yaw))
            except Exception as exc:
                self.get_logger().warn(
                    f'OSM route had one pose and robot pose was unavailable for display: {exc}',
                    throttle_duration_sec=5.0,
                )
        path.poses = display_poses
        self.route_pub.publish(path)
        self.publish_route_visual_markers(display_poses, path.header.stamp)

    def publish_route_visual_markers(self, poses, stamp):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame
        delete_marker.header.stamp = stamp
        delete_marker.ns = 'delivery_route'
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        if not poses:
            self.route_marker_array_pub.publish(marker_array)
            return

        route_marker = Marker()
        route_marker.header.frame_id = self.map_frame
        route_marker.header.stamp = stamp
        route_marker.ns = 'delivery_route'
        route_marker.id = 0
        route_marker.type = Marker.LINE_STRIP
        route_marker.action = Marker.ADD
        route_marker.pose.orientation.w = 1.0
        route_marker.scale.x = 0.35
        route_marker.color.r = 0.05
        route_marker.color.g = 0.38
        route_marker.color.b = 1.0
        route_marker.color.a = 0.95
        for pose in poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.35
            route_marker.points.append(point)
        marker_array.markers.append(route_marker)

        start_marker = Marker()
        start_marker.header.frame_id = self.map_frame
        start_marker.header.stamp = stamp
        start_marker.ns = 'delivery_route'
        start_marker.id = 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = poses[0].pose.position.x
        start_marker.pose.position.y = poses[0].pose.position.y
        start_marker.pose.position.z = 0.45
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.8
        start_marker.scale.y = 0.8
        start_marker.scale.z = 0.25
        start_marker.color.r = 0.0
        start_marker.color.g = 0.85
        start_marker.color.b = 0.25
        start_marker.color.a = 0.95
        marker_array.markers.append(start_marker)

        goal_marker = Marker()
        goal_marker.header.frame_id = self.map_frame
        goal_marker.header.stamp = stamp
        goal_marker.ns = 'delivery_route'
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = poses[-1].pose.position.x
        goal_marker.pose.position.y = poses[-1].pose.position.y
        goal_marker.pose.position.z = 0.45
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.9
        goal_marker.scale.y = 0.9
        goal_marker.scale.z = 0.3
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.15
        goal_marker.color.b = 0.10
        goal_marker.color.a = 0.95
        marker_array.markers.append(goal_marker)

        goal_arrow = Marker()
        goal_arrow.header.frame_id = self.map_frame
        goal_arrow.header.stamp = stamp
        goal_arrow.ns = 'delivery_route'
        goal_arrow.id = 3
        goal_arrow.type = Marker.ARROW
        goal_arrow.action = Marker.ADD
        goal_arrow.pose.position.x = poses[-1].pose.position.x
        goal_arrow.pose.position.y = poses[-1].pose.position.y
        goal_arrow.pose.position.z = 0.65
        goal_arrow.pose.orientation.x = poses[-1].pose.orientation.x
        goal_arrow.pose.orientation.y = poses[-1].pose.orientation.y
        goal_arrow.pose.orientation.z = poses[-1].pose.orientation.z
        goal_arrow.pose.orientation.w = poses[-1].pose.orientation.w
        goal_arrow.scale.x = 1.4
        goal_arrow.scale.y = 0.22
        goal_arrow.scale.z = 0.22
        goal_arrow.color.r = 1.0
        goal_arrow.color.g = 0.15
        goal_arrow.color.b = 0.10
        goal_arrow.color.a = 0.95
        marker_array.markers.append(goal_arrow)

        self.route_marker_array_pub.publish(marker_array)

    def pose_from_xy_yaw(self, x: float, y: float, yaw: float):
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def set_pose_yaw(self, pose: PoseStamped, yaw: float):
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def robot_to_goal_distance_yaw(self, goal_pose_map: PoseStamped):
        robot_x, robot_y, robot_yaw = self.current_robot_map_pose_with_yaw()
        dx = goal_pose_map.pose.position.x - robot_x
        dy = goal_pose_map.pose.position.y - robot_y
        distance = math.hypot(dx, dy)
        if distance > 0.05:
            yaw = math.atan2(dy, dx)
        else:
            yaw = robot_yaw
        return distance, yaw, robot_x, robot_y, robot_yaw

    def send_direct_goal(
        self,
        goal_pose_map: PoseStamped,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ):
        start_pose = self.pose_from_xy_yaw(robot_x, robot_y, robot_yaw)
        self.publish_route_path([start_pose, goal_pose_map])
        return self.send_route_to_nav2([goal_pose_map])

    def build_route_poses(
        self,
        goal_lat: float,
        goal_lon: float,
        final_yaw: float,
        exact_goal_pose: PoseStamped = None,
    ):
        if not self.wait_for_gps_fix():
            raise RuntimeError('No GPS fix received on /gps/fix yet')
        if not self.graph:
            raise RuntimeError(
                'OSM graph is empty. Set a valid osm_file before requesting routes.'
            )

        robot_x, robot_y, robot_yaw = self.current_robot_map_pose_with_yaw()
        if self.has_static_map_alignment():
            start_lat, start_lon = self.static_map_xy_to_latlon(robot_x, robot_y)
        else:
            start_lat = self.latest_gps_fix.latitude
            start_lon = self.latest_gps_fix.longitude
        nearest_start_projection = None
        nearest_goal_projection = None
        start_projection = None
        goal_projection = None

        if self.snap_start_to_graph:
            nearest_start_projection = self.nearest_graph_projection(
                start_lat,
                start_lon,
                preferred_yaw=robot_yaw,
            )
            if (
                nearest_start_projection is not None
                and nearest_start_projection['distance_m'] <= self.max_snap_distance_m
            ):
                start_projection = nearest_start_projection
                start_lat = start_projection['lat']
                start_lon = start_projection['lon']

        if self.snap_goal_to_graph:
            nearest_goal_projection = self.nearest_graph_projection(
                goal_lat,
                goal_lon,
                preferred_yaw=final_yaw,
            )
            if (
                nearest_goal_projection is not None
                and nearest_goal_projection['distance_m'] <= self.max_snap_distance_m
            ):
                goal_projection = nearest_goal_projection
                goal_lat = goal_projection['lat']
                goal_lon = goal_projection['lon']

        extra_nodes = {}
        extra_edges = {}
        start_id = None
        goal_id = None

        if start_projection is not None:
            start_id = '__start_projection__'
            start_dist = start_projection['distance_m']
            self.add_projection_node(
                start_id,
                start_projection,
                extra_nodes,
                extra_edges,
            )
        else:
            start_id, start_dist = self.nearest_graph_node(start_lat, start_lon)

        if goal_projection is not None:
            goal_id = '__goal_projection__'
            goal_dist = goal_projection['distance_m']
            self.add_projection_node(
                goal_id,
                goal_projection,
                extra_nodes,
                extra_edges,
            )
        else:
            goal_id, goal_dist = self.nearest_graph_node(goal_lat, goal_lon)

        if start_id is None or goal_id is None:
            raise RuntimeError('Could not snap start or goal to the OSM graph')

        self.add_same_segment_projection_edge(
            start_id,
            start_projection,
            goal_id,
            goal_projection,
            extra_edges,
        )

        route_node_ids = self.plan_route_node_ids(
            start_id,
            goal_id,
            extra_nodes=extra_nodes,
            extra_edges=extra_edges,
        )
        if not route_node_ids:
            raise RuntimeError('No OSM route found between the start and goal')

        dense_latlons = self.densify_route(route_node_ids, extra_nodes=extra_nodes)
        poses = self.latlon_route_to_map_poses(dense_latlons, final_yaw)

        if (
            self.prepend_snapped_start
            and start_projection is not None
            and route_node_ids[0] != start_id
        ):
            start_x, start_y = self.project_latlon_to_map_xy(
                start_projection['lat'],
                start_projection['lon'],
            )
            if poses:
                next_pose = poses[0]
                start_yaw = math.atan2(
                    next_pose.pose.position.y - start_y,
                    next_pose.pose.position.x - start_x,
                )
            else:
                start_yaw = final_yaw
            qx, qy, qz, qw = quaternion_from_yaw(start_yaw)
            start_pose = PoseStamped()
            start_pose.header.frame_id = self.map_frame
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose.position.x = start_x
            start_pose.pose.position.y = start_y
            start_pose.pose.position.z = 0.0
            start_pose.pose.orientation.x = qx
            start_pose.pose.orientation.y = qy
            start_pose.pose.orientation.z = qz
            start_pose.pose.orientation.w = qw
            poses.insert(0, start_pose)

        if exact_goal_pose is None and self.append_exact_goal:
            exact_goal_pose = self.latlon_to_map_pose(goal_lat, goal_lon, final_yaw)

        if exact_goal_pose is not None:
            if poses:
                last_pose = poses[-1]
                goal_dx = exact_goal_pose.pose.position.x - last_pose.pose.position.x
                goal_dy = exact_goal_pose.pose.position.y - last_pose.pose.position.y
                if math.hypot(goal_dx, goal_dy) > max(0.75, self.waypoint_spacing_m * 0.5):
                    poses.append(exact_goal_pose)
                else:
                    poses[-1] = exact_goal_pose
            else:
                poses.append(exact_goal_pose)

        poses = self.prune_initial_route_poses(poses)

        snap_summary = {
            'start_dist_m': start_dist,
            'goal_dist_m': goal_dist,
            'start_proj_dist_m': (
                nearest_start_projection['distance_m']
                if nearest_start_projection is not None
                else float('nan')
            ),
            'goal_proj_dist_m': (
                nearest_goal_projection['distance_m']
                if nearest_goal_projection is not None
                else float('nan')
            ),
            'start_heading_error_deg': (
                math.degrees(nearest_start_projection['heading_error_rad'])
                if nearest_start_projection is not None
                and math.isfinite(nearest_start_projection['heading_error_rad'])
                else float('nan')
            ),
            'goal_heading_error_deg': (
                math.degrees(nearest_goal_projection['heading_error_rad'])
                if nearest_goal_projection is not None
                and math.isfinite(nearest_goal_projection['heading_error_rad'])
                else float('nan')
            ),
        }
        return poses, route_node_ids, snap_summary

    @staticmethod
    def pose_xy(pose: PoseStamped):
        return pose.pose.position.x, pose.pose.position.y

    @staticmethod
    def perpendicular_distance_m(point, start, end):
        px, py = point
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            return math.hypot(px - sx, py - sy)
        ratio = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / length_sq))
        proj_x = sx + ratio * dx
        proj_y = sy + ratio * dy
        return math.hypot(px - proj_x, py - proj_y)

    def rdp_pose_indices(self, poses, start_idx: int, end_idx: int, tolerance_m: float):
        if end_idx <= start_idx + 1:
            return [start_idx, end_idx] if end_idx > start_idx else [start_idx]

        start_xy = self.pose_xy(poses[start_idx])
        end_xy = self.pose_xy(poses[end_idx])
        max_dist = -1.0
        split_idx = start_idx
        for idx in range(start_idx + 1, end_idx):
            dist = self.perpendicular_distance_m(
                self.pose_xy(poses[idx]),
                start_xy,
                end_xy,
            )
            if dist > max_dist:
                max_dist = dist
                split_idx = idx

        if max_dist <= tolerance_m:
            return [start_idx, end_idx]

        left = self.rdp_pose_indices(poses, start_idx, split_idx, tolerance_m)
        right = self.rdp_pose_indices(poses, split_idx, end_idx, tolerance_m)
        return left[:-1] + right

    def simplify_nav2_poses(self, poses):
        if len(poses) <= 2:
            return list(poses)

        if self.nav2_simplify_tolerance_m > 0.0:
            indices = self.rdp_pose_indices(
                poses,
                0,
                len(poses) - 1,
                self.nav2_simplify_tolerance_m,
            )
        else:
            indices = list(range(len(poses)))

        indices = sorted(set(indices))
        if len(indices) > self.max_nav2_waypoints:
            stride = (len(indices) - 1) / float(self.max_nav2_waypoints - 1)
            capped = [indices[round(i * stride)] for i in range(self.max_nav2_waypoints)]
            capped[0] = 0
            capped[-1] = len(poses) - 1
            indices = sorted(set(capped))

        return [poses[idx] for idx in indices]

    def plan_and_send_route(
        self,
        goal_lat: float,
        goal_lon: float,
        final_yaw: float,
        exact_goal_pose: PoseStamped = None,
    ):
        poses, route_node_ids, snap_summary = self.build_route_poses(
            goal_lat,
            goal_lon,
            final_yaw,
            exact_goal_pose=exact_goal_pose,
        )
        self.publish_route_path(poses)
        nav2_poses = self.simplify_nav2_poses(poses)
        if len(nav2_poses) < len(poses):
            self.get_logger().info(
                'Simplified OSM route for Nav2 execution: '
                f'{len(poses)} display poses -> {len(nav2_poses)} Nav2 poses '
                f'(tolerance={self.nav2_simplify_tolerance_m:.1f}m, '
                f'max={self.max_nav2_waypoints})'
            )
        sent, nav2_message = self.send_route_to_nav2(nav2_poses)
        message = (
            f'{nav2_message}; route nodes={len(route_node_ids)}, '
            f'waypoints={len(poses)}, '
            f'nav2_waypoints={len(nav2_poses)}, '
            f'start_snap={snap_summary["start_dist_m"]:.1f}m, '
            f'goal_snap={snap_summary["goal_dist_m"]:.1f}m, '
            f'start_proj={snap_summary["start_proj_dist_m"]:.1f}m, '
            f'goal_proj={snap_summary["goal_proj_dist_m"]:.1f}m, '
            f'start_heading_err={snap_summary["start_heading_error_deg"]:.0f}deg, '
            f'goal_heading_err={snap_summary["goal_heading_error_deg"]:.0f}deg'
        )
        self.get_logger().info(
            f'OSM route planned: goal=({goal_lat:.6f}, {goal_lon:.6f}), '
            f'route_nodes={len(route_node_ids)}, waypoints={len(poses)}, '
            f'nav2_waypoints={len(nav2_poses)}, '
            f'start_snap={snap_summary["start_dist_m"]:.1f}m, '
            f'goal_snap={snap_summary["goal_dist_m"]:.1f}m, '
            f'start_proj={snap_summary["start_proj_dist_m"]:.1f}m, '
            f'goal_proj={snap_summary["goal_proj_dist_m"]:.1f}m, '
            f'start_heading_err={snap_summary["start_heading_error_deg"]:.0f}deg, '
            f'goal_heading_err={snap_summary["goal_heading_error_deg"]:.0f}deg'
        )
        return sent, message

    def send_route_to_nav2(self, poses):
        if self.nav2_action_mode in ('navigate_through_poses', 'through_poses', 'through'):
            return self.send_navigate_through_poses(poses)
        if self.nav2_action_mode in ('follow_waypoints', 'waypoints', 'follow'):
            return self.send_follow_waypoints(poses)

        self.get_logger().warn(
            f'Unknown nav2_action_mode={self.nav2_action_mode!r}; using navigate_through_poses'
        )
        return self.send_navigate_through_poses(poses)

    def send_navigate_through_poses(self, poses):
        if not self.send_to_nav2:
            return True, 'Route planned; send_to_nav2 disabled'

        if not self.navigate_through_poses_client.wait_for_server(
            timeout_sec=self.follow_waypoints_wait_sec
        ):
            return (
                False,
                'Nav2 action server /navigate_through_poses is not available. '
                'Check that bt_navigator is active and navigate_through_poses is configured.',
            )

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = ''

        result = {
            'accepted': False,
            'message': 'Timed out waiting for navigate_through_poses response',
        }
        result_event = threading.Event()

        def _goal_response_cb(future):
            try:
                goal_handle = future.result()
                if goal_handle is None:
                    result['message'] = 'NavigateThroughPoses returned no goal handle'
                    result_event.set()
                    return
                if not goal_handle.accepted:
                    result['message'] = 'NavigateThroughPoses rejected the OSM route'
                    result_event.set()
                    return

                result['accepted'] = True
                result['message'] = 'NavigateThroughPoses accepted the OSM route'
            except Exception as exc:
                result['message'] = f'Failed to send OSM route through poses: {exc}'
            finally:
                result_event.set()

        future = self.navigate_through_poses_client.send_goal_async(goal_msg)
        future.add_done_callback(_goal_response_cb)

        goal_response_timeout_sec = max(3.0, self.follow_waypoints_wait_sec)
        if not result_event.wait(timeout=goal_response_timeout_sec):
            return False, result['message']
        return result['accepted'], result['message']

    def send_follow_waypoints(self, poses):
        if not self.send_to_nav2:
            return True, 'Route planned; send_to_nav2 disabled'

        if not self.follow_waypoints_client.wait_for_server(
            timeout_sec=self.follow_waypoints_wait_sec
        ):
            return (
                False,
                'Nav2 action server /follow_waypoints is not available. '
                'Check that waypoint_follower is active.',
            )

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        result = {
            'accepted': False,
            'message': 'Timed out waiting for waypoint follower response',
        }
        result_event = threading.Event()

        def _goal_response_cb(future):
            try:
                goal_handle = future.result()
                if goal_handle is None:
                    result['message'] = 'Waypoint follower returned no goal handle'
                    result_event.set()
                    return
                if not goal_handle.accepted:
                    result['message'] = 'Waypoint follower rejected the OSM route'
                    result_event.set()
                    return

                result['accepted'] = True
                result['message'] = 'Waypoint follower accepted the OSM route'
            except Exception as exc:
                result['message'] = f'Failed to send OSM route: {exc}'
            finally:
                result_event.set()

        future = self.follow_waypoints_client.send_goal_async(goal_msg)
        future.add_done_callback(_goal_response_cb)

        goal_response_timeout_sec = max(3.0, self.follow_waypoints_wait_sec)
        if not result_event.wait(timeout=goal_response_timeout_sec):
            return False, result['message']
        return result['accepted'], result['message']

    def gps_goal_callback(self, msg: GeoPoseStamped):
        try:
            goal_lat = msg.pose.position.latitude
            goal_lon = msg.pose.position.longitude
            q = msg.pose.orientation
            final_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            if self.graph:
                sent, message = self.plan_and_send_route(
                    goal_lat,
                    goal_lon,
                    final_yaw,
                )
            else:
                if not self.wait_for_gps_fix():
                    raise RuntimeError(f'No GPS fix received on {self.gps_topic} yet')
                goal_pose_map = self.latlon_to_map_pose(goal_lat, goal_lon, final_yaw)
                _, _, robot_x, robot_y, robot_yaw = self.robot_to_goal_distance_yaw(
                    goal_pose_map
                )
                sent, message = self.send_direct_goal(
                    goal_pose_map,
                    robot_x,
                    robot_y,
                    robot_yaw,
                )
            self.publish_status(
                f'osm_route_status | gps_goal=True | accepted={sent} | '
                f'gps_lat={goal_lat:.6f} | gps_lon={goal_lon:.6f} | {message}'
            )
            log_fn = self.get_logger().info if sent else self.get_logger().warn
            log_fn(f'GPS goal ({goal_lat:.6f}, {goal_lon:.6f}): {message}')
        except Exception as exc:
            self.publish_status(f'osm_route_status | gps_goal=True | error={exc}')
            self.get_logger().error(f'GPS goal failed: {exc}', exc_info=True)

    def goal_callback(self, goal_pose: PoseStamped):
        try:
            self.publish_status(
                'osm_route_status | received_rviz_goal=True | '
                f'frame={goal_pose.header.frame_id or self.map_frame}'
            )
            goal_pose_map = self.transform_goal_to_map(goal_pose)
            goal_q = goal_pose_map.pose.orientation
            goal_yaw = quaternion_to_yaw(goal_q.x, goal_q.y, goal_q.z, goal_q.w)

            goal_distance, direct_yaw, robot_x, robot_y, robot_yaw = (
                self.robot_to_goal_distance_yaw(goal_pose_map)
            )
            if not self.respect_rviz_goal_orientation:
                goal_yaw = direct_yaw
                goal_pose_map = self.set_pose_yaw(goal_pose_map, goal_yaw)

            if (
                self.short_goal_direct_distance_m > 0.0
                and goal_distance <= self.short_goal_direct_distance_m
            ):
                sent, message = self.send_direct_goal(
                    goal_pose_map,
                    robot_x,
                    robot_y,
                    robot_yaw,
                )
                if sent:
                    self.publish_status(
                        'osm_route_status | direct_goal=True | accepted=True | '
                        f'map_x={goal_pose_map.pose.position.x:.2f} | '
                        f'map_y={goal_pose_map.pose.position.y:.2f} | '
                        f'distance_m={goal_distance:.2f}'
                    )
                    self.get_logger().info(
                        'RViz goal sent directly to Nav2 without OSM graph routing: '
                        f'distance={goal_distance:.2f}m, '
                        f'map=({goal_pose_map.pose.position.x:.2f}, '
                        f'{goal_pose_map.pose.position.y:.2f})'
                    )
                else:
                    self.publish_status(
                        'osm_route_status | direct_goal=True | accepted=False | '
                        f'reason={message}'
                    )
                    self.get_logger().warn(f'Direct RViz goal was rejected: {message}')
                return

            goal_lat, goal_lon = self.map_goal_to_latlon(goal_pose_map)
            sent, message = self.plan_and_send_route(
                goal_lat,
                goal_lon,
                goal_yaw,
                exact_goal_pose=None,
            )
            if sent:
                self.publish_status(
                    'osm_route_status | accepted=True | '
                    f'map_x={goal_pose_map.pose.position.x:.2f} | '
                    f'map_y={goal_pose_map.pose.position.y:.2f} | '
                    f'gps_lat={goal_lat:.6f} | gps_lon={goal_lon:.6f} | '
                    f'{message}'
                )
                self.get_logger().info(
                    'RViz goal routed on OSM graph: '
                    f'map=({goal_pose_map.pose.position.x:.2f}, '
                    f'{goal_pose_map.pose.position.y:.2f}), '
                    f'gps=({goal_lat:.6f}, {goal_lon:.6f})'
                )
            else:
                self.publish_status(f'osm_route_status | accepted=False | reason={message}')
                self.get_logger().warn(f'RViz goal routing was rejected: {message}')
        except Exception as exc:
            self.publish_status(f'osm_route_status | error={exc}')
            self.get_logger().error(f'Failed to route RViz goal: {exc}', exc_info=True)


def main():
    rclpy.init()
    node = OSMRoutePlanner()
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
