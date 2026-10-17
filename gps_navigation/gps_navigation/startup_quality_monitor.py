#!/usr/bin/env python3
"""
Decide when GPS startup localization is good enough to begin outdoor driving.

Readiness requires a GPS fix that snaps onto the OSM road graph and a horizontal
sigma below the configured limit for several consecutive samples.
"""

import math
import os
from typing import List, Optional, Tuple

from geometry_msgs.msg import Vector3Stamped
from gps_navigation.osm_loader import load_osm_extract
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


def latlon_delta_m(ref_lat: float, ref_lon: float, lat: float, lon: float):
    meters_per_deg_lat = 111320.0
    lat_rad = math.radians(ref_lat)
    meters_per_deg_lon = 111320.0 * math.cos(lat_rad)
    delta_north = (lat - ref_lat) * meters_per_deg_lat
    delta_east = (lon - ref_lon) * meters_per_deg_lon
    return delta_east, delta_north


def is_valid_navsat_fix(msg: NavSatFix) -> bool:
    if msg.status.status < 0:
        return False
    return math.isfinite(msg.latitude) and math.isfinite(msg.longitude)


class StartupQualityMonitor(Node):
    def __init__(self):
        super().__init__('startup_quality_monitor')

        self.declare_parameter('osm_file', '')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('max_snap_distance_m', 20.0)
        self.declare_parameter('max_horizontal_sigma_m', 6.0)
        self.declare_parameter('required_good_samples', 5)
        self.declare_parameter('publish_period_sec', 1.0)
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
        self.max_snap_distance_m = float(self.get_parameter('max_snap_distance_m').value)
        self.max_horizontal_sigma_m = float(
            self.get_parameter('max_horizontal_sigma_m').value
        )
        self.required_good_samples = max(
            1,
            int(self.get_parameter('required_good_samples').value),
        )
        self.publish_period_sec = max(
            0.2,
            float(self.get_parameter('publish_period_sec').value),
        )
        self.allowed_highway_types = set(
            self.get_parameter('allowed_highway_types').value
        )

        self.latest_gps_fix: Optional[NavSatFix] = None
        self.graph_segments: List[Tuple[float, float, float, float]] = []
        self.good_samples = 0
        self.last_ready = False
        self.last_status_text = ''

        self.status_pub = self.create_publisher(String, '/startup_quality/status', 10)
        self.metrics_pub = self.create_publisher(
            Vector3Stamped,
            '/startup_quality/metrics',
            10,
        )
        self.ready_pub = self.create_publisher(Bool, '/startup_quality/ready', 10)

        self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 10)
        self.create_timer(self.publish_period_sec, self.evaluate_quality)

        self.load_graph_segments()
        self.get_logger().info(
            'Startup quality monitor started: '
            f'osm_file={self.osm_file or "n/a"}, '
            f'max_snap_distance={self.max_snap_distance_m:.1f}m, '
            f'max_horizontal_sigma={self.max_horizontal_sigma_m:.1f}m'
        )

    def gps_callback(self, msg: NavSatFix):
        if not is_valid_navsat_fix(msg):
            self.get_logger().warn(
                'Ignoring GPS fix without a valid position',
                throttle_duration_sec=5.0,
            )
            return
        self.latest_gps_fix = msg

    def load_graph_segments(self):
        self.graph_segments = []
        if not self.osm_file:
            self.get_logger().warn(
                'No OSM extract configured; startup quality monitor will stay not ready.'
            )
            return
        if not os.path.isfile(self.osm_file):
            self.get_logger().error(f'OSM extract not found: {self.osm_file}')
            return

        try:
            nodes, ways = load_osm_extract(self.osm_file)
        except Exception as exc:
            self.get_logger().error(f'Failed to load OSM extract: {exc}')
            return

        for way in ways:
            tags = way['tags']
            highway_type = tags.get('highway')
            if highway_type not in self.allowed_highway_types:
                continue

            refs = [ref for ref in way['refs'] if ref in nodes]
            if len(refs) < 2:
                continue

            for idx in range(len(refs) - 1):
                start_lat, start_lon = nodes[refs[idx]]
                end_lat, end_lon = nodes[refs[idx + 1]]
                self.graph_segments.append((start_lat, start_lon, end_lat, end_lon))

        self.get_logger().info(
            f'Startup quality monitor loaded {len(self.graph_segments)} OSM road segments'
        )

    def nearest_graph_projection(self, lat: float, lon: float):
        best_projection = None
        best_distance = float('inf')

        for start_lat, start_lon, end_lat, end_lon in self.graph_segments:
            start_east, start_north = latlon_delta_m(lat, lon, start_lat, start_lon)
            end_east, end_north = latlon_delta_m(lat, lon, end_lat, end_lon)
            seg_dx = end_east - start_east
            seg_dy = end_north - start_north
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
            if seg_len_sq <= 1e-6:
                continue

            t = -((start_east * seg_dx) + (start_north * seg_dy)) / seg_len_sq
            t = min(1.0, max(0.0, t))
            proj_east = start_east + t * seg_dx
            proj_north = start_north + t * seg_dy
            projection_distance = math.hypot(proj_east, proj_north)
            if projection_distance < best_distance:
                best_distance = projection_distance
                best_projection = {
                    'lat': start_lat + t * (end_lat - start_lat),
                    'lon': start_lon + t * (end_lon - start_lon),
                    'distance_m': projection_distance,
                }

        return best_projection

    def horizontal_sigma_m(self):
        if self.latest_gps_fix is None:
            return float('inf'), False

        covariance = self.latest_gps_fix.position_covariance
        if len(covariance) < 5:
            return float('inf'), False

        var_x = covariance[0]
        var_y = covariance[4]
        if var_x <= 0.0 or var_y <= 0.0:
            return float('inf'), False
        return math.sqrt(max(var_x, var_y)), True

    def evaluate_quality(self):
        ready = False
        snap_distance = float('inf')
        horizontal_sigma, sigma_known = self.horizontal_sigma_m()

        if self.latest_gps_fix is not None and self.graph_segments:
            projection = self.nearest_graph_projection(
                self.latest_gps_fix.latitude,
                self.latest_gps_fix.longitude,
            )
            if projection is not None:
                snap_distance = projection['distance_m']

            if (
                math.isfinite(snap_distance)
                and snap_distance <= self.max_snap_distance_m
                and (
                    (not sigma_known)
                    or (
                        math.isfinite(horizontal_sigma)
                        and horizontal_sigma <= self.max_horizontal_sigma_m
                    )
                )
            ):
                self.good_samples += 1
            else:
                self.good_samples = 0

            ready = self.good_samples >= self.required_good_samples
        else:
            self.good_samples = 0

        metrics_msg = Vector3Stamped()
        metrics_msg.header.stamp = self.get_clock().now().to_msg()
        metrics_msg.header.frame_id = 'map'
        metrics_msg.vector.x = snap_distance if math.isfinite(snap_distance) else -1.0
        metrics_msg.vector.y = (
            horizontal_sigma if sigma_known and math.isfinite(horizontal_sigma) else -1.0
        )
        metrics_msg.vector.z = float(self.good_samples)
        self.metrics_pub.publish(metrics_msg)

        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)

        sigma_text = (
            f'{horizontal_sigma:.1f} m'
            if sigma_known and math.isfinite(horizontal_sigma)
            else 'n/a'
        )
        status_text = (
            f'startup_quality | ready={ready} | '
            f'road_snap={snap_distance:.1f} m | '
            f'gps_sigma={sigma_text} | '
            f'good_samples={self.good_samples}/{self.required_good_samples}'
        )
        status_msg = String()
        status_msg.data = status_text
        self.status_pub.publish(status_msg)

        if ready != self.last_ready:
            log_fn = self.get_logger().info if ready else self.get_logger().warn
            log_fn(status_text)
            self.last_ready = ready
            self.last_status_text = status_text
        elif (
            status_text != self.last_status_text
            and self.good_samples in (0, self.required_good_samples)
        ):
            self.get_logger().info(status_text)
            self.last_status_text = status_text


def main():
    rclpy.init()
    node = StartupQualityMonitor()
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
