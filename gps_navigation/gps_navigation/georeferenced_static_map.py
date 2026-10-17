#!/usr/bin/env python3
"""
Publish a static occupancy map in the live map frame using GPS + TF alignment.

This keeps the generated OSM occupancy map usable outdoors without hand-editing
its YAML origin each time the robot is started at a new site.
"""

import math
import os
from typing import List, Optional, Tuple

from geometry_msgs.msg import Quaternion
from gps_navigation.osm_loader import load_osm_extract
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
import tf2_ros
import yaml


def latlon_delta_m(ref_lat: float, ref_lon: float, lat: float, lon: float):
    meters_per_deg_lat = 111320.0
    lat_rad = math.radians(ref_lat)
    meters_per_deg_lon = 111320.0 * math.cos(lat_rad)
    delta_north = (lat - ref_lat) * meters_per_deg_lat
    delta_east = (lon - ref_lon) * meters_per_deg_lon
    return delta_east, delta_north


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    quat = Quaternion()
    quat.z = math.sin(half)
    quat.w = math.cos(half)
    return quat


def rotate_enu_into_map(delta_east: float, delta_north: float, map_yaw: float):
    delta_map_x = math.cos(map_yaw) * delta_east - math.sin(map_yaw) * delta_north
    delta_map_y = math.sin(map_yaw) * delta_east + math.cos(map_yaw) * delta_north
    return delta_map_x, delta_map_y


def is_finite_number(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def is_valid_navsat_fix(msg: NavSatFix) -> bool:
    if msg.status.status < 0:
        return False
    return is_finite_number(msg.latitude) and is_finite_number(msg.longitude)


class GeoreferencedStaticMap(Node):
    def __init__(self):
        super().__init__('georeferenced_static_map')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('georef_json', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('samples_required', 8)
        self.declare_parameter('publish_period_sec', 5.0)
        self.declare_parameter('freeze_after_init', True)
        self.declare_parameter('osm_file', '')
        self.declare_parameter('snap_reference_to_graph', False)
        self.declare_parameter('publish_provisional_map', False)
        self.declare_parameter('align_map_yaw_to_utm', False)
        self.declare_parameter('max_snap_distance_m', 20.0)
        self.declare_parameter(
            'allowed_highway_types',
            [
                'motorway', 'trunk', 'primary', 'secondary', 'tertiary',
                'unclassified', 'residential', 'service', 'living_street',
                'road', 'track', 'path', 'footway', 'pedestrian', 'cycleway',
            ],
        )

        self.map_yaml = self.get_parameter('map_yaml').value
        self.georef_json = self.get_parameter('georef_json').value
        self.map_topic = self.get_parameter('map_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.samples_required = max(1, int(self.get_parameter('samples_required').value))
        self.publish_period_sec = max(
            0.5,
            float(self.get_parameter('publish_period_sec').value),
        )
        self.freeze_after_init = bool(self.get_parameter('freeze_after_init').value)
        self.osm_file = self.get_parameter('osm_file').value
        self.snap_reference_to_graph = bool(
            self.get_parameter('snap_reference_to_graph').value
        )
        self.publish_provisional_map = bool(
            self.get_parameter('publish_provisional_map').value
        )
        self.align_map_yaw_to_utm = bool(
            self.get_parameter('align_map_yaw_to_utm').value
        )
        self.max_snap_distance_m = float(self.get_parameter('max_snap_distance_m').value)
        self.allowed_highway_types = set(
            self.get_parameter('allowed_highway_types').value
        )

        if not self.georef_json and self.map_yaml:
            self.georef_json = os.path.splitext(self.map_yaml)[0] + '.geo.json'

        self.latest_gps_fix: Optional[NavSatFix] = None
        self.map_data: Optional[List[int]] = None
        self.map_info = None
        self.reference_lat = None
        self.reference_lon = None
        self.min_x_m = None
        self.min_y_m = None
        self.alignment_samples: List[Tuple[float, float, float]] = []
        self.aligned_origin = None
        self.alignment_complete = False
        self.loaded_once = False
        self.graph_segments = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.create_subscription(NavSatFix, self.gps_topic, self.gps_fix_callback, 10)

        self.timer = self.create_timer(self.publish_period_sec, self.timer_callback)
        self.load_osm_graph_segments()
        self.get_logger().info(
            'Georeferenced static map started: '
            f'map_yaml={self.map_yaml or "n/a"}, georef_json={self.georef_json or "n/a"}, '
            f'align_map_yaw_to_utm={self.align_map_yaw_to_utm}'
        )

    def gps_fix_callback(self, msg: NavSatFix):
        if not is_valid_navsat_fix(msg):
            self.get_logger().warn(
                'Ignoring GPS fix without a valid position',
                throttle_duration_sec=5.0,
            )
            return
        self.latest_gps_fix = msg

    def timer_callback(self):
        if not self.ensure_map_loaded():
            return

        if self.alignment_complete and self.freeze_after_init:
            self.publish_map()
            return

        if not self.alignment_complete:
            sample = self.compute_alignment_sample()
            if sample is None:
                return
            self.alignment_samples.append(sample)
            self.get_logger().info(
                f'Georef alignment sample {len(self.alignment_samples)}/{self.samples_required}: '
                f'origin=({sample[0]:.2f}, {sample[1]:.2f}), yaw={math.degrees(sample[2]):.1f} deg'
            )
            self.aligned_origin = self.average_alignment_samples()
            if len(self.alignment_samples) == 1:
                self.get_logger().info(
                    'Georeferenced static map alignment is converging'
                )
            if len(self.alignment_samples) < self.samples_required:
                if self.publish_provisional_map:
                    self.publish_map()
                return
            self.alignment_complete = True
            self.get_logger().info(
                'Georeferenced static map aligned: '
                f'origin=({self.aligned_origin[0]:.2f}, {self.aligned_origin[1]:.2f}), '
                f'yaw={math.degrees(self.aligned_origin[2]):.1f} deg'
            )

        self.publish_map()
        if self.freeze_after_init and self.alignment_complete:
            self.alignment_samples.clear()

    def load_osm_graph_segments(self):
        self.graph_segments = []
        if not self.osm_file or not os.path.isfile(self.osm_file):
            return

        try:
            raw_nodes, ways = load_osm_extract(self.osm_file)
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse OSM for snapping: {exc}')
            return

        for way in ways:
            tags = way['tags']
            highway_type = tags.get('highway')
            if highway_type not in self.allowed_highway_types:
                continue

            refs = [ref for ref in way['refs'] if ref in raw_nodes]
            if len(refs) < 2:
                continue

            for idx in range(len(refs) - 1):
                start_lat, start_lon = raw_nodes[refs[idx]]
                end_lat, end_lon = raw_nodes[refs[idx + 1]]
                self.graph_segments.append(
                    (start_lat, start_lon, end_lat, end_lon)
                )

        if self.graph_segments:
            self.get_logger().info(
                f'Loaded {len(self.graph_segments)} OSM segments for startup snapping'
            )

    def ensure_map_loaded(self) -> bool:
        if self.map_data is not None and self.map_info is not None:
            return True
        if not self.map_yaml or not self.georef_json:
            self.get_logger().error('Set map_yaml and georef_json parameters.')
            return False
        if not os.path.isfile(self.map_yaml):
            self.get_logger().warn(f'Waiting for static map YAML: {self.map_yaml}')
            return False
        if not os.path.isfile(self.georef_json):
            self.get_logger().warn(f'Waiting for georeference metadata: {self.georef_json}')
            return False

        try:
            with open(self.map_yaml, 'r', encoding='utf-8') as map_yaml_file:
                map_yaml = yaml.safe_load(map_yaml_file)
            with open(self.georef_json, 'r', encoding='utf-8') as georef_file:
                georef = yaml.safe_load(georef_file)
        except Exception as exc:
            self.get_logger().error(f'Failed to load static map metadata: {exc}')
            return False

        image_path = map_yaml.get('image', '')
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(self.map_yaml), image_path)
        if not os.path.isfile(image_path):
            self.get_logger().error(f'Static map image not found: {image_path}')
            return False

        try:
            width, height, pixels, max_value = self.load_pgm(image_path)
        except Exception as exc:
            self.get_logger().error(f'Failed to load static map image: {exc}')
            return False

        resolution = float(map_yaml.get('resolution', georef['resolution']))
        occupied_thresh = float(map_yaml.get('occupied_thresh', 0.65))
        free_thresh = float(map_yaml.get('free_thresh', 0.196))
        negate = int(map_yaml.get('negate', 0))

        occupancy_data: List[int] = []
        for y in range(height):
            src_row = height - 1 - y
            for x in range(width):
                pixel_value = pixels[src_row * width + x]
                occupancy_data.append(
                    self.pixel_to_occupancy(
                        pixel_value,
                        max_value,
                        occupied_thresh,
                        free_thresh,
                        negate,
                    )
                )

        self.map_data = occupancy_data
        self.map_info = {
            'width': width,
            'height': height,
            'resolution': resolution,
        }
        self.reference_lat = float(georef['reference_lat'])
        self.reference_lon = float(georef['reference_lon'])
        self.min_x_m = float(georef['min_x_m'])
        self.min_y_m = float(georef['min_y_m'])

        if not self.loaded_once:
            self.loaded_once = True
            self.get_logger().info(
                'Loaded georeferenced static map: '
                f'{width}x{height} @ {resolution:.2f} m/cell'
            )
        return True

    def pixel_to_occupancy(
        self,
        pixel_value: int,
        max_value: int,
        occupied_thresh: float,
        free_thresh: float,
        negate: int,
    ) -> int:
        normalized = float(pixel_value) / float(max_value)
        occupancy = normalized if negate else 1.0 - normalized
        if occupancy > occupied_thresh:
            return 100
        if occupancy < free_thresh:
            return 0
        return -1

    def load_pgm(self, image_path: str):
        with open(image_path, 'rb') as image_file:
            magic = image_file.readline().strip()
            if magic not in (b'P2', b'P5'):
                raise RuntimeError(f'Unsupported PGM format: {magic!r}')

            tokens: List[bytes] = []
            while len(tokens) < 3:
                line = image_file.readline()
                if not line:
                    raise RuntimeError('Incomplete PGM header')
                stripped = line.strip()
                if not stripped or stripped.startswith(b'#'):
                    continue
                tokens.extend(stripped.split())

            width = int(tokens[0])
            height = int(tokens[1])
            max_value = int(tokens[2])

            if magic == b'P2':
                raw = image_file.read().split()
                pixels = [int(value) for value in raw]
            else:
                pixel_count = width * height
                raw = image_file.read(pixel_count)
                if len(raw) < pixel_count:
                    raise RuntimeError('Incomplete PGM pixel data')
                pixels = list(raw)

        if len(pixels) != width * height:
            raise RuntimeError(
                f'Unexpected PGM pixel count: expected {width * height}, got {len(pixels)}'
            )
        return width, height, pixels, max_value

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

    def compute_alignment_sample(self):
        if self.latest_gps_fix is None:
            self.get_logger().warn(
                'Waiting for GPS fix before aligning static map',
                throttle_duration_sec=5.0,
            )
            return None

        if not is_valid_navsat_fix(self.latest_gps_fix):
            self.get_logger().warn(
                'GPS fix is not valid yet; skipping static map alignment',
                throttle_duration_sec=5.0,
            )
            return None

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            utm_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.utm_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().warn(f'Waiting for TF before aligning static map: {exc}')
            return None

        transform_values = [
            robot_tf.transform.translation.x,
            robot_tf.transform.translation.y,
            robot_tf.transform.translation.z,
            robot_tf.transform.rotation.x,
            robot_tf.transform.rotation.y,
            robot_tf.transform.rotation.z,
            robot_tf.transform.rotation.w,
            utm_tf.transform.translation.x,
            utm_tf.transform.translation.y,
            utm_tf.transform.translation.z,
            utm_tf.transform.rotation.x,
            utm_tf.transform.rotation.y,
            utm_tf.transform.rotation.z,
            utm_tf.transform.rotation.w,
        ]
        if not all(is_finite_number(value) for value in transform_values):
            self.get_logger().warn('TF contains non-finite values; skipping alignment sample')
            return None

        utm_map_yaw = quaternion_to_yaw(
            utm_tf.transform.rotation.x,
            utm_tf.transform.rotation.y,
            utm_tf.transform.rotation.z,
            utm_tf.transform.rotation.w,
        )
        if not is_finite_number(utm_map_yaw):
            self.get_logger().warn('Computed map yaw is invalid; skipping alignment sample')
            return None
        map_yaw = utm_map_yaw if self.align_map_yaw_to_utm else 0.0

        reference_lat = self.latest_gps_fix.latitude
        reference_lon = self.latest_gps_fix.longitude
        snap_distance = None
        if self.snap_reference_to_graph and self.graph_segments:
            snapped = self.nearest_graph_projection(reference_lat, reference_lon)
            if (
                snapped is not None
                and snapped['distance_m'] <= self.max_snap_distance_m
            ):
                reference_lat = snapped['lat']
                reference_lon = snapped['lon']
                snap_distance = snapped['distance_m']

        ref_delta_east, ref_delta_north = latlon_delta_m(
            reference_lat,
            reference_lon,
            self.reference_lat,
            self.reference_lon,
        )
        ref_map_dx, ref_map_dy = rotate_enu_into_map(
            ref_delta_east,
            ref_delta_north,
            map_yaw,
        )
        ref_map_x = robot_tf.transform.translation.x + ref_map_dx
        ref_map_y = robot_tf.transform.translation.y + ref_map_dy

        origin_dx, origin_dy = rotate_enu_into_map(
            self.min_x_m,
            self.min_y_m,
            map_yaw,
        )
        origin_x = ref_map_x + origin_dx
        origin_y = ref_map_y + origin_dy
        sample = (origin_x, origin_y, map_yaw)
        if not all(is_finite_number(value) for value in sample):
            self.get_logger().warn('Computed georef sample has invalid values; skipping')
            return None
        if snap_distance is not None:
            self.get_logger().info(
                f'Georef startup snap used nearest OSM segment at {snap_distance:.1f} m'
            )
        return sample

    def average_alignment_samples(self):
        valid_samples = [
            sample for sample in self.alignment_samples
            if all(is_finite_number(value) for value in sample)
        ]
        if not valid_samples:
            raise RuntimeError('No valid alignment samples available')
        avg_x = sum(sample[0] for sample in valid_samples) / len(valid_samples)
        avg_y = sum(sample[1] for sample in valid_samples) / len(valid_samples)
        avg_sin = sum(math.sin(sample[2]) for sample in valid_samples)
        avg_cos = sum(math.cos(sample[2]) for sample in valid_samples)
        avg_yaw = math.atan2(avg_sin, avg_cos)
        averaged = (avg_x, avg_y, avg_yaw)
        if not all(is_finite_number(value) for value in averaged):
            raise RuntimeError('Averaged alignment sample contains invalid values')
        return averaged

    def publish_map(self):
        if self.aligned_origin is None or self.map_data is None or self.map_info is None:
            return
        if not all(is_finite_number(value) for value in self.aligned_origin):
            self.get_logger().warn('Aligned origin is invalid; refusing to publish /map')
            return

        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = self.map_frame or 'map'
        occupancy_grid.info.map_load_time = occupancy_grid.header.stamp
        occupancy_grid.info.resolution = float(self.map_info['resolution'])
        occupancy_grid.info.width = int(self.map_info['width'])
        occupancy_grid.info.height = int(self.map_info['height'])
        occupancy_grid.info.origin.position.x = float(self.aligned_origin[0])
        occupancy_grid.info.origin.position.y = float(self.aligned_origin[1])
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation = quaternion_from_yaw(self.aligned_origin[2])
        occupancy_grid.data = self.map_data
        self.map_pub.publish(occupancy_grid)


def main():
    rclpy.init()
    node = GeoreferencedStaticMap()
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
