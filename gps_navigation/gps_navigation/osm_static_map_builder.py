#!/usr/bin/env python3
"""
Convert OSM XML into a simple occupancy map that can feed Nav2's StaticLayer.

This is the second layer for outdoor navigation:
  - route globally on OSM roads/paths
  - also block buildings/barriers in a static global map
"""

import json
import math
import os

import cv2
from gps_navigation.osm_loader import load_osm_extract
import numpy as np
import rclpy
from rclpy.node import Node


def latlon_delta_m(ref_lat: float, ref_lon: float, lat: float, lon: float):
    meters_per_deg_lat = 111320.0
    lat_rad = math.radians(ref_lat)
    meters_per_deg_lon = 111320.0 * math.cos(lat_rad)
    delta_north = (lat - ref_lat) * meters_per_deg_lat
    delta_east = (lon - ref_lon) * meters_per_deg_lon
    return delta_east, delta_north


def point_in_polygon(x: float, y: float, polygon):
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / max(yj - yi, 1e-9) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


class OSMStaticMapBuilder(Node):
    def __init__(self):
        super().__init__('osm_static_map_builder')

        self.declare_parameter('input_file', '')
        self.declare_parameter('output_stem', '')
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('padding_m', 10.0)
        self.declare_parameter('occupied_buffer_m', 1.0)
        self.declare_parameter('road_buffer_m', 5.0)
        self.declare_parameter('include_occupied_geometry', False)
        self.declare_parameter('unknown_as_occupied', True)
        self.declare_parameter(
            'allowed_highway_types',
            [
                'motorway', 'trunk', 'primary', 'secondary', 'tertiary',
                'unclassified', 'residential', 'service', 'living_street',
                'road', 'track', 'path', 'footway', 'pedestrian', 'cycleway',
            ],
        )

        self.input_file = self.get_parameter('input_file').value
        self.output_stem = self.get_parameter('output_stem').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.padding_m = float(self.get_parameter('padding_m').value)
        self.occupied_buffer_m = float(self.get_parameter('occupied_buffer_m').value)
        self.road_buffer_m = float(self.get_parameter('road_buffer_m').value)
        self.include_occupied_geometry = bool(
            self.get_parameter('include_occupied_geometry').value
        )
        self.unknown_as_occupied = bool(
            self.get_parameter('unknown_as_occupied').value
        )
        self.allowed_highway_types = set(
            self.get_parameter('allowed_highway_types').value
        )

        if not self.input_file or not self.output_stem:
            self.get_logger().error(
                'Set input_file and output_stem parameters to build a static map.'
            )
            return
        if not os.path.isfile(self.input_file):
            self.get_logger().error(f'Input OSM file not found: {self.input_file}')
            return

        self.build_map()

    def parse_osm_geometries(self):
        nodes, ways = load_osm_extract(self.input_file)

        polygons = []
        occupied_polylines = []
        traversable_polylines = []

        for way in ways:
            tags = way['tags']
            refs = [ref for ref in way['refs'] if ref in nodes]
            if len(refs) < 2:
                continue

            is_building = 'building' in tags
            is_wall = tags.get('barrier') in {'wall', 'fence', 'hedge', 'retaining_wall'}
            is_water = tags.get('natural') == 'water'
            is_traversable = tags.get('highway') in self.allowed_highway_types
            is_closed = refs[0] == refs[-1]

            latlons = [nodes[ref] for ref in refs]
            if is_closed and (is_building or is_wall or is_water):
                polygons.append(latlons)
            elif is_wall:
                occupied_polylines.append(latlons)
            elif is_traversable:
                traversable_polylines.append(latlons)

        return polygons, occupied_polylines, traversable_polylines

    def build_map(self):
        polygons, occupied_polylines, traversable_polylines = self.parse_osm_geometries()
        if not polygons and not occupied_polylines and not traversable_polylines:
            self.get_logger().error(
                'No supported occupied or traversable geometry found in the OSM file.'
            )
            return
        if not self.include_occupied_geometry:
            polygons = []
            occupied_polylines = []

        all_latlons = []
        for polygon in polygons:
            all_latlons.extend(polygon)
        for polyline in occupied_polylines:
            all_latlons.extend(polyline)
        for polyline in traversable_polylines:
            all_latlons.extend(polyline)

        min_lat = min(lat for lat, _ in all_latlons)
        min_lon = min(lon for _, lon in all_latlons)

        all_points_m = []
        polygons_m = []
        occupied_polylines_m = []
        traversable_polylines_m = []
        for polygon in polygons:
            poly_m = [latlon_delta_m(min_lat, min_lon, lat, lon) for lat, lon in polygon]
            polygons_m.append(poly_m)
            all_points_m.extend(poly_m)
        for polyline in occupied_polylines:
            line_m = [latlon_delta_m(min_lat, min_lon, lat, lon) for lat, lon in polyline]
            occupied_polylines_m.append(line_m)
            all_points_m.extend(line_m)
        for polyline in traversable_polylines:
            line_m = [latlon_delta_m(min_lat, min_lon, lat, lon) for lat, lon in polyline]
            traversable_polylines_m.append(line_m)
            all_points_m.extend(line_m)

        min_x = min(point[0] for point in all_points_m) - self.padding_m
        max_x = max(point[0] for point in all_points_m) + self.padding_m
        min_y = min(point[1] for point in all_points_m) - self.padding_m
        max_y = max(point[1] for point in all_points_m) + self.padding_m

        width = max(1, int(math.ceil((max_x - min_x) / self.resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / self.resolution)))

        free_value = 254
        occupied_value = 0
        unknown_value = occupied_value if self.unknown_as_occupied else 205
        grid = np.full((height, width), unknown_value, dtype=np.uint8)

        def world_to_grid(x_m: float, y_m: float):
            gx = int((x_m - min_x) / self.resolution)
            gy = int((y_m - min_y) / self.resolution)
            return gx, gy

        def world_to_image_point(x_m: float, y_m: float):
            gx, gy = world_to_grid(x_m, y_m)
            return int(gx), int(height - 1 - gy)

        road_buffer_px = max(1, int(round(self.road_buffer_m / self.resolution)))
        occupied_line_buffer_px = max(
            1, int(round(self.occupied_buffer_m / self.resolution))
        )

        def rasterize_polyline(polyline_m, buffer_px: int, value: int):
            thickness = max(1, int(buffer_px) * 2 + 1)
            for start, end in zip(polyline_m[:-1], polyline_m[1:]):
                cv2.line(
                    grid,
                    world_to_image_point(start[0], start[1]),
                    world_to_image_point(end[0], end[1]),
                    int(value),
                    thickness=thickness,
                    lineType=cv2.LINE_8,
                )

        for polyline_m in traversable_polylines_m:
            rasterize_polyline(polyline_m, road_buffer_px, free_value)

        for polygon in polygons_m:
            poly_points = np.array(
                [world_to_image_point(x_m, y_m) for x_m, y_m in polygon],
                dtype=np.int32,
            )
            if len(poly_points) >= 3:
                cv2.fillPoly(grid, [poly_points], int(occupied_value))

        for polyline_m in occupied_polylines_m:
            rasterize_polyline(polyline_m, occupied_line_buffer_px, occupied_value)

        buffer_px = max(0, int(round(self.occupied_buffer_m / self.resolution)))
        # When the background is occupied, dilating every occupied cell would
        # inflate the entire map and erase the road corridor. In road-only mode
        # the non-road space is already blocked, so no occupied dilation is
        # needed.
        if buffer_px > 0 and not self.unknown_as_occupied:
            kernel_size = buffer_px * 2 + 1
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (kernel_size, kernel_size),
            )
            occupied_mask = np.where(grid == occupied_value, 255, 0).astype(np.uint8)
            dilated_mask = cv2.dilate(occupied_mask, kernel, iterations=1)
            grid[dilated_mask > 0] = occupied_value

        pgm_path = f'{self.output_stem}.pgm'
        yaml_path = f'{self.output_stem}.yaml'
        meta_path = f'{self.output_stem}.geo.json'

        with open(pgm_path, 'wb') as pgm_file:
            pgm_file.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            pgm_file.write(grid.tobytes())

        with open(yaml_path, 'w', encoding='ascii') as yaml_file:
            yaml_file.write(f'image: {os.path.basename(pgm_path)}\n')
            yaml_file.write(f'resolution: {self.resolution}\n')
            yaml_file.write('origin: [0.0, 0.0, 0.0]\n')
            yaml_file.write('negate: 0\n')
            yaml_file.write('occupied_thresh: 0.65\n')
            yaml_file.write('free_thresh: 0.196\n')
            yaml_file.write('mode: trinary\n')

        georef = {
            'reference_lat': min_lat,
            'reference_lon': min_lon,
            'min_x_m': min_x,
            'min_y_m': min_y,
            'resolution': self.resolution,
            'width': width,
            'height': height,
        }
        with open(meta_path, 'w', encoding='ascii') as meta_file:
            json.dump(georef, meta_file, indent=2)

        self.get_logger().info(
            f'Static OSM map written: {pgm_path}, {yaml_path}, {meta_path}'
        )


def main():
    rclpy.init()
    node = OSMStaticMapBuilder()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
