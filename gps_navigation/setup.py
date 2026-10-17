from glob import glob
import os

from setuptools import setup

package_name = 'gps_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob(os.path.join('behavior_trees', '*.xml'))),
        (os.path.join('share', package_name, 'docs'),
            glob(os.path.join('docs', '*.md'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sibikarthikeyan',
    maintainer_email='sibi07sk1105@gmail.com',
    description=(
        'Generic outdoor GPS navigation for ROS 2 Nav2: GPS + heading IMU + wheel '
        'odometry fusion, OpenStreetMap road routing and a georeferenced static map.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_lidar_safety_filter=gps_navigation.cmd_vel_lidar_safety_filter:main',
            'georeferenced_static_map=gps_navigation.georeferenced_static_map:main',
            'gps_odom_covariance_filter=gps_navigation.gps_odom_covariance_filter:main',
            'imu_yaw_offsetter=gps_navigation.imu_yaw_offsetter:main',
            'map_ready_waiter=gps_navigation.map_ready_waiter:main',
            'osm_route_planner=gps_navigation.osm_route_planner:main',
            'osm_static_map_builder=gps_navigation.osm_static_map_builder:main',
            'outdoor_nav_health_monitor=gps_navigation.outdoor_nav_health_monitor:main',
            'sensor_fallback_monitor=gps_navigation.sensor_fallback_monitor:main',
            'startup_quality_monitor=gps_navigation.startup_quality_monitor:main',
            'utm_goal_bridge=gps_navigation.utm_goal_bridge:main',
            'yaw_debug_monitor=gps_navigation.yaw_debug_monitor:main',
        ],
    },
)
