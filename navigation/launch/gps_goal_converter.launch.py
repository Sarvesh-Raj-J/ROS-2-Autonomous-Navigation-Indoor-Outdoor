#!/usr/bin/env python3
"""Launch GPS goal converter node"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='gps_goal_converter',
            name='gps_goal_converter',
            output='screen',
        ),
    ])
