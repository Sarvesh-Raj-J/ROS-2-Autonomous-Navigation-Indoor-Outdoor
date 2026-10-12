import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    map_yaml_file = LaunchConfiguration('map').perform(context)
    use_sim_time  = LaunchConfiguration('use_sim_time').perform(context)
    autostart     = LaunchConfiguration('autostart').perform(context)
    params_file   = LaunchConfiguration('params_file').perform(context)

    use_sim_time_bool = use_sim_time.lower() == 'true'
    autostart_bool    = autostart.lower() == 'true'
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time_bool, 'yaml_filename': map_yaml_file}],
            remappings=remappings,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time_bool}],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_bool,
                'autostart': autostart_bool,
                'node_names': ['map_server', 'amcl'],
                'bond_timeout': 30.0,
            }],
        ),
    ]


def generate_launch_description():
    navigation_package_path = get_package_share_directory('navigation')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_package_path, 'config', 'nav2_params.yaml')
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
