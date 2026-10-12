import os
from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    navigation_package_path = get_package_share_directory('navigation')
    rtabmap = LaunchConfiguration('rtabmap')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Localization — standalone nodes (map_server + amcl + lifecycle_manager)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_package_path,
                'launch/include/localization.launch.py'
            )
        ),
        condition=UnlessCondition(PythonExpression(["'", rtabmap, "' == 'true'"])),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
    )

    # Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_package_path,
                'launch/include/navigation_base.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items(),
    )

    bringup_group = GroupAction([
        localization_launch,
        TimerAction(
            period=5.0,
            actions=[navigation_launch],
        ),
    ])

    return [
        stdout_linebuf_envvar,
        bringup_group,
    ]

def generate_launch_description():
    navigation_package_path = get_package_share_directory('navigation')
    return LaunchDescription([
        DeclareLaunchArgument('rtabmap', default_value='false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                navigation_package_path,
                'maps',
                'mini_map.yaml'
            )
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                navigation_package_path,
                'config',
                'nav2_params.yaml'
            )
        ),
        OpaqueFunction(function=launch_setup),
    ])
