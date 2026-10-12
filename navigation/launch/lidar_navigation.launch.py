import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def launch_setup(context):
    # Determine paths
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        navigation_package_path = get_package_share_directory('navigation')
    else:
        slam_package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'slam'))
        navigation_package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'navigation'))

    # Configurations
    sim = LaunchConfiguration('sim', default='false').perform(context)
    map_name = LaunchConfiguration('map', default='lidar_map').perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ.get('HOST', '/')).perform(context)
    
    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'

    # Link to the NEW params file
    params_file_path = os.path.join(navigation_package_path, 'config', 'lidar_nav_param.yaml')

    # 1. Base Robot Launch (Lidar, IMU, etc.) - Explicitly disable cameras
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'usb_cam': 'false',
            'depth_cam': 'false',
            'robot_name': robot_name
        }.items(),
    )

    # 2. Navigation Launch (AMCL, Planner, Controller)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': os.path.join(slam_package_path, 'maps', map_name + '.yaml'),
            'params_file': params_file_path,
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
        }.items(),
    )

    # Grouping with Namespace and 15s Timer
    bringup_group = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            base_launch,
            TimerAction(
                period=15.0,  # Delay to allow Lidar and Odom to start correctly
                actions=[navigation_launch],
            ),
        ]
    )

    return [bringup_group]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('map', default_value='lidar_map'),
        OpaqueFunction(function=launch_setup)
    ])
