import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def launch_setup(context, *args, **kwargs):
    navigation_package_path = get_package_share_directory('navigation')
    slam_package_path = get_package_share_directory('slam')
    map_yaml = LaunchConfiguration('map').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    rtabmap = LaunchConfiguration('rtabmap').perform(context)
    point_cloud_topic = LaunchConfiguration('point_cloud_topic').perform(context)

    # Launch robot.launch.py for robot control and TFs
    # Set depth camera launch args to match standalone launch
    depth_camera_name = 'depth_cam'
    serial_number = LaunchConfiguration('depth_camera_serial_number').perform(context)
    device_num = LaunchConfiguration('depth_camera_device_num').perform(context)
    enable_usb_cam = 'true' if rtabmap.lower() == 'true' else 'false'

    # Keep the point cloud topic used by AMCL scan conversion and Nav2 costmaps in sync.
    params_file = ReplaceString(
        source_file=os.path.join(navigation_package_path, 'config', 'nav2_3d_params.yaml'),
        replacements={
            '/depth_cam/depth/points': point_cloud_topic,
        },
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')
        ),
        launch_arguments={
            'sim': 'false',
            'use_joy': 'false',
            'use_lidar': 'false',
            'use_depth_camera': 'true',
            'robot_name': robot_name,
            'depth_camera_name': depth_camera_name,
            'depth_camera_serial_number': serial_number,
            'depth_camera_device_num': device_num,
            'depth_camera_enable_color': 'false',
            'depth_camera_enable_point_cloud': 'true',
            'depth_registration': 'false',
            'enable_usb_cam': enable_usb_cam,
            'enable_depth_odom': 'false',
        }.items(),
    )

    # Point cloud to laser scan converter for AMCL.
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'depth_cam_depth_optical_frame',
            # AMCL won't publish map->odom until it gets a usable /scan, so
            # keep this conversion tolerant to TF startup jitter.
            'transform_tolerance': 0.3,
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', point_cloud_topic),
            ('scan', '/scan'),
        ],
    )

    # The depth-only AMCL path doesn't launch RTAB-Map's static optical-frame TF,
    # so publish the camera optical frame needed by pointcloud_to_laserscan.
    depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_optical_tf',
        output='screen',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'depth_cam_link', 'depth_cam_depth_optical_frame'],
    )

    # Launch Nav2 with nav2_3d_params.yaml
    # Launch RTABMAP for localization
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/rtabmap.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('rtabmap')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'autostart': 'true',
            'rtabmap': rtabmap,
        }.items(),
    )

    return [
        robot_launch,
        pointcloud_to_laserscan_node,
        depth_optical_tf,
        rtabmap_launch,
        TimerAction(
            period=10.0,
            actions=[nav2_bringup],
        ),
    ]


def generate_launch_description():
    slam_package_path = get_package_share_directory('slam')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(slam_package_path, 'maps', 'lidar_map.yaml'),
        ),
        DeclareLaunchArgument('robot_name', default_value='/'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rtabmap', default_value='true'),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/depth_cam/depth/points',
        ),
        DeclareLaunchArgument('depth_camera_serial_number', default_value=os.getenv('DEPTH_CAMERA_SERIAL_NUMBER', '')),
        DeclareLaunchArgument('depth_camera_device_num', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])
