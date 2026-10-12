import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_package_path(package_name: str, source_path: str, compiled_flag: str) -> str:
    if compiled_flag == 'True':
        try:
            return get_package_share_directory(package_name)
        except Exception:
            return source_path
    return source_path


def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')

    slam_package_path = resolve_package_path(
        'slam',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'slam')),
        compiled,
    )
    peripherals_package_path = resolve_package_path(
        'peripherals',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'peripherals')),
        compiled,
    )

    sim = LaunchConfiguration('sim').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    master_name = LaunchConfiguration('master_name').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    use_depth_camera = LaunchConfiguration('use_depth_camera').perform(context)
    gps_port = LaunchConfiguration('gps_port').perform(context)
    imu_port = LaunchConfiguration('imu_port').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    ekf_file = LaunchConfiguration('ekf_file').perform(context)
    depth_camera_serial_number = LaunchConfiguration('depth_camera_serial_number').perform(context)
    depth_camera_device_num = LaunchConfiguration('depth_camera_device_num').perform(context)
    depth_camera_enable_color = LaunchConfiguration('depth_camera_enable_color').perform(context)
    navsat_use_odometry_yaw = LaunchConfiguration('navsat_use_odometry_yaw')
    navsat_yaw_offset = LaunchConfiguration('navsat_yaw_offset')
    navsat_magnetic_declination = LaunchConfiguration('navsat_magnetic_declination')
    gps_parent_frame = LaunchConfiguration('gps_parent_frame').perform(context)
    gps_frame = LaunchConfiguration('gps_frame').perform(context)
    gps_x = LaunchConfiguration('gps_x').perform(context)
    gps_y = LaunchConfiguration('gps_y').perform(context)
    gps_z = LaunchConfiguration('gps_z').perform(context)
    gps_roll = LaunchConfiguration('gps_roll').perform(context)
    gps_pitch = LaunchConfiguration('gps_pitch').perform(context)
    gps_yaw = LaunchConfiguration('gps_yaw').perform(context)
    gps_imu_parent_frame = LaunchConfiguration('gps_imu_parent_frame').perform(context)
    gps_imu_frame = LaunchConfiguration('gps_imu_frame').perform(context)
    gps_imu_x = LaunchConfiguration('gps_imu_x').perform(context)
    gps_imu_y = LaunchConfiguration('gps_imu_y').perform(context)
    gps_imu_z = LaunchConfiguration('gps_imu_z').perform(context)
    gps_imu_roll = LaunchConfiguration('gps_imu_roll').perform(context)
    gps_imu_pitch = LaunchConfiguration('gps_imu_pitch').perform(context)
    gps_imu_yaw = LaunchConfiguration('gps_imu_yaw').perform(context)
    gps_imu_invert_yaw = LaunchConfiguration('gps_imu_invert_yaw').perform(context)
    gps_imu_yaw_offset = LaunchConfiguration('gps_imu_yaw_offset').perform(context)

    use_sim_time = sim == 'true'

    if not os.path.isfile(params_file):
        raise RuntimeError(f'[outdoor_navigation] nav2 params not found: {params_file}')
    if not os.path.isfile(ekf_file):
        raise RuntimeError(f'[outdoor_navigation] ekf params not found: {ekf_file}')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')
        ),
        launch_arguments={
            'sim': sim,
            'use_joy': 'false',
            'use_lidar': use_lidar,
            'use_depth_camera': use_depth_camera,
            'enable_odom': 'false',
            'depth_camera_name': 'depth_cam',
            'depth_camera_serial_number': depth_camera_serial_number,
            'depth_camera_device_num': depth_camera_device_num,
            'depth_camera_enable_point_cloud': use_depth_camera,
            'depth_camera_enable_color': depth_camera_enable_color,
            'enable_usb_cam': 'false',
            'enable_depth_odom': 'false',
            'master_name': master_name,
            'robot_name': robot_name,
        }.items(),
    )

    # The robot description provides depth_cam_link, but outdoor navigation
    # consumes the point cloud in depth_cam_depth_optical_frame.
    depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_optical_tf',
        output='screen',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'depth_cam_link', 'depth_cam_depth_optical_frame'],
    )

    # navsat_transform requires the NavSatFix frame to exist in TF so it can
    # remove the GPS antenna offset from the robot origin.
    gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf',
        output='screen',
        arguments=[
            '--x', gps_x,
            '--y', gps_y,
            '--z', gps_z,
            '--roll', gps_roll,
            '--pitch', gps_pitch,
            '--yaw', gps_yaw,
            '--frame-id', gps_parent_frame,
            '--child-frame-id', gps_frame,
        ],
    )

    gps_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_imu_tf',
        output='screen',
        arguments=[
            '--x', gps_imu_x,
            '--y', gps_imu_y,
            '--z', gps_imu_z,
            '--roll', gps_imu_roll,
            '--pitch', gps_imu_pitch,
            '--yaw', gps_imu_yaw,
            '--frame-id', gps_imu_parent_frame,
            '--child-frame-id', gps_imu_frame,
        ],
    )

    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        output='screen',
        parameters=[{
            'port': gps_port,
            'baud': 9600,
            'frame_id': gps_frame,
            'time_ref_source': 'gps',
            'useRMC': False,
        }],
        remappings=[
            ('fix', '/gps/fix'),
        ],
    )

    gps_imu_driver = Node(
        package='im10a_driver',
        executable='im10a_node',
        name='gps_imu_driver',
        output='screen',
        parameters=[{
            'port': imu_port,
            'baud_rate': 9600,
            'frame_id': gps_imu_frame,
            # IM10A heading commonly needs conversion into ROS ENU before it
            # can be trusted by navsat_transform for earth-aligned map yaw.
            'invert_yaw': gps_imu_invert_yaw.lower() == 'true',
            'yaw_offset': float(gps_imu_yaw_offset),
        }],
    )

    localization_stack = GroupAction(actions=[
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/global'),
            ],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                ekf_file,
                {
                    'use_sim_time': use_sim_time,
                    # Expose heading alignment controls at launch time so we
                    # can tune map-vs-earth rotation without editing YAML.
                    'use_odometry_yaw': ParameterValue(
                        navsat_use_odometry_yaw,
                        value_type=bool,
                    ),
                    'yaw_offset': ParameterValue(
                        navsat_yaw_offset,
                        value_type=float,
                    ),
                    'magnetic_declination_radians': ParameterValue(
                        navsat_magnetic_declination,
                        value_type=float,
                    ),
                },
            ],
            remappings=[
                ('imu/data', '/gps/imu'),
                ('gps/fix', '/gps/fix'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),
        Node(
            package='navigation',
            executable='sensor_fallback_monitor',
            name='sensor_fallback_monitor',
            output='screen',
        ),
        Node(
            package='navigation',
            executable='utm_goal_bridge',
            name='utm_goal_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/goal_pose_utm',
                'output_topic': '/goal_pose',
                'input_frame': 'utm',
                'map_frame': 'map',
                'send_to_nav2': True,
            }],
        ),
    ])

    shared_params = [params_file, {'use_sim_time': use_sim_time}]

    nav2_delayed = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=shared_params,
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=shared_params,
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=shared_params,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=shared_params,
                remappings=[('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=shared_params,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=shared_params,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=shared_params,
                remappings=[
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel'),
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'bond_timeout': 30.0,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'smoother_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ],
                }],
            ),
        ],
    )

    full_stack = GroupAction(actions=[
        base_launch,
        depth_optical_tf,
        gps_tf,
        gps_imu_tf,
        gps_driver,
        gps_imu_driver,
        localization_stack,
        nav2_delayed,
    ])

    return [full_stack]


def generate_launch_description():
    compiled = os.environ.get('need_compile', 'False')
    navigation_package_path = resolve_package_path(
        'navigation',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'navigation')),
        compiled,
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('robot_name', default_value='/'),
        DeclareLaunchArgument(
            'master_name',
            default_value=os.environ.get('MASTER', 'master')
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Enable LiDAR obstacle sensing for outdoor navigation',
        ),
        DeclareLaunchArgument(
            'use_depth_camera',
            default_value='false',
            description='Enable depth camera obstacle sensing; usually leave off outdoors',
        ),
        DeclareLaunchArgument(
            'gps_port',
            default_value='/dev/gps',
            description='Serial port for the NMEA GPS module',
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/imu',
            description='Serial port for the IM10A GPS companion IMU',
        ),
        DeclareLaunchArgument(
            'gps_parent_frame',
            default_value='base_link',
            description='Parent TF frame for the GPS antenna frame',
        ),
        DeclareLaunchArgument(
            'gps_frame',
            default_value='gps_link',
            description='TF frame published by the GPS driver',
        ),
        DeclareLaunchArgument('gps_x', default_value='0.0'),
        DeclareLaunchArgument('gps_y', default_value='0.0'),
        DeclareLaunchArgument('gps_z', default_value='0.0'),
        DeclareLaunchArgument('gps_roll', default_value='0.0'),
        DeclareLaunchArgument('gps_pitch', default_value='0.0'),
        DeclareLaunchArgument('gps_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'gps_imu_parent_frame',
            default_value='base_link',
            description='Parent TF frame for the GPS companion IMU frame',
        ),
        DeclareLaunchArgument(
            'gps_imu_frame',
            default_value='gps_imu_link',
            description='TF frame published by the GPS companion IMU driver',
        ),
        DeclareLaunchArgument('gps_imu_x', default_value='0.0'),
        DeclareLaunchArgument('gps_imu_y', default_value='0.0'),
        DeclareLaunchArgument('gps_imu_z', default_value='0.0'),
        DeclareLaunchArgument('gps_imu_roll', default_value='0.0'),
        DeclareLaunchArgument('gps_imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('gps_imu_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'gps_imu_invert_yaw',
            default_value='true',
            description='Invert IM10A yaw before publishing /gps/imu to convert heading sign into ROS ENU',
        ),
        DeclareLaunchArgument(
            'gps_imu_yaw_offset',
            default_value='1.5708',
            description='Additional IM10A yaw correction in radians; pi/2 maps north-referenced heading to ROS east-zero yaw',
        ),
        DeclareLaunchArgument(
            'depth_camera_serial_number',
            default_value=os.environ.get('DEPTH_CAMERA_SERIAL_NUMBER', ''),
            description='Optional depth camera serial number',
        ),
        DeclareLaunchArgument(
            'depth_camera_device_num',
            default_value='1',
            description='Depth camera index when no serial number is provided',
        ),
        DeclareLaunchArgument(
            'depth_camera_enable_color',
            default_value='false',
            description='Enable Orbbec RGB stream for debugging or visualization',
        ),
        DeclareLaunchArgument(
            'navsat_use_odometry_yaw',
            default_value='true',
            description='Use local odometry yaw for navsat_transform instead of GPS IMU heading',
        ),
        DeclareLaunchArgument(
            'navsat_yaw_offset',
            default_value='0.0',
            description='Additional yaw offset in radians applied inside navsat_transform',
        ),
        DeclareLaunchArgument(
            'navsat_magnetic_declination',
            default_value='0.04',
            description='Magnetic declination in radians for navsat_transform',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                navigation_package_path,
                'config',
                'nav2_outdoor_params.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'ekf_file',
            default_value=os.path.join(
                navigation_package_path,
                'config',
                'ekf_outdoor.yaml',
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
