"""
Outdoor GPS navigation entry point (GPS + heading IMU + wheel odometry + LiDAR).

Brings up the full outdoor stack on top of YOUR robot:
  * GPS localization: heading corrector -> ekf_local (odom) + ekf_global (map)
    + navsat_transform, with a GPS covariance floor so non-RTK GPS cannot
    overpower odometry.
  * Optional OpenStreetMap layer: the OSM file is rasterized into a road-corridor
    static map, georeferenced to the live map frame from GPS, and goals are
    routed along the road graph ("Google Maps" global layer, Nav2 local layer).
  * Nav2 servers tuned for outdoor driving, with a LiDAR cmd_vel safety filter.

This launch does NOT start your drivers. Your robot must publish (names are
launch arguments): a NavSatFix, an absolute-heading IMU, wheel odometry WITHOUT
the odom->base TF (ekf_local owns it here), a base IMU, a LaserScan, the URDF TF,
and subscribe to /cmd_vel. See README.md and docs/ROBOT_SETUP.md.

Typical commands
    ros2 launch gps_navigation gps_navigation.launch.py
    ros2 launch gps_navigation gps_navigation.launch.py osm_file:=/abs/site.osm.pbf
    ros2 launch gps_navigation gps_navigation.launch.py \
        use_nmea_driver:=true gps_port:=/dev/ttyUSB0 launch_rviz:=true
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from gps_navigation.launch_utils import render_nav2_params
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'gps_navigation'
HEADING_TOPIC = '/gps/imu_heading'


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _flag(context, name):
    return _arg(context, name).lower() == 'true'


def _float(context, name):
    return float(_arg(context, name))


def _int(context, name):
    return int(_arg(context, name))


def launch_setup(context):
    pkg = get_package_share_directory(PACKAGE)
    use_sim_time = _flag(context, 'use_sim_time')
    base_frame = _arg(context, 'base_frame')
    gps_fix_topic = _arg(context, 'gps_fix_topic')
    heading_imu_topic = _arg(context, 'heading_imu_topic')
    wheel_odom_topic = _arg(context, 'wheel_odom_topic')
    base_imu_topic = _arg(context, 'base_imu_topic')
    scan_topic = _arg(context, 'scan_topic')
    backup_scan_topic = _arg(context, 'backup_scan_topic')
    params_file = _arg(context, 'params_file')
    ekf_file = _arg(context, 'ekf_file')
    osm_file = _arg(context, 'osm_file')
    static_map_yaml = _arg(context, 'static_map_yaml')
    base_launch_file = _arg(context, 'base_launch_file')
    launch_rviz = _flag(context, 'launch_rviz')
    rviz_config = _arg(context, 'rviz_config')
    use_safety_filter = _flag(context, 'use_safety_filter')
    route_rviz_goals_on_osm = _flag(context, 'route_rviz_goals_on_osm')
    nav2_start_delay_sec = max(0.0, _float(context, 'nav2_start_delay_sec'))
    magnetic_declination = _float(context, 'magnetic_declination')

    if not os.path.isfile(params_file):
        raise RuntimeError(f'[gps_navigation] nav2 params not found: {params_file}')
    if not os.path.isfile(ekf_file):
        raise RuntimeError(f'[gps_navigation] ekf params not found: {ekf_file}')
    if osm_file and not os.path.isfile(osm_file):
        raise RuntimeError(f'[gps_navigation] OSM file not found: {osm_file}')
    if base_launch_file and not os.path.isfile(base_launch_file):
        raise RuntimeError(f'[gps_navigation] base_launch_file not found: {base_launch_file}')
    if launch_rviz and not os.path.isfile(rviz_config):
        raise RuntimeError(f'[gps_navigation] RViz config not found: {rviz_config}')

    # ---------------------------------------------------------------- static map
    # Three cases: no static map (rolling global costmap), a hand-made map
    # served by map_server, or a map generated from the OSM file and anchored
    # to the live map frame by georeferenced_static_map.
    static_map_generated = False
    static_map_output_stem = ''
    if not static_map_yaml and _flag(context, 'auto_build_static_map') and osm_file:
        map_basename = os.path.splitext(os.path.basename(osm_file))[0] or 'osm_site'
        safe_map_basename = ''.join(
            char if char.isalnum() or char in ('-', '_') else '_'
            for char in map_basename
        )
        geometry_suffix = (
            'full' if _flag(context, 'osm_static_map_include_buildings') else 'road'
        )
        static_map_output_stem = os.path.join(
            tempfile.gettempdir(),
            f'{safe_map_basename}_nav2_{geometry_suffix}_{os.getpid()}',
        )
        static_map_yaml = f'{static_map_output_stem}.yaml'
        static_map_generated = True

    static_map_georef = (
        os.path.splitext(static_map_yaml)[0] + '.geo.json' if static_map_yaml else ''
    )
    use_georeferenced_static_map = bool(static_map_yaml) and (
        static_map_generated or os.path.isfile(static_map_georef)
    )
    use_map_server = bool(static_map_yaml) and not use_georeferenced_static_map
    if static_map_yaml:
        static_params_file = os.path.join(pkg, 'config', 'nav2_params_gps_static.yaml')
        if (
            os.path.basename(params_file) == 'nav2_params_gps.yaml'
            and os.path.isfile(static_params_file)
        ):
            params_file = static_params_file
        if not static_map_generated and not os.path.isfile(static_map_yaml):
            raise RuntimeError(f'[gps_navigation] static map yaml not found: {static_map_yaml}')

    params_file = render_nav2_params(
        params_file,
        bt_dir=os.path.join(pkg, 'behavior_trees'),
        base_frame=base_frame,
        scan_topic=scan_topic,
        backup_scan_topic=backup_scan_topic,
    )

    # ------------------------------------------------------------ robot + sensors
    robot_actions = []
    if base_launch_file:
        robot_actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch_file))
        )

    if _flag(context, 'publish_gps_tf'):
        # navsat_transform needs the antenna frame in TF to remove the lever arm
        # between the GPS antenna and the robot origin.
        robot_actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_tf',
            output='screen',
            arguments=[
                '--x', _arg(context, 'gps_x'),
                '--y', _arg(context, 'gps_y'),
                '--z', _arg(context, 'gps_z'),
                '--roll', _arg(context, 'gps_roll'),
                '--pitch', _arg(context, 'gps_pitch'),
                '--yaw', _arg(context, 'gps_yaw'),
                '--frame-id', _arg(context, 'gps_parent_frame'),
                '--child-frame-id', _arg(context, 'gps_frame'),
            ],
        ))
        robot_actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='heading_imu_tf',
            output='screen',
            arguments=[
                '--x', _arg(context, 'heading_imu_x'),
                '--y', _arg(context, 'heading_imu_y'),
                '--z', _arg(context, 'heading_imu_z'),
                '--roll', _arg(context, 'heading_imu_roll'),
                '--pitch', _arg(context, 'heading_imu_pitch'),
                '--yaw', _arg(context, 'heading_imu_yaw'),
                '--frame-id', _arg(context, 'heading_imu_parent_frame'),
                '--child-frame-id', _arg(context, 'heading_imu_frame'),
            ],
        ))

    if _flag(context, 'use_nmea_driver'):
        robot_actions.append(Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver',
            output='screen',
            parameters=[{
                'port': _arg(context, 'gps_port'),
                'baud': _int(context, 'gps_baud'),
                'frame_id': _arg(context, 'gps_frame'),
                'time_ref_source': 'gps',
                'useRMC': False,
            }],
            remappings=[('fix', gps_fix_topic)],
        ))

    # --------------------------------------------------------------- localization
    ekf_overrides = {
        'use_sim_time': use_sim_time,
        'base_link_frame': base_frame,
        'odom0': wheel_odom_topic,
        'imu0': base_imu_topic,
        'imu1': HEADING_TOPIC,
    }
    gps_odom_min_horizontal_sigma_m = _float(context, 'gps_odom_min_horizontal_sigma_m')
    gps_odom_min_vertical_sigma_m = _float(context, 'gps_odom_min_vertical_sigma_m')
    max_graph_snap_distance_m = _float(context, 'max_graph_snap_distance_m')

    localization_nodes = [
        Node(
            package=PACKAGE,
            executable='imu_yaw_offsetter',
            name='gps_imu_heading_corrector',
            output='screen',
            parameters=[{
                'input_topic': heading_imu_topic,
                'output_topic': HEADING_TOPIC,
                'invert_yaw': _flag(context, 'heading_invert_yaw'),
                # Robot-heading correction for a non-standard IMU mount, plus
                # the site's magnetic declination.
                'yaw_offset': _float(context, 'heading_yaw_offset') + magnetic_declination,
                'frame_id': base_frame,
                'drop_zero_stamp': True,
            }],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_file, ekf_overrides],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_file, ekf_overrides],
            remappings=[('odometry/filtered', '/odometry/global')],
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
                    'use_odometry_yaw': _flag(context, 'navsat_use_odometry_yaw'),
                    'yaw_offset': _float(context, 'navsat_yaw_offset'),
                    'magnetic_declination_radians': magnetic_declination,
                },
            ],
            # navsat consumes the raw compass heading so UTM, map and OSM stay
            # earth-aligned; the robot-heading correction lives on HEADING_TOPIC.
            remappings=[
                ('imu/data', heading_imu_topic),
                ('gps/fix', gps_fix_topic),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps_raw'),
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),
        Node(
            package=PACKAGE,
            executable='gps_odom_covariance_filter',
            name='gps_odom_covariance_filter',
            output='screen',
            parameters=[{
                'input_topic': '/odometry/gps_raw',
                'output_topic': '/odometry/gps',
                'min_horizontal_variance_m2': gps_odom_min_horizontal_sigma_m ** 2,
                'min_vertical_variance_m2': gps_odom_min_vertical_sigma_m ** 2,
                'publish_period_sec': 2.0,
            }],
        ),
        Node(
            package=PACKAGE,
            executable='sensor_fallback_monitor',
            name='sensor_fallback_monitor',
            output='screen',
            parameters=[{'scan_topic': scan_topic}],
        ),
        Node(
            package=PACKAGE,
            executable='startup_quality_monitor',
            name='startup_quality_monitor',
            output='screen',
            parameters=[{
                'osm_file': osm_file,
                'gps_topic': gps_fix_topic,
                'max_snap_distance_m': max_graph_snap_distance_m,
                'max_horizontal_sigma_m': _float(
                    context, 'startup_quality_max_horizontal_sigma_m'),
                'required_good_samples': _int(context, 'startup_quality_required_good_samples'),
                'publish_period_sec': _float(context, 'startup_quality_publish_period_sec'),
            }],
        ),
        Node(
            package=PACKAGE,
            executable='yaw_debug_monitor',
            name='yaw_debug_monitor',
            output='screen',
            parameters=[{
                'gps_imu_raw_topic': heading_imu_topic,
                'gps_imu_topic': HEADING_TOPIC,
                'base_imu_topic': base_imu_topic,
                'odom_topic': '/odometry/filtered',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': base_frame,
                'utm_frame': 'utm',
                'report_period': 1.0,
                'log_to_console': False,
            }],
        ),
        Node(
            package=PACKAGE,
            executable='outdoor_nav_health_monitor',
            name='outdoor_nav_health_monitor',
            output='screen',
            parameters=[{
                'gps_fix_topic': gps_fix_topic,
                'gps_imu_compass_topic': heading_imu_topic,
                'gps_imu_heading_topic': HEADING_TOPIC,
                'base_imu_topic': base_imu_topic,
                'raw_odom_topic': wheel_odom_topic,
                'report_period_sec': 1.0,
            }],
        ),
        Node(
            package=PACKAGE,
            executable='utm_goal_bridge',
            name='utm_goal_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/goal_pose_utm',
                'output_topic': '/goal_pose',
                'input_frame': 'utm',
                'map_frame': 'map',
                'send_to_nav2': not bool(osm_file),
            }],
        ),
        Node(
            package=PACKAGE,
            executable='osm_route_planner',
            name='osm_route_planner',
            output='screen',
            parameters=[{
                'osm_file': osm_file,
                'gps_topic': gps_fix_topic,
                'gps_goal_topic': '/gps_goal',
                'map_frame': 'map',
                'base_frame': base_frame,
                'utm_frame': 'utm',
                'map_topic': '/map',
                'georef_json': static_map_georef if use_georeferenced_static_map else '',
                'waypoint_spacing_m': _float(context, 'osm_waypoint_spacing_m'),
                'max_nav2_waypoints': _int(context, 'osm_nav2_max_waypoints'),
                'nav2_simplify_tolerance_m': _float(context, 'osm_nav2_simplify_tolerance_m'),
                'send_to_nav2': True,
                'route_topic': '/osm_route_plan',
                'route_marker_array_topic': '/osm_route_markers',
                'graph_marker_array_topic': '/osm_graph_markers',
                'goal_topic': '/goal_pose',
                'accept_rviz_goals': route_rviz_goals_on_osm and bool(osm_file),
                'append_exact_goal': True,
                'prepend_snapped_start': True,
                'snap_start_to_graph': True,
                'snap_goal_to_graph': True,
                'max_snap_distance_m': max_graph_snap_distance_m,
                'snap_heading_weight_m': _float(context, 'snap_heading_weight_m'),
                'prune_start_behind_robot': _flag(context, 'prune_osm_start_behind_robot'),
                'start_prune_distance_m': _float(context, 'osm_start_prune_distance_m'),
                'short_goal_direct_distance_m': _float(context, 'short_goal_direct_distance_m'),
                'respect_rviz_goal_orientation': _flag(context, 'respect_rviz_goal_orientation'),
                'nav2_action_mode': _arg(context, 'nav2_action_mode'),
                'graph_display_radius_m': _float(context, 'osm_graph_display_radius_m'),
                'follow_waypoints_wait_sec': _float(context, 'follow_waypoints_wait_sec'),
            }],
        ),
    ]

    # ------------------------------------------------------- OSM map pipeline
    static_map_nodes = []
    osm_static_map_builder_node = None
    map_ready_waiter_node = None
    if static_map_generated:
        osm_static_map_builder_node = Node(
            package=PACKAGE,
            executable='osm_static_map_builder',
            name='osm_static_map_builder',
            output='screen',
            parameters=[{
                'input_file': osm_file,
                'output_stem': static_map_output_stem,
                'resolution': _float(context, 'static_map_resolution'),
                'padding_m': _float(context, 'static_map_padding_m'),
                'occupied_buffer_m': _float(context, 'static_map_buffer_m'),
                'road_buffer_m': _float(context, 'static_map_road_buffer_m'),
                'include_occupied_geometry': _flag(context, 'osm_static_map_include_buildings'),
                'unknown_as_occupied': _flag(context, 'osm_static_map_unknown_as_occupied'),
            }],
        )
        static_map_nodes.append(osm_static_map_builder_node)
    if use_georeferenced_static_map:
        static_map_nodes.append(Node(
            package=PACKAGE,
            executable='georeferenced_static_map',
            name='georeferenced_static_map',
            output='screen',
            parameters=[{
                'map_yaml': static_map_yaml,
                'georef_json': static_map_georef,
                'map_topic': '/map',
                'gps_topic': gps_fix_topic,
                'map_frame': 'map',
                'base_frame': base_frame,
                'utm_frame': 'utm',
                'osm_file': osm_file,
                'snap_reference_to_graph': _flag(context, 'snap_static_map_to_graph'),
                'publish_provisional_map': False,
                'align_map_yaw_to_utm': _flag(context, 'align_static_map_yaw_to_utm'),
                'max_snap_distance_m': max_graph_snap_distance_m,
                'samples_required': _int(context, 'georef_samples_required'),
                'publish_period_sec': _float(context, 'georef_publish_period_sec'),
                'freeze_after_init': True,
            }],
        ))
        map_ready_waiter_node = Node(
            package=PACKAGE,
            executable='map_ready_waiter',
            name='map_ready_waiter',
            output='screen',
            # Zero timeout waits forever: Nav2 must not start against an empty
            # static layer (invalid costmap bounds, rejected goals).
            parameters=[{'map_topic': '/map', 'timeout_sec': 0.0}],
        )
        static_map_nodes.append(map_ready_waiter_node)

    # ------------------------------------------------------------------- Nav2
    shared_params = [params_file, {'use_sim_time': use_sim_time}]
    controller_cmd_out = 'cmd_vel_nav_raw' if use_safety_filter else 'cmd_vel_nav'
    # With OSM routing, RViz /goal_pose must go through osm_route_planner only;
    # otherwise bt_navigator would plan a second, competing direct route.
    bt_goal_remappings = (
        [('goal_pose', '/nav2_direct_goal_pose')]
        if route_rviz_goals_on_osm and bool(osm_file)
        else []
    )

    nav2_nodes = []
    if use_map_server:
        nav2_nodes.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=shared_params + [{'yaml_filename': static_map_yaml}],
        ))
    nav2_nodes.extend([
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=shared_params,
             remappings=[('cmd_vel', controller_cmd_out)]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=shared_params),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', parameters=shared_params),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=shared_params,
             remappings=[('cmd_vel', controller_cmd_out)]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=shared_params,
             remappings=bt_goal_remappings),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=shared_params),
    ])
    if use_safety_filter:
        nav2_nodes.append(Node(
            package=PACKAGE,
            executable='cmd_vel_lidar_safety_filter',
            name='cmd_vel_lidar_safety_filter',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_nav_raw',
                'output_topic': '/cmd_vel_nav',
                'scan_topic': scan_topic,
                'backup_scan_topic': backup_scan_topic,
                'front_center_degrees': [
                    float(angle)
                    for angle in _arg(context, 'safety_front_angles_deg').split(',')
                ],
                'front_angle_deg': 80.0,
                'slow_distance_m': _float(context, 'safety_slow_distance_m'),
                'stop_distance_m': _float(context, 'safety_stop_distance_m'),
                'scan_timeout_sec': 0.7,
                'max_forward_speed_when_slow': 0.05,
                'allow_rotation_when_blocked': False,
                'avoid_turn_speed': 0.25,
                'min_turn_cmd_when_blocked': 0.12,
            }],
        ))
    nav2_nodes.extend([
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=shared_params,
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'bond_timeout': 30.0,
                'node_names': (
                    (['map_server'] if use_map_server else []) + [
                        'controller_server',
                        'planner_server',
                        'smoother_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother',
                    ]
                ),
            }],
        ),
    ])

    # --------------------------------------------------------------- ordering
    # Nav2 starts only once the static map is usable (or right away without one).
    launch_actions = robot_actions + static_map_nodes + localization_nodes
    nav2_start = TimerAction(period=nav2_start_delay_sec, actions=nav2_nodes)
    if map_ready_waiter_node is not None:
        launch_actions.append(RegisterEventHandler(OnProcessExit(
            target_action=map_ready_waiter_node, on_exit=[nav2_start])))
    elif osm_static_map_builder_node is not None:
        launch_actions.append(RegisterEventHandler(OnProcessExit(
            target_action=osm_static_map_builder_node, on_exit=[nav2_start])))
    else:
        launch_actions.append(nav2_start)

    if launch_rviz:
        rviz_process = ExecuteProcess(cmd=['rviz2', '-d', rviz_config], output='screen')
        if osm_static_map_builder_node is not None:
            launch_actions.append(RegisterEventHandler(OnProcessExit(
                target_action=osm_static_map_builder_node,
                on_exit=[TimerAction(period=2.0, actions=[rviz_process])])))
        else:
            launch_actions.append(TimerAction(period=3.0, actions=[rviz_process]))

    return launch_actions


def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE)

    def arg(name, default, description=''):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    return LaunchDescription([
        # Robot interface: the only things most robots need to change.
        arg('use_sim_time', 'false'),
        arg('base_frame', 'base_footprint', 'Robot base frame used by Nav2 and the EKFs'),
        arg('gps_fix_topic', '/gps/fix', 'sensor_msgs/NavSatFix from your GPS'),
        arg('heading_imu_topic', '/gps/imu_compass',
            'sensor_msgs/Imu with ABSOLUTE heading in ROS ENU (0 = east, CCW+)'),
        arg('wheel_odom_topic', '/odom',
            'nav_msgs/Odometry from your base; must NOT also publish odom->base TF'),
        arg('base_imu_topic', '/imu', 'sensor_msgs/Imu used for yaw rate'),
        arg('scan_topic', '/scan', 'sensor_msgs/LaserScan for costmaps and the safety filter'),
        arg('backup_scan_topic', '', 'Optional second LaserScan source; empty disables it'),
        arg('base_launch_file', '', 'Optional absolute path to YOUR robot bringup launch file'),

        # GPS / heading IMU mounting (static TF) and optional NMEA driver.
        arg('publish_gps_tf', 'true',
            'Publish the GPS and heading-IMU static TFs below (false if your URDF has them)'),
        arg('gps_parent_frame', 'base_link'),
        arg('gps_frame', 'gps_link', 'frame_id of your NavSatFix messages'),
        arg('gps_x', '0.0'), arg('gps_y', '0.0'), arg('gps_z', '0.0'),
        arg('gps_roll', '0.0'), arg('gps_pitch', '0.0'), arg('gps_yaw', '0.0'),
        arg('heading_imu_parent_frame', 'base_link'),
        arg('heading_imu_frame', 'gps_imu_link', 'frame_id of your heading IMU messages'),
        arg('heading_imu_x', '0.0'), arg('heading_imu_y', '0.0'), arg('heading_imu_z', '0.0'),
        arg('heading_imu_roll', '0.0'), arg('heading_imu_pitch', '0.0'),
        arg('heading_imu_yaw', '0.0'),
        arg('use_nmea_driver', 'false', 'Start nmea_navsat_driver for a serial NMEA GPS'),
        arg('gps_port', '/dev/ttyUSB0'),
        arg('gps_baud', '9600'),

        # Heading calibration.
        arg('heading_invert_yaw', 'false',
            'Invert the heading fused into the robot pose (IMU mounted upside down / mirrored)'),
        arg('heading_yaw_offset', '0.0',
            'Robot-heading offset in radians (e.g. 3.14159 if the IMU faces backward)'),
        arg('magnetic_declination', '0.0', 'Site magnetic declination in radians'),
        arg('navsat_use_odometry_yaw', 'false'),
        arg('navsat_yaw_offset', '0.0'),

        # GPS trust.
        arg('gps_odom_min_horizontal_sigma_m', '3.0',
            'Floor on GPS horizontal sigma fused by the EKF (non-RTK GPS must not dominate)'),
        arg('gps_odom_min_vertical_sigma_m', '10.0'),

        # OpenStreetMap layer.
        arg('osm_file', '', 'Optional .osm / .osm.pbf extract of the site for road routing'),
        arg('static_map_yaml', '', 'Optional hand-made static map yaml'),
        arg('auto_build_static_map', 'true',
            'Rasterize osm_file into a static map when static_map_yaml is empty'),
        arg('static_map_resolution', '0.30'),
        arg('static_map_padding_m', '10.0'),
        arg('static_map_buffer_m', '1.0'),
        arg('static_map_road_buffer_m', '3.0',
            'Half-width of the drivable road corridor; keep wider than GPS drift'),
        arg('osm_static_map_include_buildings', 'false'),
        arg('osm_static_map_unknown_as_occupied', 'true'),
        arg('osm_waypoint_spacing_m', '1.5'),
        arg('osm_nav2_max_waypoints', '80'),
        arg('osm_nav2_simplify_tolerance_m', '0.25'),
        arg('max_graph_snap_distance_m', '20.0'),
        arg('snap_heading_weight_m', '8.0'),
        arg('osm_graph_display_radius_m', '1000.0'),
        arg('snap_static_map_to_graph', 'true'),
        arg('align_static_map_yaw_to_utm', 'false'),
        arg('georef_samples_required', '3'),
        arg('georef_publish_period_sec', '1.0'),
        arg('route_rviz_goals_on_osm', 'true'),
        arg('prune_osm_start_behind_robot', 'true'),
        arg('osm_start_prune_distance_m', '1.0'),
        arg('short_goal_direct_distance_m', '0.0'),
        arg('respect_rviz_goal_orientation', 'true'),
        arg('follow_waypoints_wait_sec', '20.0'),
        arg('nav2_action_mode', 'navigate_through_poses',
            'navigate_through_poses | follow_waypoints'),

        # Startup gate.
        arg('startup_quality_max_horizontal_sigma_m', '6.0'),
        arg('startup_quality_required_good_samples', '5'),
        arg('startup_quality_publish_period_sec', '1.0'),

        # Nav2.
        arg('params_file', os.path.join(pkg, 'config', 'nav2_params_gps.yaml')),
        arg('ekf_file', os.path.join(pkg, 'config', 'ekf_gps.yaml')),
        arg('use_safety_filter', 'true', 'LiDAR cmd_vel safety filter before the smoother'),
        arg('safety_front_angles_deg', '0.0',
            "LiDAR angle(s) that face the robot's forward direction, comma separated "
            '(180.0 for a LiDAR mounted backward)'),
        arg('safety_slow_distance_m', '0.80', 'Cap forward speed below this front range'),
        arg('safety_stop_distance_m', '0.45', 'Block forward motion below this front range'),
        arg('nav2_start_delay_sec', '1.0'),
        arg('launch_rviz', 'false'),
        arg('rviz_config', os.path.join(pkg, 'rviz', 'gps_navigation.rviz')),

        OpaqueFunction(function=launch_setup),
    ])
