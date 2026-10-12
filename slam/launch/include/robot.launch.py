import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    TimerAction,
)


def resolve_package_path(package_name: str, source_path: str, compiled_flag: str) -> str:
    if compiled_flag == 'True':
        try:
            return get_package_share_directory(package_name)
        except Exception:
            return source_path
    return source_path


def launch_setup(context):
    compiled = os.getenv('need_compile', 'True')

    sim                         = LaunchConfiguration('sim',                         default='true').perform(context)
    use_joy                     = LaunchConfiguration('use_joy',                     default='true').perform(context)
    use_lidar                   = LaunchConfiguration('use_lidar',                   default='true')
    use_depth_camera            = LaunchConfiguration('use_depth_camera',            default='false')
    enable_odom                 = LaunchConfiguration('enable_odom',                 default='true').perform(context)
    master_name                 = LaunchConfiguration('master_name',                 default=os.getenv('MASTER', '/')).perform(context)
    robot_name                  = LaunchConfiguration('robot_name',                  default=os.getenv('HOST', '/')).perform(context)
    depth_camera_name           = LaunchConfiguration('depth_camera_name',           default='depth_cam').perform(context)
    depth_camera_serial_number  = LaunchConfiguration('depth_camera_serial_number',  default=os.getenv('DEPTH_CAMERA_SERIAL_NUMBER', '')).perform(context)
    depth_camera_device_num     = LaunchConfiguration('depth_camera_device_num',     default='1').perform(context)
    depth_registration          = LaunchConfiguration('depth_registration',          default='true').perform(context)
    depth_camera_enable_color   = LaunchConfiguration('depth_camera_enable_color',   default='false').perform(context)
    depth_camera_enable_point_cloud = LaunchConfiguration('depth_camera_enable_point_cloud', default='false').perform(context)
    enable_usb_cam              = LaunchConfiguration('enable_usb_cam',              default='true').perform(context)
    enable_depth_odom          = LaunchConfiguration('enable_depth_odom',           default=use_depth_camera).perform(context)
    action_name                 = LaunchConfiguration('action_name',                 default='init').perform(context)

    default_lidar_port = os.getenv('LIDAR_SERIAL_PORT', '/dev/lidar')
    lidar_serial_port  = LaunchConfiguration('lidar_serial_port', default=default_lidar_port).perform(context)

    default_lidar_baudrate = os.getenv('LIDAR_BAUDRATE')
    if not default_lidar_baudrate:
        default_lidar_baudrate = '115200' if os.getenv('LIDAR_TYPE', 'A1') == 'A1' else '256000'
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate', default=default_lidar_baudrate).perform(context)

    # --- Declare Arguments ---
    sim_arg                        = DeclareLaunchArgument('sim',                        default_value=sim)
    master_name_arg                = DeclareLaunchArgument('master_name',                default_value=master_name)
    robot_name_arg                 = DeclareLaunchArgument('robot_name',                 default_value=robot_name)
    depth_camera_name_arg          = DeclareLaunchArgument('depth_camera_name',          default_value=depth_camera_name)
    depth_camera_serial_number_arg = DeclareLaunchArgument('depth_camera_serial_number', default_value=depth_camera_serial_number)
    depth_camera_device_num_arg    = DeclareLaunchArgument('depth_camera_device_num',    default_value=depth_camera_device_num)
    depth_registration_arg         = DeclareLaunchArgument('depth_registration',         default_value=depth_registration)
    depth_camera_enable_color_arg  = DeclareLaunchArgument('depth_camera_enable_color',  default_value=depth_camera_enable_color)
    depth_camera_enable_point_cloud_arg = DeclareLaunchArgument('depth_camera_enable_point_cloud', default_value=depth_camera_enable_point_cloud)
    enable_usb_cam_arg             = DeclareLaunchArgument('enable_usb_cam',             default_value=enable_usb_cam)
    enable_depth_odom_arg          = DeclareLaunchArgument('enable_depth_odom',          default_value=enable_depth_odom)
    lidar_serial_port_arg          = DeclareLaunchArgument('lidar_serial_port',          default_value=lidar_serial_port)
    lidar_serial_baudrate_arg      = DeclareLaunchArgument('lidar_serial_baudrate',      default_value=lidar_serial_baudrate)
    use_joy_arg                    = DeclareLaunchArgument('use_joy',                    default_value=use_joy)
    use_lidar_arg                  = DeclareLaunchArgument('use_lidar',                  default_value=use_lidar)
    use_depth_camera_arg           = DeclareLaunchArgument('use_depth_camera',           default_value=use_depth_camera)
    enable_odom_arg                = DeclareLaunchArgument('enable_odom',                default_value=enable_odom)
    action_name_arg                = DeclareLaunchArgument('action_name',                default_value=action_name)

    # --- Velocity Limits ---
    # MECANUM NOTE: max_vel_y must be non-zero to enable lateral (strafing) movement.
    # These must match or be within the bounds set in nav2_params_mecanum.yaml:
    #   max_vel_x: 0.5, max_vel_y: 0.5, max_vel_theta: 1.0
    max_linear_x_sim   = '0.5'   # Match nav2 max_vel_x
    max_linear_x       = '0.3'   # Real robot conservative limit
    max_linear_y_sim   = '0.5'   # Match nav2 max_vel_y — REQUIRED for mecanum strafing
    max_linear_y       = '0.3'   # Real robot lateral limit
    max_angular_sim    = '1.0'   # Match nav2 max_vel_theta
    max_angular        = '0.5'

    # --- Frame / Topic Setup ---
    topic_prefix  = '' if robot_name == '/' else '/%s' % robot_name
    frame_prefix  = '' if robot_name == '/' else '%s/' % robot_name
    use_namespace = 'false' if robot_name == '/' else 'true'
    namespace     = '' if robot_name == '/' else robot_name
    use_sim_time  = 'true' if sim == 'true' else 'false'

    map_frame    = '{}map'.format(frame_prefix) if robot_name == master_name else '{}/map'.format(master_name)
    cmd_vel_topic = 'controller/cmd_vel'
    scan_raw     = 'scan_raw'
    scan_topic   = 'scan'
    odom_frame   = '{}odom'.format(frame_prefix)
    base_frame   = '{}base_footprint'.format(frame_prefix)
    lidar_frame  = '{}lidar_frame'.format(frame_prefix)
    imu_frame    = '{}imu_link'.format(frame_prefix)

    source_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    peripherals_package_path = resolve_package_path(
        'peripherals',
        os.path.join(source_root, 'peripherals'),
        compiled,
    )
    controller_package_path = resolve_package_path(
        'controller',
        os.path.join(source_root, 'driver', 'controller'),
        compiled,
    )

    # --- Controller Launch ---
    # CRITICAL FOR MECANUM:
    # Your controller.launch.py and the underlying hardware driver MUST:
    #   1. Accept Twist messages with non-zero linear.y (lateral velocity)
    #   2. Publish odometry with correct linear.y from wheel kinematics
    #   3. Use mecanum/omni inverse kinematics to compute per-wheel velocities
    # If your controller uses a diff-drive plugin, lateral movement will be ignored.
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')
        ),
        launch_arguments={
            'namespace':          namespace,
            'use_namespace':      use_namespace,
            'frame_prefix':       frame_prefix,
            'odom_frame':         odom_frame,
            'base_frame':         base_frame,
            'map_frame':          map_frame,
            'imu_frame':          imu_frame,
            'use_sim_time':       use_sim_time,
            'enable_odom':        enable_odom,
            'laser_scan_topic':   scan_raw,
            'enable_lidar_odom':  'false',
            'enable_depth_odom':  enable_depth_odom,
            # Pass lateral velocity limit so the controller knows the mecanum envelope
            'max_linear_y':       max_linear_y_sim if sim == 'true' else max_linear_y,
        }.items()
    )

    # --- Depth Camera Launch (conditional) ---
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')
        ),
        launch_arguments={
            'depth_camera_name': depth_camera_name,
            'serial_number':     depth_camera_serial_number,
            'device_num':        depth_camera_device_num,
            'depth_registration': depth_registration,
            'enable_color':      depth_camera_enable_color,
            'enable_point_cloud': depth_camera_enable_point_cloud,
            'enable_usb_cam':    enable_usb_cam,
            'publish_tf':        'false',
            'tf_prefix':         frame_prefix,
        }.items(),
        condition=IfCondition(use_depth_camera)
    )

    # --- Lidar Launch (conditional) ---
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')
        ),
        launch_arguments={
            'lidar_frame':      lidar_frame,
            'scan_topic':       scan_topic,
            'scan_raw':         scan_raw,
            'serial_port':      lidar_serial_port,
            'serial_baudrate':  lidar_serial_baudrate,
        }.items(),
        condition=IfCondition(use_lidar)
    )

    # --- Joystick Launch (conditional) ---
    # MECANUM NOTE: Make sure joystick_control.launch.py maps an axis to linear.y
    # so you can test lateral strafing manually. Most default joy configs only map
    # linear.x and angular.z — you will need to add a linear.y axis mapping.
    joystick_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')
        ),
        launch_arguments={
            'max_linear':    max_linear_x_sim if sim == 'true' else max_linear_x,
            'max_linear_y':  max_linear_y_sim if sim == 'true' else max_linear_y,  # Added for mecanum strafing
            'max_angular':   max_angular_sim  if sim == 'true' else max_angular,
            'remap_cmd_vel': cmd_vel_topic,
        }.items(),
        condition=IfCondition(use_joy)
    )

    # --- Init Pose Launch ---
    init_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/init_pose.launch.py')
        ),
        launch_arguments={
            'namespace':      namespace,
            'use_namespace':  use_namespace,
            'action_name':    action_name,
        }.items(),
    )

    # --- Bringup Group ---
    bringup_launch = GroupAction(
        actions=[
            TimerAction(
                period=2.0,  # Delay to allow other nodes to initialize
                actions=[
                    controller_launch,
                    lidar_launch,
                    joystick_control_launch,
                    init_pose_launch,
                ],
            ),
        ]
    )

    return [
        sim_arg,
        master_name_arg,
        robot_name_arg,
        depth_camera_name_arg,
        depth_camera_serial_number_arg,
        depth_camera_device_num_arg,
        depth_registration_arg,
        depth_camera_enable_color_arg,
        depth_camera_enable_point_cloud_arg,
        enable_usb_cam_arg,
        enable_depth_odom_arg,
        lidar_serial_port_arg,
        lidar_serial_baudrate_arg,
        use_joy_arg,
        use_lidar_arg,
        use_depth_camera_arg,
        enable_odom_arg,
        action_name_arg,
        depth_camera_launch,
        bringup_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
