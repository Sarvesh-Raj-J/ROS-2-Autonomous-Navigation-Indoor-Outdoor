import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction


def resolve_package_path(package_name: str, source_path: str, compiled_flag: str) -> str:
    """Prefer the installed share directory but fall back to source if missing."""
    if compiled_flag == 'True':
        try:
            return get_package_share_directory(package_name)
        except Exception:
            return source_path
    return source_path

def launch_setup(context, sim, use_lidar, rtabmap_use_scan, master_name, robot_name, depth_camera_name, depth_camera_serial_number, depth_camera_device_num, depth_registration):
    compiled = os.getenv('need_compile', 'True')
    sim_value = sim.perform(context)
    use_lidar_value = use_lidar.perform(context)
    rtabmap_use_scan_value = rtabmap_use_scan.perform(context)
    master_value = master_name.perform(context)
    robot_value = robot_name.perform(context)
    depth_camera_name_value = depth_camera_name.perform(context)
    depth_camera_serial_number_value = depth_camera_serial_number.perform(context)
    depth_camera_device_num_value = depth_camera_device_num.perform(context)
    depth_registration_value = depth_registration.perform(context)
    frame_prefix = '' if robot_value == '/' else '%s/' % robot_value
    topic_prefix = '' if robot_value == '/' else '/%s' % robot_value
    use_sim_time = 'true' if sim_value == 'true' else 'false'

    slam_package_path = resolve_package_path(
        'slam',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..')),
        compiled,
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim_value,
            'use_lidar': use_lidar_value,
            'master_name': master_value,
            'robot_name': robot_value,
            'action_name': 'horizontal',
            'use_depth_camera': 'true',
            'depth_camera_name': depth_camera_name_value,
            'depth_camera_serial_number': depth_camera_serial_number_value,
            'depth_camera_device_num': depth_camera_device_num_value,
            'depth_registration': depth_registration_value,
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'qos': '2',
            'subscribe_scan': rtabmap_use_scan_value,
            'approx_sync': 'true',
            'approx_sync_max_interval': '1.0',
            'sync_queue_size': '500',
            'topic_queue_size': '100',
            'wait_for_transform': '2.0',
            'wait_for_transform_duration': '5.0',
        }.items(),
    )

    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_value),
         base_launch,
         TimerAction(
             period=10.0,  # 延时等待其它节点启动好(delay to enable other nodes)
             actions=[rtabmap_launch],
         ),
      ]
    )

    return [bringup_launch]


def generate_launch_description():
    sim = LaunchConfiguration('sim', default='false')
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    rtabmap_use_scan = LaunchConfiguration('rtabmap_use_scan', default='true')
    master_name = LaunchConfiguration('master_name', default=os.getenv('MASTER', '/'))
    robot_name = LaunchConfiguration('robot_name', default=os.getenv('HOST', '/'))
    depth_camera_name = LaunchConfiguration('depth_camera_name', default='depth_cam')
    depth_camera_serial_number = LaunchConfiguration('depth_camera_serial_number', default='ACR384300GZ')
    depth_camera_device_num = LaunchConfiguration('depth_camera_device_num', default='0')
    depth_registration = LaunchConfiguration('depth_registration', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value=sim, description='Use simulation time'),
        DeclareLaunchArgument('use_lidar', default_value=use_lidar, description='Enable LiDAR and scan fusion'),
        DeclareLaunchArgument('rtabmap_use_scan', default_value=rtabmap_use_scan, description='Use scan in RTAB-Map'),
        DeclareLaunchArgument('master_name', default_value=master_name, description='Master robot namespace'),
        DeclareLaunchArgument('robot_name', default_value=robot_name, description='Robot namespace'),
        DeclareLaunchArgument('depth_camera_name', default_value=depth_camera_name, description='Depth camera namespace/name'),
        DeclareLaunchArgument('depth_camera_serial_number', default_value=depth_camera_serial_number, description='Depth camera serial number'),
        DeclareLaunchArgument('depth_camera_device_num', default_value=depth_camera_device_num, description='Depth camera index'),
        DeclareLaunchArgument('depth_registration', default_value=depth_registration, description='Enable depth registration'),
        OpaqueFunction(function=lambda context: launch_setup(context, sim, use_lidar, rtabmap_use_scan, master_name, robot_name, depth_camera_name, depth_camera_serial_number, depth_camera_device_num, depth_registration)),
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
