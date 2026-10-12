import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    OpaqueFunction,
    TimerAction,
)


def launch_setup(context):

    compiled = os.environ.get('need_compile', 'False')

    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        navigation_package_path = get_package_share_directory('navigation')
    else:
        slam_package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'slam'))
        navigation_package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'navigation'))

    sim = LaunchConfiguration('sim').perform(context)
    use_joy = LaunchConfiguration('use_joy').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    lidar_serial_port = LaunchConfiguration('lidar_serial_port').perform(context)
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate').perform(context)
    map_name = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    master_name = LaunchConfiguration('master_name').perform(context)
    use_teb = LaunchConfiguration('use_teb').perform(context)

    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'

    if not params_file:
        if use_teb == 'true':
            params_file = os.path.join(navigation_package_path, 'config', 'nav2_controller_teb.yaml')
        else:
            params_file = os.path.join(navigation_package_path, 'config', 'nav2_params.yaml')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')
        ),
        launch_arguments={
            'sim': sim,
            'use_joy': 'false',
            'use_lidar': use_lidar,
            'use_depth_camera': 'false',
            'lidar_serial_port': lidar_serial_port,
            'lidar_serial_baudrate': lidar_serial_baudrate,
            'master_name': master_name,
            'robot_name': robot_name,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_package_path,
                'launch/include/bringup.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': os.path.join(slam_package_path, 'maps', map_name + '.yaml'),
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    bringup_launch = GroupAction(
        actions=[
            base_launch,
            TimerAction(
                period=8.0,
                actions=[navigation_launch],
            ),
        ]
    )

    return [bringup_launch]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/lidar'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('map', default_value='lidar_map'),

        DeclareLaunchArgument(
            'master_name',
            default_value=os.environ.get('MASTER', 'master')
        ),

        DeclareLaunchArgument(
            'robot_name',
            default_value='/'
        ),

        DeclareLaunchArgument('use_teb', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=''),

        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
