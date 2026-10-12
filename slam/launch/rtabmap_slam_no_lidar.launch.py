import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction


def resolve_package_path(package_name: str, source_path: str, compiled_flag: str) -> str:
    if compiled_flag == 'True':
        try:
            return get_package_share_directory(package_name)
        except Exception:
            return source_path
    return source_path


def launch_setup(context):
    compiled = os.getenv('need_compile', 'True')
    sim = LaunchConfiguration('sim', default='false').perform(context)
    master_name = LaunchConfiguration('master_name', default=os.getenv('MASTER', '/')).perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.getenv('HOST', '/')).perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    use_sim_time = 'true' if sim == 'true' else 'false'

    slam_package_path = resolve_package_path(
        'slam',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..')),
        compiled,
    )

    # Include rtabmap launch only — omit robot/driver launch that starts the lidar
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_name),
         rtabmap_launch,
      ]
    )

    return [sim_arg, master_name_arg, robot_name_arg, bringup_launch]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
