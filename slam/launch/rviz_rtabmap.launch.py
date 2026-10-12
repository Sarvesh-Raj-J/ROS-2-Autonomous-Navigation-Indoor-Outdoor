import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, ExecuteProcess


def resolve_package_path(package_name: str, source_path: str, compiled_flag: str) -> str:
    if compiled_flag == 'True':
        try:
            return get_package_share_directory(package_name)
        except Exception:
            return source_path
    return source_path


def launch_setup(context):
    compiled = os.getenv('need_compile', 'True')

    slam_package_path = resolve_package_path(
        'slam',
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..')),
        compiled,
    )

    rviz_node = ExecuteProcess(
            cmd=['rviz2', 'rviz2', '-d', os.path.join(slam_package_path, 'rviz/rtabmap.rviz')],
            output='screen'
        )

    return [rviz_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
