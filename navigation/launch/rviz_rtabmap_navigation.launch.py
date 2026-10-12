import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess

def launch_setup(context):
    compiled = os.environ.get('need_compile', 'False')

    if compiled == 'True':
        navigation_package_path = get_package_share_directory('navigation')
    else:
        navigation_package_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'navigation'))

    rviz_node = ExecuteProcess(
            cmd=['rviz2', 'rviz2', '-d', os.path.join(navigation_package_path, 'rviz/rtabmap.rviz')],
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
