import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time_bool = use_sim_time.lower() == 'true'

    if not os.path.isfile(params_file):
        raise RuntimeError(f'params_file not found: {params_file}')
    print(f'\n>>> Loading nav2 params from: {params_file}\n')

    shared_params = [params_file, {'use_sim_time': use_sim_time_bool}]

    return [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=shared_params,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=shared_params),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', parameters=shared_params),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=shared_params,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=shared_params),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=shared_params),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=shared_params,
             remappings=[('cmd_vel','cmd_vel_nav'),('cmd_vel_smoothed','cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time_bool, 'autostart': True,
                'node_names': ['controller_server','planner_server','smoother_server',
                               'behavior_server','bt_navigator','waypoint_follower',
                               'velocity_smoother']}]),
    ]


def generate_launch_description():
    navigation_package_path = get_package_share_directory('navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            navigation_package_path, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        OpaqueFunction(function=launch_setup),
    ])
