from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    subscribe_scan = LaunchConfiguration('subscribe_scan')
    approx_sync = LaunchConfiguration('approx_sync')
    approx_sync_max_interval = LaunchConfiguration('approx_sync_max_interval')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    topic_queue_size = LaunchConfiguration('topic_queue_size')
    wait_for_transform = LaunchConfiguration('wait_for_transform')
    wait_for_transform_duration = LaunchConfiguration('wait_for_transform_duration')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': subscribe_scan,
        'use_tf_camera': False,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        'tf_tolerance': 0.5,
        'wait_for_transform': wait_for_transform,
        'wait_for_transform_duration': wait_for_transform_duration,
        # RTAB-Map's parameters should be strings:
        'queue_size': sync_queue_size,
        'sync_queue_size': sync_queue_size,
        'topic_queue_size': topic_queue_size,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/OptimizeMaxError': '1.5',
        'RGBD/LoopThr': '0.16',  # tighten loop acceptance to reduce false closures
        'Grid/RangeMin': '0.2',  # ignore laser scan points on the robot itself
        'Optimizer/GravitySigma': '0',  # Disable imu constraints (we are already in 2D)
        'Vis/CorType': '0',
        'Vis/MinInliers': '30',  # require more feature support for loop closures
        'OdomF2M/MaxSize': '4000',
        'Vis/MaxFeatures': '4000',
        'Optimizer/Slam2D': 'true',
        'grid_size': '20',
        'Grid/Sensor': '1',
        'RGBD/ProximityPathMaxNeighbors': '0',  # disable proximity loop closures by distance
        'RGBD/ProximityBySpace': 'false',  # rely on visual similarity only
        'proj_max_ground_height': '0.01',
        'proj_max_ground_angle': '10',
        'proj_min_cluster_size': '10',
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', '/depth_cam/rgb/image_raw'),
        ('rgb/camera_info', '/depth_cam/rgb/camera_info'),
        ('depth/image', '/depth_cam/depth/image_raw'),
        ('odom', '/odom'),
        ('scan', '/scan_raw')
    ]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        DeclareLaunchArgument(
            'subscribe_scan', default_value='false',
            description='Subscribe to laser scan topic'),

        DeclareLaunchArgument(
            'approx_sync', default_value='true',
            description='Use approximate time sync for RGB-D'),

        DeclareLaunchArgument(
            'approx_sync_max_interval', default_value='0.2',
            description='Max interval for approximate sync'),

        DeclareLaunchArgument(
            'sync_queue_size', default_value='100',
            description='Queue size for RGB-D sync'),

        DeclareLaunchArgument(
            'topic_queue_size', default_value='100',
            description='Queue size for RTAB-Map subscriptions'),

        DeclareLaunchArgument(
            'wait_for_transform', default_value='2.0',
            description='TF wait duration per lookup'),

        DeclareLaunchArgument(
            'wait_for_transform_duration', default_value='5.0',
            description='Total timeout for TF waits'),

        # Static TF for camera optical frame so RGBD messages are usable
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'depth_cam_link', 'depth_cam_color_optical_frame']
        ),
            
        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{
                'approx_sync': approx_sync,
                'approx_sync_max_interval': approx_sync_max_interval,
                'use_sim_time': use_sim_time,
                'qos': qos,
                'topic_queue_size': topic_queue_size,
                'sync_queue_size': sync_queue_size,
            }],
            remappings=remappings),

        # SLAM Mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
        
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
