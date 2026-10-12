import os
from glob import glob
from setuptools import setup

package_name = 'navigation'

launch_files = glob(os.path.join('launch', '*.launch.py'))
launch_include_files = glob(os.path.join('launch', 'include', '*.launch.py'))
config_files = (
    glob(os.path.join('config', '*.yaml')) +
    glob(os.path.join('config', '*.rviz'))
)
rviz_files = glob(os.path.join('rviz', '*.rviz'))
behavior_tree_files = glob(os.path.join('behavior_trees', '*.xml'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (
            os.path.join('share', package_name, 'launch/include'),
            launch_include_files,
        ),
        (os.path.join('share', package_name, 'config'), config_files),
        (os.path.join('share', package_name, 'rviz'), rviz_files),
        (
            os.path.join('share', package_name, 'behavior_trees'),
            behavior_tree_files,
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_goal_converter=navigation.gps_goal_converter:main',
            'sensor_fallback_monitor=navigation.sensor_fallback_monitor:main',
            'yaw_debug_monitor=navigation.yaw_debug_monitor:main',
            'utm_goal_bridge=navigation.utm_goal_bridge:main',
        ],
    },
)
