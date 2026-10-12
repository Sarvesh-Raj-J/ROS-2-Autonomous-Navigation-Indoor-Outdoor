# Autonomous Navigation (Indoor and Outdoor)

Reusable ROS 2 navigation and SLAM packages extracted from the JetAutoPro
workspace. Hardware drivers and robot bringup are intentionally not included.

## Included

- `navigation/`: Nav2 indoor and outdoor launch files, GPS/IMU localization,
  lidar navigation, depth-camera navigation, RTAB-Map integration, RViz
  configurations, and navigation helper nodes.
- `slam/`: SLAM Toolbox and RTAB-Map launch files, mapping parameters, maps,
  robot launch integration, and map-saving support.
- `videos/`: demonstrations of indoor depth-camera navigation and obstacle
  avoidance.

## Robot-provided interfaces

The robot and its drivers must provide these ROS 2 topics and transforms:

```text
/scan
/odom
/cmd_vel
/camera/color/image_raw
/camera/depth/image_raw
/camera/color/camera_info
/gps/fix
/imu/data
/tf
/tf_static
```

The exact sensor topic names can be adapted through launch remappings and
configuration files. This repository does not include lidar, depth-camera,
GPS, IMU, motor-controller, or robot-base drivers.

## Modes

- **Indoor lidar:** SLAM Toolbox or map-based Nav2 localization with a 2D
  lidar.
- **Indoor depth camera:** RTAB-Map localization/mapping and depth obstacle
  avoidance.
- **Outdoor:** GPS/IMU localization with robot_localization and Nav2.

## External ROS 2 dependencies

Install the ROS 2 distributions of Nav2, SLAM Toolbox, RTAB-Map,
robot_localization, nmea_navsat_driver, tf2_ros, and the message packages
listed in `navigation/package.xml` and `slam/package.xml`.

Build these packages inside a ROS 2 workspace with:

```bash
colcon build --symlink-install --packages-select slam navigation
source install/setup.bash
```
