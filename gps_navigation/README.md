# gps_navigation: outdoor GPS navigation for any ground robot

**Tuned, field-validated outdoor navigation that you drop onto your own robot.** Bring a GPS, a
compass-style IMU, wheel odometry and a 2D LiDAR. Tell the stack your topic names and where the
sensors are mounted, and it drives: GPS goals, RViz goals, or whole routes that stay on the roads of
an OpenStreetMap extract.

Getting a robot to navigate outdoors on GPS is mostly about the details that don't appear in
tutorials. Consumer GPS reports covariances that are far too optimistic, compass headings disagree
with odometry, a static map has to line up with the live world each time you power on, and Nav2
must not start before any of that is settled. This package wraps those lessons into one launch
file:

| Problem outdoors | What this stack does about it |
|---|---|
| Non-RTK GPS jumps several metres and claims to be accurate | A **covariance floor** (`gps_odom_covariance_filter`) keeps GPS as a slow, steady correction instead of a source of jitter |
| The map, odom and earth frames drift apart in heading | A **two-EKF design** (`ekf_local` + `ekf_global` + `navsat_transform`) anchors both frames to the same corrected compass heading |
| Robots don't know where the roads are | **OpenStreetMap road routing** (`osm_route_planner`) plans on the road graph and hands Nav2 one smooth route that stays on it |
| Without SLAM there is no global map to plan in | **OSM → occupancy map** (`osm_static_map_builder`) turns the same OSM file into a road-corridor costmap, with no outdoor mapping run needed |
| A static map is only useful if it lines up with the robot | **Live georeferencing** (`georeferenced_static_map`) anchors the map to the robot's frame from GPS at startup |
| Driving before localization settles sends the robot the wrong way | A **startup gate**: `map_ready_waiter` holds Nav2 until the anchored map exists, and `startup_quality_monitor` publishes `/startup_quality/ready` once GPS is trustworthy |
| Close obstacles in sunlight | A **LiDAR safety filter** (`cmd_vel_lidar_safety_filter`) slows and stops forward motion independently of the planner |

Each node and the reasoning behind it is written up in [docs/NODES.md](docs/NODES.md).

## Where it fits in this repository

```
ROS-2-Autonomous-Navigation-Indoor-Outdoor/
├── navigation/        indoor Nav2: LiDAR / depth-camera navigation, RTAB-Map
├── slam/              SLAM Toolbox / RTAB-Map mapping
└── gps_navigation/    outdoor GPS + OpenStreetMap navigation  ← this package
```

`gps_navigation` is **self-contained**. It doesn't depend on `navigation` or `slam` and ships no
robot drivers, so all three packages build in the same workspace and you pick a mode by choosing
the launch file:

```bash
colcon build --symlink-install --packages-select slam navigation gps_navigation
```

## Quickstart

```bash
# 1. Dependencies (ROS 2 Humble)
cd ~/ros2_ws/src && git clone https://github.com/Sarvesh-Raj-J/ROS-2-Autonomous-Navigation-Indoor-Outdoor.git
cd ~/ros2_ws
rosdep install --from-paths src/ROS-2-Autonomous-Navigation-Indoor-Outdoor/gps_navigation --ignore-src -r -y

# 2. Build
colcon build --symlink-install --packages-select gps_navigation
source install/setup.bash

# 3. Start YOUR robot (drivers, URDF/TF, base controller). Wheel odometry must not publish odom->base TF.

# 4a. GPS-only outdoor navigation (rolling costmap, goals by GPS or RViz)
ros2 launch gps_navigation gps_navigation.launch.py launch_rviz:=true

# 4b. Road-following navigation on an OpenStreetMap extract of your site
ros2 launch gps_navigation gps_navigation.launch.py osm_file:=/abs/path/site.osm.pbf launch_rviz:=true
```

Before the first run, go through [docs/ROBOT_SETUP.md](docs/ROBOT_SETUP.md). It's a short checklist
of the topic names, mounting offsets, footprint and speed limits for your robot.

### Sending goals

| How | What happens |
|-----|--------------|
| RViz **2D Goal Pose** (`/goal_pose`) | With `osm_file`: snapped onto the road graph and routed along roads. Without: straight to Nav2 |
| `/gps_goal` (`geographic_msgs/GeoPoseStamped`) | Latitude/longitude goal. Routed on roads with `osm_file`, sent directly to Nav2 without it. Yaw is ENU (0 = east, CCW+) |
| RViz goal in the `utm` frame on `/goal_pose_utm` | Converted from UTM into `map` by `utm_goal_bridge` |

```bash
ros2 topic pub --once /gps_goal geographic_msgs/msg/GeoPoseStamped \
  "{pose: {position: {latitude: 12.971, longitude: 80.241}, orientation: {w: 1.0}}}"
```

## Interface contract (what your robot provides)

| Input | Type | Default (launch argument) | Notes |
|-------|------|---------------------------|-------|
| GPS fix | `sensor_msgs/NavSatFix` | `/gps/fix` (`gps_fix_topic`) | Or start the bundled NMEA driver: `use_nmea_driver:=true gps_port:=...` |
| Heading IMU | `sensor_msgs/Imu` | `/gps/imu_compass` (`heading_imu_topic`) | **Absolute** heading in ROS ENU (0 = east, counter-clockwise positive) |
| Wheel odometry | `nav_msgs/Odometry` | `/odom` (`wheel_odom_topic`) | **Without** the `odom → base` TF: `ekf_local` publishes it in this mode |
| Base IMU | `sensor_msgs/Imu` | `/imu` (`base_imu_topic`) | Used for yaw rate |
| LiDAR | `sensor_msgs/LaserScan` | `/scan` (`scan_topic`) | Optional second source: `backup_scan_topic` |
| URDF TF | `/tf_static` | `base_frame` = `base_footprint` | GPS/IMU mount TFs are published for you (`publish_gps_tf`) |
| Velocity | `geometry_msgs/Twist` | `/cmd_vel` | Your base controller subscribes |

TF chain built by the stack: `utm → map → odom → base_footprint`. `navsat_transform` publishes
`utm → map`, `ekf_global` publishes `map → odom` and `ekf_local` publishes `odom → base_footprint`.

## Supported robots

The tuned controller is MPPI with `motion_model: DiffDrive`.

| Drive type | Status |
|------------|--------|
| Differential drive (2WD/4WD) | ✅ Works as tuned |
| Skid-steer 4WD | ✅ Works as tuned |
| Mecanum / omni | ✅ Drives like a diff-drive. Set `motion_model: Omni` and `vy_max` to use strafing |
| Ackermann (car-like steering) | ❌ Not supported by these params. Needs a Smac Hybrid-A* planner and an Ackermann motion model |

## Launch arguments (most used)

| Argument | Default | Purpose |
|----------|---------|---------|
| `osm_file` | `''` | `.osm` / `.osm.pbf` site extract: enables road routing and the generated static map |
| `static_map_yaml` | `''` | Your own static map instead of the OSM-generated one |
| `base_frame` | `base_footprint` | Robot base frame (Nav2 params and EKFs are rewritten at launch) |
| `heading_invert_yaw` / `heading_yaw_offset` | `false` / `0.0` | Correct a mirrored or rotated heading-IMU mount |
| `magnetic_declination` | `0.0` | Site magnetic declination in radians |
| `gps_x … gps_yaw`, `heading_imu_x … heading_imu_yaw` | `0.0` | Antenna / IMU mounting offsets from `base_link` |
| `gps_odom_min_horizontal_sigma_m` | `3.0` | How much the EKF may trust GPS position at best |
| `static_map_road_buffer_m` | `3.0` | Half-width of the drivable road corridor |
| `use_safety_filter` | `true` | LiDAR slow-down/stop filter before the velocity smoother |
| `base_launch_file` | `''` | Optionally start your robot bringup from the same command |
| `launch_rviz` | `false` | Open RViz with route, graph and costmap displays |

Run `ros2 launch gps_navigation gps_navigation.launch.py --show-args` to see every argument.

## Package layout

```
gps_navigation/
├── launch/gps_navigation.launch.py   single entry point
├── config/
│   ├── nav2_params_gps.yaml          Nav2 (rolling global costmap)
│   ├── nav2_params_gps_static.yaml   Nav2 (static OSM/user map layer), auto-selected when a map is used
│   └── ekf_gps.yaml                  robot_localization: ekf_local, ekf_global, navsat_transform
├── behavior_trees/                   replanning trees without spin/back-up recoveries
├── rviz/gps_navigation.rviz
├── gps_navigation/                   the nodes (see docs/NODES.md)
└── docs/  NODES.md · ROBOT_SETUP.md
```

## Notes

- Validated on **ROS 2 Humble**. On Jazzy or newer, Nav2 loads its built-in behavior-tree nodes
  automatically, so delete the `plugin_lib_names` list under `bt_navigator` in the params files.
- Nothing here depends on a particular robot. Every name is a launch argument, and the tuned values
  stay in the YAML files.

## License

MIT, same as the rest of this repository.
