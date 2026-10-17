# Robot setup checklist

The tuning is done. What's left is telling the stack about **your** robot. Work through this list
once, top to bottom. Most robots only need launch arguments (sections 1–3), plus a few lines in
the YAML (section 4).

## 1. Sensors and topics (launch arguments)

| Check | Argument | Default |
|-------|----------|---------|
| GPS publishes `sensor_msgs/NavSatFix` | `gps_fix_topic` | `/gps/fix` |
| …or use a serial NMEA receiver directly | `use_nmea_driver:=true gps_port:=/dev/ttyUSB0 gps_baud:=9600` | off |
| Heading IMU gives **absolute** yaw (magnetometer / dual-antenna), ROS ENU | `heading_imu_topic` | `/gps/imu_compass` |
| Wheel odometry topic | `wheel_odom_topic` | `/odom` |
| Base IMU (yaw rate) | `base_imu_topic` | `/imu` |
| LiDAR scan | `scan_topic` | `/scan` |
| Second LiDAR (optional) | `backup_scan_topic` | `''` (disabled) |

> **ENU heading:** yaw 0 points **east** and increases counter-clockwise. Many compass modules output
> 0 = north, clockwise. Convert that in your IMU driver: `yaw_enu = π/2 − heading_compass`.

## 2. TF and frames

- [ ] Your base publishes the URDF (`robot_state_publisher`): `base_footprint → base_link → sensors`.
- [ ] Your base **does not** publish `odom → base_footprint`. In GPS mode `ekf_local` owns that TF.
      Disable the TF in your odometry node, but keep publishing the `nav_msgs/Odometry` message.
- [ ] A different base frame name: `base_frame:=base_link`.
- [ ] GPS antenna position relative to `gps_parent_frame` (`base_link`):
      `gps_x`, `gps_y`, `gps_z` (metres). The antenna's `frame_id` must equal `gps_frame`.
- [ ] Heading IMU mounting relative to `heading_imu_parent_frame`: `heading_imu_x … heading_imu_yaw`.
      The IMU's `frame_id` must equal `heading_imu_frame`.
- [ ] Already have these frames in your URDF? Set `publish_gps_tf:=false`.

## 3. Heading calibration (the step that matters most)

1. Launch the stack with `launch_rviz:=true` and drive straight for ~10 m.
2. Watch `ros2 topic echo /yaw_debug/status`. The GPS-IMU heading, base IMU and filtered odometry
   should agree within a few degrees, and the robot arrow in RViz should point along the drive.
3. If not:
   - The arrow points backward → `heading_yaw_offset:=3.14159`
   - Turning left makes the heading go right → `heading_invert_yaw:=true`
   - Constant small offset → `magnetic_declination:=<radians for your site>`
     (look up the World Magnetic Model value for your latitude/longitude).
4. Repeat until the deltas stay small while driving and turning.

## 4. Robot geometry and limits (edit `config/nav2_params_gps*.yaml`)

| What | Where | Validated value |
|------|-------|-----------------|
| Footprint | `local_costmap` / `global_costmap` → `footprint` | `[[-0.18,-0.15],[-0.18,0.15],[0.18,0.15],[0.18,-0.15]]` |
| Inflation | `inflation_layer` → `inflation_radius` | 0.45 (local), 0.55 (global) |
| Max speed (controller) | `FollowPath` → `vx_max`, `wz_max` | 0.16 m/s, 0.45 rad/s |
| Max speed (smoother) | `velocity_smoother` → `max_velocity` / `min_velocity` | `[0.18, 0.0, 0.35]` |
| Drive type | `FollowPath` → `motion_model` | `DiffDrive` (diff / skid-steer). `Omni` for mecanum with `vy_max` > 0 |

Edit both `nav2_params_gps.yaml` (no static map) and `nav2_params_gps_static.yaml` (OSM or user map),
or pass your own file with `params_file:=`.

> Ackermann (car-like steering) robots aren't covered by these params. They need a Smac
> Hybrid-A* planner and an Ackermann motion model.

## 5. Safety filter

| Check | Argument | Default |
|-------|----------|---------|
| LiDAR angle that faces the robot's front | `safety_front_angles_deg` | `0.0` (`180.0` if the LiDAR is mounted backward; `0.0,180.0` to guard both ends) |
| Slow-down distance | `safety_slow_distance_m` | `0.80` |
| Stop distance | `safety_stop_distance_m` | `0.45` |
| Disable entirely | `use_safety_filter:=false` | — |

## 6. OpenStreetMap (optional, for road routing)

1. Export your site from <https://www.openstreetmap.org/export> (`.osm`) or cut a `.osm.pbf` from a
   regional extract.
2. Check that the paths the robot should drive on exist as `highway=*` ways (footway, path, service…).
   Add any that are missing in OSM, or edit your local copy.
3. Launch with `osm_file:=/abs/path/site.osm`. Tune `static_map_road_buffer_m` to be wider than your
   GPS error, so the robot never "leaves" the road on the map.

## 7. Smoke test

```bash
ros2 topic echo /outdoor_nav_health/status     # every input has a non-zero rate
ros2 topic echo /startup_quality/ready         # true before sending goals (with osm_file)
ros2 topic echo /osm_route_status              # route accepted / reasons for rejection
```

Then send a short goal in RViz, followed by a GPS goal:

```bash
ros2 topic pub --once /gps_goal geographic_msgs/msg/GeoPoseStamped \
  "{pose: {position: {latitude: <lat>, longitude: <lon>}, orientation: {w: 1.0}}}"
```
