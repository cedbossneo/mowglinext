# Mowgli ROS2

Complete ROS2 rewrite of the OpenMower robot mower. Targets ROS2 Jazzy with Nav2, slam_toolbox, Fields2Cover v2, and BehaviorTree.CPP v4.

## Quick Reference

- **ROS distro:** Jazzy
- **Build:** `colcon build` (ament_cmake for C++, ament_python for launch)
- **Run simulation:** `docker compose up dev-sim` (Gazebo + Nav2 + Foxglove on ws://localhost:8765)
- **Run hardware:** `docker compose up mowgli` (requires /dev/mowgli)
- **Send mow command:** `ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl "{command: 1}"`
- **Source workspace inside container:** `source /ros2_ws/install/setup.bash`

## Packages (10)

| Package | Purpose |
|---------|---------|
| `mowgli_interfaces` | ROS2 msg/srv/action definitions |
| `mowgli_hardware` | COBS serial bridge to STM32 (IMU, blade, E-stop, battery) |
| `mowgli_bringup` | Launch files, URDF, Nav2 config, EKF config |
| `mowgli_localization` | Wheel odometry, GPS converter, dual EKF, localization monitor |
| `mowgli_nav2_plugins` | FTCController (Nav2 plugin), oscillation detector |
| `mowgli_behavior` | BehaviorTree.CPP v4 action nodes + main_tree.xml |
| `mowgli_map` | 4-layer GridMap (occupancy, mow_progress, keepout, speed masks) |
| `mowgli_coverage_planner` | Fields2Cover v2 (headland + boustrophedon + Dubins curves) |
| `mowgli_monitoring` | Diagnostics aggregator (8 checks) + optional MQTT bridge |
| `mowgli_simulation` | Gazebo Harmonic worlds, mower SDF model, VNC support |

## Architecture

```
Behavior Tree (main_tree.xml)
  |-- Emergency / Rain / Battery guards (reactive, highest priority)
  |-- Mowing sequence:
  |     Undock (BackUp 1.5m) -> PlanCoverage -> NavigateToPose -> FollowCoveragePath
  |-- Recovery: 3 retries with ClearCostmap + BackUp between attempts
  |-- On completion or failure: SaveSlamMap -> Dock

Localization (dual EKF):
  ekf_odom (50Hz): wheel_odom velocity + IMU yaw -> odom->base_link
  ekf_map  (20Hz): filtered_odom velocity + GPS pose if fixed or SLAM when GPS float -> map->odom

Navigation:
  Transit:   RotationShimController wrapping RegulatedPurePursuit (FollowPath)
  Coverage:  RegulatedPurePursuit bare (FollowCoveragePath)
  Planner:   SmacPlanner2D
  Costmaps:  ObstacleLayer (LiDAR /scan) + InflationLayer
```

## Key Config Files

All live-editable via docker-compose.override.yml bind-mounts (no rebuild needed):

| File | What it controls |
|------|-----------------|
| `src/mowgli_bringup/config/nav2_params.yaml` | All Nav2 params: controllers, planner, costmaps, collision monitor, velocity smoother |
| `src/mowgli_bringup/config/coverage_planner.yaml` | F2C parameters: tool_width, spacing, headland, turning radius |
| `src/mowgli_localization/config/localization.yaml` | Dual EKF tuning, GPS converter, wheel odometry |
| `src/mowgli_behavior/trees/main_tree.xml` | BT structure (guards, mowing sequence, recovery) |
| `src/mowgli_simulation/models/mowgli_mower/model.sdf` | Gazebo robot model (LiDAR, IMU, diff_drive) -- NOT bind-mounted, needs image rebuild |

## Docker Services

| Service | Description |
|---------|-------------|
| `mowgli` | Real hardware (USB serial to STM32) |
| `simulation` | Headless Gazebo + Nav2 + Foxglove |
| `simulation-gui` | Gazebo with VNC GUI (noVNC on :6080) |
| `dev-sim` | Development sim with live config/launch/tree bind-mounts |

`docker-compose.override.yml` activates DEBUG logging and bind-mounts for all services.

## Critical Design Decisions

### Coverage Path Following: RPP not MPPI
MPPI's Euclidean nearest-point matching jumps between adjacent parallel boustrophedon swaths (only 0.18m apart). RPP uses sequential lookahead which tracks correctly. `max_robot_pose_search_dist: 5.0` prevents even RPP from jumping to adjacent swaths.

### Localization: GPS dominates, wheel ticks are unreliable
Wheel encoders slip on wet/soft terrain. GPS RTK is the primary position source. Wheel ticks provide velocity hints only, with HIGH process noise in the EKF. IMU is the most reliable heading source.

### Docking: undock = reverse, dock = frontal
The robot reverses 1.5m to undock. Docking requires precise frontal alignment with charging contacts.

### Progress Checker: single with long timeout
Nav2 Jazzy's bt_navigator sends empty `progress_checker_id` for NavigateToPose. Multiple progress checker plugins cause failures. Use one checker with `movement_time_allowance: 3600s` (effectively disabled for long coverage runs).

### Collision Monitor: disabled scan source in simulation
The Gazebo LiDAR (range_min=0.10m) produces self-reflections from the robot chassis at ~0.26-0.30m. These phantom readings trigger PolygonStop and FootprintApproach, permanently blocking the robot. The scan source is disabled in the nav2_params.yaml config. Re-enable for real hardware where self-reflections don't occur.

## Known Issues & TODOs

### Active Issues
- **BT tick log flooding:** PublishHighLevelStatus oscillates IDLE<->SUCCESS every ~600ms in idle state, generating ~444K log lines per 40 min. Needs throttling or conditional publishing.
- **Gazebo model not bind-mounted:** Changes to model.sdf require Docker image rebuild. Mounting the models/ directory breaks Gazebo sensor initialization (suspected permission or path resolution issue).
- **distance_to_goal feedback:** FollowPath reports distance_to_goal starting at full path length (~1395m) which is correct but initially misleading.

### TODO: High Priority
- [ ] Verify full 13329-pose coverage path completion in simulation (RPP + disabled collision monitor)
- [ ] Build MowingAreaManager node: persistent LiDAR obstacle detection feeding into F2C replanning
- [x] Implement zone management: costmap filter mask publisher for keepout/speed zones
- [x] Area recording: GUI → Go backend → ROS2 map_server_node services (add_area, set_docking_point, clear_map)
- [x] Area persistence: auto-save/load areas and docking point to /ros2_ws/maps/areas.dat
- [ ] Wire BT dock_pose from map_server_node/docking_pose topic instead of static parameter
- [ ] Reduce BT log verbosity (conditional publish, rate-limit tick logging)
- [ ] Fix Gazebo LiDAR self-reflections at source (increase range_min to 0.35m in model.sdf, rebuild image)

### TODO: Medium Priority
- [ ] Wire `mowgli_robot.yaml` centralized config into all launch files
- [ ] Test rain and battery dock/resume flows end-to-end in simulation
- [ ] Implement stuck detection: wheel ticks advancing but GPS/SLAM position unchanged -> backup + reroute
- [ ] FTCController integration (replace RPP for coverage once validated)
- [ ] Obstacle avoidance: detect, navigate around, add persistent obstacles to SLAM map

### TODO: Lower Priority
- [ ] SLAM map persistence across container restarts (currently saves before dock, but load-on-startup may need work)
- [ ] GPS + odom fusion tuning on real hardware (field testing required)
- [ ] opennav_docking integration for precise dock approach
- [ ] Mow progress tracking visualization (map_server mow_progress layer -> Foxglove)

## Development Workflow

### Edit config/launch (no rebuild):
```bash
# Edit files under src/mowgli_bringup/config/ or src/mowgli_behavior/trees/
docker compose restart dev-sim  # picks up changes via bind-mount
```

### Edit C++ source (rebuild required):
```bash
docker compose build dev-sim    # rebuilds image
docker compose up -d dev-sim    # starts with new code
```

### Monitor mowing:
```bash
# Stream logs (filter BT noise):
docker logs mowgli_dev_sim -f 2>&1 | grep -v "PublishHighLevelStatus\|Inverter\|Guard\|NeedsDocking\|IsRainDetected\|IsEmergency"

# Check robot position:
docker exec mowgli_dev_sim bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /wheel_odom --once' | grep -A3 position:

# Check coverage progress:
docker exec mowgli_dev_sim bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /follow_path/_action/feedback --once' | grep distance
```

### Foxglove Studio:
Connect to `ws://localhost:8765` to visualize:
- `/scan` (LiDAR)
- `/coverage_planner_node/coverage_path` (planned path)
- `/local_costmap/costmap` (obstacle map)
- `/behavior_tree_node/high_level_status` (BT state)

## Conventions

- **C++ standard:** C++17, ament_cmake build
- **Naming:** snake_case for files/params, CamelCase for C++ classes
- **Frames:** `map` (global), `odom` (local), `base_link` (robot), `lidar_link`, `imu_link`
- **Units:** metres, radians, seconds (SI throughout)
- **Config:** YAML files under each package's `config/` directory
- **Topics:** namespaced under node name (`~/`) for internal, absolute for shared (`/scan`, `/cmd_vel`)
