# Mowgli ROS2

Complete ROS2 rewrite of the OpenMower robot mower. Targets ROS2 Jazzy with Nav2, slam_toolbox, Fields2Cover v2, and BehaviorTree.CPP v4.

## Maintenance Instructions

**CLAUDE.md is the living project reference. Keep it accurate.**

After completing any task that changes the project state (new features, bug fixes, config changes, topic changes, new nodes, completed TODOs, etc.):
1. Update the relevant section of this CLAUDE.md to reflect the change.
2. If a TODO was completed, mark it `[x]` and move it to the completed group.
3. If a TODO is no longer relevant (superseded, removed, or the approach changed), remove it and note why in a brief comment if non-obvious.
4. If new work creates a new TODO, add it to the appropriate priority section.

Before starting a session, scan the TODO list and verify each item against the codebase — mark done items that were completed in previous sessions but not yet updated here.

## Quick Reference

- **ROS distro:** Jazzy
- **DDS:** Cyclone DDS (`rmw_cyclonedds_cpp`)
- **Build:** `make build` (or `colcon build` directly)
- **Run simulation:** `make sim` (Gazebo + Nav2 + Foxglove on ws://localhost:8765)
- **Run E2E test:** `make e2e-test` (simulation must be running)
- **Run hardware:** `cd ../docker && docker compose up mowgli` (requires /dev/mowgli)
- **Send mow command:** `ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl "{command: 1}"`
- **Source workspace inside container:** `source /ros2_ws/install/setup.bash`
- **GUI:** openmower-gui (Go backend + React frontend), connects to rosbridge on port 9090

## Packages (11)

| Package | Purpose |
|---------|---------|
| `mowgli_interfaces` | ROS2 msg/srv definitions (12 msgs, 9 srvs) |
| `mowgli_hardware` | COBS serial bridge to STM32 (IMU, blade, E-stop, battery) |
| `mowgli_bringup` | Launch files (mowgli.launch.py + full_system.launch.py), Nav2 config, EKF config, URDF |
| `mowgli_localization` | Wheel odometry, GPS converter (navsat_to_absolute_pose), dual EKF, localization monitor |
| `mowgli_nav2_plugins` | FTCController (Nav2 plugin), oscillation detector |
| `mowgli_behavior` | BehaviorTree.CPP v4 action nodes, DockRobot/UndockRobot, ExecuteSwathBySwath + main_tree.xml |
| `mowgli_map` | map_server_node (area management, GridMap) + obstacle_tracker_node (persistent LiDAR obstacle detection) |
| `mowgli_coverage_planner` | Fields2Cover v2.0.0 direct integration (headland + boustrophedon + ReedsShepp turns) |
| `mowgli_monitoring` | Diagnostics aggregator (8 checks) + optional MQTT bridge |
| `mowgli_simulation` | Gazebo Harmonic worlds, mower SDF model, GPS degradation sim, VNC support |
| `mowgli_description` | URDF/xacro model (currently in mowgli_bringup/urdf/) |

## Launch Structure

Two-tier launch:

1. **`mowgli.launch.py`** — hardware layer (always runs first):
   - `robot_state_publisher` (URDF → TF)
   - `hardware_bridge_node` (STM32 serial: publishes `~/status`, `~/power`, `~/emergency`, `~/imu/data_raw`, `~/wheel_odom`; remapped to `/status`, `/power`, `/emergency`, `/imu/data`, `/wheel_odom`)
   - `twist_mux` (merges `/cmd_vel` sources → `/hardware_bridge/cmd_vel`)

2. **`full_system.launch.py`** — full nav stack (includes mowgli.launch.py):
   - `behavior_tree_node` (BT executor, publishes `/behavior_tree_node/high_level_status`)
   - `map_server_node` (area CRUD, docking point, GridMap layers)
   - `coverage_planner_node` (Fields2Cover v2 direct integration)
   - `docking_server` (opennav_docking, SimpleChargingDock) + `lifecycle_manager_docking`
   - `wheel_odometry_node` → `navsat_to_absolute_pose_node` → `gps_pose_converter_node` → `localization_monitor_node`
   - Nav2 stack (via `nav2_bringup` include)
   - `diagnostics_node`, `mqtt_bridge_node` (optional)
   - `foxglove_bridge` (ws://0.0.0.0:8765)
   - `rosbridge_websocket` (ws://0.0.0.0:9090, conditional on `enable_rosbridge`)
   - `obstacle_tracker_node` (persistent LiDAR obstacle detection)

## Key Topics

All Mowgli-specific topics are now namespaced under `/mowgli/` for clean separation from Nav2/system topics.

| Topic | Type | Source | Rate |
|-------|------|--------|------|
| `/status` | `mowgli_interfaces/msg/Status` | hardware_bridge (remapped) | ~10 Hz |
| `/power` | `mowgli_interfaces/msg/Power` | hardware_bridge (remapped) | ~1 Hz |
| `/emergency` | `mowgli_interfaces/msg/Emergency` | hardware_bridge (remapped) | ~1 Hz |
| `/imu/data` | `sensor_msgs/msg/Imu` | hardware_bridge (remapped) | ~50 Hz |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | wheel_odometry_node | ~50 Hz |
| `/mowgli/gps/fix` | `sensor_msgs/msg/NavSatFix` | ublox_gps_node (remapped) | ~5 Hz |
| `/mowgli/gps/absolute_pose` | `mowgli_interfaces/msg/AbsolutePose` | navsat_to_absolute_pose | ~5 Hz |
| `/mowgli/gps/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | gps_pose_converter | ~5 Hz |
| `/mowgli/gps/pose_sim` | `geometry_msgs/msg/PoseWithCovarianceStamped` | gps_degradation_sim | ~5 Hz |
| `/mowgli/slam/heading` | `geometry_msgs/msg/PoseWithCovarianceStamped` | slam_heading_node | ~5 Hz |
| `/mowgli/localization/mode` | `std_msgs/msg/String` | localization_monitor | ~10 Hz |
| `/mowgli/localization/mode_id` | `std_msgs/msg/Int32` | localization_monitor | ~10 Hz |
| `/mowgli/dock/pose_fix` | `geometry_msgs/msg/PoseWithCovarianceStamped` | hardware_bridge | on dock |
| `/mowgli/hardware/status` | `mowgli_interfaces/msg/Status` | hardware_bridge (remapped) | ~10 Hz |
| `/mowgli/hardware/emergency` | `mowgli_interfaces/msg/Emergency` | hardware_bridge (remapped) | ~1 Hz |
| `/mowgli/hardware/power` | `mowgli_interfaces/msg/Power` | hardware_bridge (remapped) | ~1 Hz |
| `/mowgli/hardware/mower_control` | service | hardware_bridge | on demand |
| `/mowgli/behavior/status` | `mowgli_interfaces/msg/HighLevelStatus` | behavior_tree_node (remapped) | on BT tick |
| `/mowgli/coverage/plan` | action | coverage_planner_node (remapped) | on demand |
| `/mowgli/map/get_area` | service | map_server_node (remapped) | on demand |
| `/mowgli/obstacles/tracked` | `mowgli_interfaces/msg/ObstacleArray` | obstacle_tracker_node (remapped) | ~1 Hz |
| `/mowgli/obstacles/markers` | `visualization_msgs/msg/MarkerArray` | obstacle_tracker_node (remapped) | ~1 Hz |
| `/mowgli/obstacles/save` | service | obstacle_tracker_node (remapped) | on demand |
| `/mowgli/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | diagnostics_node | ~1 Hz |
| `/odometry/filtered_map` | `nav_msgs/msg/Odometry` | robot_localization (ekf_map) | ~20 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver / Gazebo | ~10 Hz |

**Note:** Topics published on `~/` (node-relative) in C++ source are remapped to `/mowgli/` paths in launch files. The C++ source remains unchanged for relative names.

## mowgli_interfaces

### Messages (12)
`AbsolutePose`, `CoveragePath`, `ESCStatus`, `Emergency`, `HighLevelStatus`, `ImuRaw`, `MapArea`, `ObstacleArray`, `Power`, `Status`, `TrackedObstacle`, `WheelTick`

**Note:** `DockingSensor.msg` does NOT exist yet. The old OpenMower had it in `mower_msgs` but it has not been ported. Do not subscribe to `/mower/docking_sensor` — it will cause rosbridge type resolution errors.

### Services (9)
`AddMowingArea`, `ClearMap`, `ClearObstacle`, `EmergencyStop`, `GetMowingArea`, `HighLevelControl`, `MowerControl`, `SetDockingPoint`, `TriggerReplan`

### AbsolutePose Flags (important for GUI)
- `FLAG_GPS_RTK = 1` — has GPS fix (poorly named; does NOT mean actual RTK)
- `FLAG_GPS_RTK_FIXED = 2` — RTK fixed (centimetre accuracy)
- `FLAG_GPS_RTK_FLOAT = 4` — RTK float (decimetre accuracy)
- `FLAG_GPS_DEAD_RECKONING = 8` — dead reckoning fallback

## Architecture

```
Behavior Tree (main_tree.xml)
  |-- Emergency / Rain / Battery guards (reactive, highest priority)
  |-- Mowing sequence:
  |     UndockRobot (opennav_docking) -> PlanCoverage -> ExecuteSwathBySwath
  |     ExecuteSwathBySwath per swath: NavigateToPose (transit) -> FollowPath (mow)
  |     Mid-swath obstacle: cancel FollowPath -> NavigateToPose (reroute 1.5m) -> resume FollowPath
  |-- Recovery: 3 retries with ClearCostmap + BackUp between attempts
  |-- On completion or failure: SaveSlamMap -> DockRobot (opennav_docking)

Localization (dual EKF):
  ekf_odom (50Hz): wheel_odom velocity + IMU yaw -> odom->base_link
  ekf_map  (20Hz): filtered_odom velocity + GPS pose if fixed or SLAM when GPS float -> map->odom

Navigation:
  Transit:   RotationShimController wrapping RegulatedPurePursuit (FollowPath)
  Coverage:  RegulatedPurePursuit bare (FollowCoveragePath)
  Planner:   SmacPlanner2D
  Costmaps:  ObstacleLayer (LiDAR /scan) + InflationLayer

Obstacle Tracking:
  obstacle_tracker_node subscribes to /scan, promotes transient detections to
  PERSISTENT after age/observation thresholds, publishes ObstacleArray, feeds
  map_server_node for costmap updates and coverage replanning.
```

## openmower-gui Integration

The GUI lives in `../openmower-gui/` (Go backend + React/Vite frontend).

### Backend → rosbridge connection
- Go backend connects to rosbridge at `ws://<ROSBRIDGE_URL>:9090`
- Subscribes to ROS2 topics via rosbridge v2 JSON protocol (NOT CBOR — `compression: "none"` is explicit to avoid a known rosbridge Jazzy bug with fixed-size arrays like covariance matrices)
- Topic mapping is in `pkg/providers/ros.go` (`topicMap`)
- Adapters in `pkg/providers/transform.go` convert Odometry → AbsolutePose format for the frontend

### Settings
- GUI settings use snake_case YAML keys (e.g., `datum_lon`, `datum_lat`, `tool_width`, `battery_full_voltage`, `battery_empty_voltage`, `battery_capacity_mah`)
- All legacy `OM_*` environment variable keys have been removed from the frontend
- Settings loaded via `useSettings` hook from both shell config and YAML endpoints

### Known rosbridge issues
- **rosbridge CBOR bug (Jazzy):** `'bytes' object has no attribute 'get_fields_and_field_types'` — triggered by `float64[36]` covariance arrays in Odometry/Imu messages. **Fixed** both client-side (`compression: "none"`) and server-side (`scripts/patch_rosbridge.py` patches `cbor_conversion.py` in the Docker image). Rebuild image to apply.
- **`/initialpose` type conflict:** nav2 advertises as `PoseWithCovarianceStamped` but another client registered it as `PoseStamped`. Not a GUI issue — comes from nav2/foxglove interaction.
- **foxglove `/rtcm` schema error:** **Fixed** — `ros-jazzy-rtcm-msgs` added to Dockerfile base stage. Rebuild image to apply.

## Key Config Files

All live-editable via docker-compose.override.yml bind-mounts (no rebuild needed):

| File | What it controls |
|------|-----------------|
| `src/mowgli_bringup/config/nav2_params.yaml` | All Nav2 params: controllers, planner, costmaps, collision monitor, velocity smoother |
| `src/mowgli_bringup/config/coverage_planner.yaml` | F2C parameters: tool_width, spacing, headland, turning radius |
| `src/mowgli_localization/config/localization.yaml` | Dual EKF tuning, GPS converter, wheel odometry |
| `src/mowgli_bringup/config/obstacle_tracker.yaml` | LiDAR obstacle detection thresholds, promotion criteria |
| `src/mowgli_behavior/trees/main_tree.xml` | BT structure (guards, mowing sequence, recovery) |
| `src/mowgli_simulation/models/mowgli_mower/model.sdf` | Gazebo robot model (LiDAR, IMU, diff_drive) -- NOT bind-mounted, needs image rebuild |

## Docker

### Dockerfile stages
1. `base` — ros:jazzy-ros-base + all apt deps (nav2, slam_toolbox, rosbridge-suite, foxglove-bridge, etc.)
2. `deps` — build tools + rosdep + Fields2Cover v2.0.0 from source
3. `build-interfaces` — builds mowgli_interfaces only (cached layer)
4. `build` — builds all remaining packages
5. `runtime` — minimal image with compiled install tree + entrypoint
6. `simulation` — extends runtime with Gazebo Harmonic + VNC

### Services

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

### Docking: opennav_docking (SimpleChargingDock)
Docking and undocking use opennav_docking's action servers via DockRobot/UndockRobot BT nodes. The docking_server runs with its own lifecycle manager. SimpleChargingDock plugin handles staging, approach, and contact alignment. RecordUndockStart + CalibrateHeadingFromUndock still run for GPS heading calibration after undock.

### Progress Checker: single with long timeout
Nav2 Jazzy's bt_navigator sends empty `progress_checker_id` for NavigateToPose. Multiple progress checker plugins cause failures. Use one checker with `movement_time_allowance: 3600s` (effectively disabled for long coverage runs).

### Collision Monitor: enabled with self-reflection mitigation
The Gazebo LiDAR model.sdf has `range_min=0.35m` to filter chassis self-reflections (~0.26-0.30m). PolygonStop and FootprintApproach are both enabled in nav2_params.yaml. The scan source `obstacle_min_range: 0.35` in costmaps also filters self-reflections.

### Obstacles: navigate around, not just stop
Obstacles detected by LiDAR are tracked by obstacle_tracker_node and promoted to persistent after meeting age/observation thresholds. The SLAM map must persist obstacles so replanning routes around them. ExecuteSwathBySwath implements mid-swath obstacle rerouting: when stuck/aborted during a swath, it cancels FollowPath, uses NavigateToPose (SmacPlanner2D) to route 1.5m past the obstacle along the swath, then resumes FollowPath for the remaining swath. Up to 2 reroute attempts per swath before skipping.

## Known Issues & TODOs

### Active Issues
- **BT tick log flooding:** PublishHighLevelStatus oscillates IDLE<->SUCCESS every ~600ms in idle state, generating ~444K log lines per 40 min. Needs throttling or conditional publishing.
- **Gazebo model not bind-mounted:** Changes to model.sdf require Docker image rebuild. Mounting the models/ directory breaks Gazebo sensor initialization (suspected permission or path resolution issue).
- **distance_to_goal feedback:** FollowPath reports distance_to_goal starting at full path length (~1395m) which is correct but initially misleading.

### TODO: High Priority
- [ ] Verify full 13329-pose coverage path completion in simulation (RPP + disabled collision monitor)
- [ ] Wire BT dock_pose from map_server_node/docking_pose topic instead of static parameter (map_server publishes `~/docking_pose`, but BT still reads a static `dock_pose` parameter in behavior_tree_node.cpp:168)
- [ ] Reduce BT log verbosity — PublishHighLevelStatus::tick() publishes every tick with no throttling (action_nodes.cpp:205-254)
- [ ] Add DockingSensor.msg to mowgli_interfaces (port from old mower_msgs/DockingSensor.msg: stamp, detected_left, detected_right)
- [ ] Wire `mowgli_robot.yaml` centralized config into all launch files (file exists and is loaded in full_system.launch.py:131 but params not used to override other configs)
- [ ] FTCController integration — plugin exists (ftc_controller.cpp) but nav2_params still uses RPP for both FollowPath and FollowCoveragePath
- [x] Wire opennav_docking into BT — DockRobot/UndockRobot BT nodes replace BackUp undock and NavigateToPose dock

### TODO: Medium Priority
- [ ] Test rain and battery dock/resume flows end-to-end in simulation (BT guards exist, no automated tests)
- [ ] GPS + odom fusion tuning on real hardware (field testing required; simulation tuning done)
- [x] Install `rtcm_msgs` package — added `ros-jazzy-rtcm-msgs` to Dockerfile base stage

### Completed
- [x] Implement zone management: costmap filter mask publisher for keepout/speed zones
- [x] Area recording: GUI → Go backend → ROS2 map_server_node services (add_area, set_docking_point, clear_map)
- [x] Area persistence: auto-save/load areas and docking point to /ros2_ws/maps/areas.dat
- [x] Obstacle tracker: persistent LiDAR obstacle detection with promotion thresholds (obstacle_tracker_node)
- [x] GUI topic mapping: all topics aligned with actual ROS2 topic names after hardware_bridge remapping
- [x] GUI settings: migrated from OM_* env vars to snake_case YAML keys
- [x] Rosbridge CBOR fix: explicit `compression: "none"` in GUI client + server-side patch in Dockerfile (scripts/patch_rosbridge.py)
- [x] Rosbridge reconnect: stores msgType per topic for correct resubscription
- [x] Removed DockingSensor subscription from GUI backend (msg doesn't exist, caused rosbridge errors)
- [x] Fix Gazebo LiDAR self-reflections: range_min set to 0.35m in model.sdf
- [x] Stuck detection: ExecuteSwathBySwath::checkStuck() in coverage_nodes.cpp (10s timeout, 0.05m threshold)
- [x] SLAM map persistence: lifelong mode + SaveSlamMap action before dock, saves to /ros2_ws/maps/garden_map
- [x] Mow progress tracking: map_server_node publishes ~/mow_progress OccupancyGrid
- [x] Obstacle avoidance: obstacle_tracker promotes detections to persistent, feeds costmap for inflation-based avoidance
- [x] Mid-swath obstacle rerouting: ExecuteSwathBySwath REROUTING_AROUND_OBSTACLE phase — NavigateToPose around obstacle, then resume FollowPath
- [x] opennav_docking integration: DockRobot/UndockRobot BT nodes, docking_server with SimpleChargingDock plugin, own lifecycle manager
- [x] Simulation docker-compose: docker/docker-compose.simulation.yaml with simulation, dev-sim, and simulation-gui services
- [ ] Install `rtcm_msgs` package to silence foxglove_bridge schema error

## Development Workflow

### Git workflow: feature branches + PRs
**NEVER commit directly to main.** Always work on a feature branch and create a PR:
```bash
git checkout main && git pull
git checkout -b feat/my-feature    # or fix/, refactor/, test/
# ... make changes ...
git add <files> && git commit
gh pr create --title "feat: my feature" --body "..."
```
Branch naming: `feat/`, `fix/`, `refactor/`, `test/`, `chore/`, `docs/`

### Edit config/launch (no rebuild with --symlink-install):
```bash
# Edit files under src/mowgli_bringup/config/ or src/mowgli_behavior/trees/
# Restart the simulation to pick up changes (Ctrl-C + make sim)
```

### Edit C++ source (rebuild required):
```bash
make build                              # rebuild all
make build-pkg PKG=mowgli_behavior      # rebuild one package
# Then restart the simulation
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

### GUI development:
```bash
# Backend (Go):
cd ../openmower-gui && ROSBRIDGE_URL=ws://<robot-ip>:9090 go run .

# Frontend (Vite dev server with proxy to backend on :4006):
cd ../openmower-gui/web && npm run dev
```

## Conventions

- **C++ standard:** C++17, ament_cmake build
- **Naming:** snake_case for files/params, CamelCase for C++ classes
- **Frames:** `map` (global), `odom` (local), `base_link` (robot), `lidar_link`, `imu_link`
- **Units:** metres, radians, seconds (SI throughout)
- **Config:** YAML files under each package's `config/` directory
- **Topics:** namespaced under node name (`~/`) for internal, absolute for shared (`/scan`, `/cmd_vel`)
- **Settings keys:** snake_case (e.g., `datum_lon`, `tool_width`, `battery_full_voltage`). No `OM_*` prefix.
