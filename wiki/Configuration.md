# Configuration Reference

Complete guide to all configuration files and parameters in the Mowgli ROS2 system.

This documentation is for ROS2 Kilted with Gazebo Harmonic.

[CLAUDE.md](https://github.com/cedbossneo/mowglinext/blob/main/CLAUDE.md) is the authoritative short-form reference. If any section here contradicts it, CLAUDE.md wins.

## Overview

Configuration is centralized in `src/mowgli_bringup/config/`:

```
config/
├── hardware_bridge.yaml          # Serial port, baud, publish rate, IMU cal sample count
├── localization.yaml              # FusionCore UKF: IMU + GPS covariances, ZUPT, adaptive R,
│                                  #   lever arms (imu_*, gnss.*), apply_lever_arm_pre_heading
├── kinematic_icp.yaml             # Kinematic-ICP tuning (voxel, threshold, registration)
│                                  #   for the 2D LD19 profile
├── nav2_params.yaml               # Navigation stack with LiDAR (obstacle layer, FTCController, docking)
├── nav2_params_no_lidar.yaml      # Navigation stack for GPS-only operation
├── twist_mux.yaml                 # Velocity multiplexer priorities
├── foxglove_bridge.yaml           # Foxglove Studio visualization bridge
└── mowgli_robot.yaml              # Centralized robot config (bind-mounted from
                                   #   install/config/mowgli at /ros2_ws/config/)
```

There is **no** `slam_toolbox.yaml`, `ekf_*.yaml`, or `kiss_icp.yaml`. FusionCore (sole UKF localizer) is tuned through `localization.yaml`, and Kinematic-ICP (LiDAR drift correction on a parallel TF tree) is tuned through `kinematic_icp.yaml`.

All YAML files use the ROS2 `ros__parameters` namespace convention. Parameters can be overridden via command-line:

```bash
ros2 launch mowgli_bringup mowgli.launch.py \
    serial_port:=/dev/ttyACM0 \
    use_sim_time:=true
```

---

## 1. foxglove_bridge.yaml

**File:** `src/mowgli_bringup/config/foxglove_bridge.yaml`

**Purpose:** Configure the Foxglove Studio visualization bridge for remote monitoring and debugging.

**Full Example:**

```yaml
foxglove_bridge:
  ros__parameters:
    # Server port for Foxglove Studio client connections
    port: 8765

    # Enable/disable the WebSocket server
    enabled: true

    # Maximum number of concurrent WebSocket connections
    max_connections: 10

    # Message queue size for buffering (prevents dropping data)
    queue_size: 100

    # Send messages to all connected clients
    send_buffer_limit: 10000000      # bytes (~10 MB per client)

    # Subscribed topics (publish to all connected clients)
    subscribed_topics:
      - /scan                         # LiDAR scan
      - /fusion/odom                  # Robot pose estimate from FusionCore
      - /costmap/costmap              # Global costmap
      - /local_costmap/costmap        # Local costmap
      - /path                         # Global plan
      - /cmd_vel                      # Velocity commands
      - /status                        # Hardware status
      - /power                         # Battery voltage
      - /gps/absolute_pose             # GPS absolute pose
      - /imu/data                     # IMU data

    # Camera feed (if available from camera node)
    # camera_topic: /usb_cam/image_raw

    # Custom transformation frame (if needed)
    # tf_frame: map
```

### Key Parameters

#### `port`

- **Type:** integer
- **Default:** `8765`
- **Description:** WebSocket server port for Foxglove Studio connections
- **Note:** Ensure this port is not blocked by firewall on deployment machine

#### `enabled`

- **Type:** boolean
- **Default:** `true`
- **Description:** Enable/disable the Foxglove bridge
- **Use case:** Disable on production for reduced overhead; enable for debugging

#### `max_connections`

- **Type:** integer
- **Default:** `10`
- **Description:** Maximum concurrent WebSocket connections
- **Tuning:** Increase if multiple users need simultaneous access (higher memory overhead)

#### `subscribed_topics`

- **Type:** list of strings
- **Description:** ROS2 topics to stream to Foxglove clients
- **Performance:** More topics = higher network bandwidth and CPU load
- **Typical setup:** Core navigation and hardware status topics (shown above)

---

## 2. hardware_bridge.yaml

**File:** `src/mowgli_bringup/config/hardware_bridge.yaml`

**Purpose:** Configure serial communication with the STM32 firmware.

**Full Example:**

```yaml
hardware_bridge:
  ros__parameters:
    # Serial port device path
    serial_port: "/dev/mowgli"

    # Baud rate (must match firmware configuration)
    baud_rate: 115200

    # Heartbeat transmission rate (Hz)
    # Keeps watchdog on firmware alive; also transmits emergency state
    heartbeat_rate: 4.0

    # Sensor publishing rate (Hz)
    # Controls frequency of /status, /power, /imu/data_raw messages
    publish_rate: 100.0

    # High-level state update rate (Hz)
    # Sends current_mode and gps_quality to firmware (informational)
    high_level_rate: 2.0
```

### Parameter Details

#### `serial_port`

- **Type:** string
- **Default:** `"/dev/mowgli"`
- **Description:** Device path for USB serial connection to STM32
- **Common values:**
  - `/dev/mowgli` – FTDI/CH340 USB-serial adapter
  - `/dev/ttyACM0` – Native STM32 CDC (preferred, more reliable)
  - `/dev/ttyACM1` – Alternative CDC address if multiple USB devices
  - `COM3` (Windows) – Serial port number
- **Note:** Use `ls /dev/tty*` to identify the correct device
- **Tip:** On Linux, persistent device names can be configured via udev rules

#### `baud_rate`

- **Type:** integer
- **Default:** `115200`
- **Description:** Serial port baud rate
- **Must match firmware setting exactly**
- **Performance vs. Latency:**
  - 115200 – Standard, good for 100 Hz sensors (current default)
  - 230400 – Higher throughput (rarely needed for Mowgli)
  - 57600 – Legacy (ROS1 default, not recommended)
- **Note:** USB virtual serial ports are not affected by baud rate in hardware; setting is mainly for consistency

#### `heartbeat_rate`

- **Type:** double (Hz)
- **Default:** `4.0`
- **Range:** 1.0–10.0 Hz typical
- **Description:** Rate at which hardware_bridge sends heartbeat packets to firmware
- **Purpose:**
  - Keeps firmware watchdog alive (typically 500 ms timeout)
  - Transmits emergency stop control bits
  - Transmits emergency release signal (one-shot per service call)
- **Formula:** `heartbeat_period = 1.0 / heartbeat_rate`
  - 4.0 Hz → 250 ms period
  - 2.0 Hz → 500 ms period (risky if firmware watchdog is 500 ms)
- **Tuning:** Increase if firmware reports watchdog timeout; decrease for tighter safety response

#### `publish_rate`

- **Type:** double (Hz)
- **Default:** `100.0`
- **Range:** 10.0–200.0 Hz typical
- **Description:** Rate at which hardware_bridge publishes sensor data to ROS2 topics
- **Affects:** `/hardware_bridge/status`, `/hardware_bridge/power`, `/hardware_bridge/imu/data_raw`
- **Note:** These are the node-local topic names (`~/status`, `~/power`, `~/imu/data_raw`). In `mowgli.launch.py`, they are remapped to `/status`, `/power`, and `/imu/data` respectively.
- **Upstream drivers:** Should match firmware's sensor acquisition rate (typically 100 Hz)
- **Tuning:**
  - Increase for more responsive sensor feedback (higher CPU load on Pi)
  - Decrease to reduce USB serial traffic (may miss fast transients)
  - Typical sweet spot: 50–100 Hz

#### `high_level_rate`

- **Type:** double (Hz)
- **Default:** `2.0`
- **Range:** 1.0–10.0 Hz typical
- **Description:** Rate at which hardware_bridge sends high-level state to firmware
- **Payload:** Current high-level mode (idle/mowing/docking/recording) and GPS RTK quality
- **Purpose:** Firmware uses this for telemetry, sound notifications, LED feedback
- **Tuning:** Lower rate OK (2 Hz sufficient for informational updates)

### Typical Configurations

**High-Performance (Low Latency):**
```yaml
hardware_bridge:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    heartbeat_rate: 10.0      # Fast watchdog feed
    publish_rate: 100.0       # High sensor rate
    high_level_rate: 5.0
```

**Reliable (Conservative):**
```yaml
hardware_bridge:
  ros__parameters:
    serial_port: "/dev/mowgli"
    baud_rate: 115200
    heartbeat_rate: 4.0       # Standard watchdog
    publish_rate: 50.0        # Reduced sensor rate
    high_level_rate: 1.0      # Minimal overhead
```

---

## 2. localization.yaml

**File:** `src/mowgli_bringup/config/localization.yaml`

**Purpose:** Configure FusionCore UKF for sensor fusion (wheel odometry, IMU, GPS).

### Architecture

The system uses **FusionCore**, a single Unscented Kalman Filter (UKF) that fuses:
- Wheel odometry (50 Hz from wheel_odometry_node)
- IMU data (50 Hz from imu_filter_madgwick)
- GPS NavSatFix (10-20 Hz directly from GPS driver)

**Outputs:**
- `/fusion/odom` (nav_msgs/Odometry) — fused state at 100 Hz
- `/tf: odom → base_footprint` — transform at 100 Hz

`map → odom` is a **static identity transform**, published once at launch by `tf2_ros static_transform_publisher` (no SLAM). FusionCore is the only node that publishes `odom → base_footprint`; Kinematic-ICP's output goes into FusionCore via `/encoder2/odom` and never as a TF.

### FusionCore Configuration

```yaml
fusioncore:
  ros__parameters:
    # Frequency (Hz)
    frequency: 50.0

    # Frame names (per REP-105)
    odom_frame: odom
    base_frame: base_footprint            # Robot frame for Nav2
    map_frame: map

    # ─────────────────────────────────────────────────────────────
    # Sensor inputs
    # ─────────────────────────────────────────────────────────────
    
    # Wheel odometry (velocity-based fusion)
    wheel_odometry_topic: /wheel_odom
    
    # IMU (orientation + angular velocity)
    imu_topic: /imu/data
    
    # GPS (absolute position, NavSatFix)
    gps_topic: /gps/fix
    
    # ─────────────────────────────────────────────────────────────
    # Process noise (UKF motion model)
    # ─────────────────────────────────────────────────────────────
    # How much to trust the motion model vs. sensor updates
    
    process_noise_std_x: 0.05          # m, process noise for X position
    process_noise_std_y: 0.05          # m, process noise for Y position
    process_noise_std_theta: 0.06      # rad, process noise for yaw
    process_noise_std_vx: 0.025        # m/s, process noise for X velocity
    process_noise_std_vy: 0.025        # m/s, process noise for Y velocity
    process_noise_std_vtheta: 0.02     # rad/s, process noise for yaw rate
    
    # ─────────────────────────────────────────────────────────────
    # Measurement noise (sensor reliability)
    # ─────────────────────────────────────────────────────────────
    
    # Wheel odometry measurement noise
    wheel_odom_std_vx: 0.02            # m/s
    wheel_odom_std_vy: 0.02            # m/s
    wheel_odom_std_vtheta: 0.01        # rad/s
    
    # IMU measurement noise
    imu_std_theta: 0.05                # rad (yaw from IMU)
    imu_std_vtheta: 0.02               # rad/s (yaw rate)
    
    # GPS measurement noise (depends on RTK quality)
    gps_std_x: 0.01                    # m (RTK Fixed ~1cm)
    gps_std_y: 0.01                    # m
    gps_outlier_threshold: 100.0       # m (reject fixes > 100m from prediction)
```

### Tuning FusionCore

**Concept:** Adjust process noise and measurement noise to balance sensor trust.

**If odometry drifts too much:**
- Increase process noise (motion model less trusted)
- Example: `process_noise_std_x: 0.05` → `0.10`
- Filter will rely more on sensor measurements (IMU, GPS)

**If filter jumps on sensor noise:**
- Increase measurement noise (sensors less trusted)
- Decrease process noise (motion model more trusted)
- Example: `wheel_odom_std_vx: 0.02` → `0.05`

**GPS outlier handling:**
- `gps_outlier_threshold`: reject fixes > this distance from prediction
- Typical: 100 m (accounts for initialization uncertainty)
- Increase if robot is in GPS-denied area at startup
- Decrease to 10-20 m in dense urban canyons

**Typical values (for Mowgli):**
- Position noise: 0.01 m (FusionCore fuses three sources)
- Yaw noise: 0.05 rad (IMU + wheel differential)
- GPS outlier: 100 m (stationary initialization)

**Monitoring FusionCore Health:**
The diagnostics system monitors FusionCore via `/fusion/odom`:
- **Rate:** Checks for 50 Hz update frequency (warn if < 20 Hz)
- **Position variance:** Monitors x, y covariance for convergence
- **Orientation (yaw):** Tracks yaw angle and angular variance
- **Z-drift detection:** Alerts if vertical variance grows uncontrolled (filter divergence)

Access diagnostics at `http://<mower-ip>:4006/#/diagnostics` → FusionCore section

---

## 3. nav2_params.yaml

**File:** `src/mowgli_bringup/config/nav2_params.yaml`

**Purpose:** Configure Nav2 navigation stack (planning, control, costmaps).

### Navigation Stack Overview

```
nav2_bringup (lifecycle manager)
  ├── planner_server (global planner)
  │   └── SmacPlanner2D: plans path from start to goal
  │
  ├── controller_server (local planner + motion controller)
  │   └── RegulatedPurePursuit: follows path and generates motor commands
  │
  ├── bt_navigator (behavior tree)
  │   └── Default Nav2 BT: compute path → follow path → success
  │
  ├── nav2_costmap_2d (obstacle map)
  │   ├── Static layer: /map from SLAM
  │   ├── Obstacle layer: /scan from LiDAR
  │   └── Inflation layer: costmap + inflation radius
  │
  └── nav2_map_server (loads static map, if available)
```

### Key Sections

#### bt_navigator Configuration

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint       # REP-105: ground contact point
    odom_topic: /fusion/odom               # Use FusionCore fused odometry

    # Behavior tree execution
    bt_loop_duration: 10                   # ms per tick (100 Hz)
    default_server_timeout: 20             # seconds to wait for action servers

    # Custom BT with GoalCheckerSelector for dual-mode navigation
    default_nav_to_pose_bt_xml: "src/mowgli_bringup/config/navigate_to_pose.xml"
    default_nav_through_poses_bt_xml: ""

    # Kilted auto-loads plugins; no manual registration needed
```

#### controller_server Configuration

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false

    # Update rate of velocity commands to motors
    controller_frequency: 10.0             # Hz

    # Minimum velocity thresholds (to avoid numerical issues)
    min_x_velocity_threshold: 0.001        # m/s
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001    # rad/s

    # Failure tolerance (stop after this duration of failed path tracking)
    failure_tolerance: 0.3                 # seconds

    # Plugin selection: dual controller setup
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["stopped_goal_checker", "coverage_goal_checker"]
    controller_plugins: ["FollowPath", "FollowCoveragePath"]

    # Enable stamped velocity commands (Kilted requirement)
    enable_stamped_cmd_vel: false

    # Progress checker: has robot moved at least 0.5 m in 10 seconds?
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5        # m
      movement_time_allowance: 10.0        # seconds

    # Goal checker: robot within tolerance of goal?
    stopped_goal_checker:
      plugin: "nav2_controller::StoppedGoalChecker"
      trans_stopped_velocity: 0.25         # m/s
      xy_goal_tolerance: 0.25              # m
      yaw_goal_tolerance: 0.25             # rad

    # Coverage goal checker: for mowing pattern termination
    coverage_goal_checker:
      plugin: "mowgli_nav2_plugins::CoverageGoalChecker"
      coverage_radius: 0.5                 # m
      coverage_tolerance: 0.1              # m

    # ─────────────────────────────────────────────────────────────
    # FollowPath: Rotation-Shim Controller wrapping Regulated Pure Pursuit
    # ─────────────────────────────────────────────────────────────
    FollowPath:
      plugin: "mowgli_nav2_plugins::RotationShimController"
      desired_linear_vel: 0.3              # m/s
      max_linear_vel: 0.5                  # m/s
      max_angular_vel: 1.0                 # rad/s
      use_regulated_linear_velocity_scaling: true
      lookahead_dist: 0.6                  # m
      min_lookahead_dist: 0.3              # m
      max_lookahead_dist: 0.9              # m
      lookahead_time: 1.5                  # seconds
      rotate_to_heading_angular_vel: 1.57  # rad/s (90°/s)
      transform_tolerance: 0.1             # seconds
      use_cost_regulated_linear_velocity_scaling: false

    # ─────────────────────────────────────────────────────────────
    # FollowCoveragePath: Regulated Pure Pursuit for mowing patterns
    # Note: FTCController plugin exists in mowgli_nav2_plugins but is
    # not activated. RPP is used for both FollowPath and FollowCoveragePath
    # because MPPI's Euclidean nearest-point matching jumps between
    # adjacent parallel boustrophedon swaths.
    # ─────────────────────────────────────────────────────────────
    FollowCoveragePath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

      desired_linear_vel: 0.3              # m/s
      max_linear_vel: 0.5                  # m/s
      max_angular_vel: 1.0                 # rad/s
      use_regulated_linear_velocity_scaling: true
      lookahead_dist: 0.6                  # m
      min_lookahead_dist: 0.3              # m
      max_lookahead_dist: 0.9              # m
      lookahead_time: 1.5                  # seconds
      max_robot_pose_search_dist: 5.0      # m (prevents jumping to adjacent swaths)
      transform_tolerance: 0.1             # seconds
      use_cost_regulated_linear_velocity_scaling: false
```

#### planner_server Configuration

```yaml
planner_server:
  ros__parameters:
    use_sim_time: false

    # Expected planner plugins
    planner_plugins: ["GridBased"]

    # ─────────────────────────────────────────────────────────────
    # SmacPlanner2D: A* search on 2D grid
    # ─────────────────────────────────────────────────────────────
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"

      # Planning parameters
      tolerance: 0.125                     # m, tolerance to goal
      downsample_costmap: false            # Use full resolution
      downsampling_factor: 1

      # Search algorithm tuning
      angle_quantization_bins: 72          # 5° angle resolution (360/72)
      maximum_iterations: 1000             # Max A* iterations
      max_planning_time: 5.0               # Max planning time (seconds)

      # Cost function
      w_cost: 100.0                        # Weight for path length vs. safety
      w_heuristic: 100.0                   # Weight for heuristic (A* guidance)

      # Lethal cost threshold (treat as obstacle if > this)
      lethal_cost: 252.0

      # Motion primitives (allowed moves)
      # For a differential drive, typical 16-direction grid
      use_grid_path_stitching: true
```

#### costmap_2d Configuration

```yaml
# Global costmap (used by planner)
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0                # Hz (slower for global map)
      publish_frequency: 1.0               # Hz
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false

      # Plugins (layers)
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      # Static layer: SLAM-generated map
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        subscribe_to_updates: true

      # Obstacle layer: LiDAR detects new obstacles
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.5         # m (tallest obstacle to consider)
          min_obstacle_height: 0.0         # m
          clearing: true                   # LiDAR clears old obstacles
          marking: true                    # LiDAR marks new obstacles
          expected_update_rate: 0.0        # No timeout (use latest)

      # Inflation layer: create safety buffer around obstacles
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.55             # m (half robot width + clearance)
        cost_scaling_factor: 10.0

      # Resolution and bounds
      resolution: 0.05                     # m/pixel (fine resolution for precise paths)
      width: 200                           # Grid width (pixels)
      height: 200                          # Grid height (pixels)
      origin_x: -5.0                       # m (relative to global origin)
      origin_y: -5.0                       # m

# Local costmap (used by controller, smaller window around robot)
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0                # Hz (faster for reactive obstacle avoidance)
      publish_frequency: 5.0               # Hz
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.5
          min_obstacle_height: 0.0
          clearing: true
          marking: true
          expected_update_rate: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.55
        cost_scaling_factor: 10.0

      resolution: 0.05
      width: 50                            # Smaller window: ~2.5 m × 2.5 m
      height: 50
      origin_x: -1.25
      origin_y: -1.25
```

#### velocity_smoother Configuration

The velocity smoother applies acceleration limits to reduce jerky motion:

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0              # Hz
    scale_velocities: false
    feedback: "odometry"
    max_velocity: [0.5, 0.0, 0.8]          # [linear_x, linear_y, angular_z]
    max_accel: [0.4, 0.0, 1.0]             # Acceleration limits (m/s², rad/s²)
    max_decel: [0.4, 0.0, 1.0]             # Deceleration limits
    odom_topic: "/fusion/odom"
    cmd_vel_in_topic: "/cmd_vel"
    cmd_vel_out_topic: "/cmd_vel_smoothed"
```

---

## 4. coverage_planner.yaml

**File:** `src/mowgli_bringup/config/coverage_planner.yaml`

**Purpose:** Configure the B-RV autonomous mowing coverage planner (mowgli_brv_planner).

**Full Configuration:**

```yaml
coverage_planner_node:
  ros__parameters:
    # Tool and grid parameters
    tool_width: 0.18                       # m (mowing blade width and grid cell size)

    # Headland (boundary pass) parameters
    headland_passes: 2                     # Number of passes around the perimeter
    headland_width: 0.18                   # m (offset per pass)

    # Coordinate frame
    map_frame: "map"
```

### Key Parameters

#### `tool_width`

- **Type:** double (m)
- **Default:** `0.18`
- **Description:** Physical width of mowing blade; also used as grid cell size for the B-RV discretization
- **Impact:** Determines sweep lane spacing and coverage resolution

#### `headland_passes`

- **Type:** integer
- **Default:** `2`
- **Description:** Number of passes around yard perimeter before mowing interior
- **Rationale:** Ensures boundary coverage and creates safety margin before interior work

#### `headland_width`

- **Type:** double (m)
- **Default:** `0.18`
- **Description:** Offset distance per headland pass (typically matches tool_width)

---

## 5. kinematic_icp.yaml

**File:** `src/mowgli_bringup/config/kinematic_icp.yaml`

**Purpose:** Tune the Kinematic-ICP (PRBonn 2024) LiDAR drift corrector for the LD19 2D LiDAR. The node runs on a parallel TF tree (`wheel_odom_raw → base_footprint_wheels → lidar_link_wheels`), consumes `/scan_kicp` (a frame-relayed copy of `/scan`), and publishes `/kinematic_icp/lidar_odometry` which `kinematic_icp_encoder_adapter` finite-differences into `/encoder2/odom` for FusionCore's UKF.

**Full Configuration:**

```yaml
/**:
  ros__parameters:
    # Trim below LD19's nominal range — returns past 6 m outdoors are
    # dominated by haze, sun, and moving foliage.
    max_range: 6.0
    # Reject self-reflections off the chassis/wheels.
    min_range: 0.4

    # Voxel size + points per voxel. 2D scans are sparse (~455 pts over 270°);
    # 10 cm voxels see <=1 pt each and adjacent scans drop into different
    # voxels from range jitter. 20 cm gives reliable overlap across scans
    # and averages wind wobble. 5 points/voxel is enough for local plane
    # fits in 2D.
    voxel_size: 0.2
    max_points_per_voxel: 5

    # Adaptive correspondence threshold — scales with observed model error.
    use_adaptive_threshold: true
    fixed_threshold: 0.3

    # Registration
    max_num_iterations: 100
    convergence_criterion: 0.001
    max_num_threads: 1                    # ARM CPU budget; Nav2 needs cores

    use_adaptive_odometry_regularization: true
    fixed_regularization: 0.0

    # LaserScan has no per-point timestamps, so deskew is always off for 2D.
    deskew: false

    # Reported on /kinematic_icp/lidar_odometry; the adapter overrides the
    # twist covariance before republishing on /encoder2/odom.
    orientation_covariance: 0.1
    position_covariance: 0.1
```

### Tuning notes

| Symptom | Try |
|---------|-----|
| Lots of "extrapolation into the future" TF warnings | Raise `wheel_odom_tf_node` `rebroadcast_hz` (default 50), or bump `tf_timeout` in the K-ICP launch file (default 0.1 s). |
| K-ICP output drifts at rest | Voxel map may be accumulating wind-moved foliage. Lower `max_points_per_voxel` (3–4), or reduce `max_range`. |
| K-ICP output over-corrects during turns | Increase `use_adaptive_odometry_regularization` weight by raising `fixed_regularization` and disabling `use_adaptive_odometry_regularization`. |
| Encoder2 fixes rejected by FusionCore | Inspect Mahalanobis innovations (`fusioncore_node` logs). Usually means the scene is too feature-poor and K-ICP's twist is inconsistent with the wheel prior — reduce `voxel_size` or increase `fixed_regularization`. |

See [`CLAUDE.md`](https://github.com/cedbossneo/mowglinext/blob/main/CLAUDE.md) invariant #1 for why K-ICP runs on a parallel TF tree rather than sharing FusionCore's frames (the short version: it prevents the corrector's output from feeding back into its own motion prior).

---

## 6. twist_mux.yaml

**File:** `src/mowgli_bringup/config/twist_mux.yaml`

**Purpose:** Priority-based multiplexing of velocity commands from multiple sources.

**Full Configuration:**

```yaml
twist_mux:
  ros__parameters:
    # Input topics (velocity sources)
    # Topics evaluated in ascending order of priority
    # Higher priority sources suppress lower priority when active
    topics:
      # Lowest priority: autonomous navigation
      navigation:
        topic: /cmd_vel_nav
        timeout: 0.5                 # seconds (stale if no message for 0.5 s)
        priority: 10                 # Lowest priority

      # Medium priority: manual teleoperation
      teleop:
        topic: /cmd_vel_teleop
        timeout: 0.5
        priority: 20                 # Override navigation

      # Highest velocity priority: emergency commands
      emergency:
        topic: /cmd_vel_emergency
        timeout: 0.2                 # Tighter timeout for safety-critical
        priority: 100                # Highest velocity priority

    # Locks: hard stops that disable all velocity output
    # Regardless of velocity topics, output is zero if any lock is active
    locks:
      emergency_stop:
        topic: /emergency_stop       # Must be boolean or std_msgs/Bool
        timeout: 0.0                 # 0 = never timeout (stay locked until explicit release)
        priority: 255                # Absolute: highest possible priority
```

### Priority Resolution

**Example Scenario:**

```
Time 0: Navigation publishes cmd_vel_nav (0.1 m/s)
  → Output: 0.1 m/s (only source active)

Time 1: Teleop publishes cmd_vel_teleop (0.2 m/s)
  → Output: 0.2 m/s (teleop priority 20 > navigation priority 10)

Time 2: Emergency publishes cmd_vel_emergency (0.3 m/s)
  → Output: 0.3 m/s (emergency priority 100 > all others)

Time 3: All sources timeout or become stale
  → Output: 0 m/s (no active source, robot stops)

Time 4: Emergency stop lock engaged
  → Output: 0 m/s (lock overrides all, even with active velocity sources)
  → Must call service to unlock before any motion possible
```

### Service Interface

```bash
# Check current mux state
ros2 service call /twist_mux/get_status std_srvs/GetBool

# Lock the mux (emergency stop)
ros2 service call /emergency_stop std_srvs/SetBool "{data: true}"

# Unlock the mux
ros2 service call /emergency_stop std_srvs/SetBool "{data: false}"
```

### Typical Deployments

**Autonomous Mowing (Default):**
```yaml
# High-priority emergency source, low-priority autonomous
# Allows emergency override at any time
```

**Teleoperation (Manual Control):**
```yaml
topics:
  teleop:
    priority: 10
  emergency:
    priority: 100
# Remove navigation source entirely
```

---

## Parameter Tuning Workflow

### Step 1: Identify Performance Issue

| Issue | Likely Culprit | Action |
|-------|----------------|--------|
| Robot drifts without sensor updates | EKF process noise too low | Increase `process_noise_covariance` in localization.yaml |
| Robot ignores GPS corrections | EKF process noise too high | Decrease `process_noise_covariance` |
| Path tracking oscillates | Lookahead or velocity scaling too aggressive | Adjust `lookahead_dist`, `desired_linear_vel` in RegulatedPurePursuit |
| Robot can't follow curves | Lookahead distance too short | Increase `lookahead_dist` in nav2_params.yaml |
| Planner is very slow | Grid resolution too fine or search space too large | Increase resolution (0.05 → 0.10) or reduce grid size |
| SLAM diverges in loop closure | Loop closure parameters too aggressive | Decrease `loop_match_minimum_chain_size`, increase `loop_search_maximum_distance` |

### Step 2: Modify Parameters

Edit the relevant YAML file:
```bash
nano src/mowgli_bringup/config/localization.yaml
```

### Step 3: Test and Monitor

```bash
# Launch with new parameters
ros2 launch mowgli_bringup mowgli.launch.py

# Monitor topics in RViz
rviz2 -d src/mowgli_bringup/config/mowgli.rviz

# Check diagnostics
ros2 topic echo /localization/status
ros2 topic echo /fusion/odom
```

### Step 4: Iterate

Rerun with adjusted parameters, observe results, adjust again.

---

## Reference: Default Parameters Quick Lookup

| Parameter | Default | Unit | Range |
|-----------|---------|------|-------|
| `serial_port` | `/dev/mowgli` | – | any `/dev/tty*` |
| `baud_rate` | 115200 | baud | 9600–230400 |
| `FusionCore` frequency | 50.0 | Hz | 30–100 |
| `SLAM` frequency | 20.0 | Hz | 5–50 |
| `controller_frequency` | 10.0 | Hz | 5–50 |
| `desired_linear_vel` | 0.3 | m/s | 0.1–1.0 |
| `lookahead_dist` | 0.6 | m | 0.2–1.5 |
| `linear_p_gain` | 2.0 | – | 0.5–5.0 |
| `slam_resolution` | 0.05 | m/pixel | 0.01–0.20 |
| `loop_search_max_distance` | 3.0 | m | 1.0–5.0 |

---

**For detailed system architecture, see [ARCHITECTURE.md](ARCHITECTURE.md).**

**For firmware integration, see [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md).**
