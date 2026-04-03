# Configuration Reference

Complete guide to all configuration files and parameters in the Mowgli ROS2 system.

This documentation is for ROS2 Jazzy with Gazebo Harmonic.

## Overview

Configuration is centralized in `src/mowgli_bringup/config/`:

```
config/
├── hardware_bridge.yaml      # Serial protocol parameters
├── localization.yaml          # Dual EKF tuning
├── nav2_params.yaml           # Navigation stack (planner, controller, costmap)
├── coverage_planner.yaml      # Coverage path planning parameters
├── slam_toolbox.yaml          # SLAM parameters
├── twist_mux.yaml             # Velocity multiplexer priorities
├── foxglove_bridge.yaml       # Foxglove Studio visualization bridge
├── obstacle_tracker.yaml      # LiDAR obstacle detection thresholds
└── mowgli_robot.yaml          # Centralized robot config (exists but not fully wired)
```

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
      - /odometry/filtered_map        # Robot pose estimate
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

**Purpose:** Tune the dual Extended Kalman Filter (EKF) for localization.

### Architecture

The system uses two EKF instances (from `robot_localization` package):

1. **ekf_odom** (50 Hz)
   - Inputs: Wheel odometry + IMU
   - Output: `odom` → `base_link` frame
   - Local estimation, fast update rate

2. **ekf_map** (20 Hz)
   - Inputs: Filtered odometry (from ekf_odom) + GPS
   - Output: `map` → `odom` frame
   - Global estimation, slower due to GPS latency

### ekf_odom Configuration

```yaml
ekf_odom:
  ros__parameters:
    # Update rate (Hz)
    frequency: 50.0

    # Timeout: if sensor hasn't published in this time, treat as stale
    sensor_timeout: 0.1       # 100 ms

    # 2D mode: constrain Z and roll/pitch to zero (lawn mowing is planar)
    two_d_mode: true

    # Publish TF updates (odom → base_link)
    publish_tf: true

    # Frame names
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom         # Local frame (odom_frame is the world)

    # ─────────────────────────────────────────────────────────────
    # Sensor configuration: wheel odometry
    # ─────────────────────────────────────────────────────────────
    odom0: /wheel_odom

    # Which state dimensions to fuse from /wheel_odom
    # [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    # We fuse only linear velocity (vx, vy) and angular velocity (vyaw)
    odom0_config: [false, false, false,      # Position (x, y, z)
                   false, false, false,      # Orientation (roll, pitch, yaw)
                   true,  true,  false,     # Velocity (vx, vy, vz)
                   false, false, true,      # Angular velocity (vroll, vpitch, vyaw)
                   false, false, false]     # Acceleration (ax, ay, az)

    odom0_queue_size: 10
    odom0_nodelay: false
    odom0_differential: false
    odom0_relative: false

    # ─────────────────────────────────────────────────────────────
    # Sensor configuration: IMU (for attitude and angular velocity)
    # ─────────────────────────────────────────────────────────────
    imu0: /imu/data

    # Fuse absolute yaw (from compass/gyro fusion) and yaw rate
    imu0_config: [false, false, false,
                  false, false, true,       # Yaw orientation
                  false, false, false,
                  false, false, true,       # Yaw rate
                  false, false, false]

    imu0_queue_size: 10
    imu0_nodelay: false
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true

    # ─────────────────────────────────────────────────────────────
    # Filter tuning: process noise covariance
    # ─────────────────────────────────────────────────────────────
    # This is a 15×15 matrix (one variance per state dimension).
    # Higher values = less trust in the motion model.
    # If odometry drifts too much, increase these values.
    # If filter ignores sensor inputs, decrease these values.

    process_noise_covariance: [
      0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015
    ]
```

### ekf_map Configuration

```yaml
ekf_map:
  ros__parameters:
    frequency: 20.0           # Slower due to GPS latency
    sensor_timeout: 0.2       # Longer timeout window
    two_d_mode: true
    publish_tf: true

    odom_frame: odom
    base_link_frame: base_link
    world_frame: map          # Global frame

    # ─────────────────────────────────────────────────────────────
    # Fuse filtered odometry from ekf_odom (full pose + velocity)
    # ─────────────────────────────────────────────────────────────
    odom0: /odometry/filtered_odom

    odom0_config: [true,  true,  false,      # Position (x, y, z)
                   false, false, true,       # Orientation (yaw only)
                   true,  true,  false,     # Velocity (vx, vy)
                   false, false, true,      # Angular velocity (vyaw)
                   false, false, false]

    odom0_queue_size: 10
    odom0_nodelay: false
    odom0_differential: false
    odom0_relative: false

    # ─────────────────────────────────────────────────────────────
    # Fuse GPS pose (from gps_pose_converter_node)
    # ─────────────────────────────────────────────────────────────
    pose0: /gps/pose

    # GPS gives us absolute X, Y position (in local ENU)
    # But not orientation or velocity (GPS gives velocity but not reliable for 0.25 m field)
    pose0_config: [true,  true,  false,
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]

    pose0_queue_size: 10
    pose0_nodelay: false
    pose0_differential: false
    pose0_relative: false

    # Process noise (looser than odom EKF due to GPS latency and variable accuracy)
    process_noise_covariance: [
      0.10, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0.10, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015
    ]
```

### Tuning Process Noise

**Concept:** Process noise represents trust in the motion model (physics).

**If odometry drifts too much:**
- Increase process noise covariance (e.g., 0.05 → 0.10)
- Tells EKF "the robot isn't moving as predictably as we think"
- Filter will rely more heavily on sensor measurements

**If filter ignores sensor inputs:**
- Decrease process noise covariance (e.g., 0.05 → 0.02)
- Tells EKF "we predict motion quite accurately"
- Filter will trust the motion model over noisy sensor data

**Typical values (for Mowgli):**
- Position: 0.05–0.10 (odometry is noisy on grass)
- Orientation: 0.03–0.06 (IMU is reasonably accurate)
- Velocity: 0.025–0.04 (derived from position, inherits noise)

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
    robot_base_frame: base_link
    odom_topic: /odometry/filtered_map     # Use the fused GPS-corrected odometry

    # Behavior tree execution
    bt_loop_duration: 10                   # ms per tick (100 Hz)
    default_server_timeout: 20             # seconds to wait for action servers

    # Custom BT with GoalCheckerSelector for dual-mode navigation
    default_nav_to_pose_bt_xml: "src/mowgli_bringup/config/navigate_to_pose.xml"
    default_nav_through_poses_bt_xml: ""

    # Jazzy auto-loads plugins; no manual registration needed
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

    # Enable stamped velocity commands (Jazzy requirement)
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
    odom_topic: "/odometry/filtered_map"
    cmd_vel_in_topic: "/cmd_vel"
    cmd_vel_out_topic: "/cmd_vel_smoothed"
```

---

## 4. coverage_planner.yaml

**File:** `src/mowgli_bringup/config/coverage_planner.yaml`

**Purpose:** Configure the autonomous mowing coverage planner.

**Full Configuration:**

```yaml
coverage_planner:
  ros__parameters:
    # Headland (boundary pass) parameters
    headland_passes: 3                     # Number of passes around the perimeter
    headland_width: 0.5                    # m (how far from boundary)

    # Tool and cutting parameters
    tool_width: 0.55                       # m (mowing deck width)
    cutting_height: 0.08                   # m (grass cutting height)

    # Mowing pattern
    mowing_angle: 0.0                      # rad (direction of mowing rows; 0 = North-South)
    mowing_angle_variance: 0.2             # rad (random offset per pass)
    row_spacing: 0.5                       # m (distance between passes)

    # Robot constraints
    min_turning_radius: 0.4                # m (minimum curvature radius)
    max_angular_vel: 1.0                   # rad/s
    goal_tolerance: 0.3                    # m

    # Optimization
    use_coverage_goal_checker: true        # Use CoverageGoalChecker instead of StoppedGoalChecker
    optimize_path_smoothness: true
```

### Key Parameters

#### `headland_passes`

- **Type:** integer
- **Default:** `3`
- **Description:** Number of passes around yard perimeter before mowing interior
- **Rationale:** Ensures boundary coverage and creates safety margin before interior work

#### `tool_width`

- **Type:** double (m)
- **Default:** `0.55`
- **Description:** Physical width of mowing deck
- **Use case:** Row spacing auto-calculated as `tool_width * 0.95` (5% overlap for safety)

#### `mowing_angle`

- **Type:** double (radians)
- **Default:** `0.0`
- **Range:** `0.0` to `6.28` (0° to 360°)
- **Description:** Preferred direction of mowing rows (0 = North-South, π/2 = East-West)
- **Tuning:** Adjust based on field topography to minimize drift on slopes

#### `min_turning_radius`

- **Type:** double (m)
- **Default:** `0.4`
- **Description:** Minimum radius the robot can execute (physical constraint)
- **Must match:** Robot wheelbase and max angular velocity: `radius = linear_vel / angular_vel`

---

## 5. slam_toolbox.yaml

**File:** `src/mowgli_bringup/config/slam_toolbox.yaml`

**Purpose:** Configure SLAM (Simultaneous Localization and Mapping) for outdoor operation.

**Full Configuration:**

```yaml
slam_toolbox:
  ros__parameters:
    # General
    use_sim_time: false
    mode: localization                    # or "mapping" for recording phase

    # Frame setup
    map_start_at_origin: false
    map_frame: map
    base_frame: base_link
    odom_frame: odom
    resolution: 0.05                      # 5 cm resolution

    # ─────────────────────────────────────────────────────────────
    # Laser scan processing
    # ─────────────────────────────────────────────────────────────
    scan_topic: /scan
    scan_queue_size: 10

    # Maximum range (ignore returns beyond this)
    maximum_laser_range: 25.0             # m

    # Angular range (typically full 360° or limited for efficiency)
    minimum_scan_angle: -3.14             # rad (-180°)
    maximum_scan_angle: 3.14              # rad (+180°)

    # Minimum range (ignore returns closer than this, avoid self-reflection)
    minimum_laser_range: 0.15             # m

    # ─────────────────────────────────────────────────────────────
    # Motion model (for incremental updates)
    # ─────────────────────────────────────────────────────────────
    # SLAM uses odometry as motion prior; update if odometry drifts
    transform_tolerance: 0.1              # seconds

    # ─────────────────────────────────────────────────────────────
    # Map update parameters
    # ─────────────────────────────────────────────────────────────
    # When to update the map (conservative for outdoor to avoid noise)
    map_update_interval: 5.0              # seconds

    # Minimum translation before new scan is added to map
    minimum_travel_distance: 0.5          # m

    # Minimum rotation before new scan is added to map
    minimum_travel_heading: 0.5           # rad (~30°)

    # ─────────────────────────────────────────────────────────────
    # Loop closure: detecting when robot revisits a location
    # ─────────────────────────────────────────────────────────────
    enable_loop_closure: true

    # Only close loops if robot is within this distance
    loop_search_maximum_distance: 3.0     # m (conservative for small yards)

    # Number of consecutive scans to form a "chain" before loop closure
    loop_match_minimum_chain_size: 10     # scans (avoids spurious closures)

    # Minimum loop closure match score (0.0–1.0, higher = more confident)
    loop_match_minimum_response_fine: 0.1 # Fine scan matching threshold
    loop_match_minimum_response_coarse: 0.05

    # ─────────────────────────────────────────────────────────────
    # Scan matching (registration of consecutive scans)
    # ─────────────────────────────────────────────────────────────
    # Determine pose delta between consecutive scans

    # Correlation window (search area for matching)
    correlation_search_space_dimension: 0.5  # m
    correlation_search_space_resolution: 0.01 # m

    # Search size for finer correlation
    correlation_search_space_smear_deviation: 0.1 # m

    # ─────────────────────────────────────────────────────────────
    # Optimization: graph refinement (Pose Graph Optimization)
    # ─────────────────────────────────────────────────────────────
    enable_interactive_mode: false        # Don't use interactive SLAM

    # Optimization parameters
    do_loop_closure_optimization: true
    do_scan_matching_optimization: true
    optimization_angle_variance: 0.0349   # rad² (0.2° std dev)
    optimization_translation_variance: 0.01 # m² (0.1 m std dev)

    # ─────────────────────────────────────────────────────────────
    # Output and visualization
    # ─────────────────────────────────────────────────────────────
    publish_pose_graph: true              # Publish pose graph for debug
    publish_transform: true               # Publish map→odom TF
    tf_buffer_duration: 30.0              # seconds (TF history for debugging)
```

### Tuning for Outdoor Operation

**Challenge:** Grass, trees, and seasonal changes cause false loop closures.

**Conservative Settings (Recommended):**
```yaml
loop_search_maximum_distance: 2.0     # Stricter: only close very confident loops
loop_match_minimum_chain_size: 15     # Require longer chains
minimum_travel_distance: 1.0          # Don't add scans too frequently
```

**Aggressive Settings (Faster Mapping, Risk of Artifacts):**
```yaml
loop_search_maximum_distance: 5.0
loop_match_minimum_chain_size: 5
minimum_travel_distance: 0.2
```

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
ros2 topic echo /odometry/filtered_map
```

### Step 4: Iterate

Rerun with adjusted parameters, observe results, adjust again.

---

## Reference: Default Parameters Quick Lookup

| Parameter | Default | Unit | Range |
|-----------|---------|------|-------|
| `serial_port` | `/dev/mowgli` | – | any `/dev/tty*` |
| `baud_rate` | 115200 | baud | 9600–230400 |
| `ekf_odom` frequency | 50.0 | Hz | 10–100 |
| `ekf_map` frequency | 20.0 | Hz | 5–50 |
| `controller_frequency` | 10.0 | Hz | 5–50 |
| `desired_linear_vel` | 0.3 | m/s | 0.1–1.0 |
| `lookahead_dist` | 0.6 | m | 0.2–1.5 |
| `linear_p_gain` | 2.0 | – | 0.5–5.0 |
| `slam_resolution` | 0.05 | m/pixel | 0.01–0.20 |
| `loop_search_max_distance` | 3.0 | m | 1.0–5.0 |

---

**For detailed system architecture, see [ARCHITECTURE.md](ARCHITECTURE.md).**

**For firmware integration, see [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md).**
