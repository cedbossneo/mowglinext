# Mowgli ROS2

A complete rewrite of the [OpenMower](https://github.com/ClemensElflein/open_mower_ros) autonomous lawn mower in ROS2 Jazzy, featuring modern navigation, SLAM-based mapping, behavior tree control, and advanced coverage path planning.

## Overview

Mowgli ROS2 is a professional-grade autonomous lawn mower platform built on the Robot Operating System 2 (ROS2). It combines proven robotics technologies—LiDAR-based simultaneous localization and mapping (SLAM), RTK-GPS fusion, dynamic obstacle avoidance, and reactive behavior trees—to deliver intelligent yard coverage.

The system maintains the proven hardware platform (Raspberry Pi 4, STM32 firmware board, LiDAR, RTK-GPS) while completely reimplementing the software stack for modularity, extensibility, and modern practices.

## Features

- **LiDAR SLAM** - Real-time mapping and localization using SLAM Toolbox with outdoor optimization
- **RTK-GPS Fusion** - Dual Extended Kalman Filter (EKF) architecture combining wheel odometry, IMU, and RTK-GPS for accurate outdoor positioning
- **Nav2 Integration** - Industrial-grade navigation stack with dynamic costmaps and path planning
- **Coverage Path Planning** - Fields2Cover v2 integration for optimal boustrophedon patterns with configurable swath angles and Dubins turn optimization
- **Dual Controller Architecture** - RPP (Regulated Pure Pursuit) with RotationShimController for transit/docking, FTC (Follow-The-Carrot) for coverage paths
- **Behavior Trees** - Reactive, composable control logic using BehaviorTree.CPP v4 with path-indexed state machine for coverage execution
- **COBS Serial Protocol** - Robust binary communication with STM32 firmware (Consistent Overhead Byte Stuffing with CRC-16 error detection)
- **Hardware Bridge** - Single USB serial interface abstracting all firmware interactions
- **Foxglove Integration** - Real-time visualization via Foxglove Studio (ws://localhost:8765)
- **Modular Architecture** - 9 focused packages with clear responsibilities and minimal coupling

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Navigation & Behavior Layer                   │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │  Nav2 Stack  │  │  Behavior    │  │  Localization       │   │
│  │ (SmacPlanner │  │  Tree (BT.   │  │  Monitor            │   │
│  │  FTCControl) │  │  CPP)        │  │  & GPS Fusion       │   │
│  └──────────────┘  └──────────────┘  └─────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
           │                  │                    │
           └──────────────────┴────────────────────┘
                      │
┌─────────────────────────────────────────────────────────────────┐
│              Hardware Bridge & Serial Protocol                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  COBS Framing + CRC16  ←→  ROS2 Topics & Services     │   │
│  │  /cmd_vel ←→ LlCmdVel  /status ← LlStatus              │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                      │
                  [USB Serial]
                      │
┌─────────────────────────────────────────────────────────────────┐
│               STM32 Firmware (Mowgli Board)                      │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌──────────┐ │
│  │   Motors   │  │   IMU      │  │  Sensors   │  │  Power   │ │
│  │  (ESC)     │  │            │  │            │  │  Mgmt    │ │
│  └────────────┘  └────────────┘  └────────────┘  └──────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### Package Dependencies

```
mowgli_interfaces (messages, services, actions)
    ↓
mowgli_hardware (COBS bridge to STM32)
    ↓
mowgli_localization (odometry, GPS fusion, monitoring)
mowgli_map (map server, boundary/zone management)
mowgli_bringup (URDF, Nav2 config, launch files)
mowgli_nav2_plugins (RPP & FTC controller plugins)
mowgli_coverage_planner (Fields2Cover v2 coverage path planning)
mowgli_behavior (BehaviorTree nodes & main control)
mowgli_monitoring (diagnostics, health monitoring, MQTT bridge)
mowgli_simulation (Gazebo Harmonic simulation assets)
    ↓
Application / Remote Control / Foxglove Studio
```

## Hardware Requirements

### Computing Platform
- **Raspberry Pi 4** (4GB+ recommended) running Ubuntu 24.04 + ROS2 Jazzy

### Mower Hardware
- **STM32-based Mowgli board** with USB CDC serial interface
- **Two independent drive motors** (rear-wheel differential drive)
- **Blade motor** (optional, controllable)
- **Encoder feedback** on both drive wheels
- **Inertial Measurement Unit (IMU)** (9-DOF or 6-DOF)

### Sensors
- **LiDAR** (e.g., RpLiDAR A1, Livox Mid-360) publishing `/scan` at 5–20 Hz
- **RTK-GPS receiver** (e.g., u-blox F9P) with fix status
- **Power sensing** (voltage, current monitoring on battery)

### Electrical
- **12–24V lithium battery** with voltage monitoring
- **USB-to-serial adapter** or direct USB CDC from STM32 to Raspberry Pi

## Quick Start

### 1. Building from Source

#### Prerequisites
- **ROS2 Jazzy** installed on Ubuntu 24.04
- **colcon** build tool (`sudo apt install python3-colcon-common-extensions`)
- Standard build tools: `gcc`, `cmake`, `git`

#### Clone and Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/ClemensElflein/mowgli-ros2.git mowgli
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launching Hardware (Real Robot)

Connect the Mowgli board via USB and identify the port:
```bash
ls /dev/ttyUSB*    # or /dev/ttyACM* for native CDC
```

Launch the full stack:
```bash
ros2 launch mowgli_bringup mowgli.launch.py serial_port:=/dev/mowgli
```

This brings up:
- Robot state publisher (URDF → static transforms)
- Hardware bridge (USB-serial COBS communication)
- Twist multiplexer (priority-based command routing)
- Localization (wheel odometry + IMU fusion)
- Behavior tree controller

### 3. Launching Simulation (Gazebo Harmonic)

#### Option A: GUI Simulation with noVNC

```bash
make docker-sim
make run-sim-gui
```

Visit http://localhost:6080/vnc.html to view the Gazebo GUI. Foxglove Studio is available at ws://localhost:8765.

#### Option B: Development Simulation

```bash
make dev-sim
```

Then in the container:
```bash
ros2 launch mowgli_bringup simulation.launch.py
```

This spawns a virtual Mowgli in a Gazebo Harmonic world with:
- Simulated sensors (LiDAR, IMU, wheel odometry)
- Physics-based motor and blade control
- ROS2 ↔ Gazebo bridging for sensor data and commands
- Foxglove Bridge for real-time visualization

### 4. Docker Development Workflow

The Makefile provides convenient development targets:

```bash
# Build Docker image for simulation
make docker-sim

# Run simulation with GUI (noVNC at http://localhost:6080/vnc.html)
make run-sim-gui

# Development shell with all build dependencies
make dev-shell

# Build individual package in dev container
make dev-build-pkg PACKAGE=mowgli_behavior

# Rebuild entire workspace in dev container
make dev-build

# Restart dev container
make dev-restart

# Full development simulation (no GUI, ROS2 commands available)
make dev-sim
```

Or use docker-compose directly:
```bash
docker compose build
docker compose up simulation    # For simulation testing
# or
docker compose up hardware      # For real robot (mounts /dev/mowgli)
```

## Package Descriptions

### mowgli_interfaces
**Message, service, and action definitions** for the entire system.

Key types:
- `mowgli_interfaces::msg::Status` – Mower mode, charging, rain, sound/UI module availability
- `mowgli_interfaces::msg::Emergency` – Stop button, lift detection, emergency latching
- `mowgli_interfaces::msg::Power` – Battery voltage, charging current, charger status
- `mowgli_interfaces::srv::MowerControl` – Enable/disable cutting blade and mower
- `mowgli_interfaces::srv::EmergencyStop` – Trigger or release emergency stop
- Standard ROS2 types: `sensor_msgs/Imu`, `geometry_msgs/Twist`, `nav_msgs/Odometry`, etc.

### mowgli_hardware
**Serial bridge between STM32 firmware and ROS2** using the COBS protocol.

**Topics Published:**
- `~/status` (Status) – Mower state, hardware availability
- `~/emergency` (Emergency) – Emergency stop status and reason
- `~/power` (Power) – Battery voltage, charge current
- `~/imu/data_raw` (sensor_msgs/Imu) – Raw accelerometer and gyroscope data

**Topics Subscribed:**
- `~/cmd_vel` (geometry_msgs/Twist) – Motor velocity commands from the navigation stack

**Services Offered:**
- `~/mower_control` – Enable/disable cutting or mower drive
- `~/emergency_stop` – Assert or release emergency stop

**Protocol:**
- COBS-framed binary packets with CRC-16 error detection
- 115200 baud USB serial (configurable)
- Automatic reconnection on serial port errors
- Heartbeat every 250 ms to maintain watchdog on firmware

### mowgli_localization
**Three-node localization pipeline** producing accurate, multi-modal pose estimates.

**Nodes:**

1. **wheel_odometry_node**
   - Differentially integrates left/right wheel encoder ticks
   - Outputs `/wheel_odom` odometry at 50 Hz
   - Used as base motion model in the EKF

2. **gps_pose_converter_node**
   - Converts RTK-GPS fix to local ENU coordinate system
   - Scales covariance based on GPS quality indicators
   - Publishes `/gps/pose` as PoseWithCovarianceStamped

3. **localization_monitor_node**
   - Monitors EKF filter health and variance
   - Detects 5 degradation modes: stale odometry, GPS timeout, filter divergence, etc.
   - Publishes degradation warnings for behavior tree adaptation

**Dual EKF Architecture:**
- `ekf_odom` (50 Hz) – Fuses wheel odometry + IMU → `odom` frame
- `ekf_map` (20 Hz) – Fuses filtered odometry + GPS → `map` frame

### mowgli_bringup
**Configuration, URDFs, and launch infrastructure** for the entire stack.

**Files:**
- `urdf/mowgli.urdf.xacro` – Robot description with differential drive kinematics
- `config/hardware_bridge.yaml` – Serial port and communication rates
- `config/localization.yaml` – Dual EKF tuning for outdoor operation
- `config/nav2_params.yaml` – Nav2 costmaps, planners, and controller parameters
- `config/slam_toolbox.yaml` – Outdoor SLAM tuning (loop closure, map update rates)
- `config/twist_mux.yaml` – Priority multiplexing of navigation, teleoperation, and emergency commands
- `launch/mowgli.launch.py` – Main real-hardware bringup
- `launch/simulation.launch.py` – Gazebo Harmonic simulation

### mowgli_nav2_plugins
**Custom Nav2 controller plugins** for navigation and coverage path following.

**RPPController (Regulated Pure Pursuit):**
- Used for transit to waypoints and docking navigation
- Wrapped in RotationShimController for initial heading alignment
- Smooth, stable control for general-purpose navigation
- Integrates with standard Nav2 goalchecker framework

**FTCController (Follow-The-Carrot):**
- Specialized for coverage path execution with path-indexed state machine
- **5-State FSM:** Pre-Rotate → Forward → Approach → Stop → Oscillation Recovery
- **3-Channel PID:** Independent control of linear velocity, angular velocity, and acceleration
- **Path Interpolation:** SLERP-based carrot point following for smooth curves
- **Collision Detection:** Dynamic costmap-based obstacle checking
- **Oscillation Detection:** Reactive recovery when the robot becomes trapped
- **Tuned for Lawn Mowing:** Handles the unique dynamics of slow, heavy outdoor vehicles

**Custom Navigate-To-Pose Behavior Tree XML:**
- `navigate_to_pose.xml` with GoalCheckerSelector for multiple goal validation strategies
- Seamless switching between RPP (transit) and FTC (coverage) controllers

### mowgli_coverage_planner
**Fields2Cover v2 integration for optimal coverage path planning**.

**Service Provided:**
- `~/plan_coverage` (PlanCoverage action) – Accepts boundary polygon, zone inclusion/exclusion, desired swath width, and returns coverage waypoint path

**Features:**
- **Boustrophedon (parallel line) pattern generation** – Optimal lawn coverage with configurable swath angles
- **Dubins turn optimization** – Smooth curved transitions between mowing lines minimizing non-productive movement
- **Zone-based planning** – Supports inclusion/exclusion zones for obstacles and areas of interest
- **Real-time planning** – Generates optimal path in seconds even for large properties
- **Swath angle optimization** – Automatically or manually sets the best mowing direction for field shape

**Typical Usage (from behavior tree):**
```cpp
PlanCoveragePath(boundary, zone_config) → waypoint_list
```

### mowgli_map
**Map server and boundary/zone management** for coverage planning context.

**Nodes:**
- `map_server_node` – Loads boundary map and zone definitions from saved files
- Publishes map metadata for coverage planner

**Configuration:**
- Boundary polygon (polygon from map file or GPS-defined)
- Inclusion/exclusion zones with penalty costs
- Saved coverage sessions for replay and analysis

### mowgli_monitoring
**System health monitoring, diagnostics, and MQTT integration**.

**Nodes:**
1. **diagnostics_node**
   - Monitors EKF health, battery voltage trends, motor encoder faults
   - Publishes diagnostic_msgs/DiagnosticArray to aggregator
   - Detects 5 degradation modes: stale odometry, GPS timeout, filter divergence, motor stall, battery fault

2. **mqtt_bridge_node**
   - Forwards selected topics to MQTT broker for remote monitoring
   - Publishes: battery voltage, charging status, mowing progress, error codes
   - Subscribes to remote control commands if needed

**Published Topics:**
- `~/diagnostics` (diagnostic_msgs/DiagnosticArray) – System health status
- (MQTT) Battery voltage, mower mode, GPS quality, error codes

### mowgli_behavior
**High-level control logic** using BehaviorTree.CPP v4 with reactive patterns and coverage integration.

**Tree Structure:**
```
ReactiveSequence (Root)
├── EmergencyGuard (Fallback)
│   ├── Inverter → IsEmergency
│   └── EmergencyHandler (stop, publish status)
└── MainLogic (Fallback)
    ├── DockingSequence (battery critical → navigate to dock)
    ├── MowingSequence (START command)
    │   ├── PlanCoveragePath (Fields2Cover)
    │   ├── NavigateToPose to first waypoint (RPP)
    │   ├── FollowCoveragePath (FTC controller, path-indexed SM)
    │   ├── Recovery with retry (3 attempts)
    │   └── Navigate to dock on completion
    ├── HomeSequence (HOME command)
    ├── RecordingSequence (S1 command)
    └── IdleSequence (default)
```

**Condition Nodes:**
- `IsEmergency` – Checks emergency latch state
- `IsBatteryLow` – Checks power messages for low voltage
- `IsGpsAvailable` – Checks GPS quality from monitor node
- `IsLocalizationHealthy` – Checks EKF variance from monitor node

**Action Nodes:**
- `PlanCoveragePath` – Calls mowgli_coverage_planner service with Fields2Cover
- `NavigateToPose` – Sends goal to Nav2 action server (uses RPP by default)
- `FollowCoveragePath` – Executes coverage waypoints using FTC controller
- `SetMowerEnabled` – Calls mower_control service
- `RecoveryHandler` – Attempts goal re-planning with backoff strategy
- `UpdateHighLevelState` – Sends current mode to firmware

All nodes are registered in `register_nodes.cpp` for dynamic loading from XML.

### mowgli_simulation
**Gazebo Harmonic simulation assets and physics models** for testing without hardware.

**Contents:**
- `models/mowgli/` – Complete mower URDF with Gazebo physics, contact sensors, and joint actuators
- `worlds/backyard.world` – Example lawn simulation environment with grass, obstacles, charging dock
- `launch/spawn_mowgli.launch.py` – Spawns the mower in Gazebo with sensor bridging
- Sensor simulation: LiDAR point cloud, IMU (noise and gravity), wheel encoder odometry
- Motor simulation: Direct velocity control → motor commands via Gazebo plugins

**Docker Usage:**
```bash
docker compose up simulation    # Headless Gazebo with ROS2 bridge
```

**GUI Access:**
- noVNC: http://localhost:6080/vnc.html (Gazebo visualizer)
- Foxglove: ws://localhost:8765 (real-time telemetry)

## Configuration Guide

### Serial Communication

**File:** `src/mowgli_bringup/config/hardware_bridge.yaml`

```yaml
hardware_bridge:
  ros__parameters:
    serial_port: "/dev/mowgli"      # Change to match your connection
    baud_rate: 115200                 # Must match firmware
    heartbeat_rate: 4.0                # Hz – keep-alive to firmware
    publish_rate: 100.0                # Hz – sensor polling frequency
    high_level_rate: 2.0               # Hz – mode/GPS quality updates
```

### Localization Tuning

**File:** `src/mowgli_bringup/config/localization.yaml`

The dual EKF uses two pipelines:
1. **odom EKF** – Local fusion of wheel odometry + IMU (high frequency, low latency)
2. **map EKF** – Global fusion of filtered odometry + GPS (slower, lower variance)

Key parameters:
- `frequency` – EKF update rate (Hz). Higher for faster response, lower for smoother estimates.
- `process_noise_covariance` – Trust in the motion model. Increase if odometry drifts; decrease if GPS variance is high.
- `odom0_config`, `pose0_config` – Which state dimensions to fuse from each sensor.

### Nav2 Parameters

**File:** `src/mowgli_bringup/config/nav2_params.yaml`

**Controller Server:**
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0         # Hz – how fast to compute motor commands
    FollowPath:
      plugin: "mowgli_nav2_plugins::FTCController"
      desired_linear_vel: 0.3           # m/s – target speed for lawn mowing
      lookahead_dist: 0.6               # m – carrot distance ahead of robot
```

**Planner Server:**
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      maximum_iterations: 1000
      angle_quantization_bins: 72      # 5° resolution
```

### SLAM Tuning

**File:** `src/mowgli_bringup/config/slam_toolbox.yaml`

For outdoor operation with changing grass height and seasonal variation:
```yaml
slam_toolbox:
  ros__parameters:
    map_start_at_origin: false
    map_frame: map
    base_frame: base_link
    odom_frame: odom

    # Outdoor tuning
    maximum_laser_range: 25.0          # m – beyond this, ignore returns
    minimum_scan_angle: -3.14
    maximum_scan_angle: 3.14

    # Loop closure (conservative for outdoor)
    enable_loop_closure: true
    loop_search_maximum_distance: 3.0  # m – only close loops < 3m
    loop_match_minimum_chain_size: 10
```

### Twist Multiplexer

**File:** `src/mowgli_bringup/config/twist_mux.yaml`

Priority-based velocity command routing:

```yaml
twist_mux:
  ros__parameters:
    topics:
      navigation:                       # Lowest priority (10)
        topic: /cmd_vel_nav
        priority: 10
        timeout: 0.5                    # Seconds

      teleop:                           # Higher priority (20)
        topic: /cmd_vel_teleop
        priority: 20
        timeout: 0.5

      emergency:                        # Highest velocity source (100)
        topic: /cmd_vel_emergency
        priority: 100
        timeout: 0.2
```

### FTCController Parameters

**File:** `src/mowgli_bringup/config/nav2_params.yaml` (controller section)

```yaml
FollowPath:
  plugin: "mowgli_nav2_plugins::FTCController"

  # Motion control
  desired_linear_vel: 0.3              # m/s target speed
  lookahead_dist: 0.6                  # m carrot distance
  lookahead_max_angle: 0.5             # rad max carrot angle deviation

  # PID gains (linear velocity)
  linear_p_gain: 2.0
  linear_i_gain: 0.5
  linear_d_gain: 0.1
  linear_max_integral: 0.5

  # PID gains (angular velocity)
  angular_p_gain: 2.0
  angular_i_gain: 0.5
  angular_d_gain: 0.1
  angular_max_integral: 0.5

  # Safety
  max_linear_vel: 0.5                  # m/s absolute max
  max_angular_vel: 1.0                 # rad/s absolute max

  # Oscillation recovery
  use_oscillation_recovery: true
  oscillation_recovery_min_duration: 5.0  # seconds before recovery kicks in
```

## Building and Testing

### Build the Project
```bash
colcon build --symlink-install
source install/setup.bash
```

### Run Unit Tests
```bash
colcon test --ctest-args -VV
```

All packages include unit tests for COBS serialization, wheel odometry, oscillation detection, and the FTC controller state machine.

### Visualize in RViz2
```bash
rviz2 -d src/mowgli_bringup/config/mowgli.rviz
```

Monitor:
- `/map` – SLAM output
- `/scan` – LiDAR data
- `/tf_tree` – Transform tree
- `/cmd_vel` – Motor commands
- `/odometry/filtered_map` – Fused pose estimate

## Development Workflow

### Adding a New Behavior Tree Node

1. **Define the node in** `src/mowgli_behavior/include/mowgli_behavior/[condition|action]_nodes.hpp`
2. **Implement in** `src/mowgli_behavior/src/[condition|action]_nodes.cpp`
3. **Register in** `src/mowgli_behavior/src/register_nodes.cpp` using `BT_REGISTER_NODES`
4. **Reference in tree XML** (e.g., `mowgli_behavior.xml`)
5. **Add unit tests** in `src/mowgli_behavior/test/`

### Adding a New ROS2 Node

1. Create source files in the appropriate package
2. Add a `Node` entry in `CMakeLists.txt` and link dependencies
3. Add launch file entry (or include in existing launch)
4. Test with: `ros2 launch mowgli_bringup [launch_file].py`

### Debugging Serial Communication

Enable debug logging:
```bash
ros2 launch mowgli_bringup mowgli.launch.py --log-level mowgli_hardware:=DEBUG
```

Monitor raw serial traffic:
```bash
timeout 10 cat /dev/mowgli | xxd    # Raw bytes
```

## Contributing

We welcome contributions! Please:

1. **Follow the coding style** – See `~/.claude/rules/` for linting and formatting standards
2. **Write tests** – Minimum 80% code coverage for new features
3. **Document changes** – Update relevant README sections and inline code comments
4. **Submit a PR** – Include a clear description of the change and testing performed

## License

Mowgli ROS2 is licensed under the **GNU General Public License v3.0** (GPL-3.0), maintaining compatibility with OpenMower. See `LICENSE` file for details.

## Credits

- **Clemens Elflein** – Original OpenMower design and firmware architecture
- **Cedric** – ROS2 rewrite, Mowgli firmware, and OpenMower GUI
- **ROS2 Community** – Nav2, SLAM Toolbox, BehaviorTree.CPP, and ecosystem
- **Contributors** – All those who have tested and improved the system

## Support

For questions and issues:
- **GitHub Issues** – Report bugs and feature requests
- **ROS2 Discourse** – General ROS2 questions and help
- **Mowgli Documentation** – Full guides in `docs/` directory

---

**Happy mowing!** 🚜
