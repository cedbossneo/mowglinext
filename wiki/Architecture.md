# Mowgli ROS2 Architecture

Comprehensive technical documentation of the Mowgli ROS2 system design, including package organization, data flow, communication protocols, and integration points.

Built on **ROS2 Jazzy** with **Gazebo Harmonic** simulation, this architecture spans 12 focused packages providing complete autonomous lawn mower functionality.

## System Overview

Mowgli ROS2 is organized as a **12-package ecosystem** with clear separation of concerns and layered dependencies:

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        Application / Remote Control                      │
│                    (GUI, teleoperation, mission planning)                │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                   High-Level Control & Decision Layer                    │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────┐   │
│  │  mowgli_behavior │  │ mowgli_nav2_     │  │  mowgli_localization │   │
│  │  (Behavior Tree) │  │  plugins         │  │  (EKF + monitoring)  │   │
│  │                  │  │  (FTC + RPP      │  │                      │   │
│  │  10 Hz reactive  │  │   Controllers)   │  │  Multiple nodes:     │   │
│  │  control         │  │                  │  │  - Wheel odometry    │   │
│  │                  │  │  Nav2 local plan │  │  - GPS converter     │   │
│  │                  │  │  10 Hz           │  │  - Health monitor    │   │
│  │                  │  │                  │  │  - Diagnostics       │   │
│  └──────────────────┘  └──────────────────┘  └──────────────────────┘   │
│                                                                           │
│  ┌──────────────────────────────┐  ┌──────────────────────────────────┐ │
│  │  mowgli_brv_planner           │  │  mowgli_monitoring               │ │
│  │  (B-RV algorithm, action     │  │  (Diagnostics aggregator,        │ │
│  │   server, Voronoi transit)   │  │   MQTT bridge)                   │ │
│  └──────────────────────────────┘  └──────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                         Config & Launch Layer                            │
│                      (mowgli_bringup: URDF, params)                      │
│                      (mowgli_map: map server, storage)                   │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                    Hardware Abstraction & Protocol                       │
│            (mowgli_hardware: COBS serial bridge to STM32)                │
│                                                                           │
│  Publishers:                         Subscribers:                        │
│    - ~/status (Status msg)            - ~/cmd_vel (Twist)                │
│    - ~/emergency (Emergency msg)                                         │
│    - ~/power (Power msg)            Services:                            │
│    - ~/imu/data_raw (Imu msg)         - ~/mower_control                  │
│                                        - ~/emergency_stop                │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
                      [USB Serial: COBS-framed packets]
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                        STM32 Firmware (Mowgli Board)                     │
│  Motor control, sensor acquisition, real-time loop, watchdog            │
└──────────────────────────────────────────────────────────────────────────┘
```

## Package Overview

| Package | Purpose | Dependencies |
|---------|---------|--------------|
| **mowgli_interfaces** | Message, service, and action type definitions | ROS2 core |
| **mowgli_hardware** | Serial bridge to STM32 firmware (COBS + CRC-16 protocol) | mowgli_interfaces |
| **mowgli_localization** | Multi-source localization (wheel odometry, IMU, RTK-GPS fusion, EKF) | mowgli_interfaces, robot_localization |
| **mowgli_nav2_plugins** | Nav2 controller plugins (FTC, RPP + RotationShimController, goal checkers) | nav2_core, mowgli_interfaces |
| **mowgli_brv_planner** | Coverage path planning using B-RV algorithm (MBB sweep direction, grid-based boustrophedon, Voronoi roadmap transit) | mowgli_interfaces, nav_msgs |
| **mowgli_behavior** | Reactive behavior tree control (BehaviorTree.CPP v4) | mowgli_interfaces, nav2_msgs |
| **mowgli_monitoring** | Diagnostics aggregation and MQTT bridge for external monitoring | diagnostic_msgs |
| **mowgli_simulation** | Gazebo Harmonic worlds, robot models, and ros_gz_bridge configuration | mowgli_bringup, ros_gz_sim, ros_gz_bridge |
| **mowgli_map** | Map server, storage, persistence for offline maps, and obstacle_tracker_node (persistent LiDAR obstacle detection) | nav_msgs, nav2_map_server, mowgli_interfaces |
| **mowgli_description** | URDF/xacro robot model and meshes | xacro, robot_state_publisher |
| **mowgli_bringup** | Configuration, launch orchestration, and integration layer | All packages above |

## Package Dependency Graph

```
mowgli_interfaces (base layer)
    │
    ├──→ mowgli_hardware
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_localization
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_nav2_plugins
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_behavior
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_brv_planner
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_monitoring
    │       └──→ mowgli_bringup
    │
    └──→ mowgli_map
            └──→ mowgli_bringup

mowgli_description (robot model)
    └──→ mowgli_bringup

mowgli_simulation (standalone testing)
    ├──→ mowgli_bringup
    ├──→ ros_gz_sim, ros_gz_bridge
    └──→ Gazebo Harmonic

mowgli_bringup (integration layer)
    ├──→ launch files
    ├──→ URDF/xacro
    └──→ configuration files

Application layer
    └──→ mowgli_bringup (and sub-packages)
```

## Detailed Package Architecture

### 1. mowgli_interfaces

**Purpose:** Define all ROS2 message, service, and action types.

**Location:** `src/mowgli_interfaces/`

**Key Definitions:**

#### Messages

- **Status.msg** – Mower operational state
  ```
  builtin_interfaces/Time stamp
  uint8 mower_status              # MOWER_STATUS_OK, MOWER_STATUS_INITIALIZING
  bool raspberry_pi_power         # Pi on/off switch state
  bool is_charging                # Battery charging active
  bool rain_detected              # Rain sensor
  bool sound_module_available     # Sound module present
  bool sound_module_busy          # Sound playing
  bool ui_board_available         # UI board detected
  bool mow_enabled                # Cutting blade enabled
  bool esc_power                  # Motor power enabled
  ```

- **Emergency.msg** – Safety stop status
  ```
  builtin_interfaces/Time stamp
  bool latched_emergency          # Emergency is latched (requires explicit release)
  bool active_emergency           # Any emergency condition active
  string reason                   # Human-readable description
  ```

- **Power.msg** – Battery and charging information
  ```
  builtin_interfaces/Time stamp
  float32 v_charge                # Charging port voltage
  float32 v_battery               # Battery voltage
  float32 charge_current          # Charging current (mA)
  bool charger_enabled            # Charger plugged and active
  string charger_status           # "charging", "idle", "error"
  ```

- **WheelTick.msg** – Encoder pulse counts with validity bitmasks
  ```
  builtin_interfaces/Time stamp
  float32 wheel_tick_factor       # Ticks-to-distance conversion factor
  uint8 valid_wheels              # Bitmask: WHEEL_VALID_FL=1, FR=2, RL=4, RR=8
  bool wheel_direction_fl         # Front-left direction
  bool wheel_direction_fr         # Front-right direction
  bool wheel_direction_rl         # Rear-left direction
  bool wheel_direction_rr         # Rear-right direction
  uint32 wheel_ticks_fl           # Front-left tick count
  uint32 wheel_ticks_fr           # Front-right tick count
  uint32 wheel_ticks_rl           # Rear-left tick count
  uint32 wheel_ticks_rr           # Rear-right tick count
  ```

- **AbsolutePose.msg** – Robot position with GPS quality flags (FLAG_GPS_RTK=1, FLAG_GPS_RTK_FIXED=2, FLAG_GPS_RTK_FLOAT=4, FLAG_GPS_DEAD_RECKONING=8)
- **HighLevelStatus.msg** – Behavior tree state (IDLE, UNDOCKING, MOWING, RECOVERING, DOCKING, RECORDING)
- **ESCStatus.msg** – Motor ESC telemetry
- **ImuRaw.msg** – Raw IMU data from STM32 firmware
- **MapArea.msg** – Mowing area polygon definition for map_server_node
- **CoveragePath.msg** – Coverage path with metadata
- **ObstacleArray.msg** – Collection of tracked obstacles from obstacle_tracker_node
- **TrackedObstacle.msg** – Individual persistent obstacle with position, age, and observation count

#### Services

- **MowerControl.srv** – Blade and drive control
  ```
  Request:
    bool mow_enabled              # Enable/disable blade motor
    uint8 mow_direction           # CW, CCW, or off
  Response:
    bool success
  ```

- **EmergencyStop.srv** – Safety control
  ```
  Request:
    bool emergency                # true=assert, false=release
  Response:
    bool success
  ```

#### Actions

- **NavigateToPose.action** – Nav2 navigation goal (standard nav2_msgs)

**Design Notes:**
- All timestamps use `builtin_interfaces/Time` (ROS2 idiom, replacing `rosgraph_msgs/Time` from ROS1)
- Floating-point values are `float32` (hardware native) except for precise GPS data (`float64`)
- Bitmasks used for compactness in Status and Emergency (reduces firmware packet size)

---

### 2. mowgli_hardware

**Purpose:** Serial bridge between STM32 firmware and ROS2 via COBS protocol.

**Location:** `src/mowgli_hardware/`

**Architecture:**

```
SerialPort (open/read/write raw bytes)
    ↓
PacketHandler (COBS framing, CRC16 validation)
    ↓
HardwareBridgeNode (ROS2 topics/services interface)
    ↓
ROS2 ecosystem
```

#### Key Components

**SerialPort (serial_port.cpp/.hpp)**
- Low-level serial port abstraction
- Non-blocking read/write
- Automatic reconnection on error
- Configurable baud rate (115200 default)

**PacketHandler (packet_handler.cpp/.hpp)**
- COBS (Consistent Overhead Byte Stuffing) encoding/decoding
- CRC-16 CCITT checksum calculation and verification
- Packet type dispatch via enum `PacketId`
- Thread-safe callback for complete packets

**HardwareBridgeNode (hardware_bridge_node.cpp)**
- ROS2 node instantiation (singleton pattern)
- Parameter declaration (serial_port, baud_rate, heartbeat_rate, etc.)
- Publishers, subscribers, services
- Timer-based read loop (100 Hz default)
- Heartbeat transmission (4 Hz default)
- High-level state updates (2 Hz default, GPS quality + mode)

#### Wire Protocol: COBS + CRC-16

**Packet Structure:**

```
[COBS_FLAG] [COBS_ENCODED_PAYLOAD] [COBS_FLAG]
   0x00                                 0x00

PAYLOAD structure (binary, native endianness):
  [packet_type: uint8] [payload_data] [crc16: uint16_le]
```

**Example: LlCmdVel (motor command)**
```c
struct LlCmdVel {
  uint8_t type;           // PACKET_ID_LL_CMD_VEL (0x02)
  float linear_x;         // m/s linear velocity
  float angular_z;        // rad/s angular velocity
  uint16_t crc16;         // Calculated by hardware_bridge
};
// 1 + 4 + 4 + 2 = 11 bytes unencoded
// After COBS: 13 bytes (overhead for 0x00 bytes)
```

**COBS Encoding:**
- Byte stuffing scheme: encodes data so no 0x00 bytes appear in the payload
- Overhead: worst-case +1 byte per 254 data bytes
- Delimiter: 0x00 frame flag marks packet boundaries (both start and end)
- Enables robust framing even without external length fields

**CRC-16 (CCITT polynomial 0x1021):**
- Calculated over [packet_type][payload_data] only (not the CRC field itself)
- Polynomial: 0xA001 (reversed CCITT)
- Detects single and double bit errors, all error patterns < 16 bits

**Packet Types (from ll_datatypes.hpp):**

| Type ID | Name | Direction | Purpose |
|---------|------|-----------|---------|
| 0x00 | LL_HEARTBEAT | Pi → STM32 | Keep-alive, emergency control, release |
| 0x01 | LL_HIGH_LEVEL_STATE | Pi → STM32 | Mode, GPS quality, localization health |
| 0x02 | LL_CMD_VEL | Pi → STM32 | Motor velocity commands |
| 0x10 | LL_STATUS | STM32 → Pi | Mower state, charging, rain, sensors |
| 0x11 | LL_IMU | STM32 → Pi | Accelerometer + gyroscope data |
| 0x12 | LL_UI_EVENT | STM32 → Pi | Button press, duration |

#### Data Flow Diagrams

**Incoming (STM32 → Pi → ROS2):**
```
LlStatus (firmware)
    ↓ [COBS + CRC]
SerialPort::read()
    ↓
PacketHandler::feed() → on_packet_received()
    ↓
handle_status() → Status msg + Emergency msg + Power msg → pub_status_, pub_emergency_, pub_power_
    ↓
ROS2 network: /status, /emergency, /power
```

**Outgoing (ROS2 → Pi → STM32):**
```
ROS2: /cmd_vel (Twist msg)
    ↓
on_cmd_vel() callback
    ↓
Create LlCmdVel packet
    ↓
send_raw_packet() → PacketHandler::encode_packet() → [COBS + CRC]
    ↓
SerialPort::write()
    ↓
STM32 firmware
```

**Heartbeat (periodic, Pi → STM32):**
```
Timer callback (4 Hz)
    ↓
send_heartbeat()
    ↓
Create LlHeartbeat with emergency_active, emergency_release_pending flags
    ↓
send_raw_packet() → [COBS + CRC] → STM32
    ↓
STM32 watchdog reset
```

#### Configuration

**File:** `src/mowgli_bringup/config/hardware_bridge.yaml`

```yaml
hardware_bridge:
  ros__parameters:
    serial_port: "/dev/mowgli"     # Device path (USB serial)
    baud_rate: 115200               # Must match firmware
    heartbeat_rate: 4.0             # Hz – watchdog feed
    publish_rate: 100.0             # Hz – sensor polling
    high_level_rate: 2.0            # Hz – mode/GPS updates
```

**Topics Published (rates):**
- `~/status` (Status msg) – 100 Hz max (firmware sensor rate)
- `~/emergency` (Emergency msg) – 100 Hz max (with Status)
- `~/power` (Power msg) – 100 Hz max (with Status)
- `~/imu/data_raw` (sensor_msgs/Imu) – 100 Hz max (firmware IMU rate)

**Topics Subscribed:**
- `~/cmd_vel` (geometry_msgs/Twist) – On-demand callback (no rate limit)

**Services:**
- `~/mower_control` – Synchronous, blocks until acknowledged
- `~/emergency_stop` – Synchronous, blocks until acknowledged

---

### 3. mowgli_localization

**Purpose:** Multi-source localization pipeline (odometry, GPS fusion, health monitoring).

**Location:** `src/mowgli_localization/`

**Architecture:**

```
Inputs:
  - /status (WheelTick in Status msg)
  - /imu/data (sensor_msgs/Imu) → smoothed IMU
  - /gps/rtk_fix (sensor_msgs/NavSatFix, RTK status)

↓

three_nodes:

1) wheel_odometry_node
   - Integrates RL/RR encoder ticks
   - Publishes /wheel_odom (Odometry)
   - 50 Hz

2) gps_pose_converter_node
   - RTK fix → local ENU pose
   - Publishes /gps/pose (PoseWithCovarianceStamped)
   - Variable rate (10-20 Hz depending on RTK health)

3) localization_monitor_node
   - Monitors EKF variance
   - Detects degradation (5 modes)
   - Publishes /localization/status (DiagnosticStatus)

↓

Inputs to robot_localization (launched by mowgli_bringup):

1) ekf_odom node (50 Hz)
   - Fuses: /wheel_odom + /imu/data
   - Output: /odometry/filtered_odom (odom → base_link)

2) ekf_map node (20 Hz)
   - Fuses: /odometry/filtered_odom + /gps/pose
   - Output: /odometry/filtered_map (map → odom)

↓

Final Output:
  /tf tree: map → odom → base_link
  /odometry/filtered_odom (local estimate)
  /odometry/filtered_map (global estimate with GPS correction)
```

#### 3a. wheel_odometry_node

**Inputs:**
- Hardware bridge's Status messages (contains WheelTick data)

**Outputs:**
- `/wheel_odom` (nav_msgs/Odometry, 50 Hz)

**Algorithm: Differential Drive Kinematics**

```
Input:
  RL/RR tick deltas since last update
  Odometry estimate: (x, y, theta)

Process (midpoint integration):
  d_left  = ticks_rl_delta / TICKS_PER_METER
  d_right = ticks_rr_delta / TICKS_PER_METER

  d_center = (d_left + d_right) / 2       # Forward motion
  d_theta  = (d_right - d_left) / TRACK   # Rotation (TRACK = wheel separation)

  # Midpoint integration: use orientation at mid-turn
  theta_mid = theta + d_theta / 2
  x += d_center * cos(theta_mid)
  y += d_center * sin(theta_mid)
  theta += d_theta

Output:
  Odometry message with pose (x, y, theta) and twist (vx, vy, vtheta)
  Covariance:
    - Pose covariance: large (odometry-only estimates drift)
    - Twist covariance: moderate (reflects encoder noise)
```

**Covariance Strategy:**
- Only `vx` and `vyaw` components are configured in the odom EKF (see localization.yaml)
- Pose covariance intentionally large to prevent odometry from dominating the filter
- EKF will apply heavier corrections from IMU and GPS

**Parameters (wheel_odometry.yaml):**
```yaml
wheel_odometry:
  ros__parameters:
    wheel_separation_distance: 0.35    # Left-to-right wheel centre distance (m)
    ticks_per_meter: 1000              # Encoder resolution
    timeout_period_ms: 5000            # Warn if no WheelTick for 5s
```

#### 3b. gps_pose_converter_node

**Inputs:**
- `/gps/rtk_fix` (sensor_msgs/NavSatFix with fix type indicator)

**Outputs:**
- `/gps/pose` (geometry_msgs/PoseWithCovarianceStamped)

**Algorithm: GNSS to Local ENU**

```
1. First fix sets local origin (lat0, lon0, alt0)
2. All subsequent fixes converted to ENU relative to origin:

   Δlat = lat - lat0
   Δlon = lon - lon0
   Δalt = alt - alt0

   e = EARTH_RADIUS_M * Δlon * cos(lat0)
   n = EARTH_RADIUS_M * Δlat
   u = Δalt

   Output: [e, n, u] in local ENU frame

3. Covariance scaling based on RTK fix type:
   RTK Fixed:     covariance *= 1.0   (best, ~0.01-0.05 m)
   RTK Float:     covariance *= 10.0  (good, ~0.1-0.5 m)
   DGPS/SPS:      covariance *= 100.0 (poor, ~1-5 m)
   No fix:        skip publishing
```

**Parameters (gps_pose_converter.yaml):**
```yaml
gps_pose_converter:
  ros__parameters:
    map_frame_id: "map"
    earth_radius_m: 6371008.8
    origin_lat: 0.0                   # Set on first fix if not specified
    origin_lon: 0.0
    origin_alt: 0.0
```

#### 3c. localization_monitor_node

**Inputs:**
- `/odometry/filtered_odom` (from ekf_odom)
- `/odometry/filtered_map` (from ekf_map)
- `/status` (for wheel tick freshness)
- `/gps/rtk_fix` (for fix status)

**Outputs:**
- `/localization/status` (diagnostic_msgs/DiagnosticStatus)
- `/localization/mode` (std_msgs/String, for debug/logging)

**Degradation Modes (5 levels):**

| Level | Name | Condition | Response |
|-------|------|-----------|----------|
| 0 | OK | EKF healthy, all sensors fresh | Continue normally |
| 1 | ODOMETRY_STALE | No wheel ticks for 2s | Warn in logs, reduce planner timeout |
| 2 | GPS_TIMEOUT | No fix for 10s (RTK Float OK) | Use odom-only, increase drift tolerance |
| 3 | GPS_DEGRADED | RTK Float (not Fixed) | Use GPS but with higher variance |
| 4 | FILTER_DIVERGENCE | EKF variance > threshold | Emergency stop recommended |

**Parameters (localization_monitor.yaml):**
```yaml
localization_monitor:
  ros__parameters:
    odom_stale_timeout_sec: 2.0
    gps_timeout_sec: 10.0
    max_acceptable_ekf_variance: 0.25   # m² for position
```

---

### 4. mowgli_bringup

**Purpose:** Configuration, URDF, and launch orchestration for the entire stack.

**Location:** `src/mowgli_bringup/`

#### URDF: mowgli.urdf.xacro

**Robot Description:**

```
base_footprint (on ground, fixed to base_link)
    │
    ├── base_link (chassis centre at wheel height, 0.10 m above ground)
    │   │
    │   ├── left_wheel_link
    │   │   └── left_wheel_joint (revolute, axis Y)
    │   │
    │   ├── right_wheel_link
    │   │   └── right_wheel_joint (revolute, axis Y)
    │   │
    │   ├── blade_link
    │   │   └── blade_joint (revolute, axis Z)
    │   │
    │   ├── imu_link (fixed to chassis)
    │   │   └── imu_joint (fixed)
    │   │
    │   ├── gps_link (fixed to chassis top)
    │   │   └── gps_joint (fixed)
    │   │
    │   └── laser_link (LiDAR mount, typically on top)
    │       └── laser_joint (fixed)
    │
    └── (caster wheels as collision-only links, no TF)
```

**Key Dimensions:**

- **Chassis:** 0.55 m long × 0.40 m wide × 0.25 m tall
- **Wheels:** 0.10 m radius, 0.05 m width, 0.35 m track (centre-to-centre)
- **Ground clearance:** 0.10 m (wheel radius, base_link height)
- **Blade:** 0.15 m radius disc, 0.02 m height (under base_link)
- **Mass distribution:**
  - Chassis: 8.0 kg
  - Each wheel: 0.5 kg
  - Blade: 0.3 kg

**Transform Tree (TF):**

```
Map frame (SLAM origin)
    │
    ├── [ekf_map output]
    │
Odometry frame (local origin)
    │
    ├── [ekf_odom output]
    │
Base link (robot centre)
    │
    ├── [robot_state_publisher outputs]
    │
Sensor frames:
    ├── imu_link (IMU data frame)
    ├── laser_link (LiDAR data frame)
    ├── gps_link (GPS antenna location)
    └── [wheel links for visualization]
```

#### Launch Files

**mowgli.launch.py** – Real Hardware

Starts:
1. `robot_state_publisher` – Processes URDF/xacro, publishes robot_description and static TFs
2. `hardware_bridge_node` – Serial bridge to STM32
3. `twist_mux` – Priority-based cmd_vel multiplexer
4. `robot_localization` (dual EKF) – Wheel odometry fusion
5. (Optional) SLAM, Nav2, behavior tree nodes

**simulation.launch.py** – Gazebo Ignition

Starts:
1. `robot_state_publisher` – Same as real hardware
2. Gazebo Ignition (empty world or custom SDF)
3. `ros_gz_sim create` – Spawns Mowgli model at specified pose
4. `ros_gz_bridge` – Bridges sensor topics and actuator commands
5. `joint_state_publisher` – Publishes wheel joint states for visualization

**navigation.launch.py** – Nav2 Stack (included by main)

Starts:
1. `nav2_bringup` – All Nav2 servers (controller, planner, behaviors, costmap)
2. `slam_toolbox` – SLAM node for mapping (optional, environment-based)

#### Configuration Files

**hardware_bridge.yaml** – Serial communication
**localization.yaml** – Dual EKF tuning
**nav2_params.yaml** – Navigation stack (costmaps, planner, controller)
**slam_toolbox.yaml** – SLAM-specific parameters
**twist_mux.yaml** – Velocity command multiplexing

---

### 5. mowgli_nav2_plugins

**Purpose:** Custom Nav2 controller plugins for navigation and coverage path following.

**Location:** `src/mowgli_nav2_plugins/`

**Plugin Registration:** `ftc_controller_plugin.xml`

```xml
<library path="libftc_controller_plugin">
  <class type="mowgli_nav2_plugins::FTCController"
         base_class_type="nav2_core::Controller">
    <description>Follow-The-Carrot controller for both transit and coverage navigation</description>
  </class>
</library>
```

**Controller Profiles (Active Configuration):**

Two controllers are active for different navigation modes:
1. **FollowPath** – Transit navigation and docking (RPP + RotationShimController wrapper)
2. **FollowCoveragePath** – Coverage path following (FTCController with 3-axis PID, <10mm lateral accuracy)

#### FTCController: 4-State FSM & Path-Indexed Algorithm (Active for Coverage)

**State Machine:**

```
      Initial State
           │
           ▼
  ┌───────────────────┐
  │   PRE_ROTATE      │
  │ (align with path) │
  └───────────────────┘
          │
          ▼
  ┌───────────────────┐
  │   FOLLOWING       │ ← advance along path via path index
  │ (path tracking)   │   (not lookahead-based, but index-based)
  └───────────────────┘
          │
          ▼
  ┌────────────────────────────────────┐
  │   WAITING_FOR_GOAL_APPROACH        │
  │ (robot near goal, slow approach)   │
  └────────────────────────────────────┘
          │
          ▼
  ┌───────────────────┐
  │   POST_ROTATE     │
  │ (align to goal    │
  │  orientation)     │
  └───────────────────┘
          │
          ▼
  ┌───────────────────┐
  │   FINISHED        │
  │ (goal reached)    │
  └───────────────────┘

Oscillation Recovery (any state):
  If velocity < threshold for > 5 sec → hold, retry
```

**Algorithm: Path-Indexed PID Control**

Inputs:
- Global path (array of PoseStamped with positions and orientations)
- Robot pose (from TF: odom → base_link)
- Costmap (for obstacle checking)

Process:

1. **Path Index Advancement:**
   - Maintain `current_index_` along the path (not lookahead distance)
   - Advance index as robot progresses along path
   - Control point is the pose at current_index_ (or interpolated between indices)

2. **Error Calculation (in base_link frame):**
   ```
   lateral_error = cross-track distance to path
   longitudinal_error = error along path heading
   angular_error = angle to target orientation
   ```

3. **Three Independent PID Channels:**
   ```
   Lateral PID:        u_lat  = Kp_lat * lat_error   + Ki_lat * ∫lat_error   + Kd_lat * d(lat_error)/dt
   Longitudinal PID:   u_lon  = Kp_lon * lon_error   + Ki_lon * ∫lon_error   + Kd_lon * d(lon_error)/dt
   Angular PID:        u_ang  = Kp_ang * ang_error   + Ki_ang * ∫ang_error   + Kd_ang * d(ang_error)/dt
   ```

4. **Velocity Command Generation:**
   - Lateral error modulates steering (angular output)
   - Longitudinal error and state-dependent speeds control forward motion
   - Speed profile: fast (0.5 m/s) → slow (0.2 m/s) near goal

5. **Collision Avoidance:**
   - Monitor costmap within robot footprint
   - If obstacle within lookahead, trigger oscillation recovery
   - If persistent, return FAILURE to planner

**State Transitions:**

- **PRE_ROTATE → FOLLOWING:** Robot roughly aligned with path start
- **FOLLOWING → WAITING_FOR_GOAL_APPROACH:** Current_index approaches path end (robot < max_follow_distance from goal)
- **WAITING_FOR_GOAL_APPROACH → POST_ROTATE:** Robot within xy_goal_tolerance, ready to orient to final pose
- **POST_ROTATE → FINISHED:** Robot within yaw_goal_tolerance (orientation correct)
- Any state: Oscillation detected → recover, retry

**Oscillation Detection & Recovery:**

The `FailureDetector` class tracks velocity history:
- If |velocity| < `oscillation_v_eps` and |angular_velocity| < `oscillation_omega_eps` for > `oscillation_recovery_min_duration`
- Robot is considered stuck
- Controller holds position for 2 seconds, then retries the path

**Parameters (mowgli_nav2_plugins section, nav2_params.yaml):**

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

FollowCoveragePath:
  plugin: "mowgli_nav2_plugins::FTCController"

  # Speed profiles (state-dependent)
  speed_fast: 0.5                           # m/s in FOLLOWING state
  speed_slow: 0.2                           # m/s in WAITING_FOR_GOAL_APPROACH
  speed_fast_threshold: 1.5                 # distance to goal (m) before slowing
  speed_angular: 20.0                       # angular speed target (rad/s, virtual)
  acceleration: 1.0                         # m/s²

  # PID longitudinal
  kp_lon: 1.0
  ki_lon: 0.0
  ki_lon_max: 10.0
  kd_lon: 0.0

  # PID lateral
  kp_lat: 1.0
  ki_lat: 0.0
  ki_lat_max: 10.0
  kd_lat: 0.0

  # PID angular
  kp_ang: 1.0
  ki_ang: 0.0
  ki_ang_max: 10.0
  kd_ang: 0.0

  # Robot limits
  max_cmd_vel_speed: 0.5                    # m/s (clamping saturation)
  max_cmd_vel_ang: 1.0                      # rad/s
  max_goal_distance_error: 1.0              # m (triggers failure if exceeded)
  max_goal_angle_error: 10.0                # degrees
  goal_timeout: 5.0                         # seconds before goal declared unreachable
  max_follow_distance: 1.0                  # m (distance at which path end is "reached")

  # Options
  forward_only: true                        # no backward driving
  debug_pid: false
  debug_obstacle: false

  # Recovery
  oscillation_recovery: true
  oscillation_v_eps: 5.0                    # cm/s threshold for velocity stagnation
  oscillation_omega_eps: 5.0                # deg/s threshold for rotation stagnation
  oscillation_recovery_min_duration: 5.0    # seconds

  # Obstacle checking
  check_obstacles: true
  obstacle_lookahead: 5                     # number of path points to check ahead
  obstacle_footprint: true                  # use full robot footprint
```

#### FailureDetector (oscillation_detector.hpp)

Ring-buffer-based failure detection:

```cpp
class FailureDetector {
public:
  void setBufferLength(int length);
  void update(double linear_vel, double angular_vel);
  bool isOscillating() const;               // Returns true if stuck

private:
  std::vector<double> velocity_history_;
  int buffer_index_{0};
};
```

Fills a history buffer with (v, omega) samples at each `computeVelocityCommands()` call. Returns true if all buffered values fall below threshold (stagnation detected).

---

### 6. mowgli_behavior

**Purpose:** High-level reactive control using BehaviorTree.CPP v4 with multi-mode state machine.

**Location:** `src/mowgli_behavior/`

**Architecture:**

```
BehaviorTreeNode (main ROS2 node, 10 Hz)
    │
    ├── BTContext (shared state across all nodes)
    │   ├── node reference (for publishing, services, actions)
    │   ├── latest_status (from hardware bridge)
    │   ├── latest_emergency (latched emergency flag)
    │   ├── latest_power (battery voltage)
    │   ├── command_queue (high-level commands from GUI)
    │   └── [other sensory state]
    │
    └── BehaviorTree instance (XML: main_tree.xml)
        │
        └── ReactiveSequence: Root
            │
            ├── Fallback: EmergencyGuard
            │   ├── Inverter(IsEmergency) → continue if safe
            │   └── Sequence: EmergencyHandler
            │       ├── SetMowerEnabled(false)
            │       ├── StopMoving()
            │       └── PublishHighLevelStatus(EMERGENCY)
            │
            └── Fallback: MainLogic
                ├── Sequence: DockingSequence (battery critical < 20.0 V)
                │   ├── NeedsDocking threshold="20.0"
                │   ├── SetMowerEnabled(false)
                │   ├── StopMoving()
                │   └── NavigateToPose(dock_pose)
                │
                ├── Sequence: MowingSequence (COMMAND_START = 1)
                │   ├── IsCommand(1)
                │   ├── PublishHighLevelStatus(UNDOCKING)
                │   ├── SetMowerEnabled(true)
                │   ├── PlanCoveragePath(area_index=0)
                │   ├── NavigateToPose(first_waypoint)
                │   ├── Fallback: CoverageWithRecovery
                │   │   ├── RetryUntilSuccessful(3 attempts)
                │   │   │   ├── FollowCoveragePath()
                │   │   │   └── Recover: StopMoving + 2s wait
                │   │   └── FailedCoverageDock: shutdown, navigate to dock
                │   └── NavigateToPose(dock_pose) → IDLE_DOCKED
                │
                ├── Sequence: HomeSequence (COMMAND_HOME = 2)
                │   ├── IsCommand(2)
                │   ├── SetMowerEnabled(false)
                │   ├── StopMoving()
                │   └── NavigateToPose(dock_pose)
                │
                ├── Sequence: RecordingSequence (COMMAND_S1 = 3)
                │   ├── IsCommand(3)
                │   ├── PublishHighLevelStatus(RECORDING)
                │   └── WaitForDuration(0.5s)
                │
                └── Sequence: IdleSequence (default)
                    ├── PublishHighLevelStatus(IDLE)
                    └── WaitForDuration(0.5s)

Update frequency: 10 Hz tick() cycle
Execution pattern: ReactiveSequence re-evaluates all children each tick
```

**Tree Structure (from main_tree.xml):**

The tree implements a priority-based fallback selector:
1. **Emergency Guard (highest priority):** If emergency active → disable, stop, halt
2. **Docking (critical battery):** If battery < 20% → dock immediately (uninterruptible)
3. **Mowing (user-commanded):** Coverage path planning and execution with recovery
4. **Home (user-commanded):** Return to dock on user request
5. **Recording (experimental):** Waypoint recording mode
6. **Idle (default):** Standby, periodic status updates

Each sequence transitions through defined high-level states (IDLE, UNDOCKING, MOWING, RECOVERING, DOCKING, RECORDING) published to STM32 firmware for synchronization.

#### Condition Nodes (condition_nodes.cpp)

```cpp
class IsEmergency : public BT::ConditionNode
// Returns SUCCESS if active_emergency bit set

class NeedsDocking : public BT::ConditionNode
// Checks battery_voltage < threshold parameter (default 20.0 V)

class IsCommand : public BT::ConditionNode
// Port In: command (uint8)
// Returns SUCCESS if command matches current high-level command from GUI

class IsGpsAvailable : public BT::ConditionNode
// Checks RTK fix status (RTK Fixed = best)

class IsLocalizationHealthy : public BT::ConditionNode
// Queries /localization/status; returns SUCCESS if EKF healthy
```

#### Action Nodes (action_nodes.cpp)

```cpp
class NavigateToPose : public BT::AsyncActionNode
// Contacts Nav2 /navigate_to_pose action server
// Port In: goal="x;y;yaw" (string format)
// Returns RUNNING (in progress), SUCCESS (reached), FAILURE (abort/timeout)

class PlanCoveragePath : public BT::AsyncActionNode
// Sends PlanCoverage action to mowgli_brv_planner
// Port In: area_index (0-based area number)
// Publishes feedback during planning (progress_percent, phase)
// Returns SUCCESS with path, FAILURE if planning failed

class FollowCoveragePath : public BT::AsyncActionNode
// Executes the coverage path from coverage_planner
// No input ports; reads from shared result
// Returns RUNNING (following), SUCCESS (completed), FAILURE (collision/stuck)

class SetMowerEnabled : public BT::ActionNode
// Calls /mower_control service
// Port In: enabled (bool)
// Fire-and-forget; always returns SUCCESS (or gracefully continues in simulation)

class StopMoving : public BT::ActionNode
// Publishes zero Twist to /cmd_vel
// Returns SUCCESS

class PublishHighLevelStatus : public BT::ActionNode
// Publishes state enum to STM32 via firmware interface
// Port In: state (uint8), state_name (string)
// Returns SUCCESS

class WaitForDuration : public BT::ActionNode
// Sleep for specified duration
// Port In: duration_sec (double)
// Returns SUCCESS after duration elapsed

class ClearCommand : public BT::ActionNode
// Clears the pending high-level command (e.g., COMMAND_START)
// Returns SUCCESS

class RetryUntilSuccessful : public BT::ControlNode
// Wraps a child node; retries up to N times
// Port In: num_attempts (int)
// Returns SUCCESS on any child success, FAILURE after all attempts fail
```

#### Tree Control (BehaviorTreeNode)

**Subscriptions:**
- `/status` – Mower state, rain sensor, blade status
- `/emergency` – Latched emergency flag
- `/power` – Battery voltage (v_battery)
- `/high_level_control` (service) – Receive mode commands from GUI (START, HOME, S1, S2)

**Services Called:**
- `/mower_control` – Enable/disable blade
- `/emergency_stop` – Release latched emergency
- `/navigate_to_pose` (Nav2) – Send navigation goals
- `/plan_coverage` (mowgli_brv_planner) – Generate coverage paths

**Publishing:**
- `/high_level_status` (std_msgs/UInt8) – Current state (IDLE, UNDOCKING, MOWING, etc.)

**Execution Model:**
- 10 Hz tick() cycle (100 ms)
- ReactiveSequence: re-evaluates all children on each tick
- Emergency guard is always first: any emergency → abort all activity
- Fallback selectors: try sequences in priority order (docking > mowing > home > idle)
- Action nodes (NavigateToPose, FollowCoveragePath) are async: return RUNNING while in progress

#### Node Registration (register_nodes.cpp)

BehaviorTree factory registration:

```cpp
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<IsEmergency>("IsEmergency");
  factory.registerNodeType<NeedsDocking>("NeedsDocking");
  factory.registerNodeType<IsCommand>("IsCommand");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<PlanCoveragePath>("PlanCoveragePath");
  factory.registerNodeType<FollowCoveragePath>("FollowCoveragePath");
  factory.registerNodeType<SetMowerEnabled>("SetMowerEnabled");
  factory.registerNodeType<StopMoving>("StopMoving");
  factory.registerNodeType<PublishHighLevelStatus>("PublishHighLevelStatus");
  factory.registerNodeType<WaitForDuration>("WaitForDuration");
  factory.registerNodeType<ClearCommand>("ClearCommand");
  // ... etc
}
```

---

### 7. mowgli_brv_planner

**Purpose:** Autonomous coverage path planning using the B-RV algorithm (Huang et al., 2021: "A novel solution with rapid Voronoi-based coverage path planning in irregular environment for robotic mowing systems").

**Location:** `src/mowgli_brv_planner/`

**Architecture:**

```
Coverage Planner Node (ROS2)
    │
    ├── Inputs:
    │   ├── /plan_coverage action server (PlanCoverage.action)
    │   │   └── Goal: outer_boundary (geometry_msgs/Polygon), obstacles, mow_angle_deg, skip_outline
    │   │   └── Feedback: progress_percent (0-100), phase (string)
    │   │   └── Result: path (nav_msgs/Path), outline_path, total_distance, coverage_area
    │   │
    │   └── Parameters:
    │       ├── tool_width (m) – mower cutting width (grid cell size)
    │       ├── headland_passes – number of perimeter passes
    │       ├── headland_width (m) – offset distance per pass
    │       └── map_frame – coordinate frame
    │
    ├── B-RV Algorithm Pipeline:
    │   ├── 1. Headland pass generation (perimeter coverage)
    │   ├── 2. MBB (Minimum Bounding Box) – rotating calipers on convex hull for optimal sweep direction
    │   ├── 3. Grid-based coverage – discretize area into cells (size = tool_width), boustrophedon zigzag
    │   ├── 4. Obstacle handling – sweep full columns, skip obstacle cells
    │   ├── 5. Voronoi roadmap – Boost.Polygon Voronoi from boundary/obstacle samples, kd-tree KNN, Dijkstra shortest path for inter-region transit
    │   └── 6. B-RV orchestrator – Headland → MBB → Grid → Sweep+Voronoi loop
    │
    └── Outputs:
        ├── /coverage_planner_node/coverage_path (nav_msgs/Path, transient_local QoS)
        │   └── Full coverage path with poses for FTCController
        ├── /coverage_planner_node/coverage_outline (nav_msgs/Path, transient_local QoS)
        │   └── Headland outline for visualization in RViz
        └── GeoJSON output compatible with Nav2 route_server
```

**Algorithm: B-RV Workflow**

The coverage planner follows this pipeline:

1. **Headland Generation:**
   - Offset the outer boundary inward by `headland_passes * headland_width`
   - Creates a nested ring of passes around the perimeter
   - Used for edge coverage (e.g., trim grass borders)

2. **MBB (Minimum Bounding Box):**
   - Compute convex hull of the mowing area
   - Apply rotating calipers to find the minimum bounding box
   - Determines optimal sweep direction for minimal turns

3. **Grid-based Coverage:**
   - Discretize the interior area into grid cells (cell size = tool_width)
   - Generate boustrophedon zigzag sweep pattern across full columns
   - Skip cells that overlap with obstacle polygons

4. **Voronoi Roadmap Transit:**
   - Build Voronoi diagram from boundary and obstacle sample points using Boost.Polygon
   - Connect Voronoi vertices via kd-tree KNN
   - Use Dijkstra shortest path for inter-region transit between disconnected sweep segments

5. **Path Assembly:**
   - Combine headland passes, sweep segments, and Voronoi transit paths
   - Generate poses with orientations for in-place rotation at swath turns
   - Output path suitable for FTCController (differential drive, in-place turns)

**Parameters (coverage_planner.yaml):**

```yaml
coverage_planner_node:
  ros__parameters:
    tool_width: 0.18                  # m – mower blade width and grid cell size
    headland_passes: 2                # number of perimeter passes
    headland_width: 0.18              # m – offset per pass
    map_frame: "map"
```

**Action Feedback:**

During planning, the action publishes feedback at each phase:
```
Phase: headland → mbb → grid → sweep → voronoi → complete
```

Allows behavior tree to monitor progress and detect hangs.

---

### 8. mowgli_monitoring

**Purpose:** System health diagnostics aggregation and external MQTT bridge.

**Location:** `src/mowgli_monitoring/`

**Architecture:**

```
Monitoring System
    │
    ├── DiagnosticsNode (1 Hz publish rate)
    │   │
    │   ├── Subscriptions (sensor QoS):
    │   │   ├── /status (Status) – mower state, sensors
    │   │   ├── /emergency (Emergency) – emergency status
    │   │   ├── /power (Power) – battery voltage, charging
    │   │   ├── /imu/data_raw (sensor_msgs/Imu)
    │   │   ├── /scan (sensor_msgs/LaserScan)
    │   │   ├── /wheel_odom (nav_msgs/Odometry)
    │   │   └── /gps/fix (sensor_msgs/NavSatFix)
    │   │
    │   └── Diagnostic Checks (aggregated to DiagnosticArray):
    │       ├── check_hardware_bridge() – last status age, mower state
    │       ├── check_emergency() – latched/active emergency status
    │       ├── check_battery() – voltage → SOC %, charger status
    │       ├── check_imu() – data freshness
    │       ├── check_lidar() – scan freshness, obstacles
    │       ├── check_gps() – fix type, lat/lon, age
    │       ├── check_odometry() – wheel odom freshness
    │       └── check_motors() – ESC/motor temperature
    │
    └── MqttBridgeNode (optional, for cloud telemetry)
        └── Republishes selected diagnostics to MQTT broker
            └── Topic pattern: /mowgli/diagnostics/{subsystem}
```

**Diagnostic Levels:**
- **OK** – All systems nominal
- **WARN** – Degraded but operational (e.g., GPS float, high temp)
- **ERROR** – Critical failure (e.g., no GPS fix, emergency active)
- **STALE** – Data stream timeout (sensor not reporting)

**Health Classification Functions:**

```cpp
uint8_t classify_freshness(age_sec, never, warn_sec, error_sec)
  // Returns OK, WARN, or ERROR based on age threshold

uint8_t classify_battery(percentage, warn_pct, error_pct)
  // Returns OK, WARN, or ERROR based on SOC threshold

uint8_t classify_temperature(temp_c, warn_c, error_c)
  // Returns OK, WARN, or ERROR based on temperature threshold
```

**Parameters (monitoring.yaml):**

```yaml
diagnostics_node:
  ros__parameters:
    publish_rate: 1.0                    # Hz – how often to aggregate
    freshness_warn_sec: 5.0              # sensor data age before warn
    freshness_error_sec: 10.0            # sensor data age before error
    battery_warn_pct: 20.0               # SOC % before warn
    battery_error_pct: 10.0              # SOC % before error
    motor_temp_warn_c: 60.0
    motor_temp_error_c: 80.0
```

**Output:**

Publishes `diagnostic_msgs/DiagnosticArray` to `/diagnostics` topic:
- Used by system monitors, RViz diagnostics viewer, and external dashboards
- Also ingested by BehaviorTree condition nodes (e.g., IsLocalizationHealthy, IsBatteryLow)

---

### 9. mowgli_simulation

**Purpose:** Gazebo Harmonic simulation environment with ros_gz_bridge integration.

**Location:** `src/mowgli_simulation/`

**Architecture:**

```
Simulation Stack (ros_gz_sim + ros_gz_bridge)
    │
    ├── Gazebo Harmonic Server
    │   └── World SDF (garden.sdf or empty_garden.sdf)
    │       └── Robot model (model.sdf)
    │           ├── Physics (differential drive plugin)
    │           ├── LiDAR sensor (Gazebo plugin publishes Gazebo topic)
    │           ├── IMU sensor (Gazebo plugin)
    │           └── Caster wheels (collision only, no TF)
    │
    ├── ros_gz_bridge (YAML-configured)
    │   └── Bridges Gazebo topics ↔ ROS2 topics
    │       ├── gz/model/mowgli_mower/cmd_vel → /cmd_vel (Twist)
    │       ├── gz/model/mowgli_mower/scan ← /scan (LaserScan)
    │       ├── gz/model/mowgli_mower/imu ← /imu/data_raw (Imu)
    │       └── gz/model/mowgli_mower/odometry ← /odom (Odometry, if available)
    │
    ├── Static TF Bridges (identity transforms)
    │   ├── lidar_link → mowgli_mower/laser_link/lidar_sensor
    │   └── imu_link → mowgli_mower/base_link/imu_sensor
    │
    └── ROS2 Stack
        └── (identical to real hardware)
            ├── robot_state_publisher (URDF)
            ├── Nav2 stack
            ├── SLAM (if enabled)
            └── Behavior tree
```

**Launch File: simulation.launch.py**

```python
Sequence:
  1. Declare arguments (world, use_rviz, headless, spawn_x/y/z/yaw)
  2. Launch Gazebo Ignition with SDF world file
  3. Spawn mowgli_mower model at (spawn_x, spawn_y, spawn_z, spawn_yaw)
  4. Start robot_state_publisher (URDF from mowgli_bringup)
  5. Start ros_gz_bridge with gazebo_bridge.yaml
  6. Create static TF bridges (Gazebo sensor frames → URDF frames)
  7. Optionally start RViz2 (mowgli_sim.rviz config)
```

**Gazebo Worlds:**

| World | File | Purpose |
|-------|------|---------|
| garden | worlds/garden.sdf | Realistic lawn with obstacles, trees, slope |
| empty_garden | worlds/empty_garden.sdf | Flat rectangular field (testing, no obstacles) |

**Robot Model (model.sdf):**

- **Differential drive plugin:** Subscribes to `cmd_vel`, drives wheels
- **LiDAR plugin:** 16-beam, 25 m range, publishes point cloud + LaserScan
- **IMU plugin:** 6-DOF gyro + accel, publishes Imu messages
- **Gazebo physics:** ODE or Bullet engine with friction/gravity
- **Collision meshes:** Wheel contact points, chassis boundary
- **Visual models:** 3D mesh for rendering

**Bridge Configuration (gazebo_bridge.yaml):**

```yaml
# Example bridge configuration
bridges:
  - topic_name: "/cmd_vel"
    ros_type_name: "geometry_msgs/msg/Twist"
    gz_type_name: "gz.msgs.Twist"
    direction: ROS_TO_GZ

  - topic_name: "/scan"
    ros_type_name: "sensor_msgs/msg/LaserScan"
    gz_type_name: "gz.msgs.LaserScan"
    direction: GZ_TO_ROS

  - topic_name: "/imu/data_raw"
    ros_type_name: "sensor_msgs/msg/Imu"
    gz_type_name: "gz.msgs.IMU"
    direction: GZ_TO_ROS
```

**Usage:**

```bash
# Full simulation with RViz
ros2 launch mowgli_simulation simulation.launch.py

# Headless (CI/testing, no GUI)
ros2 launch mowgli_simulation simulation.launch.py headless:=true use_rviz:=false

# Custom world
ros2 launch mowgli_simulation simulation.launch.py world:=/path/to/custom.sdf
```

The simulated robot is fully compatible with the real robot's ROS2 stack, allowing testing of Nav2, behavior trees, and coverage planning without hardware.

---

### 10. mowgli_map

**Purpose:** Map storage, persistence, and serving for offline navigation.

**Location:** `src/mowgli_map/`

**Features:**
- Loads pre-recorded SLAM maps from disk
- Serves /map topic (occupancy grid) to Nav2
- Persists maps generated during online SLAM runs
- Supports multi-map environments (e.g., different properties/zones)

---

## Custom Navigate-to-Pose Behavior Tree

Nav2's internal behavior tree is extended with a **GoalCheckerSelector** node to support the dual goal-checker architecture:

**File:** `src/mowgli_bringup/config/navigate_to_pose.xml`

```xml
<BehaviorTree ID="NavigateToPose">
  <Fallback name="Root">
    <!-- Try path-following with stopped_goal_checker (transit mode) -->
    <Sequence name="TransitSequence">
      <GoalCheckerSelector goal_checker="stopped_goal_checker"/>
      <FollowPath path="global_path"/>
    </Sequence>

    <!-- Fallback to coverage goal-checker (coverage mode) -->
    <Sequence name="CoverageSequence">
      <GoalCheckerSelector goal_checker="coverage_goal_checker"/>
      <FollowCoveragePath path="coverage_path"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
```

The **GoalCheckerSelector** invokes the appropriate goal checker based on the current navigation mode, allowing different success criteria for transit (full orientation alignment) vs. coverage (path completion index).

---

## Foxglove Bridge Integration

Instead of rosbridge_suite, the system uses **Foxglove Bridge** for remote web UI and telemetry:

**Port:** 8765 (WebSocket)

**Benefits:**
- Modern TypeScript client library
- Native ROS2 support (Foxglove Studio)
- Lower latency than rosbridge
- Reduced CPU overhead

**Launch:** Included in main bringup
```yaml
foxglove_bridge:
  port: 8765
  num_threads: 2
```

---

## Complete Data Flow Diagram

### Scenario: Autonomous Coverage Mowing Run

```
1. User sends START command via GUI (or mobile app via Foxglove Bridge)
   └─→ /high_level_control message: COMMAND_START (1)

2. BehaviorTree (10 Hz):
   └─→ MowingSequence triggered:
       ├─ SetMowerEnabled(true) → blade motor on
       ├─ PublishHighLevelStatus(UNDOCKING)
       └─ PlanCoveragePath action:
            └─→ mowgli_brv_planner processes goal
                ├─ 1. Headland generation (perimeter passes)
                ├─ 2. MBB optimal sweep direction (rotating calipers)
                ├─ 3. Grid-based boustrophedon sweep (obstacle-aware)
                ├─ 4. Voronoi roadmap transit (inter-region Dijkstra)
                └─ 5. Publishes coverage_path and coverage_outline
                    [BT receives path in result]

3. Navigation to coverage start:
   NavigateToPose(first_waypoint):
     ├─ Nav2 planner: global path from odometry to start
     ├─ RPP controller (RegulatedPurePursuit + RotationShimController)
     ├─ Costmap: /scan + odom → local obstacles
     └─ cmd_vel → hardware_bridge → STM32 → wheels

4. Coverage path following (CoverageWithRecovery loop):
   FollowCoveragePath:
     ├─ FTCController (Follow-the-Carrot with 3-axis PID, <10mm lateral accuracy)
     ├─ State machine: PRE_ROTATE → FOLLOWING → WAITING_FOR_GOAL_APPROACH → POST_ROTATE
     ├─ In-place rotation at swath turns (differential drive)
     ├─ Oscillation detection (FailureDetector)
     ├─ Obstacle avoidance (costmap checking)
     └─ Updates: state → WAITING_FOR_GOAL_APPROACH → POST_ROTATE → FINISHED
        Returns: RUNNING (in progress), SUCCESS (path complete), FAILURE (stuck)

5. Feedback loop (real-time):
   STM32 (100 Hz):
     ├─ LL_STATUS packet (encoder ticks, IMU, sensors, rain detection)
     └─→ hardware_bridge

   wheel_odometry_node (50 Hz):
     ├─ Integrates left/right encoder ticks
     └─→ /wheel_odom (odometry only, high drift)

   imu_filter_madgwick (50 Hz):
     ├─ Fuses IMU gyro + accel
     └─→ /imu/data (filtered orientation)

   robot_localization (ekf_odom, 50 Hz):
     ├─ Fuses /wheel_odom + /imu/data
     ├─ Output: /odometry/filtered_odom (local estimate, odom frame)
     └─→ /tf: odom → base_link

   GPS fusion (ekf_map, 20 Hz, if RTK fix):
     ├─ /gps/rtk_fix → gps_pose_converter → /gps/pose (ENU)
     ├─ Fuses /odometry/filtered_odom + /gps/pose
     ├─ Output: /odometry/filtered_map (global estimate, map frame)
     └─→ /tf: map → odom (corrects drift)

   SLAM (slam_toolbox, async):
     ├─ /scan + /odometry/filtered_odom
     ├─ Output: /map (occupancy grid)
     └─→ /tf: map → odom (loop closure correction)

   FTCController (10 Hz, coverage) / RPP Controller (10 Hz, transit):
     ├─ Reads robot pose from /tf: odom → base_link
     ├─ FTC: path-indexed PID on longitudinal, lateral, angular axes
     ├─ RPP: regulated pure pursuit with curvature-based speed scaling (transit only)
     └─→ /cmd_vel (geometry_msgs/Twist)

6. Command routing (twist_mux, priority-based):
   /cmd_vel sources:
     ├─ /cmd_vel_emergency (highest priority)
     ├─ /cmd_vel_teleop (manual override)
     └─ /cmd_vel_nav (navigation, from Nav2)
   Route to:
     └─→ /hardware_bridge/cmd_vel

7. Hardware bridge → STM32:
   /cmd_vel (Twist) → LlCmdVel packet (COBS + CRC16) → USB serial

8. STM32 motor control:
   LlCmdVel:
     ├─ linear.x → left/right ESC PWM (duty cycle)
     ├─ angular.z → differential for steering
     └─ Watchdog: expects heartbeat every 250 ms (4 Hz)
        If no heartbeat: safe stop (motor PWM cut)

9. Safety monitoring (BehaviorTree, 10 Hz):
   Condition checks:
     ├─ IsEmergency (latched_emergency bit)
     ├─ NeedsDocking (battery < 20 V)
     ├─ IsLocalizationHealthy (EKF variance < threshold)
     └─ IsCommand (COMMAND_START still active)
   On failure:
     ├─ SetMowerEnabled(false)
     ├─ StopMoving() → /cmd_vel = 0
     └─ PublishHighLevelStatus(RECOVERING or DOCKING)

10. Completion:
    FollowCoveragePath returns SUCCESS:
      ├─ Robot completed coverage path
      ├─ All path indices traversed
      └─ Final orientation aligned

    BehaviorTree continues:
      ├─ SetMowerEnabled(false) → blade off
      ├─ PublishHighLevelStatus(MOWING_COMPLETE)
      ├─ NavigateToPose(dock_pose) → return to dock
      └─ PublishHighLevelStatus(IDLE_DOCKED)

11. Telemetry (Foxglove Bridge, 8765/ws):
    → Web UI receives:
       ├─ /odometry/filtered_map (robot pose on map)
       ├─ /coverage_path and /coverage_outline (visualization)
       ├─ /scan (LiDAR pointcloud)
       ├─ /diagnostics (system health)
       └─ /tf tree (all frame transformations)
```

---

## TF Tree Reference

**Standard ROS2 conventions (REP-103 + REP-105):**

```
map (SLAM origin, or GPS-corrected)
  │ [published by ekf_map in robot_localization @ 20 Hz]
  │
  odom (local dead-reckoning origin)
  │ [published by ekf_odom in robot_localization @ 50 Hz]
  │
  base_link (robot body frame, wheel axle height)
  │ [published by robot_state_publisher from URDF]
  │
  ├── base_footprint (ground contact point, fixed offset from base_link)
  │   └── used by Nav2 costmap for footprint inflation
  │
  ├── imu_link (fixed to chassis, IMU measurement frame)
  │   └── [hardware_bridge publishes IMU data in this frame]
  │
  ├── laser_link (fixed to chassis, LiDAR origin)
  │   └── [SLAM and costmap read /scan in this frame]
  │
  ├── gps_link (fixed to antenna, GPS measurement point)
  │
  ├── left_wheel_link (rotating joint, visual only)
  │
  └── right_wheel_link (rotating joint, visual only)
```

**Frame Hierarchy:**

| Frame | Publisher | Rate | Purpose |
|-------|-----------|------|---------|
| map | ekf_map (robot_localization) | 20 Hz | Global origin (SLAM/GPS) |
| odom | ekf_odom (robot_localization) | 50 Hz | Local dead-reckoning origin |
| base_link | robot_state_publisher | Static | Robot body (from URDF) |
| base_footprint | robot_state_publisher | Static | Footprint inflation center |
| imu_link | robot_state_publisher | Static | IMU sensor frame |
| laser_link | robot_state_publisher | Static | LiDAR origin |
| gps_link | robot_state_publisher | Static | GPS antenna location |
| [Gazebo sensor frames] | (simulation only) | Static | Gazebo model sensor origins |

**Frame Conventions:**

- `map` – Global frame, typically z-up, x-east, y-north (REP-103). Set by first SLAM loop closure or GPS initial fix.
- `odom` – Local odometry frame, z-up. Origin drifts due to integration error; corrected by ekf_map.
- `base_link` – Robot body frame. Wheel axle height z = 0 (convention). Xy center at robot geometric center.
- `base_footprint` – Projection of base_link onto z = 0 (ground level). Used for Nav2 footprint inflation.

**Simulation (Gazebo Harmonic):**

Static TF bridges connect Gazebo sensor frame names to URDF frames:
- `mowgli_mower/laser_link/lidar_sensor` ↔ `laser_link` (identity)
- `mowgli_mower/base_link/imu_sensor` ↔ `imu_link` (identity)

This allows SLAM, costmap, and other nodes to use standard ROS2 frame names regardless of simulation vs. real hardware.

---

## Topic Map

**Publishers (Sources):**

| Topic | Type | Publisher | Rate | Purpose |
|-------|------|-----------|------|---------|
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox | 1 Hz | Occupancy map from SLAM (if enabled) |
| `/scan` | sensor_msgs/LaserScan | Gazebo bridge / LiDAR driver | 10 Hz | LiDAR range data (Gazebo simulated or real) |
| `/imu/data` | sensor_msgs/Imu | imu_filter_madgwick | 50 Hz | Filtered IMU (9-DOF fusion) |
| `/imu/data_raw` | sensor_msgs/Imu | hardware_bridge_node | 100 Hz | Raw accelerometer + gyroscope |
| `/status` | mowgli_interfaces/Status | hardware_bridge_node | 100 Hz | Mower state (blade on/off, rain, charging) |
| `/emergency` | mowgli_interfaces/Emergency | hardware_bridge_node | 100 Hz | Emergency stop status (latched, active) |
| `/power` | mowgli_interfaces/Power | hardware_bridge_node | 100 Hz | Battery voltage, charging current |
| `/wheel_odom` | nav_msgs/Odometry | wheel_odometry_node | 50 Hz | Dead-reckoning from wheel encoders |
| `/gps/rtk_fix` | sensor_msgs/NavSatFix | GPS hardware driver | 10–20 Hz | RTK fix (latitude, longitude, altitude) |
| `/gps/pose` | geometry_msgs/PoseWithCovarianceStamped | gps_pose_converter_node | 10–20 Hz | RTK position converted to local ENU |
| `/odometry/filtered_odom` | nav_msgs/Odometry | ekf_odom (robot_localization) | 50 Hz | Local estimate (odom frame) |
| `/odometry/filtered_map` | nav_msgs/Odometry | ekf_map (robot_localization) | 20 Hz | Global estimate (map frame) with GPS correction |
| `/localization/status` | diagnostic_msgs/DiagnosticStatus | localization_monitor_node | 2 Hz | EKF health, degradation mode |
| `/coverage_planner_node/coverage_path` | nav_msgs/Path | coverage_planner_node | Once per plan | B-RV coverage path with poses |
| `/coverage_planner_node/coverage_outline` | nav_msgs/Path | coverage_planner_node | Once per plan | Headland outline for visualization |
| `/path` | nav_msgs/Path | planner_server (Nav2) | 1 Hz | Global path from Nav2 planner |
| `/cmd_vel` | geometry_msgs/Twist | twist_mux | 10–50 Hz | Final velocity command (to hardware/Gazebo) |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | diagnostics_node (mowgli_monitoring) | 1 Hz | System health aggregation |
| `/high_level_status` | std_msgs/UInt8 | behavior_tree_node | 10 Hz | Current high-level mode (IDLE, MOWING, DOCKING) |

**Subscribers (Sinks):**

| Topic | Subscriber | Purpose |
|-------|-----------|---------|
| `/cmd_vel` | hardware_bridge_node / Gazebo | Motor/wheel commands |
| `/scan` | slam_toolbox, nav2_local_costmap, diagnostics_node | SLAM, obstacle detection, monitoring |
| `/imu/data_raw` | diagnostics_node | Monitor IMU freshness |
| `/odometry/filtered_odom` | slam_toolbox, nav2 controller, diagnostics_node | SLAM input, localized pose for control |
| `/odometry/filtered_map` | nav2 planner, behavior_tree (goal comparison), diagnostics_node | Global navigation reference |
| `/gps/rtk_fix` | gps_pose_converter_node, diagnostics_node | Convert fix to local frame, monitor GPS |
| `/status` | behavior_tree_node, localization_monitor_node, diagnostics_node | Health checks, sensor freshness |
| `/emergency` | behavior_tree_node, diagnostics_node | Emergency monitoring |
| `/power` | behavior_tree_node, diagnostics_node | Battery level monitoring |
| `/wheel_odom` | diagnostics_node | Monitor odometry freshness |
| `/coverage_planner_node/coverage_path` | FollowCoveragePath BT node, diagnostics_node (optional) | Coverage execution |

**Services (Request-Response):**

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/mower_control` | MowerControl | hardware_bridge_node | behavior_tree_node | Enable/disable blade motor |
| `/emergency_stop` | EmergencyStop | hardware_bridge_node | behavior_tree_node, safety system | Assert/release latched emergency |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | behavior_tree_node | Send goal to Nav2 |
| `/robot_localization/set_pose` | robot_localization/SetPose | ekf_odom | startup/testing tools | Initialize odometry origin |

**Actions (Async Request-Response):**

| Action | Type | Server | Client | Purpose |
|--------|------|--------|--------|---------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | behavior_tree_node (NavigateToPose BT node) | Non-blocking navigation goal |
| `/plan_coverage` | mowgli_interfaces/PlanCoverage | coverage_planner_node (mowgli_brv_planner) | behavior_tree_node (PlanCoveragePath BT node) | Coverage path planning with feedback |

---

## Summary: Architectural Principles

1. **12-Package Modular Design:** Separation of concerns across hardware, localization, navigation, planning, monitoring, and behavior layers.
   - **Core:** mowgli_interfaces (message definitions)
   - **Hardware:** mowgli_hardware (STM32 bridge via COBS)
   - **Perception:** mowgli_localization (EKF fusion, multi-sensor)
   - **Control:** mowgli_nav2_plugins (RPP for transit, FTCController for coverage)
   - **Planning:** mowgli_brv_planner (B-RV algorithm, Voronoi roadmap transit)
   - **Behavior:** mowgli_behavior (BehaviorTree.CPP, 10 Hz reactive control)
   - **Monitoring:** mowgli_monitoring (diagnostics, MQTT bridge)
   - **Simulation:** mowgli_simulation (Gazebo Harmonic, ros_gz_bridge)
   - **Infrastructure:** mowgli_bringup (launch, config), mowgli_description (URDF/xacro), mowgli_map (map storage)
   - **Third-party:** opennav_coverage (Nav2 coverage server)

2. **ROS2 Jazzy + Gazebo Harmonic:** Modern robotics stack with first-class simulation support and lifecycle management.

3. **Decoupled Communication:** ROS2 pub/sub (topics), services, and actions isolate packages. Easy to substitute, test, or extend components independently.

4. **Robust Serial Protocol (COBS + CRC-16):** Enables reliable bidirectional communication between Raspberry Pi and STM32 firmware over noisy USB at 115200 baud.

5. **Dual-Tier EKF Localization:**
   - Local (ekf_odom, 50 Hz): Fast odometry + IMU fusion for real-time control
   - Global (ekf_map, 20 Hz): Corrects drift using GPS or SLAM loop closures
   - Graceful degradation: operates without GPS in GNSS-denied areas

6. **Coverage Path Following:** FTCController (Follow-the-Carrot) is the active controller for FollowCoveragePath, achieving <10mm lateral accuracy with 3-axis PID control and in-place rotation at swath turns. RPP remains active for FollowPath (transit and docking).

7. **B-RV Coverage Planning:** Grid-based boustrophedon sweep with MBB optimal direction (rotating calipers on convex hull), obstacle-aware cell skipping, and Voronoi roadmap transit (Boost.Polygon Voronoi + Dijkstra shortest path) for inter-region connections. No Fields2Cover dependency.

8. **Reactive Behavior Trees (BehaviorTree.CPP v4):** 10 Hz non-preemptive tree execution with priority-based fallback selection:
   - Emergency guard: highest priority, interrupts all activity
   - Multi-mode state machine: IDLE → UNDOCKING → MOWING (with recovery) → DOCKING
   - Composable async action nodes (NavigateToPose, PlanCoveragePath, FollowCoveragePath)

9. **Priority-Based Command Routing:** twist_mux mediates three command sources (emergency > teleoperation > navigation) before forwarding to hardware bridge.

10. **Comprehensive Health Monitoring:** Diagnostics aggregator tracks 8 subsystems (hardware bridge, emergency, battery, IMU, LiDAR, GPS, odometry, motors) with multi-level status (OK, WARN, ERROR, STALE) for autonomous decision-making.

11. **Unified Simulation-to-Hardware Workflow:**
    - Gazebo Harmonic with ros_gz_bridge enables identical ROS2 stack on both sim and real hardware
    - Static TF bridges map Gazebo sensor frames to URDF frames
    - Behavior tree, Nav2, SLAM, and diagnostics unchanged between environments

12. **Foxglove Bridge for Remote UI:** Modern WebSocket-based telemetry (port 8765) replaces legacy rosbridge, reducing latency and CPU overhead.

---

## Development & Testing

**Key Resources:**

| Resource | Location | Purpose |
|----------|----------|---------|
| URDF/Xacro | src/mowgli_bringup/urdf/mowgli.urdf.xacro | Robot kinematics, sensor frames, collision geometry |
| SLAM Config | src/mowgli_bringup/config/slam_toolbox.yaml | Loop closure, occupancy grid updates |
| Nav2 Config | src/mowgli_bringup/config/nav2_params.yaml | Planner, controller, costmap tuning |
| Behavior Tree | src/mowgli_behavior/trees/main_tree.xml | High-level state machine and sequencing |
| Coverage Config | src/mowgli_brv_planner/config/coverage_planner.yaml | B-RV planner parameters (tool width, headland) |
| Gazebo Worlds | src/mowgli_simulation/worlds/ | garden.sdf (realistic), empty_garden.sdf (testing) |

**Testing Workflow:**

```bash
# Simulation (Gazebo Harmonic, full stack)
ros2 launch mowgli_simulation simulation.launch.py

# Real hardware (Raspberry Pi + STM32)
ros2 launch mowgli_bringup mowgli.launch.py

# SLAM + mapping
ros2 launch mowgli_bringup navigation.launch.py

# Coverage planning (standalone test)
ros2 service call /plan_coverage mowgli_interfaces/srv/PlanCoverage '{outer_boundary: {...}}'

# Diagnostics monitoring
ros2 topic echo /diagnostics
```

---

## Next Steps

- See [CONFIGURATION.md](CONFIGURATION.md) for parameter tuning (PID gains, speed profiles, costmap).
- See [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md) for STM32 packet protocol and firmware integration.
- See [SIMULATION.md](SIMULATION.md) for detailed Gazebo world setup and physics tuning.
- See [DEVELOPMENT.md](DEVELOPMENT.md) for build instructions and dev container setup.
