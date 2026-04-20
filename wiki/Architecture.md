# Mowgli ROS2 Architecture

> **⚠️ Sections below still describe the old SLAM-based architecture.**
> The stack has moved to **FusionCore** as the sole UKF localizer and
> **Kinematic-ICP** (PRBonn 2024) as an optional LiDAR drift corrector
> running on a parallel TF tree. Where this page and [CLAUDE.md](https://github.com/cedbossneo/mowglinext/blob/main/CLAUDE.md)
> disagree, CLAUDE.md is authoritative. The "System Overview" and
> "TF / Localization" sub-sections here are scheduled for rewrite —
> see the short summary below for the current architecture.

## Current architecture (one-page summary)

| Layer | Owner | Notes |
|---|---|---|
| `map → odom` | static identity transform | Published once at launch; the odom frame IS the GPS-ENU frame. |
| `odom → base_footprint` | FusionCore UKF @ 100 Hz | 22D quaternion state; fuses GPS, IMU, wheels, and optionally Kinematic-ICP twist on `/encoder2/odom`. |
| Kinematic-ICP | parallel TF tree | Reads `/scan` via a scan-frame relay onto `wheel_odom_raw → base_footprint_wheels → lidar_link_wheels`. Output goes only through the encoder2 adapter — no feedback into the main TF. |
| Map / areas | `map_server_node` | Polygon-based area DB + `mow_progress` GridMap layer, persisted to disk. No SLAM back-end. |

Comprehensive technical documentation of the Mowgli ROS2 system design follows. Ignore any "slam_toolbox", "Cartographer", or "Dual EKF" wording — those components are no longer in the stack.

Built on **ROS2 Kilted** with **Gazebo Harmonic** simulation, this architecture spans 12 focused packages providing complete autonomous lawn mower functionality.

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
| **mowgli_localization** | Multi-source localization (wheel odometry, IMU, RTK-GPS fusion via FusionCore UKF) | mowgli_interfaces, fusioncore_cpp |
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
- **HighLevelStatus.msg** – Behavior tree state with coverage progress
  ```
  uint8 HIGH_LEVEL_STATE_NULL=0
  uint8 HIGH_LEVEL_STATE_IDLE=1
  uint8 HIGH_LEVEL_STATE_AUTONOMOUS=2
  uint8 HIGH_LEVEL_STATE_RECORDING=3
  uint8 HIGH_LEVEL_STATE_MANUAL_MOWING=4

  uint8 state
  string state_name
  string sub_state_name
  int16 current_area / current_path / current_path_index
  int16 total_swaths / completed_swaths / skipped_swaths
  float32 gps_quality_percent / battery_percent
  bool is_charging / emergency
  ```
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

- **HighLevelControl.srv** – Behavior tree mode commands from GUI
  ```
  Constants:
    COMMAND_START=1             # Begin autonomous mowing
    COMMAND_HOME=2              # Return to dock
    COMMAND_RECORD_AREA=3       # Start area recording (drive boundary)
    COMMAND_S2=4                # Mow next area
    COMMAND_RECORD_FINISH=5     # Finish recording, save polygon
    COMMAND_RECORD_CANCEL=6     # Cancel recording, discard trajectory
    COMMAND_MANUAL_MOW=7        # Enter manual mowing mode (teleop + blade)
    COMMAND_RESET_EMERGENCY=254 # Reset latched emergency
    COMMAND_DELETE_MAPS=255     # Delete all maps

  Request:
    uint8 command
  Response:
    bool success
  ```

- **AddMowingArea.srv** – Save a mowing area polygon to map_server_node
  ```
  Request:
    MapArea area                  # Polygon defining the area
    bool is_navigation_area       # true=navigation, false=mowing
  Response:
    bool success
  ```

- **AreaRecording.srv** – Area recording lifecycle control
  ```
  Constants:
    COMMAND_START=1 / COMMAND_FINISH=2 / COMMAND_CANCEL=3

  Request:
    uint8 command
    string area_name
    bool is_exclusion_zone
  Response:
    bool success
    string message
    geometry_msgs/Polygon polygon
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

Source nodes:

1) wheel_odometry_node
   - Integrates RL/RR encoder ticks
   - Publishes /wheel_odom (Odometry)
   - 50 Hz

2) navsat_to_absolute_pose_node
   - RTK fix → local ENU pose (for GUI and BT reference only)
   - Publishes /gps/absolute_pose (PoseWithCovarianceStamped)
   - Variable rate (10-20 Hz depending on RTK health)

3) localization_monitor_node
   - Monitors FusionCore variance
   - Detects degradation (5 modes)
   - Publishes /localization/status (DiagnosticStatus)

↓

FusionCore Sensor Fusion (launched by mowgli_bringup):

FusionCore node (50 Hz, single UKF)
   - Fuses: /wheel_odom + /imu/data + /gps/fix (NavSatFix directly)
   - Output: /fusion/odom (nav_msgs/Odometry, odom → base_footprint)
   - Publishes TF: odom → base_footprint

↓

SLAM Toolbox (20 Hz):
   - Fuses: /scan + /fusion/odom
   - Output: /map (occupancy grid)
   - Publishes TF: map → odom (TF authority for map→odom)

↓

Final Output:
  /tf tree: map → odom → base_footprint
  /fusion/odom (sensor fusion result, both local and GPS-corrected)
  /map (occupancy grid from SLAM)
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
    ticks_per_meter: 300              # Encoder resolution
    timeout_period_ms: 5000            # Warn if no WheelTick for 5s
```

#### 3b. navsat_to_absolute_pose_node

**Inputs:**
- `/gps/rtk_fix` (sensor_msgs/NavSatFix with fix type indicator)

**Outputs:**
- `/gps/absolute_pose` (geometry_msgs/PoseWithCovarianceStamped)

**Purpose:** Convert NavSatFix (latitude/longitude) to local ENU pose for GUI visualization and Behavior Tree reference. Note that FusionCore fuses `/gps/fix` directly (no intermediate converter).

**Algorithm: GNSS to Local ENU**

```
1. GPS origin (datum) set from mowgli_robot.yaml (datum_lat, datum_long)
2. All fixes converted to ENU relative to datum origin:

   Δlat = lat - datum_lat
   Δlon = lon - datum_long
   Δalt = alt - datum_alt

   e = EARTH_RADIUS_M * Δlon * cos(datum_lat)
   n = EARTH_RADIUS_M * Δlat
   u = Δalt

   Output: [e, n, u] in local ENU frame

3. Covariance scaling based on RTK fix type:
   RTK Fixed:     covariance *= 1.0   (best, ~0.01-0.05 m)
   RTK Float:     covariance *= 10.0  (good, ~0.1-0.5 m)
   DGPS/SPS:      covariance *= 100.0 (poor, ~1-5 m)
   No fix:        skip publishing
```

**Parameters (navsat_to_absolute_pose.yaml):**
```yaml
navsat_to_absolute_pose:
  ros__parameters:
    map_frame_id: "map"
    earth_radius_m: 6371008.8
    # Datum origin read from mowgli_robot.yaml
```

#### 3c. localization_monitor_node

**Inputs:**
- `/fusion/odom` (from FusionCore UKF)
- `/status` (for wheel tick freshness)
- `/gps/rtk_fix` (for fix status)

**Outputs:**
- `/localization/status` (diagnostic_msgs/DiagnosticStatus)
- `/localization/mode` (std_msgs/String, for debug/logging)

**Degradation Modes (5 levels):**

| Level | Name | Condition | Response |
|-------|------|-----------|----------|
| 0 | OK | FusionCore healthy, all sensors fresh | Continue normally |
| 1 | ODOMETRY_STALE | No wheel ticks for 2s | Warn in logs, reduce planner timeout |
| 2 | GPS_TIMEOUT | No fix for 10s (RTK Float OK) | Use wheel odom only, increase drift tolerance |
| 3 | GPS_DEGRADED | RTK Float (not Fixed) | Use GPS but with higher variance |
| 4 | FILTER_DIVERGENCE | FusionCore variance > threshold | Emergency stop recommended |

**Parameters (localization_monitor.yaml):**
```yaml
localization_monitor:
  ros__parameters:
    odom_stale_timeout_sec: 2.0
    gps_timeout_sec: 10.0
    max_acceptable_fusion_variance: 0.25   # m² for position
```

#### 3d. FusionCore Diagnostics Integration

**Monitoring:**
The diagnostics_node monitors FusionCore health by subscribing to `/fusion/odom` and checking:
- **Rate:** Verify 50 Hz update rate (report WARN if < 20 Hz)
- **Position variance:** Monitor x, y, z components of pose covariance
- **Orientation (yaw):** Check yaw angle convergence and variance
- **Flat constraint:** Monitor if z-position error is within expected bounds (grass terrain variation)
- **Z-drift detection:** Flag if vertical variance grows uncontrolled (indicates filter divergence)

**Diagnostics Output:**
Aggregated into `/diagnostics` as sub-status with levels:
- **OK:** Rate nominal, variances converged, no drift
- **WARN:** Rate degraded or variance increasing
- **ERROR:** Rate critical or variance diverging (filter failure)

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
    ├── [SLAM Toolbox publishes map→odom @ 20 Hz]
    │
Odometry frame (local dead-reckoning origin)
    │
    ├── [FusionCore publishes odom→base_footprint @ 50 Hz]
    │
Base footprint (on ground, robot frame for Nav2/FusionCore)
    │
    ├── [robot_state_publisher outputs static TFs]
    │
Base link (rear wheel axle, OpenMower convention)
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
4. `FusionCore` (single UKF, lifecycle node) – Wheel odometry + IMU + GPS fusion
5. `wheel_odometry_node`, `imu_filter_madgwick`, `navsat_to_absolute_pose_node`, `localization_monitor_node` – Sensor preparation
6. (Optional) SLAM, Nav2, behavior tree nodes

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
            │       ├── PublishHighLevelStatus(EMERGENCY)
            │       └── Fallback: AutoResetOrWait
            │           ├── Sequence: DockAutoReset
            │           │   ├── IsCharging() → on dock?
            │           │   ├── ResetEmergency() → release firmware latch
            │           │   └── WaitForDuration(1.0s)
            │           └── WaitForDuration(1.0s)  (not on dock — retry)
            │
            ├── Fallback: BoundaryGuard
            │   ├── Inverter(IsBoundaryViolation) → continue if inside
            │   └── Sequence: BoundaryHandler
            │       ├── SetMowerEnabled(false), StopMoving()
            │       ├── RetryUntilSuccessful(5): BackUp + check boundary
            │       └── Fallback: BoundaryEmergencyStop
            │
            ├── Fallback: GPSModeSelector
            │   ├── IsGPSFixed → SetNavMode(precise)
            │   └── SetNavMode(degraded)
            │
            └── Fallback: MainLogic
                ├── Sequence: CriticalBatteryDock (battery < 10%)
                │   ├── IsBatteryLow(10.0)
                │   ├── SetMowerEnabled(false), StopMoving()
                │   ├── SaveObstacles(), SaveSlamMap()
                │   └── DockRobot() → IDLE_DOCKED, ClearCommand
                │
                ├── Sequence: MowingSequence (COMMAND_START = 1)
                │   ├── IsCommand(1)
                │   ├── Undock (with GPS wait, SLAM save, heading calibration)
                │   ├── Multi-area coverage loop:
                │   │   └── Repeat(until no more areas): AreaLoop
                │   │       ├── GetNextUnmowedArea() — fetch next unmowed area polygon
                │   │       ├── Cell-based strip coverage loop:
                │   │       │   ├── ReactiveSequence: MowingCommandGuard
                │   │       │   │   ├── IsCommand(1) — abort if user cancels
                │   │       │   │   ├── RainGuard — dock, wait, resume
                │   │       │   │   ├── BatteryGuard — dock, charge to 95%, resume
                │   │       │   │   └── Repeat(500): StripLoop
                │   │       │   │       ├── GetNextStrip(current_area)
                │   │       │   │       ├── TransitToStrip (RPP)
                │   │       │   │       └── FollowStrip (FTCController)
                │   │       │   └── FailedCoverageDock
                │   │       └── Mark area as complete, continue to next area
                │   └── All areas complete → disable blade, save, dock → IDLE_DOCKED
                │
                ├── Sequence: HomeSequence (COMMAND_HOME = 2)
                │   ├── IsCommand(2)
                │   ├── SetMowerEnabled(false), StopMoving()
                │   ├── SaveObstacles(), SaveSlamMap()
                │   └── DockRobot() → IDLE_DOCKED, ClearCommand
                │
                ├── Sequence: RecordingSequence (COMMAND_RECORD_AREA = 3)
                │   ├── IsCommand(3)
                │   ├── PublishHighLevelStatus(RECORDING)
                │   ├── RecordArea (records trajectory at 2 Hz, Douglas-Peucker
                │   │   simplification, saves polygon via /map_server_node/add_area)
                │   │   Listens for COMMAND_RECORD_FINISH=5 or COMMAND_RECORD_CANCEL=6
                │   ├── PublishHighLevelStatus(RECORDING_COMPLETE)
                │   └── ClearCommand
                │
                ├── Sequence: ManualMowingSequence (COMMAND_MANUAL_MOW = 7)
                │   ├── IsCommand(7)
                │   ├── PublishHighLevelStatus(MANUAL_MOWING)
                │   └── WaitForDuration(0.5s)  (teleop via /cmd_vel_teleop)
                │
                └── Sequence: IdleSequence (default)
                    ├── SetMowerEnabled(false), StopMoving()
                    ├── PublishHighLevelStatus(IDLE)
                    └── WaitForDuration(0.5s)

Update frequency: 10 Hz tick() cycle
Execution pattern: ReactiveSequence re-evaluates all children each tick
```

**Tree Structure (from main_tree.xml):**

The tree implements a priority-based fallback selector with reactive guards:
1. **Emergency Guard (highest priority):** If emergency active → disable, stop, halt. Auto-resets when robot is placed on dock (charging detected) by calling `ResetEmergency` — firmware decides whether to actually clear the latch based on physical trigger state.
2. **Boundary Guard:** If outside mowing area → stop, back up (up to 5 attempts), emergency stop if still outside.
3. **GPS Mode Selector:** Switch between precise (RTK) and degraded navigation modes.
4. **Critical Battery Dock:** If battery < 10% → dock immediately (uninterruptible).
5. **Mowing (COMMAND_START=1):** Multi-area coverage: iterates through all unmowed areas via GetNextUnmowedArea, then cell-based strip coverage per area with GPS wait, heading calibration from undock, rain/battery reactive guards, strip-by-strip execution via GetNextStrip/TransitToStrip/FollowStrip. Coverage progress tracked per area.
6. **Home (COMMAND_HOME=2):** Return to dock on user request.
7. **Area Recording (COMMAND_RECORD_AREA=3):** Drive the boundary, record trajectory at 2 Hz, finish (cmd 5) saves Douglas-Peucker simplified polygon via map_server_node, cancel (cmd 6) discards.
8. **Manual Mowing (COMMAND_MANUAL_MOW=7):** Teleop via `/cmd_vel_teleop` (twist_mux priority). Blade managed by GUI (fire-and-forget to firmware). Collision_monitor, GPS, SLAM all remain active.
9. **Idle (default):** Standby, periodic status updates.

Each sequence transitions through defined high-level states (NULL=0, IDLE=1, AUTONOMOUS=2, RECORDING=3, MANUAL_MOWING=4) published via HighLevelStatus.msg for GUI and firmware synchronization.

#### Condition Nodes (condition_nodes.cpp)

```cpp
class IsEmergency : public BT::ConditionNode
// Returns SUCCESS if active_emergency bit set

class IsCharging : public BT::ConditionNode
// Returns SUCCESS if dock charging state is active

class IsBatteryLow : public BT::ConditionNode
// Checks battery below threshold

class NeedsDocking : public BT::ConditionNode
// Checks battery_voltage < threshold parameter (default 20.0 V)

class IsBatteryAbove : public BT::ConditionNode
// Checks battery_percent > threshold (used for charge-to-95% logic)

class IsCommand : public BT::ConditionNode
// Port In: command (uint8)
// Returns SUCCESS if command matches current high-level command from GUI

class IsGPSFixed : public BT::ConditionNode
// Returns SUCCESS if GPS has RTK fix

class IsBoundaryViolation : public BT::ConditionNode
// Returns SUCCESS if robot is outside mowing area boundary

class IsRainDetected : public BT::ConditionNode
// Returns SUCCESS if rain sensor detects rain

class IsNewRain : public BT::ConditionNode
// Returns SUCCESS only on new rain onset (not if it was raining at start)

class IsResumeUndockAllowed : public BT::ConditionNode
// Tracks resume-undock attempts (max_attempts port), prevents infinite loops

class IsChargingProgressing : public BT::ConditionNode
// Returns SUCCESS if charger is active and battery level increasing

class ReplanNeeded : public BT::ConditionNode
// Returns SUCCESS if coverage replanning is required
```

#### Action Nodes (action_nodes.cpp, utility_nodes.cpp, recording_nodes.cpp)

```cpp
class NavigateToPose : public BT::AsyncActionNode
// Contacts Nav2 /navigate_to_pose action server
// Port In: goal="x;y;yaw" (string format)
// Returns RUNNING (in progress), SUCCESS (reached), FAILURE (abort/timeout)

class SetMowerEnabled : public BT::ActionNode
// Calls /mower_control service
// Port In: enabled (bool)
// Fire-and-forget; always returns SUCCESS (or gracefully continues in simulation)

class StopMoving : public BT::ActionNode
// Publishes zero Twist to /cmd_vel
// Returns SUCCESS

class PublishHighLevelStatus : public BT::ActionNode
// Publishes HighLevelStatus.msg (state enum + state_name string + coverage progress)
// Port In: state (uint8), state_name (string)
// Returns SUCCESS

class WaitForDuration : public BT::ActionNode
// Sleep for specified duration
// Port In: duration_sec (double)
// Returns SUCCESS after duration elapsed

class ClearCommand : public BT::ActionNode
// Clears the pending high-level command (e.g., COMMAND_START)
// Returns SUCCESS

class ClearCostmap : public BT::ActionNode
// Clears Nav2 local costmap
// Returns SUCCESS

class SaveSlamMap : public BT::ActionNode
// Persists SLAM map to disk
// Port In: map_path (string)
// Returns SUCCESS

class SaveObstacles : public BT::ActionNode
// Persists tracked obstacles to disk
// Returns SUCCESS

class SetNavMode : public BT::ActionNode
// Switches between "precise" (RTK) and "degraded" navigation modes
// Port In: mode (string)
// Returns SUCCESS

class BackUp : public BT::ActionNode
// Drives robot backward
// Port In: backup_dist (double), backup_speed (double)
// Returns SUCCESS/FAILURE

class ResetEmergency : public BT::ActionNode
// Calls /hardware_bridge/emergency_stop with emergency=false to release firmware latch
// Firmware is safety authority — only clears if physical trigger is no longer asserted
// Returns SUCCESS/FAILURE

class RecordUndockStart : public BT::ActionNode
// Records robot position at start of undock for heading calibration
// Returns SUCCESS

class CalibrateHeadingFromUndock : public BT::ActionNode
// Reads EKF TF to compute heading after undock, clears costmaps
// Returns SUCCESS

class WasRainingAtStart : public BT::ActionNode
// Records rain state at mowing start (to distinguish new rain from ongoing)
// Returns SUCCESS

class RecordResumeUndockFailure : public BT::ActionNode
// Increments resume-undock failure counter
// Returns SUCCESS

class DockRobot : public BT::AsyncActionNode
// Uses opennav_docking to dock the robot
// Port In: dock_id, dock_type
// Returns RUNNING/SUCCESS/FAILURE

class UndockRobot : public BT::AsyncActionNode
// Uses opennav_docking to undock the robot
// Port In: dock_type
// Returns RUNNING/SUCCESS/FAILURE

// Multi-area coverage node:
class GetNextUnmowedArea : public BT::AsyncActionNode
// Fetches next unmowed area from map_server_node ~/get_next_unmowed_area
// Returns SUCCESS with area polygon and coverage status, FAILURE when all areas complete

// Cell-based coverage nodes:
class GetNextStrip : public BT::AsyncActionNode
// Fetches next unvisited strip from map_server_node ~/get_next_strip
// Port In: area_index
// Returns SUCCESS with strip path, FAILURE when current area coverage complete

class TransitToStrip : public BT::AsyncActionNode
// Navigates to start of current strip using Nav2 (RPP controller)
// Returns RUNNING/SUCCESS/FAILURE

class FollowStrip : public BT::AsyncActionNode
// Follows current strip using Nav2 (FTCController for <10mm accuracy)
// Returns RUNNING/SUCCESS/FAILURE

// Area recording node:
class RecordArea : public BT::StatefulActionNode
// Records robot trajectory at configurable rate while user drives boundary
// Douglas-Peucker simplification, saves polygon via /map_server_node/add_area
// Publishes live preview on ~/recording_trajectory
// Port In: simplification_tolerance, min_vertices, min_area, record_rate_hz, is_exclusion_zone
// Returns RUNNING (recording), SUCCESS (finish cmd 5), FAILURE (cancel cmd 6 or error)
```

#### Tree Control (BehaviorTreeNode)

**Subscriptions:**
- `/status` – Mower state, rain sensor, blade status
- `/emergency` – Latched emergency flag
- `/power` – Battery voltage (v_battery)
- `/high_level_control` (service) – Receive mode commands from GUI (START, HOME, S1, S2, RECORD_AREA, RECORD_FINISH, RECORD_CANCEL, MANUAL_MOW)

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
void registerAllNodes(BT::BehaviorTreeFactory& factory) {
  // Condition nodes
  factory.registerNodeType<IsEmergency>("IsEmergency");
  factory.registerNodeType<IsCharging>("IsCharging");
  factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
  factory.registerNodeType<IsRainDetected>("IsRainDetected");
  factory.registerNodeType<NeedsDocking>("NeedsDocking");
  factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove");
  factory.registerNodeType<IsCommand>("IsCommand");
  factory.registerNodeType<IsGPSFixed>("IsGPSFixed");
  factory.registerNodeType<ReplanNeeded>("ReplanNeeded");
  factory.registerNodeType<IsBoundaryViolation>("IsBoundaryViolation");
  factory.registerNodeType<IsNewRain>("IsNewRain");
  factory.registerNodeType<IsResumeUndockAllowed>("IsResumeUndockAllowed");
  factory.registerNodeType<IsChargingProgressing>("IsChargingProgressing");

  // Action nodes
  factory.registerNodeType<SetMowerEnabled>("SetMowerEnabled");
  factory.registerNodeType<StopMoving>("StopMoving");
  factory.registerNodeType<ClearCostmap>("ClearCostmap");
  factory.registerNodeType<PublishHighLevelStatus>("PublishHighLevelStatus");
  factory.registerNodeType<WaitForDuration>("WaitForDuration");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<SaveSlamMap>("SaveSlamMap");
  factory.registerNodeType<BackUp>("BackUp");
  factory.registerNodeType<ClearCommand>("ClearCommand");
  factory.registerNodeType<SaveObstacles>("SaveObstacles");
  factory.registerNodeType<SetNavMode>("SetNavMode");
  factory.registerNodeType<WasRainingAtStart>("WasRainingAtStart");
  factory.registerNodeType<RecordUndockStart>("RecordUndockStart");
  factory.registerNodeType<CalibrateHeadingFromUndock>("CalibrateHeadingFromUndock");
  factory.registerNodeType<DockRobot>("DockRobot");
  factory.registerNodeType<UndockRobot>("UndockRobot");
  factory.registerNodeType<RecordResumeUndockFailure>("RecordResumeUndockFailure");
  factory.registerNodeType<ResetEmergency>("ResetEmergency");

  // Multi-area coverage node
  factory.registerNodeType<GetNextUnmowedArea>("GetNextUnmowedArea");

  // Cell-based coverage nodes
  factory.registerNodeType<GetNextStrip>("GetNextStrip");
  factory.registerNodeType<FollowStrip>("FollowStrip");
  factory.registerNodeType<TransitToStrip>("TransitToStrip");

  // Area recording node
  factory.registerNodeType<RecordArea>("RecordArea");
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

#### Multi-Area Coverage

**Concept:** Instead of a single coverage path, users can define multiple mowing areas (polygons) that are mowed sequentially in a single autonomous session.

**Workflow:**
1. User records multiple areas using `COMMAND_RECORD_AREA` (3) — each area is a polygon boundary saved to map_server
2. User initiates mowing with `COMMAND_START` (1)
3. Behavior tree calls `GetNextUnmowedArea()` to fetch first area
4. Coverage planner generates strip path for that area
5. Robot mows current area strip-by-strip via GetNextStrip/TransitToStrip/FollowStrip
6. Once area is complete, BT calls `GetNextUnmowedArea()` again to fetch next area
7. Process repeats until all areas are mowed

**Progress Tracking:**
- Each area maintains its own coverage grid (`mow_progress` layer) persisted across sessions
- High-level status includes `current_area` and `areas_remaining` fields
- GUI shows progress per area and overall session progress
- If session interrupted, robot resumes from last completed area on restart

**Map Server Integration:**
- `map_server_node` maintains list of mowing areas and their coverage state
- Service: `/map_server_node/get_next_unmowed_area` — returns next area and its current coverage grid
- Service: `/map_server_node/mark_area_complete` — marks area as finished
- Enables robust recovery from power loss or manual pause

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

#### Behavior Tree Visualization

**BT State Logging:**
The behavior_tree_node publishes active node information via `/behavior_tree_log` topic:
- **Message type:** `BehaviorTreeLog` (custom msg in mowgli_interfaces)
- **Contents:** Active node name, node type, tick timestamp, execution status
- **Frequency:** Every 10 Hz tick, only when BT status changes
- **Use:** GUI displays active BT node in real-time diagnostics page

**GUI Integration:**
- Diagnostics page shows current BT node path and state
- Helps identify where robot is stuck or failing (e.g., "FollowCoveragePath" stuck on obstacle)
- Updates in real-time as BT executes

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

   FusionCore (single UKF, 50 Hz):
     ├─ Fuses /wheel_odom + /imu/data + /gps/fix (NavSatFix directly)
     ├─ Output: /fusion/odom (sensor fusion result, odom → base_footprint)
     └─→ /tf: odom → base_footprint

   SLAM (slam_toolbox, 20 Hz):
     ├─ /scan + /fusion/odom
     ├─ Output: /map (occupancy grid)
     └─→ /tf: map → odom (scan matching + loop closure, TF authority)

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
       ├─ /fusion/odom (robot pose from sensor fusion)
       ├─ /map (occupancy grid from SLAM)
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
  │ [published by SLAM Toolbox @ 20 Hz]
  │
  odom (local dead-reckoning origin)
  │ [published by FusionCore @ 50 Hz]
  │
  base_footprint (on ground, robot frame for Nav2/FusionCore)
  │ [published by FusionCore from URDF]
  │
  ├── base_link (robot body frame, wheel axle height per OpenMower convention)
  │   └── [published by robot_state_publisher from URDF]
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
| map | SLAM Toolbox | 20 Hz | Global origin (scan matching + loop closure) |
| odom | FusionCore | 50 Hz | Local dead-reckoning origin (wheel odometry reference) |
| base_footprint | FusionCore | 50 Hz | On ground, robot frame for Nav2/controllers (per REP-105) |
| base_link | robot_state_publisher | Static | Rear wheel axle (OpenMower convention) |
| imu_link | robot_state_publisher | Static | IMU sensor frame |
| laser_link | robot_state_publisher | Static | LiDAR origin |
| gps_link | robot_state_publisher | Static | GPS antenna location |
| [Gazebo sensor frames] | (simulation only) | Static | Gazebo model sensor origins |

**Frame Conventions:**

- `map` – Global frame, typically z-up, x-east, y-north (REP-103). Set by SLAM Toolbox via scan matching.
- `odom` – Local odometry frame, z-up. Represents wheel odometry reference; drift corrected by SLAM.
- `base_footprint` – Ground contact point, robot frame for Nav2. Published by FusionCore.
- `base_link` – Robot body frame at rear wheel axle height (OpenMower convention). Static offset from base_footprint.

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
| `/gps/absolute_pose` | geometry_msgs/PoseWithCovarianceStamped | navsat_to_absolute_pose_node | 10–20 Hz | RTK position converted to local ENU (for visualization) |
| `/fusion/odom` | nav_msgs/Odometry | FusionCore | 50 Hz | Fused estimate (odom→base_footprint) with GPS+IMU+wheels |
| `/localization/status` | diagnostic_msgs/DiagnosticStatus | localization_monitor_node | 2 Hz | EKF health, degradation mode |
| `/coverage_planner_node/coverage_path` | nav_msgs/Path | coverage_planner_node | Once per plan | B-RV coverage path with poses |
| `/coverage_planner_node/coverage_outline` | nav_msgs/Path | coverage_planner_node | Once per plan | Headland outline for visualization |
| `/path` | nav_msgs/Path | planner_server (Nav2) | 1 Hz | Global path from Nav2 planner |
| `/cmd_vel` | geometry_msgs/Twist | twist_mux | 10–50 Hz | Final velocity command (to hardware/Gazebo) |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | diagnostics_node (mowgli_monitoring) | 1 Hz | System health aggregation |
| `/behavior_tree_log` | mowgli_interfaces/BehaviorTreeLog | behavior_tree_node | 10 Hz | Active BT node, node type, execution status (for GUI diagnostics visualization) |
| `/high_level_status` | mowgli_interfaces/HighLevelStatus | behavior_tree_node | 10 Hz | Current high-level mode (IDLE=1, AUTONOMOUS=2, RECORDING=3, MANUAL_MOWING=4) with coverage progress per area |

**Subscribers (Sinks):**

| Topic | Subscriber | Purpose |
|-------|-----------|---------|
| `/cmd_vel` | hardware_bridge_node / Gazebo | Motor/wheel commands |
| `/scan` | slam_toolbox, nav2_local_costmap, diagnostics_node | SLAM, obstacle detection, monitoring |
| `/imu/data_raw` | diagnostics_node | Monitor IMU freshness |
| `/fusion/odom` | slam_toolbox, nav2 controller, diagnostics_node | SLAM input, localized pose for control |
| `/map` | nav2 planner, behavior_tree (goal comparison), diagnostics_node | Global navigation reference |
| `/gps/rtk_fix` | FusionCore, navsat_to_absolute_pose_node, diagnostics_node | Sensor fusion, convert fix to local frame, monitor GPS |
| `/status` | behavior_tree_node, localization_monitor_node, diagnostics_node | Health checks, sensor freshness |
| `/emergency` | behavior_tree_node, diagnostics_node | Emergency monitoring |
| `/power` | behavior_tree_node, diagnostics_node | Battery level monitoring |
| `/wheel_odom` | diagnostics_node | Monitor odometry freshness |
| `/coverage_planner_node/coverage_path` | FollowCoveragePath BT node, diagnostics_node (optional) | Coverage execution |

**Services (Request-Response):**

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/mower_control` | MowerControl | hardware_bridge_node | behavior_tree_node | Enable/disable blade motor |
| `/emergency_stop` | EmergencyStop | hardware_bridge_node | behavior_tree_node, ResetEmergency BT node | Assert/release latched emergency |
| `/high_level_control` | HighLevelControl | behavior_tree_node | GUI | Mode commands (START=1, HOME=2, RECORD_AREA=3, S2=4, RECORD_FINISH=5, RECORD_CANCEL=6, MANUAL_MOW=7) |
| `/map_server_node/add_area` | AddMowingArea | map_server_node | RecordArea BT node | Save recorded mowing area polygon |
| `/map_server_node/get_next_unmowed_area` | GetNextUnmowedArea | map_server_node | behavior_tree_node (GetNextUnmowedArea BT node) | Fetch next unmowed area polygon and coverage grid |
| `/map_server_node/mark_area_complete` | MarkAreaComplete | map_server_node | behavior_tree_node | Mark area as mowing complete, persist coverage state |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | behavior_tree_node | Send goal to Nav2 |

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

2. **ROS2 Kilted + Gazebo Harmonic:** Modern robotics stack with first-class simulation support and lifecycle management.

3. **Decoupled Communication:** ROS2 pub/sub (topics), services, and actions isolate packages. Easy to substitute, test, or extend components independently.

4. **Robust Serial Protocol (COBS + CRC-16):** Enables reliable bidirectional communication between Raspberry Pi and STM32 firmware over noisy USB at 115200 baud.

5. **Single-Tier UKF Localization (FusionCore):**
   - Fuses wheel odometry + IMU + GPS in single UKF (50 Hz)
   - Outputs `/fusion/odom` (odom→base_footprint) with GPS correction
   - SLAM Toolbox provides map→odom TF authority via scan matching (20 Hz)
   - Graceful degradation: operates without GPS in GNSS-denied areas (wheel+IMU only)

6. **Coverage Path Following:** FTCController (Follow-the-Carrot) is the active controller for FollowCoveragePath, achieving <10mm lateral accuracy with 3-axis PID control and in-place rotation at swath turns. RPP remains active for FollowPath (transit and docking).

7. **B-RV Coverage Planning:** Grid-based boustrophedon sweep with MBB optimal direction (rotating calipers on convex hull), obstacle-aware cell skipping, and Voronoi roadmap transit (Boost.Polygon Voronoi + Dijkstra shortest path) for inter-region connections. No Fields2Cover dependency.

8. **Reactive Behavior Trees (BehaviorTree.CPP v4):** 10 Hz non-preemptive tree execution with priority-based fallback selection:
   - Emergency guard: highest priority, interrupts all activity
   - Multi-mode state machine: IDLE → UNDOCKING → MOWING (with recovery) → DOCKING
   - Multi-area coverage: iterates through multiple mowing areas sequentially, each with independent coverage progress tracking
   - Composable async action nodes (NavigateToPose, GetNextUnmowedArea, PlanCoveragePath, FollowCoveragePath)

9. **Priority-Based Command Routing:** twist_mux mediates three command sources (emergency > teleoperation > navigation) before forwarding to hardware bridge.

10. **Comprehensive Health Monitoring:** Diagnostics aggregator tracks 8 subsystems (hardware bridge, emergency, battery, IMU, LiDAR, GPS, odometry, motors) and FusionCore UKF health (rate, variance, z-drift) with multi-level status (OK, WARN, ERROR, STALE) for autonomous decision-making. Real-time BT visualization shows active node path and execution state in GUI diagnostics page.

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
