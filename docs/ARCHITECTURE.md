# Mowgli ROS2 Architecture

Comprehensive technical documentation of the Mowgli ROS2 system design, including package organization, data flow, communication protocols, and integration points.

## System Overview

Mowgli ROS2 is organized as a **six-package ecosystem** with clear separation of concerns and layered dependencies:

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
│  │                  │  │  (FTC            │  │                      │   │
│  │  10 Hz reactive  │  │   Controller)    │  │  Multiple nodes:     │   │
│  │  control         │  │                  │  │  - Wheel odometry    │   │
│  │                  │  │  Nav2 local plan │  │  - GPS converter     │   │
│  │                  │  │  10 Hz           │  │  - Health monitor    │   │
│  └──────────────────┘  └──────────────────┘  └──────────────────────┘   │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                         Config & Launch Layer                            │
│                      (mowgli_bringup: URDF, params)                      │
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
    └──→ mowgli_behavior
            └──→ mowgli_bringup

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

- **WheelTick.msg** – Encoder pulse counts (internal)
  ```
  builtin_interfaces/Time stamp
  int32 fl_tick                   # Front-left wheel (unused on Mowgli)
  int32 fr_tick                   # Front-right wheel (unused on Mowgli)
  int32 rl_tick                   # Rear-left wheel (active)
  int32 rr_tick                   # Rear-right wheel (active)
  ```

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
- **DockingSequence.action** – Custom docking behavior (if implemented)

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
ROS2 network: /hardware_bridge/status, /hardware_bridge/emergency, /hardware_bridge/power
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
    serial_port: "/dev/ttyUSB0"     # Device path (USB serial)
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
  - /hardware_bridge/status (WheelTick in Status msg)
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
- `/hardware_bridge/status` (for wheel tick freshness)
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

**Purpose:** Custom Nav2 controller plugin (Follow-The-Carrot algorithm).

**Location:** `src/mowgli_nav2_plugins/`

**Plugin Registration:** `ftc_controller_plugin.xml`

```xml
<library path="libftc_controller_plugin">
  <class type="mowgli_nav2_plugins::FTCController"
         base_class_type="nav2_core::Controller">
    <description>Follow-The-Carrot local planner for lawn mowing robots</description>
  </class>
</library>
```

#### FTCController: State Machine & Algorithm

**5-State FSM:**

```
          Initial State
               │
               ▼
    ┌─────────────────────┐
    │   PRE_ROTATE        │
    │ (orientation to path)
    └─────────────────────┘
            │
     Goal within lookahead?
     │                      │
    YES                     NO
     │                      │
     ▼                      ▼
┌──────────────┐  ┌──────────────────┐
│   FORWARD    │  │   APPROACH       │
│              │  │ (reduce speed)   │
└──────────────┘  └──────────────────┘
     │                      │
     └──────────────────────┘
            │
      Goal reached?
            │
            ▼
     ┌────────────┐
     │   STOP     │
     │ (brake)    │
     └────────────┘

Oscillation Recovery:
  Any state + oscillation detected → hold position, then retry
```

**Algorithm: PID-based Velocity Control**

Inputs:
- Global path (array of PoseStamped)
- Robot pose (from TF: odom → base_link)
- Costmap (for obstacle checking)

Process:

1. **Carrot Point Selection:**
   - Find nearest point on path
   - Lookahead distance ahead: `lookahead_dist` (default 0.6 m)
   - SLERP interpolation for smooth paths

2. **Error Calculation:**
   ```
   cross_track_error = perpendicular distance to path
   heading_error = angle to carrot point
   velocity_error = desired_vel - current_vel
   ```

3. **PID Control (3-channel independent):**
   ```
   linear_pid:
     u_linear = Kp * cross_track_error
              + Ki * integral(cross_track_error)
              + Kd * d_cross_track_error/dt

   angular_pid:
     u_angular = Kp * heading_error
               + Ki * integral(heading_error)
               + Kd * d_heading_error/dt

   acceleration_pid:
     u_accel = Kp * velocity_error
             + Ki * integral(velocity_error)
             + Kd * d_velocity_error/dt
   ```

4. **Output Saturation:**
   ```
   cmd_vel.linear.x = clamp(desired_linear_vel + u_linear,
                             -max_linear_vel, max_linear_vel)
   cmd_vel.angular.z = clamp(u_angular, -max_angular_vel, max_angular_vel)
   ```

5. **Collision Avoidance:**
   - Check costmap within robot footprint
   - If obstacle detected, trigger oscillation recovery
   - Or return FAILURE to planner

**State Transitions:**

- **PRE_ROTATE → FORWARD:** Goal is within lookahead distance AND robot is roughly aligned
- **FORWARD → APPROACH:** Robot within 1.0 m of goal
- **APPROACH → STOP:** Robot within `xy_goal_tolerance` (0.25 m) and `yaw_goal_tolerance` (0.25 rad)
- **STOP → FORWARD/PRE_ROTATE:** If another goal provided

**Oscillation Recovery:**

Detects when the robot is trapped (oscillating back-and-forth or stuck):

```
If |velocity| < min_velocity_threshold for > oscillation_recovery_min_duration:
    Hold position (zero velocity for 2 seconds)
    Then retry navigation
```

**Parameters (in nav2_params.yaml):**

```yaml
FollowPath:
  plugin: "mowgli_nav2_plugins::FTCController"

  # Motion targets
  desired_linear_vel: 0.3           # m/s – cruising speed for mowing
  max_linear_vel: 0.5               # m/s – safety limit
  max_angular_vel: 1.0              # rad/s

  # Lookahead (carrot point distance)
  lookahead_dist: 0.6               # m
  lookahead_max_angle: 0.5          # rad max deviation from heading

  # PID gains (linear)
  linear_p_gain: 2.0
  linear_i_gain: 0.5
  linear_d_gain: 0.1
  linear_max_integral: 0.5

  # PID gains (angular)
  angular_p_gain: 2.0
  angular_i_gain: 0.5
  angular_d_gain: 0.1
  angular_max_integral: 0.5

  # Oscillation recovery
  use_oscillation_recovery: true
  oscillation_recovery_min_duration: 5.0    # seconds
  oscillation_recovery_max_cycles: 3        # max retry attempts
```

#### oscillation_detector.cpp

Helper class that tracks velocity history to detect stuck-robot conditions.

```cpp
class OscillationDetector {
  void setBufferLength(int length);      // Set history window
  void update(double linear_vel, double angular_vel);
  bool isOscillating() const;            // Returns true if stuck
};
```

Internal buffer (ring) stores the last N velocity commands and checks for stagnation.

---

### 6. mowgli_behavior

**Purpose:** High-level reactive control using BehaviorTree.CPP v4.

**Location:** `src/mowgli_behavior/`

**Architecture:**

```
BehaviorTreeNode (main ROS2 node)
    │
    ├── BTContext (shared state)
    │   ├── node reference (for publishing, services)
    │   ├── latest_status (from hardware bridge)
    │   ├── latest_emergency (from hardware bridge)
    │   ├── latest_power (from hardware bridge)
    │   └── [other sensory state]
    │
    └── BehaviorTree instance (XML-loaded)
        │
        └── Tree root: ReactiveSequence
            │
            ├── Child 1: IsEmergency (condition)
            ├── Child 2: IsBatteryLow (condition)
            ├── Child 3: ModeSelector (selector)
            │   ├── IsInIdleMode? → IdleMode (sequence)
            │   ├── IsInMowingMode? → MowingMode (sequence)
            │   │   ├── IsMowerHealthy? (condition)
            │   │   ├── NavigateToPose (action to Nav2)
            │   │   └── SetMowerEnabled (service call)
            │   ├── IsInDockingMode? → DockingMode (sequence)
            │   │   └── NavigateToDockingStation (action)
            │   └── IsInRecordingMode? → RecordingMode (sequence)
            │
            └── Child 4: UpdateHighLevelState (action)
                └── Sends current mode to STM32 firmware

Update frequency: 10 Hz (controlled by BehaviorTreeNode timer)
```

#### Condition Nodes (in condition_nodes.cpp)

```cpp
class IsEmergency : public BT::ConditionNode
// Returns SUCCESS if emergency latched, FAILURE otherwise

class IsBatteryLow : public BT::ConditionNode
// Checks v_battery < LOW_VOLTAGE_THRESHOLD (e.g., 18.0 V)

class IsGpsAvailable : public BT::ConditionNode
// Checks RTK fix type (must be RTK Fixed, not Float or No-fix)

class IsLocalizationHealthy : public BT::ConditionNode
// Queries localization_monitor_node status (EKF variance)

class IsMowerHealthy : public BT::ConditionNode
// Checks hardware_bridge status: sound available, UI available, etc.

class IsRaining : public BT::ConditionNode
// Checks rain sensor bit from Status message
```

#### Action Nodes (in action_nodes.cpp)

```cpp
class NavigateToPose : public BT::AsyncActionNode
// Sends goal to Nav2 action server (/navigate_to_pose)
// Port In: goal_x, goal_y, goal_theta
// Returns RUNNING while in progress, SUCCESS on completion, FAILURE on abort

class SetMowerEnabled : public BT::ActionNode
// Calls /hardware_bridge/mower_control service
// Port In: enabled (bool)
// Returns SUCCESS on service success, FAILURE otherwise

class UpdateHighLevelState : public BT::ActionNode
// Sends current_mode and gps_quality to STM32 via /hardware_bridge/high_level_state
// Informs firmware of high-level decision (idle, mowing, docking, recording)

class SendEmergencyStop : public BT::ActionNode
// Calls /hardware_bridge/emergency_stop with emergency=true
// For autonomous safety triggers

class SkipRecovery : public BT::ActionNode
// Calls /hardware_bridge/emergency_stop with emergency=false
// Releases latched emergency stop
```

#### Tree Control (BehaviorTreeNode)

**Subscriptions:**
- `/hardware_bridge/status` – Mower state, sensor availability
- `/hardware_bridge/emergency` – Emergency status
- `/hardware_bridge/power` – Battery voltage, charging

**Services Offered:**
- `/mowgli_behavior/set_mode` – Switch between Idle, Mowing, Docking, Recording modes (high-level)

**Services Called:**
- `/hardware_bridge/mower_control` – Enable/disable blade
- `/hardware_bridge/emergency_stop` – Safety control
- `/navigate_to_pose` (Nav2 action) – Send navigation goals

**Tree Updates:**
- 10 Hz tick() cycle
- ReactiveSequence: on any child returning FAILURE, entire sequence restarts
- Emergency guard: IsEmergency always evaluated first, aborts all other branches on true

#### Node Registration (register_nodes.cpp)

All node classes registered with BehaviorTree factory:

```cpp
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<IsEmergency>("IsEmergency");
  factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<SetMowerEnabled>("SetMowerEnabled");
  // ... etc
}
```

Allows tree XML to reference nodes by name.

---

## Complete Data Flow Diagram

### Scenario: Autonomous Mowing Run

```
1. User sends goal via GUI (map coordinates)
   └─→ /navigate_to_pose goal (nav2_msgs/NavigateToPose action)

2. BehaviorTree processes:
   └─→ NavigateToPose action → contacts Nav2 action server

3. Nav2 pipeline:
   Planner (SmacPlanner2D):
     Global map (/map) + start (odom) + goal
     └─→ Global path (nav_msgs/Path)

   Controller (FTCController):
     Global path + robot pose (from EKF)
     └─→ cmd_vel (geometry_msgs/Twist)

   Costmap managers:
     /scan (LiDAR) + odometry
     └─→ costmap_2d (for obstacles)

4. cmd_vel routing (twist_mux):
   Priority: emergency > teleop > navigation
   └─→ /cmd_vel (to hardware_bridge)

5. Hardware bridge translates:
   Twist msg → LlCmdVel packet (COBS + CRC)
   └─→ USB serial → STM32

6. STM32 firmware:
   LL_CMD_VEL packet
   └─→ Motor ESC commands (PWM)
   └─→ Wheel encoders measure actual motion

7. Feedback loop (50 Hz):
   STM32 → LL_STATUS packet (encoder ticks, IMU, sensors)
   └─→ USB serial → hardware_bridge
   └─→ /wheel_odom (wheel_odometry_node)
   └─→ /imu/data (filtered)
   └─→ ekf_odom (robot_localization): fuses odometry + IMU
       └─→ /odometry/filtered_odom

8. GPS fusion (20 Hz, if RTK fix available):
   /gps/rtk_fix → gps_pose_converter → /gps/pose
   └─→ ekf_map (robot_localization): fuses filtered odometry + GPS
       └─→ /odometry/filtered_map
       └─→ /tf: map → odom → base_link

9. SLAM (if running):
   /scan + /odometry/filtered_odom
   └─→ slam_toolbox
       └─→ /map (occupancy grid)
       └─→ /tf: map → odom correction

10. Navigation feedback:
    Robot's actual pose (from EKF) vs. goal
    └─→ Controller recomputes path, adjusts cmd_vel

11. Safety monitoring (BehaviorTree, 10 Hz):
    - Check emergency latch (IsEmergency)
    - Check battery voltage (IsBatteryLow)
    - Check localization health (IsLocalizationHealthy)
    - If any unsafe condition → halt (zero cmd_vel) + log warning

12. Completion:
    Robot reaches goal (xy_goal_tolerance + yaw_goal_tolerance)
    └─→ FTCController returns SUCCESS
    └─→ Nav2 action completes
    └─→ NavigateToPose BT node returns SUCCESS
    └─→ BehaviorTree transitions to next mode (e.g., mowing pattern)
```

---

## TF Tree Reference

**Standard ROS2 conventions (REP-103):**

```
map
  │ (map → odom: published by ekf_map in robot_localization)
  │
  odom
  │ (odom → base_link: published by ekf_odom in robot_localization)
  │
  base_link (robot centre at wheel axle height)
  │ (base_link → sensors: published by robot_state_publisher)
  │
  ├── imu_link (fixed)
  │
  ├── laser_link (fixed, LiDAR origin)
  │
  ├── gps_link (fixed, GPS antenna)
  │
  ├── left_wheel_link (rotating, odometry input)
  │
  └── right_wheel_link (rotating, odometry input)
```

**Frame Conventions:**

- `map` – Global (SLAM-corrected or GPS-based)
- `odom` – Local, drifting dead-reckoning frame
- `base_link` – Robot body frame (geometric centre at wheel height)
- `base_footprint` – Robot at ground level (rarely used in Nav2)

**Update Rates:**

- `map → odom`: 20 Hz (ekf_map, GPS-corrected)
- `odom → base_link`: 50 Hz (ekf_odom, local estimation)
- Static TFs (base_link → sensors): Once at startup (robot_state_publisher)

---

## Topic Map

**Publishers (Sources):**

| Topic | Type | Publisher | Rate | Purpose |
|-------|------|-----------|------|---------|
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox | 1 Hz | Occupancy map from SLAM |
| `/scan` | sensor_msgs/LaserScan | [hardware] (LiDAR driver) | 5–20 Hz | LiDAR range data |
| `/imu/data` | sensor_msgs/Imu | [filtered by imu_filter_madgwick or direct] | 50 Hz | Filtered IMU (orientation computed) |
| `/hardware_bridge/status` | mowgli_interfaces/Status | hardware_bridge_node | 100 Hz | Mower state, sensors |
| `/hardware_bridge/emergency` | mowgli_interfaces/Emergency | hardware_bridge_node | 100 Hz | Emergency stop status |
| `/hardware_bridge/power` | mowgli_interfaces/Power | hardware_bridge_node | 100 Hz | Battery, charging info |
| `/hardware_bridge/imu/data_raw` | sensor_msgs/Imu | hardware_bridge_node | 100 Hz | Raw accelerometer, gyroscope |
| `/wheel_odom` | nav_msgs/Odometry | wheel_odometry_node | 50 Hz | Dead-reckoning from wheels |
| `/gps/pose` | geometry_msgs/PoseWithCovarianceStamped | gps_pose_converter_node | 10–20 Hz | RTK position in local ENU |
| `/odometry/filtered_odom` | nav_msgs/Odometry | ekf_odom (robot_localization) | 50 Hz | Fused odometry + IMU |
| `/odometry/filtered_map` | nav_msgs/Odometry | ekf_map (robot_localization) | 20 Hz | Fused odometry + GPS (global) |
| `/localization/status` | diagnostic_msgs/DiagnosticStatus | localization_monitor_node | 2 Hz | EKF health and degradation mode |
| `/path` | nav_msgs/Path | planner_server (Nav2) | 1 Hz | Global path from planner |
| `/cmd_vel` | geometry_msgs/Twist | twist_mux | 10–50 Hz | Final motor velocity command |

**Subscribers (Sinks):**

| Topic | Subscriber | Purpose |
|-------|-----------|---------|
| `/hardware_bridge/cmd_vel` | hardware_bridge_node | Motor velocity input |
| `/scan` | slam_toolbox, nav2_costmap | LiDAR for mapping and obstacles |
| `/odometry/filtered_odom` | slam_toolbox | Odometry for SLAM |
| `/odometry/filtered_map` | nav2_behavior_tree, behavior_tree_navigator | Localized pose for navigation |
| `/cmd_vel_nav` | twist_mux | Nav2 velocity output |
| `/cmd_vel_teleop` | twist_mux | Teleoperation input |
| `/cmd_vel_emergency` | twist_mux | Emergency velocity (safety system) |
| `/emergency_stop` | twist_mux | Emergency stop lock |
| `/hardware_bridge/status` | behavior_tree_node, localization_monitor | Hardware state |
| `/hardware_bridge/emergency` | behavior_tree_node | Emergency status |
| `/hardware_bridge/power` | behavior_tree_node | Battery monitoring |

**Services (Request-Response):**

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/hardware_bridge/mower_control` | MowerControl | hardware_bridge_node | behavior_tree_node, external tools | Enable/disable blade |
| `/hardware_bridge/emergency_stop` | EmergencyStop | hardware_bridge_node | behavior_tree_node, safety system | Emergency stop control |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2 | behavior_tree_node | Send navigation goal |
| `/robot_localization/set_pose` | robot_localization/SetPose | ekf_odom | startup/initialization tools | Initialize odometry pose |

**Actions:**

| Action | Type | Server | Client | Purpose |
|--------|------|--------|--------|---------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | mowgli_behavior | Non-blocking navigation goal |

---

## Summary: Architectural Principles

1. **Layered Abstraction:** Each package handles one responsibility (hardware, localization, navigation, behavior).
2. **Decoupled Communication:** ROS2 pub/sub + services isolate packages. Easy to swap implementations.
3. **Robust Serial Protocol:** COBS + CRC-16 enables reliable STM32 ↔ Pi communication over noisy USB.
4. **Dual EKF Localization:** Separates fast local estimation (odometry + IMU) from slower global correction (GPS).
5. **Reactive Behavior Trees:** Allows preemptable, composable high-level control without state bloat.
6. **Outdoor-Optimized SLAM:** Conservative loop closure and map updates suit changing outdoor environments.
7. **Priority-Based Command Routing:** Emergency commands override teleoperation, which overrides autonomous navigation.
8. **Modular Nav2 Plugins:** Custom FTC controller maintains proven mowing-specific tuning while using Nav2's infrastructure.

---

## Next Steps

- See [CONFIGURATION.md](CONFIGURATION.md) for parameter tuning details.
- See [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md) for STM32 integration.
- See [SIMULATION.md](SIMULATION.md) for testing in Gazebo Ignition.
