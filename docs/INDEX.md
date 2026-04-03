# MowgliNext Documentation Index

Comprehensive documentation for MowgliNext — a complete rewrite of OpenMower in ROS2 Jazzy with modern navigation, SLAM, behavior trees, and autonomous coverage planning.

## Quick Navigation

### For New Users

Start here to get up and running:

1. **[../README.md](../README.md)** – Project overview, features, and quick start
   - What is Mowgli ROS2?
   - Hardware requirements
   - Building from source
   - Quick start (Docker, hardware, simulation)

2. **[SIMULATION.md](SIMULATION.md)** – Running in Gazebo Harmonic
   - Virtual testing without hardware
   - Common workflows (navigation, SLAM, behavior trees)
   - Troubleshooting simulation issues

3. **[CONFIGURATION.md](CONFIGURATION.md)** – Parameter tuning reference
   - All configuration files explained
   - How to adjust parameters for your environment
   - Common configurations (high-performance, conservative, etc.)

### For System Architects

Understand the complete system design:

1. **[ARCHITECTURE.md](ARCHITECTURE.md)** – Detailed technical architecture
   - System overview and layered design
   - Package descriptions and responsibilities
   - Data flow diagrams and topic maps
   - Wire protocol (COBS + CRC-16)
   - Transform tree (TF) reference

2. **[FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md)** – STM32 firmware integration
   - Migrating from ROS1 rosserial to COBS protocol
   - Packet structure definitions
   - Implementation examples and testing

3. **[CONFIGURATION.md](CONFIGURATION.md)** – Deep parameter reference
   - EKF tuning for localization
   - Nav2 controller and planner configuration
   - SLAM parameters for outdoor operation

### For Developers

Build new features and extend the system:

1. **[ARCHITECTURE.md](ARCHITECTURE.md)** – Package organization
   - mowgli_interfaces (messages and services)
   - mowgli_hardware (serial bridge)
   - mowgli_description (URDF/xacro model)
   - mowgli_localization (odometry and GPS fusion)
   - mowgli_nav2_plugins (FTC and RotationShim controllers)
   - mowgli_coverage_planner (autonomous mowing patterns)
   - mowgli_map (area management and obstacle tracking)
   - mowgli_monitoring (diagnostic and telemetry)
   - mowgli_behavior (behavior trees)
   - mowgli_simulation (Gazebo Harmonic worlds)
   - mowgli_bringup (configuration and launch)
   - opennav_coverage (third-party Nav2 coverage server)

2. **[../README.md](../README.md)** – Development workflow
   - Contributing guidelines
   - Test coverage requirements
   - Building and testing

3. **Development Setup** – Local environment configuration
   - Installing ROS2 Jazzy
   - Building workspace from source
   - Running tests and linters

### For DevOps / Deployment

Run the system in production:

1. **[../README.md](../README.md)** – Hardware launch
   - Serial port configuration
   - Hardware bringup

2. **[CONFIGURATION.md](CONFIGURATION.md)** – Tuning for your environment
   - Hardware-specific parameters
   - Localization calibration

3. **[SIMULATION.md](SIMULATION.md)** – Testing before deployment
   - Validation workflows
   - Docker deployment

---

## Documentation by Topic

### Getting Started

| Topic | Document | Key Section |
|-------|----------|-------------|
| What is Mowgli? | [README.md](../README.md) | Overview |
| Build instructions | [README.md](../README.md) | Quick Start → Building from Source |
| First launch | [SIMULATION.md](SIMULATION.md) | Quick Start |
| Quick start checklist | [README.md](../README.md) | Quick Start |

### System Design

| Topic | Document | Key Section |
|-------|----------|-------------|
| Architecture overview | [ARCHITECTURE.md](ARCHITECTURE.md) | System Overview |
| Package dependencies | [ARCHITECTURE.md](ARCHITECTURE.md) | Package Dependency Graph |
| Data flow (end-to-end) | [ARCHITECTURE.md](ARCHITECTURE.md) | Complete Data Flow Diagram |
| Wire protocol | [ARCHITECTURE.md](ARCHITECTURE.md) | Wire Protocol: COBS + CRC-16 |
| Transform tree (TF) | [ARCHITECTURE.md](ARCHITECTURE.md) | TF Tree Reference |
| Topic map | [ARCHITECTURE.md](ARCHITECTURE.md) | Topic Map |

### Hardware & Firmware

| Topic | Document | Key Section |
|-------|----------|-------------|
| Hardware requirements | [README.md](../README.md) | Hardware Requirements |
| Serial communication | [ARCHITECTURE.md](ARCHITECTURE.md) | mowgli_hardware |
| Firmware migration | [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md) | Full guide |
| Packet reference | [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md) | Packet Structure Reference |
| Testing firmware | [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md) | Testing & Validation |

### Localization & Navigation

| Topic | Document | Key Section |
|-------|----------|-------------|
| Localization overview | [ARCHITECTURE.md](ARCHITECTURE.md) | mowgli_localization |
| Dual EKF tuning | [CONFIGURATION.md](CONFIGURATION.md) | localization.yaml |
| Nav2 configuration | [CONFIGURATION.md](CONFIGURATION.md) | nav2_params.yaml |
| FTC controller | [ARCHITECTURE.md](ARCHITECTURE.md) | mowgli_nav2_plugins |
| SLAM parameters | [CONFIGURATION.md](CONFIGURATION.md) | slam_toolbox.yaml |
| Twist multiplexing | [CONFIGURATION.md](CONFIGURATION.md) | twist_mux.yaml |

### Testing & Simulation

| Topic | Document | Key Section |
|-------|----------|-------------|
| Simulation overview | [SIMULATION.md](SIMULATION.md) | Overview |
| Launch simulation | [SIMULATION.md](SIMULATION.md) | Quick Start |
| Common workflows | [SIMULATION.md](SIMULATION.md) | Common Workflows |
| Gazebo Harmonic controls | [SIMULATION.md](SIMULATION.md) | Gazebo Controls |
| Foxglove visualization | [SIMULATION.md](SIMULATION.md) | Foxglove Monitoring |
| Troubleshooting | [SIMULATION.md](SIMULATION.md) | Troubleshooting |

### Configuration & Tuning

| Topic | Document | Key Section |
|-------|----------|-------------|
| Serial port setup | [CONFIGURATION.md](CONFIGURATION.md) | hardware_bridge.yaml |
| EKF tuning | [CONFIGURATION.md](CONFIGURATION.md) | localization.yaml |
| Motion control tuning | [CONFIGURATION.md](CONFIGURATION.md) | nav2_params.yaml → FTCController |
| Coverage planning | [CONFIGURATION.md](CONFIGURATION.md) | coverage_planner.yaml |
| Foxglove monitoring | [CONFIGURATION.md](CONFIGURATION.md) | foxglove_bridge.yaml |
| Outdoor SLAM | [CONFIGURATION.md](CONFIGURATION.md) | slam_toolbox.yaml |
| Behavior tree control | [ARCHITECTURE.md](ARCHITECTURE.md) | mowgli_behavior |
| Parameter tuning workflow | [CONFIGURATION.md](CONFIGURATION.md) | Parameter Tuning Workflow |

### Behavior Trees

| Topic | Document | Key Section |
|-------|----------|-------------|
| Behavior tree overview | [ARCHITECTURE.md](ARCHITECTURE.md) | mowgli_behavior |
| Tree structure | [ARCHITECTURE.md](ARCHITECTURE.md) | Tree Control (BehaviorTreeNode) |
| Condition nodes | [ARCHITECTURE.md](ARCHITECTURE.md) | Condition Nodes |
| Action nodes | [ARCHITECTURE.md](ARCHITECTURE.md) | Action Nodes |
| Node registration | [ARCHITECTURE.md](ARCHITECTURE.md) | Node Registration |

---

## Monorepo Structure

```
mowglinext/
├── README.md                          # Project overview & quick start
├── docs/                              # All documentation (single source of truth)
│   ├── INDEX.md                       # This file
│   ├── ARCHITECTURE.md                # Technical architecture (detailed)
│   ├── CONFIGURATION.md               # Parameter reference & tuning
│   ├── FIRMWARE_MIGRATION.md          # STM32 integration guide
│   └── SIMULATION.md                  # Gazebo Harmonic simulation guide
├── ros2/                              # ROS2 stack
│   └── src/
│       ├── mowgli_interfaces/         # Message definitions (12 msgs, 9 srvs)
│       ├── mowgli_hardware/           # Serial bridge to STM32
│       ├── mowgli_description/        # URDF/xacro model & meshes
│       ├── mowgli_localization/       # Odometry & GPS fusion (dual EKF)
│       ├── mowgli_nav2_plugins/       # FTC & RotationShim controllers
│       ├── mowgli_coverage_planner/   # Autonomous mowing patterns
│       ├── mowgli_map/               # Area management & obstacle tracking
│       ├── mowgli_monitoring/         # Diagnostics and telemetry
│       ├── mowgli_behavior/           # Behavior tree nodes
│       ├── mowgli_simulation/         # Gazebo Harmonic worlds & SDF model
│       ├── mowgli_bringup/            # Launch files & configuration
│       └── opennav_coverage/          # Third-party Nav2 coverage server
├── docker/                            # Docker Compose deployment & config
├── sensors/                           # Dockerized sensor drivers
│   ├── gps/                           # u-blox ZED-F9P (RTK GPS)
│   └── lidar/                         # LDRobot LD19 (2D LiDAR)
├── gui/                               # React + Go web interface
└── firmware/                          # STM32 motor control, IMU, blade safety
```

---

## Common Tasks

### I want to...

#### Launch the robot on hardware
1. Read [README.md](../README.md) → Quick Start → Launching Hardware
2. Configure serial port in [CONFIGURATION.md](CONFIGURATION.md) → hardware_bridge.yaml
3. Run: `ros2 launch mowgli_bringup mowgli.launch.py serial_port:=/dev/mowgli`

#### Test in simulation
1. Read [SIMULATION.md](SIMULATION.md) → Quick Start
2. Run: `ros2 launch mowgli_bringup simulation.launch.py`
3. Open Foxglove and send goals, or use command line

#### Tune localization
1. Read [CONFIGURATION.md](CONFIGURATION.md) → localization.yaml
2. Identify the issue in [CONFIGURATION.md](CONFIGURATION.md) → Parameter Tuning Workflow
3. Adjust process noise covariance
4. Test with: `ros2 topic echo /odometry/filtered_map`

#### Tune motion control
1. Read [CONFIGURATION.md](CONFIGURATION.md) → nav2_params.yaml → FTCController
2. Adjust PID gains or lookahead distance
3. Test in simulation: `ros2 launch mowgli_bringup simulation.launch.py`

#### Understand the system design
1. Start with [ARCHITECTURE.md](ARCHITECTURE.md) → System Overview
2. Review the package description that interests you (12 packages total)
3. Check the data flow diagram for end-to-end flow

#### Configure autonomous mowing
1. Read [CONFIGURATION.md](CONFIGURATION.md) → coverage_planner.yaml
2. Set `tool_width`, `headland_passes`, `mowing_angle` for your environment
3. Launch and monitor with Foxglove dashboard

#### Integrate new STM32 firmware
1. Read [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md)
2. Copy COBS, CRC-16, and protocol files
3. Implement packet handlers
4. Follow the migration checklist

#### Deploy to production
1. Build and test in simulation: [SIMULATION.md](SIMULATION.md)
2. Configure parameters: [CONFIGURATION.md](CONFIGURATION.md)
3. Launch on hardware: [README.md](../README.md)
4. Monitor with diagnostics: `ros2 topic echo /localization/status`

---

## Development Workflow

### Setting Up Your Development Environment

**Prerequisites:**
- Ubuntu 24.04 LTS (recommended for Jazzy)
- ROS2 Jazzy installed
- Gazebo Harmonic installed
- 8+ GB RAM, 50 GB disk space

**Quick Setup:**

```bash
# 1. Install ROS2 Jazzy (if not already installed)
curl -sSL https://repo.ros2.org/ros.key | sudo apt-key add -
sudo apt-add-repository "deb http://repo.ros2.org/ubuntu $(lsb_release -cs) main"
sudo apt install ros-jazzy-desktop gazebo-harmonic

# 2. Create workspace
mkdir -p ~/mowgli_ws/src
cd ~/mowgli_ws

# 3. Clone Mowgli
git clone https://github.com/cedbossneo/mowglinext.git src/mowglinext

# 4. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
cd ~/mowgli_ws
colcon build

# 6. Source setup
source install/setup.bash
```

### Building and Testing

**Building the workspace:**
```bash
colcon build --parallel-workers 4 --symlink-install
```

**Running tests:**
```bash
colcon test
colcon test-result --all
```

**Building a single package:**
```bash
colcon build --packages-select mowgli_nav2_plugins
```

### Running in Simulation

```bash
# Terminal 1: Launch simulation
ros2 launch mowgli_bringup simulation.launch.py

# Terminal 2: Open Foxglove
# Navigate to http://localhost:8765 in your browser

# Terminal 3: Send navigation goals (optional)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{goal: {pose: {pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}}}"
```

### Modifying Code

**Making changes to a package:**
1. Edit files in `ros2/src/<package-name>/`
2. If using `--symlink-install`, changes take effect immediately
3. Otherwise rebuild: `colcon build --packages-select <package-name>`
4. Restart the launch:
   ```bash
   ros2 launch mowgli_bringup simulation.launch.py
   ```

**Common editing tasks:**
- **Navigation parameters:** Edit `ros2/src/mowgli_bringup/config/nav2_params.yaml`
- **Coverage planner:** Edit `ros2/src/mowgli_bringup/config/coverage_planner.yaml`
- **Controller tuning:** Modify FTC or RotationShim gains, then relaunch
- **Hardware behavior:** Edit `ros2/src/mowgli_hardware/` source files

### Debugging with Logs

**Set log level for specific package:**
```bash
ros2 launch mowgli_bringup mowgli.launch.py \
  --log-level mowgli_nav2_plugins:=DEBUG
```

**View all logs:**
```bash
# Printed to terminal (default)
# Or stored in ~/.ros/log/

cat ~/.ros/log/latest_run/mowgli_nav2_plugins_*.log
```

**Monitor runtime topics:**
```bash
# Odometry
ros2 topic echo /odometry/filtered_map

# Navigation feedback
ros2 topic echo /navigate_to_pose/_action/feedback

# Coverage planner status
ros2 topic echo /coverage_status

# Diagnostics
ros2 topic echo /diagnostics
```

---

## Key Concepts

### Dual EKF Localization

Two Extended Kalman Filters work together:

- **ekf_odom** (50 Hz): Fuses wheel odometry + IMU → local `odom` frame
- **ekf_map** (20 Hz): Fuses filtered odometry + GPS → global `map` frame

[See ARCHITECTURE.md → mowgli_localization for details]

### Dual-Mode Navigation

The system uses two specialized controllers selectable via behavior tree:

- **FollowPath** (RotationShimController): Standard point-to-point navigation with heading control
- **FollowCoveragePath** (FTCController): Lateral-control-focused for mowing patterns

[See ARCHITECTURE.md → mowgli_nav2_plugins and CONFIGURATION.md → controller_server for details]

### Coverage Planning

Autonomous generation of mowing patterns:

- Headland passes around boundary
- Interior stripe pattern with configurable spacing
- Path smoothing and robot constraint satisfaction
- Integration with dual goal checkers

[See CONFIGURATION.md → coverage_planner.yaml for details]

### COBS Protocol

Consistent Overhead Byte Stuffing for robust serial communication:

- No 0x00 bytes in payload (enables framing)
- CRC-16 error detection
- <1% overhead

[See ARCHITECTURE.md → Wire Protocol: COBS + CRC-16 for details]

### Behavior Trees

Reactive, composable control logic using BehaviorTree.CPP v4:

- 10 Hz tick cycle
- Emergency guard (IsEmergency checked first)
- ReactiveSequence root (restarts on child failure)

[See ARCHITECTURE.md → mowgli_behavior for details]

---

## Support & Contributing

### Getting Help

- **General ROS2 Questions:** [ROS2 Discourse](https://discourse.ros.org)
- **OpenMower Community:** [GitHub Issues](https://github.com/ClemensElflein/open_mower_ros/issues)
- **MowgliNext Issues:** [GitHub Issues](https://github.com/cedbossneo/mowglinext/issues)

### Contributing

- Follow guidelines in [README.md](../README.md) → Contributing
- Ensure 80%+ test coverage
- Reference relevant documentation in your PRs

### Reporting Bugs

Include:
1. Hardware/OS version
2. Steps to reproduce
3. Relevant logs (use `--log-level mowgli_*:=DEBUG`)
4. Expected vs. actual behavior

---

## Glossary

| Term | Definition |
|------|-----------|
| **COBS** | Consistent Overhead Byte Stuffing (framing protocol) |
| **CRC-16** | 16-bit cyclic redundancy check (error detection) |
| **EKF** | Extended Kalman Filter (sensor fusion) |
| **FTC** | Follow-The-Carrot (path tracking algorithm) |
| **Gazebo** | Physics simulator for robotics |
| **IMU** | Inertial Measurement Unit (accelerometer + gyroscope) |
| **LiDAR** | Light Detection and Ranging (laser scanner) |
| **SLAM** | Simultaneous Localization and Mapping |
| **Nav2** | Navigation 2 (ROS2 navigation stack) |
| **RTK-GPS** | Real-Time Kinematic GPS (precise positioning) |
| **ROS2** | Robot Operating System version 2 |
| **STM32** | ARM Cortex-M microcontroller (mower firmware) |
| **TF / TF2** | Transform library (coordinate frame management) |

---

## Version History

**Current Version:** 0.2.0 (Development)

**Last Updated:** 2026-04-01

| Component | Version | Notes |
|-----------|---------|-------|
| ROS2 Distro | Jazzy | Released May 2024 |
| Gazebo | Harmonic | Latest simulation engine |
| Nav2 | Latest (Jazzy) | Auto-loaded plugins, no manual registration |
| SLAM Toolbox | Latest (Jazzy) | Outdoor-tuned defaults |
| BehaviorTree.CPP | v4 | ReactiveSequence with emergency guard |
| Total Packages | 12 | Core system complete |

---

**Start with [../README.md](../README.md) and [SIMULATION.md](SIMULATION.md) for a hands-on introduction.**

**Reference [ARCHITECTURE.md](ARCHITECTURE.md) and [CONFIGURATION.md](CONFIGURATION.md) for deep technical details.**

**Monitor with Foxglove Studio for remote visualization and diagnostics.**

Happy mowing!
