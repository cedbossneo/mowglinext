# Mowgli ROS2 Documentation Index

Welcome to the comprehensive documentation for the Mowgli ROS2 project—a complete rewrite of OpenMower in ROS2 Humble with modern navigation, SLAM, and behavior tree control.

## Quick Navigation

### For New Users

Start here to get up and running:

1. **[../README.md](../README.md)** – Project overview, features, and quick start
   - What is Mowgli ROS2?
   - Hardware requirements
   - Building from source
   - Quick start (Docker, hardware, simulation)

2. **[SIMULATION.md](SIMULATION.md)** – Running in Gazebo Ignition
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
   - mowgli_localization (odometry and GPS fusion)
   - mowgli_nav2_plugins (FTC controller)
   - mowgli_behavior (behavior trees)
   - mowgli_bringup (configuration and launch)

2. **[../README.md](../README.md)** – Development workflow
   - Contributing guidelines
   - Test coverage requirements
   - Building and testing

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
| Gazebo controls | [SIMULATION.md](SIMULATION.md) | Gazebo Keyboard Controls |
| RViz visualization | [SIMULATION.md](SIMULATION.md) | RViz2 Visualization |
| Troubleshooting | [SIMULATION.md](SIMULATION.md) | Troubleshooting |

### Configuration & Tuning

| Topic | Document | Key Section |
|-------|----------|-------------|
| Serial port setup | [CONFIGURATION.md](CONFIGURATION.md) | hardware_bridge.yaml |
| EKF tuning | [CONFIGURATION.md](CONFIGURATION.md) | localization.yaml |
| Motion control tuning | [CONFIGURATION.md](CONFIGURATION.md) | nav2_params.yaml → FTCController |
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

## File Structure

```
mowgli_ros2/
├── README.md                          # Project overview & quick start
│
├── docs/
│   ├── INDEX.md                       # This file
│   ├── ARCHITECTURE.md                # Technical architecture (detailed)
│   ├── CONFIGURATION.md               # Parameter reference & tuning
│   ├── FIRMWARE_MIGRATION.md          # STM32 integration guide
│   └── SIMULATION.md                  # Gazebo simulation guide
│
├── src/
│   ├── mowgli_interfaces/             # Message definitions
│   ├── mowgli_hardware/               # Serial bridge to STM32
│   ├── mowgli_localization/           # Odometry & GPS fusion
│   ├── mowgli_nav2_plugins/           # FTC controller plugin
│   ├── mowgli_behavior/               # Behavior tree nodes
│   └── mowgli_bringup/                # Launch files & configuration
│
└── [build artifacts]
```

---

## Common Tasks

### I want to...

#### Launch the robot on hardware
1. Read [README.md](../README.md) → Quick Start → Launching Hardware
2. Configure serial port in [CONFIGURATION.md](CONFIGURATION.md) → hardware_bridge.yaml
3. Run: `ros2 launch mowgli_bringup mowgli.launch.py serial_port:=/dev/ttyUSB0`

#### Test in simulation
1. Read [SIMULATION.md](SIMULATION.md) → Quick Start
2. Run: `ros2 launch mowgli_bringup simulation.launch.py`
3. Send goals from RViz or command line

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
2. Review the package description that interests you
3. Check the data flow diagram for end-to-end flow

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

## Key Concepts

### Dual EKF Localization

Two Extended Kalman Filters work together:

- **ekf_odom** (50 Hz): Fuses wheel odometry + IMU → local `odom` frame
- **ekf_map** (20 Hz): Fuses filtered odometry + GPS → global `map` frame

[See ARCHITECTURE.md → mowgli_localization for details]

### FTC Local Controller

Follow-The-Carrot algorithm with three independent PIDs:

- Linear velocity (cross-track error)
- Angular velocity (heading error)
- Acceleration (velocity error)

[See ARCHITECTURE.md → mowgli_nav2_plugins for details]

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
- **Mowgli ROS2 Issues:** [GitHub Issues](https://github.com/ClemensElflein/mowgli-ros2/issues)

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

**Current Version:** 0.1.0 (Development)

**Last Updated:** 2026-03-27

| Component | Version | ROS2 Distro |
|-----------|---------|-------------|
| mowgli_ros2 | 0.1.0 | Humble |
| Nav2 | Latest | Humble |
| SLAM Toolbox | Latest | Humble |
| BehaviorTree.CPP | v4 | – |
| Gazebo Ignition | Fortress+ | – |

---

**Start with [../README.md](../README.md) and [SIMULATION.md](SIMULATION.md) for a hands-on introduction.**

**Reference [ARCHITECTURE.md](ARCHITECTURE.md) and [CONFIGURATION.md](CONFIGURATION.md) for deep technical details.**

Happy mowing! 🚜
