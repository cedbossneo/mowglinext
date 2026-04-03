# MowgliNext Documentation

## Getting Started

- [Deployment Guide](../docker/README.md) — Docker Compose setup, hardware requirements, installation, configuration, troubleshooting
- [Configuration Reference](CONFIGURATION.md) — All YAML parameters for Nav2, SLAM, EKF, coverage planner
- [Simulation](SIMULATION.md) — Gazebo Harmonic simulation for testing without hardware

## Architecture

- [System Architecture](ARCHITECTURE.md) — ROS2 packages, data flow, TF tree, node graph
- [Firmware Integration](FIRMWARE_MIGRATION.md) — STM32 serial protocol (COBS), packet structure, topic mapping

## Components

| Component | README | Description |
|-----------|--------|-------------|
| ROS2 Stack | [`ros2/README.md`](../ros2/README.md) | Nav2, SLAM, behavior trees, coverage planner |
| Docker | [`docker/README.md`](../docker/README.md) | Compose services, DDS config, deployment modes |
| Sensors | [`sensors/README.md`](../sensors/README.md) | GPS and LiDAR Dockerized drivers |
| GUI | [`gui/README.md`](../gui/README.md) | React + Go web interface |
| Firmware | [`firmware/README.md`](../firmware/README.md) | STM32 motor control, IMU, blade safety |

## Key Design Decisions

1. **base_link at rear wheel axis** — follows OpenMower convention for differential drive
2. **SLAM is sole TF authority** — EKF publishes odometry only, `publish_tf: false`
3. **Cyclone DDS** — replaces FastRTPS due to stale shared memory on ARM
4. **Map frame = GPS frame** — X=east, Y=north, no rotation needed for area coordinates
5. **Firmware is blade safety authority** — ROS2 blade commands are fire-and-forget
6. **Collision monitor for real-time avoidance** — costmap obstacles disabled in coverage planner
7. **dock_pose_yaw from phone compass** — measured once at dock installation

## Configuration

Robot-specific config lives in `docker/config/mowgli/mowgli_robot.yaml`. This single file controls:
- GPS datum (latitude, longitude, heading)
- Dock position and orientation
- NTRIP RTK correction server
- Battery voltage thresholds
- Mowing parameters (speed, blade height)
- Sensor positions (GPS antenna, LiDAR)

See [CONFIGURATION.md](CONFIGURATION.md) for the full parameter reference.
