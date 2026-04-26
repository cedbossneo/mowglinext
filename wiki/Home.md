# MowgliNext Wiki

Welcome to the MowgliNext documentation wiki — the reference hub for the open-source ROS2 autonomous robot mower.

## Quick Links

| Resource | Description |
|----------|-------------|
| [Getting Started](Getting-Started) | First-time setup, DevContainer, and deployment |
| [Architecture](Architecture) | System design, packages, data flow |
| [Configuration](Configuration) | All YAML parameters explained |
| [Deployment](Deployment) | Docker Compose setup and troubleshooting |
| [Simulation](Simulation) | Gazebo Harmonic testing and E2E test |
| [Sensors](Sensors) | GPS and LiDAR driver setup |
| [Firmware](Firmware) | STM32 integration and COBS protocol |
| [Behavior Trees](Behavior-Trees) | BT nodes, tree structure, control flow |
| [GUI](GUI) | Web interface (React + Go) |
| [Contributing](Contributing) | How to contribute to MowgliNext |
| [FAQ](FAQ) | Frequently asked questions |

## Project Links

- **Website:** https://mowgli.garden
- **GitHub:** https://github.com/cedbossneo/mowglinext
- **Issues:** https://github.com/cedbossneo/mowglinext/issues
- **Discussions:** https://github.com/cedbossneo/mowglinext/discussions

## Monorepo Structure

```
mowglinext/
├── ros2/        ROS2 stack (Nav2, robot_localization dual EKF, Kinematic-ICP, BT, coverage planner)
├── docker/      Docker Compose deployment and config
├── sensors/     Dockerized sensor drivers (GPS, LiDAR)
├── gui/         React + Go web interface
├── firmware/    STM32 firmware (motor, IMU, blade)
├── install/     Interactive installer + gh-pages install composer
├── docs/        GitHub Pages landing page + first-boot checklist
└── wiki/        This wiki (auto-synced to the GitHub wiki)
```

## Key Design Decisions

1. **base_link at rear wheel axis** — OpenMower convention.
2. **robot_localization (dual EKF) is the sole localizer.** `ekf_odom_node` owns `odom → base_footprint` (wheels + gyro, continuous), `ekf_map_node` owns `map → odom` (adds `/gps/pose_cov` + GPS-COG yaw). Both run under `two_d_mode`. Kinematic-ICP's output enters `ekf_odom_node` as a body-frame twist on `/encoder2/odom`. No SLAM.
3. **Cyclone DDS** — replaces FastRTPS (stale shm on ARM).
4. **Map frame = GPS frame** — X=east, Y=north, no rotation.
5. **Firmware is blade safety authority** — ROS2 is fire-and-forget.
6. **Collision monitor for avoidance** — costmap obstacles disabled in planner.
7. **Cell-based strip coverage** — no full-path pre-planning, strips fetched one at a time.
8. **Emergency auto-reset on dock** — firmware decides whether to clear latch.
9. **Area recording via BT** — drive boundary, Douglas-Peucker simplification, save polygon.
10. **Kinematic-ICP runs on a parallel TF tree** — no feedback into robot_localization; LiDAR drift correction only.
11. **Dedicated manual mowing mode** — teleop with collision_monitor, GPS, robot_localization, and Kinematic-ICP active.
