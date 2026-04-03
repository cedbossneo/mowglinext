# MowgliNext Documentation

All project documentation lives here — this is the single source of truth.

## Guides

| Document | Description |
|----------|-------------|
| [INDEX.md](INDEX.md) | Full navigation index — start here to find anything |
| [ARCHITECTURE.md](ARCHITECTURE.md) | System architecture, packages, data flow, TF tree, wire protocol |
| [CONFIGURATION.md](CONFIGURATION.md) | All YAML parameters for Nav2, SLAM, EKF, coverage planner |
| [SIMULATION.md](SIMULATION.md) | Gazebo Harmonic simulation guide |
| [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md) | STM32 COBS protocol integration |

## Component READMEs

Each component has its own README with build/run instructions:

| Component | README |
|-----------|--------|
| ROS2 Stack | [`ros2/README.md`](../ros2/README.md) |
| Docker Deployment | [`docker/README.md`](../docker/README.md) |
| Sensors | [`sensors/README.md`](../sensors/README.md) |
| GUI | [`gui/README.md`](../gui/README.md) |
| Firmware | [`firmware/README.md`](../firmware/README.md) |

## Quick Links

- **Deploy to hardware:** See [`docker/README.md`](../docker/README.md) for Docker Compose setup
- **Test in simulation:** See [SIMULATION.md](SIMULATION.md)
- **Tune parameters:** See [CONFIGURATION.md](CONFIGURATION.md)
- **Add a sensor:** See [`sensors/README.md`](../sensors/README.md)

## Key Design Decisions

1. **base_link at rear wheel axis** — follows OpenMower convention for differential drive
2. **SLAM is sole TF authority** — EKF publishes odometry only, `publish_tf: false`
3. **Cyclone DDS** — replaces FastRTPS due to stale shared memory on ARM
4. **Map frame = GPS frame** — X=east, Y=north, no rotation needed
5. **Firmware is blade safety authority** — ROS2 blade commands are fire-and-forget
6. **Collision monitor for real-time avoidance** — costmap obstacles disabled in coverage planner
7. **dock_pose_yaw from phone compass** — measured once at dock installation
