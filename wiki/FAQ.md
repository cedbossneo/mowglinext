# FAQ

## General

### What hardware do I need?

At minimum: YardForce Classic 500, ARM64 SBC (Pi 4+), u-blox ZED-F9P GPS, LDRobot LD19 LiDAR, and the Mowgli STM32 board. See [Getting Started](Getting-Started#hardware).

### Is this compatible with OpenMower?

MowgliNext is a complete ROS2 rewrite inspired by OpenMower. It uses the same hardware but a completely different software stack (ROS2 Jazzy vs ROS1 Noetic).

### Do I need an NTRIP service for RTK?

Yes, for centimeter-accurate positioning you need an RTK correction source. Many countries have free government NTRIP services, or you can set up your own base station.

## Deployment

### Can I run this on a Raspberry Pi 4?

Yes, with 4GB+ RAM. The Pi 5 is recommended for better performance. Rockchip RK3588 boards offer the best experience.

### Why Cyclone DDS instead of FastRTPS?

FastRTPS had stale shared memory issues on ARM boards causing DDS discovery failures. Cyclone DDS is more reliable for containerized multi-process setups.

### How do I update?

Watchtower auto-updates container images. For manual updates:
```bash
cd docker && docker compose pull && docker compose up -d
```

## Navigation

### The robot stops but doesn't avoid obstacles

Check that `collision_monitor` is running and the LiDAR is publishing `/scan`. The collision monitor handles real-time avoidance — costmap obstacles are disabled in the coverage planner by design.

### GPS position drifts after undocking

This is expected — GPS needs a few seconds to converge after the robot moves away from the dock. The `gps_wait_after_undock_sec` parameter controls the wait time.

### SLAM map doesn't persist

Check that the Docker volume `mowgli_maps` is mounted. The behavior tree saves the map before docking via `SaveSLAMMap`.

## Development

### How do I test without a real mower?

Use the Gazebo Harmonic simulation. The fastest way:

```bash
cd docker
docker compose -f docker-compose.simulation.yaml up dev-sim
```

There's also an automated E2E test that validates the full mowing cycle:

```bash
docker compose -f docker-compose.simulation.yaml exec dev-sim \
  bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
```

See [Simulation](Simulation) for full details.

### Can I develop in the cloud without local setup?

Yes! MowgliNext supports **GitHub Codespaces** with a pre-configured devcontainer. Click **Code → Codespaces** on the repo page to get a full ROS2 Jazzy development environment with Nav2, Gazebo, and all tools — no local installation needed. 8-core machine recommended (16-core for simulation). See [Getting Started](Getting-Started#development-with-github-codespaces--devcontainer).

### How do I add support for a different LiDAR?

Create a new directory in `sensors/` with a Dockerfile for your LiDAR driver. It must publish `/scan` (LaserScan). See [Sensors](Sensors#adding-a-new-sensor).

### Can Claude help me with my contribution?

Yes! Mention `@claude` in any issue or PR comment and it will read the codebase, answer questions, and suggest code. The repo also includes Claude Code configuration (CLAUDE.md files) for local AI-assisted development. See [AI-Assisted Contributing](AI-Assisted-Contributing).
