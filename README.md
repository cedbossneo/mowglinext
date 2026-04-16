<p align="center">
  <img src="logo.svg" alt="MowgliNext" width="320">
</p>

<p align="center">
  Autonomous robot mower built on ROS2 Kilted — a fresh start from the ground up<br>
  with LiDAR SLAM, RTK-GPS, behavior trees, and intelligent coverage planning.
</p>

<p align="center">
  <strong><a href="https://mowgli.garden">Website</a></strong> · <strong><a href="https://github.com/cedbossneo/mowglinext/wiki">Wiki</a></strong> · <strong><a href="https://github.com/cedbossneo/mowglinext/discussions">Discussions</a></strong> · <strong><a href="https://github.com/cedbossneo/mowglinext/issues">Issues</a></strong>
</p>

---

> **Beta — Work in Progress**
>
> MowgliNext is under active development and **not ready for production use**. Expect breaking changes, incomplete features, and rough edges. We're building in the open and welcome early adopters and contributors, but please don't rely on this for your daily mowing just yet. If you're looking for a stable, proven solution today, check out [OpenMower](https://openmower.de/) below.

---

## Project Status

MowgliNext has a fully functional autonomous mowing stack running on real hardware. Here's what's built and working today, and what's coming next.

### Core Stack

| Component | Status | Details |
|-----------|:------:|---------|
| ROS2 Kilted | :white_check_mark: | Full stack on differential-drive mower, Cyclone DDS, multi-arch Docker |
| RTAB-Map SLAM | :white_check_mark: | 2D LiDAR scan matching publishes `map → odom`; database persisted to `/ros2_ws/maps/rtabmap.db`, built from source from `ros2/src/rtabmap_ros` on top of `introlab3it/rtabmap:noble-kilted` |
| FusionCore Localization | :white_check_mark: | Single UKF (50 Hz) fuses GPS, IMU, and wheel odometry; publishes `odom → base_footprint`. Pose sigma ~25 mm with RTK fixed |
| RTK-GPS | :white_check_mark: | u-blox F9P support with GPS degradation handling and wait-for-fix logic. Fused directly in FusionCore — no separate GPS-SLAM corrector |
| Nav2 Navigation | :white_check_mark: | RPP controller for transit paths |
| Collision Monitor | :white_check_mark: | LiDAR-based real-time obstacle detection with 3-zone approach (stop, slow, approach) |
| Obstacle Tracker | :white_check_mark: | DBSCAN clustering, persistence promotion, overlapping merge |

### Coverage Planning & Execution

| Component | Status | Details |
|-----------|:------:|---------|
| Cell-based Strip Planner | :white_check_mark: | `map_server_node` plans strips on demand — no pre-planned full path. Progress persisted in `mow_progress` grid layer (survives restarts) |
| FTCController | :white_check_mark: | Follow-the-Carrot with 3-axis PID — sub-10 mm lateral accuracy on coverage paths |
| Strip-by-strip BT | :white_check_mark: | `GetNextStrip` -> `TransitToStrip` -> `FollowStrip`, one strip at a time with dynamic replanning |
| Obstacle-aware routing | :white_check_mark: | Skips blocked strips, reroutes around obstacles, recovers from stuck |

### Autonomy & Behavior

| Component | Status | Details |
|-----------|:------:|---------|
| Behavior Tree | :white_check_mark: | Full mowing cycle: undock, plan, mow, dock — BehaviorTree.CPP v4 |
| Area Recording | :white_check_mark: | Drive the boundary to define mowing areas; Douglas-Peucker simplification, live trajectory preview |
| Manual Mowing | :white_check_mark: | Dedicated teleop + blade mode with collision monitor, GPS, and SLAM still active |
| Emergency Auto-Reset | :white_check_mark: | Robot on dock auto-clears emergency. Firmware remains sole safety authority |
| Rain detection | :white_check_mark: | Pause-and-wait behavior during rain, resume when clear |
| Battery monitoring | :white_check_mark: | Low-battery dock with resume after charge (95% threshold) |
| Obstacle replanning | :white_check_mark: | Re-plan coverage when new obstacles detected |
| Recovery sequences | :white_check_mark: | Stuck detection, backup, costmap clear, re-attempt |

### Infrastructure

| Component | Status | Details |
|-----------|:------:|---------|
| Simulation | :white_check_mark: | Gazebo Harmonic with full sensor simulation (LiDAR, IMU, GPS, wheel odom) |
| E2E Testing | :white_check_mark: | Automated simulation testing with live dashboard metrics |
| Web GUI | :white_check_mark: | React + Go interface for monitoring and control |
| Docker Deployment | :white_check_mark: | Multi-arch (amd64 / arm64), Cyclone DDS, Docker Compose |
| Firmware | :white_check_mark: | STM32F103 for motor control, IMU, blade safety, battery |
| Interactive Installer | :white_check_mark: | Shell-based with hardware presets, i18n, UART detection |

### Planned

| Feature | Description |
|---------|-------------|
| Headland passes | Mow the perimeter outline before filling the interior |
| 3D terrain handling | Slope-aware planning and speed adjustment |
| Multi-zone mowing | Schedule and sequence multiple mowing areas |
| Improved obstacle shapes | Track obstacle contours beyond bounding circles |
| Coverage checkpoint resume | Resume a mowing session after reboot or power loss |

### Supported Hardware

| Category | Tested Models |
|----------|---------------|
| Chassis | YardForce Classic 500, 500B, LUV1000Ri (OpenMower-compatible differential drive) |
| Compute | Rockchip RK3566 / RK3588, Raspberry Pi 4 / 5 |
| RTK-GPS | u-blox ZED-F9P (recommended), other u-blox F9P boards |
| LiDAR | LDRobot LD19, RPLiDAR A-series |
| IMU | WT931 or similar 9-DOF |
| Motor controller | STM32F103-based (Mowgli firmware) |

---

## A Word About OpenMower

MowgliNext exists because of [OpenMower](https://openmower.de/). Full stop.

OpenMower is an incredible project that proved robot mowers can be truly intelligent — not just following a random bounce pattern or a buried wire, but actually understanding where they are and planning where to go. It inspired an entire community of builders, and we owe them a huge debt of gratitude.

**MowgliNext is not a competitor to OpenMower. It's a different approach born from different needs.**

The OpenMower philosophy is to replace the stock electronics inside the mower with custom boards designed for the job. This gives them full control and a clean hardware platform to build on. It's a great approach, and their rapid iteration on board design is impressive.

The Mowgli philosophy is different: we work with the existing stock boards. We started by adding features on top of the original YardForce hardware — custom firmware, additional sensors, new capabilities. Over time, the list of features we wanted to build grew so ambitious that evolving within the original ROS1 architecture became increasingly complex. We needed a fresh foundation.

So MowgliNext is a ground-up rewrite on ROS2, designed to let Mowgli evolve quickly without being constrained by the original architecture. By going our own way, we also give OpenMower more freedom — they can iterate on their boards and software without worrying about breaking things for people trying to follow along with different hardware.

We'd be happy to support OpenMower firmware in MowgliNext if the community is interested. At the end of the day, we're all trying to make our mowers smarter. Different paths, same goal.

Thank you, OpenMower team. You showed us what's possible.

---

## Monorepo Structure

| Directory | Description |
|-----------|-------------|
| [`ros2/`](ros2/) | ROS2 stack: Nav2, RTAB-Map SLAM, behavior trees, coverage planner, hardware bridge |
| [`install/`](install/) | Interactive installer, hardware presets, modular Docker Compose configs |
| [`docker/`](docker/) | Docker Compose deployment for manual setup, DDS config |
| [`sensors/`](sensors/) | Dockerized sensor drivers (GPS, LiDAR) — one directory per model |
| [`gui/`](gui/) | React + Go web interface for configuration, map editing, and monitoring |
| [`firmware/`](firmware/) | STM32 firmware for motor control, IMU, blade safety |
| [`docs/`](docs/) | GitHub Pages site at [mowgli.garden](https://mowgli.garden) — install composer |

## Quick Start

Visit [mowgli.garden](https://mowgli.garden/#getting-started) to configure your hardware and get a personalized install command. Or run the installer directly:

```bash
curl -sSL https://mowgli.garden/install.sh | bash
```

The web composer lets you pick your GPS, LiDAR, and rangefinders — the generated command pre-configures the installer so you skip those prompts. It still walks you through GPS datum, dock position, NTRIP credentials, and launches everything automatically.

GUI at `http://<mower-ip>:4006` | Foxglove at `ws://<mower-ip>:8765`

See the [Getting Started](https://github.com/cedbossneo/mowglinext/wiki/Getting-Started) wiki page for full setup instructions and manual install options.

## Architecture

```
┌─────────────────────────────────────────────────┐
│  GUI (React + Go)          :4006                │
├─────────────────────────────────────────────────┤
│  ROS2 Stack (Kilted)                             │
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐│
│  │ Nav2     │ │ RTAB-Map │ │ Behavior Tree    ││
│  │ (navigate│ │ SLAM     │ │ (main_tree.xml)  ││
│  │  dock    │ │ map→odom │ │                  ││
│  │  cover)  │ │          │ │                  ││
│  └──────────┘ └──────────┘ └──────────────────┘│
│  ┌──────────┐ ┌──────────┐ ┌──────────────────┐│
│  │ Coverage │ │FusionCore│ │ Hardware Bridge  ││
│  │ Planner  │ │ UKF      │ │ (serial ↔ ROS2) ││
│  │(cell-str)│ │ GPS+IMU+ │ │                  ││
│  │          │ │ wheels   │ │                  ││
│  └──────────┘ └──────────┘ └──────────────────┘│
├──────────────────────┬──────────────────────────┤
│  Sensors (Docker)    │  STM32 Firmware          │
│  GPS (u-blox F9P)    │  Motor control           │
│  LiDAR (LD19)        │  IMU, blade safety       │
└──────────────────────┴──────────────────────────┘
```

## Documentation

| Resource | What's there |
|----------|-------------|
| [Website](https://mowgli.garden) | Landing page, features overview, getting started |
| [Wiki](https://github.com/cedbossneo/mowglinext/wiki) | Full reference: architecture, configuration, deployment, sensors, firmware, BT, FAQ |
| [Discussions](https://github.com/cedbossneo/mowglinext/discussions) | Community Q&A |

## Hardware

- YardForce Classic 500 chassis (500B, LUV1000Ri also supported)
- ARM64 SBC — Rockchip RK3566/RK3588, Raspberry Pi 4/5
- LDRobot LD19 LiDAR (2D, UART)
- u-blox ZED-F9P RTK GPS (USB-CDC)
- Custom STM32 board for motor/blade/IMU

## Contributing

We welcome contributions! Claude AI reviews every PR and assists in issues.

- [Contributing Guide](CONTRIBUTING.md)
- [AI-Assisted Contributing](https://github.com/cedbossneo/mowglinext/wiki/AI-Assisted-Contributing) — how to use AI tools effectively
- [Code of Conduct](CODE_OF_CONDUCT.md)
- Mention **@claude** in any issue or PR for AI assistance

## Acknowledgments

- **[cloudn1ne](https://github.com/cloudn1ne)** — for the original Mowgli reverse engineering work that made everything possible
- **nekraus** — for the countless late nights spent together making things actually work
- **[OpenMower](https://openmower.de/)** — for proving robot mowers can be truly intelligent and inspiring this entire effort
- **Mowgli French Community** — for all the testing, feedback, and encouragement that kept us going
- **Every Mowgli user** — every install, every bug report, every "it works!" gives us the courage to keep spending nights on this project

## License

[GPLv3](LICENSE) — same as OpenMower, because open source is how we all win.
