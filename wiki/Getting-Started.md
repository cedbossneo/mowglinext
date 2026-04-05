# Getting Started

## Hardware

### Compute Board

Any ARM64 SBC running Linux with Docker support:
- **Recommended:** Rockchip RK3566, RK3588
- **Also works:** Raspberry Pi 4, Pi 5
- **Minimum:** 4-core ARM64, 4 GB RAM, 16 GB storage

### Mower Chassis

| Model | Status |
|-------|--------|
| YardForce Classic 500 | Primary target |
| YardForce Classic 500B | Supported |
| YardForce LUV1000Ri | Supported |

### Sensors

| Sensor | Model | Connection |
|--------|-------|------------|
| RTK GPS | u-blox ZED-F9P (simpleRTK2B) | USB-CDC |
| LiDAR | LDRobot LD19 | UART 230400 |
| IMU | Pololu AltIMU-10 v5 | I2C (on STM32) |

### Firmware Board

Custom Mowgli STM32F103 PCB — handles motor control, IMU, blade safety, battery monitoring.

## Development with GitHub Codespaces / DevContainer

The fastest way to explore and develop MowgliNext — no local setup required:

### GitHub Codespaces (cloud)

1. Go to [github.com/cedbossneo/mowglinext](https://github.com/cedbossneo/mowglinext)
2. Click **Code → Codespaces → Create codespace on main**
3. Select a **8-core** machine type (16-core recommended for simulation)
4. Wait for the container to build (~10 min first time, cached after)

### VS Code DevContainer (local)

1. Install [Docker](https://docs.docker.com/get-docker/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. Clone the repo: `git clone https://github.com/cedbossneo/mowglinext.git`
3. Open in VS Code → **Reopen in Container** when prompted

### What's included

The devcontainer provides a complete ROS2 Jazzy development environment:

- Full Nav2 navigation stack, SLAM Toolbox, robot_localization
- Gazebo Harmonic simulation (headless)
- Fields2Cover v2.0.0 coverage planner (built from source)
- Foxglove Bridge + rosbridge for visualization
- Claude Code CLI + GitHub CLI for AI-assisted development
- clang-format, gdb, htop, and other dev tools
- Auto-sourced ROS2 workspace

**Forwarded ports:**

| Port | Service |
|------|---------|
| 8765 | Foxglove WebSocket |
| 6080 | noVNC (Gazebo GUI) |
| 9090 | rosbridge |

### Build and run simulation in devcontainer

```bash
# Build all packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Launch simulation
ros2 launch mowgli_bringup sim_full_system.launch.py headless:=true
```

## Quick Start on Hardware (Automated)

The easiest way to deploy on real hardware is the interactive install script. SSH into your mower's board and run:

```bash
curl -sSL https://raw.githubusercontent.com/cedbossneo/mowglinext/main/docker/install.sh | bash
```

The installer handles:
- Docker installation (if needed)
- udev rules for serial devices (`/dev/mowgli`, `/dev/gps`)
- Interactive configuration (GPS datum, dock position, NTRIP, battery, sensor positions)
- Pulling and launching all containers
- Post-install diagnostics

Run with `--check` to diagnose an existing installation without reinstalling.

## Manual Install

If you prefer to set things up manually:

### 1. Clone

```bash
git clone https://github.com/cedbossneo/mowglinext.git
cd mowglinext
```

### 2. Set Up udev Rules

Create stable device symlinks:

```bash
# /etc/udev/rules.d/50-mowgli.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="6018", SYMLINK+="mowgli", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
```

Reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### 3. Configure

```bash
cd docker
cp .env.example .env
nano config/mowgli/mowgli_robot.yaml
```

Key settings to change:
- `datum_lat` / `datum_long` — your GPS reference point
- `dock_pose_x` / `dock_pose_y` / `dock_pose_yaw` — dock position
- `ntrip_host` / `ntrip_user` / `ntrip_password` — RTK correction source

### 4. Launch

```bash
docker compose up -d
```

### 5. Access

| Service | URL |
|---------|-----|
| GUI | `http://<mower-ip>:4006` |
| Foxglove | `ws://<mower-ip>:8765` |
| Rosbridge | `ws://<mower-ip>:9090` |

## Next Steps

- [Simulation](Simulation) — test without hardware, run E2E tests
- [Configuration](Configuration) — tune parameters for your environment
- [Deployment](Deployment) — advanced Docker deployment options
- [Contributing](Contributing) — contribute to MowgliNext
