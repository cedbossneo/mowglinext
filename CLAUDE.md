# MowgliNext

Open-source autonomous robot mower monorepo. ROS2 Jazzy, Nav2, SLAM Toolbox, BehaviorTree.CPP v4, Fields2Cover v2.

**Website:** https://mowgli.garden | **Wiki:** https://github.com/cedbossneo/mowglinext/wiki

## Safety — READ FIRST

This robot has spinning blades. The STM32 firmware is the sole blade safety authority.

- NEVER bypass firmware blade safety checks from ROS2
- Blade commands from ROS2 are fire-and-forget — firmware decides whether to execute
- Emergency stop is handled by firmware, not software
- Flag ANY change that could affect physical behavior as safety-critical in PR reviews

## Monorepo Layout

| Directory | Language | Build | Description |
|-----------|----------|-------|-------------|
| `ros2/` | C++17, Python | `colcon build` | ROS2 stack: 12 packages (Nav2, SLAM, BT, coverage, hardware bridge) |
| `gui/` | Go, TypeScript/React | `go build`, `yarn build` | Web interface for config, map editing, monitoring |
| `docker/` | YAML, Shell | `docker compose` | Deployment configs, DDS, service orchestration |
| `sensors/` | Dockerfile | `docker build` | Dockerized sensor drivers (GPS, LiDAR) |
| `firmware/` | C | `pio run` | STM32F103 firmware (motor, IMU, blade, battery) |
| `docs/` | HTML, CSS | GitHub Pages | Landing page at mowgli.garden |

## Architecture Invariants (DO NOT VIOLATE)

1. **SLAM is sole TF authority** — `ekf_map` has `publish_tf: false`, its output is unused for TF
2. **base_link at rear wheel axis** — OpenMower convention, do not move
3. **Cyclone DDS** — not FastRTPS (stale shm issues on ARM)
4. **Map frame = GPS frame** — X=east, Y=north, no rotation transform
5. **Costmap obstacles disabled in coverage planner** — collision_monitor handles real-time avoidance
6. **dock_pose_yaw from phone compass** — measured once at installation, not computed

## Code Style

| Component | Style | Tool |
|-----------|-------|------|
| C++ (ros2/) | 2-space indent, `snake_case` files/params, `CamelCase` classes | `clang-format` (config in `ros2/.clang-format`) |
| Go (gui/) | Standard Go | `gofmt` |
| TypeScript (gui/web/) | Prettier + ESLint | `yarn lint` |
| Python (launch files) | PEP 8 | — |
| YAML (config) | 2-space indent, `snake_case` keys | — |

## Commit Conventions

```
<type>: <description>

Types: feat, fix, refactor, docs, test, chore, perf, ci
```

No Co-Authored-By lines. Keep messages concise and focused on "why".

## ROS2 Specifics

- **Distro:** Jazzy
- **DDS:** Cyclone DDS (all containers share `docker/config/cyclonedds.xml`)
- **Topics:** Mowgli-specific topics under `/mowgli/` namespace
- **Frames:** `map` (global), `odom` (local), `base_link` (robot), `lidar_link`, `imu_link`
- **Units:** SI throughout (metres, radians, seconds)
- **Dual EKF:** `ekf_odom` (50Hz, wheel+IMU) and `ekf_map` (20Hz, odom+GPS)
- **Navigation:** RPP for both transit and coverage (NOT MPPI — it jumps between adjacent swaths)

See `ros2/CLAUDE.md` for detailed package descriptions, topics, and architecture.

## Git Workflow

**NEVER commit directly to main.** Always use feature branches and PRs:
```bash
git checkout main && git pull
git checkout -b feat/my-feature    # or fix/, refactor/, test/, chore/, docs/
# ... make changes ...
git add <files> && git commit -m "feat: description"
gh pr create --title "feat: my feature" --body "..."
```

## Quick Commands

```bash
# Deploy on hardware
cd docker && docker compose up -d

# Build ROS2 locally
cd ros2 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run simulation
cd docker && docker compose -f docker-compose.simulation.yaml up dev-sim

# Build firmware
cd firmware/stm32/ros_usbnode && pio run

# GUI development
cd gui && go build -o openmower-gui && cd web && yarn dev
```

## Recommended Skills and Agents

When using Claude Code on this project:

### Skills to Use
- `/ros2-engineering` — ROS2 node patterns, QoS, launch files, Nav2, SLAM (use for any ros2/ work)
- `/cpp-coding-standards` — C++ Core Guidelines (use for C++ reviews)
- `/docker-patterns` — Dockerfile and compose patterns (use for docker/ and sensors/ work)
- `/tdd` — Test-driven development (use when adding new features)

### Agents to Invoke
- **code-reviewer** — after any code changes
- **cpp-reviewer** — after C++ changes in ros2/
- **security-reviewer** — before commits touching auth, configs, or firmware commands
- **build-error-resolver** — when colcon or Docker builds fail
- **tdd-guide** — when implementing new features
- **architect** — for design decisions spanning multiple packages

## What NOT to Do

- Do NOT add ROS1 patterns (rosserial, roscore, catkin) — this is ROS2 only
- Do NOT use FastRTPS — Cyclone DDS is required
- Do NOT mock the database/firmware in integration tests — use real interfaces
- Do NOT add `publish_tf: true` to ekf_map — SLAM is the sole TF authority
- Do NOT send blade commands without firmware safety checks
- Do NOT hardcode GPS coordinates, dock poses, or NTRIP credentials
- Do NOT use MPPI controller for coverage paths — it jumps between swaths
