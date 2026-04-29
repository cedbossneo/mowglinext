# FAQ

## General

### What hardware do I need?

At minimum: YardForce Classic 500, ARM64 SBC (Pi 4+), u-blox ZED-F9P GPS, LDRobot LD19 LiDAR, and the Mowgli STM32 board. See [Getting Started](Getting-Started#hardware).

### Is this compatible with OpenMower?

MowgliNext is a complete ROS2 rewrite inspired by OpenMower. It uses the same hardware but a completely different software stack (ROS2 Kilted vs ROS1 Noetic).

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

### Where does the map come from?

There is **no SLAM**. The `/map` OccupancyGrid is built by `map_server_node` from polygons you record during area-recording mode (drive the boundary, hit *Finish*, the trajectory is Douglas-Peucker simplified and saved). Persistence is just JSON files inside the `mowgli_maps` Docker volume — make sure that volume is mounted.

### EKF or fusion_graph: which map-frame localizer should I use?

Both publish `map → odom` + `/odometry/filtered_map`. Pick one:

- **`ekf_map_node` (default — `use_fusion_graph:=false`)** — robot_localization global EKF. Simpler, mature, fuses wheels + IMU + GPS + GPS-COG yaw under `two_d_mode`. Right answer for most installations and the only one used by CI.
- **`fusion_graph_node` (opt-in — `use_fusion_graph:=true`)** — GTSAM iSAM2 factor graph. Same inputs as the EKF, plus optional LiDAR scan-matching (`use_scan_matching`) and loop-closure (`use_loop_closure`) factors. Worth enabling if your garden has multi-minute RTK-Float windows, GPS-denied corners, or you simply want LiDAR drift correction in the map frame. Costs ~5 ms/tick of CPU at 10 Hz with scan matching on.

The two are mutually exclusive — only one publishes `map → odom`. `ekf_odom_node` (local EKF, `odom → base_footprint`) is unchanged in either mode. Toggle from the GUI's *Settings → Localization* section, then *Restart ROS2*.

### What do the *Save graph* / *Clear graph* buttons in Diagnostics do?

They call the `~/save_graph` / `~/clear_graph` services on `fusion_graph_node`. Only relevant when `use_fusion_graph:=true`.

- **Save graph** — persists `<graph_save_prefix>.{graph,scans,meta}` to disk. The node also auto-saves on dock arrival, on RECORDING→IDLE transitions, and every 5 min during AUTONOMOUS state, so the button is mostly a "checkpoint before I shut down ROS2 manually" affordance.
- **Clear graph** — wipes iSAM2 + accumulated factors + per-node scans. The node stays alive; the next valid pose seed (GPS, set_pose, or scan-match relocalization) re-initializes. Use after relocating the robot to a new garden.

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

Yes! MowgliNext supports **GitHub Codespaces** with a pre-configured devcontainer. Click **Code → Codespaces** on the repo page to get a full ROS2 Kilted development environment with Nav2, Gazebo, and all tools — no local installation needed. 8-core machine recommended (16-core for simulation). See [Getting Started](Getting-Started#development-with-github-codespaces--devcontainer).

### How do I add support for a different LiDAR?

Create a new directory in `sensors/` with a Dockerfile for your LiDAR driver. It must publish `/scan` (LaserScan). See [Sensors](Sensors#adding-a-new-sensor).

### Can Claude help me with my contribution?

Yes! Mention `@claude` in any issue or PR comment and it will read the codebase, answer questions, and suggest code. The repo also includes Claude Code configuration (CLAUDE.md files) for local AI-assisted development. See [AI-Assisted Contributing](AI-Assisted-Contributing).
