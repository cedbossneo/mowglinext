# Mowgli ROS2 Runtime Configuration

This directory is bind-mounted read-only into the `mowgli` container at
`/ros2_ws/config/`.

The `mowgli_ros2` runtime image reads parameter files from that path at launch.
Place any customised YAML parameter files here to override the built-in
defaults without rebuilding the image.

## Shipped default

| File | Purpose |
|------|---------|
| `mowgli_robot.yaml` | **Site-specific overrides** — GPS datum, NTRIP, dock pose, battery, mowing speed. Edit this first. |

## Additional overrideable files

| File | Purpose |
|------|---------|
| `nav2_params.yaml` | Nav2 stack tuning (costmap inflation, planner, controller, collision monitor) |
| `localization.yaml` | FusionCore UKF tuning (GPS/IMU/wheel fusion covariances, base noise) |
| `hardware_bridge.yaml` | Serial port, baud rate, heartbeat/publish rates |
| `coverage_planner.yaml` | B-RV coverage planner (tool width, headland, Voronoi transit, sweep direction) |
| `behavior_tree.yaml` | Behavior tree tick rate, battery thresholds |
| `mqtt_bridge.yaml` | MQTT broker connection for Home Assistant integration |

RTAB-Map SLAM parameters (scan subsampling, loop closure, grid output)
live in `src/mowgli_bringup/launch/rtabmap.launch.py` inside the
`mowgli_ros2` image — change and rebuild to tune them.

## How parameter override works

The launch files in `mowgli_bringup` check for the presence of files under
`/ros2_ws/config/` and, when found, pass them to each node via the
`params_file` argument.  If a file is absent the built-in default is used.

## Quick start

1. Edit `mowgli_robot.yaml` — set your GPS datum, NTRIP, and dock pose
2. Restart: `docker compose restart mowgli`
3. For advanced tuning, copy more files from the image:

```bash
# See all built-in config files
docker exec mowgli-ros2 ls /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/

# Copy one out for local editing
docker exec mowgli-ros2 cat /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params.yaml > config/mowgli/nav2_params.yaml
```

## Note on GPS device

If your GPS receiver appears as `/dev/ttyUSB1` (or `/dev/gps` via udev) rather
than the default, add a `devices` override in a `docker-compose.override.yaml`:

```yaml
services:
  mowgli:
    devices:
      - /dev/mowgli:/dev/mowgli
      - /dev/ttyUSB1:/dev/gps
```
