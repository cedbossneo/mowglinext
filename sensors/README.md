# Sensors

Dockerized ROS2 drivers for each supported sensor. Each subdirectory contains a Dockerfile and configuration for one sensor model.

## Supported Sensors

| Sensor | Type | Directory | ROS2 Topic | Protocol |
|--------|------|-----------|------------|----------|
| u-blox ZED-F9P | RTK GPS | [`gps/`](gps/) | `/ublox_gps_node/fix` (NavSatFix) | USB-CDC |
| LDRobot LD19 | 2D LiDAR | [`lidar/`](lidar/) | `/scan` (LaserScan) | UART 230400 |

## Adding a New Sensor

To add support for a different GPS or LiDAR model:

1. Create a new directory (e.g., `sensors/lidar-rplidar/`)
2. Add a `Dockerfile` that builds the ROS2 driver and publishes the expected topic
3. Add a `ros2_entrypoint.sh` for environment setup
4. Update `docker/docker-compose.yaml` to point the service's `build.context` at your new directory
5. Ensure the driver publishes on the standard topic (`/scan` for LiDAR, `/ublox_gps_node/fix` or `/gps/fix` for GPS)

## Building

Images are built automatically by the CI workflow (`.github/workflows/docker.yml`) for `linux/amd64` and `linux/arm64`.

To build locally:

```bash
docker build -t mowgli-gps sensors/gps/
docker build -t mowgli-lidar --target runtime sensors/lidar/
```
