# Sensors

Dockerized ROS2 drivers for each supported sensor. Each subdirectory contains a Dockerfile and configuration for one sensor model.

## Supported Sensors

| Sensor | Type | Directory | ROS2 Topic | Transport |
|--------|------|-----------|------------|-----------|
| u-blox ZED-F9P | RTK GPS (UBX) | [`gps/`](gps/) | `/gps/fix` (NavSatFix) | USB-CDC (libusb) |
| Unicore UM980 / generic NMEA | RTK GPS (NMEA) | [`gps-nmea/`](gps-nmea/) | `/gps/fix` (NavSatFix) | Serial (`/dev/gps`) |
| LDRobot LD19 / LD06 / LD14 | 2D LiDAR | [`lidar-ldlidar/`](lidar-ldlidar/) | `/scan` (LaserScan) | UART 230400 |
| Slamtec RPLiDAR A-series | 2D LiDAR | [`lidar-rplidar/`](lidar-rplidar/) | `/scan` (LaserScan) | USB / UART |
| LDRobot STL27L | 2D LiDAR (experimental) | [`lidar-stl27l/`](lidar-stl27l/) | `/scan` (LaserScan) | UART 230400 |
| MAVLink autopilot | MAVROS bridge | [`mavros/`](mavros/) | MAVROS topics | Serial / UDP |

## Choosing a GPS image

- `gps/` — for u-blox ZED-F9P over USB-CDC. Uses `ublox_dgnss` (libusb) and a bundled NTRIP client; exposes the full UBX topic tree including RTK Fixed/Float status.
- `gps-nmea/` — for any RTK receiver that streams NMEA over a serial port (Unicore UM980/UM982, ArduSimple simpleRTK2B, Septentrio, etc.). Uses `nmea_navsat_driver` plus `str2str` (rtklib) for NTRIP RTCM3 injection back to the serial device.

Both images publish `/gps/fix` (`sensor_msgs/NavSatFix`) so downstream consumers (FusionCore, `navsat_to_absolute_pose`) don't care which one is running.

> **Fork note:** this fork's CI builds only `gps-nmea`. The `gps` (ublox) image
> is pulled from upstream `cedbossneo/mowglinext` — its source under `sensors/gps/`
> stays in sync, but the image push to this fork's GHCR is currently disabled
> (push-by-digest race for the multi-stage ublox build). To re-enable, add
> `gps` back to both `matrix.image` lists in `.github/workflows/sensors-docker.yml`.

## Adding a New Sensor

1. Create a new directory (e.g., `sensors/gps-foo/`).
2. Add a `Dockerfile` that builds the ROS2 driver and publishes the expected topic.
3. Add a `ros2_entrypoint.sh` for environment setup.
4. Add a matching `install/compose/docker-compose.<name>.yml` snippet.
5. Wire it into `install/lib/compose.sh::build_compose_stack` (and `install/lib/env.sh` for image defaults).
6. Add the new image to the matrix in `.github/workflows/sensors-docker.yml`.

## Building

Images are built automatically by the CI workflow ([`.github/workflows/sensors-docker.yml`](../.github/workflows/sensors-docker.yml)) for `linux/amd64` and `linux/arm64`.

To build locally:

```bash
docker build -t mowgli-gps         sensors/gps/
docker build -t mowgli-gps-nmea    sensors/gps-nmea/
docker build -t mowgli-lidar-ld19  --target runtime sensors/lidar-ldlidar/
```
