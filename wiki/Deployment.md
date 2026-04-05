# Deployment

Docker Compose deployment for MowgliNext.

## Deployment Modes

### Standard (All-in-One)

All services on one board:

```bash
cd docker
docker compose up -d
```

Services: `mowgli` (ROS2 full stack), `gps`, `lidar`, `gui`, `mosquitto`, `watchtower`

### Simulation

For testing without hardware — see the [Simulation](Simulation) page for full details:

```bash
cd docker

# Headless (CI / automated testing)
docker compose -f docker-compose.simulation.yaml up simulation

# Development with live config mounts
docker compose -f docker-compose.simulation.yaml up dev-sim

# GUI via noVNC browser
docker compose -f docker-compose.simulation.yaml up simulation-gui
```

## Container Architecture

| Container | Image | Purpose |
|-----------|-------|---------|
| `mowgli` | `mowgli-ros2` | Full ROS2 stack (Nav2, SLAM, BT, coverage) |
| `mowgli-gps` | `sensors/gps` | u-blox driver + NTRIP RTK |
| `mowgli-lidar` | `sensors/lidar` | LD19 LiDAR driver |
| `gui` | `openmower-gui` | Web interface (Go + React) |
| `mosquitto` | `eclipse-mosquitto` | MQTT broker |
| `watchtower` | `containrrr/watchtower` | Auto-updates (GUI image) |

## DDS Configuration

All ROS2 containers use **Cyclone DDS** with shared config and localhost-only discovery:

```xml
<!-- docker/config/cyclonedds.xml -->
<CycloneDDS>
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true"/>
      </Interfaces>
    </General>
    <Discovery>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

All containers share this environment for DDS:

```yaml
ROS_DOMAIN_ID: 0
RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
CYCLONEDDS_URI: file:///cyclonedds.xml
ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST
```

## Updating

Images auto-update via Watchtower, or manually:

```bash
docker compose pull
docker compose up -d
```

## Troubleshooting

See the [full troubleshooting section](https://github.com/cedbossneo/mowglinext/blob/main/docker/README.md#troubleshooting) in the Docker README.

Common issues:
- **DDS discovery fails** — check `ROS_DOMAIN_ID` matches across containers
- **GPS no fix** — verify NTRIP credentials and serial device
- **LiDAR no /scan** — check UART baud rate and device path
- **Nav2 timeout** — increase startup delay for ARM boards
