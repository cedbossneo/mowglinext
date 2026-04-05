# GUI

OpenMower GUI — React frontend + Go backend for mower configuration and monitoring.

## Access

Default: `http://<mower-ip>:4006`

## Features

- Map editor — define mowing areas, navigation areas, dock position
- Robot footprint visualization on map
- Per-model chassis dimensions
- Real-time status monitoring
- Configuration editor (writes `mowgli_robot.yaml`)
- Firmware flashing via web UI

## Architecture

- **Frontend:** React (TypeScript), served from `/app/web`
- **Backend:** Go, connects to ROS2 via rosbridge (ws://localhost:9090)
- **MQTT:** Publishes/subscribes to status topics via Mosquitto

## Development

```bash
cd gui

# Backend
go build -o openmower-gui
./openmower-gui

# Frontend
cd web
yarn install
yarn dev
```

## Docker

```bash
docker build -t openmower-gui gui/
```

## Configuration

The GUI reads and writes `docker/config/mowgli/mowgli_robot.yaml`. Changes take effect after restarting the `mowgli` container.
