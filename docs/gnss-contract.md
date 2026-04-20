# GNSS Backend Contract

`feat/gps` defines the common external GNSS interface for backend-specific
compose fragments.

The goal is to let different GNSS backends expose the same ROS 2 surface so
downstream consumers stay backend-agnostic.

## Standard Topics

Every GNSS backend selected by compose should publish the same external topics:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/gnss/fix` | `sensor_msgs/msg/NavSatFix` | Primary GNSS position output |
| `/gnss/azimuth` | `compass_msgs/msg/Azimuth` | GNSS-derived heading/azimuth when available |
| `/gnss/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | GNSS backend health and data freshness |

## Common Parameter Vocabulary

GNSS backends should use this parameter vocabulary for their external runtime
configuration:

| Parameter | Meaning |
| --- | --- |
| `port` | Device path used by the backend |
| `baudrate` | Serial baud rate when applicable |
| `frame_id` | Frame ID used in published GNSS messages |
| `data_timeout_sec` | Maximum age before data is considered stale |
| `reconnect_interval_sec` | Retry interval when the device is unavailable |

Backend-specific parameters may still exist, but these names should be shared
across GNSS backends.

## Expected Backend Behavior

- Backend selection happens at the compose level.
- `docker-compose.ublox.yaml` should implement this contract for the u-blox path.
- `docker-compose.unicore.yaml` should implement this contract for the Unicore path.
- MAVROS remains independent and is not required to conform to this GNSS backend contract.
- Backend internals may remain vendor-specific, but the external ROS 2 interface should stay on `/gnss/*`.

## Compatibility Note

Current compatibility behavior remains unchanged on `feat/gps`.

Some existing launch files and containers still bridge legacy GPS topic names,
such as `/gps/fix`, into the newer `/gnss/*` abstraction boundary. That
compatibility layer stays in place for now so backend convergence can happen in
the dedicated driver branches without forcing a broad refactor here.
