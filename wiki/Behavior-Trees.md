# Behavior Trees

MowgliNext uses BehaviorTree.CPP v4 for reactive, composable robot control.

## Overview

- **Tick rate:** 10 Hz
- **Root:** ReactiveSequence (restarts on child failure)
- **Reactive guards:** Emergency, boundary violation, GPS mode — checked every tick before main logic
- **Tree file:** `ros2/src/mowgli_behavior/trees/main_tree.xml`

## High-Level States (HighLevelStatus.msg)

| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `HIGH_LEVEL_STATE_NULL` | Emergency or transitional |
| 1 | `HIGH_LEVEL_STATE_IDLE` | Idle, docked, or returning home |
| 2 | `HIGH_LEVEL_STATE_AUTONOMOUS` | Autonomous mowing (undocking, transit, mowing, recovering) |
| 3 | `HIGH_LEVEL_STATE_RECORDING` | Area recording in progress |
| 4 | `HIGH_LEVEL_STATE_MANUAL_MOWING` | Manual mowing via teleop |

## Commands (HighLevelControl.srv)

| Value | Constant | Description |
|-------|----------|-------------|
| 1 | `COMMAND_START` | Begin autonomous mowing |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` | Start area boundary recording |
| 4 | `COMMAND_S2` | Mow next area |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, simplify and save polygon |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual mowing mode |
| 254 | `COMMAND_RESET_EMERGENCY` | Reset latched emergency |
| 255 | `COMMAND_DELETE_MAPS` | Delete all maps |

## Tree Structure

```
Root (ReactiveSequence) — re-evaluates all children every tick
│
├── EmergencyGuard
│   ├── Inverter(IsEmergency) → continue if safe
│   └── EmergencyHandler
│       ├── SetMowerEnabled(false), StopMoving()
│       ├── PublishHighLevelStatus(EMERGENCY)
│       └── AutoResetOrWait
│           ├── IsCharging → ResetEmergency (auto-reset on dock)
│           └── WaitForDuration(1s) (retry if not on dock)
│
├── BoundaryGuard
│   ├── Inverter(IsBoundaryViolation) → continue if inside
│   └── Stop, BackUp (5 attempts), emergency stop if still outside
│
├── GPSModeSelector
│   ├── IsGPSFixed → SetNavMode(precise)
│   └── SetNavMode(degraded)
│
└── MainLogic (Fallback — priority order)
    ├── CriticalBatteryDock (< 10%) → save, dock, clear command
    │
    ├── MowingSequence (COMMAND_START = 1)
    │   ├── Undock (GPS wait, RTAB-Map save, heading calibration)
    │   ├── Strip coverage loop (reactive rain/battery guards)
    │   │   ├── GetNextStrip → TransitToStrip → FollowStrip
    │   │   └── Recovery: backup, clear costmap, retry
    │   └── Complete → disable blade, save, dock
    │
    ├── HomeSequence (COMMAND_HOME = 2) → save, dock
    │
    ├── RecordingSequence (COMMAND_RECORD_AREA = 3)
    │   ├── RecordArea (trajectory at 2 Hz, live preview)
    │   ├── Finish (cmd 5) → Douglas-Peucker simplify → save polygon
    │   └── Cancel (cmd 6) → discard
    │
    ├── ManualMowingSequence (COMMAND_MANUAL_MOW = 7)
    │   ├── Teleop via /cmd_vel_teleop (twist_mux priority)
    │   ├── Blade managed by GUI (fire-and-forget to firmware)
    │   └── Collision_monitor, GPS, SLAM all remain active
    │
    └── IdleSequence (default) → disable blade, stop, wait
```

## Node Types

### Condition Nodes
- `IsEmergency` — checks emergency stop state (with staleness detection)
- `IsCharging` — dock charging state
- `IsBatteryLow` — configurable voltage threshold
- `IsBatteryAbove` — checks battery percent above threshold (for charge-to-95%)
- `NeedsDocking` — battery voltage below threshold (default 20.0 V)
- `IsRainDetected` — rain sensor active
- `IsNewRain` — new rain onset (not if raining at mow start)
- `IsGPSFixed` — GPS RTK fix quality check
- `IsCommand` — matches current high-level command from GUI
- `IsBoundaryViolation` — robot outside mowing area boundary
- `IsResumeUndockAllowed` — tracks resume-undock attempts (max_attempts)
- `IsChargingProgressing` — charger active and battery increasing
- `ReplanNeeded` — coverage replanning required

### Action Nodes
- `NavigateToPose` — Nav2 navigate_to_pose async action
- `DockRobot` / `UndockRobot` — opennav_docking actions
- `SetMowerEnabled` — fire-and-forget blade commands via `/mower_control`
- `StopMoving` — publishes zero Twist to `/cmd_vel`
- `BackUp` — drives robot backward (configurable distance/speed)
- `ClearCostmap` — clears Nav2 local costmap
- `PublishHighLevelStatus` — publishes state + state_name to HighLevelStatus topic
- `SaveSlamMap` — triggers RTAB-Map database save (rtabmap.db) before docking
- `SaveObstacles` — persists tracked obstacles to disk
- `SetNavMode` — switches between precise/degraded navigation
- `ClearCommand` — clears pending high-level command
- `WaitForDuration` — timed wait
- `RecordUndockStart` / `CalibrateHeadingFromUndock` — heading calibration from EKF TF after undock
- `WasRainingAtStart` — records rain state at mow start
- `RecordResumeUndockFailure` — tracks resume failures
- `ResetEmergency` — calls `/emergency_stop` with emergency=false (firmware decides whether to clear)

### Coverage Nodes (cell-based strip-by-strip)
- `GetNextStrip` — fetches next unvisited strip from `map_server_node ~/get_next_strip`
- `TransitToStrip` — navigates to strip start (RPP controller)
- `FollowStrip` — follows strip path (FTCController, <10mm lateral accuracy)

### Recording Nodes
- `RecordArea` — records robot trajectory while user drives boundary, Douglas-Peucker simplification, saves polygon via `/map_server_node/add_area`. Publishes live trajectory preview on `~/recording_trajectory`. Listens for finish (cmd 5) or cancel (cmd 6).
  - Ports: `simplification_tolerance` (0.2m), `min_vertices` (3), `min_area` (1.0 m^2), `record_rate_hz` (2.0), `is_exclusion_zone` (false)

## Adding a New BT Node

1. Define the node class in `ros2/src/mowgli_behavior/include/`
2. Implement in `ros2/src/mowgli_behavior/src/`
3. Register in `ros2/src/mowgli_behavior/src/register_nodes.cpp`
4. Use in `main_tree.xml`

## Configuration

Behavior tree parameters are in `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`:

```yaml
behavior_tree_node:
  ros__parameters:
    battery_low_voltage: 23.0
    battery_resume_voltage: 25.0
    rain_delay_sec: 300
    stuck_timeout_sec: 30
    gps_wait_timeout_sec: 30
```

See [Configuration](Configuration) for the full parameter reference.
