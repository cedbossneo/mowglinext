# Behavior Trees

MowgliNext uses BehaviorTree.CPP v4 for reactive, composable robot control.

## Overview

- **Tick rate:** 10 Hz
- **Root:** ReactiveSequence (restarts on child failure)
- **Emergency guard:** `IsEmergency` checked first every tick
- **Tree file:** `ros2/src/mowgli_behavior/trees/main_tree.xml`

## Tree Structure

```
Root (ReactiveSequence)
├── IsEmergency → EmergencyStop
├── IsBatteryLow → NavigateToDock → DockRobot
├── IsRaining → NavigateToDock → DockRobot
└── MainSequence
    ├── WaitForGPS
    ├── UndockRobot
    ├── NavigateToArea
    ├── StartBlade
    ├── ExecuteCoverage
    ├── StopBlade
    └── NavigateToDock → DockRobot
```

## Node Types

### Condition Nodes
- `IsEmergency` — checks emergency stop state (with staleness detection)
- `IsBatteryLow` — configurable voltage threshold with hysteresis
- `IsRaining` — rain sensor with configurable delay
- `IsCharging` — dock charging state
- `HasGPSFix` — GPS quality check (RTK fix required)

### Action Nodes
- `NavigateToDock` / `NavigateToArea` — Nav2 navigate_to_pose
- `DockRobot` / `UndockRobot` — opennav_docking actions
- `StartBlade` / `StopBlade` — fire-and-forget blade commands
- `ExecuteCoverage` — runs F2C coverage plan through Nav2
- `SaveSLAMMap` — persists SLAM map before docking

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
