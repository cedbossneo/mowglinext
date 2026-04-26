# ROS2 Project Rules

> These rules are specific to the MowgliNext ROS2 stack. They extend the common rules with ROS2/robotics conventions.

## Node Patterns

- Use rclcpp lifecycle nodes for anything managed by Nav2's lifecycle manager
- Use regular rclcpp::Node for standalone nodes (hardware_bridge, behavior_tree)
- Declare ALL parameters in the constructor — no undeclared parameter access
- Use `declare_parameter<T>()` with default values, never `get_parameter()` without prior declaration

## QoS Profiles

- Sensor data (IMU, LiDAR, GPS): `rclcpp::SensorDataQoS()` (best effort, volatile)
- Commands (cmd_vel): `rclcpp::SystemDefaultsQoS()` (reliable)
- Status/diagnostics: `rclcpp::QoS(10)` (reliable, depth 10)
- TF: use `tf2_ros` defaults — never override TF QoS

## Topic Naming

- Node-internal topics: use relative names (`~/status`, `~/cmd_vel`)
- Remap to `/mowgli/` namespace in launch files, not in C++ source
- Shared system topics: absolute (`/scan`, `/cmd_vel`, `/odometry/filtered_map`)

## Launch Files

- Python launch files only (`.launch.py`), not XML or YAML launch
- Load parameters from YAML files via `os.path.join(get_package_share_directory(...), 'config', '...')`
- Use `LaunchConfiguration` for runtime arguments, with sensible defaults

## Testing

- Use `ament_cmake_gtest` for C++ unit tests
- Use `launch_testing` for integration tests
- Test coverage target: 80% for new packages
- All BT nodes must have unit tests for tick() behavior

## Build System

- `ament_cmake` for C++ packages
- `ament_python` for pure Python packages (launch files only)
- Always specify dependencies in both `CMakeLists.txt` AND `package.xml`
- Use `find_package()` for build deps, `<depend>` in package.xml for runtime

## Safety Rules

- Blade commands are fire-and-forget — firmware decides execution
- Emergency stop is firmware-level, not software-level
- Never disable collision_monitor on real hardware
- GPS wait timeout must exist before undock (avoid blind navigation)
- Battery voltage thresholds must have hysteresis (low != resume)

## Docker / ARM Considerations

- Test Docker builds for both `linux/amd64` and `linux/arm64`
- Avoid composition mode on ARM (causes crashes) — use separate processes
- Increase Nav2 startup delays on ARM (30s minimum)
- Cyclone DDS only — FastRTPS has stale shm issues on ARM
