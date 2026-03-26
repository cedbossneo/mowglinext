# Simulation Guide

This guide explains how to run the Mowgli ROS2 system in Gazebo Ignition simulation for testing and development without physical hardware.

## Overview

The simulation provides:

- **Virtual Mowgli robot** in a realistic Gazebo Ignition environment
- **Simulated sensors:** LiDAR (2D laser scan), IMU (accelerometer + gyroscope), wheel odometry
- **Physics simulation:** Motor torques, friction, collision detection
- **ROS2 bridging:** Full integration with the ROS2 navigation stack
- **Repeatable scenarios:** Consistent environment for testing mowing patterns and navigation

**What's NOT simulated:**
- RTK-GPS (can be mocked via GPS pose converter)
- Battery drain / charging
- Grass cutting blade physics (motor control still available)
- Weathering / seasonal map changes

## Requirements

### Software
- **ROS2 Humble** (latest, with `ament_cmake`)
- **Gazebo Ignition Fortress** or newer
- **ros_gz_sim** and **ros_gz_bridge** packages
- **mowgli_ros2** source (fully built)

### System
- **CPU:** Multi-core processor (8+ cores recommended)
- **RAM:** 4 GB minimum, 8 GB recommended
- **GPU:** Optional (Ignition rendering faster with GPU, CPU fallback available)
- **Disk:** ~500 MB for Gazebo installation

### Installation

```bash
# Install Gazebo Ignition (Ubuntu 22.04)
sudo apt-get update
sudo apt-get install ignition-fortress

# Install ROS2 Gazebo bridges
sudo apt-get install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge

# Verify installation
ign gazebo --help
ros2 launch ros_gz_sim gz_sim.launch.py
```

## Quick Start

### 1. Build the Project

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select mowgli_bringup mowgli_hardware mowgli_localization mowgli_nav2_plugins mowgli_behavior
source install/setup.bash
```

### 2. Launch Simulation

```bash
ros2 launch mowgli_bringup simulation.launch.py
```

**First launch will take 30–60 seconds** (Gazebo initialization, plugin loading).

**Console output:**
```
[robot_state_publisher-1] [INFO] Starting robot state publisher
[gazebo-1] [INFO] Starting gazebo server
[spawn_mowgli-1] [INFO] Spawning model 'mowgli' at pose (0.0, 0.0, 0.1)
[gz_ros2_bridge-1] [INFO] Bridging ROS 2 <-> Gazebo
[joint_state_publisher-1] [INFO] Publishing joint states for wheels
```

**Window opens with 3D simulation view (Gazebo GUI).**

### 3. Verify Topics are Publishing

In a new terminal:

```bash
source ~/ros2_ws/install/setup.bash

# Check sensor topics
ros2 topic list | grep -E "scan|imu|odom"
# Expected output:
#   /scan
#   /imu/data
#   /wheel_odom
#   /odometry/filtered_odom
#   /odometry/filtered_map
#   /tf

# Verify data is flowing
ros2 topic echo /scan --no-arr (limit output)
ros2 topic echo /odometry/filtered_odom --no-arr
```

### 4. Send Navigation Goal

From a third terminal:

```bash
source ~/ros2_ws/install/setup.bash

# Bring up Nav2 (in background)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true &

# Wait a few seconds for Nav2 to initialize...
sleep 3

# Send a goal (relative to spawning position)
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**Robot should move toward the goal in the Gazebo window.**

## Launch File Arguments

### Customize Spawn Position

```bash
ros2 launch mowgli_bringup simulation.launch.py \
    spawn_x:=1.0 \
    spawn_y:=2.0 \
    spawn_z:=0.1 \
    spawn_yaw:=0.785  # 45 degrees
```

### Use Custom World

```bash
ros2 launch mowgli_bringup simulation.launch.py \
    world:=/path/to/custom_world.sdf
```

### Disable GUI (Headless Mode)

```bash
ros2 launch mowgli_bringup simulation.launch.py \
    headless:=true
```

**Useful for CI/CD testing or running on remote servers.**

## Gazebo Keyboard Controls

While the Gazebo window is active:

| Key | Function |
|-----|----------|
| `G` | Toggle grid |
| `T` | Toggle collision shapes (debug) |
| `V` | Cycle camera view |
| `O` | Show odometry (IMU, wheel ticks) |
| `?` | Show all hotkeys |
| Scroll | Zoom in/out |
| Right-click drag | Pan camera |
| Middle-click drag | Rotate view |

## RViz2 Visualization

Run RViz in parallel to visualize the ROS2 side:

```bash
# Terminal 1: Launch simulation
ros2 launch mowgli_bringup simulation.launch.py

# Terminal 2: Launch RViz
rviz2 -d src/mowgli_bringup/config/mowgli.rviz
```

**RViz displays:**
- `/map` (SLAM-generated map, if SLAM is running)
- `/scan` (LiDAR point cloud)
- `/tf_tree` (robot transform tree)
- `/odometry/filtered_map` (robot pose estimate)
- `/cmd_vel` (velocity commands)
- Robot model with joint states

**You can now:**
- Use RViz "Nav2 Goal" tool to send goals with the mouse
- Monitor localization convergence
- Debug SLAM loop closures
- Track path planning in real-time

## Common Workflows

### Workflow 1: Test Navigation Stack

```bash
# Terminal 1: Simulation
ros2 launch mowgli_bringup simulation.launch.py

# Terminal 2: RViz for visualization
rviz2 -d src/mowgli_bringup/config/mowgli.rviz

# Terminal 3: Launch Nav2 + SLAM (optional)
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# Terminal 4: Send goals from RViz (click "Nav2 Goal" in RViz toolbar)
# or manually via:
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0}, orientation: {w: 1.0}}}"
```

### Workflow 2: Test BehaviorTree Control

```bash
# Terminal 1: Simulation
ros2 launch mowgli_bringup simulation.launch.py

# Terminal 2: RViz
rviz2 -d src/mowgli_bringup/config/mowgli.rviz

# Terminal 3: Launch full stack (Nav2 + BehaviorTree)
ros2 launch mowgli_bringup mowgli.launch.py use_sim_time:=true

# Terminal 4: Trigger mowing behavior
ros2 service call /mowgli_behavior/set_mode std_srvs/SetBool "{data: true}"

# Monitor behavior in another terminal:
ros2 topic echo /hardware_bridge/status
```

### Workflow 3: SLAM Mapping

```bash
# Terminal 1: Simulation
ros2 launch mowgli_bringup simulation.launch.py

# Terminal 2: Bring up SLAM (maps during navigation)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3: RViz
rviz2 -d src/mowgli_bringup/config/mowgli.rviz

# Terminal 4: Send goals to drive around and build map
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}"

# Save map after exploration
ros2 run nav2_map_server map_saver_cli -f ~/mowgli_map
```

## Simulated Topics

### Gazebo → ROS2 (Bridged via ros_gz_bridge)

| Topic | Type | Publisher | Rate | Description |
|-------|------|-----------|------|-------------|
| `/clock` | rosgraph_msgs/Clock | Gazebo | 1000 Hz | Simulation time (use_sim_time) |
| `/scan` | sensor_msgs/LaserScan | gz_bridge | 5 Hz | Virtual 2D LiDAR scan |
| `/imu/data` | sensor_msgs/Imu | gz_bridge | 100 Hz | IMU accelerometer + gyro |
| `/wheel_odom` | nav_msgs/Odometry | gz_bridge | 50 Hz | Wheel encoder + odometry |
| `/cmd_vel` | geometry_msgs/Twist | (ROS → Gazebo) | – | Motor velocity commands |

### ROS2 → Gazebo (Hardware bridge simulation)

In the actual deployment, `/cmd_vel` goes to hardware_bridge → STM32 → motors.

In simulation, `/cmd_vel` is bridged directly to Gazebo physics for wheel actuation.

**No hardware_bridge node needed in simulation** (launch files handle this automatically).

### Standard ROS2 Topics (Same as Hardware)

| Topic | Type | Purpose |
|-------|------|---------|
| `/odometry/filtered_odom` | nav_msgs/Odometry | Fused wheel + IMU odometry (EKF) |
| `/odometry/filtered_map` | nav_msgs/Odometry | Fused odometry + GPS (EKF) |
| `/tf` | – | Transform tree (map → odom → base_link) |

## Gazebo World Description

### World SDF Structure

The simulation uses an SDF (Simulation Description Format) world file.

**Default world:** `empty.sdf` (minimal environment)

**Example custom world (mowgli_world.sdf):**

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="mowgli_yard">
    <!-- Physics engine -->
    <physics name="default" type="dart">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Lighting -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
    </scene>

    <!-- Terrain (grass) -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.2 0.5 0.2 1</ambient>  <!-- Green -->
            <diffuse>0.2 0.5 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle: wall -->
    <model name="obstacle_wall">
      <static>true</static>
      <pose>10 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 5 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

To use:
```bash
ros2 launch mowgli_bringup simulation.launch.py world:=mowgli_world.sdf
```

## Troubleshooting

### Issue 1: "Gazebo is taking forever to start"

**Cause:** First launch loads plugins and shaders.

**Solution:** Be patient (30–60 seconds normal). Subsequent launches are faster.

### Issue 2: "/cmd_vel not received by Gazebo"

**Cause:** ros_gz_bridge not properly bridging command topics.

**Check:**
```bash
# Verify bridge is running
ros2 node list | grep gz

# Check bridge topics
ros2 topic list | grep cmd_vel
```

**Fix:** Restart the bridge:
```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```

### Issue 3: "Simulation time not synchronized (use_sim_time=false)"

**Cause:** `/clock` bridge not active.

**Check:**
```bash
ros2 topic echo /clock --no-arr
# Should show timestamps, not "No messages"
```

**Fix:** Ensure `use_sim_time: true` is set in launch arguments.

### Issue 4: "Robot doesn't move when I send cmd_vel"

**Cause:** Motor control plugin not properly configured, or physics settings too high friction.

**Check:**
```bash
# Monitor what Gazebo is receiving
ros2 topic echo /cmd_vel
# Should show velocity commands

# Monitor actual odometry
ros2 topic echo /wheel_odom
# Should show changing position
```

**Fix:** Check Gazebo physics settings (increase max_step_size, reduce friction):

```yaml
# In world SDF
<physics>
  <max_step_size>0.01</max_step_size>
  <friction>
    <coefficient>0.1</coefficient>
  </friction>
</physics>
```

### Issue 5: "RViz can't find /map or /scan"

**Cause:** SLAM not running, or bridge not bridging laser data.

**Solution:**
- Start SLAM: `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true`
- Verify /scan is publishing: `ros2 topic echo /scan --no-arr`
- In RViz, check fixed frame and add tf plugin

## Integration Testing Examples

### Test 1: Drive in Square Pattern

```bash
# Terminal 1: Simulation
ros2 launch mowgli_bringup simulation.launch.py

# Terminal 2: Send goals in sequence
for i in {1..4}; do
  x=$((i * 2))
  y=0
  echo "Goal $i: ($x, $y)"
  ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
    "pose: {header: {frame_id: 'map'}, pose: {position: {x: $x, y: 0}, orientation: {w: 1.0}}}"
  sleep 10
done
```

### Test 2: Measure Localization Drift

```bash
# Terminal 1: Simulation
ros2 launch mowgli_bringup simulation.launch.py

# Terminal 2: Record odometry
ros2 bag record -o test_drift /odometry/filtered_odom /wheel_odom /tf --duration 60

# Terminal 3: Send goal and let robot drive
ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
  "pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 10.0}, orientation: {w: 1.0}}}"

# Post-analysis:
# Compare wheel_odom vs. filtered_odom drift over time
ros2 bag info test_drift_0/
```

### Test 3: SLAM Accuracy

```bash
# Terminal 1: Simulation with SLAM
ros2 launch mowgli_bringup simulation.launch.py
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &

# Terminal 2: RViz
rviz2 -d src/mowgli_bringup/config/mowgli.rviz

# Terminal 3: Drive a large loop (should close with <0.5 m error)
# Use RViz goal tool or script to send goals forming a loop
# Observe /map in RViz for loop closure artifact

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/mowgli_test_map
```

## Performance Tuning

### Reduce CPU Usage

```bash
# Disable GUI (headless)
ros2 launch mowgli_bringup simulation.launch.py headless:=true

# Reduce sensor frequencies (in config files)
# E.g., /scan from 5 Hz → 1 Hz
# Reduces bridge overhead
```

### Faster Real-time Factor

Real-time factor = simulation speed / wall clock time.

Target: 1.0 or higher (simulation runs faster than real-time).

**Tuning:**
```bash
# In launch file, increase max_step_size
# But be careful: too large → physics instability

# Typical: 0.001 s (1 ms) per step for stable dynamics
# Can increase to 0.005 s if collision detection is less critical
```

## Docker Deployment

Simulate in a Docker container (useful for CI/CD):

```bash
# Build Docker image
docker compose build simulation

# Run simulation (headless)
docker compose run --rm simulation \
  ros2 launch mowgli_bringup simulation.launch.py headless:=true
```

**docker-compose.yml (example):**
```yaml
services:
  simulation:
    build:
      context: .
      dockerfile: Dockerfile
    environment:
      - ROS_DOMAIN_ID=0
      - ROS_LOCALHOST_ONLY=1
    volumes:
      - ./src:/workspace/src
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 launch mowgli_bringup simulation.launch.py headless:=true
      "
```

## Next Steps

- **[CONFIGURATION.md](CONFIGURATION.md)** – Tune navigation and localization parameters
- **[ARCHITECTURE.md](ARCHITECTURE.md)** – Understand the full system design
- **Real Hardware** – Follow [README.md](../README.md) to deploy on actual Mowgli robot

---

**Happy simulating!** 🤖
