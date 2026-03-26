# =============================================================================
# Stage 1: base
# Installs all runtime ROS2 dependencies. Both runtime and simulation stages
# inherit from here, keeping the image lean.
# =============================================================================
FROM ros:humble-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Navigation
    ros-humble-nav2-bringup \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-velocity-smoother \
    # SLAM and localization
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    # Motion control
    ros-humble-twist-mux \
    # Behavior trees
    ros-humble-behaviortree-cpp \
    # Robot description / TF
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    # Simulation bridge (needed at runtime for ros_gz_bridge topic bridging)
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    # Utilities
    python3-argcomplete \
 && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Stage 2: build
# Compiles all workspace packages from source and runs tests.
# =============================================================================
FROM base AS build

ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_TYPE=Release

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    build-essential \
    cmake \
    git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy source tree
COPY src/ src/

# Install any rosdep dependencies not already satisfied by the base stage
RUN rosdep update --rosdistro humble \
 && rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro humble \
      -y \
 || true

# Build workspace
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
      --parallel-workers \$(nproc) \
      --event-handlers console_cohesion+ \
    "

# Run tests (failures cause the image build to fail)
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    colcon test --return-code-on-test-failure && \
    colcon test-result --verbose \
    "

# =============================================================================
# Stage 3: runtime
# Minimal image containing only what is needed to run the robot.
# =============================================================================
FROM base AS runtime

WORKDIR /ros2_ws

# Pull compiled install tree from the build stage
COPY --from=build /ros2_ws/install/ install/

# Copy all launch files and config from every package
COPY src/mowgli_bringup/launch/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/launch/
COPY src/mowgli_bringup/config/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/
COPY src/mowgli_localization/launch/  /ros2_ws/install/mowgli_localization/share/mowgli_localization/launch/
COPY src/mowgli_localization/config/  /ros2_ws/install/mowgli_localization/share/mowgli_localization/config/

# Entrypoint script sources both ROS and workspace setups before exec'ing CMD
COPY scripts/ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "mowgli_bringup", "mowgli.launch.py"]

# =============================================================================
# Stage 4: simulation
# Extends runtime with a full Gazebo installation for headless/web simulation.
# =============================================================================
FROM runtime AS simulation

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Fortress is the LTS pairing for ROS2 Humble
    gz-fortress \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    # Web bridge for Gazebo web client (port 8080)
    libwebsockets-dev \
 && rm -rf /var/lib/apt/lists/*

EXPOSE 8080

CMD ["ros2", "launch", "mowgli_bringup", "simulation.launch.py"]
