# =============================================================================
# Stage 1: base
# Installs all runtime ROS2 dependencies. Both runtime and simulation stages
# inherit from here, keeping the image lean.
# =============================================================================
FROM ros:jazzy-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Navigation
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-velocity-smoother \
    # SLAM and localization
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    # Motion control
    ros-jazzy-twist-mux \
    # Behavior trees
    ros-jazzy-behaviortree-cpp \
    # Robot description / TF
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    # Grid map
    ros-jazzy-grid-map-core \
    ros-jazzy-grid-map-ros \
    ros-jazzy-grid-map-msgs \
    # Diagnostics
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-diagnostic-updater \
    # Foxglove Bridge (binary WebSocket bridge for Foxglove Studio)
    ros-jazzy-foxglove-bridge \
    # Rosbridge (WebSocket bridge for openmower-gui)
    ros-jazzy-rosbridge-suite \
    # u-blox and RTCM message definitions (for foxglove_bridge schema resolution)
    ros-jazzy-ublox-msgs \
    ros-jazzy-nmea-msgs \
    ros-jazzy-rtcm-msgs \
    # tinyxml2 (required at runtime by BehaviorTree.CPP)
    libtinyxml2-dev \
    # DDS middleware — Cyclone DDS required for all containers
    ros-jazzy-rmw-cyclonedds-cpp \
    # Topic tools (relay, throttle — used in simulation for topic remapping)
    ros-jazzy-topic-tools \
    # Docking (opennav_docking — dock/undock action server)
    # DockRobot/UndockRobot actions are in nav2_msgs (already installed).
    # The docking_server executable is in opennav_docking.
    ros-jazzy-opennav-docking \
    # Rotation shim controller for transit navigation
    ros-jazzy-nav2-rotation-shim-controller \
    # Nav2 core headers (FTC controller plugin depends on nav2_core)
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-util \
    # Nav2 msgs (CostmapFilterInfo for costmap filters)
    ros-jazzy-nav2-msgs \
    # Coverage planning (F2C v1.2.1 built from source in deps stage)
    # ros-jazzy-fields2cover -- removed: we build F2C v2.0.0 from source
    libgdal-dev \
    # Simulation bridge (needed at runtime for ros_gz_bridge topic bridging)
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    # Utilities
    python3-argcomplete \
 && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Stage 2: deps
# Installs build tools and resolves rosdep dependencies. Cached as long as
# package.xml files don't change.
# =============================================================================
FROM base AS deps

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-ament-cmake \
    ros-jazzy-rosidl-default-generators \
    build-essential \
    cmake \
    git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# ---- Fields2Cover v2.0.0 from source -----------------------------------------
# v2 adds cell decomposition for irregular gardens, OR-tools route optimization,
# and better obstacle handling. opennav_coverage is no longer used — the project
# calls F2C directly from coverage_planner_node.
RUN git clone --branch main --depth 1 \
      https://github.com/Fields2Cover/Fields2Cover.git /tmp/f2c && \
    cd /tmp/f2c && mkdir build && cd build && \
    cmake .. \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DBUILD_PYTHON=OFF && \
    make -j"$(nproc)" && make install && ldconfig && \
    rm -rf /tmp/f2c

# Copy only package.xml + CMakeLists.txt to resolve rosdep deps (cache layer).
# This ensures the expensive rosdep install is only re-run when package
# definitions change, NOT when source code changes.
COPY src/mowgli_interfaces/package.xml    src/mowgli_interfaces/package.xml
COPY src/mowgli_interfaces/CMakeLists.txt src/mowgli_interfaces/CMakeLists.txt
COPY src/mowgli_hardware/package.xml      src/mowgli_hardware/package.xml
COPY src/mowgli_hardware/CMakeLists.txt   src/mowgli_hardware/CMakeLists.txt
COPY src/mowgli_localization/package.xml  src/mowgli_localization/package.xml
COPY src/mowgli_localization/CMakeLists.txt src/mowgli_localization/CMakeLists.txt
COPY src/mowgli_map/package.xml           src/mowgli_map/package.xml
COPY src/mowgli_map/CMakeLists.txt        src/mowgli_map/CMakeLists.txt
COPY src/mowgli_coverage_planner/package.xml src/mowgli_coverage_planner/package.xml
COPY src/mowgli_coverage_planner/CMakeLists.txt src/mowgli_coverage_planner/CMakeLists.txt
COPY src/mowgli_nav2_plugins/package.xml  src/mowgli_nav2_plugins/package.xml
COPY src/mowgli_nav2_plugins/CMakeLists.txt src/mowgli_nav2_plugins/CMakeLists.txt
COPY src/mowgli_behavior/package.xml      src/mowgli_behavior/package.xml
COPY src/mowgli_behavior/CMakeLists.txt   src/mowgli_behavior/CMakeLists.txt
COPY src/mowgli_monitoring/package.xml    src/mowgli_monitoring/package.xml
COPY src/mowgli_monitoring/CMakeLists.txt src/mowgli_monitoring/CMakeLists.txt
COPY src/mowgli_simulation/package.xml    src/mowgli_simulation/package.xml
COPY src/mowgli_simulation/CMakeLists.txt src/mowgli_simulation/CMakeLists.txt
COPY src/mowgli_bringup/package.xml       src/mowgli_bringup/package.xml
COPY src/mowgli_bringup/CMakeLists.txt    src/mowgli_bringup/CMakeLists.txt

# Resolve rosdep dependencies (cached until package.xml changes)
RUN rosdep update --rosdistro jazzy \
 && rosdep install \
      --from-paths src \
      --ignore-src \
      --rosdistro jazzy \
      --skip-keys "fields2cover" \
      -y \
 || true

# =============================================================================
# Stage 3: build-interfaces
# Build only mowgli_interfaces (msg/srv/action definitions). These rarely
# change and are a dependency for all other packages.
# =============================================================================
FROM deps AS build-interfaces

# Copy the full mowgli_interfaces package
COPY src/mowgli_interfaces/ src/mowgli_interfaces/

RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    colcon build \
      --packages-select mowgli_interfaces \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers \$(nproc) \
    "

# =============================================================================
# Stage 4: build
# Compiles all remaining workspace packages from source.
# =============================================================================
FROM build-interfaces AS build

ARG BUILD_TYPE=Release

# Now copy the rest of the source tree
COPY src/ src/

# Build all packages (mowgli_interfaces is already built and cached)
RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source install/setup.bash && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DBUILD_TESTING=OFF -DFields2Cover_DIR=/usr/local/cmake/fields2cover \
      --parallel-workers \$(nproc) \
      --event-handlers console_cohesion+ \
    "

# Run unit tests (non-blocking)
RUN /bin/bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source install/setup.bash && \
    colcon test \
      --ctest-args -L 'gtest' --output-on-failure \
    ; colcon test-result --verbose \
    " || true

# =============================================================================
# Stage 5: runtime
# Minimal image containing only what is needed to run the robot.
# =============================================================================
FROM base AS runtime

WORKDIR /ros2_ws

# Pull Fields2Cover and all its runtime dependencies from build stage
# (needed at runtime by coverage_planner_node)
COPY --from=build /usr/local/lib/libFields2Cover* /usr/local/lib/
COPY --from=build /usr/local/lib/libsteering_functions* /usr/local/lib/
COPY --from=build /usr/local/lib/libortools* /usr/local/lib/
COPY --from=build /usr/local/lib/libmatplot* /usr/local/lib/
COPY --from=build /usr/local/include/fields2cover* /usr/local/include/

# Ensure libtinyxml2 .so.9 and .so.10 both resolve. The build stage may
# compile against a different minor version than the runtime base provides
# when Docker layer caching spans a base image update.
RUN reallib=$(find /usr/lib -name 'libtinyxml2.so.*.*.*' | head -1) && \
    dir=$(dirname "$reallib") && \
    [ ! -e "$dir/libtinyxml2.so.9" ] && ln -sf "$(basename "$reallib")" "$dir/libtinyxml2.so.9" || true && \
    [ ! -e "$dir/libtinyxml2.so.10" ] && ln -sf "$(basename "$reallib")" "$dir/libtinyxml2.so.10" || true && \
    ldconfig

# Pull compiled install tree from the build stage
COPY --from=build /ros2_ws/install/ install/

# Copy all launch files and config from every package
COPY src/mowgli_bringup/launch/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/launch/
COPY src/mowgli_bringup/config/    /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/
COPY src/mowgli_localization/launch/  /ros2_ws/install/mowgli_localization/share/mowgli_localization/launch/
COPY src/mowgli_localization/config/  /ros2_ws/install/mowgli_localization/share/mowgli_localization/config/

# Patch rosbridge_library CBOR serialization bug (Jazzy):
# Fixed-size arrays (float64[36] covariance, uint8[N]) crash the CBOR encoder.
# See: https://github.com/RobotWebTools/rosbridge_suite/issues/965
COPY scripts/patch_rosbridge.py /tmp/patch_rosbridge.py
RUN python3 /tmp/patch_rosbridge.py && rm /tmp/patch_rosbridge.py

# Entrypoint script sources both ROS and workspace setups before exec'ing CMD
COPY scripts/ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "mowgli_bringup", "mowgli.launch.py"]

# =============================================================================
# Stage 6: simulation
# Extends runtime with a full Gazebo installation for headless/web simulation.
# =============================================================================
FROM runtime AS simulation

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic comes via ros-jazzy-ros-gz-sim (Jazzy default pairing)
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    # VNC + lightweight desktop for Gazebo GUI access from macOS
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    openbox \
    xterm \
    x11-utils \
 && rm -rf /var/lib/apt/lists/*

# Copy VNC startup script
COPY scripts/start_vnc.sh /ros2_ws/scripts/start_vnc.sh
RUN chmod +x /ros2_ws/scripts/start_vnc.sh

# Copy Foxglove layout for easy import
COPY foxglove/ /ros2_ws/foxglove/

# Foxglove Bridge WebSocket port
EXPOSE 8765
# noVNC web port (Gazebo GUI via browser)
EXPOSE 6080

CMD ["ros2", "launch", "mowgli_bringup", "sim_full_system.launch.py", "headless:=true", "use_rviz:=false"]
