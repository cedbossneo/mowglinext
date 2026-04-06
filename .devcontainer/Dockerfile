# =============================================================================
# MowgliNext Development Container
#
# All-in-one image for GitHub Codespaces and local devcontainer usage.
# Includes: ROS2 Jazzy, Nav2, Gazebo Harmonic (headless), Fields2Cover v2,
# build tools, and developer utilities.
#
# Gazebo runs headless (CPU rendering) — connect Foxglove Studio to
# ws://localhost:8765 for visualization.
# =============================================================================
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# 1. All ROS2 runtime + build + simulation dependencies in one layer
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ── Navigation ──
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
    ros-jazzy-nav2-rotation-shim-controller \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-util \
    ros-jazzy-nav2-msgs \
    # ── SLAM and localization ──
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    # ── DDS ──
    ros-jazzy-rmw-cyclonedds-cpp \
    # ── Docking ──
    ros-jazzy-opennav-docking \
    # ── Motion control ──
    ros-jazzy-twist-mux \
    ros-jazzy-topic-tools \
    # ── Behavior trees ──
    ros-jazzy-behaviortree-cpp \
    # ── Robot description / TF ──
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    # ── Grid map ──
    ros-jazzy-grid-map-core \
    ros-jazzy-grid-map-ros \
    ros-jazzy-grid-map-msgs \
    # ── Diagnostics ──
    ros-jazzy-diagnostic-msgs \
    ros-jazzy-diagnostic-updater \
    # ── Bridges ──
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rosbridge-suite \
    # ── Message definitions ──
    ros-jazzy-ublox-msgs \
    ros-jazzy-nmea-msgs \
    ros-jazzy-rtcm-msgs \
    # ── Gazebo Harmonic (headless simulation) ──
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    # ── Build tools ──
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-ament-cmake \
    ros-jazzy-rosidl-default-generators \
    build-essential \
    cmake \
    git \
    # ── Fields2Cover build dependency ──
    libgdal-dev \
    # ── Runtime deps ──
    libtinyxml2-dev \
    python3-argcomplete \
    # ── Developer tools ──
    gdb \
    htop \
    vim \
    curl \
    wget \
    bash-completion \
    python3-pip \
    clang-format \
    # ── GitHub CLI ──
    gpg \
 && rm -rf /var/lib/apt/lists/* \
 && curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
      | gpg --dearmor -o /usr/share/keyrings/githubcli-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
      > /etc/apt/sources.list.d/github-cli.list \
 && apt-get update && apt-get install -y --no-install-recommends gh \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Node.js 22 (needed for app/web, app/mobile, app/common)
# ---------------------------------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. Fields2Cover v2.0.0 from source
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# 4. rosdep init (needed for resolving deps on first build)
# ---------------------------------------------------------------------------
RUN rosdep update --rosdistro jazzy || true

# ---------------------------------------------------------------------------
# 5. Workspace directory
# ---------------------------------------------------------------------------
WORKDIR /ros2_ws

# ---------------------------------------------------------------------------
# 6. Environment setup — auto-source ROS2 + workspace in every shell
# ---------------------------------------------------------------------------
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /etc/bash.bashrc && \
    echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /etc/bash.bashrc

# ---------------------------------------------------------------------------
# 7. Cyclone DDS config (localhost-only discovery for containers)
# ---------------------------------------------------------------------------
COPY docker/config/cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
