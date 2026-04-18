# =============================================================================
# MowgliNext Development Container
#
# All-in-one image for GitHub Codespaces and local devcontainer usage.
# Includes: ROS2 Jazzy, Nav2, Gazebo Harmonic (headless),
# build tools, and developer utilities.
#
# Gazebo runs headless (CPU rendering) — connect Foxglove Studio to
# ws://localhost:8765 for visualization.
# =============================================================================
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# 1. All ROS2 runtime + build + simulation dependencies in one layer
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ── Navigation ──
    ros-kilted-nav2-bringup \
    ros-kilted-nav2-bt-navigator \
    ros-kilted-nav2-controller \
    ros-kilted-nav2-planner \
    ros-kilted-nav2-behaviors \
    ros-kilted-nav2-regulated-pure-pursuit-controller \
    ros-kilted-nav2-smac-planner \
    ros-kilted-nav2-costmap-2d \
    ros-kilted-nav2-map-server \
    ros-kilted-nav2-lifecycle-manager \
    ros-kilted-nav2-waypoint-follower \
    ros-kilted-nav2-velocity-smoother \
    ros-kilted-nav2-rotation-shim-controller \
    ros-kilted-nav2-core \
    ros-kilted-nav2-util \
    ros-kilted-nav2-msgs \
    # ── SLAM (FusionCore is built from source in the workspace) ──
    ros-kilted-slam-toolbox \
    # ── DDS ──
    ros-kilted-rmw-cyclonedds-cpp \
    # ── Docking ──
    ros-kilted-opennav-docking \
    # ── Motion control ──
    ros-kilted-twist-mux \
    ros-kilted-topic-tools \
    # ── Behavior trees ──
    ros-kilted-behaviortree-cpp \
    # ── Robot description / TF ──
    ros-kilted-xacro \
    ros-kilted-robot-state-publisher \
    ros-kilted-joint-state-publisher \
    ros-kilted-tf2-ros \
    ros-kilted-tf2-eigen \
    ros-kilted-tf2-geometry-msgs \
    # ── MavROS (for PX4 SITL) ──
    ros-kilted-mavros \
    ros-kilted-mavros-msgs \
    ros-kilted-mavros-extras \
    # ── Grid map ──
    ros-kilted-grid-map-core \
    ros-kilted-grid-map-ros \
    ros-kilted-grid-map-msgs \
    # ── Diagnostics ──
    ros-kilted-diagnostic-msgs \
    ros-kilted-diagnostic-updater \
    # ── Bridges ──
    ros-kilted-foxglove-bridge \
    ros-kilted-rosbridge-suite \
    # ── Message definitions ──
    ros-kilted-ublox-msgs \
    ros-kilted-nmea-msgs \
    ros-kilted-rtcm-msgs \
    # ── Gazebo Harmonic (headless simulation) ──
    ros-kilted-ros-gz-sim \
    ros-kilted-ros-gz-bridge \
    ros-kilted-ros-gz-interfaces \
    # ── Build tools ──
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-kilted-ament-cmake \
    ros-kilted-rosidl-default-generators \
    build-essential \
    cmake \
    git \
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
    # ── Headless rendering (Gazebo sensors need a virtual display on ARM) ──
    xvfb \
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
# 2. rosdep init (needed for resolving deps on first build)
# ---------------------------------------------------------------------------
RUN rosdep update --rosdistro kilted || true

# ---------------------------------------------------------------------------
# 3. Workspace directory
# ---------------------------------------------------------------------------
WORKDIR /ros2_ws

# ---------------------------------------------------------------------------
# 4. Environment setup — auto-source ROS2 + workspace in every shell
# ---------------------------------------------------------------------------
RUN echo 'source /opt/ros/kilted/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /etc/bash.bashrc && \
    echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> /etc/bash.bashrc

# ---------------------------------------------------------------------------
# 5. Cyclone DDS config (localhost-only discovery for containers)
# ---------------------------------------------------------------------------
COPY docker/config/cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
