# syntax=docker/dockerfile:1.7
# =============================================================================
# Unicore UM982 GNSS driver (mowgli_unicore_gnss) — ROS 2 C++
#
# Publishes:
#   /gps/fix            sensor_msgs/NavSatFix (NMEA GGA or Unicore PVTSLNA)
#   /gps/azimuth        compass_msgs/Azimuth (heading from HDT or HPR)
#   /gps/diagnostics    diagnostic_msgs/DiagnosticArray (status + counters)
#
# Serial device mounted at runtime: /dev/gps (configurable via um982.yaml)
# =============================================================================

# ─── Builder ────────────────────────────────────────────────────────────────
FROM ros:kilted-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      ros-kilted-rclcpp \
      ros-kilted-rosidl-default-generators \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
      ros-kilted-diagnostic-msgs \
      ros-kilted-geometry-msgs

WORKDIR /ws/src
COPY . mowgli_unicore_gnss

WORKDIR /ws
RUN . /opt/ros/kilted/setup.sh \
 && colcon build --merge-install \
      --packages-up-to mowgli_unicore_gnss \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
                   -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src

# ─── Runtime ────────────────────────────────────────────────────────────────
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-kilted-rmw-cyclonedds-cpp \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-std-msgs

COPY --from=builder /ws/install /opt/mowgli_unicore_gnss

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_gps.sh /start_gps.sh
RUN chmod +x /ros2_entrypoint.sh /start_gps.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
