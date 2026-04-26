# syntax=docker/dockerfile:1.7

ARG UM982_REPO=https://github.com/mowglifrenchtouch/UM982Driver.git
ARG UM982_REF=mowgli

FROM ros:kilted-ros-base AS builder

ARG DEBIAN_FRONTEND=noninteractive
ARG UM982_REPO
ARG UM982_REF

ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:${PATH}"

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      git \
      ccache \
      build-essential \
      cmake \
      libcurl4-openssl-dev \
      python3-colcon-common-extensions \
      ros-kilted-rclcpp \
      ros-kilted-rclcpp-components \
      ros-kilted-diagnostic-msgs \
      ros-kilted-rosidl-default-generators \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src

RUN git clone --depth 1 --branch "${UM982_REF}" "${UM982_REPO}" um982_driver

WORKDIR /ws

RUN --mount=type=cache,target=/root/.ccache,sharing=locked \
    . /opt/ros/kilted/setup.sh \
 && colcon build --merge-install \
      --base-paths src/um982_driver/compass_msgs \
                   src/um982_driver/ntrip_client_node \
                   src/um982_driver \
      --packages-up-to ntrip_client_node mowgli_unicore_gnss \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -Wno-dev \
 && rm -rf /ws/build /ws/log /ws/src

FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libcurl4 \
      ros-kilted-diagnostic-msgs \
      ros-kilted-rclcpp-components \
      ros-kilted-rmw-cyclonedds-cpp \
      ros-kilted-rosidl-default-runtime \
      ros-kilted-rtcm-msgs \
      ros-kilted-sensor-msgs \
      ros-kilted-std-msgs \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY --from=builder /ws/install /ws/install
COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_gps.sh /start_gps.sh

RUN chmod +x /ros2_entrypoint.sh /start_gps.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
