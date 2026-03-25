ARG IMAGE
FROM ${IMAGE}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    python3-catkin-tools \
    build-essential \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/open_mower_ros/src
RUN git clone https://github.com/foxglove/ros-foxglove-bridge.git foxglove_bridge_src

WORKDIR /opt/open_mower_ros
RUN source /opt/ros/noetic/setup.bash && \
    rosdep install -y --from-paths src --ignore-src || true && \
    catkin_make

COPY docker/foxglove/scripts/start_foxglove.sh /usr/local/bin/start-foxglove.sh
RUN chmod +x /usr/local/bin/start-foxglove.sh