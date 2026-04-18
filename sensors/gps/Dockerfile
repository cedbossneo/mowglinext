# =============================================================================
# u-blox GPS driver + NTRIP client for ROS2 Jazzy
#
# Publishes sensor_msgs/NavSatFix on /ublox_gps_node/fix.
# Receives RTK corrections via ntrip_client and forwards to u-blox receiver.
#
# All config is read from /config/mowgli_robot.yaml (bind-mounted).
# =============================================================================
FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-ublox \
    ros-kilted-ntrip-client \
    ros-kilted-rtcm-msgs \
    ros-kilted-rmw-cyclonedds-cpp \
    ros-kilted-nmea-navsat-driver \
 && rm -rf /var/lib/apt/lists/*

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_gps.sh /start_gps.sh
COPY f9p_rover.yaml /f9p_rover.yaml
COPY rtcm_serial_bridge.py /rtcm_serial_bridge.py
RUN chmod +x /ros2_entrypoint.sh /start_gps.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps.sh"]
