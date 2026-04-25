# syntax=docker/dockerfile:1.7
# =============================================================================
# Generic NMEA GPS driver + NTRIP client (str2str) for ROS2 Kilted.
#
# Use this image when sensors/gps/ (ublox_dgnss / ZED-F9P over USB-CDC) does
# not apply — i.e. for any RTK receiver that streams NMEA over a serial port:
#
#   * Unicore UM980 / UM982            (USB-CDC, NMEA at 115200)
#   * ArduSimple simpleRTK2B variants   (USB-CDC, NMEA configurable)
#   * Septentrio Mosaic                 (USB-CDC, NMEA configurable)
#   * u-blox ZED-F9P configured to NMEA-only (rare; ublox_dgnss is preferred)
#
# Components:
#   1. nmea_serial_bridge.py     reads /dev/gps NMEA → publishes nmea_msgs/Sentence
#                                on /nmea_sentence (RELIABLE QoS)
#   2. nmea_navsat_driver        nmea_topic_driver: /nmea_sentence → /gps/fix
#                                (sensor_msgs/NavSatFix), /gps/vel, /gps/time_reference
#   3. str2str (rtklib)          NTRIP client: pulls RTCM3 from caster, writes
#                                directly to the serial device opened by (1)
#                                — Linux allows multiple opens of the same
#                                CDC node, so read (Python) and write (str2str)
#                                cohabit fine.
#
# Config is read from /config/mowgli_robot.yaml (bind-mounted from the
# install dir) — same convention as sensors/gps/.
# =============================================================================

FROM ros:kilted-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://azure.archive.ubuntu.com/ubuntu|g' \
      /etc/apt/sources.list.d/*.sources 2>/dev/null || true

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-kilted-nmea-navsat-driver \
      ros-kilted-rmw-cyclonedds-cpp \
      python3-serial \
      rtklib \
      inotify-tools

COPY ros2_entrypoint.sh /ros2_entrypoint.sh
COPY start_gps_nmea.sh /start_gps_nmea.sh
COPY nmea_serial_bridge.py /nmea_serial_bridge.py
RUN chmod +x /ros2_entrypoint.sh /start_gps_nmea.sh /nmea_serial_bridge.py

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["/start_gps_nmea.sh"]
