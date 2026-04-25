#!/bin/bash
set -e
set +u
source /opt/ros/kilted/setup.bash
set -u
exec "$@"
