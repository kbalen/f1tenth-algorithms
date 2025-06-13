#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/foxy/setup.bash
if [ -f "/root/algorithm_ws/install/setup.bash" ]; then
    source /root/algorithm_ws/install/setup.bash
fi

exec "$@"