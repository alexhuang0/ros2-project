#!/bin/bash
set -e

# Set up ROS2 and the workspace overlay.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /opt/watonomous/setup.bash ]; then
    source /opt/watonomous/setup.bash
fi
exec "$@"
