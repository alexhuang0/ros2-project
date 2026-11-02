#!/bin/bash

WATO_WS="${WATO_WS:-/home/bolty/ament_ws}"
WATO_INSTALL="${WATO_INSTALL:-/opt/watonomous}"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
fi

cb() {
    cd "$WATO_WS" || return
    colcon build \
        --packages-up-to bringup_robot \
        --install-base "$WATO_INSTALL" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release || return
    source "$WATO_INSTALL/setup.bash"
}

run() {
    cb && ros2 launch bringup_robot robot.launch.py
}
