#!/usr/bin/env bash
# ==============================================================================
# ZMQ Stereo Depth Camera ROS 2 Driver Startup Script
# ==============================================================================

# Restrict discovery to localhost only if explicitly requested (--localhost)
if [[ "$*" == *"--localhost"* ]]; then
    export ROS_LOCALHOST_ONLY=1
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    DISCOVERY_MODE="Localhost Only"
else
    DISCOVERY_MODE="Network / Subnet"
fi

# 1. Source ROS 2 base installation
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/rolling/setup.bash ]; then
    source /opt/ros/rolling/setup.bash
fi

# 2. Source workspace overlay
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
elif [ -f "$SCRIPT_DIR/../install/setup.bash" ]; then
    source "$SCRIPT_DIR/../install/setup.bash"
elif [ -f "/media/juza/8AE62852E62840B5/rover_slam_ws/install/setup.bash" ]; then
    source "/media/juza/8AE62852E62840B5/rover_slam_ws/install/setup.bash"
fi

echo "============================================="
echo " Starting ZMQ Stereo Depth Camera ROS 2 Driver"
echo " Stream Target : tcp://10.11.11.1:20000"
echo " Distro        : ${ROS_DISTRO:-jazzy}"
echo " Discovery     : ${DISCOVERY_MODE:-Network}"
echo " PointCloud    : Built-in fast 3D generation (decimated for Nav2)"
echo "============================================="

# Check if user requested RViz
if [[ "$*" == *"--rviz"* ]]; then
    echo "Launching camera driver with RViz2..."
    exec ros2 launch zmq_depth_camera camera_with_rviz.launch.py
else
    echo "Launching camera driver (press Ctrl+C to stop)..."
    exec ros2 launch zmq_depth_camera camera.launch.py "$@"
fi
