#!/usr/bin/env bash
# ==============================================================================
# ZMQ Stereo Depth Camera ROS 2 Driver Startup Script
# ==============================================================================
set -e

# 1. Source complete ROS 2 Jazzy environment
if [ -f "$HOME/setup_env.sh" ]; then
    source "$HOME/setup_env.sh"
elif [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/install/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# CycloneDDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "$HOME/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://"$HOME/cyclonedds.xml"
fi

# 2. Source rover_twin_ws overlay where zmq_depth_camera is installed
if [ -f "$HOME/rover_twin_ws/install/setup.bash" ]; then
    source "$HOME/rover_twin_ws/install/setup.bash"
fi

echo "============================================="
echo " Starting ZMQ Stereo Depth Camera ROS 2 Driver"
echo " Stream Target : tcp://10.11.11.1:20000"
echo " Distro        : ${ROS_DISTRO:-jazzy}"
echo " Range Mode    : Close-Range (8cm - 3.5m)"
echo " Mounting      : Base plate above caster mount (X=+0.150m, Z=+0.002m)"
echo " Topics        : /camera/depth/points, /camera/depth/image_raw, /camera/ir/image_raw"
echo "============================================="

if [[ "$*" == *"--rviz"* ]]; then
    exec ros2 launch zmq_depth_camera camera_with_rviz.launch.py
else
    exec ros2 launch zmq_depth_camera camera.launch.py "$@"
fi
