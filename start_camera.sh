#!/usr/bin/env bash
# Camera ROS2 Startup Script

# Prevent cross-network DDS conflicts with other robots on the local network
export ROS_LOCALHOST_ONLY=1

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
if [ -f /home/ganga/depth_ws/install/setup.bash ]; then
    source /home/ganga/depth_ws/install/setup.bash
fi

echo "============================================="
echo " Starting ZMQ Stereo Depth Camera ROS2 Driver"
echo " Camera Stream: tcp://10.11.11.1:20000"
echo " Mode: ROS_LOCALHOST_ONLY=1"
echo "============================================="

# Function to clean up background processes on exit
cleanup() {
    echo "Shutting down camera driver..."
    kill -INT "$PID_CAM" "$PID_PC" 2>/dev/null
    wait "$PID_CAM" "$PID_PC" 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 1. Start ZMQ Camera publisher
ros2 run zmq_depth_camera ir_pub "$@" &
PID_CAM=$!

# 2. Start Depth to 3D PointCloud converter
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/depth/camera_info \
  -r points:=/camera/depth/points &
PID_PC=$!

echo "Camera Node PID: $PID_CAM"
echo "Point Cloud Node PID: $PID_PC"
echo "Streaming active. Press Ctrl+C to stop."

wait "$PID_CAM" "$PID_PC"
