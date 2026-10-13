# ZMQ Stereo Depth Camera ROS 2 Driver

ROS 2 driver package for the IR Stereo Depth Camera streaming over ZeroMQ (`tcp://10.11.11.1:20000`).

## Features
- **All-in-One ROS 2 Driver**: Consolidates streaming, dynamic calibration decoding, optical TF frames, and point cloud generation into a clean, unified node.
- **Fast Built-In 3D PointCloud2**: Generates 3D point clouds directly (< 0.5 ms/frame) without requiring external dependencies like `depth_image_proc`.
- **Dynamic Calibration**: Automatically decodes runtime calibration frames from the camera stream, updating `CameraInfo` and projection grids dynamically while supporting fallback calibration YAMLs.
- **Full Stereo Support**: Publishes left and right rectified stereo pairs (`/camera/left/image_rect`, `/camera/right/image_rect`) with their corresponding stereo `CameraInfo` for stereo SLAM / VIO (RTAB-Map, ORB-SLAM3).
- **ROS 2 Jazzy Compatible**: Cleanly configured and verified on Ubuntu 24.04 with ROS 2 Jazzy.

---

## Published Topics

| Topic | Type | Resolution / Encoding | Description |
|---|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 320×240, `16UC1` | Rectified depth map (mm) |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 320×240 | Intrinsics for depth camera |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 3D PointCloud | XYZ point cloud in `camera_depth_optical_frame` |
| `/camera/ir/image_raw` | `sensor_msgs/Image` | 640×480, `mono8` | Rectified infrared image |
| `/camera/left/image_rect` | `sensor_msgs/Image` | 640×480, `mono8` | Left rectified stereo image |
| `/camera/left/camera_info` | `sensor_msgs/CameraInfo` | 640×480 | Left camera calibration matrix |
| `/camera/right/image_rect` | `sensor_msgs/Image` | 640×480, `mono8` | Right rectified stereo image |
| `/camera/right/camera_info` | `sensor_msgs/CameraInfo` | 640×480 | Right camera calibration matrix |
| `/camera/confidence/image_raw` | `sensor_msgs/Image` | 320×240, `mono8` | Confidence / downscaled map |
| `/camera/raw/image_raw` | `sensor_msgs/Image` | 640×480, `16UC1` | Raw infrared sensor feed |

---

## TF Frame Hierarchy

The driver broadcasts static transforms compliant with ROS REP-103:
```
camera_link (base frame: X forward, Y left, Z up)
├── camera_depth_optical_frame (optical: Z forward, X right, Y down)
├── camera_left_optical_frame  (optical, Y=+0.0125m offset)
└── camera_right_optical_frame (optical, Y=-0.0125m offset)
```
Optionally, setting `publish_base_tf:=true` broadcasts `base_link -> camera_link` with configurable `(camera_pos_x, camera_pos_y, camera_pos_z, camera_yaw)`.

---

## Quick Start

### 1. Launch Driver Directly
```bash
./start_camera.sh
```

### 2. Launch Driver with RViz2 Visualization
```bash
./start_camera.sh --rviz
```

### 3. Using ROS 2 Launch
```bash
source install/setup.bash
ros2 launch zmq_depth_camera camera.launch.py
```

### 4. Overriding Parameters via CLI
```bash
ros2 launch zmq_depth_camera camera.launch.py zmq_url:=tcp://10.11.11.1:20000 enable_pointcloud:=true
```

---

## Configuration Files
Located in [`src/zmq_depth_camera/config/`](src/zmq_depth_camera/config/):
- `camera_params.yaml`: Node parameters (stream URL, frames, pointcloud distance limits, etc.)
- `left_camera_info.yaml`: Left stereo intrinsics and distortion
- `right_camera_info.yaml`: Right stereo intrinsics, rotation, and projection matrix
- `depth_camera_info.yaml`: 320×240 depth sensor calibration
