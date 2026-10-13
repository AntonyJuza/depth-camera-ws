#!/usr/bin/env python3
"""
ZMQ Stereo Depth Camera ROS 2 Driver Node
-----------------------------------------
Subscribes to raw multipart frames over ZeroMQ from the IR stereo depth camera
and publishes standard ROS 2 image, camera info, and point cloud topics.
"""

import os
import math
import zmq
import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from cv_bridge import CvBridge

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


class ZmqDepthCameraNode(Node):
    def __init__(self):
        super().__init__('zmq_depth_camera_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('zmq_url', 'tcp://10.11.11.1:20000')
        self.declare_parameter('poll_rate_hz', 60.0)
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('depth_frame_id', 'camera_depth_optical_frame')
        self.declare_parameter('left_frame_id', 'camera_left_optical_frame')
        self.declare_parameter('right_frame_id', 'camera_right_optical_frame')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_base_tf', False)
        self.declare_parameter('parent_frame_id', 'base_link')
        self.declare_parameter('camera_pos_x', 0.15)
        self.declare_parameter('camera_pos_y', 0.0)
        self.declare_parameter('camera_pos_z', 0.06)
        self.declare_parameter('camera_yaw', 0.0)
        self.declare_parameter('camera_pitch', 0.1745)  # ~10 degrees down tilt to detect low obstacles
        self.declare_parameter('camera_roll', 0.0)
        self.declare_parameter('enable_pointcloud', True)
        self.declare_parameter('pointcloud_decimation', 2)  # 2x2 decimation for lightweight Nav2 costmap processing
        self.declare_parameter('pointcloud_min_depth', 0.15)
        self.declare_parameter('pointcloud_max_depth', 3.5)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('max_depth_threshold', 10000)
        self.declare_parameter('left_info_file', '')
        self.declare_parameter('right_info_file', '')
        self.declare_parameter('depth_info_file', '')

        # Retrieve parameter values
        self.zmq_url = self.get_parameter('zmq_url').get_parameter_value().string_value
        self.poll_rate_hz = self.get_parameter('poll_rate_hz').get_parameter_value().double_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.depth_frame_id = self.get_parameter('depth_frame_id').get_parameter_value().string_value
        self.left_frame_id = self.get_parameter('left_frame_id').get_parameter_value().string_value
        self.right_frame_id = self.get_parameter('right_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.publish_base_tf = self.get_parameter('publish_base_tf').get_parameter_value().bool_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.camera_pos_x = self.get_parameter('camera_pos_x').get_parameter_value().double_value
        self.camera_pos_y = self.get_parameter('camera_pos_y').get_parameter_value().double_value
        self.camera_pos_z = self.get_parameter('camera_pos_z').get_parameter_value().double_value
        self.camera_yaw = self.get_parameter('camera_yaw').get_parameter_value().double_value
        self.camera_pitch = self.get_parameter('camera_pitch').get_parameter_value().double_value
        self.camera_roll = self.get_parameter('camera_roll').get_parameter_value().double_value
        self.enable_pointcloud = self.get_parameter('enable_pointcloud').get_parameter_value().bool_value
        self.pc_decimation = max(1, self.get_parameter('pointcloud_decimation').get_parameter_value().integer_value)
        self.pc_min_depth = self.get_parameter('pointcloud_min_depth').get_parameter_value().double_value
        self.pc_max_depth = self.get_parameter('pointcloud_max_depth').get_parameter_value().double_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.max_depth_threshold = self.get_parameter('max_depth_threshold').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.frame_counter = 0

        # Create Publishers (QoS depth = 5)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 5)
        self.depth_caminfo_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 5)
        self.ir_pub = self.create_publisher(Image, '/camera/ir/image_raw', 5)
        self.left_image_pub = self.create_publisher(Image, '/camera/left/image_rect', 5)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/left/camera_info', 5)
        self.right_image_pub = self.create_publisher(Image, '/camera/right/image_rect', 5)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera/right/camera_info', 5)
        self.conf_pub = self.create_publisher(Image, '/camera/confidence/image_raw', 5)
        self.raw_pub = self.create_publisher(Image, '/camera/raw/image_raw', 5)

        if self.enable_pointcloud:
            self.pc_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 5)
        else:
            self.pc_pub = None

        # Static TF Broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Locate config files & load calibrations
        self.init_camera_info()

        # Publish initial static transforms
        if self.publish_tf:
            self.publish_static_transforms()

        # Precompute PointCloud projection meshgrid
        self.update_pointcloud_intrinsics()

        # Setup ZeroMQ Subscriber
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.setsockopt(zmq.RCVHWM, 10)
        self.get_logger().info(f"Connecting to ZMQ camera stream at {self.zmq_url}...")
        self.socket.connect(self.zmq_url)

        # Polling timer
        timer_period = 1.0 / max(1.0, self.poll_rate_hz)
        self.timer = self.create_timer(timer_period, self.poll_zmq)

        self.get_logger().info("ZMQ Stereo Depth Camera Node initialized successfully.")

    def find_config_file(self, filename: str) -> str:
        """Finds configuration file in package share, workspace, or local path."""
        # 1. Check share dir
        if get_package_share_directory:
            try:
                share_dir = get_package_share_directory('zmq_depth_camera')
                candidate = os.path.join(share_dir, 'config', filename)
                if os.path.isfile(candidate):
                    return candidate
            except Exception:
                pass

        # 2. Check relative to this script
        this_dir = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.abspath(os.path.join(this_dir, '..', 'config', filename))
        if os.path.isfile(candidate):
            return candidate

        # 3. Check current working directory
        candidate = os.path.abspath(os.path.join('config', filename))
        if os.path.isfile(candidate):
            return candidate

        return ''

    def load_camera_info_from_yaml(self, path: str, frame_id: str) -> CameraInfo:
        """Parses standard ROS CameraInfo YAML format."""
        ci = CameraInfo()
        ci.header.frame_id = frame_id
        if not path or not os.path.isfile(path):
            return None

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            ci.width = int(data.get('image_width', 640))
            ci.height = int(data.get('image_height', 480))
            ci.distortion_model = data.get('distortion_model', 'plumb_bob')
            ci.d = [float(x) for x in data.get('distortion_coefficients', {}).get('data', [0.0]*5)]
            ci.k = [float(x) for x in data.get('camera_matrix', {}).get('data', [0.0]*9)]
            ci.r = [float(x) for x in data.get('rectification_matrix', {}).get('data', [0.0]*9)]
            ci.p = [float(x) for x in data.get('projection_matrix', {}).get('data', [0.0]*12)]
            return ci
        except Exception as e:
            self.get_logger().warn(f"Failed to load CameraInfo from {path}: {e}")
            return None

    def init_camera_info(self):
        """Initializes Left, Right, and Depth CameraInfo objects."""
        left_path = self.get_parameter('left_info_file').get_parameter_value().string_value
        right_path = self.get_parameter('right_info_file').get_parameter_value().string_value
        depth_path = self.get_parameter('depth_info_file').get_parameter_value().string_value

        if not left_path:
            left_path = self.find_config_file('left_camera_info.yaml')
        if not right_path:
            right_path = self.find_config_file('right_camera_info.yaml')
        if not depth_path:
            depth_path = self.find_config_file('depth_camera_info.yaml')

        self.caminfo_left = self.load_camera_info_from_yaml(left_path, self.left_frame_id)
        self.caminfo_right = self.load_camera_info_from_yaml(right_path, self.right_frame_id)
        self.caminfo_depth = self.load_camera_info_from_yaml(depth_path, self.depth_frame_id)

        # Fallback defaults if YAML files not found
        if self.caminfo_left is None:
            self.caminfo_left = self.make_default_camera_info(640, 480, 405.3, 404.9, 328.4, 233.9, self.left_frame_id)
        if self.caminfo_right is None:
            self.caminfo_right = self.make_default_camera_info(640, 480, 408.6, 408.8, 339.9, 242.1, self.right_frame_id)
            self.caminfo_right.p[3] = -10.272  # Baseline Tx * fx
        if self.caminfo_depth is None:
            self.caminfo_depth = self.make_default_camera_info(320, 240, 202.65, 202.43, 164.22, 116.97, self.depth_frame_id)

        self.get_logger().info(f"Loaded Left CameraInfo: fx={self.caminfo_left.k[0]:.1f}, cx={self.caminfo_left.k[2]:.1f}")
        self.get_logger().info(f"Loaded Right CameraInfo: fx={self.caminfo_right.k[0]:.1f}, cx={self.caminfo_right.k[2]:.1f}")
        self.get_logger().info(f"Loaded Depth CameraInfo: fx={self.caminfo_depth.k[0]:.1f}, cx={self.caminfo_depth.k[2]:.1f}")

    def make_default_camera_info(self, w, h, fx, fy, cx, cy, frame_id):
        ci = CameraInfo()
        ci.header.frame_id = frame_id
        ci.width = w
        ci.height = h
        ci.distortion_model = 'plumb_bob'
        ci.k = [fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0]
        ci.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return ci

    def update_pointcloud_intrinsics(self):
        """Precomputes normalized image ray coordinates for fast pointcloud projection."""
        w = self.caminfo_depth.width
        h = self.caminfo_depth.height
        fx = self.caminfo_depth.k[0]
        fy = self.caminfo_depth.k[4]
        cx = self.caminfo_depth.k[2]
        cy = self.caminfo_depth.k[5]

        step = max(1, self.pc_decimation)
        u = np.arange(0, w, step, dtype=np.float32)
        v = np.arange(0, h, step, dtype=np.float32)
        u_grid, v_grid = np.meshgrid(u, v)

        self.x_coeff = (u_grid - cx) / fx
        self.y_coeff = (v_grid - cy) / fy

    def decode_stereo_calibration(self, blob: bytes):
        """Decode 472-byte stereo calibration blob from stream."""
        vals = np.frombuffer(blob, dtype=np.float64)
        if len(vals) < 59:
            return

        fx_l, _, cx_l, _, fy_l, cy_l, _, _, _ = vals[0:9]
        d_left = vals[8:13].tolist()

        fx_r, _, cx_r, _, fy_r, cy_r, _, _, _ = vals[20:29]
        d_right = vals[32:37].tolist()

        r = vals[44:53].tolist()
        t = vals[56:59].tolist()

        # Update left
        self.caminfo_left.width = 640
        self.caminfo_left.height = 480
        self.caminfo_left.k = [fx_l, 0.0, cx_l, 0.0, fy_l, cy_l, 0.0, 0.0, 1.0]
        self.caminfo_left.d = d_left
        self.caminfo_left.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.caminfo_left.p = [fx_l, 0.0, cx_l, 0.0, 0.0, fy_l, cy_l, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.caminfo_left.header.frame_id = self.left_frame_id

        # Update right
        self.caminfo_right.width = 640
        self.caminfo_right.height = 480
        self.caminfo_right.k = [fx_r, 0.0, cx_r, 0.0, fy_r, cy_r, 0.0, 0.0, 1.0]
        self.caminfo_right.d = d_right
        self.caminfo_right.r = r
        self.caminfo_right.p = [fx_r, 0.0, cx_r, t[0]*fx_r,
                                0.0, fy_r, cy_r, t[1]*fx_r,
                                0.0, 0.0, 1.0, t[2]]
        self.caminfo_right.header.frame_id = self.right_frame_id

        # Update depth (scaled by 0.5)
        self.caminfo_depth.width = 320
        self.caminfo_depth.height = 240
        fx_d, fy_d = fx_l * 0.5, fy_l * 0.5
        cx_d, cy_d = cx_l * 0.5, cy_l * 0.5
        self.caminfo_depth.k = [fx_d, 0.0, cx_d, 0.0, fy_d, cy_d, 0.0, 0.0, 1.0]
        self.caminfo_depth.p = [fx_d, 0.0, cx_d, 0.0, 0.0, fy_d, cy_d, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.caminfo_depth.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.caminfo_depth.header.frame_id = self.depth_frame_id

        self.update_pointcloud_intrinsics()
        self.get_logger().info("Dynamically updated camera calibration from stream.")

    def publish_static_transforms(self):
        """Broadcasts static TF frames according to ROS REP-103 and REP-105."""
        transforms = []
        now = self.get_clock().now().to_msg()

        # Optical frame rotation quaternion: [qx, qy, qz, qw] = [-0.5, 0.5, -0.5, 0.5]
        # Maps camera_link (X fwd, Y left, Z up) to optical frame (Z fwd, X right, Y down)
        qx, qy, qz, qw = -0.5, 0.5, -0.5, 0.5

        # 1. camera_link -> camera_depth_optical_frame
        t_depth = TransformStamped()
        t_depth.header.stamp = now
        t_depth.header.frame_id = self.camera_frame_id
        t_depth.child_frame_id = self.depth_frame_id
        t_depth.transform.translation.x = 0.0
        t_depth.transform.translation.y = 0.0
        t_depth.transform.translation.z = 0.0
        t_depth.transform.rotation.x = qx
        t_depth.transform.rotation.y = qy
        t_depth.transform.rotation.z = qz
        t_depth.transform.rotation.w = qw
        transforms.append(t_depth)

        # 2. camera_link -> camera_left_optical_frame
        t_left = TransformStamped()
        t_left.header.stamp = now
        t_left.header.frame_id = self.camera_frame_id
        t_left.child_frame_id = self.left_frame_id
        t_left.transform.translation.x = 0.0
        t_left.transform.translation.y = 0.0125  # ~1.25 cm left
        t_left.transform.translation.z = 0.0
        t_left.transform.rotation.x = qx
        t_left.transform.rotation.y = qy
        t_left.transform.rotation.z = qz
        t_left.transform.rotation.w = qw
        transforms.append(t_left)

        # 3. camera_link -> camera_right_optical_frame
        t_right = TransformStamped()
        t_right.header.stamp = now
        t_right.header.frame_id = self.camera_frame_id
        t_right.child_frame_id = self.right_frame_id
        t_right.transform.translation.x = 0.0
        t_right.transform.translation.y = -0.0125  # ~1.25 cm right (2.5 cm baseline)
        t_right.transform.translation.z = 0.0
        t_right.transform.rotation.x = qx
        t_right.transform.rotation.y = qy
        t_right.transform.rotation.z = qz
        t_right.transform.rotation.w = qw
        transforms.append(t_right)

        # 4. Optional base_link -> camera_link
        if self.publish_base_tf:
            t_base = TransformStamped()
            t_base.header.stamp = now
            t_base.header.frame_id = self.parent_frame_id
            t_base.child_frame_id = self.camera_frame_id
            t_base.transform.translation.x = self.camera_pos_x
            t_base.transform.translation.y = self.camera_pos_y
            t_base.transform.translation.z = self.camera_pos_z
            # Euler roll, pitch, yaw to quaternion (ZYX sequence)
            cr = math.cos(self.camera_roll * 0.5)
            sr = math.sin(self.camera_roll * 0.5)
            cp = math.cos(self.camera_pitch * 0.5)
            sp = math.sin(self.camera_pitch * 0.5)
            cy = math.cos(self.camera_yaw * 0.5)
            sy = math.sin(self.camera_yaw * 0.5)

            t_base.transform.rotation.x = sr * cp * cy - cr * sp * sy
            t_base.transform.rotation.y = cr * sp * cy + sr * cp * sy
            t_base.transform.rotation.z = cr * cp * sy - sr * sp * cy
            t_base.transform.rotation.w = cr * cp * cy + sr * sp * sy
            transforms.append(t_base)

        self.static_tf_broadcaster.sendTransform(transforms)

    def generate_point_cloud(self, depth_arr: np.ndarray, stamp) -> PointCloud2:
        """Fast vectorized PointCloud2 generation directly from depth image."""
        min_thresh = int(self.pc_min_depth / self.depth_scale)
        max_thresh = int(min(self.pc_max_depth / self.depth_scale, self.max_depth_threshold))

        # Downsample depth map if decimation > 1
        if self.pc_decimation > 1:
            depth_sampled = depth_arr[::self.pc_decimation, ::self.pc_decimation]
        else:
            depth_sampled = depth_arr

        mask = (depth_sampled > min_thresh) & (depth_sampled < max_thresh)
        z = depth_sampled[mask] * self.depth_scale
        x = self.x_coeff[mask] * z
        y = self.y_coeff[mask] * z

        num_points = len(z)
        pts = np.empty((num_points, 3), dtype=np.float32)
        pts[:, 0] = x
        pts[:, 1] = y
        pts[:, 2] = z

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.depth_frame_id
        msg.height = 1
        msg.width = num_points
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * num_points
        msg.is_dense = True
        msg.data = pts.tobytes()
        return msg

    def poll_zmq(self):
        """Polls all available messages from ZeroMQ socket in non-blocking loop."""
        while True:
            try:
                parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception as e:
                self.get_logger().warn(f"Error receiving ZMQ frame: {e}", throttle_duration_sec=2.0)
                break

            self.frame_counter += 1
            if not parts:
                continue

            try:
                frame_name = parts[0].decode('utf-8', errors='ignore')
            except Exception:
                continue

            num_parts = len(parts)

            # 1. Handle Calibration Frame (2 parts, 472 bytes)
            if frame_name == 'stereo_cali_version2' and num_parts >= 2 and len(parts[1]) == 472:
                try:
                    self.decode_stereo_calibration(parts[1])
                except Exception as e:
                    self.get_logger().warn(f"Error decoding calibration blob: {e}")
                continue

            # Need at least 3 parts for standard image payloads
            if num_parts < 3:
                continue

            payload = parts[2]
            L = len(payload)
            now = self.get_clock().now().to_msg()

            # 2. Stereo Depth Map (320x240, 16UC1)
            if frame_name == 'stereo_depth_map' and L in (153600, 153656):
                w, h = 320, 240
                arr = np.frombuffer(payload[-(w*h*2):], dtype=np.uint16).reshape((h, w)).copy()

                # Filter invalid error codes to 0
                arr[arr >= self.max_depth_threshold] = 0

                # Publish Depth Image
                img_msg = self.bridge.cv2_to_imgmsg(arr, encoding='16UC1')
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.depth_frame_id
                self.depth_pub.publish(img_msg)

                # Publish CameraInfo
                self.caminfo_depth.header.stamp = now
                self.depth_caminfo_pub.publish(self.caminfo_depth)

                # Publish Point Cloud
                if self.enable_pointcloud and self.pc_pub is not None:
                    try:
                        pc_msg = self.generate_point_cloud(arr, now)
                        self.pc_pub.publish(pc_msg)
                    except Exception as e:
                        self.get_logger().warn(f"Point cloud generation error: {e}", throttle_duration_sec=5.0)

            # 3. Stereo Pair Rectified Images (Left & Right, 640x480 each, mono8)
            elif frame_name == 'pair_rectify_imgs' and L in (614400, 614448):
                w, h = 640, 480
                raw_bytes = payload[-(w*h*2):]
                left_arr = np.frombuffer(raw_bytes[:w*h], dtype=np.uint8).reshape((h, w))
                right_arr = np.frombuffer(raw_bytes[w*h:], dtype=np.uint8).reshape((h, w))

                # Publish Left Rectified Image & Info
                left_msg = self.bridge.cv2_to_imgmsg(left_arr, encoding='mono8')
                left_msg.header.stamp = now
                left_msg.header.frame_id = self.left_frame_id
                self.left_image_pub.publish(left_msg)
                self.caminfo_left.header.stamp = now
                self.left_info_pub.publish(self.caminfo_left)

                # Also publish to /camera/ir/image_raw for backwards compatibility
                ir_msg = self.bridge.cv2_to_imgmsg(left_arr, encoding='mono8')
                ir_msg.header.stamp = now
                ir_msg.header.frame_id = self.left_frame_id
                self.ir_pub.publish(ir_msg)

                # Publish Right Rectified Image & Info
                right_msg = self.bridge.cv2_to_imgmsg(right_arr, encoding='mono8')
                right_msg.header.stamp = now
                right_msg.header.frame_id = self.right_frame_id
                self.right_image_pub.publish(right_msg)
                self.caminfo_right.header.stamp = now
                self.right_info_pub.publish(self.caminfo_right)

            # 4. Single Rectified IR Image (640x480, mono8)
            elif frame_name == 'rectify_img' and L in (307200, 307224):
                w, h = 640, 480
                arr = np.frombuffer(payload[-(w*h):], dtype=np.uint8).reshape((h, w))
                msg = self.bridge.cv2_to_imgmsg(arr, encoding='mono8')
                msg.header.stamp = now
                msg.header.frame_id = self.left_frame_id
                self.ir_pub.publish(msg)
                self.caminfo_left.header.stamp = now
                self.left_info_pub.publish(self.caminfo_left)

            # 5. Confidence Image (320x240, mono8)
            elif frame_name == 'gray_downscale_image' and L in (76800, 76824):
                w, h = 320, 240
                arr = np.frombuffer(payload[-(w*h):], dtype=np.uint8).reshape((h, w))
                msg = self.bridge.cv2_to_imgmsg(arr, encoding='mono8')
                msg.header.stamp = now
                msg.header.frame_id = self.depth_frame_id
                self.conf_pub.publish(msg)

            # 6. Raw Gray Image (640x480, 16UC1)
            elif frame_name == 'gray_image' and L == 614400:
                w, h = 640, 480
                arr = np.frombuffer(payload, dtype=np.uint16).reshape((h, w))
                msg = self.bridge.cv2_to_imgmsg(arr, encoding='16UC1')
                msg.header.stamp = now
                msg.header.frame_id = self.camera_frame_id
                self.raw_pub.publish(msg)

            # Periodic status logging
            if self.frame_counter % 150 == 0:
                self.get_logger().info(f"Streaming OK. Frame count: {self.frame_counter} (last: {frame_name})")


def main(args=None):
    rclpy.init(args=args)
    node = ZmqDepthCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
