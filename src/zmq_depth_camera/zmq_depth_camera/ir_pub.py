import os
import zmq
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# Canonical sizes -> properties
CANDIDATES = {
    76800:  {"w": 320, "h": 240, "bpp": 1, "enc": "mono8",  "label": "confidence (8UC1)"},
    153600: {"w": 320, "h": 240, "bpp": 2, "enc": "16UC1",  "label": "depth 320x240 (16UC1)"},
    307200: {"w": 640, "h": 480, "bpp": 1, "enc": "mono8",  "label": "mono 640x480 (8UC1)"},
    614400: {"w": 640, "h": 480, "bpp": 2, "enc": "16UC1",  "label": "640x480 2Bpp (RGB565/16UC1)"},
    472:    {"w": 0, "h": 0, "bpp": 0, "enc": None, "label": "stereo calibration blob"},
}

TOLERANCE = 128  # allow small extra header/footer
DUMP_DIR = os.path.expanduser("~/unknown_frame_dumps")
os.makedirs(DUMP_DIR, exist_ok=True)


class DetectPublisher(Node):
    def __init__(self, zmq_url="tcp://10.11.11.1:20000"):
        super().__init__("zmq_detect_pub")
        self.bridge = CvBridge()
        self.frame_counter = 0

        # Publishers
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 5)
        self.ir_pub = self.create_publisher(Image, '/camera/ir/image_raw', 5)
        self.conf_pub = self.create_publisher(Image, '/camera/confidence/image_raw', 5)
        self.caminfo_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 5)

        # Stereo-specific CameraInfo publishers
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/left/camera_info', 5)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera/right/camera_info', 5)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Default camera info (fallback)
        self.caminfo = self.make_dummy_camera_info(320, 240)

        # Stereo calibration (filled once we decode a 472B blob)
        self.caminfo_left = None
        self.caminfo_right = None

        # ZMQ Setup
        ctx = zmq.Context()
        self.socket = ctx.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")  # Python 3 style
        self.socket.connect(zmq_url)

        # Poll every 20ms
        self.timer = self.create_timer(0.02, self.poll_zmq)

        # Broadcast static TF once every second
        self.tf_timer = self.create_timer(1.0, self.broadcast_tf)

    def make_dummy_camera_info(self, w, h):
        ci = CameraInfo()
        ci.width = w
        ci.height = h
        fx = 285.0
        fy = 285.0
        cx = w / 2.0
        cy = h / 2.0
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
        ci.distortion_model = "plumb_bob"
        ci.header.frame_id = "camera_link"
        return ci

    def decode_stereo_calibration(self, blob: bytes):
        """Decode 472-byte stereo calibration blob into CameraInfo messages."""
        vals = np.frombuffer(blob, dtype=np.float64)

        # Left intrinsics
        fx_l, _, cx_l, _, fy_l, cy_l, _, _, _ = vals[0:9]
        k_left = [fx_l, 0, cx_l,
                  0, fy_l, cy_l,
                  0, 0, 1]
        d_left = vals[8:13].tolist()  # 5 distortion coeffs

        # Right intrinsics
        fx_r, _, cx_r, _, fy_r, cy_r, _, _, _ = vals[20:29]
        k_right = [fx_r, 0, cx_r,
                   0, fy_r, cy_r,
                   0, 0, 1]
        d_right = vals[32:37].tolist()

        # Rotation (3x3) and translation (Tx,Ty,Tz)
        r = vals[44:53].tolist()
        t = vals[56:59].tolist()

        # Left CameraInfo
        ci_left = CameraInfo()
        ci_left.width = 640
        ci_left.height = 480
        ci_left.k = k_left
        ci_left.d = d_left
        ci_left.distortion_model = "plumb_bob"
        ci_left.r = [1,0,0, 0,1,0, 0,0,1]  # left is reference
        ci_left.p = [fx_l, 0, cx_l, 0,
                     0, fy_l, cy_l, 0,
                     0, 0, 1, 0]
        ci_left.header.frame_id = "camera_left_optical"

        # Right CameraInfo
        ci_right = CameraInfo()
        ci_right.width = 640
        ci_right.height = 480
        ci_right.k = k_right
        ci_right.d = d_right
        ci_right.distortion_model = "plumb_bob"
        ci_right.r = r
        ci_right.p = [fx_r, 0, cx_r, t[0]*fx_r,
                      0, fy_r, cy_r, t[1]*fy_r,
                      0, 0, 1, t[2]]
        ci_right.header.frame_id = "camera_right_optical"

        return ci_left, ci_right

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "camera_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def poll_zmq(self):
        try:
            parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        self.frame_counter += 1

        if len(parts) < 3:
            return

        try:
            frame_name = parts[0].decode("utf-8")
        except Exception as e:
            self.get_logger().warn(f"Failed to decode frame name: {e}", throttle_duration_sec=2.0)
            return

        payload = parts[2]
        L = len(payload)
        matched = False

        # Handle stereo calibration blob
        if L == 472:
            try:
                self.caminfo_left, self.caminfo_right = self.decode_stereo_calibration(payload)
                self.get_logger().info("Decoded stereo calibration parameters")
            except Exception as e:
                self.get_logger().warn(f"Calibration decode failed: {e}", throttle_duration_sec=2.0)
            return

        # Frame-specific handlers
        if frame_name == "stereo_depth_map" and L in (153600, 153656):
            w, h, bpp, enc = 320, 240, 2, "16UC1"
            arr = np.frombuffer(payload[-(w*h*bpp):], dtype=np.uint16).reshape((h, w)).copy()
            # Mask out invalid depth error codes (>= 10000, e.g. 65504) to 0 (standard invalid depth)
            arr[arr >= 10000] = 0
            msg = self.bridge.cv2_to_imgmsg(arr, encoding=enc)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.caminfo.header = msg.header
            self.caminfo.width = w
            self.caminfo.height = h
            self.depth_pub.publish(msg)
            self.caminfo_pub.publish(self.caminfo)
            if self.caminfo_right:
                self.caminfo_right.header = msg.header
                self.right_info_pub.publish(self.caminfo_right)
            matched = True

        elif frame_name == "rectify_img" and L in (307200, 307224):
            w, h, bpp, enc = 640, 480, 1, "mono8"
            arr = np.frombuffer(payload[-(w*h*bpp):], dtype=np.uint8).reshape((h, w))
            msg = self.bridge.cv2_to_imgmsg(arr, encoding=enc)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.ir_pub.publish(msg)
            if self.caminfo_left:
                self.caminfo_left.header = msg.header
                self.left_info_pub.publish(self.caminfo_left)
            matched = True

        elif frame_name == "gray_image" and L == 614400:
            w, h, bpp, enc = 640, 480, 2, "16UC1"
            arr = np.frombuffer(payload[-(w*h*bpp):], dtype=np.uint16).reshape((h, w))
            msg = self.bridge.cv2_to_imgmsg(arr, encoding=enc)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            if not hasattr(self, "raw_pub"):
                self.raw_pub = self.create_publisher(Image, '/camera/raw/image_raw', 5)
            self.raw_pub.publish(msg)
            matched = True

        elif frame_name == "gray_downscale_image" and L == 76800:
            w, h, bpp, enc = 320, 240, 1, "mono8"
            arr = np.frombuffer(payload[-(w*h*bpp):], dtype=np.uint8).reshape((h, w))
            msg = self.bridge.cv2_to_imgmsg(arr, encoding=enc)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.conf_pub.publish(msg)
            matched = True

        elif frame_name == "pair_rectify_imgs" and L == 614448:
            matched = True

        if matched:
            if self.frame_counter % 50 == 0:
                self.get_logger().info(f"Streaming... [{self.frame_counter}] Last: {frame_name} ({L} bytes)")
        else:
            fname = os.path.join(DUMP_DIR, f"unknown_frame_{self.frame_counter}_{frame_name}_{L}.bin")
            with open(fname, "wb") as f:
                f.write(payload)
            self.get_logger().warn(f"Unknown frame: {frame_name} ({L} bytes) -> saved to {fname}", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = DetectPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

