import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import zmq

class DepthIRConfPublisher(Node):
    def __init__(self):
        super().__init__('depth_ir_conf_publisher')

        # Publishers
        self.ir_pub = self.create_publisher(Image, '/camera/ir/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.conf_pub = self.create_publisher(Image, '/camera/conf/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        self.bridge = CvBridge()

        # ZMQ setup
        ctx = zmq.Context()
        self.socket = ctx.socket(zmq.SUB)
        self.socket.connect("tcp://10.11.11.1:20000")  # adjust if needed
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)  # ~20 Hz poll

        # Camera intrinsics (replace with real calibration if available)
        self.width = 320
        self.height = 240
        self.fx = 285.0
        self.fy = 285.0
        self.cx = self.width / 2
        self.cy = self.height / 2

    def timer_callback(self):
        try:
            parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        for part in parts:
            # IR frame (153656 bytes -> 320x240 mono8-like)
            if len(part) == 153656:
                try:
                    frame = np.frombuffer(part, dtype=np.uint8)[:self.width*self.height]
                    frame = frame.reshape((self.height, self.width))
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_link"
                    self.ir_pub.publish(msg)
                    self.get_logger().debug("Published IR frame")
                except Exception as e:
                    self.get_logger().warn(f"IR decode failed: {e}")

            # Depth frame (614400 bytes -> 320x240 uint16 depth)
            elif len(part) == 614400:
                try:
                    frame = np.frombuffer(part, dtype=np.uint16).reshape((self.height, self.width))
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="16UC1")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_link"
                    self.depth_pub.publish(msg)

                    # CameraInfo
                    cam_info = CameraInfo()
                    cam_info.header = msg.header
                    cam_info.width = self.width
                    cam_info.height = self.height
                    cam_info.k = [self.fx, 0.0, self.cx,
                                  0.0, self.fy, self.cy,
                                  0.0, 0.0, 1.0]
                    cam_info.p = [self.fx, 0.0, self.cx, 0.0,
                                  0.0, self.fy, self.cy, 0.0,
                                  0.0, 0.0, 1.0, 0.0]
                    cam_info.r = [1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0]
                    cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                    cam_info.distortion_model = "plumb_bob"
                    self.cam_info_pub.publish(cam_info)

                    self.get_logger().debug("Published Depth + CameraInfo")
                except Exception as e:
                    self.get_logger().warn(f"Depth decode failed: {e}")

            # Confidence frame (307224 bytes -> 320x240 mono8)
            elif len(part) == 307224:
                try:
                    frame = np.frombuffer(part, dtype=np.uint8)[:self.width*self.height]
                    frame = frame.reshape((self.height, self.width))
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_link"
                    self.conf_pub.publish(msg)
                    self.get_logger().debug("Published Confidence frame")
                except Exception as e:
                    self.get_logger().warn(f"Conf decode failed: {e}")

def main():
    rclpy.init()
    node = DepthIRConfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

