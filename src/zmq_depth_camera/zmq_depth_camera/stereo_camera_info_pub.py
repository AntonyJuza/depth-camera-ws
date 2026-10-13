#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import zmq
import numpy as np

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("camera_info_publisher")

        # Publisher
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", 10)

        # ZMQ subscriber
        ctx = zmq.Context()
        self.sub = ctx.socket(zmq.SUB)
        self.sub.connect("tcp://10.11.11.1:20000")   # <-- change to your stream
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")

        # Timer to poll ZMQ
        self.timer = self.create_timer(0.05, self.poll_zmq)  # 20Hz poll

    def poll_zmq(self):
        try:
            # Non-blocking receive
            parts = self.sub.recv_multipart(flags=zmq.NOBLOCK)

            for part in parts:
                if len(part) == 472:   # <-- calibration packet
                    self.decode_and_publish(part)

        except zmq.Again:
            pass  # no message this tick

    def decode_and_publish(self, raw_bytes: bytes):
        # Decode into float64 array
        vals = np.frombuffer(raw_bytes, dtype=np.float64)
        self.get_logger().info(f"Decoded 472-byte packet into {len(vals)} floats")

        # Map values (example – you may need to reorder based on your dump)
        # Here we assume:
        #   vals[0:9]   = intrinsic matrix (row-major 3x3)
        #   vals[9:14]  = distortion coeffs (k1,k2,p1,p2,k3)
        #   vals[14:23] = rectification matrix (row-major 3x3)
        #   vals[23:35] = projection matrix (row-major 3x4)
        K = vals[0:9].tolist()
        D = vals[9:14].tolist()
        R = vals[14:23].tolist()
        P = vals[23:35].tolist()

        # Create CameraInfo msg
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = "camera_frame"
        info.height = 480
        info.width = 640

        info.k = K
        info.d = D
        info.r = R
        info.p = P
        info.distortion_model = "plumb_bob"

        self.info_pub.publish(info)
        self.get_logger().info("Published CameraInfo from 472-byte packet")

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

