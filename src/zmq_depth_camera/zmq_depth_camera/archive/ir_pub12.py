# zmq_detect_and_publish.py
import zmq, os, time, numpy as np, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Canonical sizes -> properties
CANDIDATES = {
    76800:  {"w": 320, "h": 240, "bpp": 1, "enc": "mono8",  "label": "confidence (8UC1)"},
    153600: {"w": 320, "h": 240, "bpp": 2, "enc": "16UC1",  "label": "depth 320x240 (16UC1)"},
    307200: {"w": 640, "h": 480, "bpp": 1, "enc": "mono8",  "label": "mono 640x480 (8UC1)"},
    614400: {"w": 640, "h": 480, "bpp": 2, "enc": "16UC1",  "label": "640x480 2Bpp (RGB565/16UC1)"},
}
TOLERANCE = 128  # allow small extra header/footer


class DetectPublisher(Node):
    def __init__(self, zmq_url="tcp://10.11.11.1:20000"):
        super().__init__("zmq_detect_pub")
        self.bridge = CvBridge()
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 5)
        self.ir_pub    = self.create_publisher(Image, '/camera/ir/image_raw', 5)
        self.conf_pub  = self.create_publisher(Image, '/camera/confidence/image_raw', 5)
        self.caminfo_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 5)

        self.caminfo = self.make_camera_info(320, 240)  # Default; will be adjusted per frame

        # ZMQ Setup
        ctx = zmq.Context()
        self.socket = ctx.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.socket.connect(zmq_url)

        # Timer to poll data every 20 ms
        self.timer = self.create_timer(0.02, self.poll_zmq)
        self.frame_counter = 0

    def make_camera_info(self, w, h):
        ci = CameraInfo()
        ci.width = w
        ci.height = h
        fx = 285.0; fy = 285.0; cx = w / 2.0; cy = h / 2.0
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

    def poll_zmq(self):
        try:
            parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        self.frame_counter += 1

        for i, part in enumerate(parts):
            L = len(part)
            matched = False

            for base, props in CANDIDATES.items():
                if L == base or abs(L - base) <= TOLERANCE:
                    matched = True
                    payload = part[-base:]
                    w, h, bpp, enc = props["w"], props["h"], props["bpp"], props["enc"]

                    try:
                        if bpp == 1:
                            arr = np.frombuffer(payload, dtype=np.uint8).reshape((h, w))
                        elif bpp == 2:
                            arr = np.frombuffer(payload, dtype=np.uint16).reshape((h, w))
                        else:
                            raise RuntimeError("Unsupported bpp")

                        msg = self.bridge.cv2_to_imgmsg(arr, encoding=enc)
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "camera_link"

                        # Update camera info
                        self.caminfo.header = msg.header
                        self.caminfo.width = w
                        self.caminfo.height = h
                        self.caminfo.k[2] = w / 2.0  # cx
                        self.caminfo.k[5] = h / 2.0  # cy
                        self.caminfo.p[2] = w / 2.0
                        self.caminfo.p[6] = h / 2.0

                        # Dispatch based on type
                        if base == 76800:
                            self.conf_pub.publish(msg)
                            self.get_logger().info(f"[{self.frame_counter}] Confidence mean: {np.mean(arr):.2f}")

                        elif base == 153600:
                            self.depth_pub.publish(msg)
                            self.caminfo_pub.publish(self.caminfo)

                        elif base == 307200:
                            self.ir_pub.publish(msg)

                        elif base == 614400:
                            # Optionally: convert to actual RGB
                            msg = self.bridge.cv2_to_imgmsg(arr, encoding="16UC1")
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = "camera_link"

                            if not hasattr(self, "raw_pub"):
                                self.raw_pub = self.create_publisher(Image, '/camera/raw/image_raw', 5)
                            self.raw_pub.publish(msg)

                        self.get_logger().info(f"[{self.frame_counter}] Published {props['label']} ({L} bytes, base {base})")

                    except Exception as e:
                        self.get_logger().warn(f"Decode failed for {props['label']} (base {base}): {e}")
                    break

            if not matched:
                fname = f"unknown_frame_{self.frame_counter}_part{i}_{L}.bin"
                with open(fname, "wb") as f:
                    f.write(part)
                self.get_logger().warn(f"Saved unknown part as {fname} (len={L})")


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

