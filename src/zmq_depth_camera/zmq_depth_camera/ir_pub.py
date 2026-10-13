#!/usr/bin/env python3
"""
Backwards-compatible entrypoint for zmq_depth_camera ir_pub.
Directly invokes ZmqDepthCameraNode main().
"""
from zmq_depth_camera.camera_node import main

if __name__ == '__main__':
    main()
