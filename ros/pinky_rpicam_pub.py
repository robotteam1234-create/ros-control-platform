#!/usr/bin/env python3
"""Pinky Pi-camera publisher via rpicam-vid MJPEG pipe (single file, no build).

Why this exists: on the Pinky robots the pip picamera2 stack crashes
(libcamera IPA symbol mismatch) and the camera_ros node enumerates no
cameras, while the rpicam apps drive the OV5647 fine. So we let
``rpicam-vid`` own the sensor and parse its MJPEG byte stream into
``sensor_msgs/msg/CompressedImage``.

Deploy:
    scp ros/pinky_rpicam_pub.py pinky@<robot-ip>:/home/pinky/
    ssh pinky@<robot-ip>  # then on the robot:
    source /opt/ros/jazzy/setup.bash
    ROS_DOMAIN_ID=12 python3 ~/pinky_rpicam_pub.py --topic /camera/image_raw/compressed --fps 10

Only std ROS (rclpy, sensor_msgs) + stdlib are required. No colcon build,
no OpenCV, no picamera2.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


def mjpeg_frames(pipe, chunk: int = 65536):
    """Yield complete JPEG frames from an MJPEG byte stream."""
    buf = bytearray()
    while True:
        data = pipe.read(chunk)
        if not data:
            return
        buf.extend(data)
        while True:
            start = buf.find(SOI)
            if start < 0:
                buf.clear()
                break
            end = buf.find(EOI, start + 2)
            if end < 0:
                del buf[:start]
                break
            yield bytes(buf[start : end + 2])
            del buf[: end + 2]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    period = 1.0 / max(args.fps, 0.1)
    rclpy.init()
    node = Node("pinky_rpicam_pub")
    pub = node.create_publisher(CompressedImage, args.topic, 10)
    # rpicam-vid must NOT inherit the ROS LD_LIBRARY_PATH: /opt/ros libs
    # shadow its IPA plugins (undefined symbol crash). rclpy is already
    # imported in this process, so the child only needs a plain PATH.
    child_env = {k: v for k, v in os.environ.items() if k != "LD_LIBRARY_PATH"}
    child_env["PATH"] = "/usr/local/bin:/usr/bin:/bin"
    proc = subprocess.Popen(
        [
            # stdbuf defeats libc block-buffering on the pipe: without it
            # rpicam-vid holds encoded frames and the parser starves.
            "stdbuf", "-o0", "-e0",
            "rpicam-vid",
            "-t", "0",
            "--codec", "mjpeg",
            "--width", str(args.width),
            "--height", str(args.height),
            "--framerate", str(int(args.fps)),
            "--nopreview",
            "-o", "-",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        env=child_env,
    )
    assert proc.stdout is not None
    last = 0.0
    n = 0
    try:
        for jpeg in mjpeg_frames(proc.stdout):
            n += 1
            if n == 1 or n % 100 == 0:
                print(f"frames={n} size={len(jpeg)}", flush=True)
            now = time.time()
            if now - last < period:
                continue
            last = now
            if not jpeg.startswith(SOI):
                continue
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = jpeg
            pub.publish(msg)
    finally:
        proc.terminate()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
