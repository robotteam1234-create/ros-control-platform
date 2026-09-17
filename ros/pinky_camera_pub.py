#!/usr/bin/env python3
"""Pinky camera publisher (single file, zero build).

Copy to the robot and run -- it publishes JPEG frames that the control
platform already subscribes to via rosbridge (`camera_compressed` mapping).

Deploy:
    scp ros/pinky_camera_pub.py pinky@<robot-ip>:/home/pinky/
    ssh pinky@<robot-ip>  # then on the robot:
    source /opt/ros/jazzy/setup.bash
    ROS_DOMAIN_ID=12 python3 ~/pinky_camera_pub.py --topic /camera/image_raw/compressed --fps 10

Only std ROS (rclpy, sensor_msgs) + OpenCV are required. No colcon build.
"""

from __future__ import annotations

import argparse
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def open_camera(index: int, width: int, height: int):
    try:
        from picamera2 import Picamera2

        cam = Picamera2()
        cam.configure(cam.create_still_configuration(main={"size": (width, height)}))
        cam.start()
        return ("picamera2", cam)
    except Exception:
        pass
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera {index}")
    return ("v4l2", cap)


def read_frame(handle):
    kind, dev = handle
    if kind == "picamera2":
        return True, dev.capture_array()
    return dev.read()


def close_camera(handle) -> None:
    kind, dev = handle
    if kind == "picamera2":
        dev.stop()
    else:
        dev.release()


def encode_jpeg(frame, quality: int = 80) -> bytes:
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("jpeg encode failed")
    return buf.tobytes()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    rclpy.init()
    node = Node("pinky_camera_pub")
    pub = node.create_publisher(CompressedImage, args.topic, 10)
    cap = open_camera(args.camera, args.width, args.height)
    period = 1.0 / max(args.fps, 0.1)
    try:
        while rclpy.ok():
            ok, frame = read_frame(cap)
            if not ok:
                node.get_logger().warn("camera read failed, retrying")
                time.sleep(0.5)
                continue
            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = encode_jpeg(frame)
            pub.publish(msg)
            time.sleep(period)
    finally:
        close_camera(cap)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
