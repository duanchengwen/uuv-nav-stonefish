#!/usr/bin/env python3
"""Publish the two UUV camera streams from a running Stonefish episode to ROS 2.

The Stonefish app records `images/ego/%06d.png` and `images/wrist/%06d.png` at
10 Hz. This node watches those directories and republishes the newest frame on
standard ROS 2 image topics, so RViz2 (or image_view) can display them.

Run with the ROS 2 system Python:
    source /opt/ros/humble/setup.bash
    /usr/bin/python3 ros2/camera_publisher.py --episode-dir <episode-dir>
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


CAMERAS = {
    "ego": {
        "image_topic": "/bluerov2/camera_ego/image_raw",
        "info_topic": "/bluerov2/camera_ego/camera_info",
        "frame_id": "camera_ego_optical_frame",
    },
    "wrist": {
        "image_topic": "/bluerov2/camera_wrist/image_raw",
        "info_topic": "/bluerov2/camera_wrist/camera_info",
        "frame_id": "camera_wrist_optical_frame",
    },
}
HFOV_DEG = 100.0


class CameraPublisher(Node):
    def __init__(self, episode_dir: Path, poll_hz: float, min_age_s: float):
        super().__init__("uuv_camera_publisher")
        self.episode_dir = episode_dir
        self.min_age_s = min_age_s
        qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pubs = {}
        self.info_pubs = {}
        self.last_index = {}
        self.frames = {"ego": 0, "wrist": 0}
        self.latest = {"ego": None, "wrist": None}
        self.combined_pub = self.create_publisher(Image, "/bluerov2/cameras/combined", qos)
        self.last_combined = 0.0
        for kind, topics in CAMERAS.items():
            self.pubs[kind] = self.create_publisher(Image, topics["image_topic"], qos)
            self.info_pubs[kind] = self.create_publisher(CameraInfo, topics["info_topic"], qos)
            self.last_index[kind] = -1
            self.get_logger().info(f"{kind}: {topics['image_topic']}")
        self.get_logger().info("combined: /bluerov2/cameras/combined (ego | wrist, 640x240)")
        self.get_logger().info(f"watching {episode_dir}/images/{{ego,wrist}} at {poll_hz:.0f} Hz")
        self.timer = self.create_timer(1.0 / poll_hz, self.tick)
        self.last_warn = 0.0

    def newest_index(self, directory: Path) -> int:
        if not directory.is_dir():
            return -1
        newest = -1
        for path in directory.glob("*.png"):
            try:
                idx = int(path.stem)
            except ValueError:
                continue
            if idx > newest:
                newest = idx
        return newest

    def tick(self) -> None:
        now = time.time()
        for kind, topics in CAMERAS.items():
            directory = self.episode_dir / "images" / kind
            newest = self.newest_index(directory)
            if newest < 0:
                continue
            if newest < self.last_index[kind]:
                # A new episode started and the frame numbering restarted.
                self.last_index[kind] = -1
            if newest <= self.last_index[kind]:
                continue
            path = directory / f"{newest:06d}.png"
            try:
                if now - path.stat().st_mtime < self.min_age_s:
                    continue
            except FileNotFoundError:
                continue
            bgr = cv2.imread(str(path))
            if bgr is None:
                # The recorder is still writing this PNG; retry next tick.
                continue
            self.publish(kind, bgr, path)
            self.last_index[kind] = newest
            self.frames[kind] += 1
            if self.frames[kind] % 50 == 1:
                self.get_logger().info(
                    f"{kind}: frame {newest} {bgr.shape[1]}x{bgr.shape[0]} "
                    f"(published {self.frames[kind]})"
                )
        if self.frames["ego"] == 0 and now - self.last_warn > 5.0:
            self.last_warn = now
            self.get_logger().warning(
                f"no ego frames yet under {self.episode_dir}/images/ego "
                "(is the simulation running?)"
            )

    def publish(self, kind: str, bgr: np.ndarray, path: Path) -> None:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        h, w = rgb.shape[:2]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = CAMERAS[kind]["frame_id"]

        msg = Image()
        msg.header = header
        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = rgb.tobytes()
        self.pubs[kind].publish(msg)
        self.latest[kind] = rgb

        now = time.time()
        if (self.latest["ego"] is not None and self.latest["wrist"] is not None
                and now - self.last_combined >= 0.09):
            combined = np.ascontiguousarray(np.hstack([self.latest["ego"], self.latest["wrist"]]))
            cmsg = Image()
            cmsg.header.stamp = header.stamp
            cmsg.header.frame_id = "cameras_combined"
            cmsg.height = combined.shape[0]
            cmsg.width = combined.shape[1]
            cmsg.encoding = "rgb8"
            cmsg.is_bigendian = 0
            cmsg.step = combined.shape[1] * 3
            cmsg.data = combined.tobytes()
            self.combined_pub.publish(cmsg)
            self.last_combined = now

        info = CameraInfo()
        info.header = header
        info.width = w
        info.height = h
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = (w * 0.5) / np.tan(np.deg2rad(HFOV_DEG * 0.5))
        fy = fx
        cx = (w - 1) * 0.5
        cy = (h - 1) * 0.5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.binning_x = 0
        info.binning_y = 0
        self.info_pubs[kind].publish(info)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--episode-dir",
        default="episodes_raw/live_demo",
        help="directory containing images/ego and images/wrist (default: episodes_raw/live_demo)",
    )
    parser.add_argument("--poll-hz", type=float, default=20.0)
    parser.add_argument(
        "--min-age",
        type=float,
        default=0.03,
        help="ignore PNGs modified less than this many seconds ago (avoid partial writes)",
    )
    args = parser.parse_args()

    episode_dir = Path(args.episode_dir).resolve()
    rclpy.init()
    node = CameraPublisher(episode_dir, args.poll_hz, args.min_age)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
