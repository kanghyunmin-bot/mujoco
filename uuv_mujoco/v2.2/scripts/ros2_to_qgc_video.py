#!/usr/bin/env python3
"""Bridge ROS2 Image topic to QGroundControl UDP H264 video stream."""

import argparse
import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class RosToQgcVideo(Node):
    def __init__(
        self,
        topic: str,
        host: str,
        port: int,
        fps: float,
        bitrate_kbps: int,
        width: int,
        height: int,
    ) -> None:
        super().__init__("ros2_to_qgc_video")
        self.bridge = CvBridge()
        self.topic = topic
        self.host = host
        self.port = int(port)
        self.fps = max(1.0, float(fps))
        self.min_dt = 1.0 / self.fps
        self.bitrate_kbps = int(max(200, bitrate_kbps))
        self.target_width = int(width)
        self.target_height = int(height)
        self.writer: Optional[cv2.VideoWriter] = None
        self.last_send_wall = 0.0
        self.last_report_wall = time.monotonic()
        self.sent_frames = 0

        self.create_subscription(Image, self.topic, self._on_image, 5)
        self.get_logger().info(
            f"streaming {self.topic} -> udp://{self.host}:{self.port} "
            f"({self.fps:.1f}fps, {self.bitrate_kbps}kbps)"
        )

    def _build_pipeline(self, width: int, height: int) -> str:
        return (
            "appsrc is-live=true block=true format=time do-timestamp=true "
            f"! video/x-raw,format=BGR,width={width},height={height},framerate={int(round(self.fps))}/1 "
            "! videoconvert "
            f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={self.bitrate_kbps} key-int-max={max(2, int(round(self.fps)))} "
            "! h264parse "
            "! rtph264pay config-interval=1 pt=96 "
            f"! udpsink host={self.host} port={self.port} sync=false async=false"
        )

    def _open_writer(self, width: int, height: int) -> bool:
        pipeline = self._build_pipeline(width, height)
        self.writer = cv2.VideoWriter(
            pipeline,
            cv2.CAP_GSTREAMER,
            0,
            self.fps,
            (width, height),
            True,
        )
        if self.writer is None or not self.writer.isOpened():
            self.writer = None
            self.get_logger().error("failed to open GStreamer video pipeline")
            return False
        self.get_logger().info(f"video encoder started at {width}x{height}")
        return True

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_send_wall < self.min_dt:
            return
        self.last_send_wall = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        if self.target_width > 0 and self.target_height > 0:
            if frame.shape[1] != self.target_width or frame.shape[0] != self.target_height:
                frame = cv2.resize(
                    frame,
                    (self.target_width, self.target_height),
                    interpolation=cv2.INTER_LINEAR,
                )

        h, w = frame.shape[:2]
        if self.writer is None:
            if not self._open_writer(w, h):
                return

        self.writer.write(frame)
        self.sent_frames += 1

        report_now = time.monotonic()
        if report_now - self.last_report_wall >= 2.0:
            self.get_logger().info(f"stream alive: {self.sent_frames} frames sent")
            self.last_report_wall = report_now
            self.sent_frames = 0

    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.writer = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS2 image -> QGroundControl UDP H264 bridge")
    parser.add_argument("--topic", default="/stereo/left/image_raw", help="ROS2 image topic")
    parser.add_argument("--host", default="127.0.0.1", help="UDP target host")
    parser.add_argument("--port", type=int, default=5600, help="UDP target port")
    parser.add_argument("--fps", type=float, default=15.0, help="Output video FPS")
    parser.add_argument("--bitrate-kbps", type=int, default=2000, help="H264 bitrate (kbps)")
    parser.add_argument("--width", type=int, default=640, help="Output width")
    parser.add_argument("--height", type=int, default=360, help="Output height")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = RosToQgcVideo(
        topic=args.topic,
        host=args.host,
        port=args.port,
        fps=args.fps,
        bitrate_kbps=args.bitrate_kbps,
        width=args.width,
        height=args.height,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
