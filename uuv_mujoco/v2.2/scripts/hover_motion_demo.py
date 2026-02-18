#!/usr/bin/env python3
"""Publish a gentle /cmd_vel pattern to demonstrate stable hover and movement."""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="UUV hover/motion demo publisher")
    p.add_argument("--topic", default="/cmd_vel", help="cmd_vel topic")
    p.add_argument("--hz", type=float, default=20.0, help="publish rate")
    p.add_argument("--scale", type=float, default=1.0, help="global motion scale")
    p.add_argument("--loop", action="store_true", help="repeat pattern forever")
    return p.parse_args()


class HoverMotionDemo(Node):
    def __init__(self, topic: str, hz: float, scale: float) -> None:
        super().__init__("hover_motion_demo")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.hz = max(1.0, float(hz))
        self.dt = 1.0 / self.hz
        self.scale = float(scale)

    def send(self, forward: float, sway: float, heave: float, yaw: float, seconds: float) -> None:
        msg = Twist()
        msg.linear.x = float(forward * self.scale)
        msg.linear.y = float(sway * self.scale)
        msg.linear.z = float(heave * self.scale)
        msg.angular.z = float(yaw * self.scale)
        end_t = time.monotonic() + max(0.0, float(seconds))
        while rclpy.ok() and time.monotonic() < end_t:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.dt)

    def stop(self, seconds: float = 1.0) -> None:
        self.send(0.0, 0.0, 0.0, 0.0, seconds)


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = HoverMotionDemo(args.topic, args.hz, args.scale)
    node.get_logger().info("starting hover-motion demo pattern")
    try:
        while rclpy.ok():
            # hover
            node.stop(4.0)
            # gentle forward
            node.send(forward=0.22, sway=0.0, heave=0.0, yaw=0.0, seconds=5.0)
            node.stop(3.0)
            # gentle right sway
            node.send(forward=0.0, sway=0.18, heave=0.0, yaw=0.0, seconds=4.0)
            node.stop(3.0)
            # slow yaw
            node.send(forward=0.0, sway=0.0, heave=0.0, yaw=0.20, seconds=4.0)
            node.stop(4.0)
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(0.8)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
