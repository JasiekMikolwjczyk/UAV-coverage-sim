#!/usr/bin/env python3
import subprocess
import re
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

DEFAULT_TARGET = "x500_0"


def extract_pose(text: str, target: str):
    block = re.search(
        r'pose\s*\{\s*name:\s*"' + re.escape(target) + r'".*?\n\}',
        text,
        re.S
    )
    if not block:
        return None

    b = block.group(0)

    def g(k: str) -> float:
        m = re.search(rf'{k}\s*:\s*([-\deE\.]+)', b)
        return float(m.group(1)) if m else 0.0

    x = g("x")
    y = g("y")
    z = g("z")
    w = g("w")
    if abs(w) < 1e-12:
        w = 1.0

    return x, y, z, w


class GzPose(Node):
    def __init__(self):
        super().__init__("gz_pose")

        self.declare_parameter("gz_topic", "/world/default/pose/info")
        self.declare_parameter("target_entity", DEFAULT_TARGET)
        self.declare_parameter("out_topic", "/uav/pose")
        self.declare_parameter("rate_hz", 10.0)

        self.gz_topic = self.get_parameter("gz_topic").value
        self.target_entity = self.get_parameter("target_entity").value
        self.out_topic = self.get_parameter("out_topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(f"Reading Gazebo pose from: {self.gz_topic}")
        self.get_logger().info(f"Target entity: {self.target_entity}")
        self.get_logger().info(f"Publishing PoseStamped on: {self.out_topic}")

    def tick(self):
        proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.gz_topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )

        time.sleep(0.15)
        proc.terminate()

        try:
            out = proc.stdout.read() if proc.stdout else ""
        except Exception:
            out = ""

        pose = extract_pose(out, self.target_entity)
        if pose is None:
            return

        x, y, z, w = pose
        if abs(x) < 1e-12 and abs(y) < 1e-12 and abs(z) < 1e-12:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = w

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GzPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
