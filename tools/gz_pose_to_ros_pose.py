#!/usr/bin/env python3
import os
import subprocess
import re
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

DEFAULT_TARGET = "x500_0"


def _name_matches(target: str, name: str) -> bool:
    if name == target:
        return True
    return name.startswith(target + "::")


def _iter_pose_blocks(text: str):
    idx = 0
    while True:
        start = text.find("pose", idx)
        if start == -1:
            return
        brace = text.find("{", start)
        if brace == -1:
            return
        depth = 0
        i = brace
        while i < len(text):
            c = text[i]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    yield text[start:i + 1]
                    idx = i + 1
                    break
            i += 1
        else:
            return


def extract_pose(text: str, target: str):
    b = None
    for block in _iter_pose_blocks(text):
        m = re.search(r'name:\s*"([^"]+)"', block)
        if not m:
            continue
        name = m.group(1)
        if _name_matches(target, name):
            b = block
            break

    if b is None:
        return None

    def g(k: str, buf: str) -> float:
        m = re.search(rf'{k}\s*:\s*([-\deE\.]+)', buf)
        return float(m.group(1)) if m else 0.0

    pos_block = re.search(r"position\s*\{.*?\}", b, re.S)
    ori_block = re.search(r"orientation\s*\{.*?\}", b, re.S)
    pos_buf = pos_block.group(0) if pos_block else b
    ori_buf = ori_block.group(0) if ori_block else b

    x = g("x", pos_buf)
    y = g("y", pos_buf)
    z = g("z", pos_buf)
    w = g("w", ori_buf)
    if abs(w) < 1e-12:
        w = 1.0

    return x, y, z, w


class GzPose(Node):
    def __init__(self):
        super().__init__("gz_pose")

        if "GZ_IP" not in os.environ or not os.environ["GZ_IP"]:
            detected = self._detect_gz_ip()
            if detected:
                os.environ["GZ_IP"] = detected

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
        self.empty_ticks = 0
        self.last_err = ""

        self.get_logger().info(f"Reading Gazebo pose from: {self.gz_topic}")
        self.get_logger().info(f"Target entity: {self.target_entity}")
        self.get_logger().info(f"Publishing PoseStamped on: {self.out_topic}")
        if "GZ_IP" in os.environ and os.environ["GZ_IP"]:
            self.get_logger().info(f"Using GZ_IP={os.environ['GZ_IP']}")

    def _detect_gz_ip(self) -> str:
        try:
            for cmd in (["ip", "-4", "route", "get", "1.1.1.1"],
                        ["ip", "-4", "route", "get", "1"]):
                proc = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=0.5,
                    check=False,
                )
                out = (proc.stdout or "").strip()
                if out:
                    m = re.search(r"\bsrc\s+(\S+)", out)
                    if m:
                        return m.group(1)
        except Exception:
            pass

        try:
            proc = subprocess.run(
                ["hostname", "-I"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=0.5,
                check=False,
            )
            out = (proc.stdout or "").strip()
            if out:
                return out.split()[0]
        except Exception:
            pass

        return ""

    def tick(self):
        try:
            result = subprocess.run(
                ["gz", "topic", "-e", "-t", self.gz_topic, "-n", "1"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=0.6,
                check=False,
            )
            out = result.stdout or ""
            err = result.stderr or ""
        except Exception:
            out = ""
            err = ""

        if not out.strip():
            self.empty_ticks += 1
            if self.empty_ticks % max(int(self.rate_hz), 1) == 0:
                msg = "No data from gz topic. Check gz sim is running and the topic name."
                err = err.strip()
                if err and err != self.last_err:
                    msg = f"{msg} (gz err: {err[:160]})"
                    self.last_err = err
                self.get_logger().warn(msg)
            return
        self.empty_ticks = 0

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
