#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymavlink import mavutil


class ThrottleBridge(Node):
    def __init__(self):
        super().__init__("throttle_bridge")
        self.declare_parameter("udp", "udp:127.0.0.1:14540")
        self.declare_parameter("rate_hz", 10.0)

        self.udp = self.get_parameter("udp").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(Float32, "/battery/throttle", 10)

        self.get_logger().info(f"Connecting MAVLink: {self.udp}")
        self.master = mavutil.mavlink_connection(self.udp)
        self.get_logger().info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat OK.")

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def tick(self):
        msg = self.master.recv_match(type="VFR_HUD", blocking=False)
        throttle = 0.0
        if msg is not None and hasattr(msg, "throttle"):
            throttle = float(msg.throttle) / 100.0

        out = Float32()
        out.data = throttle
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ThrottleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
