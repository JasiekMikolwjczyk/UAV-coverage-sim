#!/usr/bin/env python3
import csv
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.declare_parameter("log_path", os.path.expanduser("~/uav-coverage-sim/data/logs/battery.csv"))
        self.declare_parameter("rate_hz", 10.0)

        self.log_path = self.get_parameter("log_path").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.log_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["t", "throttle", "voltage", "percentage"])

        self.throttle = 0.0
        self.voltage = 12.6
        self.percentage = 1.0

        self.sub = self.create_subscription(Float32, "/battery/throttle", self.throttle_cb, 10)
        self.pub = self.create_publisher(BatteryState, "/battery/state", 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.get_logger().info(f"BatteryNode started. Logging to {self.log_path}")

    def throttle_cb(self, msg: Float32):
        self.throttle = float(msg.data)

    def tick(self):
        # podstawowy model zużycia baterii
        drain = 0.001 * self.throttle
        self.percentage = max(0.0, self.percentage - drain)
        self.voltage = 10.0 + 2.6 * self.percentage

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self.voltage
        msg.percentage = self.percentage
        self.pub.publish(msg)

        self.csv_writer.writerow([time.time(), self.throttle, self.voltage, self.percentage])
        self.log_file.flush()


def main():
    rclpy.init()
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.log_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
