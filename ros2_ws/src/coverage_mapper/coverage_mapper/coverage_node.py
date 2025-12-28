#!/usr/bin/env python3
import os
import time
import signal
import csv

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState

from coverage_mapper.coverage_core import CoverageParams, CoverageGrid


class CoverageNode(Node):
    def __init__(self):
        super().__init__("coverage_node")

        self.declare_parameter("mode", "test")
        self.declare_parameter("pose_topic", "/uav/pose")
        self.declare_parameter("pose_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("out_dir", os.path.expanduser("~/uav-coverage-sim/data/coverage"))
        self.declare_parameter("score_per_tick", 1.0)
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("battery_topic", "/battery/state")
        self.declare_parameter("battery_topics", Parameter.Type.STRING_ARRAY)

        self.p = CoverageParams(
            x_min=-200.0,
            x_max=200.0,
            y_min=-200.0,
            y_max=200.0,
            res=0.1,
            fov_deg=60.0,
            r_max=120.0,
        )
        self.grid = CoverageGrid(self.p)

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value

        pose_topics_param = self.get_parameter("pose_topics").get_parameter_value()
        self.pose_topics = list(pose_topics_param.string_array_value)
        if not self.pose_topics:
            raw = pose_topics_param.string_value
            if raw:
                self.pose_topics = [t.strip() for t in raw.split(",") if t.strip()]

        self.out_dir = self.get_parameter("out_dir").get_parameter_value().string_value
        self.score_per_tick = float(self.get_parameter("score_per_tick").get_parameter_value().double_value)
        self.tick_hz = float(self.get_parameter("tick_hz").get_parameter_value().double_value)
        self.battery_topic = self.get_parameter("battery_topic").get_parameter_value().string_value
        battery_topics_param = self.get_parameter("battery_topics").get_parameter_value()
        self.battery_topics = list(battery_topics_param.string_array_value)
        if not self.battery_topics:
            raw = battery_topics_param.string_value
            if raw:
                self.battery_topics = [t.strip() for t in raw.split(",") if t.strip()]

        self.t0 = time.time()
        self.last_t = self.t0

        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.multi_poses = {}
        self.battery_logs = {}

        topics = []
        if self.battery_topics:
            topics = self.battery_topics
        elif self.battery_topic:
            topics = [self.battery_topic]

        for topic in topics:
            self.battery_logs[topic] = []
            self.create_subscription(BatteryState, topic, self._make_battery_cb(topic), 10)

        if self.mode == "ros":
            self.sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
            self.get_logger().info(f"MODE=ros, listening PoseStamped on: {self.pose_topic}")
        elif self.mode == "ros_multi":
            if not self.pose_topics:
                self.pose_topics = [self.pose_topic]
            self.pose_topics = [t.strip() for t in self.pose_topics if t.strip()]
            for topic in self.pose_topics:
                self.multi_poses[topic] = None
                self.create_subscription(PoseStamped, topic, self._make_pose_cb(topic), 10)
            self.get_logger().info(f"MODE=ros_multi, topics: {self.pose_topics}")
        else:
            self.get_logger().info("MODE=test, using synthetic lawnmower trajectory")

        period = 1.0 / max(self.tick_hz, 1.0)
        self.timer = self.create_timer(period, self.step)

    def synthetic_pose(self, t: float):
        alt_m = 60.0
        v = 8.0
        lane_step = 20.0
        lane_len = 300.0
        start_x = -150.0
        start_y = -150.0

        lane_time = lane_len / v
        lane_idx = int(t // lane_time)
        tau = t - lane_idx * lane_time

        y = start_y + lane_idx * lane_step
        if lane_idx % 2 == 0:
            x = start_x + v * tau
        else:
            x = start_x + lane_len - v * tau

        z = alt_m
        return x, y, z

    def pose_cb(self, msg: PoseStamped):
        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        self.z = float(msg.pose.position.z)
        self.have_pose = True
        self.multi_poses[self.pose_topic] = (self.x, self.y, self.z)

    def _make_pose_cb(self, topic: str):
        def _cb(msg: PoseStamped):
            x = float(msg.pose.position.x)
            y = float(msg.pose.position.y)
            z = float(msg.pose.position.z)
            self.multi_poses[topic] = (x, y, z)
        return _cb

    def _make_battery_cb(self, topic: str):
        def _cb(msg: BatteryState):
            t = time.time() - self.t0
            self.battery_logs[topic].append((t, float(msg.voltage), float(msg.percentage)))
        return _cb

    def step(self):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now
        t = now - self.t0

        if self.mode == "ros":
            if not self.have_pose:
                if int(t) % 2 == 0 and abs(t - int(t)) < 0.06:
                    self.get_logger().info("Waiting for first pose...")
                return
            x, y, z = self.x, self.y, self.z
            self.grid.apply(x, y, z, dt)
        elif self.mode == "ros_multi":
            if not self.multi_poses:
                return
            score = self.score_per_tick if self.score_per_tick > 0.0 else 1.0
            for pose in self.multi_poses.values():
                if pose is None:
                    continue
                x, y, z = pose
                self.grid.apply(x, y, z, score)
        else:
            x, y, z = self.synthetic_pose(t)
            self.grid.apply(x, y, z, dt)

        if int(t) % 5 == 0 and abs(t - int(t)) < 0.06:
            if self.mode == "ros_multi":
                self.get_logger().info(f"t={t:6.1f}s poses={len([p for p in self.multi_poses.values() if p])}")
            else:
                self.get_logger().info(f"t={t:6.1f}s pose=({x:7.1f},{y:7.1f},{z:5.1f}) dt={dt:.3f}s")

    def save_outputs(self):
        os.makedirs(self.out_dir, exist_ok=True)
        if self.mode == "ros_multi":
            npy_path = os.path.join(self.out_dir, "coverage_score.npy")
            csv_path = os.path.join(self.out_dir, "coverage_score.csv")
        else:
            npy_path = os.path.join(self.out_dir, "coverage_time_s.npy")
            csv_path = os.path.join(self.out_dir, "coverage_time_s.csv")

        np.save(npy_path, self.grid.coverage)
        np.savetxt(csv_path, self.grid.coverage, delimiter=",", fmt="%.3f")

        self.get_logger().info(f"Saved: {npy_path}")
        self.get_logger().info(f"Saved: {csv_path}")

        if self.battery_logs:
            for topic, rows in self.battery_logs.items():
                if not rows:
                    continue
                safe = topic.strip("/").replace("/", "_")
                filename = f"battery_log_{safe}.csv"
                if len(self.battery_logs) == 1:
                    filename = "battery_log.csv"
                battery_csv = os.path.join(self.out_dir, filename)
                with open(battery_csv, "w", newline="") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(["t_s", "voltage_v", "percentage"])
                    writer.writerows(rows)
                self.get_logger().info(f"Saved: {battery_csv}")


def main():
    rclpy.init()
    node = CoverageNode()

    def _sigint(signum, frame):
        node.get_logger().info("Ctrl+C received -> saving coverage...")
        node.save_outputs()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.save_outputs()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
