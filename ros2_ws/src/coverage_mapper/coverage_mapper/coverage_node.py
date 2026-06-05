#!/usr/bin/env python3
import os
import time
import signal
import math
import csv

import numpy as np
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped

from coverage_mapper.coverage_core import CoverageParams, CoverageGrid


class CoverageNode(Node):
    def _parameter_value_or_none(self, name: str):
        try:
            return self.get_parameter(name).get_parameter_value()
        except ParameterUninitializedException:
            return None

    @staticmethod
    def _parse_string_array(value):
        if value is None:
            return []
        items = list(value.string_array_value)
        if items:
            return items
        raw = value.string_value
        if raw:
            return [t.strip() for t in raw.split(",") if t.strip()]
        return []

    @staticmethod
    def _parse_double_array(value):
        if value is None:
            return []
        items = list(value.double_array_value)
        if items:
            return items
        raw = value.string_value
        if raw:
            try:
                return [float(s.strip()) for s in raw.split(",") if s.strip()]
            except ValueError:
                return []
        return []

    def __init__(self):
        super().__init__("coverage_node")

        self.declare_parameter("mode", "test")
        self.declare_parameter("pose_topic", "/uav/pose")
        self.declare_parameter("pose_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("pose_topic_scores", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pose_topic_fov_deg", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pose_topic_r_max", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("pose_topic_alt", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("telemetry_dir", "")
        self.declare_parameter("out_dir", os.path.expanduser("~/uav-coverage-sim/data/coverage"))
        self.declare_parameter("score_per_tick", 1.0)
        self.declare_parameter("tick_hz", 20.0)

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

        self.pose_topics = self._parse_string_array(self._parameter_value_or_none("pose_topics"))
        pose_scores = self._parse_double_array(self._parameter_value_or_none("pose_topic_scores"))
        pose_fov = self._parse_double_array(self._parameter_value_or_none("pose_topic_fov_deg"))
        pose_rmax = self._parse_double_array(self._parameter_value_or_none("pose_topic_r_max"))
        pose_alt = self._parse_double_array(self._parameter_value_or_none("pose_topic_alt"))

        self.out_dir = self.get_parameter("out_dir").get_parameter_value().string_value
        self.telemetry_dir = self.get_parameter("telemetry_dir").get_parameter_value().string_value
        self.score_per_tick = float(self.get_parameter("score_per_tick").get_parameter_value().double_value)
        self.tick_hz = float(self.get_parameter("tick_hz").get_parameter_value().double_value)

        self.t0 = time.time()
        self.last_t = self.t0

        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.multi_poses = {}
        self.telemetry_files = {}
        self.telemetry_writers = {}
        self.telemetry_state = {}
        self.telemetry_ts = time.strftime("%Y%m%d_%H%M%S")

        if self.mode == "ros":
            self.sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
            self.get_logger().info(f"MODE=ros, listening PoseStamped on: {self.pose_topic}")
        elif self.mode == "ros_multi":
            if not self.pose_topics:
                self.pose_topics = [self.pose_topic]
            self.pose_topics = [t.strip() for t in self.pose_topics if t.strip()]
            self.pose_topic_scores = {}
            self.pose_topic_fov = {}
            self.pose_topic_rmax = {}
            self.pose_topic_alt = {}
            if pose_scores and len(pose_scores) != len(self.pose_topics):
                self.get_logger().warn(
                    "pose_topic_scores length does not match pose_topics; using default weights."
                )
                pose_scores = []
            if pose_fov and len(pose_fov) != len(self.pose_topics):
                self.get_logger().warn(
                    "pose_topic_fov_deg length does not match pose_topics; using defaults."
                )
                pose_fov = []
            if pose_rmax and len(pose_rmax) != len(self.pose_topics):
                self.get_logger().warn(
                    "pose_topic_r_max length does not match pose_topics; using defaults."
                )
                pose_rmax = []
            if pose_alt and len(pose_alt) != len(self.pose_topics):
                self.get_logger().warn(
                    "pose_topic_alt length does not match pose_topics; using pose z."
                )
                pose_alt = []
            if pose_scores:
                self.pose_topic_scores = dict(zip(self.pose_topics, pose_scores))
            else:
                self.pose_topic_scores = {t: 1.0 for t in self.pose_topics}
            if pose_fov:
                self.pose_topic_fov = dict(zip(self.pose_topics, pose_fov))
            else:
                self.pose_topic_fov = {t: math.nan for t in self.pose_topics}
            if pose_rmax:
                self.pose_topic_rmax = dict(zip(self.pose_topics, pose_rmax))
            else:
                self.pose_topic_rmax = {t: math.nan for t in self.pose_topics}
            if pose_alt:
                self.pose_topic_alt = dict(zip(self.pose_topics, pose_alt))
            else:
                self.pose_topic_alt = {t: math.nan for t in self.pose_topics}
            self._init_telemetry()
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
            default_score = self.score_per_tick if self.score_per_tick > 0.0 else 1.0
            for topic, pose in self.multi_poses.items():
                if pose is None:
                    continue
                score = self.pose_topic_scores.get(topic, default_score)
                x, y, z = pose
                self._log_telemetry(topic, x, y, z, t)
                fov = self.pose_topic_fov.get(topic, math.nan)
                rmax = self.pose_topic_rmax.get(topic, math.nan)
                alt = self.pose_topic_alt.get(topic, math.nan)
                self.grid.apply(
                    x,
                    y,
                    z,
                    score,
                    fov_deg=None if math.isnan(fov) else fov,
                    r_max=None if math.isnan(rmax) else rmax,
                    z_override=None if math.isnan(alt) else alt,
                )
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
        self._close_telemetry()

    def _init_telemetry(self):
        if self.telemetry_dir:
            base_dir = self.telemetry_dir
        else:
            base_dir = os.path.join(self.out_dir, "telemetry")
        os.makedirs(base_dir, exist_ok=True)
        for topic in self.pose_topics:
            safe = topic.strip("/").replace("/", "_")
            path = os.path.join(base_dir, f"{safe}_{self.telemetry_ts}.csv")
            f = open(path, "w", newline="")
            writer = csv.writer(f)
            writer.writerow([
                "t_s",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "speed",
                "ax",
                "ay",
                "az",
                "accel",
            ])
            self.telemetry_files[topic] = f
            self.telemetry_writers[topic] = writer
            self.telemetry_state[topic] = {
                "last_t": None,
                "last_x": None,
                "last_y": None,
                "last_z": None,
                "last_vx": 0.0,
                "last_vy": 0.0,
                "last_vz": 0.0,
            }
            self.get_logger().info(f"Telemetry: {path}")

    def _log_telemetry(self, topic: str, x: float, y: float, z: float, t: float):
        writer = self.telemetry_writers.get(topic)
        state = self.telemetry_state.get(topic)
        if writer is None or state is None:
            return
        last_t = state["last_t"]
        if last_t is None:
            state["last_t"] = t
            state["last_x"] = x
            state["last_y"] = y
            state["last_z"] = z
            return
        dt = t - last_t
        if dt <= 0.0:
            return
        dx = x - state["last_x"]
        dy = y - state["last_y"]
        dz = z - state["last_z"]
        vx = dx / dt
        vy = dy / dt
        vz = dz / dt
        ax = (vx - state["last_vx"]) / dt
        ay = (vy - state["last_vy"]) / dt
        az = (vz - state["last_vz"]) / dt
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        accel = math.sqrt(ax * ax + ay * ay + az * az)
        writer.writerow([f"{t:.3f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                         f"{vx:.6f}", f"{vy:.6f}", f"{vz:.6f}", f"{speed:.6f}",
                         f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}", f"{accel:.6f}"])
        state["last_t"] = t
        state["last_x"] = x
        state["last_y"] = y
        state["last_z"] = z
        state["last_vx"] = vx
        state["last_vy"] = vy
        state["last_vz"] = vz

    def _close_telemetry(self):
        for f in self.telemetry_files.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        self.telemetry_files = {}
        self.telemetry_writers = {}
        self.telemetry_state = {}


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
