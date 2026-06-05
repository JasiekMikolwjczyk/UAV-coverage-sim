#!/usr/bin/env python3
import argparse
import json
import os
import signal
import subprocess
import sys
import time


def _fmt_ros_list(items):
    return "[" + ",".join(items) + "]"


def _fmt_num_list(items):
    return "[" + ", ".join(f"{v:.6f}" for v in items) + "]"


def _require_key(obj, key):
    if key not in obj or obj[key] in (None, ""):
        raise ValueError(f"Missing required key: {key}")
    return obj[key]


def _wait_for_file(path, max_wait=20):
    waited = 0
    last_size = -1
    stable = 0
    while waited < max_wait:
        if os.path.isfile(path):
            size = os.path.getsize(path)
            if size > 0:
                if size == last_size:
                    stable += 1
                else:
                    stable = 0
                last_size = size
                if stable >= 2:
                    return True
        time.sleep(0.5)
        waited += 1
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="Path to JSON config file.")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    out_dir = cfg.get("out_dir", os.path.join(root_dir, "data", "coverage", "multi"))
    tick_hz = float(cfg.get("tick_hz", 20.0))
    score_default = float(cfg.get("score_per_tick", 1.0))
    drones = cfg.get("drones", [])
    if not drones:
        raise ValueError("Config must include non-empty 'drones' list.")

    pose_topics = []
    pose_scores = []
    pose_fov = []
    pose_rmax = []
    pose_alt = []
    for drone in drones:
        name = drone.get("name", "")
        pose_topic = drone.get("pose_topic")
        if not pose_topic and name:
            pose_topic = f"/{name}/pose"
        pose_topic = _require_key({"pose_topic": pose_topic}, "pose_topic")
        pose_topics.append(pose_topic)
        pose_scores.append(float(drone.get("score_per_tick", score_default)))
        fov = drone.get("fov_deg")
        rmax = drone.get("r_max")
        alt = drone.get("coverage_alt")
        pose_fov.append(float(fov) if fov is not None else float("nan"))
        pose_rmax.append(float(rmax) if rmax is not None else float("nan"))
        pose_alt.append(float(alt) if alt is not None else float("nan"))

    pose_topics_arg = _fmt_ros_list([f"'{t}'" for t in pose_topics])
    pose_scores_arg = _fmt_num_list(pose_scores)
    pose_fov_arg = _fmt_num_list(pose_fov)
    pose_rmax_arg = _fmt_num_list(pose_rmax)
    pose_alt_arg = _fmt_num_list(pose_alt)

    venv_py = os.path.join(root_dir, ".venv", "bin", "python3")
    if not os.path.isfile(venv_py):
        raise RuntimeError("Missing .venv/bin/python3; create venv and install pymavlink first.")

    cov_cmd = (
        "set +u; "
        "source /opt/ros/jazzy/setup.bash; "
        f"source '{root_dir}/ros2_ws/install/setup.bash'; "
        "set -u; "
        "ros2 run coverage_mapper coverage_node --ros-args "
        "-p mode:=ros_multi "
        f"-p pose_topics:=\"{pose_topics_arg}\" "
        f"-p pose_topic_scores:=\"{pose_scores_arg}\" "
        f"-p pose_topic_fov_deg:=\"{pose_fov_arg}\" "
        f"-p pose_topic_r_max:=\"{pose_rmax_arg}\" "
        f"-p pose_topic_alt:=\"{pose_alt_arg}\" "
        f"-p out_dir:=\"{out_dir}\" "
        f"-p score_per_tick:={score_default:.6f} "
        f"-p tick_hz:={tick_hz:.6f}"
    )

    cov_proc = subprocess.Popen(["bash", "-lc", cov_cmd])

    vehicle_proc = None
    vehicle = cfg.get("vehicle")
    if vehicle:
        vehicle_cmd = [
            sys.executable,
            os.path.join(root_dir, "tools", "move_ground_vehicle.py"),
            "--world",
            str(vehicle.get("world", "multi_quad7")),
            "--name",
            str(vehicle.get("name", "ground_vehicle")),
            "--start-x",
            str(vehicle.get("start_x", -20.0)),
            "--start-y",
            str(vehicle.get("start_y", 0.0)),
            "--speed",
            str(vehicle.get("speed", 1.0)),
            "--duration",
            str(vehicle.get("duration", 100.0)),
        ]
        if "z" in vehicle:
            vehicle_cmd += ["--z", str(vehicle["z"])]
        if "heading_deg" in vehicle:
            vehicle_cmd += ["--heading-deg", str(vehicle["heading_deg"])]
        if "rate_hz" in vehicle:
            vehicle_cmd += ["--rate-hz", str(vehicle["rate_hz"])]
        vehicle_proc = subprocess.Popen(vehicle_cmd)

    procs = []
    for drone in drones:
        udp = _require_key(drone, "udp")
        cmd = [venv_py, os.path.join(root_dir, "tools", "waypoint_runner.py"), "--udp", udp]
        if drone.get("auto_arm", True):
            cmd.append("--auto-arm")
        if drone.get("auto_takeoff", True):
            cmd.append("--auto-takeoff")
        if drone.get("start_current", False):
            cmd.append("--start-current")
        if "speed" in drone:
            cmd += ["--speed", str(drone["speed"])]
        if "rate_hz" in drone:
            cmd += ["--rate-hz", str(drone["rate_hz"])]
        if "accept" in drone:
            cmd += ["--accept", str(drone["accept"])]
        if "alt" in drone:
            cmd += ["--alt", str(drone["alt"])]

        waypoints = drone.get("waypoints")
        waypoints_file = drone.get("waypoints_file")
        if waypoints and waypoints_file:
            raise ValueError("Use only one of waypoints or waypoints_file per drone.")
        if waypoints:
            cmd += ["--waypoints", waypoints]
        elif waypoints_file:
            cmd += ["--waypoints-file", waypoints_file]
        else:
            raise ValueError("Each drone must set waypoints or waypoints_file.")

        extra_args = drone.get("extra_args", [])
        if extra_args:
            cmd.extend(extra_args)

        procs.append(subprocess.Popen(cmd))

    interrupted = False

    def _handle_sigint(signum, frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, _handle_sigint)
    signal.signal(signal.SIGTERM, _handle_sigint)

    for proc in procs:
        try:
            proc.wait()
        except KeyboardInterrupt:
            interrupted = True
            break

    if cov_proc.poll() is None:
        cov_proc.send_signal(signal.SIGINT)
        try:
            cov_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            cov_proc.terminate()

    if vehicle_proc and vehicle_proc.poll() is None:
        vehicle_proc.send_signal(signal.SIGINT)
        try:
            vehicle_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            vehicle_proc.terminate()

    run_dir = os.path.join(root_dir, "data", "coverage", "multi_runs")
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_path = os.path.join(run_dir, timestamp)
    os.makedirs(run_path, exist_ok=True)
    with open(os.path.join(run_path, "config.json"), "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, sort_keys=True)

    cov_npy = os.path.join(out_dir, "coverage_score.npy")
    cov_csv = os.path.join(out_dir, "coverage_score.csv")
    if _wait_for_file(cov_npy):
        subprocess.run(["cp", "-f", cov_npy, os.path.join(run_path, "coverage_score.npy")], check=False)
        if os.path.isfile(cov_csv):
            subprocess.run(["cp", "-f", cov_csv, os.path.join(run_path, "coverage_score.csv")], check=False)
        subprocess.run(
            [
                "python3",
                os.path.join(root_dir, "tools", "plot_coverage_score.py"),
                "--path",
                os.path.join(run_path, "coverage_score.npy"),
                "--title",
                "Multi-UAV coverage score",
                "--out",
                os.path.join(run_path, "coverage_score.png"),
                "--vehicle-config",
                os.path.join(run_path, "config.json"),
                "--no-show",
            ],
            check=False,
        )
        print(f"Saved run artifacts to {run_path}")
    else:
        print(f"coverage_score.npy not found in {out_dir}")

    if interrupted:
        sys.exit(130)


if __name__ == "__main__":
    main()
