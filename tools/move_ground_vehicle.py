#!/usr/bin/env python3
import argparse
import math
import subprocess
import time


def set_pose(world: str, name: str, x: float, y: float, z: float, yaw: float, timeout_ms: int):
    req = (
        f'name: "{name}" '
        f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
        f'orientation {{ z: {math.sin(yaw * 0.5):.9f} w: {math.cos(yaw * 0.5):.9f} }}'
    )
    return subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            req,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )


def main():
    parser = argparse.ArgumentParser(description="Move the simple Gazebo ground vehicle in a straight line.")
    parser.add_argument("--world", default="multi_quad7")
    parser.add_argument("--name", default="ground_vehicle")
    parser.add_argument("--start-x", type=float, default=-20.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.3)
    parser.add_argument("--heading-deg", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=100.0)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    args = parser.parse_args()

    yaw = math.radians(args.heading_deg)
    vx = args.speed * math.cos(yaw)
    vy = args.speed * math.sin(yaw)
    dt = 1.0 / max(args.rate_hz, 1.0)
    t0 = time.monotonic()
    next_tick = t0

    print(
        f"Moving {args.name} in world {args.world}: "
        f"start=({args.start_x:.1f},{args.start_y:.1f},{args.z:.1f}), "
        f"heading={args.heading_deg:.1f} deg, speed={args.speed:.2f} m/s"
    )

    while True:
        now = time.monotonic()
        elapsed = now - t0
        if elapsed > args.duration:
            break

        x = args.start_x + vx * elapsed
        y = args.start_y + vy * elapsed
        result = set_pose(args.world, args.name, x, y, args.z, yaw, args.timeout_ms)
        if result.returncode != 0 and result.stderr:
            print(result.stderr.strip())

        next_tick += dt
        time.sleep(max(0.0, next_tick - time.monotonic()))


if __name__ == "__main__":
    main()
