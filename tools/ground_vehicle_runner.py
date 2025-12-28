#!/usr/bin/env python3
import argparse
import math
import os
import subprocess
import time


def parse_waypoints(spec: str, default_z: float):
    points = []
    chunks = [c.strip() for c in spec.split(";") if c.strip()]
    for idx, chunk in enumerate(chunks, start=1):
        parts = [p.strip() for p in chunk.split(",") if p.strip()]
        if len(parts) == 2:
            x, y = float(parts[0]), float(parts[1])
            z = default_z
        elif len(parts) == 3:
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        else:
            raise ValueError(f"Invalid waypoint #{idx}: '{chunk}'")
        points.append((x, y, z))
    return points


def parse_waypoints_file(path: str, default_z: float):
    points = []
    with open(path, "r", encoding="utf-8") as handle:
        for line_no, raw in enumerate(handle, start=1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            if ";" in line:
                try:
                    points.extend(parse_waypoints(line, default_z))
                except ValueError as exc:
                    raise ValueError(f"{path}:{line_no}: {exc}") from exc
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) == 2:
                x, y = float(parts[0]), float(parts[1])
                z = default_z
            elif len(parts) == 3:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            else:
                raise ValueError(f"{path}:{line_no}: invalid waypoint '{line}'")
            points.append((x, y, z))
    if not points:
        raise ValueError(f"{path}: no waypoints found")
    return points


def set_pose(world: str, model: str, x: float, y: float, z: float, yaw: float, env, verbose: bool):
    qw = math.cos(yaw * 0.5)
    qz = math.sin(yaw * 0.5)
    req = (
        f'name: "{model}" '
        f'position {{ x: {x} y: {y} z: {z} }} '
        f'orientation {{ w: {qw} x: 0 y: 0 z: {qz} }}'
    )
    cmd = [
        "gz",
        "service",
        "-s",
        f"/world/{world}/set_pose",
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "1000",
        "--req",
        req,
    ]
    if verbose:
        subprocess.run(cmd, env=env, check=False)
    else:
        subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def main():
    parser = argparse.ArgumentParser(description="Kinematic ground vehicle waypoint runner.")
    parser.add_argument("--world", default="multi_quad7")
    parser.add_argument("--model", default="ground_vehicle")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--z", type=float, default=0.15)
    parser.add_argument("--waypoints", type=str, default="")
    parser.add_argument("--waypoints-file", type=str, default="")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--loop", action="store_true")
    args = parser.parse_args()

    if args.waypoints and args.waypoints_file:
        raise ValueError("Use only one of --waypoints or --waypoints-file.")
    if not args.waypoints and not args.waypoints_file:
        raise ValueError("Provide --waypoints or --waypoints-file.")
    if args.speed <= 0.0:
        raise ValueError("--speed must be > 0")

    if args.waypoints_file:
        points = parse_waypoints_file(args.waypoints_file, default_z=args.z)
    else:
        points = parse_waypoints(args.waypoints, default_z=args.z)

    env = dict(os.environ)
    env.setdefault("GZ_CONFIG_PATH", "/usr/share/gz")

    dt = 1.0 / max(args.rate_hz, 1.0)
    last_yaw = 0.0

    while True:
        for idx in range(len(points) - 1):
            x0, y0, z0 = points[idx]
            x1, y1, z1 = points[idx + 1]
            dx = x1 - x0
            dy = y1 - y0
            dz = z1 - z0
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < 1e-6:
                continue
            yaw = math.atan2(dy, dx)
            last_yaw = yaw
            steps = max(1, int(dist / (args.speed * dt)))
            for step in range(steps + 1):
                alpha = step / steps
                x = x0 + dx * alpha
                y = y0 + dy * alpha
                z = z0 + dz * alpha
                set_pose(args.world, args.model, x, y, z, yaw, env, args.verbose)
                time.sleep(dt)
        if not args.loop:
            break

    xf, yf, zf = points[-1]
    set_pose(args.world, args.model, xf, yf, zf, last_yaw, env, args.verbose)


if __name__ == "__main__":
    main()
