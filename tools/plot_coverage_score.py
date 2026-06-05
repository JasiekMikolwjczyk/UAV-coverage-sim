#!/usr/bin/env python3
import argparse
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def vehicle_path_from_config(path: str):
    if not path:
        return None
    with open(path, "r", encoding="utf-8") as handle:
        cfg = json.load(handle)
    vehicle = cfg.get("vehicle")
    if not vehicle:
        return None

    start_x = float(vehicle.get("start_x", -20.0))
    start_y = float(vehicle.get("start_y", 0.0))
    speed = float(vehicle.get("speed", 1.0))
    duration = float(vehicle.get("duration", 100.0))
    heading = math.radians(float(vehicle.get("heading_deg", 0.0)))

    end_x = start_x + speed * duration * math.cos(heading)
    end_y = start_y + speed * duration * math.sin(heading)
    return np.array([[start_x, start_y], [end_x, end_y]], dtype=np.float64)


def grid_bounds_for_points(points, shape, x_min: float, y_min: float, res: float):
    if points is None or len(points) == 0:
        return None

    cols = np.floor((points[:, 0] - x_min) / res).astype(int)
    rows = np.floor((points[:, 1] - y_min) / res).astype(int)
    rows = np.clip(rows, 0, shape[0] - 1)
    cols = np.clip(cols, 0, shape[1] - 1)
    return rows.min(), cols.min(), rows.max(), cols.max()


def main():
    parser = argparse.ArgumentParser(description="Plot multi-UAV coverage score.")
    parser.add_argument(
        "--path",
        default="/home/janmikolajczyk/uav-coverage-sim/data/coverage/multi/coverage_score.npy",
    )
    parser.add_argument("--title", default="Multi-UAV coverage score")
    parser.add_argument("--out", default="")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--x-min", type=float, default=-200.0)
    parser.add_argument("--y-min", type=float, default=-200.0)
    parser.add_argument("--res", type=float, default=0.1)
    parser.add_argument("--vehicle-config", default="", help="Mission config JSON with a vehicle block to overlay.")
    parser.add_argument("--vehicle-label", default="ground vehicle path")
    args = parser.parse_args()

    cov = np.load(args.path)
    vehicle_path = vehicle_path_from_config(args.vehicle_config)

    nonzero = np.argwhere(cov > 0.0)
    r0, c0 = 0, 0
    r1, c1 = cov.shape[0] - 1, cov.shape[1] - 1
    have_crop_bounds = False
    if nonzero.size:
        r0, c0 = nonzero.min(axis=0)
        r1, c1 = nonzero.max(axis=0)
        have_crop_bounds = True

    vehicle_bounds = grid_bounds_for_points(vehicle_path, cov.shape, args.x_min, args.y_min, args.res)
    if vehicle_bounds is not None:
        vr0, vc0, vr1, vc1 = vehicle_bounds
        if have_crop_bounds:
            r0, c0 = min(r0, vr0), min(c0, vc0)
            r1, c1 = max(r1, vr1), max(c1, vc1)
        else:
            r0, c0, r1, c1 = vr0, vc0, vr1, vc1
            have_crop_bounds = True

    if have_crop_bounds:
        margin = 20
        r0 = max(r0 - margin, 0)
        c0 = max(c0 - margin, 0)
        r1 = min(r1 + margin, cov.shape[0] - 1)
        c1 = min(c1 + margin, cov.shape[1] - 1)
        cov = cov[r0 : r1 + 1, c0 : c1 + 1]

    plt.figure()
    x0 = args.x_min + c0 * args.res
    x1 = args.x_min + (c1 + 1) * args.res
    y0 = args.y_min + r0 * args.res
    y1 = args.y_min + (r1 + 1) * args.res
    plt.imshow(cov, origin="lower", extent=[x0, x1, y0, y1], aspect="equal")
    if vehicle_path is not None:
        plt.plot(
            vehicle_path[:, 0],
            vehicle_path[:, 1],
            color="white",
            linewidth=2.5,
            linestyle="--",
        )
        plt.scatter(vehicle_path[0, 0], vehicle_path[0, 1], color="lime", s=35, zorder=3)
        plt.scatter(vehicle_path[-1, 0], vehicle_path[-1, 1], color="red", s=35, zorder=3)
    plt.colorbar(label="coverage score")
    plt.title(args.title)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.tight_layout()

    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        plt.savefig(args.out, dpi=150)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
