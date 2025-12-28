#!/usr/bin/env python3
import argparse
import os

import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Plot single-UAV coverage time.")
    parser.add_argument(
        "--path",
        default="/home/janmikolajczyk/uav-coverage-sim/data/coverage/coverage_time_s.npy",
    )
    parser.add_argument("--title", default="Coverage heatmap")
    parser.add_argument("--out", default="")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--x-min", type=float, default=-200.0)
    parser.add_argument("--y-min", type=float, default=-200.0)
    parser.add_argument("--res", type=float, default=0.1)
    args = parser.parse_args()

    cov = np.load(args.path)

    nonzero = np.argwhere(cov > 0.0)
    r0, c0 = 0, 0
    r1, c1 = cov.shape[0] - 1, cov.shape[1] - 1
    if nonzero.size:
        margin = 2
        r0, c0 = nonzero.min(axis=0)
        r1, c1 = nonzero.max(axis=0)
        r0 = max(r0 - margin, 0)
        c0 = max(c0 - margin, 0)
        r1 = min(r1 + margin, cov.shape[0] - 1)
        c1 = min(c1 + margin, cov.shape[1] - 1)
        cov = cov[r0 : r1 + 1, c0 : c1 + 1]

    x0 = args.x_min + c0 * args.res
    x1 = args.x_min + (c1 + 1) * args.res
    y0 = args.y_min + r0 * args.res
    y1 = args.y_min + (r1 + 1) * args.res

    plt.figure()
    plt.imshow(cov, origin="lower", extent=[x0, x1, y0, y1], aspect="equal")
    plt.colorbar(label="coverage time [s]")
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
