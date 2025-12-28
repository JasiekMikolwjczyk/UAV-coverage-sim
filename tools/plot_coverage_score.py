#!/usr/bin/env python3
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Plot multi-UAV coverage score.")
    parser.add_argument(
        "--path",
        default="/home/janmikolajczyk/uav-coverage-sim/data/coverage/multi/coverage_score.npy",
    )
    parser.add_argument("--title", default="Multi-UAV coverage score")
    parser.add_argument("--out", default="")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    cov = np.load(args.path)

    nonzero = np.argwhere(cov > 0.0)
    if nonzero.size:
        margin = 2
        r0, c0 = nonzero.min(axis=0)
        r1, c1 = nonzero.max(axis=0)
        r0 = max(r0 - margin, 0)
        c0 = max(c0 - margin, 0)
        r1 = min(r1 + margin, cov.shape[0] - 1)
        c1 = min(c1 + margin, cov.shape[1] - 1)
        cov = cov[r0 : r1 + 1, c0 : c1 + 1]

    plt.figure()
    plt.imshow(cov, origin="lower")
    plt.colorbar(label="coverage score")
    plt.title(args.title)
    plt.xlabel("grid x")
    plt.ylabel("grid y")
    plt.tight_layout()

    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        plt.savefig(args.out, dpi=150)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
