#!/usr/bin/env python3
"""Run one scan simulator v2 mission.

Example:
  python3 tools/run_scan_simulation_v2.py --config configs/scan_v2_paper1.json --seed 3 --preset optimistic
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scan_sim_v2.outputs import clean_json, write_outputs  # noqa: E402
from scan_sim_v2.simulate import run_mission  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="Scan simulator v2 (Paper 1).")
    p.add_argument("--config", default="configs/scan_v2_paper1.json")
    p.add_argument("--out-dir", default="")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--preset", default=None, help="screening sensor preset (also used for the confirmer if it has it)")
    p.add_argument("--no-plots", action="store_true")
    args = p.parse_args()

    with open(args.config, "r", encoding="utf-8") as h:
        cfg = json.load(h)
    if args.seed is not None:
        cfg["seed"] = args.seed
    if args.preset:
        cfg.setdefault("team", {})["sensor_preset"] = args.preset

    out_root = args.out_dir or cfg.get("out_dir", "data/scan_v2_runs")
    out_dir = os.path.join(out_root, time.strftime("%Y%m%d_%H%M%S") + f"_seed{cfg.get('seed', 1)}")
    result = run_mission(cfg, record_paths=not args.no_plots)
    write_outputs(result, out_dir, plots=not args.no_plots)
    print(f"Saved scan v2 simulation to {out_dir}")
    m = {k: v for k, v in result["metrics"].items() if k != "calibration"}
    print(json.dumps(clean_json(m), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
