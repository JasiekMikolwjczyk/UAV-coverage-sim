#!/usr/bin/env python3
"""Experiment sweep for scan simulator v2 (SPEC section 10).

  python3 tools/sweep_scan_v2.py --quick
  python3 tools/sweep_scan_v2.py --map-only
  python3 tools/sweep_scan_v2.py --groups primary --seeds 1-30 --workers 8 --no-plots

Groups (one-factor-at-a-time around the primary architecture, see docs/scan_sim_v2.md):
  primary        mag_uav x gpr presets x a x N x M (residual, clutter 0.6)
  projection     emi_uav ground_reference / eth_flight (+ GPR presets), labelled projection
  survey         gpr_uav_survey alone (M = 0)
  imaging        scatterable regime, mag_uav + gpr + thermal_uav surface layer
  regime         residual x10 density, doctrinal x{1,2}, scatterable PFM-1/POM-2, aged depth, non-metal 10%
  clutter        clutter density {0.1, 0.6, 1.0}
  vehicle        safe_distance {20, 60} x max_speed {0.5, 2.0}
  policy         priority / nearest / corridor_first
  map_validation generic screener on 20 cells of the requirements map
Outputs: runs.csv, summary.csv, queue_log_all.csv, analytic_vs_sim.csv, analytic_vs_sim_notes.txt,
requirements_map.csv, measured_points.csv, sweep_manifest.json and plots/. No interpretation is written.
"""
import argparse
import copy
import csv
import itertools
import json
import math
import os
import subprocess
import sys
import time
import warnings
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from scan_sim_v2.analytic import analyze, measured_points, requirements_map  # noqa: E402
from scan_sim_v2.model import load_sensor_library, sv  # noqa: E402
from scan_sim_v2.outputs import QUEUE_LOG_COLUMNS, clean_json  # noqa: E402
from scan_sim_v2.population import load_mine_types  # noqa: E402
from scan_sim_v2.simulate import run_mission  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BASE_CONFIG = os.path.join(REPO, "configs", "scan_v2_paper1.json")
SPEC_KEYS = ["group", "screener", "preset", "confirmer_preset", "surface", "a", "N", "M", "regime",
             "density", "density_multiplier", "scatterable_type", "aged", "non_metal_fraction", "clutter",
             "safe_distance", "max_speed", "policy", "gen_pd_metal", "gen_pd_low_metal", "gen_fa", "projection"]
SUMMARY_METRICS = ["mean_vehicle_speed_mps", "frontier_speed_mps", "fleet_energy_Wh", "energy_per_m2_cleared_Wh",
                   "false_confirmations", "confirmer_utilization", "n_suspicions", "elapsed_s",
                   "recall", "recall_metal", "recall_low_metal", "recall_non_metal",
                   "inj_p_detect", "inj_p_detect_metal", "inj_p_detect_low_metal", "inj_p_detect_non_metal",
                   "residual_risk_per_km", "mean_service_time_s", "arrival_rate_per_s", "mean_queue_wait_s"]

_LIB = None
_MT = None
_BASE = None


def _init(base_path):
    global _LIB, _MT, _BASE
    warnings.simplefilter("ignore")
    _LIB = load_sensor_library()
    _MT = load_mine_types()
    with open(base_path, "r", encoding="utf-8") as h:
        _BASE = json.load(h)


def spec(group, **kw):
    s = {k: None for k in SPEC_KEYS}
    s.update({"group": group, "screener": "mag_uav", "preset": "nominal", "confirmer_preset": "nominal",
              "surface": None, "a": 0.25, "N": 2, "M": 1, "regime": "residual", "density": 8.5e-7,
              "aged": False, "non_metal_fraction": 0.0, "clutter": 0.6, "safe_distance": 20.0,
              "max_speed": 2.0, "policy": "priority", "projection": False})
    s.update(kw)
    return s


def build_cfg(s, base, lib):
    cfg = copy.deepcopy(base)
    team = cfg["team"]
    team["screening_sensor"] = s["screener"]
    team["sensor_preset"] = s["preset"]
    team["confirmer_preset"] = s["confirmer_preset"]
    team["n_screeners"] = int(s["N"])
    team["n_confirmers"] = int(s["M"])
    team["confirmer_policy"] = s["policy"]
    if int(s["M"]) == 0:
        team["confirming_sensor"] = None
    if s["surface"]:
        team["surface_layer"] = {"sensor": s["surface"], "preset": "nominal", "n": 1}
    if s["screener"] == "generic_screen":
        swath = 0.25
        cfg["generic_screen"] = {"pd": {"metal": s["gen_pd_metal"], "low_metal": s["gen_pd_low_metal"], "non_metal": 0.0},
                                 "false_alarms_per_m2": s["gen_fa"], "swath_m": swath, "speed_mps": s["a"] / swath}
    elif s["a"] is not None:
        sensor = lib["sensors"][s["screener"]]
        pre = sensor["presets"].get(s["preset"], {})
        swath = float(sv(pre.get("swath_m", sensor.get("swath_m"))))
        team["screening_overrides"] = {"speed_mps": s["a"] / swath}
    pop = cfg["population"]
    if s["regime"] == "residual":
        pop.update({"regime": "residual", "density_per_m2": s["density"], "aged_depth_shift": bool(s["aged"])})
    elif s["regime"] == "doctrinal":
        doc = json.load(open(os.path.join(REPO, "configs", "scan_v2_doctrinal.json")))
        cfg["population"] = doc["population"]
        cfg["population"]["density_multiplier"] = s["density_multiplier"] or 1.0
        pop = cfg["population"]
    elif s["regime"] == "scatterable":
        sc = json.load(open(os.path.join(REPO, "configs", "scan_v2_scatterable.json")))
        cfg["population"] = sc["population"]
        cfg["population"]["scatterable_type"] = s["scatterable_type"] or "PFM-1"
        pop = cfg["population"]
    pop["non_metal_fraction"] = float(s["non_metal_fraction"] or 0.0)
    if s["screener"] == "emi_uav" and s["preset"] == "eth_flight":
        pop["class_filter"] = ["metal"]
    cfg["clutter"]["density_per_m2"] = float(s["clutter"])
    cfg["tractor"]["safe_distance_m"] = float(s["safe_distance"])
    cfg["tractor"]["max_speed_mps"] = float(s["max_speed"])
    return cfg


def run_one(job):
    s, seed = job
    cfg = build_cfg(s, _BASE, _LIB)
    cfg["seed"] = seed
    t0 = time.time()
    res = run_mission(cfg, _LIB, _MT, record_paths=False)
    m = res["metrics"]
    row = {k: s[k] for k in SPEC_KEYS}
    row["seed"] = seed
    row["projection"] = bool(m.get("projection"))
    for k, v in m.items():
        if isinstance(v, (dict, list)):
            continue
        row[k] = v
    row["wall_s"] = time.time() - t0
    qrows = [dict(r, seed=seed, **{k: s[k] for k in ("group", "screener", "preset", "confirmer_preset", "a", "N", "M")})
             for r in res["queue_log"]]
    return row, qrows


# --------------------------------------------------------------------------- grids

def parse_seeds(text):
    if "-" in text:
        a, b = text.split("-")
        return list(range(int(a), int(b) + 1))
    return [int(x) for x in text.split(",")]


def build_specs(groups, quick):
    if quick:
        return [spec("primary", N=N, M=M) for N in (2, 4) for M in (1, 2)]
    out = []
    A = [0.068, 0.25, 1.0]
    NS = [1, 2, 3, 4, 6, 8]
    MS = [1, 2, 3, 4]
    P = ["pessimistic", "nominal", "optimistic"]
    if "primary" in groups:
        out += [spec("primary", preset=p, confirmer_preset=cp, a=a, N=N, M=M)
                for p, cp, a, N, M in itertools.product(P, P, A, NS, MS)]
    if "projection" in groups:
        out += [spec("projection", screener="emi_uav", preset="ground_reference", confirmer_preset=cp, a=a, N=N, M=M,
                     projection=True) for cp, a, N, M in itertools.product(P, A, NS, MS)]
        out += [spec("projection", screener="emi_uav", preset="eth_flight", confirmer_preset=cp, a=0.068, N=N, M=M,
                     projection=True) for cp, N, M in itertools.product(P, NS, MS)]
    if "survey" in groups:
        out += [spec("survey", screener="gpr_uav_survey", a=a, N=N, M=0) for a, N in itertools.product(A, NS)]
    if "imaging" in groups:
        out += [spec("imaging", surface="thermal_uav", regime="scatterable", scatterable_type=t, N=N, M=M)
                for t, N, M in itertools.product(["PFM-1", "POM-2"], [2, 4], [1, 2])]
    if "regime" in groups:
        base = dict(N=4, M=2)
        out += [spec("regime", density=8.5e-6, **base),
                spec("regime", regime="doctrinal", density=None, density_multiplier=1.0, **base),
                spec("regime", regime="doctrinal", density=None, density_multiplier=2.0, **base),
                spec("regime", regime="scatterable", density=None, scatterable_type="PFM-1", **base),
                spec("regime", regime="scatterable", density=None, scatterable_type="POM-2", **base),
                spec("regime", aged=True, **base),
                spec("regime", non_metal_fraction=0.1, **base)]
    if "clutter" in groups:
        out += [spec("clutter", clutter=c, N=4, M=2) for c in (0.1, 0.6, 1.0)]
    if "vehicle" in groups:
        out += [spec("vehicle", safe_distance=d, max_speed=v, N=4, M=2) for d in (20.0, 60.0) for v in (0.5, 2.0)]
    if "policy" in groups:
        out += [spec("policy", policy=p, N=4, M=2) for p in ("priority", "nearest", "corridor_first")]
    if "map_validation" in groups:
        out += map_validation_specs()
    return out


def map_validation_specs(n=20):
    grid = list(itertools.product([0.1, 0.3, 0.5, 0.7, 0.9, 0.96], [0.9, 0.98], [0.1, 0.3, 0.6, 1.0],
                                  [0.068, 0.15, 0.25, 0.5, 1.0], [1, 2, 4, 8, 16]))
    idx = np.linspace(0, len(grid) - 1, n).round().astype(int)
    out = []
    for i in idx:
        pdl, pdm, fa, a, N = grid[i]
        out.append(spec("map_validation", screener="generic_screen", preset="nominal", gen_pd_metal=pdm,
                        gen_pd_low_metal=pdl, gen_fa=fa, a=a, N=N, M=2, clutter=0.0))
    return out


# --------------------------------------------------------------------------- aggregation

def key_of(row):
    return tuple(row.get(k) for k in SPEC_KEYS if k != "projection")


def tcrit(n):
    try:
        from scipy import stats
        return float(stats.t.ppf(0.975, n - 1))
    except Exception:
        return 1.96


def summarize(rows):
    groups = {}
    for r in rows:
        groups.setdefault(key_of(r), []).append(r)
    out = []
    for key, rs in groups.items():
        row = dict(zip([k for k in SPEC_KEYS if k != "projection"], key))
        row["projection"] = any(r.get("projection") for r in rs)
        row["n_seeds"] = len(rs)
        for mname in SUMMARY_METRICS:
            vals = np.array([r[mname] for r in rs if r.get(mname) is not None], dtype=float)
            n = vals.size
            row[f"{mname}_n"] = n
            row[f"{mname}_mean"] = float(vals.mean()) if n else None
            row[f"{mname}_std"] = float(vals.std(ddof=1)) if n > 1 else None
            if n > 1:
                h = tcrit(n) * vals.std(ddof=1) / math.sqrt(n)
                row[f"{mname}_ci_lo"] = float(vals.mean() - h)
                row[f"{mname}_ci_hi"] = float(vals.mean() + h)
            else:
                row[f"{mname}_ci_lo"] = row[f"{mname}_ci_hi"] = None
        out.append(row)
    return out


def analytic_vs_sim(summary, base, lib, mt):
    rows, notes = [], []
    for s in summary:
        sp = spec(s["group"], **{k: s[k] for k in SPEC_KEYS if k in s and k not in ("group",)})
        cfg = build_cfg(sp, base, lib)
        try:
            a = analyze(cfg, lib, mt)
        except Exception as exc:  # e.g. eth_flight unknown classes
            notes.append(f"{s['group']} {s['screener']}/{s['preset']} a={s['a']} N={s['N']} M={s['M']}: analytic not available ({exc})")
            continue
        comp = {
            "frontier_speed_mps": (a["frontier_speed_mps"], s.get("frontier_speed_mps_mean")),
            "vehicle_speed_mps": (a["predicted_vehicle_speed_mps"], s.get("mean_vehicle_speed_mps_mean")),
            "arrival_rate_per_s": (a["lambda_per_s"], s.get("arrival_rate_per_s_mean")),
            "mean_service_time_s": (a["E_S"] if s["M"] else None, s.get("mean_service_time_s_mean")),
            "utilization": (min(1.0, a["utilization"]) if s["M"] else None, s.get("confirmer_utilization_mean")),
            "inj_p_detect": (a["p_detect"], s.get("inj_p_detect_mean")),
        }
        row = {k: s[k] for k in ("group", "screener", "preset", "confirmer_preset", "a", "N", "M", "regime",
                                 "clutter", "policy", "safe_distance", "max_speed")}
        row["analytic_utilization_raw"] = a["utilization"]
        row["analytic_M_star"] = a["M_star"]
        flagged = []
        for name, (av, sm) in comp.items():
            row[f"{name}_analytic"] = av
            row[f"{name}_sim"] = sm
            if av is None or sm is None:
                row[f"{name}_rel_diff"] = None
                continue
            denom = max(abs(sm), 1e-12)
            rd = (av - sm) / denom
            row[f"{name}_rel_diff"] = rd
            if abs(rd) > 0.15:
                flagged.append((name, av, sm, rd))
        row["disagreement_gt_15pct"] = ",".join(f[0] for f in flagged)
        rows.append(row)
        if flagged:
            lines = [f"{f[0]}: analytic {f[1]:.4g} vs sim {f[2]:.4g} ({100 * f[3]:+.1f}%)" for f in flagged]
            diag = []
            if a["utilization"] >= 1.0:
                diag.append(f"analytic offered load per confirmer {a['utilization']:.3g} >= 1 (vehicle speed from the "
                            "service-throughput bound; finite 2x200 m mission includes start-up and lane-change transients)")
            elif s["M"]:
                diag.append(f"analytic offered load per confirmer {a['utilization']:.3g} < 1 (vehicle speed uses the "
                            "per-suspicion blocking-delay approximation, which ignores overlapping waits)")
            if any(f[0] == "inj_p_detect" for f in flagged):
                diag.append(f"injected mines per run = 20 x {s['n_seeds']} seeds; Wilson half-width at this n is "
                            "comparable to the difference")
            if any(f[0] == "arrival_rate_per_s" for f in flagged):
                diag.append("sim arrival rate counts suspicions over primary screening time only; analytic lambda is "
                            "steady-state at the frontier speed")
            notes.append(f"{s['group']} {s['screener']}/{s['preset']} gpr={s['confirmer_preset']} a={s['a']} N={s['N']} "
                         f"M={s['M']} regime={s['regime']} clutter={s['clutter']}:\n  " + "\n  ".join(lines)
                         + "\n  diagnostics: " + "; ".join(diag or ["see component columns in analytic_vs_sim.csv"]))
    return rows, notes


def write_rows(path, rows, columns=None):
    if not rows:
        with open(path, "w", encoding="utf-8") as h:
            h.write("")
        return
    cols = columns or sorted({k for r in rows for k in r})
    with open(path, "w", newline="", encoding="utf-8") as h:
        w = csv.DictWriter(h, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if r.get(k) is None else r.get(k)) for k in cols})


def git_commit():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    except Exception:
        return None


def git_dirty():
    try:
        return bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=REPO, text=True).strip())
    except Exception:
        return None


# --------------------------------------------------------------------------- plots

def make_plots(out_dir, rows, summary, avs, map_rows, points):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pdir = os.path.join(out_dir, "plots")
    os.makedirs(pdir, exist_ok=True)

    # 1. requirements map contours
    if map_rows:
        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        sub = [r for r in map_rows if r["pd_low_metal"] == 0.5 and r["pd_metal"] == 0.98
               and r["false_alarms_per_m2"] == 0.6 and r["M_rel"] == 0]
        Ns = sorted({r["N"] for r in sub})
        As = sorted({r["area_rate"] for r in sub})
        Z = np.array([[next(r["predicted_vehicle_speed_mps"] for r in sub if r["N"] == n and r["area_rate"] == a)
                       for n in Ns] for a in As])
        cs = axes[0].contourf(Ns, As, Z, levels=12, cmap="viridis")
        fig.colorbar(cs, ax=axes[0], label="predicted vehicle speed [m/s]")
        axes[0].set_xscale("log", base=2)
        axes[0].set_xlabel("screeners N")
        axes[0].set_ylabel("area rate per screener a [m²/s]")
        axes[0].set_title("Vehicle speed (Pd_LM 0.5, Pd_M 0.98, FA 0.6/m², M = M*)")
        for p in points:
            if p.get("area_rate"):
                axes[0].axhline(p["area_rate"], color="white", lw=0.6, ls=":")
                axes[0].text(Ns[0], p["area_rate"], f" {p['sensor']}/{p['preset']}", color="white", fontsize=7, va="bottom")
        sub2 = [r for r in map_rows if r["area_rate"] == 0.25 and r["N"] == 4 and r["pd_metal"] == 0.98 and r["M_rel"] == 0]
        P = sorted({r["pd_low_metal"] for r in sub2})
        F = sorted({r["false_alarms_per_m2"] for r in sub2})
        Z2 = np.array([[next(r["residual_risk_per_km"] for r in sub2 if r["pd_low_metal"] == p and r["false_alarms_per_m2"] == f)
                        for p in P] for f in F])
        cs2 = axes[1].contourf(P, F, Z2, levels=12, cmap="magma_r")
        fig.colorbar(cs2, ax=axes[1], label="residual risk [mines per km]")
        axes[1].set_xlabel("Pd_low_metal")
        axes[1].set_ylabel("false alarms per m²")
        axes[1].set_title("Residual risk (a 0.25, N 4, Pd_M 0.98, M = M*)")
        for p in points:
            if p.get("pd_low_metal") is not None:
                mk = "^" if p["projection"] else "o"
                axes[1].scatter([p["pd_low_metal"]], [p["false_alarms_per_m2"]], marker=mk, color="cyan", edgecolor="k", zorder=5)
                axes[1].annotate(f"{p['sensor']}/{p['preset']}" + (" (proj.)" if p["projection"] else ""),
                                 (p["pd_low_metal"], p["false_alarms_per_m2"]), fontsize=7, xytext=(3, 3),
                                 textcoords="offset points")
        axes[1].set_xlim(0, 1)
        fig.tight_layout()
        fig.savefig(os.path.join(pdir, "requirements_map.png"), dpi=140)
        plt.close(fig)

    if not summary:
        return
    # 2. vehicle speed vs N, primary nominal residual clutter 0.6
    prim = [s for s in summary if s["group"] == "primary" and s["preset"] == "nominal"
            and s["confirmer_preset"] == "nominal" and s["clutter"] == 0.6 and s["regime"] == "residual"]
    if prim:
        Ms = sorted({s["M"] for s in prim})
        fig, axes = plt.subplots(1, len(Ms), figsize=(5 * len(Ms), 4), squeeze=False)
        for ax, M in zip(axes[0], Ms):
            for a in sorted({s["a"] for s in prim}):
                ss = sorted([s for s in prim if s["M"] == M and s["a"] == a], key=lambda s: s["N"])
                if not ss:
                    continue
                Ns = [s["N"] for s in ss]
                line = ax.errorbar(Ns, [s["mean_vehicle_speed_mps_mean"] for s in ss],
                                   yerr=[(s["mean_vehicle_speed_mps_ci_hi"] - s["mean_vehicle_speed_mps_mean"])
                                         if s["mean_vehicle_speed_mps_ci_hi"] is not None else 0 for s in ss],
                                   marker="o", label=f"sim a={a}")
                an = [next((r["vehicle_speed_mps_analytic"] for r in avs if r["group"] == "primary" and r["preset"] == "nominal"
                            and r["confirmer_preset"] == "nominal" and r["a"] == a and r["N"] == n and r["M"] == M
                            and r["clutter"] == 0.6), None) for n in Ns]
                ax.plot(Ns, an, ls="--", color=line[0].get_color(), label=f"analytic a={a}")
            ax.set_yscale("log")
            ax.set_xlabel("screeners N")
            ax.set_ylabel("mean vehicle speed [m/s]")
            ax.set_title(f"mag_uav+gpr nominal, M={M}")
            ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(pdir, "vehicle_speed_vs_N.png"), dpi=140)
        plt.close(fig)

    # 3. utilisation vs lambda E[S] / M
    pts = [(r["arrival_rate_per_s"] * r["mean_service_time_s"] / r["n_confirmers"], r["confirmer_utilization"])
           for r in rows if r.get("n_confirmers") and r.get("arrival_rate_per_s") and r.get("mean_service_time_s")
           and r.get("confirmer_utilization") is not None]
    if pts:
        x, y = zip(*pts)
        fig, ax = plt.subplots(figsize=(6, 4.5))
        ax.scatter(x, y, s=8, alpha=0.5)
        ax.axvline(1.0, color="red", ls="--", label="ρ = 1")
        ax.set_xscale("log")
        ax.set_xlabel("λ·E[S]/M (measured)")
        ax.set_ylabel("confirmer utilisation (sim)")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(pdir, "utilization_vs_load.png"), dpi=140)
        plt.close(fig)

    # 4. injected P(detect before arrival) per class
    labels, vals = [], {c: [] for c in ("metal", "low_metal", "non_metal")}
    for s in sorted(summary, key=lambda s: (s["group"], str(s["screener"]), str(s["preset"]), str(s["confirmer_preset"]), str(s["regime"]))):
        labels.append(f"{s['group']}:{s['screener']}/{s['preset']}/{s['confirmer_preset']}/{s['regime']} a={s['a']} N={s['N']} M={s['M']}")
        for cls in vals:
            vals[cls].append(s.get(f"inj_p_detect_{cls}_mean"))
    if labels:
        nshow = min(len(labels), 60)
        fig, ax = plt.subplots(figsize=(max(8, 0.25 * nshow), 5))
        xs = np.arange(nshow)
        for i, cls in enumerate(vals):
            ax.bar(xs + (i - 1) * 0.27, [v if v is not None else 0 for v in vals[cls][:nshow]], width=0.27, label=cls)
        ax.set_xticks(xs)
        ax.set_xticklabels(labels[:nshow], rotation=90, fontsize=5)
        ax.set_ylabel("injected P(detect before vehicle arrival)")
        ax.set_ylim(0, 1)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(pdir, "injected_p_detect.png"), dpi=140)
        plt.close(fig)

    # 5. energy per m2 vs vehicle speed, Pareto front over (N, M)
    pr = [s for s in summary if s.get("energy_per_m2_cleared_Wh_mean") is not None and s.get("mean_vehicle_speed_mps_mean")]
    if pr:
        fig, ax = plt.subplots(figsize=(6.5, 4.5))
        x = np.array([s["mean_vehicle_speed_mps_mean"] for s in pr])
        y = np.array([s["energy_per_m2_cleared_Wh_mean"] for s in pr])
        ax.scatter(x, y, s=10, alpha=0.5, label="configurations")
        order = np.argsort(-x)
        front, best = [], math.inf
        for i in order:
            if y[i] < best:
                best = y[i]
                front.append(i)
        ax.plot(x[front], y[front], color="red", marker="o", ms=3, label="Pareto front")
        for i in front:
            ax.annotate(f"N{pr[i]['N']}M{pr[i]['M']}", (x[i], y[i]), fontsize=6)
        ax.set_xscale("log")
        ax.set_xlabel("mean vehicle speed [m/s]")
        ax.set_ylabel("fleet energy per m² cleared [Wh/m²]")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(pdir, "energy_vs_speed_pareto.png"), dpi=140)
        plt.close(fig)


# --------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Scan simulator v2 sweep (SPEC 10).")
    ap.add_argument("--config", default=BASE_CONFIG)
    ap.add_argument("--out-dir", default="")
    ap.add_argument("--groups", default="primary,projection,survey,imaging,regime,clutter,vehicle,policy,map_validation")
    ap.add_argument("--seeds", default="1-30")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--quick", action="store_true", help="primary, nominal, a=0.25, N in {2,4}, M in {1,2}, seeds 1-3")
    ap.add_argument("--map-only", action="store_true", help="analytic requirements map only, no simulation")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    warnings.simplefilter("ignore")
    out_dir = args.out_dir or os.path.join(REPO, "data", "scan_v2_sweeps", time.strftime("%Y%m%d_%H%M%S")
                                           + ("_quick" if args.quick else "_map" if args.map_only else ""))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.config, "r", encoding="utf-8") as h:
        base = json.load(h)
    lib = load_sensor_library()
    mt = load_mine_types()

    t0 = time.time()
    print("analytic requirements map ...", flush=True)
    map_rows = [clean_json(r) for r in requirements_map(base, lib, mt)]
    points = [clean_json(p) for p in measured_points(base, lib, mt)]
    write_rows(os.path.join(out_dir, "requirements_map.csv"), map_rows)
    write_rows(os.path.join(out_dir, "measured_points.csv"), points)

    groups = [g.strip() for g in args.groups.split(",") if g.strip()]
    seeds = [1, 2, 3] if args.quick else parse_seeds(args.seeds)
    specs = [] if args.map_only else build_specs(groups, args.quick)
    jobs = [(s, seed) for s in specs for seed in seeds]
    manifest = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "git_commit": git_commit(), "git_dirty": git_dirty(),
                "base_config": os.path.relpath(args.config, REPO), "base_config_content": base,
                "mode": "quick" if args.quick else "map_only" if args.map_only else "full",
                "groups": [] if args.quick or args.map_only else groups,
                "seeds": seeds, "n_specs": len(specs), "n_runs": len(jobs), "specs": specs,
                "map_grid": {"pd_low_metal": [0.1, 0.3, 0.5, 0.7, 0.9, 0.96], "pd_metal": [0.9, 0.98],
                             "false_alarms_per_m2": [0.1, 0.3, 0.6, 1.0], "area_rate": [0.068, 0.15, 0.25, 0.5, 1.0],
                             "N": [1, 2, 4, 8, 16], "M": "M* - 1, M*, M* + 1"}}
    with open(os.path.join(out_dir, "sweep_manifest.json"), "w", encoding="utf-8") as h:
        json.dump(clean_json(manifest), h, indent=2)

    rows, qrows = [], []
    if jobs:
        print(f"{len(jobs)} runs on {args.workers} workers ...", flush=True)
        with Pool(args.workers, initializer=_init, initargs=(args.config,)) as pool:
            for i, (row, q) in enumerate(pool.imap_unordered(run_one, jobs, chunksize=1), 1):
                rows.append(clean_json(row))
                qrows.extend(q)
                if i % max(1, len(jobs) // 20) == 0 or i == len(jobs):
                    print(f"  {i}/{len(jobs)} ({time.time() - t0:.0f} s)", flush=True)
        rows.sort(key=lambda r: (str(key_of(r)), r["seed"]))
    summary = [clean_json(s) for s in summarize(rows)]
    avs, notes = analytic_vs_sim(summary, base, lib, mt) if summary else ([], [])
    avs = [clean_json(r) for r in avs]
    if not args.map_only:
        write_rows(os.path.join(out_dir, "runs.csv"), rows)
        write_rows(os.path.join(out_dir, "summary.csv"), summary)
        write_rows(os.path.join(out_dir, "queue_log_all.csv"), qrows,
                   ["group", "screener", "preset", "confirmer_preset", "a", "N", "M", "seed"] + QUEUE_LOG_COLUMNS)
        write_rows(os.path.join(out_dir, "analytic_vs_sim.csv"), avs)
        with open(os.path.join(out_dir, "analytic_vs_sim_notes.txt"), "w", encoding="utf-8") as h:
            h.write("Cells where analytic and simulated values differ by more than 15%, with the model terms involved.\n"
                    "Approximations: see tools/scan_sim_v2/analytic.py docstring.\n\n")
            h.write("\n\n".join(notes) if notes else "No cell differs by more than 15%.\n")
    if not args.no_plots:
        make_plots(out_dir, rows, summary, avs, map_rows, points)
    print(f"done in {time.time() - t0:.0f} s -> {out_dir}")


if __name__ == "__main__":
    main()
