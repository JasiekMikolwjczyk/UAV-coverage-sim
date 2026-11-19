"""metrics.json, mines.csv/json, queue_log.csv and per-run plots for scan simulator v2."""
import csv
import json
import math
import os

import numpy as np

from . import geometry as geo

QUEUE_LOG_COLUMNS = ["object_idx", "is_mine", "class", "depth_m", "t_arrival", "t_assigned", "t_done",
                     "confirmer_id", "travel_s", "scan_s", "in_corridor", "confirmed", "posterior"]

MINE_COLUMNS = ["idx", "x", "y", "material", "shape", "depth_m", "size_m", "sar_prob", "mag_prob", "final_prob",
                "suspect_x", "suspect_y", "mag_closest_m", "suspicious", "verified", "found",
                "is_mine", "class", "mine_type", "injected", "screen_detected", "confirm_result", "outcome",
                "t_vehicle_reach"]


def _fmt(v, nd=4):
    if v is None:
        return ""
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return ""
        return f"{v:.{nd}f}"
    return v


def clean_json(v):
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    if isinstance(v, dict):
        return {k: clean_json(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [clean_json(x) for x in v]
    if isinstance(v, (np.floating,)):
        return clean_json(float(v))
    if isinstance(v, (np.integer,)):
        return int(v)
    return v


def object_row(o):
    return [o.idx, _fmt(o.x, 3), _fmt(o.y, 3), o.material, o.shape, _fmt(o.depth_m, 3), _fmt(o.size_m, 3),
            _fmt(o.sar_prob), _fmt(o.mag_prob), _fmt(o.final_prob), _fmt(o.suspect_x, 3), _fmt(o.suspect_y, 3),
            _fmt(o.mag_closest_m, 3), int(o.suspicious), int(o.verified), int(o.found),
            int(o.is_mine), o.cls, o.mine_type, int(o.injected), int(o.screen_detected), o.confirm_result,
            o.outcome, _fmt(o.t_vehicle_reach, 2)]


def object_dict(o):
    keys = ["idx", "x", "y", "material", "shape", "depth_m", "size_m", "sar_prob", "mag_prob", "final_prob",
            "suspect_x", "suspect_y", "mag_closest_m", "sar_seen", "suspicious", "verified", "found", "blocked",
            "is_mine", "cls", "mine_type", "injected", "screen_detected", "confirm_result", "outcome",
            "t_arrival", "t_assigned", "t_done", "t_vehicle_reach", "posterior"]
    d = {k: clean_json(getattr(o, k)) for k in keys}
    d["class"] = d.pop("cls")
    return d


def write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        w = csv.writer(handle)
        w.writerow(header)
        w.writerows(rows)


def write_queue_log(path, rows):
    write_csv(path, QUEUE_LOG_COLUMNS, [[_fmt(r[k], 4) for k in QUEUE_LOG_COLUMNS] for r in rows])


def write_outputs(result, out_dir, plots=True):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "config.json"), "w", encoding="utf-8") as h:
        json.dump(result["cfg"], h, indent=2, sort_keys=True)
    with open(os.path.join(out_dir, "metrics.json"), "w", encoding="utf-8") as h:
        json.dump(clean_json(result["metrics"]), h, indent=2, sort_keys=True)
    objs = result["objects"]
    write_csv(os.path.join(out_dir, "mines.csv"), MINE_COLUMNS, [object_row(o) for o in objs])
    with open(os.path.join(out_dir, "mines.json"), "w", encoding="utf-8") as h:
        json.dump([object_dict(o) for o in objs], h, indent=2, sort_keys=True)
    if result.get("injection_objects"):
        write_csv(os.path.join(out_dir, "injected_mines.csv"), MINE_COLUMNS,
                  [object_row(o) for o in result["injection_objects"]])
    write_queue_log(os.path.join(out_dir, "queue_log.csv"), result["queue_log"])
    np.save(os.path.join(out_dir, "coverage.npy"), result["coverage"])
    if plots:
        plot_overview(result, os.path.join(out_dir, "scan_result.png"))


def plot_overview(result, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Rectangle

    cfg = result["cfg"]
    fld = cfg["field"]
    cov = result["coverage"]
    gx0, gx1, gy0, gy1 = geo.coverage_bounds(fld)
    fig, ax = plt.subplots(figsize=(15, 5.5))
    ax.set_facecolor("#d8d8d8")
    ax.imshow(cov, origin="lower", extent=[gx0, gx1, gy0, gy1], aspect="equal", alpha=0.6,
              vmin=0.0, vmax=1.0, cmap="Greens", zorder=1)
    ax.add_patch(Rectangle((fld["x_min"], fld["y_min"]), fld["x_max"] - fld["x_min"], fld["y_max"] - fld["y_min"],
                           facecolor="none", edgecolor="black", linewidth=1.2, zorder=5))
    c = result["ctx"]
    for ly in c.lanes:
        ax.axhspan(ly - c.corridor_half, ly + c.corridor_half, color="gold", alpha=0.12, zorder=2)
    for layer_paths in result["screener_paths"]:
        for path in layer_paths:
            if path:
                arr = np.array(path)
                ax.plot(arr[:, 0], arr[:, 1], color="deepskyblue", linewidth=0.6, alpha=0.7, zorder=6)
    if result["tractor_path"]:
        arr = np.array(result["tractor_path"])
        ax.plot(arr[:, 0], arr[:, 1], color="white", linewidth=2.2, zorder=8)
    for _, s, e, mode in result["mag_paths"]:
        col = {"scan": "magenta", "idle": "orchid"}.get(mode, "violet")
        ax.plot([s[0], e[0]], [s[1], e[1]], color=col, linewidth=0.8, alpha=0.7, zorder=7)
    marks = {"true_positive": ("o", "lime"), "screening_miss": ("x", "red"), "confirmation_miss": ("x", "orange"),
             "false_confirmation": ("s", "black"), "rejected": (".", "dimgray")}
    for outcome, (mk, col) in marks.items():
        pts = np.array([(o.x, o.y) for o in result["objects"] if o.outcome == outcome and not o.injected])
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], marker=mk, color=col, s=25 if mk != "." else 6, zorder=10)
    handles = [Line2D([0], [0], color="deepskyblue", label="screener paths"),
               Line2D([0], [0], color="white", linewidth=2.2, label="vehicle"),
               Line2D([0], [0], color="magenta", label="confirmer scan")]
    handles += [Line2D([0], [0], marker=mk, color=col, linestyle="None", label=k.replace("_", " "))
                for k, (mk, col) in marks.items()]
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    margin = float(fld.get("terrain_margin_m", 5.0))
    ax.set_xlim(fld["x_min"] - margin, fld["x_max"] + margin)
    ax.set_ylim(fld["y_min"] - margin, fld["y_max"] + margin)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    m = result["metrics"]
    ax.set_title(f"{m['screening_sensor']}/{m['sensor_preset']} N={m['n_screeners']} M={m['n_confirmers']} "
                 f"({m['detection_mode']}, {m['screening_mode']})" + ("  [projection]" if m.get("projection") else ""))
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)
