"""Mine populations (SPEC 4.5), clutter (SPEC 5) and safety injection (SPEC 4.5.4).

RNG streams (all np.random.default_rng):
  seed          natural mines (positions first, then per-mine type/depth)
  seed + 2000   clutter
  seed + 4000   injected mines
  seed + 5000   non-metal sensitivity relabelling
"""
import math

import numpy as np

from .model import SimObject, ScreenSensor, sv, load_json, DEFAULT_MINE_TYPES_PATH


def load_mine_types(path=None):
    return load_json(path or DEFAULT_MINE_TYPES_PATH)


def field_area(fld):
    return (float(fld["x_max"]) - float(fld["x_min"])) * (float(fld["y_max"]) - float(fld["y_min"]))


def mix_weights(mine_types, mix_name, override=None):
    if override:
        return {k: float(sv(v)) for k, v in override.items()}
    return {k: float(sv(v)) for k, v in mine_types["mixes"][mix_name]["weights"].items()}


def choose_type(rng, weights):
    keys = list(weights.keys())
    probs = np.array([weights[k] for k in keys], dtype=np.float64)
    probs = probs / probs.sum()
    return keys[int(rng.choice(len(keys), p=probs))]


def sample_depth(rng, mine_types, type_name, aged_shift=False, emplacement_override=None):
    t = mine_types["types"][type_name]
    model_name = "surface" if emplacement_override == "surface" else t["depth_model"]
    dm = mine_types["depth_models"][model_name]
    if dm["kind"] == "fixed":
        depth = float(sv(dm["depth"]))
    elif dm["kind"] == "uniform":
        depth = float(rng.uniform(float(sv(dm["low"])), float(sv(dm["high"]))))
    else:
        raise ValueError(f"unknown depth model kind {dm['kind']}")
    if aged_shift and model_name != "surface":
        depth += float(rng.uniform(0.0, 0.05))
    return depth


class TypeDepthSampler:
    """Draws (type, class, depth) for a regime, with residual hard-target selection."""

    def __init__(self, pop_cfg, mine_types, clearance_sensor: ScreenSensor | None):
        self.pop = pop_cfg
        self.mt = mine_types
        self.regime = pop_cfg.get("regime", "residual")
        self.aged = bool(pop_cfg.get("aged_depth_shift", False))
        self.clearance = clearance_sensor
        self.class_filter = pop_cfg.get("class_filter")
        if self.regime == "residual":
            self.weights = mix_weights(mine_types, pop_cfg.get("mix", "residual_mix"), pop_cfg.get("mix_weights"))
            self.selection = bool(pop_cfg.get("hard_target_selection", True))
        elif self.regime == "doctrinal":
            w_at = mix_weights(mine_types, "doctrinal_at")
            w_ap = mix_weights(mine_types, "doctrinal_ap")
            # For injection: AT and AP rows equally likely (assumption).
            self.weights = {**{k: 0.5 * v for k, v in w_at.items()}, **{k: 0.5 * v for k, v in w_ap.items()}}
            self.selection = False
        elif self.regime == "scatterable":
            self.weights = {pop_cfg.get("scatterable_type", "PFM-1"): 1.0}
            self.selection = False
        else:
            raise ValueError(f"unknown population regime {self.regime}")
        if self.class_filter:
            self.weights = {k: v for k, v in self.weights.items() if mine_types["types"][k]["class"] in self.class_filter}
            if not self.weights:
                raise ValueError("population.class_filter removes every mine type")

    def emplacement(self, type_name):
        t = self.mt["types"][type_name]
        if self.regime == "doctrinal":
            return t.get("doctrinal_emplacement", t["emplacement"])
        if self.regime == "scatterable":
            return "surface"
        return t["emplacement"]

    def keep_probability(self, cls, depth):
        if not self.selection or self.clearance is None:
            return 1.0
        return 1.0 - self.clearance.pd_class_depth(cls, depth)

    def draw(self, rng, weights=None, max_tries=100000):
        weights = weights or self.weights
        for _ in range(max_tries):
            tname = choose_type(rng, weights)
            cls = self.mt["types"][tname]["class"]
            depth = sample_depth(rng, self.mt, tname, self.aged, self.emplacement(tname))
            if self.selection:
                if rng.random() >= self.keep_probability(cls, depth):
                    continue
            return tname, cls, depth
        raise RuntimeError("hard-target selection rejected every candidate")


def _make_mine(idx, x, y, tname, cls, depth, injected=False, row_id=-1):
    return SimObject(idx=idx, x=float(x), y=float(y), is_mine=True, cls=cls, mine_type=tname,
                     depth_m=float(depth), injected=injected, row_id=row_id)


def generate_residual(cfg, sampler, rng):
    fld = cfg["field"]
    rho = float(cfg["population"].get("density_per_m2", 8.5e-7))
    n = int(rng.poisson(rho * field_area(fld)))
    xs = rng.uniform(float(fld["x_min"]), float(fld["x_max"]), size=n)
    ys = rng.uniform(float(fld["y_min"]), float(fld["y_max"]), size=n)
    mines = []
    for i in range(n):
        tname, cls, depth = sampler.draw(rng)
        mines.append(_make_mine(i, xs[i], ys[i], tname, cls, depth))
    return mines, {"density_per_m2": rho}


def generate_doctrinal(cfg, sampler, rng):
    """AT block of 3-4 rows then AP block; rows run across the lanes (FM3342)."""
    fld = cfg["field"]
    pop = cfg["population"]
    doc = pop.get("doctrinal", {})
    mult = float(pop.get("density_multiplier", 1.0))
    x_min, x_max = float(fld["x_min"]), float(fld["x_max"])
    y_min, y_max = float(fld["y_min"]), float(fld["y_max"])
    yc = 0.5 * (y_min + y_max)
    width = y_max - y_min
    angle = math.radians(float(rng.uniform(-15.0, 15.0)))
    n_at_rows = int(rng.integers(3, 5))
    n_ap_rows = int(doc.get("n_ap_rows", 3))
    at_row_spacing = doc.get("at_row_spacing_m", [20.0, 40.0])
    at_in_row = doc.get("at_in_row_spacing_m", [4.0, 6.0])
    ap_row_spacing = doc.get("ap_row_spacing_m", [5.0, 10.0])
    ap_in_row = float(doc.get("ap_in_row_spacing_m", 1.0))
    jitter = float(doc.get("jitter_std_m", 0.3))
    x0 = float(rng.uniform(x_min + float(doc.get("start_min_m", 10.0)), x_min + float(doc.get("start_max_m", 40.0))))
    w_at = mix_weights(sampler.mt, "doctrinal_at")
    w_ap = mix_weights(sampler.mt, "doctrinal_ap")
    if sampler.class_filter:
        w_at = {k: v for k, v in w_at.items() if sampler.mt["types"][k]["class"] in sampler.class_filter} or w_at
        w_ap = {k: v for k, v in w_ap.items() if sampler.mt["types"][k]["class"] in sampler.class_filter} or w_ap

    rows = []
    x_row = x0
    for r in range(n_at_rows):
        if r > 0:
            x_row += float(rng.uniform(*at_row_spacing))
        rows.append({"role": "AT", "x": x_row, "in_row": float(rng.uniform(*at_in_row)) / mult})
    for r in range(n_ap_rows):
        x_row += float(rng.uniform(*ap_row_spacing))
        rows.append({"role": "AP", "x": x_row, "in_row": ap_in_row / mult})

    # Row direction: lane-normal (y) rotated by angle; extend so the row crosses the field.
    dir_x, dir_y = math.sin(angle), math.cos(angle)
    half_len = 0.5 * width / max(abs(dir_y), 1e-6) + 2.0
    mines = []
    for rid, row in enumerate(rows):
        spacing = row["in_row"]
        offset = float(rng.uniform(0.0, spacing))
        s = -half_len + offset
        positions = []
        while s <= half_len:
            positions.append(s)
            s += spacing
        row["n_laid"] = len(positions)
        weights = w_at if row["role"] == "AT" else w_ap
        for s in positions:
            px = row["x"] + s * dir_x + float(rng.normal(0.0, jitter))
            py = yc + s * dir_y + float(rng.normal(0.0, jitter))
            tname = choose_type(rng, weights)
            cls = sampler.mt["types"][tname]["class"]
            depth = sample_depth(rng, sampler.mt, tname, False, sampler.emplacement(tname))
            if x_min <= px <= x_max and y_min <= py <= y_max:
                mines.append(_make_mine(len(mines), px, py, tname, cls, depth, row_id=rid))
    meta = {"rows": rows, "angle_rad": angle, "density_per_m2": len(mines) / field_area(fld)}
    return mines, meta


def generate_scatterable(cfg, sampler, rng):
    fld = cfg["field"]
    pop = cfg["population"]
    sc = pop.get("scatterable", {})
    rho = float(sc.get("density_per_m2", 0.002))
    n_fp = int(sc.get("n_footprints", 1))
    length_rng = sc.get("length_m", [150.0, 300.0])
    width_rng = sc.get("width_m", [35.0, 65.0])
    tname = pop.get("scatterable_type", "PFM-1")
    cls = sampler.mt["types"][tname]["class"]
    x_min, x_max = float(fld["x_min"]), float(fld["x_max"])
    y_min, y_max = float(fld["y_min"]), float(fld["y_max"])
    mines, footprints = [], []
    for f in range(n_fp):
        cx = float(rng.uniform(x_min, x_max))
        cy = float(rng.uniform(y_min, y_max))
        length = float(rng.uniform(*length_rng))
        width = float(rng.uniform(*width_rng))
        theta = float(rng.uniform(0.0, math.pi))
        a, b = 0.5 * length, 0.5 * width
        n = int(rng.poisson(rho * math.pi * a * b))
        r = np.sqrt(rng.uniform(0.0, 1.0, size=n))
        phi = rng.uniform(0.0, 2.0 * math.pi, size=n)
        u, v = a * r * np.cos(phi), b * r * np.sin(phi)
        px = cx + u * math.cos(theta) - v * math.sin(theta)
        py = cy + u * math.sin(theta) + v * math.cos(theta)
        footprints.append({"cx": cx, "cy": cy, "a": a, "b": b, "theta": theta, "n_dropped": n})
        for i in range(n):
            if x_min <= px[i] <= x_max and y_min <= py[i] <= y_max:
                mines.append(_make_mine(len(mines), px[i], py[i], tname, cls, 0.0, row_id=f))
    return mines, {"footprints": footprints, "density_per_m2": rho}


def generate_population(cfg, mine_types, clearance_sensor):
    """Natural mines for rate_based mode. Stream: default_rng(seed)."""
    pop = cfg["population"]
    rng = np.random.default_rng(int(cfg.get("seed", 1)))
    sampler = TypeDepthSampler(pop, mine_types, clearance_sensor)
    regime = sampler.regime
    if regime == "residual":
        mines, meta = generate_residual(cfg, sampler, rng)
    elif regime == "doctrinal":
        mines, meta = generate_doctrinal(cfg, sampler, rng)
    else:
        mines, meta = generate_scatterable(cfg, sampler, rng)
    frac = float(pop.get("non_metal_fraction", 0.0))
    if frac > 0.0:
        rng_nm = np.random.default_rng(int(cfg.get("seed", 1)) + 5000)
        for m in mines:
            if rng_nm.random() < frac:
                m.cls = "non_metal"
                m.mine_type = m.mine_type + "*nonmetal"
    meta["regime"] = regime
    return mines, meta, sampler


def generate_clutter(cfg, start_idx, generic_fa_density=None):
    """Clutter objects (SPEC 5). Stream: default_rng(seed + 2000).

    For the generic screener, false alarms replace clutter: density = false_alarms_per_m2,
    class 'false_alarm' (Pd 1 for that sensor).
    """
    fld = cfg["field"]
    rng = np.random.default_rng(int(cfg.get("seed", 1)) + 2000)
    cl = cfg.get("clutter", {})
    if generic_fa_density is not None:
        density = float(generic_fa_density)
        classes = {"false_alarm": 1.0}
        depth_spec = None
    else:
        density = float(cl.get("density_per_m2", 0.0))
        classes = cl.get("classes", {"metal_clutter": 0.8, "nonmetal_clutter": 0.2})
        depth_spec = cl.get("depth_m", {"mean": 0.05, "std": 0.05, "min": 0.0, "max": 0.3})
    if density <= 0.0:
        return []
    n = int(rng.poisson(density * field_area(fld)))
    xs = rng.uniform(float(fld["x_min"]), float(fld["x_max"]), size=n)
    ys = rng.uniform(float(fld["y_min"]), float(fld["y_max"]), size=n)
    keys = list(classes.keys())
    probs = np.array([float(sv(classes[k])) for k in keys], dtype=np.float64)
    probs = probs / probs.sum()
    cls_idx = rng.choice(len(keys), p=probs, size=n)
    if depth_spec is None:
        depths = np.zeros(n)
    else:
        depths = np.clip(rng.normal(float(depth_spec["mean"]), float(depth_spec["std"]), size=n),
                         float(depth_spec["min"]), float(depth_spec["max"]))
    return [
        SimObject(idx=start_idx + i, x=float(xs[i]), y=float(ys[i]), is_mine=False,
                  cls=keys[int(cls_idx[i])], mine_type=keys[int(cls_idx[i])], depth_m=float(depths[i]))
        for i in range(n)
    ]


def generate_injected(cfg, sampler, corridors, start_idx, n_inject):
    """n_inject mines at uniform positions inside the vehicle corridors. Stream: seed + 4000.

    corridors: list of (x_min, x_max, y_lo, y_hi), one per lane, chosen proportional to area.
    """
    rng = np.random.default_rng(int(cfg.get("seed", 1)) + 4000)
    areas = np.array([(c[1] - c[0]) * (c[3] - c[2]) for c in corridors], dtype=np.float64)
    out = []
    for i in range(n_inject):
        c = corridors[int(rng.choice(len(corridors), p=areas / areas.sum()))]
        x = float(rng.uniform(c[0], c[1]))
        y = float(rng.uniform(c[2], c[3]))
        tname, cls, depth = sampler.draw(rng)
        out.append(_make_mine(start_idx + i, x, y, tname, cls, depth, injected=True))
    frac = float(cfg["population"].get("non_metal_fraction", 0.0))
    if frac > 0.0:
        rng_nm = np.random.default_rng(int(cfg.get("seed", 1)) + 5001)
        for m in out:
            if rng_nm.random() < frac:
                m.cls = "non_metal"
                m.mine_type = m.mine_type + "*nonmetal"
    return out


def selection_class_depth_distribution(sampler, n_grid=200):
    """Expected class shares and mean depth of kept mines (for analytic model and tests)."""
    out = {}
    total = 0.0
    for tname, w in sampler.weights.items():
        t = sampler.mt["types"][tname]
        cls = t["class"]
        model_name = "surface" if sampler.emplacement(tname) == "surface" else t["depth_model"]
        dm = sampler.mt["depth_models"][model_name]
        if dm["kind"] == "fixed":
            depths = np.array([float(sv(dm["depth"]))])
        else:
            lo, hi = float(sv(dm["low"])), float(sv(dm["high"]))
            depths = lo + (np.arange(n_grid) + 0.5) / n_grid * (hi - lo)
        if sampler.aged and model_name != "surface":
            depths = (depths[:, None] + (np.arange(20) + 0.5)[None, :] / 20 * 0.05).ravel()
        keep = np.array([sampler.keep_probability(cls, d) for d in depths])
        mass = w * keep.mean()
        o = out.setdefault(cls, {"mass": 0.0, "depth_mass": 0.0})
        o["mass"] += mass
        o["depth_mass"] += w * float((keep * depths).mean())
        total += mass
    for cls, o in out.items():
        o["share"] = o["mass"] / total if total > 0 else 0.0
        o["mean_depth"] = o["depth_mass"] / o["mass"] if o["mass"] > 0 else 0.0
    return out
