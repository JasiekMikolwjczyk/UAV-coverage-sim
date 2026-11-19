"""Objects, legacy (v1) generation and detection, rate-based sensors, fusion, power.

Legacy functions are copied from tools/run_scan_simulation.py with identical
arithmetic so that legacy_logistic mode reproduces v1 bit-for-bit.
"""
import json
import math
import os
import warnings
from dataclasses import dataclass

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_SENSORS_PATH = os.path.join(REPO_ROOT, "configs", "sensors_v2.json")
DEFAULT_MINE_TYPES_PATH = os.path.join(REPO_ROOT, "configs", "mine_types_v2.json")

MINE_CLASSES = ("metal", "low_metal", "non_metal")
CLUTTER_CLASSES = ("metal_clutter", "nonmetal_clutter", "false_alarm")
REFERENCE_DEPTH_M = 0.05

# Pre-drawn uniform columns per object (order-independent Bernoulli draws).
U_SCREEN, U_SURFACE, U_CONFIRM = 0, 1, 2
N_U_COLUMNS = 3


class UnusedConfigKeyWarning(UserWarning):
    pass


class DeprecatedConfigKeyWarning(UnusedConfigKeyWarning):
    """A renamed config key: still accepted and mapped, warned once."""


# --------------------------------------------------------------------------- helpers

def sv(node):
    """Unwrap a {value, source} leaf; plain values pass through."""
    if isinstance(node, dict) and "value" in node:
        return node["value"]
    return node


def src(node):
    if isinstance(node, dict) and "source" in node:
        return node["source"]
    return None


def sigmoid(v: float) -> float:
    return 1.0 / (1.0 + math.exp(-max(-60.0, min(60.0, v))))


def logit(p: float) -> float:
    p = min(max(p, 1e-12), 1.0 - 1e-12)
    return math.log(p / (1.0 - p))


def odds(p: float) -> float:
    return p / max(1e-6, 1.0 - p)


def combine_probabilities(screen_prob: float, confirm_prob: float, prior: float = 0.5):
    """Posterior from screening and confirming probability (D2).

    posterior_odds = odds(p_s) * odds(p_c) / odds(prior); equals v1 at prior 0.5.
    """
    o = odds(screen_prob) * odds(confirm_prob)
    if prior != 0.5:
        o = o / odds(prior)
    return o / (1.0 + o)


def load_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


# --------------------------------------------------------------------------- objects

@dataclass
class SimObject:
    idx: int
    x: float
    y: float
    is_mine: bool = True
    cls: str = "metal"            # metal / low_metal / non_metal / metal_clutter / nonmetal_clutter / false_alarm
    mine_type: str = ""
    depth_m: float = 0.0
    material: str = ""            # legacy
    shape: str = ""               # legacy
    size_m: float = math.nan      # legacy
    injected: bool = False
    row_id: int = -1
    # screening
    sar_prob: float = math.nan    # legacy screening score / rate-based Pd used
    screen_pd: float = math.nan
    sar_seen: bool = False
    screen_detected: bool = False
    detected_by: str = ""
    suspicious: bool = False
    suspect_x: float = math.nan
    suspect_y: float = math.nan
    lane_idx: int = -1
    in_corridor: bool = False
    t_arrival: float = math.nan
    # confirmation
    mag_prob: float = math.nan
    final_prob: float = math.nan
    posterior: float = math.nan
    mag_closest_m: float = math.nan
    verifying: bool = False
    verified: bool = False
    confirmed: bool = False
    confirm_result: str = ""
    found: bool = False
    blocked: bool = False
    block_state: str = ""         # "", "unresolved", "excavation"
    t_assigned: float = math.nan
    t_done: float = math.nan
    t_found: float = math.nan     # time the object became known as a mine (confirmation or investigation end)
    investigated: bool = False    # rejected in-corridor suspicion investigated on the ground (on_reject=investigate)
    confirmer_id: int = -1
    travel_s: float = math.nan
    scan_s: float = math.nan
    t_vehicle_reach: float = math.inf
    outcome: str = ""


# Legacy alias: v1 code and tests talk about "mines".
Mine = SimObject


# --------------------------------------------------------------------------- legacy v1

def choose_weighted(rng, weights: dict):
    keys = list(weights.keys())
    probs = np.array([float(sv(weights[k])) for k in keys], dtype=np.float64)
    probs = probs / probs.sum()
    return keys[int(rng.choice(len(keys), p=probs))]


def clipped_normal(rng, spec: dict):
    value = float(rng.normal(float(spec["mean"]), float(spec["std"])))
    return max(float(spec["min"]), min(float(spec["max"]), value))


def generate_mines_legacy(cfg: dict):
    """v1 generate_mines, identical RNG call order."""
    rng = np.random.default_rng(int(cfg.get("seed", 1)))
    fld = cfg["field"]
    dist = cfg["mine_distribution"]
    props = cfg["mine_properties"]

    count = int(dist["count"])
    mean = np.array(dist["mean"], dtype=np.float64)
    std = np.array(dist["std"], dtype=np.float64)
    mode = dist.get("mode", "gaussian_xy")
    if mode == "gaussian_x_uniform_y":
        points = np.column_stack(
            (
                rng.normal(float(mean[0]), float(std[0]), size=count),
                rng.uniform(float(fld["y_min"]), float(fld["y_max"]), size=count),
            )
        )
    elif mode == "uniform_xy":
        points = np.column_stack(
            (
                rng.uniform(float(fld["x_min"]), float(fld["x_max"]), size=count),
                rng.uniform(float(fld["y_min"]), float(fld["y_max"]), size=count),
            )
        )
    elif mode == "gaussian_xy":
        points = rng.normal(mean, std, size=(count, 2))
    else:
        raise ValueError(f"Unsupported mine_distribution.mode: {mode}")
    points[:, 0] = np.clip(points[:, 0], float(fld["x_min"]), float(fld["x_max"]))
    points[:, 1] = np.clip(points[:, 1], float(fld["y_min"]), float(fld["y_max"]))

    mines = []
    for idx, (x, y) in enumerate(points):
        material = choose_weighted(rng, props["materials"])
        shape = choose_weighted(rng, props["shapes"])
        depth = clipped_normal(rng, props["depth_m"])
        size = clipped_normal(rng, props["size_m"])
        mines.append(
            SimObject(
                idx=idx, x=float(x), y=float(y), is_mine=True,
                cls="metal" if material == "metal" else "non_metal",
                mine_type=f"legacy_{material}", material=material, shape=shape,
                depth_m=depth, size_m=size,
            )
        )
    return mines


def detection_probability(mine, model: dict):
    material_score = float(model["material"].get(mine.material, 0.0))
    shape_score = float(model.get("shape", {}).get(mine.shape, 0.0))
    value = (
        float(model.get("bias", 0.0))
        + material_score
        + shape_score
        + float(model.get("size_coeff", 0.0)) * mine.size_m
        - float(model.get("depth_coeff", 0.0)) * mine.depth_m
    )
    return sigmoid(value)


# --------------------------------------------------------------------------- power

def power_for_speed(model: dict, speed_mps: float, payload_active: bool = True):
    return (
        float(model["base_power_W"])
        + float(model["speed_quad_coeff"]) * speed_mps * speed_mps
        + (float(model.get("payload_power_W", 0.0)) if payload_active else 0.0)
    )


def consume_energy(energy_Wh: float, model: dict, speed_mps: float, dt_s: float, payload_active: bool = True):
    return energy_Wh + power_for_speed(model, speed_mps, payload_active) * dt_s / 3600.0


def battery_remaining_percent(energy_Wh: float, model: dict):
    return max(0.0, 100.0 * (1.0 - energy_Wh / float(model["capacity_Wh"])))


# --------------------------------------------------------------------------- rate-based sensors

def load_sensor_library(path=None):
    return load_json(path or DEFAULT_SENSORS_PATH)


def _num(v):
    return None if v is None else float(v)


class ScreenSensor:
    """Rate-based screening sensor: logit_depth, generic or imaging model."""

    def __init__(self, name: str, spec: dict, preset: str, overrides: dict | None = None):
        overrides = overrides or {}
        if preset not in spec["presets"]:
            if len(spec["presets"]) == 1:
                preset = next(iter(spec["presets"]))
            else:
                raise ValueError(f"sensor {name} has no preset {preset!r}; available {list(spec['presets'])}")
        pre = spec["presets"][preset]
        self.name = name
        self.preset = preset
        self.model = spec["model"]
        self.role = spec["role"]
        self.label = spec.get("label", name)
        self.projection = bool(spec.get("projection", False))
        self.calibration = {}

        def pick(key, default=None):
            if key in overrides:
                self.calibration[key] = {"value": overrides[key], "source": "config override (sweep parameter)"}
                return overrides[key]
            node = pre.get(key, spec.get(key))
            if node is None:
                return default
            self.calibration[key] = {"value": sv(node), "source": src(node)}
            return sv(node)

        self.swath_m = float(pick("swath_m", 1.0))
        self.speed_mps = float(pick("speed_mps", 1.0))
        self.overlap = float(pick("transverse_overlap", 0.0))
        self.localization_std_m = float(pick("localization_std_m", 0.15))

        if self.model == "logit_depth":
            self.a = {}
            for cls, node in pre["a"].items():
                self.a[cls] = _num(sv(node))
                self.calibration[f"a.{cls}"] = {"value": sv(node), "source": src(node)}
            self.k_depth = float(pick("k_depth"))
            self.max_depth_m = _num(pick("max_depth_m"))
            self.clutter_map = dict(spec.get("clutter_class_map", {}))
            self.clutter_a = {}
            for cls, node in pre.get("clutter_a", {}).items():
                self.clutter_a[cls] = _num(sv(node))
                self.calibration[f"clutter_a.{cls}"] = {"value": sv(node), "source": src(node)}
            for k in ("metal", "low_metal", "non_metal"):
                if f"pd.{k}" in overrides:
                    self.a[k] = logit(float(overrides[f"pd.{k}"]))
        elif self.model == "generic":
            pd = {}
            for cls, node in pre["pd"].items():
                pd[cls] = float(sv(node))
                self.calibration[f"pd.{cls}"] = {"value": sv(node), "source": src(node)}
            pd.update({k: float(v) for k, v in overrides.get("pd", {}).items()})
            if "pd" in overrides:
                self.calibration["pd_override"] = {"value": overrides["pd"], "source": "config override (requirements map)"}
            self.pd_table = pd
            self.false_alarms_per_m2 = float(pick("false_alarms_per_m2"))
        elif self.model == "imaging":
            self.pd_surface = float(pick("pd_surface"))
            self.pd_buried = float(pick("pd_buried"))
            self.surface_depth_max_m = float(pick("surface_depth_max_m", 0.01))
            self.crossover_hours = pick("crossover_hours", []) or []
        else:
            raise ValueError(f"unknown screening model {self.model}")

    @property
    def area_rate(self):
        return self.swath_m * self.speed_mps

    def logit_pd(self, a, depth_m):
        return a - self.k_depth * (depth_m - REFERENCE_DEPTH_M) / 0.10

    def pd(self, obj, hour: float = 12.0) -> float:
        return self.pd_class_depth(obj.cls, obj.depth_m, hour)

    def pd_class_depth(self, cls: str, depth_m: float, hour: float = 12.0) -> float:
        depth = max(0.0, depth_m)
        if self.model == "generic":
            if cls == "false_alarm":
                return 1.0
            if cls in ("metal_clutter", "nonmetal_clutter"):
                return 0.0
            return self.pd_table.get(cls, 0.0)
        if self.model == "imaging":
            if self.crossover_hours:
                h = hour % 24.0
                for lo, hi in self.crossover_hours:
                    if lo <= h <= hi:
                        return 0.0
            if cls == "false_alarm":
                return 0.0
            return self.pd_surface if depth <= self.surface_depth_max_m + 1e-12 else self.pd_buried
        # logit_depth
        if cls == "false_alarm":
            return 0.0
        if self.max_depth_m is not None and depth > self.max_depth_m:
            return 0.0
        if cls in self.clutter_a:
            a = self.clutter_a[cls]
        elif cls in self.clutter_map:
            a = self.a.get(self.clutter_map[cls])
        else:
            a = self.a.get(cls)
        if a is None:
            raise ValueError(
                f"sensor {self.name}/{self.preset}: Pd for class {cls!r} is unknown (not simulated); "
                "restrict the population with population.class_filter"
            )
        return sigmoid(self.logit_pd(a, depth))

    def known_classes(self):
        if self.model == "logit_depth":
            return [c for c in MINE_CLASSES if self.a.get(c) is not None]
        return list(MINE_CLASSES)


class ConfirmSensor:
    def __init__(self, name: str, spec: dict, preset: str, overrides: dict | None = None):
        overrides = overrides or {}
        if preset not in spec["presets"]:
            raise ValueError(f"confirming sensor {name} has no preset {preset!r}")
        pre = spec["presets"][preset]
        self.name = name
        self.preset = preset
        self.label = spec.get("label", name)
        self.calibration = {}
        self.p_confirm = {}
        for cls, node in pre["p_confirm"].items():
            self.p_confirm[cls] = float(sv(node))
            self.calibration[f"p_confirm.{cls}"] = {"value": sv(node), "source": src(node)}
        for key in ("k_depth", "max_depth_m", "spatial_decay_m", "swath_m", "scan_speed_mps",
                    "transit_speed_mps", "min_scan_side_m", "transverse_overlap"):
            node = overrides.get(key, spec.get(key))
            self.calibration[key] = {"value": sv(node), "source": src(node) or "config override"}
        self.k_depth = float(sv(overrides.get("k_depth", spec["k_depth"])))
        self.max_depth_m = _num(sv(overrides.get("max_depth_m", spec["max_depth_m"])))
        self.geom = {
            key: float(sv(overrides.get(key, spec[key])))
            for key in ("spatial_decay_m", "swath_m", "scan_speed_mps", "transit_speed_mps",
                        "min_scan_side_m", "transverse_overlap")
        }

    def p_class_depth(self, cls: str, depth_m: float) -> float:
        key = "clutter" if cls in CLUTTER_CLASSES else cls
        p = self.p_confirm[key]
        depth = max(0.0, depth_m)
        if self.max_depth_m is not None and depth > self.max_depth_m:
            return 0.0
        return sigmoid(logit(p) - self.k_depth * (depth - REFERENCE_DEPTH_M) / 0.10)

    def p(self, obj) -> float:
        return self.p_class_depth(obj.cls, obj.depth_m)


def build_screen_sensor(lib: dict, name: str, preset: str, overrides=None):
    spec = lib["sensors"][name]
    if spec["role"] not in ("screen", "surface"):
        raise ValueError(f"{name} is not a screening sensor")
    return ScreenSensor(name, spec, preset, overrides)


def build_confirm_sensor(lib: dict, name: str, preset: str, overrides=None):
    spec = lib["sensors"][name]
    if spec["role"] != "confirm":
        raise ValueError(f"{name} is not a confirming sensor")
    return ConfirmSensor(name, spec, preset, overrides)


def platform_block(lib: dict, cfg: dict, name: str):
    custom = cfg.get("battery", {}).get("platforms", {})
    if name in custom:
        return custom[name]
    return lib["platforms"][name]


def bernoulli(u: float, p: float) -> bool:
    """Single detection draw. u is the object's pre-drawn uniform."""
    return u < p


# --------------------------------------------------------------------------- config warnings

def warn_unused_keys(cfg: dict):
    """D3/D4: warn once per key present but unused."""
    seen = set()

    def warn(key, why):
        if key not in seen:
            seen.add(key)
            warnings.warn(f"config key {key} is not used by scan_sim_v2 ({why})", UnusedConfigKeyWarning, stacklevel=3)

    dec = cfg.get("decision", {})
    if "excavation_time_s" in dec:
        key = "decision.excavation_time_s"
        if key not in seen:
            seen.add(key)
            warnings.warn("config key decision.excavation_time_s is deprecated; use decision.investigation_time_s "
                          "(value mapped)", DeprecatedConfigKeyWarning, stacklevel=3)
    if "sar_suspicious_max" in dec:
        warn("decision.sar_suspicious_max", "D3: never implemented in v1")
    if "cleared_threshold" in dec:
        warn("decision.cleared_threshold", "D3: never implemented in v1")
    dist = cfg.get("mine_distribution", {})
    if cfg.get("detection_mode", "legacy_logistic") == "legacy_logistic" and dist.get("mode") == "gaussian_x_uniform_y":
        if len(dist.get("mean", [])) > 1:
            warn("mine_distribution.mean[1]", "D4: ignored by gaussian_x_uniform_y")
        if len(dist.get("std", [])) > 1:
            warn("mine_distribution.std[1]", "D4: ignored by gaussian_x_uniform_y")
    return sorted(seen)
