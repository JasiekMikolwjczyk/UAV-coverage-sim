"""Tests for scan simulator v2 (SPEC section 11)."""
import copy
import csv
import json
import math
import os
import sys
import warnings

import numpy as np
import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "tools"))

import run_scan_simulation as v1  # noqa: E402
import sweep_scan_v2 as sweep  # noqa: E402
from scan_sim_v2 import geometry as geo  # noqa: E402
from scan_sim_v2.analytic import allen_cunneen_wq, analyze, mm1_wq  # noqa: E402
from scan_sim_v2.model import (  # noqa: E402
    MINE_CLASSES, UnusedConfigKeyWarning, bernoulli, build_confirm_sensor, build_screen_sensor,
    combine_probabilities, load_sensor_library, sv,
)
from scan_sim_v2.policies import (  # noqa: E402
    ConfirmerView, PolicyState, QueueItem, VehicleView, corridor_first_policy, nearest_policy,
    priority_policy, resolve_policy,
)
from scan_sim_v2.population import (  # noqa: E402
    generate_population, load_mine_types,
)
from scan_sim_v2.simulate import map_stop_reason_v1, run_mission, simulate  # noqa: E402

LIB = load_sensor_library()
MT = load_mine_types()
V1_CFG = json.load(open(os.path.join(REPO, "configs", "scan_simulation.json")))
V1EQ_CFG = json.load(open(os.path.join(REPO, "configs", "scan_v2_sar_mag_v1equiv.json")))
P1_CFG = json.load(open(os.path.join(REPO, "configs", "scan_v2_paper1.json")))

warnings.filterwarnings("ignore", category=UnusedConfigKeyWarning)


def small_cfg(x_max=40.0, **team):
    cfg = copy.deepcopy(P1_CFG)
    cfg["field"]["x_max"] = x_max
    cfg["mission"]["max_time_s"] = 200000.0
    cfg["team"].update(team)
    return cfg


def sim(cfg, **kw):
    return simulate(cfg, LIB, MT, record_paths=False, **kw)


# --------------------------------------------------------------------------- 1. regression vs v1

@pytest.mark.parametrize("count", [None, 16, 48])
def test_regression_vs_v1(count):
    for seed in range(1, 21):
        a, b = copy.deepcopy(V1_CFG), copy.deepcopy(V1EQ_CFG)
        a["seed"] = b["seed"] = seed
        if count:
            a["mine_distribution"]["count"] = b["mine_distribution"]["count"] = count
        ra, rb = v1.simulate(a), simulate(b)
        ma, mb = ra["metrics"], rb["metrics"]
        for k in ("elapsed_s", "found_mines", "missed_mines", "sar_suspicious", "magnetometer_verifications",
                  "scanned_area_m2"):
            assert ma[k] == mb[k], (seed, count, k)
        assert ma["stop_reason"] == map_stop_reason_v1(mb["stop_reason"])
        assert abs(ma["sar_energy_used_Wh"] - mb["sar_energy_used_Wh"]) <= 1e-9
        assert abs(ma["mag_energy_used_Wh"] - mb["mag_energy_used_Wh"]) <= 1e-9
        assert len(ra["mines"]) == len(rb["mines"])
        for x, y in zip(ra["mines"], rb["mines"]):
            assert x.found == y.found
            if math.isnan(x.final_prob):
                assert math.isnan(y.final_prob)
            else:
                assert abs(x.final_prob - y.final_prob) <= 1e-12


def test_v1_reference_output():
    m = simulate(copy.deepcopy(V1EQ_CFG))["metrics"]
    assert m["elapsed_s"] == 661.0 and m["found_mines"] == 4 and m["missed_mines"] == 0
    assert m["sar_suspicious"] == 4 and m["magnetometer_verifications"] == 4 and m["scanned_area_m2"] == 4000.0
    assert abs(m["sar_energy_used_Wh"] - 6.290555555555558) <= 1e-12
    assert abs(m["mag_energy_used_Wh"] - 14.74671551631015) <= 1e-12
    assert m["stop_reason"] == "completed"


# --------------------------------------------------------------------------- 2. determinism

def test_determinism():
    cfg = small_cfg(n_screeners=4, n_confirmers=6)
    r1, r2 = run_mission(copy.deepcopy(cfg), LIB, MT, record_paths=False), run_mission(copy.deepcopy(cfg), LIB, MT, record_paths=False)
    assert json.dumps(r1["metrics"], sort_keys=True, default=str) == json.dumps(r2["metrics"], sort_keys=True, default=str)
    assert r1["queue_log"] == r2["queue_log"] or json.dumps(r1["queue_log"], default=str) == json.dumps(r2["queue_log"], default=str)


# --------------------------------------------------------------------------- 3. calibration

def _empirical(p_true, n=20000, seed=11):
    u = np.random.default_rng(seed).random(n)
    k = sum(bernoulli(x, p_true) for x in u)
    return k / n, math.sqrt(max(p_true * (1 - p_true), 1e-12) / n)


SCREEN_CASES = [("mag_uav", p) for p in ("optimistic", "nominal", "pessimistic")] + [
    ("emi_uav", "ground_reference"), ("emi_uav", "eth_flight"), ("gpr_uav_survey", "nominal"),
    ("rgb_uav", "nominal"), ("thermal_uav", "nominal"), ("generic_screen", "nominal")]


@pytest.mark.parametrize("name,preset", SCREEN_CASES)
def test_calibration_screening(name, preset):
    s = build_screen_sensor(LIB, name, preset)
    pre = LIB["sensors"][name]["presets"][preset]
    for i, cls in enumerate(MINE_CLASSES):
        if s.model == "logit_depth":
            a = sv(pre["a"][cls])
            if a is None:
                with pytest.raises(ValueError):
                    s.pd_class_depth(cls, 0.05)
                continue
            configured = 1.0 / (1.0 + math.exp(-a))
        elif s.model == "generic":
            configured = sv(pre["pd"][cls])
        else:
            configured = sv(pre["pd_buried"])
        pd = s.pd_class_depth(cls, 0.05, hour=12.0)
        assert abs(pd - configured) < 1e-12
        emp, se = _empirical(pd, seed=100 + i)
        assert abs(emp - configured) <= 3 * se + 1e-12, (name, preset, cls, emp, configured)
    if s.model == "imaging":
        assert abs(s.pd_class_depth("metal", 0.0) - sv(pre["pd_surface"])) < 1e-12


@pytest.mark.parametrize("preset", ["optimistic", "nominal", "pessimistic"])
def test_calibration_confirm(preset):
    c = build_confirm_sensor(LIB, "gpr_uav_confirm", preset)
    pre = LIB["sensors"]["gpr_uav_confirm"]["presets"][preset]["p_confirm"]
    for i, cls in enumerate(list(MINE_CLASSES) + ["metal_clutter"]):
        configured = sv(pre["clutter" if cls == "metal_clutter" else cls])
        p = c.p_class_depth(cls, 0.05)  # spatial factor fixed at 1
        assert abs(p - configured) < 1e-9
        emp, se = _empirical(p, seed=200 + i)
        assert abs(emp - configured) <= 3 * se + 1e-12


def test_sensor_numbers_have_sources():
    bad = []

    def walk(node, path):
        if isinstance(node, dict):
            if "value" in node:
                if "source" not in node or not node["source"]:
                    bad.append(path)
                return
            for k, v in node.items():
                if k.startswith("_") or k in ("sources",):
                    continue
                if path.startswith("platforms"):
                    continue
                walk(v, f"{path}.{k}")
        elif isinstance(node, (int, float)) and not isinstance(node, bool):
            bad.append(path)

    walk({k: v for k, v in LIB.items() if k != "platforms"}, "sensors_v2")
    walk({k: v for k, v in MT.items() if k not in ("types",)}, "mine_types_v2")
    assert "_source" in LIB["platforms"] and "assumption" in LIB["platforms"]["_source"]
    for tname, t in MT["types"].items():
        assert t.get("class_source") and t.get("emplacement_source"), tname
    assert not bad, bad


# --------------------------------------------------------------------------- 4. false-alarm rate

def _clutter_only_cfg(seed, density=0.6, generic=False):
    cfg = small_cfg(x_max=40.0, n_screeners=2, n_confirmers=0, confirming_sensor=None)
    cfg["seed"] = seed
    cfg["population"]["density_per_m2"] = 0.0
    cfg["decision"]["excavation_time_s"] = 0.0
    cfg["injection"]["enabled"] = False
    cfg["clutter"] = {"density_per_m2": density, "classes": {"metal_clutter": 0.8, "nonmetal_clutter": 0.2},
                      "depth_m": {"mean": 0.05, "std": 0.0, "min": 0.05, "max": 0.05}}
    if generic:
        cfg["team"]["screening_sensor"] = "generic_screen"
        cfg["generic_screen"] = {"pd": {"metal": 0.98, "low_metal": 0.5, "non_metal": 0.0},
                                 "false_alarms_per_m2": density, "swath_m": 0.25, "speed_mps": 4.0,
                                 "localization_std_m": 0.15}
    return cfg


def test_false_alarm_rate():
    s = build_screen_sensor(LIB, "mag_uav", "nominal")
    pd_c = 0.8 * s.pd_class_depth("metal_clutter", 0.05) + 0.2 * s.pd_class_depth("nonmetal_clutter", 0.05)
    area = 2 * 40.0 * 4.5
    counts = [sim(_clutter_only_cfg(seed))["metrics"]["n_false_alarms"] for seed in range(1, 201)]
    expected = 0.6 * area * pd_c
    se = np.std(counts, ddof=1) / math.sqrt(len(counts))
    assert abs(np.mean(counts) - expected) <= 3 * se, (np.mean(counts), expected, se)


# --------------------------------------------------------------------------- 5. corridor coverage and frontier speed

@pytest.mark.parametrize("n,swath,turn", [(2, 1.0, 3.0), (1, 1.0, 3.0), (4, 0.25, 3.0), (3, 0.5, 0.0)])
def test_corridor_coverage_and_frontier(n, swath, turn):
    fld = {"x_min": 0.0, "x_max": 60.0, "y_min": 0.0, "y_max": 10.0, "lane_width": 10.0, "grid_res": 0.25,
           "terrain_margin_m": 2.0}
    W, L, v, dt = 4.5, 10.0, 1.0, 1.0
    sep, cover, k = geo.corridor_geometry(n, swath, 0.0, W)
    pl = geo.make_planner("corridor", 5.0, 1, 0.0, 60.0, n, swath, 0.0, v, W, L, turn)
    grid = geo.make_grid(fld)
    gx0, _, gy0, _ = geo.coverage_bounds(fld)
    t = 0.0
    seg_done_t = {}
    while not pl.done:
        res = pl.step(dt)
        t += dt
        for segs in res.scan_segments:
            for p0, p1 in segs:
                geo.mark_capsule(grid, fld, p0, p1, 0.5 * swath)
        if res.frontier_x not in seg_done_t:
            seg_done_t[res.frontier_x] = t
        # every corridor cell fully behind the frontier is covered
        res_g = fld["grid_res"]
        ix1 = int(math.floor((res.frontier_x - gx0) / res_g))
        iy0 = int(math.ceil((5.0 - W / 2 - gy0) / res_g))
        iy1 = int(math.floor((5.0 + W / 2 - gy0) / res_g))
        ix0 = int(math.ceil((0.0 - gx0) / res_g))
        if ix1 > ix0:
            assert grid[iy0:iy1, ix0:ix1].all()
    v_formula, k2 = geo.frontier_speed_formula(n, swath, 0.0, v, W, L, turn)
    assert k2 == k
    if k > 1:
        # time to clear 5 segments (steady state) vs formula, within one dt
        t5 = seg_done_t[50.0] - seg_done_t[0.0 + L]
        assert abs(t5 - 4 * L / v_formula) <= dt + 1e-9
        if k % 2 == 1:
            assert abs(v_formula - L / (k * (L / v + turn))) < 1e-12


# --------------------------------------------------------------------------- 6/7. energy and time decomposition

def test_energy_bookkeeping_and_time_split():
    cfg = small_cfg(n_screeners=3, n_confirmers=4)
    m = sim(cfg)["metrics"]
    assert abs(m["fleet_energy_Wh"] - (sum(m["screener_energy_Wh"]) + sum(m["confirmer_energy_Wh"]) + sum(m["surface_energy_Wh"]))) < 1e-9
    assert abs(m["fleet_energy_screening_Wh"] - 3 * m["screener_energy_Wh"][0]) < 1e-9
    split = sum(m[k] for k in ("tractor_moving_s", "tractor_wait_confirmation_s", "tractor_wait_frontier_s",
                               "tractor_wait_excavation_s", "tractor_done_waiting_s"))
    assert abs(split - m["elapsed_s"]) <= float(cfg["mission"]["dt"])


# --------------------------------------------------------------------------- 8. stream isolation

def test_stream_isolation():
    cfg = small_cfg(x_max=200.0, n_screeners=4, n_confirmers=4)
    cfg["population"]["density_per_m2"] = 0.002
    cfg["population"]["hard_target_selection"] = False
    a, b = copy.deepcopy(cfg), copy.deepcopy(cfg)
    a["clutter"]["density_per_m2"] = 0.0
    b["clutter"]["density_per_m2"] = 0.6
    ra, rb = sim(a), sim(b)
    ma = [o for o in ra["objects"] if o.is_mine]
    mb = [o for o in rb["objects"] if o.is_mine]
    assert len(ma) == len(mb) > 0
    for x, y in zip(ma, mb):
        assert (x.x, x.y, x.cls, x.mine_type, x.depth_m) == (y.x, y.y, y.cls, y.mine_type, y.depth_m)
        assert x.sar_seen == y.sar_seen and x.screen_detected == y.screen_detected
    assert np.array_equal(ra["ctx"].U[: len(ma)], rb["ctx"].U[: len(mb)])


# --------------------------------------------------------------------------- 9. prior

def test_prior_fusion_equals_v1():
    rng = np.random.default_rng(0)
    for ps, pc in rng.random((200, 2)):
        assert combine_probabilities(ps, pc, 0.5) == v1.combine_probabilities(ps, pc)
    assert combine_probabilities(0.7, 0.7, 0.7) == pytest.approx(0.7)


# --------------------------------------------------------------------------- 10. policy interface

@pytest.mark.parametrize("name", ["priority", "nearest", "corridor_first"])
def test_policy_interface(name):
    cfg = small_cfg(n_screeners=2, n_confirmers=3, confirmer_policy=name)
    cfg2 = copy.deepcopy(cfg)
    cfg2["team"]["confirmer_policy"] = f"scan_sim_v2.policies.{name}_policy"
    m1, m2 = sim(cfg)["metrics"], sim(cfg2)["metrics"]
    keys = [k for k in m1 if k not in ("confirmer_policy", "calibration")]
    assert all(m1[k] == m2[k] for k in keys)
    # direct call == call through resolve_policy
    q = [QueueItem(i, float(i), 1.0 * (i % 3), (0, 0), float(10 - i), i % 2 == 0, 0.9, True) for i in range(6)]
    confs = [ConfirmerView(j, 0.0, 0.0, 0.0, 100.0, True) for j in range(3)]
    st = PolicyState(5.0, q, confs, VehicleView(0.0, 0, 5.0, 1, False), 10.0)
    direct = {"priority": priority_policy, "nearest": nearest_policy, "corridor_first": corridor_first_policy}[name]
    assert direct(st) == resolve_policy(name)(st)


# --------------------------------------------------------------------------- 11. analytic M/M/1

def test_analytic_mm1():
    for lam, mu in [(0.1, 1.0), (0.5, 1.0), (0.9, 1.0), (0.02, 0.05)]:
        assert allen_cunneen_wq(lam, 1.0 / mu, 1, cs2=1.0) == pytest.approx(mm1_wq(lam, mu), rel=1e-12)
        assert mm1_wq(lam, mu) == pytest.approx((lam / mu) / (mu - lam))


def test_analytic_runs_on_paper1():
    a = analyze(copy.deepcopy(P1_CFG), LIB, MT)
    assert a["frontier_speed_mps"] > 0 and a["M_star"] >= 1 and 0 <= a["p_detect"] <= 1


# --------------------------------------------------------------------------- 12. unused keys warn once each

def test_unused_keys_warn_once():
    with warnings.catch_warnings(record=True) as rec:
        warnings.simplefilter("always")
        simulate(copy.deepcopy(V1EQ_CFG))
    msgs = [str(w.message) for w in rec if issubclass(w.category, UnusedConfigKeyWarning)]
    for key in ("decision.sar_suspicious_max", "decision.cleared_threshold", "mine_distribution.mean[1]",
                "mine_distribution.std[1]"):
        assert sum(key in m for m in msgs) == 1, (key, msgs)


# --------------------------------------------------------------------------- 13. population generators

def test_doctrinal_generator():
    clearance = build_screen_sensor(LIB, "emi_uav", "ground_reference")
    doc = json.load(open(os.path.join(REPO, "configs", "scan_v2_doctrinal.json")))
    for mult in (1.0, 2.0):
        for seed in range(1, 21):
            cfg = copy.deepcopy(doc)
            cfg["seed"] = seed
            cfg["population"]["density_multiplier"] = mult
            mines, meta, _ = generate_population(cfg, MT, clearance)
            rows = meta["rows"]
            for rid, row in enumerate(rows):
                roles = {MT["types"][m.mine_type]["role"] for m in mines if m.row_id == rid}
                assert roles <= {row["role"]}
            at = [r for r in rows if r["role"] == "AT"]
            ap = [r for r in rows if r["role"] == "AP"]
            assert 3 <= len(at) <= 4
            for r in at:
                assert 4.0 / mult - 1e-9 <= r["in_row"] <= 6.0 / mult + 1e-9
            for r in ap:
                assert abs(r["in_row"] - 1.0 / mult) < 1e-9
            for r0, r1 in zip(at, at[1:]):
                assert 20.0 - 1e-9 <= r1["x"] - r0["x"] <= 40.0 + 1e-9
            seq = at[-1:] + ap
            for r0, r1 in zip(seq, seq[1:]):
                assert 5.0 - 1e-9 <= r1["x"] - r0["x"] <= 10.0 + 1e-9


def test_scatterable_generator():
    clearance = build_screen_sensor(LIB, "emi_uav", "ground_reference")
    sc = json.load(open(os.path.join(REPO, "configs", "scan_v2_scatterable.json")))
    for seed in range(1, 21):
        cfg = copy.deepcopy(sc)
        cfg["seed"] = seed
        mines, meta, _ = generate_population(cfg, MT, clearance)
        for m in mines:
            fp = meta["footprints"][m.row_id]
            dx, dy = m.x - fp["cx"], m.y - fp["cy"]
            u = dx * math.cos(fp["theta"]) + dy * math.sin(fp["theta"])
            v = -dx * math.sin(fp["theta"]) + dy * math.cos(fp["theta"])
            assert (u / fp["a"]) ** 2 + (v / fp["b"]) ** 2 <= 1.0 + 1e-9
            assert m.depth_m == 0.0


def test_residual_density_and_selection():
    from scipy import stats
    clearance = build_screen_sensor(LIB, "emi_uav", "ground_reference")
    cfg = copy.deepcopy(P1_CFG)
    cfg["field"].update({"x_min": 0.0, "x_max": 10000.0, "y_min": 0.0, "y_max": 10000.0})
    rho = 8.5e-7
    area = 1e8
    counts, kept = [], []
    for seed in range(1, 1001):
        cfg["seed"] = seed
        mines, _, sampler = generate_population(cfg, MT, clearance)
        counts.append(len(mines))
        kept.extend((m.mine_type, m.depth_m) for m in mines)
    dens = np.array(counts) / area
    se = dens.std(ddof=1) / math.sqrt(len(dens))
    assert abs(dens.mean() - rho) <= 3 * se
    # chi-square: (type, depth bin) histogram vs candidate distribution reweighted by 1 - Pd_clearance
    nb = 4
    obs, exp = [], []
    weights = sampler.weights
    wsum = sum(weights.values())
    cells = []
    for tname, w in weights.items():
        t = MT["types"][tname]
        dm = MT["depth_models"][t["depth_model"]]
        lo, hi = float(sv(dm["low"])), float(sv(dm["high"]))
        edges = np.linspace(lo, hi, nb + 1)
        for b in range(nb):
            grid = np.linspace(edges[b], edges[b + 1], 202)[1:-1]
            keep = np.mean([sampler.keep_probability(t["class"], d) for d in grid])
            cells.append((tname, edges[b], edges[b + 1], w / wsum * keep / nb))
    total = sum(c[3] for c in cells)
    for tname, e0, e1, mass in cells:
        o = sum(1 for (tn, d) in kept if tn == tname and e0 <= d < e1 + (1e-12 if e1 == cells[-1][2] else 0))
        obs.append(o)
        exp.append(mass / total * len(kept))
    obs, exp = np.array(obs, float), np.array(exp, float)
    exp *= obs.sum() / exp.sum()
    p = stats.chisquare(obs, exp).pvalue
    assert p > 0.01, (p, obs, exp)


# --------------------------------------------------------------------------- 14. injection isolation

def test_injection_isolation():
    cfg = small_cfg(n_screeners=2, n_confirmers=4)
    off = copy.deepcopy(cfg)
    off["injection"]["enabled"] = False
    m_on = run_mission(copy.deepcopy(cfg), LIB, MT, record_paths=False)
    m_off = run_mission(off, LIB, MT, record_paths=False)
    a = {k: v for k, v in m_on["metrics"].items() if not k.startswith("inj_") and k != "residual_risk_per_km"}
    b = {k: v for k, v in m_off["metrics"].items() if not k.startswith("inj_") and k != "residual_risk_per_km"}
    assert json.dumps(a, sort_keys=True, default=str) == json.dumps(b, sort_keys=True, default=str)
    inj = m_on["injection_objects"]
    assert len(inj) == cfg["injection"]["n_inject"]
    lanes = geo.lane_centers(cfg["field"])
    half = 0.5 * cfg["tractor"]["width_m"] + cfg["tractor"]["safety_margin_m"]
    for o in inj:
        assert any(abs(o.y - ly) <= half for ly in lanes)
        assert cfg["field"]["x_min"] <= o.x <= cfg["field"]["x_max"]


# --------------------------------------------------------------------------- 15. generic screener

def test_generic_screener_pd_and_fa():
    pd_cfg = {"metal": 0.9, "low_metal": 0.3, "non_metal": 0.0}
    seen = {c: 0 for c in MINE_CLASSES}
    det = {c: 0 for c in MINE_CLASSES}
    for seed in range(1, 41):
        cfg = _clutter_only_cfg(seed, density=0.0, generic=True)
        cfg["generic_screen"]["pd"] = pd_cfg
        cfg["population"].update({"density_per_m2": 0.05, "hard_target_selection": False})
        for o in sim(cfg)["objects"]:
            if o.is_mine and o.sar_seen:
                seen[o.cls] += 1
                det[o.cls] += o.screen_detected
    for cls in ("metal", "low_metal"):
        n = seen[cls]
        assert n > 200
        se = math.sqrt(pd_cfg[cls] * (1 - pd_cfg[cls]) / n)
        assert abs(det[cls] / n - pd_cfg[cls]) <= 3 * se, (cls, det[cls] / n)
    counts = [sim(_clutter_only_cfg(seed, density=0.3, generic=True))["metrics"]["n_false_alarms"] for seed in range(1, 201)]
    se = np.std(counts, ddof=1) / math.sqrt(len(counts))
    assert abs(np.mean(counts) - 0.3 * 2 * 40.0 * 4.5) <= 3 * se


def test_generic_area_rate_law():
    cfg = _clutter_only_cfg(1, density=0.0, generic=True)
    cfg["field"]["x_max"] = 200.0
    cfg["team"]["n_screeners"] = 18          # C = 18 * 0.25 = 4.5 = W
    cfg["team"]["turn_time_s"] = 0.0
    cfg["generic_screen"]["speed_mps"] = 1.0
    m = sim(cfg)["metrics"]
    W, N, a = 4.5, 18, 0.25 * 1.0
    law = 1.0 * N * a / W
    assert abs(m["frontier_speed_mps"] - law) / law <= 0.05


# --------------------------------------------------------------------------- 16. projection labelling

def test_projection_labelling(tmp_path):
    sweep._init(os.path.join(REPO, "configs", "scan_v2_paper1.json"))
    s = sweep.spec("projection", screener="emi_uav", preset="ground_reference", a=1.0, N=4, M=4, projection=True)
    cfg = sweep.build_cfg(s, sweep._BASE, sweep._LIB)
    cfg["field"]["x_max"] = 40.0
    res = run_mission(cfg, LIB, MT, record_paths=False)
    assert res["metrics"]["projection"] is True
    row, _ = (lambda r: ({**{k: s[k] for k in sweep.SPEC_KEYS}, "seed": 1, **{k: v for k, v in r.items() if not isinstance(v, (dict, list))}}, None))(res["metrics"])
    summ = sweep.summarize([row])
    path = tmp_path / "summary.csv"
    sweep.write_rows(str(path), summ)
    rows = list(csv.DictReader(open(path)))
    assert rows[0]["projection"] == "True"
    # metrics.json written by the runner carries the flag too
    from scan_sim_v2.outputs import write_outputs
    write_outputs(res, str(tmp_path / "run"), plots=False)
    assert json.load(open(tmp_path / "run" / "metrics.json"))["projection"] is True
