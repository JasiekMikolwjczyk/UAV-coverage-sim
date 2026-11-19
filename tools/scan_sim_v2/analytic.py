"""Analytic model (SPEC 9) used to validate the simulator and to build the requirements map.

Approximations (stated in docs/scan_sim_v2.md):
* Frontier speed: exact for the corridor planner leg model (geometry.frontier_speed_formula),
  including the extra transit leg for even k.
* Expectations over depth/class: deterministic numerical integration (depth grids per type, with
  the residual hard-target selection weights; clutter depth by fixed-seed Monte Carlo).
* E[travel]: numerical integration by fixed-seed Monte Carlo (20,000 samples), see
  service_time_stats: busy start (probability Erlang C) from the previous scan end, Gamma(M, v_f/lambda)
  metres along the corridor; idle start from the SPEC 6.2 idle point (vehicle + safe_distance, lateral
  offset of the lowest-index free confirmer), which trails the frontier by
  gap = max(0, v_f R + standoff - safe_distance); fixed-point iteration on E[S] and the gap.
* Waiting time: M/G/M with the Allen-Cunneen approximation
  Wq = C(M, lambda E[S]) / (M/E[S] - lambda) * (ca^2 + cs^2) / 2, ca^2 = 1 (Poisson arrivals).
* Arrivals use the screened width W_s: max(W, C) for a single along-track pass (C = formation cover),
  W for multi-pass corridor screening, W + pi r^2 / sep for cross-track lines (end caps, r = swath/2).
  Suspicions outside the corridor are still queued (SPEC 6.2) and load the confirmers.
* Vehicle speed (per lane of length L, mean over lanes):
    T_lane = max(L / v0, L * n_m * E[S] / M) + max(0, R - B)
    R = max(q_c * r, q_x * (r + T_exc)),  r = Wq + E[S],
    q_c = 1 - exp(-n_c v_f r)            (some suspicion still unresolved in the lag window),
    q_x = 1 - exp(-n_c p_c v_f (r + T_exc)) (some confirmed object still being excavated)
  v0 = min(v_max, v_f); n_m = lambda / v_f suspicions per metre; p_c = probability a suspicion is
  confirmed; n_c = n_m W / W_s in-corridor suspicions per metre; B = (safe_distance - standoff) / v0. The first term is the pace (frontier or confirmer
  throughput when rho >= 1); the second is the resolution lag the vehicle carries behind the frontier,
  paid once per lane (when rho < 1 resolutions run in parallel and do not add per suspicion).
  Wq is omitted from R when rho >= 1 (the throughput term already holds the queue).
"""
import math

import numpy as np

from . import geometry as geo
from .simulate import investigation_time
from .model import (build_confirm_sensor, build_screen_sensor, load_sensor_library, sv)
from .population import TypeDepthSampler, load_mine_types

N_MC = 20000


# --------------------------------------------------------------------------- queueing

def erlang_c(m: int, a: float) -> float:
    """Probability of waiting in M/M/m with offered load a = lambda/mu (requires a < m)."""
    if m <= 0:
        return 1.0
    if a >= m:
        return 1.0
    s = 0.0
    term = 1.0
    for k in range(m):
        if k > 0:
            term *= a / k
        s += term
    term_m = term * a / m if m > 0 else 1.0  # a^m / m!
    top = term_m * m / (m - a)
    return top / (s + top)


def allen_cunneen_wq(lam: float, es: float, m: int, cs2: float, ca2: float = 1.0) -> float:
    if m <= 0 or lam <= 0:
        return 0.0
    mu = 1.0 / es
    if lam >= m * mu:
        return math.inf
    pw = erlang_c(m, lam / mu)
    return pw / (m * mu - lam) * (ca2 + cs2) / 2.0


def mm1_wq(lam: float, mu: float) -> float:
    """Textbook M/M/1 mean waiting time in queue."""
    rho = lam / mu
    return rho / (mu - lam)


# --------------------------------------------------------------------------- population expectations

def _depth_grid(sampler, tname, n=101):
    t = sampler.mt["types"][tname]
    model_name = "surface" if sampler.emplacement(tname) == "surface" else t["depth_model"]
    dm = sampler.mt["depth_models"][model_name]
    if dm["kind"] == "fixed":
        depths = np.array([float(sv(dm["depth"]))])
    else:
        lo, hi = float(sv(dm["low"])), float(sv(dm["high"]))
        depths = lo + (np.arange(n) + 0.5) / n * (hi - lo)
    if sampler.aged and model_name != "surface":
        depths = (depths[:, None] + (np.arange(11) + 0.5)[None, :] / 11 * 0.05).ravel()
    return depths


def mine_expectations(sampler, fn, non_metal_fraction=0.0):
    """Class shares and E[fn(cls, depth)] per class over the kept-mine distribution."""
    acc = {}
    for tname, w in sampler.weights.items():
        cls = sampler.mt["types"][tname]["class"]
        depths = _depth_grid(sampler, tname)
        keep = np.array([sampler.keep_probability(cls, d) for d in depths])
        if keep.sum() <= 0:
            continue
        mass = w * keep.mean()
        targets = [(cls, 1.0 - non_metal_fraction), ("non_metal", non_metal_fraction)] if non_metal_fraction > 0 else [(cls, 1.0)]
        for tcls, f in targets:
            if f <= 0:
                continue
            vals = np.array([fn(tcls, d) for d in depths])
            a = acc.setdefault(tcls, [0.0, 0.0])
            a[0] += mass * f
            a[1] += mass * f * float((keep * vals).sum() / keep.sum())
    total = sum(v[0] for v in acc.values())
    return {cls: {"share": v[0] / total, "mean": v[1] / v[0]} for cls, v in acc.items() if v[0] > 0}


def clutter_expectation(cfg, fn):
    cl = cfg.get("clutter", {})
    dens = float(cl.get("density_per_m2", 0.0))
    classes = cl.get("classes", {"metal_clutter": 0.8, "nonmetal_clutter": 0.2})
    tot = sum(float(v) for v in classes.values()) or 1.0
    ds = cl.get("depth_m", {"mean": 0.05, "std": 0.05, "min": 0.0, "max": 0.3})
    rng = np.random.default_rng(12345)
    depths = np.clip(rng.normal(ds["mean"], ds["std"], size=2000), ds["min"], ds["max"])
    out = {}
    for k, w in classes.items():
        out[k] = {"density": dens * float(w) / tot, "mean": float(np.mean([fn(k, d) for d in depths]))}
    return out


# --------------------------------------------------------------------------- travel / service

def _scan_template(geom, sigma, size_factor=1.0):
    segs = geo.local_square_segments(0.0, 0.0, geom, geo.local_scan_side(geom, sigma, size_factor))
    start = segs[0][0]
    end = segs[-1][1]
    length = geo.segment_path_length(segs, start)
    return segs, start, end, length


_MC_CACHE = {}


def _mc_draws(W, M):
    """Fixed-seed draws for service_time_stats, cached (same numbers for every call with this W, M)."""
    key = (W, max(M, 1))
    if key not in _MC_CACHE:
        rng = np.random.default_rng(2024)
        y1 = rng.uniform(-W / 2, W / 2, N_MC)
        y2 = rng.uniform(-W / 2, W / 2, N_MC)
        g = rng.gamma(max(M, 1), 1.0, N_MC)
        u = rng.uniform(0.0, 1.0, N_MC)
        uj = rng.uniform(0.0, 1.0, N_MC)
        ub = rng.uniform(0.0, 1.0, N_MC)
        if len(_MC_CACHE) > 256:
            _MC_CACHE.clear()
        _MC_CACHE[key] = (y1, y2, u, uj, ub, g)
    return _MC_CACHE[key]


def service_time_stats(geom, lam, v_f, W, M, seg_len, sigma, gap_m=0.0, ahead=True, idle_spacing=3.0, iters=12):
    """E[S], E[travel], SCV of service time (fixed-seed Monte Carlo integration).

    A task starts either right after the confirmer's previous task (probability = Erlang C, i.e.
    the queue is non-empty when a confirmer frees up; 1 when saturated) or from the confirmer's
    idle point. Busy start: the next suspicion in FIFO order for this confirmer is Gamma(M, v_f/lambda)
    metres further along the corridor, lateral positions uniform in W. Idle start: the idle point
    is (vehicle + safe_distance, lane_y + (j - (M-1)/2) * idle_spacing) with j the lowest-index
    free confirmer (P(j) ~ rho^j); it lies gap_m behind the frontier; the suspicion lies within one
    block/segment of the frontier (ahead for corridor segments, behind for cross-track blocks).
    """
    segs, start, end, scan_len = _scan_template(geom, sigma)
    vt = float(geom["transit_speed_mps"])
    scan_s = scan_len / float(geom["scan_speed_mps"])
    y1, y2, u, uj, ub, g = _mc_draws(W, M)
    sign = 1.0 if ahead else -1.0
    es = scan_s
    for _ in range(iters):
        a = lam * es
        rho1 = min(a / max(M, 1), 0.999)
        p_busy = 1.0 if a >= M else erlang_c(M, a)
        if M > 1:
            w = rho1 ** np.arange(M)
            cdf = np.cumsum(w / w.sum())
            j = np.minimum(np.searchsorted(cdf, uj), M - 1)
            idle_y = (j - (M - 1) / 2.0) * idle_spacing
        else:
            idle_y = 0.0
        idle_dx = gap_m + sign * u * seg_len + start[0]
        idle_dy = y2 - idle_y + start[1]
        t_idle = np.hypot(idle_dx, idle_dy) / vt
        spacing = v_f / lam if lam > 0 else seg_len
        dx = g * spacing + start[0] - end[0]
        dy = (y2 + start[1]) - (y1 + end[1])
        t_busy = np.hypot(dx, dy) / vt
        travel = np.where(ub < p_busy, t_busy, t_idle)
        S = travel + scan_s
        es_new = float(S.mean())
        if abs(es_new - es) < 1e-6 * max(es, 1.0):
            es = es_new
            break
        es = es_new
    cs2 = float(S.var() / es ** 2) if es > 0 else 0.0
    return {"E_travel_s": float(travel.mean()), "scan_s": scan_s, "E_S": es, "cs2": cs2, "p_busy_start": p_busy}


def mean_spatial_factor(geom, sigma):
    segs, _, _, _ = _scan_template(geom, sigma)
    rng = np.random.default_rng(7)
    px = rng.normal(0.0, sigma, 4000)
    py = rng.normal(0.0, sigma, 4000)
    d = np.min(np.stack([geo.seg_dist_array(px, py, a, b) for a, b in segs]), axis=0)
    decay = float(geom["spatial_decay_m"])
    return float(np.mean(np.exp(-0.5 * (d / decay) ** 2)))


# --------------------------------------------------------------------------- main entry

def analyze(cfg, sensor_lib=None, mine_types=None, screen_overrides=None, n_screeners=None, n_confirmers=None):
    lib = sensor_lib or load_sensor_library(cfg.get("sensors_path"))
    mt = mine_types or load_mine_types(cfg.get("mine_types_path"))
    team = dict(cfg.get("team", {}))
    N = int(n_screeners if n_screeners is not None else team.get("n_screeners", 2))
    M = int(n_confirmers if n_confirmers is not None else team.get("n_confirmers", 1))
    if team.get("confirming_sensor") in (None, "none"):
        M = 0
    preset = team.get("sensor_preset", "nominal")
    ov = dict(team.get("screening_overrides", {}))
    if team.get("screening_sensor") == "generic_screen" and "generic_screen" in cfg:
        ov.update(cfg["generic_screen"])
    if screen_overrides:
        ov.update(screen_overrides)
    ss = build_screen_sensor(lib, team.get("screening_sensor", "mag_uav"), preset, ov)
    cs = None
    if M > 0:
        cname = team.get("confirming_sensor", "gpr_uav_confirm")
        cpre = team.get("confirmer_preset", preset)
        if cpre not in lib["sensors"][cname]["presets"]:
            cpre = "nominal"
        cs = build_confirm_sensor(lib, cname, cpre, team.get("confirming_overrides"))
    on_reject = cfg.get("decision", {}).get("on_reject", "release")
    tr = cfg["tractor"]
    W = float(tr["width_m"]) + 2.0 * float(tr["safety_margin_m"])
    seg_len = float(team.get("segment_length_m", 10.0))
    turn = float(team.get("turn_time_s", 3.0))
    mode = team.get("screening_mode", "corridor")
    _, cover, _ = geo.corridor_geometry(N, ss.swath_m, ss.overlap, W)
    if mode in ("corridor", "cross_track"):
        v_f, k = geo.frontier_speed_formula(N, ss.swath_m, ss.overlap, ss.speed_mps, W, seg_len, turn, mode)
    else:
        v_f, k = ss.speed_mps, 1
    # Width actually screened: a single along-track pass wider than the corridor also screens (and
    # queues) outside it. Cross-track lines span the corridor and their half-disc end caps add
    # pi*r^2 per line (r = swath/2) beyond it: W_s = W + pi*r^2/sep.
    if mode == "cross_track":
        W_s = W + math.pi * (0.5 * ss.swath_m) ** 2 / (ss.swath_m * (1.0 - ss.overlap))
    else:
        W_s = max(W, cover) if k == 1 else W
    if mode == "cross_track":
        # Suspicions appear within the block just screened (idle-start travel model).
        seg_len = N * ss.swath_m * (1.0 - ss.overlap)
    a = ss.swath_m * ss.speed_mps
    eta = v_f * W / (N * a) if a > 0 else 0.0
    v_max = float(tr["max_speed_mps"])

    pop = cfg["population"]
    clearance = build_screen_sensor(lib, "emi_uav", "ground_reference")
    sampler = TypeDepthSampler(pop, mt, clearance)
    nmf = float(pop.get("non_metal_fraction", 0.0))
    rho_mine = float(pop.get("density_per_m2", 8.5e-7)) if sampler.regime == "residual" else float(pop.get("density_per_m2", 0.0))

    pd_m = mine_expectations(sampler, lambda c_, d: ss.pd_class_depth(c_, d), nmf)
    pd_mine = sum(v["share"] * v["mean"] for v in pd_m.values())
    if ss.model == "generic":
        clutter_rate = ss.false_alarms_per_m2
        clut = {"false_alarm": {"density": clutter_rate, "mean": 1.0}}
    else:
        clut = clutter_expectation(cfg, lambda c_, d: ss.pd_class_depth(c_, d))
        clutter_rate = sum(v["density"] * v["mean"] for v in clut.values())
    lam = v_f * W_s * (rho_mine * pd_mine + clutter_rate)

    out = {
        "n_screeners": N, "n_confirmers": M, "area_rate_m2ps": a, "corridor_width_m": W, "screened_width_m": W_s, "k_passes": k,
        "frontier_speed_mps": v_f, "coverage_efficiency": eta, "lambda_per_s": lam,
        "pd_mine_mean": pd_mine, "pd_mine_by_class": {c_: v["mean"] for c_, v in pd_m.items()},
        "class_share": {c_: v["share"] for c_, v in pd_m.items()},
        "clutter_detections_per_m2": clutter_rate,
        "N_req_v2": math.ceil(2.0 * W / (eta * a)) if eta * a > 0 else None,
    }
    sigma = ss.localization_std_m
    v0 = min(v_max, v_f)
    n_m = lam / v_f if v_f > 0 else math.inf
    n_c = n_m * W / W_s  # suspicions per metre inside the corridor (only these block the vehicle)
    t_exc = investigation_time(cfg)
    standoff = float(tr.get("standoff_m", 5.0))
    safe = float(tr["safe_distance_m"])

    def lag(es_, wq_, rho_, p_c_):
        """Resolution lag R (see module docstring)."""
        # In-corridor suspicions that end up blocking for t_exc: confirmed, plus rejected under investigate.
        p_block = p_c_ + (1.0 - p_c_) if on_reject == "investigate" else p_c_
        r = (wq_ if (M > 0 and rho_ < 1.0 and math.isfinite(wq_)) else 0.0) + es_
        q_c = 1.0 - math.exp(-n_c * v_f * r) if math.isfinite(n_c) else 1.0
        q_x = 1.0 - math.exp(-n_c * p_block * v_f * (r + t_exc)) if (t_exc > 0 and math.isfinite(n_c)) else 0.0
        return max(q_c * r, q_x * (r + t_exc))

    if M > 0:
        geom = cs.geom
        sf = mean_spatial_factor(geom, sigma)
        pc_m = mine_expectations(sampler, lambda c_, d: ss.pd_class_depth(c_, d) * cs.p_class_depth(c_, d), nmf)
        p_detect = sum(v["share"] * v["mean"] for v in pc_m.values()) * sf
        p_detect_cls = {c_: v["mean"] * sf for c_, v in pc_m.items()}
        if ss.model == "generic":
            p_conf_clutter = cs.p_class_depth("false_alarm", 0.0) * sf
        else:
            p_conf_clutter = cs.p_class_depth("metal_clutter", 0.05) * sf
        p_c = (rho_mine * p_detect + clutter_rate * p_conf_clutter) / (rho_mine * pd_mine + clutter_rate) if lam > 0 else 0.0
        # Idle confirmers wait at vehicle + safe_distance; the vehicle trails the frontier by
        # max(safe_distance, v_f * R + standoff). Fixed-point iteration on the gap.
        gap = 0.0
        idle_spacing = float(team.get("confirmer_idle_spacing_m", 3.0))
        for _ in range(6):
            st = service_time_stats(geom, lam, v_f, W, M, seg_len, sigma, gap, mode != "cross_track", idle_spacing)
            es = st["E_S"]
            rho = lam * es / M
            wq = allen_cunneen_wq(lam, es, M, st["cs2"])
            new_gap = max(0.0, v_f * lag(es, wq, rho, p_c) + standoff - safe)
            if rho >= 1.0 or abs(new_gap - gap) < 0.05:
                gap = new_gap
                break
            gap = new_gap
        if on_reject == "investigate":
            # SPEC 4.4: rejected in-corridor suspicions are investigated, so no confirmer loss.
            p_detect = pd_mine
            p_detect_cls = {c_: v["mean"] for c_, v in pd_m.items()}
        m_star = max(1, math.ceil(lam * es / 0.8))
        out.update({"E_travel_s": st["E_travel_s"], "scan_s": st["scan_s"], "E_S": es, "cs2": st["cs2"],
                    "p_busy_start": st["p_busy_start"], "idle_gap_m": gap,
                    "utilization": rho, "Wq_s": wq, "M_star": m_star, "mean_spatial_factor": sf,
                    "p_confirm_given_suspicion": p_c})
    else:
        p_detect = pd_mine
        p_detect_cls = {c_: v["mean"] for c_, v in pd_m.items()}
        es, rho, wq, p_c = 0.0, 0.0, 0.0, 1.0
        out.update({"E_travel_s": 0.0, "scan_s": 0.0, "E_S": 0.0, "cs2": 0.0, "utilization": 0.0,
                    "Wq_s": 0.0, "M_star": 0, "p_confirm_given_suspicion": 1.0})

    B = max(0.0, safe - standoff) / v0 if v0 > 0 else 0.0
    fld = cfg["field"]
    lane_len = float(fld["x_max"]) - float(fld["x_min"])
    pace = 1.0 / v0 if v0 > 0 else math.inf
    if M > 0:
        pace = max(pace, n_m * es / M)
    R = lag(es, wq, rho, p_c)
    t_lane = lane_len * pace + max(0.0, R - B)
    v_pred = lane_len / t_lane if t_lane > 0 else 0.0
    rho_reg = rho_mine
    out.update({
        "vehicle_speed_free_mps": v0, "suspicions_per_m": n_m, "buffer_s": B, "resolution_lag_s": R,
        "predicted_vehicle_speed_mps": v_pred,
        "p_detect": p_detect, "p_detect_by_class": p_detect_cls, "on_reject": on_reject,
        "investigation_time_s": t_exc,
        "residual_risk_per_km": rho_reg * W * 1000.0 * (1.0 - p_detect),
        "approximation": "Allen-Cunneen M/G/M; E[travel] by fixed-seed MC integration (see module docstring)",
    })
    return out


def requirements_map(cfg, sensor_lib=None, mine_types=None, grid=None, on_reject=("release", "investigate")):
    """Dense analytic requirements map over the SPEC 4.6 grid (generic screener), per on_reject policy."""
    grid = grid or {
        "pd_low_metal": [0.1, 0.3, 0.5, 0.7, 0.9, 0.96],
        "pd_metal": [0.9, 0.98],
        "false_alarms_per_m2": [0.1, 0.3, 0.6, 1.0],
        "area_rate": [0.068, 0.15, 0.25, 0.5, 1.0],
        "N": [1, 2, 4, 8, 16],
    }
    lib = sensor_lib or load_sensor_library()
    mt = mine_types or load_mine_types()
    import copy
    base = copy.deepcopy(cfg)
    base["team"]["screening_sensor"] = "generic_screen"
    swath = float(base.get("generic_screen", {}).get("swath_m", 0.25))
    rows = []
    for policy in on_reject:
        base.setdefault("decision", {})["on_reject"] = policy
        rows += _map_rows(base, lib, mt, grid, swath, policy)
    return rows


def _map_rows(base, lib, mt, grid, swath, policy):
    rows = []
    for pdl in grid["pd_low_metal"]:
        for pdm in grid["pd_metal"]:
            for fa in grid["false_alarms_per_m2"]:
                for a in grid["area_rate"]:
                    for N in grid["N"]:
                        ov = {"pd": {"metal": pdm, "low_metal": pdl, "non_metal": 0.0},
                              "false_alarms_per_m2": fa, "swath_m": swath, "speed_mps": a / swath}
                        first = analyze(base, lib, mt, screen_overrides=ov, n_screeners=N, n_confirmers=1)
                        mstar = first["M_star"]
                        for M in sorted({max(1, mstar - 1), mstar, mstar + 1}):
                            r = analyze(base, lib, mt, screen_overrides=ov, n_screeners=N, n_confirmers=M)
                            rows.append({"on_reject": policy, "pd_low_metal": pdl, "pd_metal": pdm, "false_alarms_per_m2": fa,
                                         "area_rate": a, "N": N, "M": M, "M_star": mstar,
                                         "M_rel": M - mstar,
                                         "frontier_speed_mps": r["frontier_speed_mps"],
                                         "predicted_vehicle_speed_mps": r["predicted_vehicle_speed_mps"],
                                         "utilization": r["utilization"], "lambda_per_s": r["lambda_per_s"],
                                         "E_S": r["E_S"], "p_detect": r["p_detect"],
                                         "residual_risk_per_km": r["residual_risk_per_km"]})
    return rows


def measured_points(cfg, sensor_lib=None, mine_types=None):
    """Measured sensors placed on the map axes (Pd per class over the population, detections/m2, a)."""
    lib = sensor_lib or load_sensor_library()
    mt = mine_types or load_mine_types()
    import copy
    pts = []
    combos = [("mag_uav", p, False) for p in ("pessimistic", "nominal", "optimistic")]
    combos += [("emi_uav", "ground_reference", True), ("gpr_uav_survey", "nominal", False)]
    for name, preset, proj in combos:
        c2 = copy.deepcopy(cfg)
        c2["team"]["screening_sensor"] = name
        c2["team"]["sensor_preset"] = preset
        r = analyze(c2, lib, mt)
        pd = r["pd_mine_by_class"]
        pts.append({"sensor": name, "preset": preset, "projection": proj,
                    "pd_metal": pd.get("metal"), "pd_low_metal": pd.get("low_metal"),
                    "false_alarms_per_m2": r["clutter_detections_per_m2"], "area_rate": r["area_rate_m2ps"]})
    # eth_flight: metal only (low-metal Pd unknown -> not simulated)
    ss = build_screen_sensor(lib, "emi_uav", "eth_flight")
    clut = clutter_expectation(cfg, lambda c_, d: ss.pd_class_depth(c_, d))
    pts.append({"sensor": "emi_uav", "preset": "eth_flight", "projection": True,
                "pd_metal": ss.pd_class_depth("metal", 0.05), "pd_low_metal": None,
                "false_alarms_per_m2": sum(v["density"] * v["mean"] for v in clut.values()),
                "area_rate": ss.area_rate})
    return pts
