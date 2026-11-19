"""Main loop, metrics and safety injection for scan simulator v2.

Step order per dt (SPEC 1): time limit -> batteries (screeners, then confirmers) ->
completion events -> confirmer assignment -> screening -> vehicle -> idle moves -> t += dt.

RNG streams (np.random.default_rng):
  seed          natural mines                seed + 2000  clutter
  seed + 1000   mine localisation noise      seed + 2001  clutter localisation noise
  seed + 3000   mine detection/confirm draws seed + 3001  clutter detection/confirm draws
  seed + 4000   injected mines               seed + 4001  injected draws, + 4002 injected noise
Detection and confirmation draws are pre-drawn per object (columns screen/surface/confirm),
so they do not depend on how many other objects exist (stream isolation, SPEC 11.8).
"""
import copy
import heapq
import math

import numpy as np

from . import geometry as geo
from .model import (
    N_U_COLUMNS, U_CONFIRM, U_SCREEN, U_SURFACE, MINE_CLASSES,
    battery_remaining_percent, bernoulli, build_confirm_sensor, build_screen_sensor,
    combine_probabilities, consume_energy, detection_probability, generate_mines_legacy,
    load_sensor_library, platform_block, power_for_speed, warn_unused_keys,
)
from .policies import (
    SCAN_SIZES, ConfirmerView, PolicyState, QueueItem, VehicleView, resolve_policy,
)
from .population import (
    generate_clutter, generate_injected, generate_population, load_mine_types,
    selection_class_depth_distribution, field_area,
)

LEGACY_STOP_MAP = {
    "screener_0_battery_reserve_reached": "sar_battery_reserve_reached",
    "confirmer_0_battery_reserve_reached": "magnetometer_battery_reserve_reached",
}


def map_stop_reason_v1(reason):
    return LEGACY_STOP_MAP.get(reason, reason)


def wilson(k, n, z=1.959963984540054):
    if n <= 0:
        return (math.nan, math.nan, math.nan)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (p, max(0.0, centre - half), min(1.0, centre + half))


# --------------------------------------------------------------------------- confirmers

class Confirmer:
    def __init__(self, cid, pos, battery):
        self.cid = cid
        self.pos = pos
        self.available_t = 0.0
        self.energy_Wh = 0.0
        self.busy_s = 0.0
        self.battery = battery
        self.pending = 0          # own completion events not yet popped
        self.idle_target = None   # from a custom policy


class Layer:
    """One screening team (primary, or the imaging surface layer)."""

    def __init__(self, name, sensor, n, battery, u_col, legacy_cfg=None):
        self.name = name
        self.sensor = sensor
        self.n = n
        self.battery = battery
        self.u_col = u_col
        self.legacy_cfg = legacy_cfg
        self.energy_Wh = [0.0] * n
        self.paths = [[] for _ in range(n)]
        self.planner = None
        self.active_s = 0.0
        self.screened_len = 0.0


# --------------------------------------------------------------------------- setup

def _team(cfg):
    team = dict(cfg.get("team", {}))
    legacy = cfg.get("detection_mode", "legacy_logistic") == "legacy_logistic"
    team.setdefault("screening_sensor", "sar" if legacy else "mag_uav")
    team.setdefault("confirming_sensor", "magnetometer" if legacy else "gpr_uav_confirm")
    team.setdefault("sensor_preset", "nominal")
    team.setdefault("n_screeners", 2)
    team.setdefault("n_confirmers", 1)
    team.setdefault("screening_mode", "lane_abreast" if legacy else "corridor")
    team.setdefault("confirmer_policy", "priority")
    team.setdefault("confirmer_idle_spacing_m", 3.0)
    team.setdefault("scan_size", "full")
    if team.get("confirming_sensor") in (None, "none") or int(team["n_confirmers"]) == 0:
        team["n_confirmers"] = 0
        team["confirming_sensor"] = None
    return team


class Ctx:
    pass


def build_context(cfg, sensor_lib=None, mine_types=None, inject=0):
    c = Ctx()
    c.cfg = cfg
    c.mode = cfg.get("detection_mode", "legacy_logistic")
    c.legacy = c.mode == "legacy_logistic"
    c.team = _team(cfg)
    c.fld = cfg["field"]
    c.seed = int(cfg.get("seed", 1))
    c.N = int(c.team["n_screeners"])
    c.M = int(c.team["n_confirmers"])
    tr = cfg["tractor"]
    c.corridor_half = 0.5 * float(tr["width_m"]) + float(tr["safety_margin_m"])
    c.W = 2.0 * c.corridor_half
    c.safe_distance = float(tr["safe_distance_m"])
    c.max_speed = float(tr["max_speed_mps"])
    c.block_mode = tr.get("block_mode", "corridor_ahead" if c.legacy else "standoff")
    c.standoff = float(tr.get("standoff_m", 5.0))
    c.investigation_time = investigation_time(cfg, c.legacy)
    c.on_reject = cfg.get("decision", {}).get("on_reject", "release")
    if c.on_reject not in ("release", "investigate"):
        raise ValueError(f"decision.on_reject must be 'release' or 'investigate', not {c.on_reject!r}")
    c.prior = float(cfg.get("decision", {}).get("prior_mine_probability", 0.5))
    c.reserve = float(cfg.get("battery", {}).get("reserve_percent", 0.0))
    c.battery_enforce = bool(cfg.get("battery", {}).get("enforce", True))
    c.start_hour = float(cfg.get("mission", {}).get("start_hour", 12.0))
    c.policy = resolve_policy(c.team["confirmer_policy"])
    c.scan_size = c.team.get("scan_size", "full")
    c.loc_model = cfg.get("localization_model", {"kind": "static"})
    c.lanes = geo.lane_centers(c.fld)
    c.meta = {}
    c.calibration = {}
    c.projection = False

    if c.legacy:
        sname, cname = c.team["screening_sensor"], c.team["confirming_sensor"]
        scfg = cfg[sname]
        c.screen_cfg = scfg
        c.confirm_cfg = cfg[cname] if cname else None
        layer = Layer("primary", None, c.N, cfg["battery"][sname], U_SCREEN, legacy_cfg=scfg)
        layer.radius = 0.5 * float(scfg["swath_m"])
        layer.sep = float(scfg["swath_m"]) * (1.0 - float(scfg["transverse_overlap"]))
        layer.speed = float(scfg["speed_mps"])
        layer.swath = float(scfg["swath_m"])
        layer.overlap = float(scfg["transverse_overlap"])
        layer.loc_std = float(scfg.get("localization_std_m", 0.0))
        c.layers = [layer]
        c.confirm_battery = cfg["battery"][cname] if cname else None
        c.scan_geom = c.confirm_cfg
        c.transit_speed = float(c.confirm_cfg["transit_speed_mps"]) if cname else 4.0
        c.objects = generate_mines_legacy(cfg)
        c.n_natural_mines = len(c.objects)
        c.mine_types_used = None
        c.screen_sensor = None
        c.confirm_sensor = None
    else:
        lib = sensor_lib or load_sensor_library(cfg.get("sensors_path"))
        mt = mine_types or load_mine_types(cfg.get("mine_types_path"))
        preset = c.team["sensor_preset"]
        overrides = dict(c.team.get("screening_overrides", {}))
        if c.team["screening_sensor"] == "generic_screen" and "generic_screen" in cfg:
            overrides.update(cfg["generic_screen"])
        ss = build_screen_sensor(lib, c.team["screening_sensor"], preset, overrides)
        c.screen_sensor = ss
        c.projection = ss.projection
        c.calibration["screening"] = {"sensor": ss.name, "preset": ss.preset, "params": ss.calibration}
        layer = Layer("primary", ss, c.N, platform_block(lib, cfg, ss.name), U_SCREEN)
        layer.radius = 0.5 * ss.swath_m
        layer.speed = ss.speed_mps
        layer.swath = ss.swath_m
        layer.overlap = ss.overlap
        layer.sep = ss.swath_m * (1.0 - ss.overlap)
        layer.loc_std = ss.localization_std_m
        c.layers = [layer]
        surf = c.team.get("surface_layer")
        if surf:
            sl = build_screen_sensor(lib, surf["sensor"], surf.get("preset", "nominal"))
            c.calibration["surface"] = {"sensor": sl.name, "preset": sl.preset, "params": sl.calibration}
            lay = Layer("surface", sl, int(surf.get("n", 1)), platform_block(lib, cfg, sl.name), U_SURFACE)
            lay.radius = 0.5 * sl.swath_m
            lay.speed = sl.speed_mps
            lay.swath = sl.swath_m
            lay.overlap = sl.overlap
            lay.sep = sl.swath_m * (1.0 - sl.overlap)
            lay.loc_std = sl.localization_std_m
            c.layers.append(lay)
        if c.M > 0:
            cname = c.team["confirming_sensor"]
            cpreset = c.team.get("confirmer_preset", preset)
            if cpreset not in lib["sensors"][cname]["presets"]:
                cpreset = "nominal"
            cs = build_confirm_sensor(lib, cname, cpreset, c.team.get("confirming_overrides"))
            c.confirm_sensor = cs
            c.calibration["confirming"] = {"sensor": cs.name, "preset": cs.preset, "params": cs.calibration}
            c.confirm_battery = platform_block(lib, cfg, cs.name)
            c.scan_geom = cs.geom
            c.transit_speed = cs.geom["transit_speed_mps"]
        else:
            c.confirm_sensor = None
            c.confirm_battery = None
            c.scan_geom = None
            c.transit_speed = 4.0
        clearance = build_screen_sensor(lib, "emi_uav", "ground_reference")
        mines, meta, sampler = generate_population(cfg, mt, clearance)
        c.meta = meta
        c.sampler = sampler
        c.mine_types_used = mt
        c.n_natural_mines = len(mines)
        fa = ss.false_alarms_per_m2 if ss.model == "generic" else None
        clutter = generate_clutter(cfg, len(mines), generic_fa_density=fa)
        objs = mines + clutter
        if inject > 0:
            corridors = [(float(c.fld["x_min"]), float(c.fld["x_max"]), ly - c.corridor_half, ly + c.corridor_half)
                         for ly in c.lanes]
            objs += generate_injected(cfg, sampler, corridors, len(objs), inject)
        c.objects = objs
        c.calibration["population"] = {
            "regime": meta["regime"],
            "mine_types_source": "configs/mine_types_v2.json",
            "clearance_sensor": "emi_uav/ground_reference (GICHD05, ITEP04)",
            "clutter_density_source": "ITEP04 indigenous clutter FAR (class split assumption)",
        }

    # pre-drawn uniforms and localisation streams
    n_obj = len(c.objects)
    c.U = np.zeros((n_obj, N_U_COLUMNS))
    c.loc_rng = {}
    c.stream_of = np.zeros(n_obj, dtype=np.int8)  # 0 mine, 1 clutter, 2 injected
    if not c.legacy:
        mine_idx = [o.idx for o in c.objects if o.is_mine and not o.injected]
        clut_idx = [o.idx for o in c.objects if not o.is_mine]
        inj_idx = [o.idx for o in c.objects if o.injected]
        for s, ids, off in ((0, mine_idx, 3000), (1, clut_idx, 3001), (2, inj_idx, 4001)):
            if ids:
                c.U[ids] = np.random.default_rng(c.seed + off).random((len(ids), N_U_COLUMNS))
                c.stream_of[ids] = s
    c.loc_rng[0] = np.random.default_rng(c.seed + 1000)
    c.loc_rng[1] = np.random.default_rng(c.seed + 2001)
    c.loc_rng[2] = np.random.default_rng(c.seed + 4002)
    c.ox = np.array([o.x for o in c.objects], dtype=np.float64)
    c.oy = np.array([o.y for o in c.objects], dtype=np.float64)
    c.order = np.argsort(c.ox, kind="stable")
    c.sorted_x = c.ox[c.order]
    for layer in c.layers:
        layer.seen = np.zeros(n_obj, dtype=bool)
    return c


def investigation_time(cfg, legacy=False):
    """decision.investigation_time_s (SPEC 4.4); the deprecated excavation_time_s is mapped."""
    dec = cfg.get("decision", {})
    if "investigation_time_s" in dec:
        return float(dec["investigation_time_s"])
    if "excavation_time_s" in dec:
        return float(dec["excavation_time_s"])
    return 0.0 if legacy else 300.0


def localization_std(c, layer, t, x, y):
    lm = c.loc_model
    if lm.get("kind", "static") == "zones":
        for z in lm.get("zones", []):
            if z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"] and z.get("t0", -math.inf) <= t <= z.get("t1", math.inf):
                return float(z["std_m"])
    return layer.loc_std


# --------------------------------------------------------------------------- simulation

def simulate(cfg, sensor_lib=None, mine_types=None, inject=0, record_paths=True):
    warn_unused_keys(cfg)
    c = build_context(cfg, sensor_lib, mine_types, inject)
    return run(c, record_paths)


def run(c, record_paths=True):
    cfg, fld = c.cfg, c.fld
    objs = c.objects
    mission = cfg["mission"]
    dt = float(mission["dt"])
    max_time = float(mission["max_time_s"])
    x_min_f, x_max_f = float(fld["x_min"]), float(fld["x_max"])
    y_min_f, y_max_f = float(fld["y_min"]), float(fld["y_max"])
    team = c.team

    coverage = geo.make_grid(fld)
    tractor_path = []
    confirmer_paths = []
    queue = {}             # sid -> QueueItem (unassigned)
    events = []            # (done, idx, cid)
    excavations = []       # (t_end, idx)
    blocked = set()
    confirmers = [Confirmer(j, (x_min_f, y_min_f), c.confirm_battery) for j in range(c.M)]
    max_queue = 0
    n_queued = 0

    t = 0.0
    mission_complete = True
    stop_reason = "completed"
    tractor_integral = 0.0
    tractor_prev_error = 0.0
    tractor_dist = 0.0
    time_split = {"moving": 0.0, "wait_confirmation": 0.0, "wait_frontier": 0.0,
                  "wait_investigation_confirmed": 0.0, "wait_investigation": 0.0, "done_waiting": 0.0}
    thr = float(cfg.get("decision", {}).get("confirmed_mine_threshold", 0.5))

    prim = c.layers[0]
    stopped = False

    def hour(tt):
        return c.start_hour + tt / 3600.0

    # ---- posterior inputs (rate based, reporting only)
    post = _posterior_inputs(c) if not c.legacy else None

    def complete(idx, t_now):
        o = objs[idx]
        o.verifying = False
        o.verified = True
        if c.legacy:
            o.found = o.final_prob >= thr
            o.blocked = False
            blocked.discard(idx)
            return
        if o.confirmed:
            o.found = o.is_mine
            o.t_found = o.t_done
            o.confirm_result = "confirmed"
            if o.in_corridor and c.investigation_time > 0:
                o.block_state = "investigation_confirmed"
                heapq.heappush(excavations, (o.t_done + c.investigation_time, idx))
            else:
                release(o)
        else:
            o.confirm_result = "rejected"
            if c.on_reject == "investigate" and o.in_corridor:
                # SPEC 4.4: a rejected in-corridor suspicion is investigated on the ground.
                o.investigated = True
                if c.investigation_time > 0:
                    o.block_state = "investigation_rejected"
                    heapq.heappush(excavations, (o.t_done + c.investigation_time, idx))
                    return
                end_investigation(o, o.t_done)
            release(o)

    def release(o):
        o.blocked = False
        o.block_state = ""
        blocked.discard(o.idx)

    def end_investigation(o, t_end):
        if o.block_state == "investigation_rejected" or (o.investigated and not o.found):
            o.found = o.is_mine
            if o.is_mine:
                o.t_found = t_end

    def pop_events(t_now):
        while events and events[0][0] <= t_now:
            _, idx, cid = heapq.heappop(events)
            if cid >= 0:
                confirmers[cid].pending -= 1
            complete(idx, t_now)
        while excavations and excavations[0][0] <= t_now:
            t_end, idx = heapq.heappop(excavations)
            o = objs[idx]
            end_investigation(o, t_end)
            release(o)

    def execute_task(conf, sid, scan_size, t_now):
        o = objs[sid]
        del queue[sid]
        geom = c.scan_geom
        sx = o.suspect_x if not math.isnan(o.suspect_x) else o.x
        sy = o.suspect_y if not math.isnan(o.suspect_y) else o.y
        if c.legacy:
            segments = geo.local_lawnmower_segments(sx, sy, geom)
        else:
            sigma = localization_std(c, prim, o.t_arrival, sx, sy)
            side = geo.local_scan_side(geom, sigma, SCAN_SIZES[scan_size])
            segments = geo.local_square_segments(sx, sy, geom, side)
        scan_len = geo.segment_path_length(segments, segments[0][0] if segments else (sx, sy))
        travel = math.hypot((segments[0][0][0] if segments else sx) - conf.pos[0],
                            (segments[0][0][1] if segments else sy) - conf.pos[1])
        transit_speed = float(geom["transit_speed_mps"])
        scan_speed = float(geom["scan_speed_mps"])
        travel_time = travel / max(transit_speed, 0.1)
        scan_time = scan_len / max(scan_speed, 0.1)
        start_scan = t_now + travel_time
        done = start_scan + scan_time

        o.verifying = True
        closest = geo.closest_distance_to_segments((o.x, o.y), segments)
        o.mag_closest_m = closest
        if c.legacy:
            base = detection_probability(o, geom["detection_model"])
            decay = float(geom.get("spatial_decay_m", max(float(geom["swath_m"]) * 0.5, 0.1)))
            o.mag_prob = base * math.exp(-0.5 * (closest / max(decay, 1e-6)) ** 2)
            o.final_prob = combine_probabilities(o.sar_prob, o.mag_prob, c.prior)
            o.posterior = o.final_prob
        else:
            decay = float(geom["spatial_decay_m"])
            sf = math.exp(-0.5 * (closest / max(decay, 1e-6)) ** 2)
            p = c.confirm_sensor.p(o) * sf
            o.mag_prob = p
            o.confirmed = bernoulli(c.U[sid, U_CONFIRM], p)
            o.final_prob = 1.0 if o.confirmed else 0.0
            o.posterior = _posterior(post, o.confirmed)
        heapq.heappush(events, (done, sid, conf.cid))
        conf.pending += 1
        if record_paths:
            first = segments[0][0] if segments else (sx, sy)
            confirmer_paths.append((conf.cid, (conf.pos[0], conf.pos[1]), first, "transit"))
            for s0, s1 in segments:
                confirmer_paths.append((conf.cid, s0, s1, "scan"))
        energy_Wh = 0.0
        energy_Wh += power_for_speed(conf.battery, transit_speed, payload_active=False) * travel_time / 3600.0
        energy_Wh += power_for_speed(conf.battery, scan_speed, payload_active=True) * scan_time / 3600.0
        conf.energy_Wh += energy_Wh
        conf.pos = segments[-1][1] if segments else (sx, sy)
        conf.available_t = done
        conf.busy_s += travel_time + scan_time
        o.t_assigned = t_now
        o.t_done = done
        o.confirmer_id = conf.cid
        o.travel_s = travel_time
        o.scan_s = scan_time

    def assign(t_now, lane_idx, lane_y, direction, tractor_x, frontier_x, is_blocked):
        free = [cf for cf in confirmers if cf.available_t <= t_now]
        if not free:
            return
        for cf in free:
            cf.idle_target = None
        if not queue:
            return
        items = list(queue.values())
        for it in items:
            o = objs[it.sid]
            it.ahead = o.lane_idx == lane_idx and direction * (o.suspect_x - tractor_x) >= 0.0
        state = PolicyState(
            t=t_now, queue=items,
            confirmers=[ConfirmerView(cf.cid, cf.pos[0], cf.pos[1], cf.available_t,
                                      battery_remaining_percent(cf.energy_Wh, cf.battery), cf.available_t <= t_now)
                        for cf in confirmers],
            vehicle=VehicleView(tractor_x, lane_idx, lane_y, direction, is_blocked),
            frontier_x=frontier_x, legacy_ordering=c.legacy,
        )
        for a in c.policy(state):
            cf = confirmers[a.confirmer_id]
            if cf.available_t > t_now:
                continue
            if a.suspicion_id is not None and a.suspicion_id in queue:
                execute_task(cf, a.suspicion_id, a.scan_size or c.scan_size, t_now)
            elif a.idle_point is not None:
                cf.idle_target = a.idle_point

    def detect_legacy(layer, positions, lane_idx, lane_y, t_now):
        scfg = layer.legacy_cfg
        r = layer.radius
        dec = cfg["decision"]
        for o in objs:
            if o.sar_seen:
                continue
            hit = False
            for x, y in positions:
                if (o.x - x) ** 2 + (o.y - y) ** 2 <= r * r:
                    hit = True
                    break
            if hit:
                o.sar_seen = True
                o.sar_prob = detection_probability(o, scfg["detection_model"])
                if o.sar_prob >= float(dec["sar_suspicious_min"]):
                    o.suspicious = True
                    o.screen_detected = True
                    rng = c.loc_rng[0]
                    loc_std = float(scfg.get("localization_std_m", 0.0))
                    o.suspect_x = o.x + float(rng.normal(0.0, loc_std))
                    o.suspect_y = o.y + float(rng.normal(0.0, loc_std))
                    o.suspect_x = max(x_min_f, min(x_max_f, o.suspect_x))
                    o.suspect_y = max(y_min_f, min(y_max_f, o.suspect_y))
                    enqueue(o, lane_idx, lane_y, t_now, o.sar_prob)

    def enqueue(o, lane_idx, lane_y, t_now, score):
        nonlocal n_queued
        o.lane_idx = lane_idx
        o.in_corridor = abs((o.y if c.legacy else o.suspect_y) - lane_y) <= c.corridor_half
        o.t_arrival = t_now
        if c.M == 0:
            # Survey only: every detection is final.
            o.verified = True
            o.confirmed = True
            o.found = o.is_mine
            o.confirm_result = "final"
            o.t_assigned = t_now
            o.t_done = t_now
            o.t_found = t_now
            o.final_prob = 1.0
            o.posterior = post["prior"] if post else math.nan
            if o.in_corridor and c.investigation_time > 0:
                o.blocked = True
                o.block_state = "investigation_confirmed"
                blocked.add(o.idx)
                heapq.heappush(excavations, (t_now + c.investigation_time, o.idx))
            return
        queue[o.idx] = QueueItem(o.idx, o.suspect_x, o.suspect_y, (0.0, 0.0), t_now, o.in_corridor, score, True)
        n_queued += 1
        o.blocked = True
        o.block_state = "unresolved"
        blocked.add(o.idx)

    def detect_rate(layer, res, lane_idx, lane_y, t_now):
        r = layer.radius
        hits = set()
        for segs in res.scan_segments:
            for p0, p1 in segs:
                geo.mark_capsule(coverage, fld, p0, p1, r)
                lo = min(p0[0], p1[0]) - r
                hi = max(p0[0], p1[0]) + r
                a = int(np.searchsorted(c.sorted_x, lo, side="left"))
                b = int(np.searchsorted(c.sorted_x, hi, side="right"))
                if b <= a:
                    continue
                cand = c.order[a:b]
                cand = cand[~layer.seen[cand]]
                if cand.size == 0:
                    continue
                d = geo.seg_dist_array(c.ox[cand], c.oy[cand], p0, p1)
                for idx in cand[d <= r + 1e-12]:
                    hits.add(int(idx))
        if not hits:
            return
        sensor = layer.sensor
        score = sensor.pd_class_depth("metal", 0.05, 12.0) if sensor.model != "imaging" else sensor.pd_surface
        for idx in sorted(hits):
            layer.seen[idx] = True
            o = objs[idx]
            o.sar_seen = True
            pd = sensor.pd(o, hour(t_now))
            if layer.name == "primary":
                o.screen_pd = pd
            if not bernoulli(c.U[idx, layer.u_col], pd):
                continue
            o.screen_detected = True
            o.detected_by = o.detected_by + ("+" if o.detected_by else "") + layer.name
            if o.suspicious:
                continue
            o.suspicious = True
            o.sar_prob = pd
            rng = c.loc_rng[int(c.stream_of[idx])]
            std = localization_std(c, layer, t_now, o.x, o.y)
            o.suspect_x = o.x + float(rng.normal(0.0, std))
            o.suspect_y = o.y + float(rng.normal(0.0, std))
            o.suspect_x = max(x_min_f, min(x_max_f, o.suspect_x))
            o.suspect_y = max(y_min_f, min(y_max_f, o.suspect_y))
            enqueue(o, lane_idx, lane_y, t_now, score)

    lanes = c.lanes
    for lane_idx, lane_y in enumerate(lanes):
        direction = 1 if lane_idx % 2 == 0 else -1
        x_start = x_min_f if direction == 1 else x_max_f
        x_end = x_max_f if direction == 1 else x_min_f
        for layer in c.layers:
            if c.legacy:
                layer.planner = geo.LaneAbreastPlanner(lane_y, direction, x_start, x_end, layer.n, layer.sep, layer.speed)
            elif layer.name == "surface":
                layer.planner = geo.LaneAbreastPlanner(lane_y, direction, x_start, x_end, layer.n, layer.sep, layer.speed)
            else:
                layer.planner = geo.make_planner(
                    team["screening_mode"], lane_y, direction, x_start, x_end, layer.n, layer.swath,
                    layer.overlap, layer.speed, c.W, float(team.get("segment_length_m", 10.0)),
                    float(team.get("turn_time_s", 3.0)))
        tractor_x = x_start
        tractor_done = False
        # objects inside this lane's corridor (true position), in driving order, for reach times
        in_lane = [o.idx for o in objs if abs(o.y - lane_y) <= c.corridor_half]
        in_lane.sort(key=lambda i: direction * objs[i].x)
        reach_ptr = 0

        while not (all(l.planner.done for l in c.layers) and tractor_done):
            if t >= max_time:
                mission_complete = False
                stop_reason = "mission_time_limit"
                stopped = True
                break
            reason = None
            for layer in (c.layers if c.battery_enforce else []):
                prefix = "screener" if layer.name == "primary" else "surface"
                for i, e in enumerate(layer.energy_Wh):
                    if battery_remaining_percent(e, layer.battery) <= c.reserve:
                        reason = f"{prefix}_{i}_battery_reserve_reached"
                        break
                if reason:
                    break
            if reason is None and c.battery_enforce:
                for cf in confirmers:
                    if battery_remaining_percent(cf.energy_Wh, cf.battery) <= c.reserve:
                        reason = f"confirmer_{cf.cid}_battery_reserve_reached"
                        break
            if reason:
                mission_complete = False
                stop_reason = reason
                stopped = True
                break

            pop_events(t)
            frontier_now = _combined_frontier(c.layers, direction)
            assign(t, lane_idx, lane_y, direction, tractor_x, frontier_now, False)

            for layer in c.layers:
                pl = layer.planner
                if pl.done:
                    continue
                x_before = pl.frontier_x
                res = pl.step(dt)
                for i in range(layer.n):
                    e = layer.energy_Wh[i]
                    for spd, payload, dur in res.energy_legs:
                        e = consume_energy(e, layer.battery, spd, dur, payload_active=payload)
                    layer.energy_Wh[i] = e
                if record_paths:
                    for i, pos in enumerate(res.positions):
                        layer.paths[i].append(pos)
                layer.active_s += sum(d for _, _, d in res.energy_legs)
                layer.screened_len += abs(pl.frontier_x - x_before)
                if c.legacy:
                    for pos in res.positions:
                        geo.mark_disc(coverage, fld, pos[0], pos[1], layer.radius)
                    detect_legacy(layer, res.positions, lane_idx, lane_y, t)
                else:
                    detect_rate(layer, res, lane_idx, lane_y, t)
            max_queue = max(max_queue, len(queue))

            frontier_x = _combined_frontier(c.layers, direction)
            cleared_frontier = frontier_x - direction * c.safe_distance
            if not c.legacy and all(l.planner.done for l in c.layers):
                # Lane fully screened and screeners gone: the whole lane is open (v1 relies on
                # PID integral wind-up to cover the last safe_distance; kept only in legacy mode).
                cleared_frontier = x_end
            if direction == 1:
                desired_x = max(x_start, min(cleared_frontier, x_end))
                tractor_done = tractor_x >= x_end
            else:
                desired_x = min(x_start, max(cleared_frontier, x_end))
                tractor_done = tractor_x <= x_end

            binding = None
            is_blocked = False
            if c.block_mode == "corridor_ahead":
                x0, x1 = sorted((tractor_x, frontier_x))
                for idx in sorted(blocked):
                    o = objs[idx]
                    ox_, oy_ = (o.x, o.y) if c.legacy else (o.suspect_x, o.suspect_y)
                    if not c.legacy and o.lane_idx != lane_idx:
                        continue
                    if abs(oy_ - lane_y) > c.corridor_half:
                        continue
                    if x0 <= ox_ <= x1:
                        is_blocked = True
                        binding = o
                        break
            else:
                best = None
                for idx in blocked:
                    o = objs[idx]
                    if o.lane_idx != lane_idx or not o.in_corridor:
                        continue
                    ahead = direction * (o.suspect_x - tractor_x)
                    if ahead < -c.standoff:
                        continue
                    if best is None or direction * (o.suspect_x - best.suspect_x) < 0:
                        best = o
                if best is not None:
                    limit = best.suspect_x - direction * c.standoff
                    if direction * (limit - desired_x) < 0:
                        desired_x = limit
                        binding = best
                        if direction * (limit - tractor_x) <= 0:
                            is_blocked = True

            if is_blocked:
                target_speed = 0.0
            else:
                error = direction * (desired_x - tractor_x)
                tractor_integral += max(-50.0, min(50.0, error)) * dt
                derivative = (error - tractor_prev_error) / max(dt, 1e-6)
                tractor_prev_error = error
                pid = cfg["tractor"]["pid"]
                target_speed = (
                    float(pid["kp"]) * error
                    + float(pid["ki"]) * tractor_integral
                    + float(pid.get("kd", 0.0)) * derivative
                )
                target_speed = max(0.0, min(c.max_speed, target_speed))

            prev_x = tractor_x
            tractor_x += direction * target_speed * dt
            if direction == 1:
                tractor_x = min(tractor_x, x_end)
            else:
                tractor_x = max(tractor_x, x_end)
            tractor_dist += abs(tractor_x - prev_x)
            if record_paths:
                tractor_path.append((tractor_x, lane_y))

            if abs(tractor_x - prev_x) > 1e-9:
                time_split["moving"] += dt
            elif tractor_done:
                time_split["done_waiting"] += dt
            elif binding is not None and (is_blocked or c.block_mode == "standoff"):
                key = {"investigation_confirmed": "wait_investigation_confirmed",
                       "investigation_rejected": "wait_investigation"}.get(binding.block_state, "wait_confirmation")
                time_split[key] += dt
            else:
                time_split["wait_frontier"] += dt

            while reach_ptr < len(in_lane) and direction * (tractor_x - objs[in_lane[reach_ptr]].x) >= 0.0:
                o = objs[in_lane[reach_ptr]]
                if math.isinf(o.t_vehicle_reach):
                    o.t_vehicle_reach = t + dt
                reach_ptr += 1

            for cf in confirmers:
                if cf.available_t <= t and not queue and cf.pending == 0:
                    if cf.idle_target is not None:
                        idle_target = cf.idle_target
                    else:
                        idle_x = tractor_x + direction * c.safe_distance
                        idle_x = max(x_min_f, min(x_max_f, idle_x))
                        idle_y = lane_y + (cf.cid - (c.M - 1) / 2) * float(team["confirmer_idle_spacing_m"]) if c.M > 1 else lane_y
                        idle_target = (idle_x, idle_y)
                    nxt = geo.move_toward(cf.pos, idle_target, c.transit_speed * dt)
                    if math.hypot(nxt[0] - cf.pos[0], nxt[1] - cf.pos[1]) > 1e-6:
                        if record_paths:
                            confirmer_paths.append((cf.cid, cf.pos, nxt, "idle"))
                        idle_dist = math.hypot(nxt[0] - cf.pos[0], nxt[1] - cf.pos[1])
                        idle_speed = idle_dist / max(dt, 1e-6)
                        cf.energy_Wh = consume_energy(cf.energy_Wh, cf.battery, idle_speed, dt, payload_active=False)
                        cf.pos = nxt
            t += dt
        if stopped:
            break

    drain_t0 = t
    if not stopped:
        if c.legacy:
            while events:
                done_t, idx, cid = heapq.heappop(events)
                t = max(t, done_t)
                if cid >= 0:
                    confirmers[cid].pending -= 1
                complete(idx, t)
        else:
            # Serve what is left (suspicions outside the corridor) so every suspicion is resolved.
            last = lanes[-1]
            ldir = 1 if (len(lanes) - 1) % 2 == 0 else -1
            while events or queue:
                if t >= max_time:
                    mission_complete = False
                    stop_reason = "mission_time_limit"
                    break
                assign(t, len(lanes) - 1, last, ldir, x_max_f if ldir == 1 else x_min_f,
                       x_max_f if ldir == 1 else x_min_f, False)
                if not events:
                    break
                done_t = events[0][0]
                t = max(t, done_t)
                pop_events(t)
            while excavations:
                t_end, idx = heapq.heappop(excavations)
                end_investigation(objs[idx], t_end)
                objs[idx].blocked = False
                objs[idx].block_state = ""
    time_split["done_waiting"] += t - drain_t0

    result = {
        "cfg": cfg, "ctx": c, "mines": objs, "objects": objs, "coverage": coverage,
        "sar_paths": prim.paths, "screener_paths": [l.paths for l in c.layers],
        "tractor_path": tractor_path, "mag_paths": confirmer_paths,
        "confirmers": confirmers, "elapsed_s": t,
    }
    result["metrics"] = build_metrics(c, result, t, stop_reason, mission_complete, confirmers,
                                      time_split, tractor_dist, max_queue, n_queued, drain_t0)
    result["queue_log"] = build_queue_log(objs)
    return result


def _combined_frontier(layers, direction):
    xs = [l.planner.frontier_x for l in layers]
    return min(xs) if direction == 1 else max(xs)


# --------------------------------------------------------------------------- posterior (reporting only)

def _posterior_inputs(c):
    cfg = c.cfg
    ss = c.screen_sensor
    rho_mine = float(c.meta.get("density_per_m2", 0.0)) or 1e-12
    cl = cfg.get("clutter", {})
    if ss.model == "generic":
        rho_cd = ss.false_alarms_per_m2
    else:
        dens = float(cl.get("density_per_m2", 0.0))
        classes = cl.get("classes", {"metal_clutter": 0.8, "nonmetal_clutter": 0.2})
        tot = sum(float(v) for v in classes.values()) or 1.0
        depth = float(cl.get("depth_m", {}).get("mean", 0.05))
        rho_cd = sum(dens * float(w) / tot * ss.pd_class_depth(k, depth) for k, w in classes.items())
    prior = rho_mine / (rho_mine + rho_cd) if (rho_mine + rho_cd) > 0 else 0.5
    if c.confirm_sensor is None:
        return {"prior": prior, "p_mine": 1.0, "p_clutter": 1.0}
    shares = selection_class_depth_distribution(c.sampler)
    pm = sum(v["share"] * c.confirm_sensor.p_class_depth(k, v["mean_depth"]) for k, v in shares.items())
    pc = c.confirm_sensor.p_class_depth("metal_clutter", 0.05)
    return {"prior": prior, "p_mine": pm, "p_clutter": pc}


def _posterior(post, confirmed):
    pi = post["prior"]
    lm = post["p_mine"] if confirmed else 1.0 - post["p_mine"]
    lc = post["p_clutter"] if confirmed else 1.0 - post["p_clutter"]
    den = pi * lm + (1 - pi) * lc
    return pi * lm / den if den > 0 else math.nan


# --------------------------------------------------------------------------- outcomes / metrics

def object_outcome(o):
    if o.is_mine:
        if not o.screen_detected:
            return "screening_miss"
        if o.found and o.investigated:
            return "found_by_investigation"
        if o.found:
            return "true_positive"
        if o.verified:
            return "confirmation_miss"
        return "unresolved"
    if not o.screen_detected:
        return "undetected_clutter"
    if o.confirmed and o.verified:
        return "false_confirmation"
    if o.verified:
        return "rejected"
    return "unresolved"


def build_queue_log(objs):
    rows = []
    for o in objs:
        if not o.suspicious:
            continue
        rows.append({
            "object_idx": o.idx, "is_mine": int(o.is_mine), "class": o.cls, "depth_m": o.depth_m,
            "t_arrival": o.t_arrival, "t_assigned": o.t_assigned, "t_done": o.t_done,
            "confirmer_id": o.confirmer_id, "travel_s": o.travel_s, "scan_s": o.scan_s,
            "in_corridor": int(o.in_corridor), "confirmed": int(o.confirmed if o.verified else 0),
            "posterior": o.posterior,
        })
    return rows


def _nan_to_none(v):
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    return v


def build_metrics(c, result, elapsed_s, stop_reason, mission_complete, confirmers, time_split,
                  tractor_dist, max_queue, n_queued, drain_t0):
    objs = c.objects
    fld = c.fld
    for o in objs:
        o.outcome = object_outcome(o)
    natural = [o for o in objs if o.is_mine and not o.injected]
    clutter = [o for o in objs if not o.is_mine]
    prim = c.layers[0]
    scanned_area = geo.count_scanned_field_cells(result["coverage"], fld) * float(fld["grid_res"]) ** 2

    screener_E = list(prim.energy_Wh)
    surface_E = [e for l in c.layers[1:] for e in l.energy_Wh]
    conf_E = [cf.energy_Wh for cf in confirmers]
    m = {
        "mission_complete": mission_complete,
        "stop_reason": stop_reason,
        "elapsed_s": elapsed_s,
        "sar_energy_used_Wh": screener_E[0] if screener_E else 0.0,
        "mag_energy_used_Wh": conf_E[0] if conf_E else 0.0,
        "sar_battery_remaining_percent": battery_remaining_percent(screener_E[0], prim.battery) if screener_E else None,
        "mag_battery_remaining_percent": battery_remaining_percent(conf_E[0], c.confirm_battery) if conf_E else None,
        "battery_reserve_percent": c.reserve,
        "scanned_area_m2": scanned_area,
        "mine_count": len(natural),
        "found_mines": sum(1 for o in natural if o.found),
        "missed_mines": sum(1 for o in natural if not o.found),
        "sar_suspicious": sum(1 for o in natural if o.suspicious),
        "magnetometer_verifications": sum(1 for o in natural if o.verified),
        # team / preset
        "detection_mode": c.mode,
        "screening_sensor": c.team["screening_sensor"],
        "confirming_sensor": c.team["confirming_sensor"],
        "sensor_preset": c.team["sensor_preset"],
        "confirmer_preset": c.confirm_sensor.preset if c.confirm_sensor else None,
        "surface_layer": (c.team.get("surface_layer") or {}).get("sensor"),
        "screening_mode": c.team["screening_mode"],
        "n_screeners": c.N,
        "n_confirmers": c.M,
        "confirmer_policy": c.team["confirmer_policy"] if isinstance(c.team["confirmer_policy"], str) else "custom",
        "projection": bool(c.projection),
        "area_rate_m2ps": prim.swath * prim.speed,
        "screener_swath_m": prim.swath,
        "screener_speed_mps": prim.speed,
        "corridor_width_m": c.W,
    }
    # safety (natural mines inside some lane corridor)
    relevant = [o for o in natural if any(abs(o.y - ly) <= c.corridor_half for ly in c.lanes)]
    m["recall"] = (sum(o.found for o in relevant) / len(relevant)) if relevant else None
    for cls in MINE_CLASSES:
        rc = [o for o in relevant if o.cls == cls]
        m[f"recall_{cls}"] = (sum(o.found for o in rc) / len(rc)) if rc else None
        m[f"screening_misses_{cls}"] = sum(1 for o in rc if o.outcome == "screening_miss")
        m[f"confirmation_misses_{cls}"] = sum(1 for o in rc if o.outcome == "confirmation_miss")
    m["screening_misses"] = sum(1 for o in relevant if o.outcome == "screening_miss")
    m["confirmation_misses"] = sum(1 for o in relevant if o.outcome == "confirmation_miss")
    m["residual_mines_in_corridor"] = sum(1 for o in relevant if not o.found)
    # population
    m["population_regime"] = c.meta.get("regime", "legacy")
    m["n_mines_natural"] = len(natural)
    m["population_density_per_m2"] = c.meta.get("density_per_m2", len(natural) / field_area(fld))
    by_type, by_class, depth_class = {}, {}, {}
    for o in natural:
        by_type[o.mine_type] = by_type.get(o.mine_type, 0) + 1
        by_class[o.cls] = by_class.get(o.cls, 0) + 1
        depth_class.setdefault(o.cls, []).append(o.depth_m)
    m["mines_per_type"] = by_type
    m["mines_per_class"] = by_class
    m["mean_depth_per_class"] = {k: float(np.mean(v)) for k, v in depth_class.items()}
    m["n_clutter"] = len(clutter)
    # workload
    susp = [o for o in objs if o.suspicious and not o.injected]
    fa = [o for o in susp if not o.is_mine]
    tp = sum(1 for o in natural if o.outcome == "true_positive")
    fc = sum(1 for o in clutter if o.outcome == "false_confirmation")
    m["n_suspicions"] = len(susp)
    m["n_false_alarms"] = len(fa)
    m["false_confirmations"] = fc
    m["n_investigated_rejections"] = sum(1 for o in objs if o.investigated and not o.injected)
    m["n_found_by_investigation"] = sum(1 for o in natural if o.outcome == "found_by_investigation")
    m["on_reject"] = c.on_reject
    m["investigation_time_s"] = c.investigation_time
    m["precision"] = tp / (tp + fc) if (tp + fc) > 0 else None
    m["false_alarms_per_m2"] = len(fa) / scanned_area if scanned_area > 0 else None
    # throughput
    vehicle_time = drain_t0
    m["vehicle_distance_m"] = tractor_dist
    m["vehicle_time_s"] = vehicle_time
    m["mean_vehicle_speed_mps"] = tractor_dist / vehicle_time if vehicle_time > 0 else 0.0
    m["frontier_speed_mps"] = prim.screened_len / prim.active_s if prim.active_s > 0 else 0.0
    m["coverage_efficiency"] = (m["frontier_speed_mps"] * c.W / (c.N * prim.swath * prim.speed)
                                if prim.speed > 0 else None)
    m["tractor_moving_s"] = time_split["moving"]
    m["tractor_wait_confirmation_s"] = time_split["wait_confirmation"]
    m["tractor_wait_frontier_s"] = time_split["wait_frontier"]
    m["tractor_wait_investigation_confirmed_s"] = time_split["wait_investigation_confirmed"]
    m["tractor_wait_investigation_s"] = time_split["wait_investigation"]
    # Deprecated alias (v2.0 name), kept for one release; not part of the time split.
    m["tractor_wait_excavation_s"] = time_split["wait_investigation_confirmed"]
    m["tractor_done_waiting_s"] = time_split["done_waiting"]
    # energy
    m["screener_energy_Wh"] = screener_E
    m["surface_energy_Wh"] = surface_E
    m["confirmer_energy_Wh"] = conf_E
    m["fleet_energy_screening_Wh"] = sum(screener_E) + sum(surface_E)
    m["fleet_energy_confirming_Wh"] = sum(conf_E)
    m["fleet_energy_Wh"] = m["fleet_energy_screening_Wh"] + m["fleet_energy_confirming_Wh"]
    cleared_area = tractor_dist * c.W
    m["energy_per_m2_cleared_Wh"] = m["fleet_energy_Wh"] / cleared_area if cleared_area > 0 else None
    # queue
    served = [o for o in objs if not math.isnan(o.t_assigned) and o.confirmer_id >= 0]
    services = np.array([o.travel_s + o.scan_s for o in served]) if served else np.array([])
    waits = np.array([o.t_assigned - o.t_arrival for o in served]) if served else np.array([])
    m["confirmer_busy_s"] = [cf.busy_s for cf in confirmers]
    m["confirmer_utilization"] = (sum(cf.busy_s for cf in confirmers) / (len(confirmers) * elapsed_s)
                                  if confirmers and elapsed_s > 0 else None)
    m["mean_queue_wait_s"] = float(waits.mean()) if waits.size else None
    m["max_queue_length"] = max_queue
    m["mean_service_time_s"] = float(services.mean()) if services.size else None
    m["scv_service_time"] = float(services.var() / services.mean() ** 2) if services.size > 1 and services.mean() > 0 else None
    m["n_queued"] = n_queued
    m["arrival_rate_per_s"] = n_queued / prim.active_s if prim.active_s > 0 else None
    m["calibration"] = c.calibration
    return {k: _nan_to_none(v) for k, v in m.items()}


# --------------------------------------------------------------------------- injection (SPEC 4.5.4)

def injection_metrics(result, prefix="inj_"):
    inj = [o for o in result["objects"] if o.injected]
    m = {}

    def before(o):
        return o.found and o.t_found <= o.t_vehicle_reach

    k = sum(1 for o in inj if before(o))
    p, lo, hi = wilson(k, len(inj))
    m[f"{prefix}n"] = len(inj)
    m[f"{prefix}p_detect"] = p
    m[f"{prefix}p_detect_lo"] = lo
    m[f"{prefix}p_detect_hi"] = hi
    for cls in MINE_CLASSES:
        oc = [o for o in inj if o.cls == cls]
        kc = sum(1 for o in oc if before(o))
        p, lo, hi = wilson(kc, len(oc))
        m[f"{prefix}n_{cls}"] = len(oc)
        m[f"{prefix}detected_{cls}"] = kc
        m[f"{prefix}p_detect_{cls}"] = p
        m[f"{prefix}p_detect_{cls}_lo"] = lo
        m[f"{prefix}p_detect_{cls}_hi"] = hi
        m[f"{prefix}unscreened_{cls}"] = sum(1 for o in oc if not o.sar_seen)
        m[f"{prefix}screen_miss_{cls}"] = sum(1 for o in oc if o.sar_seen and not o.screen_detected)
        m[f"{prefix}confirm_miss_{cls}"] = sum(1 for o in oc if o.screen_detected and not o.found)
        m[f"{prefix}late_{cls}"] = sum(1 for o in oc if o.found and o.t_found > o.t_vehicle_reach)
        m[f"{prefix}found_by_investigation_{cls}"] = sum(1 for o in oc if o.found and o.investigated)
    rm = result["metrics"]
    for key in ("elapsed_s", "mean_vehicle_speed_mps", "fleet_energy_Wh", "n_suspicions", "stop_reason"):
        m[f"{prefix}{key}"] = rm[key]
    return {k: _nan_to_none(v) for k, v in m.items()}


def run_mission(cfg, sensor_lib=None, mine_types=None, record_paths=True):
    """Natural run plus, for rate_based with injection enabled, a separate injection pass."""
    result = simulate(cfg, sensor_lib, mine_types, inject=0, record_paths=record_paths)
    inj_cfg = cfg.get("injection", {})
    if cfg.get("detection_mode", "legacy_logistic") == "rate_based" and inj_cfg.get("enabled", True):
        n = int(inj_cfg.get("n_inject", 20))
        inj_result = simulate(copy.deepcopy(cfg), sensor_lib, mine_types, inject=n, record_paths=False)
        im = injection_metrics(inj_result)
        c = result["ctx"]
        rho = float(result["metrics"]["population_density_per_m2"] or 0.0)
        pbar = im["inj_p_detect"]
        im["residual_risk_per_km"] = (rho * c.W * 1000.0 * (1.0 - pbar)) if pbar is not None else None
        result["metrics"].update(im)
        result["injection_objects"] = [o for o in inj_result["objects"] if o.injected]
    return result
