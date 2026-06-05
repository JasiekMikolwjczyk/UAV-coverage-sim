#!/usr/bin/env python3
import argparse
import csv
import heapq
import json
import math
import os
import time
from dataclasses import dataclass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
import numpy as np


@dataclass
class Mine:
    idx: int
    x: float
    y: float
    material: str
    shape: str
    depth_m: float
    size_m: float
    sar_prob: float = math.nan
    mag_prob: float = math.nan
    final_prob: float = math.nan
    suspect_x: float = math.nan
    suspect_y: float = math.nan
    mag_closest_m: float = math.nan
    sar_seen: bool = False
    suspicious: bool = False
    verifying: bool = False
    verified: bool = False
    found: bool = False
    blocked: bool = False


def sigmoid(v: float) -> float:
    return 1.0 / (1.0 + math.exp(-max(-60.0, min(60.0, v))))


def choose_weighted(rng, weights: dict):
    keys = list(weights.keys())
    probs = np.array([float(weights[k]) for k in keys], dtype=np.float64)
    probs = probs / probs.sum()
    return keys[int(rng.choice(len(keys), p=probs))]


def clipped_normal(rng, spec: dict):
    value = float(rng.normal(float(spec["mean"]), float(spec["std"])))
    return max(float(spec["min"]), min(float(spec["max"]), value))


def generate_mines(cfg: dict):
    rng = np.random.default_rng(int(cfg.get("seed", 1)))
    field = cfg["field"]
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
                rng.uniform(float(field["y_min"]), float(field["y_max"]), size=count),
            )
        )
    elif mode == "uniform_xy":
        points = np.column_stack(
            (
                rng.uniform(float(field["x_min"]), float(field["x_max"]), size=count),
                rng.uniform(float(field["y_min"]), float(field["y_max"]), size=count),
            )
        )
    elif mode == "gaussian_xy":
        points = rng.normal(mean, std, size=(count, 2))
    else:
        raise ValueError(f"Unsupported mine_distribution.mode: {mode}")
    points[:, 0] = np.clip(points[:, 0], float(field["x_min"]), float(field["x_max"]))
    points[:, 1] = np.clip(points[:, 1], float(field["y_min"]), float(field["y_max"]))

    mines = []
    for idx, (x, y) in enumerate(points):
        mines.append(
            Mine(
                idx=idx,
                x=float(x),
                y=float(y),
                material=choose_weighted(rng, props["materials"]),
                shape=choose_weighted(rng, props["shapes"]),
                depth_m=clipped_normal(rng, props["depth_m"]),
                size_m=clipped_normal(rng, props["size_m"]),
            )
        )
    return mines


def detection_probability(mine: Mine, model: dict):
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


def magnetometer_probability(mine: Mine, mag_cfg: dict, scan_segments):
    base_prob = detection_probability(mine, mag_cfg["detection_model"])
    closest = closest_distance_to_segments((mine.x, mine.y), scan_segments)
    mine.mag_closest_m = closest
    decay = float(mag_cfg.get("spatial_decay_m", max(float(mag_cfg["swath_m"]) * 0.5, 0.1)))
    spatial_factor = math.exp(-0.5 * (closest / max(decay, 1e-6)) ** 2)
    return base_prob * spatial_factor


def combine_probabilities(sar_prob: float, mag_prob: float):
    sar_odds = sar_prob / max(1e-6, 1.0 - sar_prob)
    mag_odds = mag_prob / max(1e-6, 1.0 - mag_prob)
    odds = sar_odds * mag_odds
    return odds / (1.0 + odds)


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


def make_grid(field: dict):
    x_min, x_max, y_min, y_max = coverage_bounds(field)
    res = float(field["grid_res"])
    nx = int(math.ceil((x_max - x_min) / res))
    ny = int(math.ceil((y_max - y_min) / res))
    return np.zeros((ny, nx), dtype=np.float32)


def coverage_bounds(field: dict):
    margin = float(field.get("terrain_margin_m", 0.0))
    return (
        float(field["x_min"]) - margin,
        float(field["x_max"]) + margin,
        float(field["y_min"]) - margin,
        float(field["y_max"]) + margin,
    )


def mark_disc(grid, field, x: float, y: float, radius: float):
    res = float(field["grid_res"])
    x_min, _, y_min, _ = coverage_bounds(field)
    ix0 = max(0, int(math.floor((x - radius - x_min) / res)))
    ix1 = min(grid.shape[1] - 1, int(math.ceil((x + radius - x_min) / res)))
    iy0 = max(0, int(math.floor((y - radius - y_min) / res)))
    iy1 = min(grid.shape[0] - 1, int(math.ceil((y + radius - y_min) / res)))
    rr = radius * radius
    for iy in range(iy0, iy1 + 1):
        cy = y_min + (iy + 0.5) * res
        dy2 = (cy - y) * (cy - y)
        for ix in range(ix0, ix1 + 1):
            cx = x_min + (ix + 0.5) * res
            if (cx - x) * (cx - x) + dy2 <= rr:
                grid[iy, ix] = 1.0


def mine_in_sar_footprint(mine: Mine, drone_positions, radius: float):
    for x, y in drone_positions:
        if (mine.x - x) ** 2 + (mine.y - y) ** 2 <= radius * radius:
            return True
    return False


def unresolved_blocks(mines, lane_y: float, tractor_x: float, frontier_x: float, direction: int, corridor_half_width: float):
    x0, x1 = sorted((tractor_x, frontier_x))
    for mine in mines:
        if not mine.blocked:
            continue
        if abs(mine.y - lane_y) > corridor_half_width:
            continue
        if x0 <= mine.x <= x1:
            return True
    return False


def enqueue_suspicion(queue, mine: Mine):
    heapq.heappush(queue, (-mine.sar_prob, mine.idx))
    mine.blocked = True


def local_lawnmower_segments(cx: float, cy: float, mag_cfg: dict):
    area = float(mag_cfg.get("local_scan_area_m", mag_cfg.get("line_length_m", 6.0)))
    line_len = float(mag_cfg.get("line_length_m", area))
    swath = float(mag_cfg["swath_m"])
    overlap = float(mag_cfg.get("transverse_overlap", 0.5))
    spacing = max(0.1, swath * (1.0 - overlap))
    half_area = 0.5 * area
    half_line = 0.5 * line_len

    offsets = []
    x = -half_area
    while x <= half_area + 1e-9:
        offsets.append(x)
        x += spacing
    if offsets and offsets[-1] < half_area:
        offsets.append(half_area)

    segments = []
    reverse = False
    for dx in offsets:
        x_line = cx + dx
        if reverse:
            start = (x_line, cy + half_line)
            end = (x_line, cy - half_line)
        else:
            start = (x_line, cy - half_line)
            end = (x_line, cy + half_line)
        segments.append((start, end))
        reverse = not reverse
    return segments


def point_to_segment_distance(point, start, end):
    px, py = point
    sx, sy = start
    ex, ey = end
    vx = ex - sx
    vy = ey - sy
    length2 = vx * vx + vy * vy
    if length2 <= 1e-12:
        return math.hypot(px - sx, py - sy)
    u = max(0.0, min(1.0, ((px - sx) * vx + (py - sy) * vy) / length2))
    cx = sx + u * vx
    cy = sy + u * vy
    return math.hypot(px - cx, py - cy)


def closest_distance_to_segments(point, segments):
    if not segments:
        return math.inf
    return min(point_to_segment_distance(point, start, end) for start, end in segments)


def segment_path_length(segments, start_pos):
    if not segments:
        return 0.0
    total = math.hypot(segments[0][0][0] - start_pos[0], segments[0][0][1] - start_pos[1])
    for idx, (start, end) in enumerate(segments):
        total += math.hypot(end[0] - start[0], end[1] - start[1])
        if idx + 1 < len(segments):
            next_start = segments[idx + 1][0]
            total += math.hypot(next_start[0] - end[0], next_start[1] - end[1])
    return total


def assign_mag_task(
    t: float,
    queue,
    mines,
    mag_cfg,
    decision,
    current_pos,
    events,
    mag_paths,
    mag_battery,
):
    while queue:
        _, idx = heapq.heappop(queue)
        mine = mines[idx]
        if mine.verified or mine.verifying:
            continue
        transit_speed = float(mag_cfg["transit_speed_mps"])
        scan_speed = float(mag_cfg["scan_speed_mps"])
        sx = mine.suspect_x if not math.isnan(mine.suspect_x) else mine.x
        sy = mine.suspect_y if not math.isnan(mine.suspect_y) else mine.y
        segments = local_lawnmower_segments(sx, sy, mag_cfg)
        scan_len = segment_path_length(segments, segments[0][0] if segments else (sx, sy))
        travel = math.hypot((segments[0][0][0] if segments else sx) - current_pos[0],
                            (segments[0][0][1] if segments else sy) - current_pos[1])
        travel_time = travel / max(transit_speed, 0.1)
        scan_time = scan_len / max(scan_speed, 0.1)
        start_scan = t + travel_time
        done = start_scan + scan_time

        mine.verifying = True
        mine.mag_prob = magnetometer_probability(mine, mag_cfg, segments)
        mine.final_prob = combine_probabilities(mine.sar_prob, mine.mag_prob)
        heapq.heappush(events, (done, idx))
        first_scan = segments[0][0] if segments else (sx, sy)
        mag_paths.append(((current_pos[0], current_pos[1]), first_scan, "transit"))
        for start, end in segments:
            mag_paths.append((start, end, "scan"))
        energy_Wh = 0.0
        energy_Wh += power_for_speed(mag_battery, transit_speed, payload_active=False) * travel_time / 3600.0
        energy_Wh += power_for_speed(mag_battery, scan_speed, payload_active=True) * scan_time / 3600.0
        next_pos = segments[-1][1] if segments else (sx, sy)
        used_time = travel_time + scan_time
        return done, next_pos, used_time, energy_Wh
    return t, current_pos, 0.0, 0.0


def move_toward(pos, target, max_dist: float):
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dist = math.hypot(dx, dy)
    if dist <= max_dist or dist <= 1e-9:
        return target
    scale = max_dist / dist
    return pos[0] + dx * scale, pos[1] + dy * scale


def lane_centers(field: dict):
    y = float(field["y_min"]) + 0.5 * float(field["lane_width"])
    centers = []
    while y <= float(field["y_max"]) + 1e-6:
        centers.append(y)
        y += float(field["lane_width"])
    return centers


def simulate(cfg: dict):
    field = cfg["field"]
    sar = cfg["sar"]
    mag = cfg["magnetometer"]
    decision = cfg["decision"]
    tractor = cfg["tractor"]
    mission = cfg["mission"]
    battery = cfg["battery"]
    sar_battery = battery["sar"]
    mag_battery = battery["magnetometer"]
    rng = np.random.default_rng(int(cfg.get("seed", 1)) + 1000)

    mines = generate_mines(cfg)
    coverage = make_grid(field)
    sar_paths = [[], []]
    tractor_path = []
    mag_paths = []
    pending = []
    mag_events = []

    t = 0.0
    dt = float(mission["dt"])
    reserve_percent = float(battery.get("reserve_percent", 0.0))
    sar_energy_Wh = 0.0
    mag_energy_Wh = 0.0
    max_time = float(mission["max_time_s"])
    mag_available_t = 0.0
    mag_pos = (float(field["x_min"]), float(field["y_min"]))

    sar_radius = 0.5 * float(sar["swath_m"])
    sar_sep = float(sar["swath_m"]) * (1.0 - float(sar["transverse_overlap"]))
    corridor_half = 0.5 * float(tractor["width_m"]) + float(tractor["safety_margin_m"])
    safe_distance = float(tractor["safe_distance_m"])

    tractor_integral = 0.0
    tractor_prev_error = 0.0
    mission_complete = True
    stop_reason = "completed"

    for lane_idx, lane_y in enumerate(lane_centers(field)):
        direction = 1 if lane_idx % 2 == 0 else -1
        x_start = float(field["x_min"]) if direction == 1 else float(field["x_max"])
        x_end = float(field["x_max"]) if direction == 1 else float(field["x_min"])
        sar_x = x_start
        tractor_x = x_start
        lane_done = False
        tractor_done = False

        while not (lane_done and tractor_done):
            if t >= max_time:
                mission_complete = False
                stop_reason = "mission_time_limit"
                return build_result(
                    cfg, mines, coverage, sar_paths, tractor_path, mag_paths, t, stop_reason, mission_complete,
                    sar_energy_Wh, mag_energy_Wh
                )

            if battery_remaining_percent(sar_energy_Wh, sar_battery) <= reserve_percent:
                mission_complete = False
                stop_reason = "sar_battery_reserve_reached"
                return build_result(
                    cfg, mines, coverage, sar_paths, tractor_path, mag_paths, t, stop_reason, mission_complete,
                    sar_energy_Wh, mag_energy_Wh
                )

            if battery_remaining_percent(mag_energy_Wh, mag_battery) <= reserve_percent:
                mission_complete = False
                stop_reason = "magnetometer_battery_reserve_reached"
                return build_result(
                    cfg, mines, coverage, sar_paths, tractor_path, mag_paths, t, stop_reason, mission_complete,
                    sar_energy_Wh, mag_energy_Wh
                )

            while mag_events and mag_events[0][0] <= t:
                _, idx = heapq.heappop(mag_events)
                mine = mines[idx]
                mine.verifying = False
                mine.verified = True
                mine.found = mine.final_prob >= float(decision["confirmed_mine_threshold"])
                mine.blocked = False

            if mag_available_t <= t:
                mag_available_t, mag_pos, used, energy_Wh = assign_mag_task(
                    t, pending, mines, mag, decision, mag_pos, mag_events, mag_paths, mag_battery
                )
                mag_energy_Wh += energy_Wh

            if not lane_done:
                sar_energy_Wh = consume_energy(
                    sar_energy_Wh,
                    sar_battery,
                    float(sar["speed_mps"]),
                    dt,
                    payload_active=True,
                )
                sar_x += direction * float(sar["speed_mps"]) * dt
                if direction == 1:
                    sar_x = min(sar_x, x_end)
                    lane_done = sar_x >= x_end
                else:
                    sar_x = max(sar_x, x_end)
                    lane_done = sar_x <= x_end

                drone_positions = [
                    (sar_x, lane_y - 0.5 * sar_sep),
                    (sar_x, lane_y + 0.5 * sar_sep),
                ]
                for i, pos in enumerate(drone_positions):
                    sar_paths[i].append(pos)
                    mark_disc(coverage, field, pos[0], pos[1], sar_radius)

                for mine in mines:
                    if mine.sar_seen:
                        continue
                    if mine_in_sar_footprint(mine, drone_positions, sar_radius):
                        mine.sar_seen = True
                        mine.sar_prob = detection_probability(mine, sar["detection_model"])
                        if mine.sar_prob >= float(decision["sar_suspicious_min"]):
                            mine.suspicious = True
                            loc_std = float(sar.get("localization_std_m", 0.0))
                            mine.suspect_x = mine.x + float(rng.normal(0.0, loc_std))
                            mine.suspect_y = mine.y + float(rng.normal(0.0, loc_std))
                            mine.suspect_x = max(float(field["x_min"]), min(float(field["x_max"]), mine.suspect_x))
                            mine.suspect_y = max(float(field["y_min"]), min(float(field["y_max"]), mine.suspect_y))
                            enqueue_suspicion(pending, mine)

            cleared_frontier = sar_x - direction * safe_distance
            if direction == 1:
                desired_x = max(x_start, min(cleared_frontier, x_end))
                tractor_done = tractor_x >= x_end
            else:
                desired_x = min(x_start, max(cleared_frontier, x_end))
                tractor_done = tractor_x <= x_end

            blocked = unresolved_blocks(mines, lane_y, tractor_x, sar_x, direction, corridor_half)
            if blocked:
                target_speed = 0.0
            else:
                error = direction * (desired_x - tractor_x)
                tractor_integral += max(-50.0, min(50.0, error)) * dt
                derivative = (error - tractor_prev_error) / max(dt, 1e-6)
                tractor_prev_error = error
                pid = tractor["pid"]
                target_speed = (
                    float(pid["kp"]) * error
                    + float(pid["ki"]) * tractor_integral
                    + float(pid.get("kd", 0.0)) * derivative
                )
                target_speed = max(0.0, min(float(tractor["max_speed_mps"]), target_speed))

            tractor_x += direction * target_speed * dt
            if direction == 1:
                tractor_x = min(tractor_x, x_end)
            else:
                tractor_x = max(tractor_x, x_end)
            tractor_path.append((tractor_x, lane_y))

            if mag_available_t <= t and not pending and not mag_events:
                idle_x = tractor_x + direction * safe_distance
                idle_x = max(float(field["x_min"]), min(float(field["x_max"]), idle_x))
                idle_target = (idle_x, lane_y)
                next_mag_pos = move_toward(
                    mag_pos,
                    idle_target,
                    float(mag["transit_speed_mps"]) * dt,
                )
                if math.hypot(next_mag_pos[0] - mag_pos[0], next_mag_pos[1] - mag_pos[1]) > 1e-6:
                    mag_paths.append((mag_pos, next_mag_pos, "idle"))
                    idle_dist = math.hypot(next_mag_pos[0] - mag_pos[0], next_mag_pos[1] - mag_pos[1])
                    idle_speed = idle_dist / max(dt, 1e-6)
                    mag_energy_Wh = consume_energy(
                        mag_energy_Wh,
                        mag_battery,
                        idle_speed,
                        dt,
                        payload_active=False,
                    )
                    mag_pos = next_mag_pos

            t += dt

    while mag_events:
        done_t, idx = heapq.heappop(mag_events)
        t = max(t, done_t)
        mine = mines[idx]
        mine.verifying = False
        mine.verified = True
        mine.found = mine.final_prob >= float(decision["confirmed_mine_threshold"])
        mine.blocked = False

    return build_result(
        cfg, mines, coverage, sar_paths, tractor_path, mag_paths, t, stop_reason, mission_complete,
        sar_energy_Wh, mag_energy_Wh
    )


def build_result(
    cfg,
    mines,
    coverage,
    sar_paths,
    tractor_path,
    mag_paths,
    elapsed_s,
    stop_reason,
    mission_complete,
    sar_energy_Wh=0.0,
    mag_energy_Wh=0.0,
):
    found = [m for m in mines if m.found]
    missed = [m for m in mines if not m.found]
    scanned_cells = count_scanned_field_cells(coverage, cfg["field"])
    scanned_area = scanned_cells * float(cfg["field"]["grid_res"]) ** 2
    battery = cfg["battery"]
    sar_remaining = battery_remaining_percent(sar_energy_Wh, battery["sar"])
    mag_remaining = battery_remaining_percent(mag_energy_Wh, battery["magnetometer"])
    metrics = {
        "mission_complete": mission_complete,
        "stop_reason": stop_reason,
        "elapsed_s": elapsed_s,
        "sar_battery_remaining_percent": sar_remaining,
        "mag_battery_remaining_percent": mag_remaining,
        "sar_energy_used_Wh": sar_energy_Wh,
        "mag_energy_used_Wh": mag_energy_Wh,
        "battery_reserve_percent": float(battery.get("reserve_percent", 0.0)),
        "scanned_area_m2": scanned_area,
        "mine_count": len(mines),
        "found_mines": len(found),
        "missed_mines": len(missed),
        "sar_suspicious": sum(1 for m in mines if m.suspicious),
        "magnetometer_verifications": sum(1 for m in mines if m.verified),
    }
    return {
        "cfg": cfg,
        "mines": mines,
        "coverage": coverage,
        "sar_paths": sar_paths,
        "tractor_path": tractor_path,
        "mag_paths": mag_paths,
        "metrics": metrics,
    }


def count_scanned_field_cells(coverage, field: dict):
    res = float(field["grid_res"])
    x0, _, y0, _ = coverage_bounds(field)
    ix0 = max(0, int(math.floor((float(field["x_min"]) - x0) / res)))
    ix1 = min(coverage.shape[1], int(math.ceil((float(field["x_max"]) - x0) / res)))
    iy0 = max(0, int(math.floor((float(field["y_min"]) - y0) / res)))
    iy1 = min(coverage.shape[0], int(math.ceil((float(field["y_max"]) - y0) / res)))
    return int(np.count_nonzero(coverage[iy0:iy1, ix0:ix1] > 0.0))


def save_csv(path, mines):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "idx",
            "x",
            "y",
            "material",
            "shape",
            "depth_m",
            "size_m",
            "sar_prob",
            "mag_prob",
            "final_prob",
            "suspect_x",
            "suspect_y",
            "mag_closest_m",
            "suspicious",
            "verified",
            "found",
        ])
        for m in mines:
            writer.writerow([
                m.idx,
                f"{m.x:.3f}",
                f"{m.y:.3f}",
                m.material,
                m.shape,
                f"{m.depth_m:.3f}",
                f"{m.size_m:.3f}",
                "" if math.isnan(m.sar_prob) else f"{m.sar_prob:.4f}",
                "" if math.isnan(m.mag_prob) else f"{m.mag_prob:.4f}",
                "" if math.isnan(m.final_prob) else f"{m.final_prob:.4f}",
                "" if math.isnan(m.suspect_x) else f"{m.suspect_x:.3f}",
                "" if math.isnan(m.suspect_y) else f"{m.suspect_y:.3f}",
                "" if math.isnan(m.mag_closest_m) else f"{m.mag_closest_m:.3f}",
                int(m.suspicious),
                int(m.verified),
                int(m.found),
            ])


def mine_to_dict(mine: Mine):
    def clean(value):
        if isinstance(value, float) and math.isnan(value):
            return None
        return value

    return {
        "idx": mine.idx,
        "x": mine.x,
        "y": mine.y,
        "material": mine.material,
        "shape": mine.shape,
        "depth_m": mine.depth_m,
        "size_m": mine.size_m,
        "sar_prob": clean(mine.sar_prob),
        "mag_prob": clean(mine.mag_prob),
        "final_prob": clean(mine.final_prob),
        "suspect_x": clean(mine.suspect_x),
        "suspect_y": clean(mine.suspect_y),
        "mag_closest_m": clean(mine.mag_closest_m),
        "sar_seen": mine.sar_seen,
        "suspicious": mine.suspicious,
        "verified": mine.verified,
        "found": mine.found,
        "blocked": mine.blocked,
    }


def save_mines_json(path, mines):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump([mine_to_dict(mine) for mine in mines], handle, indent=2, sort_keys=True)


def field_bounds_with_margin(field: dict):
    x_min = float(field["x_min"])
    x_max = float(field["x_max"])
    y_min = float(field["y_min"])
    y_max = float(field["y_max"])
    margin = float(field.get("terrain_margin_m", max(5.0, 0.08 * max(x_max - x_min, y_max - y_min))))
    return x_min, x_max, y_min, y_max, margin


def setup_field_axes(ax, field: dict):
    x_min, x_max, y_min, y_max, margin = field_bounds_with_margin(field)
    ax.set_facecolor("#d8d8d8")
    ax.add_patch(
        Rectangle(
            (x_min, y_min),
            x_max - x_min,
            y_max - y_min,
            facecolor="none",
            edgecolor="black",
            linewidth=1.2,
            zorder=5,
        )
    )
    lane_width = float(field["lane_width"])
    y = y_min + lane_width
    while y < y_max - 1e-9:
        ax.axhline(y, color="black", linewidth=0.5, linestyle=":", alpha=0.35, zorder=4)
        y += lane_width
    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")


def plot_paths(result, ax):
    for idx, path in enumerate(result["sar_paths"], start=1):
        if path:
            arr = np.array(path)
            color = "cyan" if idx == 1 else "deepskyblue"
            ax.plot(arr[:, 0], arr[:, 1], color=color, linewidth=1.3, zorder=8)

    if result["tractor_path"]:
        tractor = np.array(result["tractor_path"])
        ax.plot(tractor[:, 0], tractor[:, 1], color="white", linewidth=2.4, zorder=9)

    for start, end, mode in result["mag_paths"]:
        if mode == "scan":
            color = "magenta"
            linewidth = 1.6
        elif mode == "idle":
            color = "orchid"
            linewidth = 1.0
        else:
            color = "violet"
            linewidth = 1.0
        ax.plot([start[0], end[0]], [start[1], end[1]], color=color, linewidth=linewidth, alpha=0.8, zorder=7)


def plot_mines(result, ax):
    found = np.array([(m.x, m.y) for m in result["mines"] if m.found], dtype=np.float64)
    missed = np.array([(m.x, m.y) for m in result["mines"] if not m.found], dtype=np.float64)
    if len(missed):
        ax.scatter(missed[:, 0], missed[:, 1], marker="x", color="red", s=52, linewidths=1.9, zorder=10)
    if len(found):
        ax.scatter(
            found[:, 0],
            found[:, 1],
            marker="o",
            facecolors="none",
            edgecolors="lime",
            s=70,
            linewidths=2.0,
            zorder=11,
        )


def plot_legend(ax):
    handles = [
        Patch(facecolor="#d8d8d8", edgecolor="none", label="unscanned terrain outside field"),
        Patch(facecolor="#2bb3a3", edgecolor="none", alpha=0.75, label="SAR scanned cells"),
        Line2D([0], [0], color="cyan", linewidth=1.5, label="SAR drone 1 path"),
        Line2D([0], [0], color="deepskyblue", linewidth=1.5, label="SAR drone 2 path"),
        Line2D([0], [0], color="white", linewidth=2.4, label="tractor path"),
        Line2D([0], [0], color="violet", linewidth=1.4, label="magnetometer transit"),
        Line2D([0], [0], color="magenta", linewidth=1.8, label="magnetometer verification scan"),
        Line2D([0], [0], marker="o", color="lime", markerfacecolor="none", linestyle="None", label="found mine"),
        Line2D([0], [0], marker="x", color="red", linestyle="None", label="not found mine"),
    ]
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=True)


def plot_result(result, out_path):
    cfg = result["cfg"]
    field = cfg["field"]
    cov = result["coverage"]
    cov_x_min, cov_x_max, cov_y_min, cov_y_max = coverage_bounds(field)

    fig, ax = plt.subplots(figsize=(15, 6.5))
    setup_field_axes(ax, field)
    img = ax.imshow(
        cov,
        origin="lower",
        extent=[cov_x_min, cov_x_max, cov_y_min, cov_y_max],
        aspect="equal",
        alpha=0.75,
        vmin=0.0,
        vmax=1.0,
        zorder=1,
    )
    fig.colorbar(img, ax=ax, label="SAR scanned", orientation="horizontal", pad=0.12, fraction=0.055)

    plot_paths(result, ax)
    plot_mines(result, ax)
    plot_legend(ax)

    ax.set_title("Mine scanning simulation overview")
    fig.subplots_adjust(right=0.72, bottom=0.20)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_paths_only(result, out_path):
    fig, ax = plt.subplots(figsize=(13, 5.5))
    setup_field_axes(ax, result["cfg"]["field"])
    plot_paths(result, ax)
    plot_legend(ax)
    ax.set_title("Vehicle and UAV paths")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_detection_probabilities(result, out_path):
    mines = sorted(result["mines"], key=lambda m: m.idx)
    idx = np.array([m.idx for m in mines])
    sar_probs = np.array([0.0 if math.isnan(m.sar_prob) else m.sar_prob for m in mines])
    mag_probs = np.array([0.0 if math.isnan(m.mag_prob) else m.mag_prob for m in mines])
    final_probs = np.array([0.0 if math.isnan(m.final_prob) else m.final_prob for m in mines])
    found = np.array([m.found for m in mines])

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.bar(idx - 0.25, sar_probs, width=0.25, color="deepskyblue", label="SAR probability")
    ax.bar(idx, mag_probs, width=0.25, color="magenta", label="magnetometer probability")
    ax.bar(idx + 0.25, final_probs, width=0.25, color="limegreen", label="combined probability")
    ax.scatter(idx[~found], final_probs[~found] + 0.03, marker="x", color="red", label="not found")
    ax.scatter(idx[found], final_probs[found] + 0.03, marker="o", facecolors="none", edgecolors="black", label="found")
    for mine, final_prob in zip(mines, final_probs):
        label = "M" if mine.material == "metal" else "P"
        ax.text(
            mine.idx,
            min(1.06, final_prob + 0.08),
            label,
            ha="center",
            va="bottom",
            fontsize=8,
            fontweight="bold",
        )
    decision = result["cfg"]["decision"]
    ax.axhline(float(decision["sar_suspicious_min"]), color="orange", linestyle="--", linewidth=1.2, label="SAR suspicious min")
    ax.axhline(float(decision["confirmed_mine_threshold"]), color="green", linestyle="--", linewidth=1.2, label="confirmed threshold")
    ax.set_ylim(0.0, 1.08)
    ax.set_xlabel("mine id")
    ax.set_ylabel("probability")
    ax.set_title("Mine detection probabilities (M=metal, P=plastic)")
    ax.legend(loc="upper right")
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def write_plot_notes(path):
    text = """Plot explanations

scan_result.png:
- Grey background: terrain outside the simulated field. It is intentionally shown so scanned vs. unscanned space is visible.
- Black rectangle: field boundary.
- Dotted horizontal lines: lane boundaries for the lawnmower pattern.
- Heatmap: SAR scanned cells. Dark/purple means not scanned; green/teal means scanned.
- Cyan and blue lines: paths of the two SAR drones.
- White line: tractor path.
- Violet lines: magnetometer drone transit between tasks or idle movement ahead of the tractor.
- Magenta lines: local magnetometer verification scan over suspicious SAR detections.
- Green open circles: mines found after verification.
- Red crosses: mines not found by the current probability thresholds and sensor model.

paths_only.png:
- Same paths as scan_result.png, without the SAR heatmap and mine markers.

detection_probabilities.png:
- Per-mine SAR, magnetometer, and combined probabilities.
- Orange dashed line: SAR suspicious threshold. Anything above this blocks the tractor until verified.
- Green dashed line: final confirmation threshold.
"""
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)


def write_outputs(result, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "config.json"), "w", encoding="utf-8") as handle:
        json.dump(result["cfg"], handle, indent=2, sort_keys=True)
    with open(os.path.join(out_dir, "metrics.json"), "w", encoding="utf-8") as handle:
        json.dump(result["metrics"], handle, indent=2, sort_keys=True)
    np.save(os.path.join(out_dir, "sar_coverage.npy"), result["coverage"])
    np.savetxt(os.path.join(out_dir, "sar_coverage.csv"), result["coverage"], delimiter=",", fmt="%.0f")
    save_csv(os.path.join(out_dir, "mines.csv"), result["mines"])
    save_mines_json(os.path.join(out_dir, "mines.json"), result["mines"])
    plot_result(result, os.path.join(out_dir, "scan_result.png"))
    plot_paths_only(result, os.path.join(out_dir, "paths_only.png"))
    plot_detection_probabilities(result, os.path.join(out_dir, "detection_probabilities.png"))
    write_plot_notes(os.path.join(out_dir, "plot_notes.txt"))


def main():
    parser = argparse.ArgumentParser(description="Run the config-driven UAV mine scanning simulation.")
    parser.add_argument("--config", default="configs/scan_simulation.json")
    parser.add_argument("--out-dir", default="")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as handle:
        cfg = json.load(handle)

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_root = args.out_dir or cfg.get("out_dir", "data/scan_runs")
    out_dir = os.path.join(out_root, timestamp)

    result = simulate(cfg)
    write_outputs(result, out_dir)

    print(f"Saved scan simulation to {out_dir}")
    print(json.dumps(result["metrics"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
