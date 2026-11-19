"""Lanes, screening planners (lane_abreast, corridor) and coverage grid (SPEC 6.1)."""
import math
from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------- lanes / grid (v1 copies)

def lane_centers(fld: dict):
    y = float(fld["y_min"]) + 0.5 * float(fld["lane_width"])
    centers = []
    while y <= float(fld["y_max"]) + 1e-6:
        centers.append(y)
        y += float(fld["lane_width"])
    return centers


def coverage_bounds(fld: dict):
    margin = float(fld.get("terrain_margin_m", 0.0))
    return (
        float(fld["x_min"]) - margin,
        float(fld["x_max"]) + margin,
        float(fld["y_min"]) - margin,
        float(fld["y_max"]) + margin,
    )


def make_grid(fld: dict):
    x_min, x_max, y_min, y_max = coverage_bounds(fld)
    res = float(fld["grid_res"])
    nx = int(math.ceil((x_max - x_min) / res))
    ny = int(math.ceil((y_max - y_min) / res))
    return np.zeros((ny, nx), dtype=np.float32)


def mark_disc(grid, fld, x: float, y: float, radius: float):
    res = float(fld["grid_res"])
    x_min, _, y_min, _ = coverage_bounds(fld)
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


def mark_capsule(grid, fld, p0, p1, radius: float):
    """Mark every cell whose centre is within radius of segment p0-p1."""
    res = float(fld["grid_res"])
    gx0, _, gy0, _ = coverage_bounds(fld)
    xa, xb = sorted((p0[0], p1[0]))
    ya, yb = sorted((p0[1], p1[1]))
    ix0 = max(0, int(math.floor((xa - radius - gx0) / res)))
    ix1 = min(grid.shape[1] - 1, int(math.ceil((xb + radius - gx0) / res)))
    iy0 = max(0, int(math.floor((ya - radius - gy0) / res)))
    iy1 = min(grid.shape[0] - 1, int(math.ceil((yb + radius - gy0) / res)))
    if ix1 < ix0 or iy1 < iy0:
        return
    cx = gx0 + (np.arange(ix0, ix1 + 1) + 0.5) * res
    cy = gy0 + (np.arange(iy0, iy1 + 1) + 0.5) * res
    CX, CY = np.meshgrid(cx, cy)
    d = seg_dist_array(CX, CY, p0, p1)
    sub = grid[iy0:iy1 + 1, ix0:ix1 + 1]
    sub[d <= radius + 1e-9] = 1.0


def count_scanned_field_cells(coverage, fld: dict):
    res = float(fld["grid_res"])
    x0, _, y0, _ = coverage_bounds(fld)
    ix0 = max(0, int(math.floor((float(fld["x_min"]) - x0) / res)))
    ix1 = min(coverage.shape[1], int(math.ceil((float(fld["x_max"]) - x0) / res)))
    iy0 = max(0, int(math.floor((float(fld["y_min"]) - y0) / res)))
    iy1 = min(coverage.shape[0], int(math.ceil((float(fld["y_max"]) - y0) / res)))
    return int(np.count_nonzero(coverage[iy0:iy1, ix0:ix1] > 0.0))


def seg_dist_array(px, py, p0, p1):
    sx, sy = p0
    vx, vy = p1[0] - sx, p1[1] - sy
    l2 = vx * vx + vy * vy
    if l2 <= 1e-12:
        return np.hypot(px - sx, py - sy)
    u = np.clip(((px - sx) * vx + (py - sy) * vy) / l2, 0.0, 1.0)
    return np.hypot(px - (sx + u * vx), py - (sy + u * vy))


# --------------------------------------------------------------------------- local scans (v1 copies)

def local_lawnmower_segments(cx: float, cy: float, scan_cfg: dict):
    """v1 local scan (legacy_logistic mode only)."""
    area = float(scan_cfg.get("local_scan_area_m", scan_cfg.get("line_length_m", 6.0)))
    line_len = float(scan_cfg.get("line_length_m", area))
    swath = float(scan_cfg["swath_m"])
    overlap = float(scan_cfg.get("transverse_overlap", 0.5))
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


def local_scan_side(geom: dict, sigma: float, size_factor: float = 1.0) -> float:
    """Rate-based local scan side (SPEC 4.3): L = max(min_scan_side_m, 2*2.5*sigma + swath_m).

    scan_size factors (Paper 2 hook) scale L.
    """
    swath = float(geom["swath_m"])
    side = max(float(geom["min_scan_side_m"]), 2.0 * 2.5 * sigma + swath)
    return side * size_factor


def local_square_segments(cx: float, cy: float, geom: dict, side: float):
    """Square boustrophedon of the given side centred on (cx, cy) (rate-based mode).

    Lines run along y over the full side; their x positions go from -side/2 + swath/2 to
    +side/2 - swath/2 at spacing swath*(1 - overlap) (last spacing shortened), so the swath
    covers the square exactly. n_lines = ceil((side - swath)/spacing) + 1 and the path length
    (first line start to last line end) is n_lines*side + (side - swath).
    """
    swath = float(geom["swath_m"])
    spacing = max(1e-3, swath * (1.0 - float(geom.get("transverse_overlap", 0.0))))
    span = max(0.0, side - swath)
    n = int(math.ceil(span / spacing - 1e-9)) + 1
    half = 0.5 * side
    segments = []
    for j in range(n):
        dx = -0.5 * span + min(j * spacing, span)
        x_line = cx + dx
        if j % 2 == 0:
            segments.append(((x_line, cy - half), (x_line, cy + half)))
        else:
            segments.append(((x_line, cy + half), (x_line, cy - half)))
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


def move_toward(pos, target, max_dist: float):
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dist = math.hypot(dx, dy)
    if dist <= max_dist or dist <= 1e-9:
        return target
    scale = max_dist / dist
    return pos[0] + dx * scale, pos[1] + dy * scale


# --------------------------------------------------------------------------- planners

def formation_offsets(n: int, sep: float):
    return [(i - (n - 1) / 2) * sep for i in range(n)]


def corridor_geometry(n: int, swath: float, overlap: float, width: float):
    """Returns sep, C (formation cover), k (lateral passes; 1 when C >= W)."""
    sep = swath * (1.0 - overlap)
    cover = (n - 1) * sep + swath
    k = 1 if cover >= width - 1e-9 else int(math.ceil(width / cover - 1e-9))
    return sep, cover, k


@dataclass
class StepResult:
    positions: list          # end-of-step position per screener
    scan_segments: list      # per screener: list of (p0, p1) flown with payload on, in scan mode
    energy_legs: list        # list of (speed_mps, payload_active, duration_s) for each screener
    frontier_x: float
    done: bool


class LaneAbreastPlanner:
    """Continuous formation along the lane (v1 geometry, arithmetic kept identical)."""

    def __init__(self, lane_y, direction, x_start, x_end, n, sep, speed, center_offset=0.0):
        self.lane_y = lane_y
        self.direction = direction
        self.x_start = x_start
        self.x_end = x_end
        self.speed = speed
        self.offsets = formation_offsets(n, sep)
        self.center_y = lane_y + center_offset if center_offset else lane_y
        self.x = x_start
        self.done = False
        self.n = n

    @property
    def frontier_x(self):
        return self.x

    def positions(self):
        return [(self.x, self.center_y + off) for off in self.offsets]

    def step(self, dt):
        prev = self.positions()
        self.x += self.direction * self.speed * dt
        if self.direction == 1:
            self.x = min(self.x, self.x_end)
            self.done = self.x >= self.x_end
        else:
            self.x = max(self.x, self.x_end)
            self.done = self.x <= self.x_end
        cur = self.positions()
        segs = [[(p, c)] for p, c in zip(prev, cur)]
        return StepResult(cur, segs, [(self.speed, True, dt)], self.x, self.done)


@dataclass
class Leg:
    p0: tuple
    p1: tuple
    duration: float
    speed: float
    payload: bool
    kind: str               # scan / turn / transit
    completes_x: float = math.nan


class CorridorPlanner:
    """Corridor screening in segments with k lateral passes (SPEC 6.1).

    Per segment: k x (pass of segment_length at v, turn of turn_time). For even k an extra
    transit leg of segment_length (payload off) returns the formation to the next segment.
    The last segment of a lane has no trailing turn/transit.
    """

    def __init__(self, lane_y, direction, x_start, x_end, n, sep, swath, speed, width,
                 segment_length, turn_time, transit_speed=None):
        self.lane_y = lane_y
        self.direction = direction
        self.x_start = x_start
        self.x_end = x_end
        self.offsets = formation_offsets(n, sep)
        self.offset_vecs = [(0.0, off) for off in self.offsets]
        self.n = n
        cover = (n - 1) * sep + swath
        self.k = int(math.ceil(width / cover - 1e-9))
        lo = lane_y - 0.5 * width + 0.5 * cover
        hi = lane_y + 0.5 * width - 0.5 * cover
        ys = [min(lo + p * cover, hi) for p in range(self.k)]
        transit_speed = transit_speed or speed
        total = abs(x_end - x_start)
        nseg = max(1, int(math.ceil(total / segment_length - 1e-9)))
        legs = []
        for j in range(nseg):
            x0 = x_start + direction * j * segment_length
            x1 = x_start + direction * min((j + 1) * segment_length, total)
            seg_len = abs(x1 - x0)
            order = ys if j % 2 == 0 else list(reversed(ys))
            last_seg = j == nseg - 1
            for p in range(self.k):
                xa, xb = (x0, x1) if p % 2 == 0 else (x1, x0)
                legs.append(Leg((xa, order[p]), (xb, order[p]), seg_len / speed, speed, True, "scan",
                                x1 if p == self.k - 1 else math.nan))
                if p < self.k - 1:
                    dy = abs(order[p + 1] - order[p])
                    legs.append(Leg((xb, order[p]), (xb, order[p + 1]), turn_time,
                                    dy / turn_time if turn_time > 0 else 0.0, True, "turn"))
                elif not last_seg:
                    legs.append(Leg((xb, order[p]), (xb, order[p]), turn_time, 0.0, True, "turn"))
            if self.k % 2 == 0 and not last_seg:
                y_end = order[-1]
                legs.append(Leg((x0, y_end), (x1, y_end), seg_len / transit_speed, transit_speed, False, "transit"))
        self.legs = [leg for leg in legs if leg.duration > 0.0]
        self.leg_i = 0
        self.leg_t = 0.0
        self.center = self.legs[0].p0 if self.legs else (x_start, lane_y)
        self.frontier = x_start
        self.done = not self.legs

    @property
    def frontier_x(self):
        return self.frontier

    def positions(self):
        return [(self.center[0] + ox, self.center[1] + oy) for ox, oy in self.offset_vecs]

    @staticmethod
    def _lerp(leg, t):
        f = 0.0 if leg.duration <= 0 else min(1.0, t / leg.duration)
        return (leg.p0[0] + f * (leg.p1[0] - leg.p0[0]), leg.p0[1] + f * (leg.p1[1] - leg.p0[1]))

    def step(self, dt):
        remaining = dt
        center_segs = []
        energy = []
        while remaining > 1e-12 and self.leg_i < len(self.legs):
            leg = self.legs[self.leg_i]
            take = min(remaining, leg.duration - self.leg_t)
            a = self._lerp(leg, self.leg_t)
            self.leg_t += take
            remaining -= take
            b = self._lerp(leg, self.leg_t)
            energy.append((leg.speed, leg.payload, take))
            if leg.kind == "scan":
                center_segs.append((a, b))
            self.center = b
            if self.leg_t >= leg.duration - 1e-9:
                if not math.isnan(leg.completes_x):
                    self.frontier = leg.completes_x
                self.leg_i += 1
                self.leg_t = 0.0
        self.done = self.leg_i >= len(self.legs)
        segs = [[((a[0] + ox, a[1] + oy), (b[0] + ox, b[1] + oy)) for a, b in center_segs]
                for ox, oy in self.offset_vecs]
        return StepResult(self.positions(), segs, energy, self.frontier, self.done)


class CrossTrackPlanner(CorridorPlanner):
    """Cross-track screening (SPEC 6.1, v2.1).

    Lines run across the corridor (y from lane_y - W/2 to lane_y + W/2, then back). The N
    screeners fly the same line side by side, offset along x by sep = swath*(1 - overlap).
    After each line, a turn of turn_time moves the block forward by N*sep. The frontier is the
    x up to which the ground is screened, i.e. the leading boundary (block start + N*sep) of the
    last completed block, clipped to the lane end. Long-run frontier speed: N*sep / (W/v + turn_time).
    """

    def __init__(self, lane_y, direction, x_start, x_end, n, sep, swath, speed, width, turn_time):
        self.lane_y = lane_y
        self.direction = direction
        self.x_start = x_start
        self.x_end = x_end
        self.n = n
        self.k = 1
        self.offsets = formation_offsets(n, sep)
        self.offset_vecs = [(direction * off, 0.0) for off in self.offsets]
        adv = n * sep
        total = abs(x_end - x_start)
        nblk = max(1, int(math.ceil(total / adv - 1e-9)))
        y_lo, y_hi = lane_y - 0.5 * width, lane_y + 0.5 * width
        centre_off = 0.5 * swath + 0.5 * (n - 1) * sep
        legs = []
        for b in range(nblk):
            xb = x_start + direction * b * adv
            xc = xb + direction * centre_off
            ya, yb = (y_lo, y_hi) if b % 2 == 0 else (y_hi, y_lo)
            done_x = x_start + direction * min((b + 1) * adv, total)
            legs.append(Leg((xc, ya), (xc, yb), width / speed, speed, True, "scan", done_x))
            if b < nblk - 1:
                legs.append(Leg((xc, yb), (xc + direction * adv, yb), turn_time,
                                adv / turn_time if turn_time > 0 else 0.0, True, "turn"))
        self.legs = [leg for leg in legs if leg.duration > 0.0]
        self.leg_i = 0
        self.leg_t = 0.0
        self.center = self.legs[0].p0 if self.legs else (x_start, lane_y)
        self.frontier = x_start
        self.done = not self.legs


def corridor_width(cfg):
    tr = cfg["tractor"]
    return float(tr["width_m"]) + 2.0 * float(tr["safety_margin_m"])


def make_planner(mode, lane_y, direction, x_start, x_end, n, swath, overlap, speed, width,
                 segment_length=10.0, turn_time=3.0, legacy_sep=None):
    if mode == "lane_abreast":
        sep = legacy_sep if legacy_sep is not None else swath * (1.0 - overlap)
        return LaneAbreastPlanner(lane_y, direction, x_start, x_end, n, sep, speed)
    if mode == "corridor":
        sep, cover, k = corridor_geometry(n, swath, overlap, width)
        if k == 1:
            return LaneAbreastPlanner(lane_y, direction, x_start, x_end, n, sep, speed)
        return CorridorPlanner(lane_y, direction, x_start, x_end, n, sep, swath, speed, width,
                               segment_length, turn_time)
    if mode == "cross_track":
        sep = swath * (1.0 - overlap)
        return CrossTrackPlanner(lane_y, direction, x_start, x_end, n, sep, swath, speed, width, turn_time)
    raise ValueError(f"unknown screening_mode {mode}")


def frontier_speed_formula(n, swath, overlap, speed, width, segment_length, turn_time, mode="corridor"):
    """Long-run frontier speed (exact for the leg models above). Returns (v_f, k)."""
    if mode == "cross_track":
        sep = swath * (1.0 - overlap)
        return n * sep / (width / speed + turn_time), 1
    sep, cover, k = corridor_geometry(n, swath, overlap, width)
    if k == 1:
        return speed, k
    period = k * (segment_length / speed + turn_time)
    if k % 2 == 0:
        period += segment_length / speed
    return segment_length / period, k
