import numpy as np
from dataclasses import dataclass


@dataclass
class CoverageParams:
    x_min: float = -200.0
    x_max: float = 200.0
    y_min: float = -200.0
    y_max: float = 200.0
    res: float = 0.1

    fov_deg: float = 60.0
    r_max: float = 120.0


class CoverageGrid:
    def __init__(self, p: CoverageParams):
        self.p = p
        self.nx = int(np.ceil((p.x_max - p.x_min) / p.res))
        self.ny = int(np.ceil((p.y_max - p.y_min) / p.res))
        self.coverage = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.fov_half_rad = np.deg2rad(p.fov_deg * 0.5)

    def apply(self, x: float, y: float, z: float, dt: float):
        r_fov = max(0.0, z * np.tan(self.fov_half_rad))

        if self.p.r_max > z:
            r_geom = np.sqrt(max(0.0, self.p.r_max**2 - z**2))
            r = min(r_fov, r_geom)
        else:
            r = 0.0

        if r <= 0.0 or dt <= 0.0:
            return

        x0, x1 = x - r, x + r
        y0, y1 = y - r, y + r

        ix0 = int(np.floor((x0 - self.p.x_min) / self.p.res))
        ix1 = int(np.ceil((x1 - self.p.x_min) / self.p.res))
        iy0 = int(np.floor((y0 - self.p.y_min) / self.p.res))
        iy1 = int(np.ceil((y1 - self.p.y_min) / self.p.res))

        ix0 = max(0, ix0)
        iy0 = max(0, iy0)
        ix1 = min(self.nx - 1, ix1)
        iy1 = min(self.ny - 1, iy1)
        if ix0 > ix1 or iy0 > iy1:
            return

        rr = r * r
        for iy in range(iy0, iy1 + 1):
            cy = self.p.y_min + (iy + 0.5) * self.p.res
            dy = cy - y
            dy2 = dy * dy
            for ix in range(ix0, ix1 + 1):
                cx = self.p.x_min + (ix + 0.5) * self.p.res
                dx = cx - x
                if dx * dx + dy2 <= rr:
                    self.coverage[iy, ix] += dt
