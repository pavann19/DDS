"""
Ego-centric log-odds occupancy grid (Phase 6, P6-4).

100m x 100m grid at 0.25m resolution (400x400 cells), re-centered on the
ego every tick (a "rolling" local grid, not a persistent world map -- this
project has no global SLAM/localization layer, so a world-fixed grid would
imply a capability that doesn't exist here).

Log-odds representation: L(m) = log(P(m=1) / (1 - P(m=1))), updated
additively per Bayes' rule in log-odds form (the standard occupancy-grid
formulation -- Thrun/Burgard/Fox, Probabilistic Robotics). Free-space rays
are cast from the ego cell to each occupied cell with a vectorized DDA line
rasterization (numerically equivalent to Bresenham's algorithm for this
purpose, computed via np.linspace instead of a per-cell loop -- see the
performance note on update()) so cells between the ego and a detected actor
are marked free, not just the actor's own footprint cells.
"""
import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

GRID_EXTENT_M = 100.0
CELL_SIZE_M = 0.25
GRID_CELLS = int(GRID_EXTENT_M / CELL_SIZE_M)  # 400

L_OCCUPIED = 0.85   # log-odds increment for a hit
L_FREE = -0.4        # log-odds increment for a free-space ray cell
L_MIN = -6.0
L_MAX = 6.0

# Fixed sample count for every free-space ray, regardless of its real
# length. This trades a small amount of precision (very short rays get a
# few redundant duplicate cells; np.add.at simply adds L_FREE to the same
# cell more than once, which is harmless -- log-odds are clipped either
# way) for the ability to batch every footprint's ray into ONE array
# operation instead of one np.linspace call per footprint. See update()'s
# performance note.
RAY_SAMPLE_STEPS = 80

# Occupied-box sample templates are cached per (length_m, width_m) --
# entities.py's ENTITY_DIMENSIONS_M has exactly 7 distinct box sizes, so
# this cache never grows past 7 entries regardless of actor count.
_FOOTPRINT_TEMPLATE_CACHE: dict = {}


@dataclass
class OccupiedFootprint:
    x: float
    z: float
    length_m: float
    width_m: float
    heading_rad: float = 0.0


class OccupancyGrid:
    def __init__(self):
        self.log_odds = np.zeros((GRID_CELLS, GRID_CELLS), dtype=float)
        self._occupied_mask_buffer = np.zeros((GRID_CELLS, GRID_CELLS), dtype=bool)
        self.ego_x = 0.0
        self.ego_z = 0.0

    def _world_to_cell(self, x: float, z: float) -> Tuple[int, int]:
        # Ego sits at the grid center cell.
        col = int((x - self.ego_x) / CELL_SIZE_M) + GRID_CELLS // 2
        row = int((z - self.ego_z) / CELL_SIZE_M) + GRID_CELLS // 2
        return row, col

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < GRID_CELLS and 0 <= col < GRID_CELLS

    def reset(self, ego_x: float, ego_z: float) -> None:
        """Re-center the grid on the ego's current position. Called once
        per tick before update() -- a rolling grid has no reason to carry
        occupancy for a footprint that's now off the edge of the new
        window."""
        self.ego_x = ego_x
        self.ego_z = ego_z
        self.log_odds.fill(0.0)

    @staticmethod
    def _footprint_template(length_m: float, width_m: float) -> Tuple[np.ndarray, np.ndarray]:
        """Local (length-axis, width-axis) sample offsets for one box size,
        cached -- see _FOOTPRINT_TEMPLATE_CACHE's module-level note."""
        key = (length_m, width_m)
        cached = _FOOTPRINT_TEMPLATE_CACHE.get(key)
        if cached is not None:
            return cached
        half_l, half_w = length_m / 2.0, width_m / 2.0
        steps_l = max(1, int(length_m / CELL_SIZE_M))
        steps_w = max(1, int(width_m / CELL_SIZE_M))
        local_l = np.linspace(-half_l, half_l, steps_l + 1)
        local_w = np.linspace(-half_w, half_w, steps_w + 1)
        ll, ww = np.meshgrid(local_l, local_w, indexing="ij")
        template = (ll.ravel(), ww.ravel())
        _FOOTPRINT_TEMPLATE_CACHE[key] = template
        return template

    def update(self, footprints: List[OccupiedFootprint]) -> None:
        # Fully vectorized and batched ACROSS footprints, not just within
        # one: every footprint's ray and every footprint's occupied-box
        # cells are built with a small, fixed number of numpy calls total
        # for the whole tick, not once per footprint (let alone once per
        # cell). An earlier version did one np.linspace/meshgrid PER
        # footprint (still ~10 numpy calls x 30 actors = 300 calls/tick,
        # each paying numpy's per-call dispatch overhead) and blew Gate
        # 6.3's 2ms budget by ~5x even after fixing the original per-cell
        # version's 25x overrun. Stacking every footprint's position into
        # one array and broadcasting the ray/box templates across all of
        # them at once is what actually gets this under budget.
        if not footprints:
            return

        ego_row, ego_col = self._world_to_cell(self.ego_x, self.ego_z)

        # --- occupied footprint cells first, grouped by box size so
        # identical classes (the common case -- most NPCs on the road are
        # the same few vehicle types) reuse one template instead of
        # rebuilding it ---
        groups: dict = {}
        for fp in footprints:
            groups.setdefault((fp.length_m, fp.width_m), []).append(fp)

        occ_row_arrays, occ_col_arrays = [], []
        for (length_m, width_m), group in groups.items():
            ll, ww = self._footprint_template(length_m, width_m)
            xs = np.array([fp.x for fp in group])
            zs = np.array([fp.z for fp in group])
            cos_h = np.array([math.cos(fp.heading_rad) for fp in group])
            sin_h = np.array([math.sin(fp.heading_rad) for fp in group])

            wx = xs[:, None] + ll[None, :] * cos_h[:, None] - ww[None, :] * sin_h[:, None]
            wz = zs[:, None] + ll[None, :] * sin_h[:, None] + ww[None, :] * cos_h[:, None]
            occ_col_arrays.append((((wx - self.ego_x) / CELL_SIZE_M) + GRID_CELLS // 2).astype(np.intp).ravel())
            occ_row_arrays.append((((wz - self.ego_z) / CELL_SIZE_M) + GRID_CELLS // 2).astype(np.intp).ravel())

        occ_rows = np.concatenate(occ_row_arrays)
        occ_cols = np.concatenate(occ_col_arrays)

        # --- free-space rays for every footprint, batched in one shot ---
        # RAY_SAMPLE_STEPS is fixed regardless of a ray's real length, so a
        # SHORT ray (a nearby actor) samples the same handful of cells many
        # times over -- np.add.at correctly accumulates those duplicates,
        # which would otherwise let a close actor's own occupied cell get
        # free-space-decremented more times than its single occupied hit
        # can outweigh. Fixed by marking a cell's occupied-vs-free status
        # mutually exclusive per tick: any cell that IS one of this tick's
        # occupied cells is dropped from the free-ray set entirely, exactly
        # like a real range sensor's ray stopping at the first return
        # rather than reporting free space past/at a detected obstacle.
        # Same world->cell arithmetic as _world_to_cell, done once as a
        # vectorized pass over every footprint instead of calling
        # _world_to_cell per footprint (and, in an earlier version, twice
        # per footprint -- once for the row array, once for the col array).
        fp_x = np.array([fp.x for fp in footprints])
        fp_z = np.array([fp.z for fp in footprints])
        target_cols = ((fp_x - self.ego_x) / CELL_SIZE_M) + GRID_CELLS // 2
        target_rows = ((fp_z - self.ego_z) / CELL_SIZE_M) + GRID_CELLS // 2
        t = np.linspace(0.0, 1.0, RAY_SAMPLE_STEPS, endpoint=False)
        ray_rows = np.round(ego_row + t[None, :] * (target_rows[:, None] - ego_row)).astype(np.intp).ravel()
        ray_cols = np.round(ego_col + t[None, :] * (target_cols[:, None] - ego_col)).astype(np.intp).ravel()

        # Reuse one preallocated mask buffer across ticks (cleared, not
        # reallocated) -- a fresh 400x400 bool array every tick is a real
        # allocation cost multiplied over the whole simulation's lifetime.
        self._occupied_mask_buffer.fill(False)
        occupied_mask = self._occupied_mask_buffer
        in_bounds_occ = (occ_rows >= 0) & (occ_rows < GRID_CELLS) & (occ_cols >= 0) & (occ_cols < GRID_CELLS)
        valid_occ_rows = occ_rows[in_bounds_occ]
        valid_occ_cols = occ_cols[in_bounds_occ]
        occupied_mask[valid_occ_rows, valid_occ_cols] = True

        in_bounds_ray = (ray_rows >= 0) & (ray_rows < GRID_CELLS) & (ray_cols >= 0) & (ray_cols < GRID_CELLS)
        valid_ray_rows = ray_rows[in_bounds_ray]
        valid_ray_cols = ray_cols[in_bounds_ray]
        free_mask = ~occupied_mask[valid_ray_rows, valid_ray_cols]

        if np.any(free_mask):
            np.add.at(self.log_odds, (valid_ray_rows[free_mask], valid_ray_cols[free_mask]), L_FREE)

        if valid_occ_rows.size > 0:
            np.add.at(self.log_odds, (valid_occ_rows, valid_occ_cols), L_OCCUPIED)

        np.clip(self.log_odds, L_MIN, L_MAX, out=self.log_odds)

    def _scatter_add(self, rows: np.ndarray, cols: np.ndarray, delta: float) -> None:
        if rows.size == 0:
            return
        in_bounds = (rows >= 0) & (rows < GRID_CELLS) & (cols >= 0) & (cols < GRID_CELLS)
        if not np.any(in_bounds):
            return
        np.add.at(self.log_odds, (rows[in_bounds], cols[in_bounds]), delta)

    def probability_at(self, x: float, z: float) -> float:
        row, col = self._world_to_cell(x, z)
        if not self._in_bounds(row, col):
            return 0.5
        l = self.log_odds[row, col]
        return 1.0 / (1.0 + math.exp(-l))

    def is_occupied(self, x: float, z: float, threshold: float = 0.7) -> bool:
        return self.probability_at(x, z) >= threshold
