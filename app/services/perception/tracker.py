"""
Multi-target Kalman tracker with GNN data association (Phase 6, P6-3).

Constant-acceleration (CA) motion model, tracked with a linear Kalman
filter -- the CA model's dynamics and this project's position-only
measurement model are both linear, so a full Extended Kalman Filter
degenerates exactly to a linear KF here (a "special case of EKF", not a
downgrade -- there is no non-linearity in this state/measurement pair to
linearize). If a future phase adds a non-linear measurement (e.g. raw
range+bearing instead of fused x/z), this is the file that would grow an
actual Jacobian.

Data association is Global Nearest Neighbor via the Hungarian algorithm
(scipy.optimize.linear_sum_assignment) over a Euclidean cost matrix, gated
by MAX_ASSOCIATION_DIST_M so a detection far from every track starts a new
one instead of being force-matched to the nearest (wrong) track.
"""
import math
from dataclasses import dataclass, field
from enum import Enum
from functools import lru_cache
from itertools import count
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from app.services.perception.entities import DetectedEntity, EntityClass

MAX_ASSOCIATION_DIST_M = 5.0
TENTATIVE_HITS_TO_CONFIRM = 3
MAX_COAST_TICKS = 5

# Process noise (per-axis, applied identically to x/z via the acceleration
# component). A steady-state KF's estimation error does not shrink to zero
# as more measurements arrive -- it converges to a fixed floor set by the
# ratio of process noise to measurement noise. This value is tuned so that
# floor sits comfortably under Gate 6.2's <0.15m/<0.25m/s targets against
# sigma=0.3m measurement noise, verified in tests/test_perception.py by
# averaging many independent noisy trials (a single realization's steady-
# state error is itself a random variable, so a one-seed assertion would be
# testing luck, not the filter).
PROCESS_NOISE_ACCEL_VAR = 0.0002
MEASUREMENT_NOISE_VAR = 0.3 ** 2  # matches Gate 6.2's specified sigma=0.3m


class TrackStatus(str, Enum):
    TENTATIVE = "TENTATIVE"
    CONFIRMED = "CONFIRMED"
    COASTED = "COASTED"
    DELETED = "DELETED"


# dt is almost always the same value tick-to-tick (0.1s at 10Hz), so caching
# these by dt avoids rebuilding identical 6x6 matrices every tick. Safe only
# because nothing downstream mutates the returned array in place (every use
# is `F @ ...` / `... + Q`, which allocates a new array) -- an in-place
# mutation on a cached array would corrupt it for every other caller sharing
# that dt.
@lru_cache(maxsize=16)
def _ca_transition_matrix(dt: float) -> np.ndarray:
    return np.array([
        [1, 0, dt, 0, 0.5 * dt * dt, 0],
        [0, 1, 0, dt, 0, 0.5 * dt * dt],
        [0, 0, 1, 0, dt, 0],
        [0, 0, 0, 1, 0, dt],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
    ], dtype=float)


@lru_cache(maxsize=16)
def _ca_process_noise(dt: float, accel_var: float = PROCESS_NOISE_ACCEL_VAR) -> np.ndarray:
    """Discretized white-noise-acceleration process noise: perturbs the
    acceleration terms directly, propagated back onto position/velocity via
    the standard piecewise-constant-Wiener-acceleration construction."""
    q = np.array([0.5 * dt * dt, dt, 1.0])  # position/vel/accel loading per axis
    Q_axis = np.outer(q, q) * accel_var
    Q = np.zeros((6, 6))
    # Interleave: [px, pz, vx, vz, ax, az] -> axis x uses rows/cols 0,2,4; axis z uses 1,3,5
    idx_x = [0, 2, 4]
    idx_z = [1, 3, 5]
    for i, ix in enumerate(idx_x):
        for j, jx in enumerate(idx_x):
            Q[ix, jx] = Q_axis[i, j]
    for i, iz in enumerate(idx_z):
        for j, jz in enumerate(idx_z):
            Q[iz, jz] = Q_axis[i, j]
    return Q


_I6 = np.eye(6)
_H = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
], dtype=float)

_R = np.eye(2) * MEASUREMENT_NOISE_VAR


@dataclass
class Track:
    track_id: int
    entity_class: EntityClass
    state: np.ndarray          # 6x1: [px, pz, vx, vz, ax, az]
    covariance: np.ndarray     # 6x6
    status: TrackStatus = TrackStatus.TENTATIVE
    hits: int = 1
    misses: int = 0
    age_ticks: int = 0
    heading_rad: float = 0.0
    source_id: str = ""

    @property
    def x(self) -> float:
        return float(self.state[0])

    @property
    def z(self) -> float:
        return float(self.state[1])

    @property
    def vx(self) -> float:
        return float(self.state[2])

    @property
    def vz(self) -> float:
        return float(self.state[3])

    def predict(self, dt: float) -> None:
        F = _ca_transition_matrix(dt)
        Q = _ca_process_noise(dt)
        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T + Q
        self.age_ticks += 1

    def update(self, measurement_xz: Tuple[float, float]) -> None:
        z = np.array(measurement_xz, dtype=float)
        y = z - _H @ self.state
        S = _H @ self.covariance @ _H.T + _R
        K = self.covariance @ _H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.covariance = (np.eye(6) - K @ _H) @ self.covariance


def _new_track_state(det: DetectedEntity) -> np.ndarray:
    return np.array([det.x, det.z, det.vx, det.vz, 0.0, 0.0], dtype=float)


class MultiTargetTracker:
    """Owns the full set of active tracks across ticks. One instance per
    live simulation session (mirrors PhysicsEngine/TrafficModel's own
    per-session lifetime)."""

    def __init__(self):
        self._id_counter = count(1)
        self.tracks: Dict[int, Track] = {}
        # Set by _batched_predict each tick, consumed by _associate/
        # _batched_update -- see their docstrings on why these are cached
        # rather than rebuilt.
        self._predicted_track_ids: List[int] = []
        self._predicted_positions = np.zeros((0, 2))
        self._predicted_states = np.zeros((0, 6))
        self._predicted_covs = np.zeros((0, 6, 6))

    def step(self, dt: float, detections: List[DetectedEntity]) -> List[Track]:
        self._batched_predict(dt)

        matches, match_rows, unmatched_track_ids, unmatched_det_idxs = self._associate(detections)
        self._batched_update(matches, match_rows, detections)

        # entity_class/heading/source_id are per-detection metadata, not
        # part of the batched Kalman math -- cheap enough (30 plain
        # attribute assignments) to leave as a direct loop.
        for track_id, det_idx in matches:
            track = self.tracks[track_id]
            det = detections[det_idx]
            track.entity_class = det.entity_class
            track.heading_rad = det.heading_rad
            track.source_id = det.source_id

        for track_id in unmatched_track_ids:
            track = self.tracks[track_id]
            track.misses += 1
            if track.status in (TrackStatus.CONFIRMED, TrackStatus.COASTED):
                track.status = TrackStatus.COASTED
                if track.misses > MAX_COAST_TICKS:
                    track.status = TrackStatus.DELETED
            else:  # TENTATIVE track that missed immediately -- drop it, no coast grace period
                track.status = TrackStatus.DELETED

        for det_idx in unmatched_det_idxs:
            det = detections[det_idx]
            track_id = next(self._id_counter)
            self.tracks[track_id] = Track(
                track_id=track_id,
                entity_class=det.entity_class,
                state=_new_track_state(det),
                covariance=np.eye(6) * 1.0,
                heading_rad=det.heading_rad,
                source_id=det.source_id,
            )

        self.tracks = {tid: t for tid, t in self.tracks.items() if t.status != TrackStatus.DELETED}
        return list(self.tracks.values())

    def _batched_predict(self, dt: float) -> None:
        # One F/Q pair applies to every track this tick (dt is shared), so
        # predicting all N tracks is a single stacked (N,6)/(N,6,6) numpy
        # operation via numpy's native batched matmul (the `@` operator
        # batches over any leading dimensions when the trailing two match a
        # matrix multiply), instead of N separate per-track Track.predict()
        # calls. Per-track calls were the dominant cost in Gate 6.3's
        # <2ms/tick @ 30-actor budget -- numpy's per-call dispatch overhead,
        # paid ~8 times per track per tick, added up fast at N=30.
        if not self.tracks:
            self._predicted_track_ids = []
            self._predicted_positions = np.zeros((0, 2))
            self._predicted_states = np.zeros((0, 6))
            self._predicted_covs = np.zeros((0, 6, 6))
            return
        track_ids = list(self.tracks.keys())
        F = _ca_transition_matrix(dt)
        Q = _ca_process_noise(dt)
        states = np.stack([self.tracks[tid].state for tid in track_ids])       # (N, 6)
        covs = np.stack([self.tracks[tid].covariance for tid in track_ids])    # (N, 6, 6)

        new_states = states @ F.T                                              # (N, 6)
        new_covs = F @ covs @ F.T + Q                                          # (N, 6, 6), F/Q broadcast over N

        for i, tid in enumerate(track_ids):
            track = self.tracks[tid]
            track.state = new_states[i]
            track.covariance = new_covs[i]
            track.age_ticks += 1

        # Cached for _associate/_batched_update, which otherwise re-gather
        # these exact same arrays via a second (or third) pass of dict
        # lookups + np.stack -- redundant work paid every tick at N tracks.
        # These are the SAME arrays a track's individual .state/.covariance
        # were just set to above, so slicing them by row index is exactly
        # equivalent to re-stacking from the dict, just without repaying
        # numpy's per-call stack overhead a second/third time.
        self._predicted_track_ids = track_ids
        self._predicted_positions = new_states[:, 0:2]
        self._predicted_states = new_states
        self._predicted_covs = new_covs

    def _batched_update(
        self,
        matches: List[Tuple[int, int]],
        match_rows: np.ndarray,
        detections: List[DetectedEntity],
    ) -> None:
        """Kalman-update every matched track in one batched numpy pass (same
        H/R for all, and numpy's np.linalg.inv natively batches over stacked
        (M,2,2) matrices) -- see _batched_predict's note on why this matters
        for Gate 6.3. match_rows indexes directly into _batched_predict's
        cached (N,6)/(N,6,6) arrays, avoiding a second per-track np.stack."""
        if not matches:
            return
        states = self._predicted_states[match_rows]                            # (M, 6)
        covs = self._predicted_covs[match_rows]                                # (M, 6, 6)
        measurements = np.array([[detections[det_idx].x, detections[det_idx].z] for _, det_idx in matches])  # (M, 2)

        y = measurements - states @ _H.T                                       # (M, 2)
        S = _H @ covs @ _H.T + _R                                              # (M, 2, 2)
        S_inv = np.linalg.inv(S)                                               # (M, 2, 2)
        K = covs @ _H.T @ S_inv                                                # (M, 6, 2)

        new_states = states + np.einsum("mij,mj->mi", K, y)                    # (M, 6)
        new_covs = (_I6 - K @ _H) @ covs                                        # (M, 6, 6)

        for i, (track_id, _) in enumerate(matches):
            track = self.tracks[track_id]
            track.state = new_states[i]
            track.covariance = new_covs[i]
            track.hits += 1
            track.misses = 0
            if track.status == TrackStatus.TENTATIVE and track.hits >= TENTATIVE_HITS_TO_CONFIRM:
                track.status = TrackStatus.CONFIRMED
            elif track.status == TrackStatus.COASTED:
                track.status = TrackStatus.CONFIRMED

    def _associate(
        self, detections: List[DetectedEntity]
    ) -> Tuple[List[Tuple[int, int]], np.ndarray, List[int], List[int]]:
        track_ids = self._predicted_track_ids
        if not track_ids or not detections:
            return [], np.array([], dtype=np.intp), list(track_ids), list(range(len(detections)))

        # Vectorized pairwise-distance cost matrix via broadcasting instead
        # of an N*M Python-level math.hypot loop -- at 30 tracks x 30
        # detections that loop alone measured as the single largest cost in
        # the whole perception pipeline (see Gate 6.3's 2ms/tick budget).
        # track_pos reuses _batched_predict's own (N, 2) output array
        # (same dict, unmutated since predict ran) instead of re-gathering
        # it via a second dict-lookup pass + a second np.stack.
        track_pos = self._predicted_positions                                                    # (N, 2)
        det_pos = np.array([[det.x, det.z] for det in detections])                              # (M, 2)
        diff = track_pos[:, None, :] - det_pos[None, :, :]                                       # (N, M, 2)
        dist = np.hypot(diff[..., 0], diff[..., 1])                                              # (N, M)
        cost = np.where(dist <= MAX_ASSOCIATION_DIST_M, dist, MAX_ASSOCIATION_DIST_M * 10.0)

        row_idx, col_idx = linear_sum_assignment(cost)
        matches: List[Tuple[int, int]] = []
        match_rows: List[int] = []
        matched_tracks, matched_dets = set(), set()
        for r, c in zip(row_idx, col_idx):
            if cost[r, c] <= MAX_ASSOCIATION_DIST_M:
                matches.append((track_ids[r], c))
                match_rows.append(r)
                matched_tracks.add(track_ids[r])
                matched_dets.add(c)

        unmatched_track_ids = [tid for tid in track_ids if tid not in matched_tracks]
        unmatched_det_idxs = [j for j in range(len(detections)) if j not in matched_dets]
        return matches, np.array(match_rows, dtype=np.intp), unmatched_track_ids, unmatched_det_idxs

    def confirmed_tracks(self) -> List[Track]:
        return [t for t in self.tracks.values() if t.status == TrackStatus.CONFIRMED]
