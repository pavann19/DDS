"""Per-agent trajectory forecasting (Phase 7).

Two motion models, picked per agent per tick:

* **CTRA** -- Constant Turn Rate and Acceleration. Used for agents that are
  actually maneuvering (estimated |yaw rate| above a threshold) or when no
  road geometry is available. Midpoint-integrated in the ego-local metric
  frame so it needs no closed-form basis assumptions.
* **Frenet lane-following** -- for agents tracking the road: advance station
  ``s`` at the along-track speed (+ along-track accel), and relax the
  lateral offset ``d`` toward the nearest lane centre with a quintic
  (zero lateral velocity/accel at the settle point). This is what stops the
  planner treating a car mid-curve as if it will fly off tangentially.

Inputs are ``AgentKinematics`` -- the minimal per-agent state, built from a
``SurroundTrack``/``TrackEstimate`` (the ego's *sensor-resolved* picture,
never raw NPC truth) plus, optionally, the previous tick's kinematics to
finite-difference a yaw rate and a longitudinal acceleration.

Output is ``interfaces.AgentPrediction`` (a tuple of ``PredictedState`` at
``step_s`` spacing out to ``horizon_s``).

Local frame convention matches ``frenet.py``: x = East, z = South, and the
forward unit vector at heading ``h`` is ``(sin h, -cos h)``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from app.services.frenet import (
    FrenetFrame,
    frenet_to_local_xz,
    frenet_to_local_xz_batch,
)
from app.services.interfaces import AgentPrediction, PredictedState

DEFAULT_HORIZON_S = 3.0
DEFAULT_STEP_S = 0.1
# Sub-step for the midpoint integrator; DEFAULT_STEP_S must be an integer
# multiple of it.
_INTERNAL_STEP_S = 0.05

# Above this estimated yaw rate (~2.9 deg/s) an agent is "maneuvering" and
# gets the CTRA model even when road geometry is available.
YAW_RATE_MANEUVER_THRESH_RADPS = 0.05

# Clamps on finite-differenced estimates -- one noisy tracker frame must not
# produce an absurd forecast.
MAX_ABS_YAW_RATE_RADPS = 1.0          # ~57 deg/s, a hard cornering limit
MAX_ABS_LONG_ACCEL_MPS2 = 4.0        # firm braking / strong acceleration

# Default lateral settle time for the lane-following quintic.
LANE_SETTLE_S = 2.0

# Same-direction lane centres (traffic.py LANE_OFFSETS positive side) plus
# the oncoming pair, so an agent in any modelled lane relaxes to its own
# centre rather than being yanked across the road.
DEFAULT_LANE_CENTERS_M: Tuple[float, ...] = (-5.25, -1.75, 1.75, 5.25)


@dataclass
class AgentKinematics:
    """Minimal per-agent state the forecaster consumes for one tick."""

    track_id: int
    x: float
    z: float
    vx: float
    vz: float
    yaw_rate_radps: float = 0.0
    a_long_mps2: float = 0.0

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.vx, self.vz)

    @property
    def heading_rad(self) -> float:
        # forward = (sin h, -cos h)  =>  h = atan2(vx, -vz)
        return math.atan2(self.vx, -self.vz)


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def kinematics_from_track(
    track,
    prev: Optional[AgentKinematics] = None,
    dt: float = DEFAULT_STEP_S,
) -> AgentKinematics:
    """Build ``AgentKinematics`` from a track object exposing
    ``track_id, x, z, vx, vz``. When ``prev`` (same agent, previous tick) is
    supplied, finite-difference a yaw rate and longitudinal acceleration
    from it; both are clamped so a single noisy frame cannot blow up the
    forecast.
    """
    k = AgentKinematics(
        track_id=int(track.track_id),
        x=float(track.x),
        z=float(track.z),
        vx=float(track.vx),
        vz=float(track.vz),
    )
    if prev is not None and dt > 1e-6:
        yaw_rate = _wrap_pi(k.heading_rad - prev.heading_rad) / dt
        a_long = (k.speed_mps - prev.speed_mps) / dt
        k.yaw_rate_radps = max(-MAX_ABS_YAW_RATE_RADPS, min(MAX_ABS_YAW_RATE_RADPS, yaw_rate))
        k.a_long_mps2 = max(-MAX_ABS_LONG_ACCEL_MPS2, min(MAX_ABS_LONG_ACCEL_MPS2, a_long))
    return k


def _sample_indices(horizon_s: float, step_s: float) -> int:
    return max(1, int(round(horizon_s / step_s)))


def forecast_ctra(
    k: AgentKinematics,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
) -> List[PredictedState]:
    """Constant Turn Rate and Acceleration, midpoint-integrated.

    Speed is floored at 0 (a decelerating agent stops, it does not reverse).
    Yaw rate and longitudinal accel are held constant over the horizon.
    """
    n_out = _sample_indices(horizon_s, step_s)
    sub_per_out = max(1, int(round(step_s / _INTERNAL_STEP_S)))
    h = step_s / sub_per_out

    x, z = k.x, k.z
    v = k.speed_mps
    theta = k.heading_rad
    omega = k.yaw_rate_radps
    a = k.a_long_mps2

    out: List[PredictedState] = []
    for _ in range(n_out):
        for _ in range(sub_per_out):
            v_mid = max(0.0, v + a * (h * 0.5))
            theta_mid = theta + omega * (h * 0.5)
            fwd_x, fwd_z = math.sin(theta_mid), -math.cos(theta_mid)
            x += v_mid * fwd_x * h
            z += v_mid * fwd_z * h
            v = max(0.0, v + a * h)
            theta += omega * h
        fwd_x, fwd_z = math.sin(theta), -math.cos(theta)
        out.append(PredictedState(t_s=round((len(out) + 1) * step_s, 4),
                                  x=x, z=z, vx=v * fwd_x, vz=v * fwd_z))
    return out


def _quintic_coeffs(p0: float, v0: float, a0: float, p1: float, T: float) -> Tuple[float, float, float, float, float, float]:
    """5th-order polynomial with (p0, v0, a0) at t=0 and (p1, 0, 0) at t=T."""
    if T <= 1e-6:
        return p1, 0.0, 0.0, 0.0, 0.0, 0.0
    c0, c1, c2 = p0, v0, 0.5 * a0
    T2, T3, T4, T5 = T * T, T ** 3, T ** 4, T ** 5
    dp = p1 - (c0 + c1 * T + c2 * T2)
    dv = -(c1 + 2 * c2 * T)
    da = -(2 * c2)
    c3 = (10 * dp - 4 * dv * T + 0.5 * da * T2) / T3
    c4 = (-15 * dp + 7 * dv * T - da * T2) / T4
    c5 = (6 * dp - 3 * dv * T + 0.5 * da * T2) / T5
    return c0, c1, c2, c3, c4, c5


def _poly(coeffs: Sequence[float], t: float) -> float:
    return sum(c * (t ** i) for i, c in enumerate(coeffs))


def _project_xz_to_frenet(
    frame: FrenetFrame, px: float, pz: float,
    hint_idx: int = 0, window: int = 0,
) -> Tuple[float, float, int]:
    """(x, z) -> (s, d, matched_segment_idx) by exact clamped point-to-segment
    projection. Mirrors frenet.project_to_frenet's math without the lat/lng
    round-trip (agents are already in the local metric frame).

    ``window`` > 0 bounds the search to ``[hint_idx - window, hint_idx +
    window]`` -- pass the previous tick's matched index so a slow-moving
    agent is not re-scanned against the whole route every tick.
    """
    pts = frame.points_xz
    station = frame.station
    n_seg = len(pts) - 1
    if window > 0:
        lo = max(0, hint_idx - window)
        hi = min(n_seg, hint_idx + window)
    else:
        lo, hi = 0, n_seg
    best_dist_sq = float("inf")
    best_s, best_d, best_idx = station[0], 0.0, lo
    for i in range(lo, hi):
        ax, az = pts[i]
        bx, bz = pts[i + 1]
        dx, dz = bx - ax, bz - az
        seg_len_sq = dx * dx + dz * dz
        if seg_len_sq < 1e-9:
            continue
        t = ((px - ax) * dx + (pz - az) * dz) / seg_len_sq
        t = max(0.0, min(1.0, t))
        cx, cz = ax + t * dx, az + t * dz
        dist_sq = (px - cx) ** 2 + (pz - cz) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            seg_len = math.sqrt(seg_len_sq)
            best_s = station[i] + t * seg_len
            right_x, right_z = dz / seg_len, -dx / seg_len
            best_d = (px - cx) * right_x + (pz - cz) * right_z
            best_idx = i
    return best_s, best_d, best_idx


def _nearest_lane_center(d: float, lane_centers: Sequence[float]) -> float:
    return min(lane_centers, key=lambda c: abs(c - d))


def project_agent_frenet(
    frame: FrenetFrame, x: float, z: float, vx: float, vz: float,
    hint_idx: int = 0, window: int = 0,
) -> Tuple[float, float, float, float, int]:
    """(x, z, vx, vz) -> (s, d, v_s, v_d, matched_idx): station, signed
    lateral offset, the velocity resolved onto the route tangent / normal at
    ``s``, and the matched segment index (feed back as ``hint_idx`` next
    tick, with ``window`` > 0, to skip the full-route scan).

    ``v_d`` is *lane-relative* lateral drift -- an agent faithfully
    following a curve has ``v_d ~= 0`` even while its Cartesian heading
    sweeps, which is exactly what keeps intent estimation from crying
    "cut-in" on every bend (Gate 7.3).
    """
    s, d, idx = _project_xz_to_frenet(frame, x, z, hint_idx=hint_idx, window=window)
    _, _, dir_x, dir_z = frenet_to_local_xz(frame, s, 0.0)
    right_x, right_z = dir_z, -dir_x
    v_s = vx * dir_x + vz * dir_z
    v_d = vx * right_x + vz * right_z
    return s, d, v_s, v_d, idx


def forecast_lane_following(
    k: AgentKinematics,
    frame: FrenetFrame,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    lane_centers: Sequence[float] = DEFAULT_LANE_CENTERS_M,
    settle_s: float = LANE_SETTLE_S,
    frenet0: Optional[Tuple[float, float, float, float]] = None,
) -> List[PredictedState]:
    """Advance the agent along the road: ``s`` at along-track speed (+ accel),
    ``d`` relaxed to its nearest lane centre with a quintic that has zero
    lateral velocity/accel at ``settle_s`` and holds afterwards.

    ``frenet0`` -- precomputed ``(s0, d0, v_s, v_d)`` from
    :func:`project_agent_frenet`; pass it when the caller already projected
    this agent this tick to avoid a second full-route scan.
    """
    if frenet0 is None:
        s0, d0, v_s, v_d, _ = project_agent_frenet(frame, k.x, k.z, k.vx, k.vz)
    else:
        s0, d0, v_s, v_d = frenet0
    a_s = k.a_long_mps2                        # treat longitudinal accel as along-track

    d_target = _nearest_lane_center(d0, lane_centers)
    T = max(1e-3, min(settle_s, horizon_s))
    d_coeffs = _quintic_coeffs(d0, v_d, 0.0, d_target, T)

    s_end = frame.station[-1]
    n_out = _sample_indices(horizon_s, step_s)

    ts = [i * step_s for i in range(1, n_out + 1)]
    ss = [max(0.0, min(s0 + v_s * t + 0.5 * a_s * t * t, s_end)) for t in ts]
    ds = [(_poly(d_coeffs, t) if t < T else d_target) for t in ts]
    xs, zs, _, _ = frenet_to_local_xz_batch(frame, ss, ds)
    xs = [float(v) for v in xs]
    zs = [float(v) for v in zs]

    out: List[PredictedState] = []
    for i in range(n_out):
        lo = max(0, i - 1)
        hi = min(n_out - 1, i + 1)
        span = (hi - lo) * step_s or step_s
        vx = (xs[hi] - xs[lo]) / span
        vz = (zs[hi] - zs[lo]) / span
        out.append(PredictedState(t_s=round(ts[i], 4), x=xs[i], z=zs[i], vx=vx, vz=vz))
    return out


def forecast_agent(
    k: AgentKinematics,
    frame: Optional[FrenetFrame] = None,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    lane_centers: Sequence[float] = DEFAULT_LANE_CENTERS_M,
    frenet0: Optional[Tuple[float, float, float, float]] = None,
) -> AgentPrediction:
    """Pick a model and forecast one agent.

    CTRA when the agent is maneuvering (|yaw rate| >= threshold) or when no
    road frame is available; Frenet lane-following otherwise. ``frenet0``
    (precomputed projection) is forwarded to the lane-following model.
    """
    maneuvering = abs(k.yaw_rate_radps) >= YAW_RATE_MANEUVER_THRESH_RADPS
    if frame is None or maneuvering:
        states = forecast_ctra(k, horizon_s, step_s)
    else:
        states = forecast_lane_following(k, frame, horizon_s, step_s, lane_centers, frenet0=frenet0)
    return AgentPrediction(track_id=k.track_id, states=tuple(states))
