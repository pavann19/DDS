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

from app.services.frenet import FrenetFrame, frenet_to_local_xz
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


def _project_xz_to_frenet(frame: FrenetFrame, px: float, pz: float) -> Tuple[float, float]:
    """(x, z) -> (s, d) by exact clamped point-to-segment projection over the
    whole route. Mirrors frenet.project_to_frenet's math without the lat/lng
    round-trip (agents are already in the local metric frame)."""
    pts = frame.points_xz
    station = frame.station
    best_dist_sq = float("inf")
    best_s, best_d = station[0], 0.0
    for i in range(len(pts) - 1):
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
    return best_s, best_d


def _nearest_lane_center(d: float, lane_centers: Sequence[float]) -> float:
    return min(lane_centers, key=lambda c: abs(c - d))


def project_agent_frenet(
    frame: FrenetFrame, x: float, z: float, vx: float, vz: float
) -> Tuple[float, float, float, float]:
    """(x, z, vx, vz) -> (s, d, v_s, v_d): station, signed lateral offset,
    and the velocity resolved onto the route tangent / normal at ``s``.

    ``v_d`` is *lane-relative* lateral drift -- an agent faithfully
    following a curve has ``v_d ~= 0`` even while its Cartesian heading
    sweeps, which is exactly what keeps intent estimation from crying
    "cut-in" on every bend (Gate 7.3).
    """
    s, d = _project_xz_to_frenet(frame, x, z)
    _, _, dir_x, dir_z = frenet_to_local_xz(frame, s, 0.0)
    right_x, right_z = dir_z, -dir_x
    v_s = vx * dir_x + vz * dir_z
    v_d = vx * right_x + vz * right_z
    return s, d, v_s, v_d


def forecast_lane_following(
    k: AgentKinematics,
    frame: FrenetFrame,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    lane_centers: Sequence[float] = DEFAULT_LANE_CENTERS_M,
    settle_s: float = LANE_SETTLE_S,
) -> List[PredictedState]:
    """Advance the agent along the road: ``s`` at along-track speed (+ accel),
    ``d`` relaxed to its nearest lane centre with a quintic that has zero
    lateral velocity/accel at ``settle_s`` and holds afterwards.
    """
    s0, d0, v_s, v_d = project_agent_frenet(frame, k.x, k.z, k.vx, k.vz)
    a_s = k.a_long_mps2                        # treat longitudinal accel as along-track

    d_target = _nearest_lane_center(d0, lane_centers)
    T = max(1e-3, min(settle_s, horizon_s))
    d_coeffs = _quintic_coeffs(d0, v_d, 0.0, d_target, T)

    s_end = frame.station[-1]
    n_out = _sample_indices(horizon_s, step_s)
    out: List[PredictedState] = []
    for i in range(1, n_out + 1):
        t = i * step_s
        s = s0 + v_s * t + 0.5 * a_s * t * t
        s = max(0.0, min(s, s_end))
        d = _poly(d_coeffs, t) if t < T else d_target
        x, z, _, _ = frenet_to_local_xz(frame, s, d)
        out.append(PredictedState(t_s=round(t, 4), x=x, z=z, vx=0.0, vz=0.0))

    # Fill velocities by finite difference (forward for the first point).
    for i, st in enumerate(out):
        prev = out[i - 1] if i > 0 else None
        nxt = out[i + 1] if i + 1 < len(out) else None
        if prev is not None and nxt is not None:
            vx = (nxt.x - prev.x) / (2 * step_s)
            vz = (nxt.z - prev.z) / (2 * step_s)
        elif nxt is not None:
            vx = (nxt.x - st.x) / step_s
            vz = (nxt.z - st.z) / step_s
        elif prev is not None:
            vx = (st.x - prev.x) / step_s
            vz = (st.z - prev.z) / step_s
        else:
            vx = vz = 0.0
        out[i] = PredictedState(t_s=st.t_s, x=st.x, z=st.z, vx=vx, vz=vz)
    return out


def forecast_agent(
    k: AgentKinematics,
    frame: Optional[FrenetFrame] = None,
    horizon_s: float = DEFAULT_HORIZON_S,
    step_s: float = DEFAULT_STEP_S,
    lane_centers: Sequence[float] = DEFAULT_LANE_CENTERS_M,
) -> AgentPrediction:
    """Pick a model and forecast one agent.

    CTRA when the agent is maneuvering (|yaw rate| >= threshold) or when no
    road frame is available; Frenet lane-following otherwise.
    """
    maneuvering = abs(k.yaw_rate_radps) >= YAW_RATE_MANEUVER_THRESH_RADPS
    if frame is None or maneuvering:
        states = forecast_ctra(k, horizon_s, step_s)
    else:
        states = forecast_lane_following(k, frame, horizon_s, step_s, lane_centers)
    return AgentPrediction(track_id=k.track_id, states=tuple(states))
