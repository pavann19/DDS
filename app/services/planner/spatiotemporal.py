"""Phase 8 -- joint spatiotemporal ``(s, d, t)`` motion planner.

Replaces the decoupled "pick a lateral offset, then separately cap the
speed" decision with a single joint search over Frenet trajectories, after
Werling et al. (ICRA 2010):

  * a lattice of **lateral** quintics ``d(t)`` -- terminal offsets around
    the intended lane centre, several maneuver durations, always arriving
    with zero lateral speed / acceleration;
  * a lattice of **longitudinal** quartics ``s(t)`` in velocity-keeping
    mode -- terminal speeds fanned below the target, several durations;
  * every lateral x longitudinal pair is one joint candidate;
  * infeasible candidates (comfort / road-width limits) are filtered;
  * the rest are ranked by a quadratic cost over lateral + longitudinal
    jerk, maneuver time, terminal lane deviation, terminal speed
    deviation, and the integral of the Phase-7 risk field along the path;
  * the minimum-cost feasible trajectory wins.

The winner's state at ``t = dt`` (one controller tick ahead) is handed
back as ``d_target_m`` / ``v_target_mps`` -- the existing pure-pursuit
steering and jerk-limited speed controller then *track* that, and the
IDM / Safety-Shield vetoes still compose on top. This planner decides
*what line and speed to aim for*; it does not replace the tracking layer.

Longitudinal "following / stopping" (quintic to a terminal station) is
deliberately out of scope here -- the IDM car-following veto in
physics_engine already brakes for a sensed lead, and folding it into the
lattice as well would double-count. It is noted as a later refinement.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from app.services.planner.polynomials import QuarticPolynomial, QuinticPolynomial


@dataclass(frozen=True)
class PlannerConfig:
    # --- lattice ---
    d_lattice_offsets_m: Tuple[float, ...] = (-0.6, -0.3, 0.0, 0.3, 0.6)
    # A rest-to-rest quintic of displacement dd has peak lateral jerk
    # 60*dd/T^3, so a comfortable (< 1.5 m/s^3) 3.5 m lane change needs
    # T >= ~5.2 s. The longer options exist for exactly that; the maneuver
    # runs across several 10 Hz re-plans, it does not have to finish inside
    # the 4 s cost/feasibility window.
    t_lat_options_s: Tuple[float, ...] = (2.0, 3.0, 4.0, 5.0, 6.0)
    v_lattice_frac: Tuple[float, ...] = (1.0, 0.85, 0.7, 0.5, 0.0)
    t_lon_options_s: Tuple[float, ...] = (2.0, 4.0)
    sample_n: int = 9
    risk_sample_n: int = 5

    # --- feasibility (roadmap Phase 8) ---
    max_lat_accel_mps2: float = 2.0
    max_lat_jerk_mps3: float = 1.5
    min_long_accel_mps2: float = -4.5
    max_long_accel_mps2: float = 2.5
    # roadmap's "|d| <= 3.0 m" -- read as how far d(t) may stray *outside*
    # the [d0, d1] maneuver corridor (an absolute Frenet-d bound makes no
    # sense: the adjacent lane centre is already at d = 5.25 m). A
    # monotonic quintic barely leaves the corridor unless the start
    # lateral velocity opposes the maneuver.
    max_corridor_excursion_m: float = 3.0
    # stay on the modelled road surface (matches planner.lateral.ROAD_HALF_WIDTH_M).
    road_half_width_m: float = 7.0
    edge_margin_m: float = 0.5

    # --- cost weights ---
    w_lat_jerk: float = 1.0
    w_lon_jerk: float = 1.0
    w_time: float = 0.1
    w_lane_dev: float = 3.0
    w_speed_dev: float = 0.4
    w_risk: float = 60.0


DEFAULT_CONFIG = PlannerConfig()


@dataclass(frozen=True)
class PlannerStart:
    """Ego Frenet state at the start of the tick."""

    s0: float
    sd0: float   # ds/dt  (~ speed along the route)
    sdd0: float  # d2s/dt2
    d0: float
    dd0: float   # dd/dt  (lateral speed)
    ddd0: float  # d2d/dt2


@dataclass
class PlannerContext:
    target_speed_mps: float
    lane_center_d_m: float
    max_speed_mps: float           # hard cap from curvature / tracking etc.
    dt: float                      # controller tick
    frenet_frame: object | None = None
    risk_field: object | None = None
    # only these terminal lateral offsets are allowed this tick (the
    # lane-change state machine narrows this: LANE_KEEP -> the current lane
    # centre only; EXECUTE_LANE_CHANGE -> the adjacent lane centre).
    allowed_lane_centers_m: Optional[Sequence[float]] = None


@dataclass(frozen=True)
class PlannedTrajectory:
    # The winning polynomials -- the caller COMMITS to these and samples
    # them at the elapsed maneuver time each tick, re-planning only on an
    # event (abort, plan expiry, large tracking error). Sampling a fresh
    # plan at t = dt every tick would keep the car stuck in the quintic's
    # near-zero-velocity opening and the maneuver would never progress.
    lat_poly: QuinticPolynomial
    lon_poly: QuarticPolynomial
    d_samples: Tuple[Tuple[float, float], ...]   # (t, d)
    s_samples: Tuple[Tuple[float, float], ...]   # (t, s)
    cost: float
    cost_terms: dict
    peak_lat_accel_mps2: float
    peak_lat_jerk_mps3: float
    peak_long_accel_mps2: float
    t_lat_s: float
    t_lon_s: float
    d1_m: float
    v1_mps: float
    d_target_m: float
    v_target_mps: float
    # winning lateral quintic's lateral velocity / acceleration one tick
    # ahead -- the caller feeds these back as the next tick's (dd0, ddd0)
    # so a receding-horizon re-plan carries the maneuver's lateral motion
    # forward instead of restarting from ~zero every tick.
    d_vel_target_mps: float
    d_acc_target_mps2: float
    n_candidates_evaluated: int
    n_candidates_feasible: int


def _feasible(
    lat: QuinticPolynomial,
    lon: QuarticPolynomial,
    d1: float,
    t_eval: float,
    n: int,
    cfg: PlannerConfig,
) -> Optional[Tuple[float, float, float]]:
    """Sample the joint trajectory; return (peak |d''|, peak |d'''|,
    peak |s''|) if every sample is within the comfort / road-width
    envelope, else None.

    ``d''(t)`` is used as the lateral-acceleration proxy. The full vehicle
    lateral accel also carries an ``s'(t)^2 * kappa_path`` term, but the
    path curvature already has its own hard speed cap upstream
    (``physics_engine`` ``lateral_accel_limit``); this filter is the
    maneuver's *own* contribution, matching the roadmap's literal
    ``|a_lat| <= 2.0`` feasibility item.
    """
    peak_da = 0.0
    peak_dj = 0.0
    peak_sa = 0.0
    d0 = lat.pos(0.0)
    lo, hi = (d0, d1) if d0 <= d1 else (d1, d0)
    for i in range(n + 1):
        t = t_eval * i / n
        d = lat.pos(t)
        if abs(d) > cfg.road_half_width_m - cfg.edge_margin_m:
            return None          # would leave the modelled road surface
        if d < lo - cfg.max_corridor_excursion_m or d > hi + cfg.max_corridor_excursion_m:
            return None          # strays too far outside the maneuver corridor
        da = abs(lat.acc(t))
        dj = abs(lat.jerk(t))
        sa = lon.acc(t)
        if da > cfg.max_lat_accel_mps2 or dj > cfg.max_lat_jerk_mps3:
            return None
        if sa < cfg.min_long_accel_mps2 or sa > cfg.max_long_accel_mps2:
            return None
        peak_da = max(peak_da, da)
        peak_dj = max(peak_dj, dj)
        peak_sa = max(peak_sa, abs(sa))
    return peak_da, peak_dj, peak_sa


def _risk_integral(
    lat: QuinticPolynomial,
    lon: QuarticPolynomial,
    t_eval: float,
    ctx: PlannerContext,
    cfg: PlannerConfig,
) -> float:
    if ctx.risk_field is None or ctx.frenet_frame is None:
        return 0.0
    from app.services.frenet import frenet_to_local_xz  # local import: optional path

    total = 0.0
    n = cfg.risk_sample_n
    step = t_eval / n
    for i in range(1, n + 1):
        t = step * i
        s = lon.pos(t)
        d = lat.pos(t)
        x, z, _, _ = frenet_to_local_xz(ctx.frenet_frame, s, d)
        total += ctx.risk_field.risk_at(x, z, t) * step
    return total


def plan(
    start: PlannerStart,
    ctx: PlannerContext,
    cfg: PlannerConfig = DEFAULT_CONFIG,
) -> Optional[PlannedTrajectory]:
    """Search the joint lattice; return the minimum-cost feasible
    trajectory, or ``None`` if nothing is feasible (caller falls back to
    the decoupled lane-centre controller)."""
    lane_centers = (
        list(ctx.allowed_lane_centers_m)
        if ctx.allowed_lane_centers_m is not None
        else [ctx.lane_center_d_m]
    )
    v_ref = min(ctx.target_speed_mps, ctx.max_speed_mps)

    # --- lateral quintics ---
    laterals: List[Tuple[QuinticPolynomial, float, float, float]] = []  # poly, T, d1, lane_c
    d_edge = cfg.road_half_width_m - cfg.edge_margin_m
    for lane_c in lane_centers:
        for off in cfg.d_lattice_offsets_m:
            d1 = max(-d_edge, min(d_edge, lane_c + off))
            for T in cfg.t_lat_options_s:
                laterals.append(
                    (QuinticPolynomial(start.d0, start.dd0, start.ddd0, d1, 0.0, 0.0, T), T, d1, lane_c)
                )

    # --- longitudinal quartics (velocity keeping) ---
    longs: List[Tuple[QuarticPolynomial, float, float]] = []  # poly, T, v1
    for frac in cfg.v_lattice_frac:
        v1 = max(0.0, min(ctx.max_speed_mps, ctx.target_speed_mps * frac))
        for T in cfg.t_lon_options_s:
            longs.append((QuarticPolynomial(start.s0, start.sd0, start.sdd0, v1, 0.0, T), T, v1))

    best: Optional[PlannedTrajectory] = None
    evaluated = 0
    feasible_count = 0

    for lat, t_lat, d1, lane_c in laterals:
        for lon, t_lon, v1 in longs:
            evaluated += 1
            t_eval = min(t_lat, t_lon)
            feas = _feasible(lat, lon, d1, t_eval, cfg.sample_n, cfg)
            if feas is None:
                continue
            feasible_count += 1
            peak_da, peak_dj, peak_sa = feas

            jerk_lat = lat.jerk_squared_integral(t_eval)
            jerk_lon = lon.jerk_squared_integral(t_eval)
            time_cost = t_lat + t_lon
            lane_dev = (d1 - lane_c) ** 2
            speed_dev = (v1 - v_ref) ** 2
            risk = _risk_integral(lat, lon, t_eval, ctx, cfg)

            cost = (
                cfg.w_lat_jerk * jerk_lat
                + cfg.w_lon_jerk * jerk_lon
                + cfg.w_time * time_cost
                + cfg.w_lane_dev * lane_dev
                + cfg.w_speed_dev * speed_dev
                + cfg.w_risk * risk
            )

            if best is not None and cost >= best.cost:
                continue

            n = cfg.sample_n
            d_samples = tuple((t_eval * i / n, lat.pos(t_eval * i / n)) for i in range(n + 1))
            s_samples = tuple((t_eval * i / n, lon.pos(t_eval * i / n)) for i in range(n + 1))
            dt = max(ctx.dt, 1e-3)
            best = PlannedTrajectory(
                lat_poly=lat,
                lon_poly=lon,
                d_samples=d_samples,
                s_samples=s_samples,
                cost=cost,
                cost_terms={
                    "lat_jerk": cfg.w_lat_jerk * jerk_lat,
                    "lon_jerk": cfg.w_lon_jerk * jerk_lon,
                    "time": cfg.w_time * time_cost,
                    "lane_dev": cfg.w_lane_dev * lane_dev,
                    "speed_dev": cfg.w_speed_dev * speed_dev,
                    "risk": cfg.w_risk * risk,
                },
                peak_lat_accel_mps2=peak_da,
                peak_lat_jerk_mps3=peak_dj,
                peak_long_accel_mps2=peak_sa,
                t_lat_s=t_lat,
                t_lon_s=t_lon,
                d1_m=d1,
                v1_mps=v1,
                d_target_m=lat.pos(min(dt, t_lat)),
                v_target_mps=max(0.0, lon.vel(min(dt, t_lon))),
                d_vel_target_mps=lat.vel(min(dt, t_lat)),
                d_acc_target_mps2=lat.acc(min(dt, t_lat)),
                n_candidates_evaluated=0,   # filled after the loop
                n_candidates_feasible=0,
            )

    if best is None:
        return None
    return PlannedTrajectory(
        **{
            **best.__dict__,
            "n_candidates_evaluated": evaluated,
            "n_candidates_feasible": feasible_count,
        }
    )
