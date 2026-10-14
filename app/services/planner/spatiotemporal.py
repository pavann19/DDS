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
from app.services.planner.joint_physics import _joint_acceleration_components, reference_curvature_and_derivative
from app.services.planner.flat_physics_batch import flat_feasible_lattice
from app.services.planner.corridor_proof import lateral_corridor_within, longitudinal_corridor_within
from app.services.planner.actuator_reachability import initial_actuator_reachable
from app.services.grip_budget import grip_acceleration_limit
from app.services.vehicle_geometry import lateral_half_extent,MIN_LATERAL_HALF_EXTENT_M


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
    physics_max_dt_s: float = 0.1
    physics_max_station_step_m: float = 2.0
    physics_max_samples: int = 512

    # --- feasibility (roadmap Phase 8) ---
    wheelbase_m: float = 2.8
    max_steer_rad: float = math.radians(35.0)
    max_steer_rate_radps: float = 0.6
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
    vehicle_half_width_m: float = MIN_LATERAL_HALF_EXTENT_M
    vehicle_chassis_half_width_m: float = 0.99
    vehicle_half_length_m: float = 2.4
    corridor_numeric_tolerance_m: float = 1e-7

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
    # Road friction coefficient in (0, 1]. 1.0 = dry asphalt. Values < 1
    # scale the *lateral* feasibility envelope: available lateral grip is
    # a_lat_max * mu, so a wet (mu=0.5) or icy (mu=0.2) surface makes the
    # same lane-change quintic infeasible unless it is stretched in time.
    # Feasibility also checks combined road curvature and maneuver demand
    # projected normal to actual Cartesian velocity at each map sample.
    # The aggregate Cartesian acceleration must also fit modeled mu*g.
    friction_mu: float = 1.0
    actual_steer_rad: Optional[float] = None


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
    peak_reference_curvature_1pm: float = 0.0
    feasibility_friction_mu: float = 1.0
    checked_horizon_s: float = 0.0
    sampled_aggregate_grip_verified: bool = False
    initial_speed_recovery: bool = False
    continuous_modeled_corridor_verified: bool = False
    initial_actual_steering_checked: bool = False
    full_cartesian_swept_corridor_verified: bool = False
    continuous_route_station_verified: bool = False
    modeled_straight_vehicle_width_m: float = 2.18
    sampled_modeled_vehicle_envelope_verified: bool = False
    continuous_min_half_width_margin_m: float = 1.09


def _feasible(
    lat: QuinticPolynomial,
    lon: QuarticPolynomial,
    d1: float,
    t_eval: float,
    n: int,
    cfg: PlannerConfig,
    max_lat_accel: Optional[float] = None,
    max_lat_jerk: Optional[float] = None,
    sample_cache: Optional[dict] = None,
    context: Optional[PlannerContext] = None,
    t_lat_s: Optional[float] = None,
    t_lon_s: Optional[float] = None,
    start_time_s: float = 0.0,
) -> Optional[Tuple[float, float, float]]:
    """Check the full sampled horizon, including combined Cartesian lateral demand.

    Polynomial comfort, modeled road-width, longitudinal acceleration and jerk
    checks remain. Map curvature is a window estimate, not a corridor certificate.
    """
    if context is not None and context.actual_steer_rad is not None:
        if not math.isfinite(context.actual_steer_rad) or abs(context.actual_steer_rad)>cfg.max_steer_rad+1e-9:
            return None
    try:
        aggregate_grip_cap = grip_acceleration_limit(context.friction_mu) if context is not None else None
    except ValueError:
        return None
    lat_accel_cap = cfg.max_lat_accel_mps2 if max_lat_accel is None else max_lat_accel
    lat_jerk_cap = cfg.max_lat_jerk_mps3 if max_lat_jerk is None else max_lat_jerk
    peak_da = 0.0
    peak_dj = 0.0
    peak_sa = 0.0
    d0 = lat.pos(0.0)
    lo, hi = (d0, d1) if d0 <= d1 else (d1, d0)
    # The lattice reuses each polynomial across many joint candidates.
    # Cache its samples only within this plan call; preserve every check.
    cache = sample_cache if sample_cache is not None else {}
    corridor_key=("continuous_lat",id(lat),d1,start_time_s,min(t_eval,lat.T))
    if corridor_key not in cache:
        cache[corridor_key]=lateral_corridor_within(lat,d1,cfg,start_time_s,t_eval)
    if not cache[corridor_key]:return None
    if context is not None and context.frenet_frame is not None:
        total=context.frenet_frame.station[-1]
        station_key=("continuous_station",id(lon),start_time_s,t_eval,total)
        if station_key not in cache:
            cache[station_key]=longitudinal_corridor_within(lon,t_eval,total,cfg,start_time_s)
        if not cache[station_key]:return None
    lat_duration = lat.T if t_lat_s is None else t_lat_s
    lon_duration = lon.T if t_lon_s is None else t_lon_s
    times_key = ("sample_times", t_eval, n, lat_duration, lon_duration, start_time_s)
    if times_key not in cache:
        times = {start_time_s+(t_eval-start_time_s)*i/n for i in range(n+1)}
        times.update(t for t in (lat_duration, lon_duration) if start_time_s <= t <= t_eval)
        cache[times_key] = tuple(sorted(times))
    sample_times = cache[times_key]
    lat_key = ("lat", id(lat), sample_times)
    lon_key = ("lon", id(lon), sample_times)
    if lat_key not in cache:
        cache[lat_key] = tuple(_lat_state(lat, t, lat_duration) for t in sample_times)
    if lon_key not in cache:
        cache[lon_key] = tuple(_lon_state(lon, t, lon_duration) for t in sample_times)
    # Independent polynomial constraints are invariant across the Cartesian
    # product. Validate them once, then preserve the joint vector check below.
    lat_check_key = ("lat_check", lat_key, d1, lat_accel_cap, lat_jerk_cap)
    if lat_check_key not in cache:
        samples = cache[lat_key]
        cache[lat_check_key] = all(
            all(math.isfinite(v) for v in row)
            and abs(row[0]) <= cfg.road_half_width_m-cfg.edge_margin_m
            and lo-cfg.max_corridor_excursion_m <= row[0] <= hi+cfg.max_corridor_excursion_m
            and abs(row[2]) <= lat_accel_cap and row[3] <= lat_jerk_cap for row in samples)
    lon_check_key = ("lon_check", lon_key)
    if lon_check_key not in cache:
        cache[lon_check_key] = all(all(math.isfinite(v) for v in row)
            and row[1] >= -1e-6 and cfg.min_long_accel_mps2 <= row[2] <= cfg.max_long_accel_mps2
            for row in cache[lon_key])
        if cache[lon_check_key]:
            velocity_key = ("velocity_extrema", id(lon), lon_duration)
            if velocity_key not in cache:
                cache[velocity_key] = _longitudinal_velocity_extrema(lon, lon_duration)
            velocity_min, _ = cache[velocity_key]
            cache[lon_check_key] = velocity_min >= -1e-6
    if not cache[lat_check_key] or not cache[lon_check_key]:
        return None
    previous_reference_steer = None
    previous_time = None
    previous_speed = None
    recovery_allowed = None
    for sample_time, (d, dv, da_signed, dj), (station, sv, sa) in zip(sample_times, cache[lat_key], cache[lon_key]):
        sample_dt = 0.0 if previous_time is None else sample_time-previous_time
        previous_time = sample_time
        da = abs(da_signed)
        sa_peak = abs(sa)
        if context is not None:
            k, k_s = 0.0, 0.0
            if context.frenet_frame is not None:
                total = context.frenet_frame.station[-1]
                if station < -1e-6 or station > total+1e-6:
                    return None
                key = ("curvature", station)
                if key not in cache:
                    try:
                        cache[key] = reference_curvature_and_derivative(context.frenet_frame, station)
                    except ValueError:
                        return None
                k, k_s = cache[key]
            try:
                joint_sa, joint_da, joint_speed = _joint_acceleration_components(k, k_s, d, dv, da_signed, sv, sa)
            except ValueError:
                return None
            # A route-following car cannot continue a lateral maneuver while
            # its route-tangent motion has stopped (sideways polynomial motion).
            if abs(sv) < 1e-6 and abs(dv) > 1e-6:
                return None
            reference_curvature = 0.0 if joint_speed < 1e-6 else -joint_da/(joint_speed*joint_speed)
            q = cfg.wheelbase_m*reference_curvature
            if abs(q) >= 2.0:
                return None
            # Curvature has no steering inverse at standstill. Preserve the
            # measured wheel angle instead of inventing a zero-angle jump.
            resting_steer=(context.actual_steer_rad if previous_reference_steer is None
                           and context.actual_steer_rad is not None else previous_reference_steer or 0.)
            reference_steer = resting_steer if joint_speed < 1e-6 else math.atan(q/math.sqrt(1.0-.25*q*q))
            if abs(reference_steer) > cfg.max_steer_rad+1e-9:
                return None
            if previous_reference_steer is not None and sample_dt > 0:
                if abs(reference_steer-previous_reference_steer)/sample_dt > cfg.max_steer_rate_radps+1e-9:
                    return None
            previous_reference_steer = reference_steer
            body_steer = context.actual_steer_rad if sample_time==start_time_s and context.actual_steer_rad is not None else reference_steer
            body_heading = -math.atan2(dv,(1+k*d)*sv)-math.atan(.5*math.tan(body_steer))
            body_extent = lateral_half_extent(body_heading,body_steer,cfg.wheelbase_m,cfg.vehicle_chassis_half_width_m,cfg.vehicle_half_length_m)
            if abs(d)+body_extent > cfg.road_half_width_m-cfg.edge_margin_m+cfg.corridor_numeric_tolerance_m:
                return None
            if recovery_allowed is None:
                recovery_allowed = joint_speed > context.max_speed_mps+1e-6
            if joint_speed > context.max_speed_mps+1e-6:
                # Causal recovery never labels existing overspeed as compliant.
                # It may only decrease, with no positive physical acceleration,
                # and must be within the cap when the longitudinal plan ends.
                if (not recovery_allowed or joint_sa > 1e-6 or
                    (previous_speed is not None and joint_speed > previous_speed+1e-6) or
                    sample_time >= lon_duration-1e-9):
                    return None
            previous_speed = joint_speed
            if math.hypot(joint_sa,joint_da)>aggregate_grip_cap+1e-9:
                return None
            joint_da_abs = abs(joint_da)
            if joint_da_abs > lat_accel_cap:
                return None
            if joint_sa < cfg.min_long_accel_mps2 or joint_sa > cfg.max_long_accel_mps2:
                return None
            if joint_da_abs > da:
                da = joint_da_abs
            if abs(joint_sa) > sa_peak:
                sa_peak = abs(joint_sa)
        if da > peak_da:
            peak_da = da
        if dj > peak_dj:
            peak_dj = dj
        if sa_peak > peak_sa:
            peak_sa = sa_peak
    if context is not None and not initial_actuator_reachable(lat,lon,context,cfg,start_time_s):
        return None
    return peak_da, peak_dj, peak_sa


def _lat_state(poly, t, duration):
    if t > duration:
        return poly.pos(duration), 0.0, 0.0, 0.0
    return poly.pos(t), poly.vel(t), poly.acc(t), abs(poly.jerk(t))


def _lon_state(poly, t, duration):
    if t > duration:
        return poly.pos(duration)+poly.vel(duration)*(t-duration), poly.vel(duration), 0.0
    return poly.pos(t), poly.vel(t), poly.acc(t)


def _initial_physical_speed(start, ctx):
    k = 0.0 if ctx.frenet_frame is None else reference_curvature_and_derivative(ctx.frenet_frame,start.s0)[0]
    return math.hypot((1+k*start.d0)*start.sd0,start.dd0)


def _longitudinal_velocity_extrema(lon, t_lon):
    """Exact minimum and maximum of the quartic trajectory velocity."""
    a2, a3, a4 = lon.coef[2:5]
    roots = []
    qa, qb, qc = 12*a4, 6*a3, 2*a2
    if abs(qa) < 1e-12:
        if abs(qb) > 1e-12:
            roots.append(-qc/qb)
    else:
        discriminant = qb*qb-4*qa*qc
        if discriminant >= 0:
            roots.extend(((-qb+math.sqrt(discriminant))/(2*qa),(-qb-math.sqrt(discriminant))/(2*qa)))
    times = [0., t_lon] + [t for t in roots if 0. < t < t_lon]
    velocities = [lon.vel(t) for t in times]
    return min(velocities), max(velocities)


def _physics_sample_count(lon, t_lon, horizon, ctx, cfg, velocity_bounds=None):
    """Bound sample spacing by time and exact polynomial peak station speed."""
    minimum, maximum = _longitudinal_velocity_extrema(lon, t_lon) if velocity_bounds is None else velocity_bounds
    if minimum < -1e-6:
        return cfg.physics_max_samples+1
    if ctx.frenet_frame is None:
        return cfg.sample_n
    peak_speed = max(abs(minimum), abs(maximum))
    return max(cfg.sample_n, math.ceil(horizon/cfg.physics_max_dt_s),
               math.ceil(peak_speed*horizon/cfg.physics_max_station_step_m))


def _risk_integral(
    lat: QuinticPolynomial,
    lon: QuarticPolynomial,
    t_eval: float,
    ctx: PlannerContext,
    cfg: PlannerConfig,
    t_lat_s: Optional[float] = None,
    t_lon_s: Optional[float] = None,
) -> float:
    if ctx.risk_field is None or ctx.frenet_frame is None:
        return 0.0
    from app.services.frenet import frenet_to_local_xz  # local import: optional path

    total = 0.0
    n = cfg.risk_sample_n
    step = t_eval / n
    for i in range(1, n + 1):
        t = step * i
        s = _lon_state(lon, t, t_eval if t_lon_s is None else t_lon_s)[0]
        d = _lat_state(lat, t, t_eval if t_lat_s is None else t_lat_s)[0]
        x, z, _, _ = frenet_to_local_xz(ctx.frenet_frame, s, d)
        total += ctx.risk_field.risk_at(x, z, t) * step
    return total


def plan(
    start: PlannerStart,
    ctx: PlannerContext,
    cfg: PlannerConfig = DEFAULT_CONFIG,
) -> Optional[PlannedTrajectory]:
    """Search the joint lattice; return the minimum-cost feasible
    trajectory, or ``None`` if nothing is feasible. The caller must hold
    lateral position and slow safely instead of bypassing this filter."""
    lane_centers = (
        list(ctx.allowed_lane_centers_m)
        if ctx.allowed_lane_centers_m is not None
        else [ctx.lane_center_d_m]
    )
    v_ref = min(ctx.target_speed_mps, ctx.max_speed_mps)

    # Friction-scaled lateral feasibility envelope. Lateral acceleration is
    # grip-limited -> scales linearly with mu. Lateral jerk is a comfort /
    # steering-rate limit, only weakly grip-coupled -> scales with sqrt(mu).
    _mu = max(0.1, min(1.0, ctx.friction_mu))
    _eff_lat_accel = cfg.max_lat_accel_mps2 * _mu
    _eff_lat_jerk = cfg.max_lat_jerk_mps3 * math.sqrt(_mu)

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

    # A car that stopped between lane centres cannot start a lateral quintic
    # at the same instant as a zero-acceleration longitudinal launch: its
    # nonzero lateral jerk demands unreachable initial curvature. Offer an
    # held-offset launch alongside the regular lane-centre candidates.
    # It passes every existing geometric, actuator, risk and grip check;
    # once launch speed is reached, the ordinary lattice can finish the
    # maneuver. Include measured low-speed replans as well as exact rest;
    # retain their actual lateral derivatives rather than inventing a hold.
    if _initial_physical_speed(start,ctx) < 2.0:
        lane_c=min(lane_centers,key=lambda value:abs(value-start.d0))
        for T in cfg.t_lat_options_s:
            laterals.append((QuinticPolynomial(start.d0,start.dd0,start.ddd0,start.d0,0.,0.,T),T,start.d0,lane_c))

    # --- longitudinal quartics (velocity keeping) ---
    longs: List[Tuple[QuarticPolynomial, float, float]] = []  # poly, T, v1
    for frac in cfg.v_lattice_frac:
        v1 = max(0.0, min(ctx.max_speed_mps, ctx.target_speed_mps * frac))
        for T in cfg.t_lon_options_s:
            longs.append((QuarticPolynomial(start.s0, start.sd0, start.sdd0, v1, 0.0, T), T, v1))

    if _initial_physical_speed(start,ctx) < 2.0:
        # The cruise-relative lattice's smallest positive speed can require
        # excessive acceleration even over its longest horizon (e.g.50km/h
        # cruise ->6.94m/s in4s). Include physically checked low-speed launches
        # so rest/creeping is not the only feasible candidate at high cruise
        # settings or after a measured low-speed replan.
        for launch_speed in (1.0,2.0):
            v1=min(ctx.max_speed_mps,ctx.target_speed_mps,launch_speed)
            if v1>0.0:
                for T in cfg.t_lon_options_s:
                    longs.append((QuarticPolynomial(start.s0,start.sd0,start.sdd0,v1,0.,T),T,v1))

    best: Optional[PlannedTrajectory] = None
    evaluated = 0
    feasible_count = 0

    sample_cache = {}
    flat_results = None
    if ctx.frenet_frame is None:
        velocity_bounds = {id(lon):_longitudinal_velocity_extrema(lon,T) for lon,T,_ in longs}
        flat_results = flat_feasible_lattice(laterals,longs,ctx,cfg,_eff_lat_accel,_eff_lat_jerk,velocity_bounds)
    for lat, t_lat, d1, lane_c in laterals:
        for lon, t_lon, v1 in longs:
            evaluated += 1
            t_eval = max(t_lat, t_lon)
            if flat_results is not None:
                # Flat batch already checked exact longitudinal velocity bounds
                # and uses precisely cfg.sample_n. Do not repeat that work for
                # every lateral/longitudinal pairing.
                n=cfg.sample_n
                if n>cfg.physics_max_samples:
                    continue
                feas=flat_results[(id(lat),id(lon))]
            else:
                count_key = ("sample_count", id(lon), t_eval)
                if count_key not in sample_cache:
                    velocity_key = ("velocity_extrema", id(lon), t_lon)
                    if velocity_key not in sample_cache:
                        sample_cache[velocity_key] = _longitudinal_velocity_extrema(lon, t_lon)
                    sample_cache[count_key] = _physics_sample_count(lon, t_lon, t_eval, ctx, cfg, sample_cache[velocity_key])
                n = sample_cache[count_key]
                if n > cfg.physics_max_samples:
                    continue
                feas = _feasible(
                    lat, lon, d1, t_eval, n, cfg,
                    max_lat_accel=_eff_lat_accel, max_lat_jerk=_eff_lat_jerk,
                    sample_cache=sample_cache, context=ctx, t_lat_s=t_lat, t_lon_s=t_lon,
                )
            if feas is None:
                continue
            feasible_count += 1
            peak_da, peak_dj, peak_sa = feas

            lat_jerk_key, lon_jerk_key = ("lat_jerk", id(lat)), ("lon_jerk", id(lon))
            if lat_jerk_key not in sample_cache:
                sample_cache[lat_jerk_key] = lat.jerk_squared_integral(t_lat)
            if lon_jerk_key not in sample_cache:
                sample_cache[lon_jerk_key] = lon.jerk_squared_integral(t_lon)
            jerk_lat, jerk_lon = sample_cache[lat_jerk_key], sample_cache[lon_jerk_key]
            time_cost = t_lat + t_lon
            lane_dev = (d1 - lane_c) ** 2
            speed_dev = (v1 - v_ref) ** 2
            risk = _risk_integral(lat, lon, t_eval, ctx, cfg, t_lat, t_lon)

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

            winner_times = sorted({t_eval*i/n for i in range(n+1)} | {t_lat,t_lon})
            d_samples = tuple((t, _lat_state(lat,t,t_lat)[0]) for t in winner_times)
            s_samples = tuple((t, _lon_state(lon,t,t_lon)[0]) for t in winner_times)
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
                peak_reference_curvature_1pm=max((abs(sample_cache.get(("curvature", station), (0.,0.))[0]) for _,station in s_samples), default=0.),
                feasibility_friction_mu=_mu, checked_horizon_s=t_eval,
                sampled_aggregate_grip_verified=True,
                initial_speed_recovery=_initial_physical_speed(start,ctx)>ctx.max_speed_mps+1e-6,
                continuous_modeled_corridor_verified=True,
                initial_actual_steering_checked=ctx.actual_steer_rad is not None,
                continuous_route_station_verified=ctx.frenet_frame is not None,
                sampled_modeled_vehicle_envelope_verified=True,
                continuous_min_half_width_margin_m=cfg.vehicle_half_width_m,
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


def sample_trajectory_state(traj, t):
    """Return (s, d, s_dot, d_dot), holding each polynomial's terminal state.

    Feasibility applies only through checked_horizon_s, not beyond that horizon.
    """
    if not math.isfinite(t) or t < 0:
        raise ValueError("invalid trajectory sample time")
    s, sd, _ = _lon_state(traj.lon_poly, t, traj.t_lon_s)
    d, dd, _, _ = _lat_state(traj.lat_poly, t, traj.t_lat_s)
    return s, d, sd, dd


def revalidate_trajectory_remaining(traj, elapsed_s, ctx, cfg=DEFAULT_CONFIG):
    """Recheck the remaining committed polynomial against the current envelope.

    Uses its recorded map-relative station, not a certificate of actual tracking
    pose. Callers must also replan on tracking error or map/frame changes.
    """
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        return False
    horizon = max(traj.t_lat_s, traj.t_lon_s)
    if elapsed_s >= horizon:
        return False
    mu = max(0.1, min(1., ctx.friction_mu))
    n = _physics_sample_count(traj.lon_poly, traj.t_lon_s, horizon, ctx, cfg)
    if n > cfg.physics_max_samples:
        return False
    return _feasible(traj.lat_poly, traj.lon_poly, traj.d1_m, horizon,
        n, cfg, max_lat_accel=cfg.max_lat_accel_mps2*mu,
        max_lat_jerk=cfg.max_lat_jerk_mps3*math.sqrt(mu), context=ctx,
        t_lat_s=traj.t_lat_s, t_lon_s=traj.t_lon_s, start_time_s=elapsed_s) is not None
