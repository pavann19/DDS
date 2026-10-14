"""``Driver`` -- the DDS decision loop, extracted from ``PhysicsEngine.update()``
(ADR-001 Phase 6.5 "refactor first", CARLA_MIGRATION_PLAN.md Phase B).

The Driver is **simulator-independent**: it imports no ``PhysicsEngine``, no
``TrafficModel``, no ``NpcVehicle``. Each tick it is handed a
``SensorObservation`` (sensor picture) and a ``DriverContext`` (map /
navigation scalars + a bare lane-clear bool query) and returns the lateral
and longitudinal commands. The *world* (deterministic sim today, CARLA later)
keeps ownership of integration: jerk-limited speed, ``advance_position``,
powertrain, yaw-rate diagnostics.

The decision math itself is moved **verbatim** from the former inline block --
same formulas, same ordering, same constants (passed in via ``cfg`` so the
single source of truth stays ``PhysicsEngine``'s class attributes). Behaviour
is byte-for-byte preserved; the full regression suite is the gate.

State the Driver owns (was on ``PhysicsEngine``, now delegated back via
properties): the lane-change FSM, committed-maneuver trajectory, lateral
motion smoothing, the independent ``SafetyMonitor``, the planner's HMI
candidate/target bookkeeping, the shield verdict, and the binding-constraint
/ friction-derate diagnostics.
"""
from __future__ import annotations

import math
from bisect import bisect_left
from app.services.vehicle_kinematics import sideslip
from app.services.vehicle_geometry import lateral_half_extent, CHASSIS_HALF_LENGTH_M
from app.services.driver.sensor_freshness import SensorFreshnessGuard
from app.services.planner.joint_physics import reference_curvature, reference_curvature_and_derivative
from app.services.driver.frenet_motion import project_motion
from app.services.driver.turn_reference import TurnSpeedReference, raw_route_curvature, predict_front_axle, front_axle_lateral_bias, front_axle_heading_advance, steering_demand_constraint, TURN_REFERENCE_HALF_WINDOW_M
from app.services.driver.turn_tracking import continuous_path_heading, center_tracking_steer, offset_path_curvature
from dataclasses import dataclass, replace
from typing import Any, List, Optional

from app.services import safety_shield
from app.services.safety_geometry import Footprint, intersects_during
from app.services.junctions import JunctionBehavior
from app.services.car_following import idm_acceleration
from app.services.driver.context import DriverContext
from app.services.driver.lateral_planner import plan_lateral_offset
from app.services.driver.safety_monitor import SafetyMonitor
from app.services.frenet import (
    frenet_to_local_xz,
    latlng_to_local,
    local_to_latlng,
    project_to_frenet,
    signed_curvature_at,
)
from app.services.interfaces import (
    ActuatorCommand,
    SensorObservation,
    SOURCE_PLANNER,
    SOURCE_SAFETY_OVERRIDE,
)
from app.services.planner import (
    LANE_CENTER_D_M,
    LANE_CHANGE_TRIGGER_GAP_M,
    ff_stanley_steer,
    generate_candidates,
)
from app.services.planner.spatiotemporal import (
    DEFAULT_CONFIG as DEFAULT_PLANNER_CONFIG,
    PlannerContext,
    PlannerStart,
    plan as plan_spatiotemporal,
    revalidate_trajectory_remaining,
    sample_trajectory_state,
)
from app.services.planner.state_machine import LaneChangeState, LaneChangeStateMachine

_CRUISE_SPEED_KMH = 50.0
_CORNER_MAX_TURN_DEG = 90.0
_CORNER_MIN_SPEED_KMH = 15.0


@dataclass(frozen=True)
class DriverConfig:
    """Vehicle / control constants -- mirrors the ``PhysicsEngine`` class
    attributes of the same name so there is one source of truth."""

    WHEELBASE_M: float
    A_LAT_MAX_MPS2: float
    MAX_STEER_RAD: float
    STEER_RATE_MAX_RADPS: float
    STEER_KP: float
    SPEED_KP: float
    A_MAX_ACCEL_MPS2: float
    A_MAX_BRAKE_MPS2: float
    MIN_CORNER_SPEED_KMH: float
    PROACTIVE_CUT_IN_MAX_DIP_KMH: float
    TRACKING_ERROR_STEER_FRACTION_THRESHOLD: float
    LATERAL_TARGET_RATE_MPS: float
    PP_LOOKAHEAD_K: float
    PP_LOOKAHEAD_MIN_M: float
    # Lane geometry (metres of signed Frenet offset). Supplied by the World
    # so the Driver never imports ``app.services.traffic`` (ADR-001 Gate
    # 6.5.4 -- the boundary is a type signature).
    EGO_LANE_OFFSET_M: float
    ADJACENT_LANE_OFFSET_M: float
    JERK_MAX_MPS3: float = 2.5


@dataclass
class LateralDecision:
    steering_angle_rad: float
    steering_angle: float          # normalised [-1, 1]
    heading_deg: float             # world heading AFTER this tick's yaw integration


@dataclass
class LongitudinalDecision:
    accel_mps2: float                       # demanded, clamped to [-brake, +accel]
    speed_kmh_override: Optional[float]     # legacy point-mass path only; else None
    speed_limit_reason: str


class Driver:
    def __init__(self, *, cfg: DriverConfig) -> None:
        self.cfg = cfg
        self.sensor_guard = SensorFreshnessGuard()
        self.planner_cfg = replace(DEFAULT_PLANNER_CONFIG,
            wheelbase_m=cfg.WHEELBASE_M, max_steer_rad=cfg.MAX_STEER_RAD,
            max_steer_rate_radps=cfg.STEER_RATE_MAX_RADPS,
            max_lat_accel_mps2=min(DEFAULT_PLANNER_CONFIG.max_lat_accel_mps2, cfg.A_LAT_MAX_MPS2),
            max_long_accel_mps2=min(DEFAULT_PLANNER_CONFIG.max_long_accel_mps2, cfg.A_MAX_ACCEL_MPS2),
            min_long_accel_mps2=max(DEFAULT_PLANNER_CONFIG.min_long_accel_mps2, -cfg.A_MAX_BRAKE_MPS2))
        # -- HMI / planner bookkeeping (was on PhysicsEngine) --------------
        self.cruise_speed_kmh = _CRUISE_SPEED_KMH
        self._last_map_limit = None
        self.junctions = JunctionBehavior()
        self.lateral_target_d_m: float = LANE_CENTER_D_M
        self.planner_candidates: List[Any] = []
        self.planner_chosen_d_m: float = LANE_CENTER_D_M
        self.spatiotemporal_traj: Optional[Any] = None
        self._planner_v_target_mps: Optional[float] = None
        self._changing_lane_now: bool = False
        # -- lane-change FSM + committed maneuver -------------------------
        self._lc_sm = LaneChangeStateMachine(
            origin_lane_d_m=cfg.EGO_LANE_OFFSET_M, adjacent_lane_d_m=cfg.ADJACENT_LANE_OFFSET_M
        )
        self._prev_lat_offset_m: float = cfg.EGO_LANE_OFFSET_M
        self._prev_lat_speed_mps: float = 0.0
        self._lat_speed_mps: float = 0.0
        self._lat_accel_mps2: float = 0.0
        self._maneuver_traj: Optional[Any] = None
        self._maneuver_t: float = 0.0
        self._maneuver_replanned_for_abort: bool = False
        self._committed_d_target: float = LANE_CENTER_D_M
        self._committed_v_target: float = 0.0
        self._joint_plan_unavailable = False
        self._maneuver_friction_mu: Optional[float] = None
        self._motion_beta_sample = None
        self._sensor_loss_lateral_hold = False
        # -- independent safety supervisor + verdict --------------------
        self.safety_monitor = SafetyMonitor()
        self.shield_verdict = safety_shield.ShieldVerdict(
            approved=True, risk_level=safety_shield.RISK_NONE
        )
        # -- binding-constraint / friction-derate diagnostics ----------
        self.speed_limit_reason: str = "cruise"
        self.target_speed_mps: Optional[float] = None
        self.friction_speed_derate_active: bool = False
        self.friction_curve_cap_kmh: Optional[float] = None
        self.friction_derate_kmh: float = 0.0

    # ------------------------------------------------------------------
    # Resets -- mirror the two flavours PhysicsEngine already had.
    # ------------------------------------------------------------------
    def on_route_reset(self) -> None:
        """A fresh route (``set_route``): full lateral-stack reset."""
        self.lateral_target_d_m = LANE_CENTER_D_M
        self._last_map_limit = None
        self.junctions = JunctionBehavior()
        self.planner_candidates = []
        self.planner_chosen_d_m = LANE_CENTER_D_M
        self.shield_verdict = safety_shield.ShieldVerdict(
            approved=True, risk_level=safety_shield.RISK_NONE
        )
        self._lc_sm = LaneChangeStateMachine(
            origin_lane_d_m=self.cfg.EGO_LANE_OFFSET_M, adjacent_lane_d_m=self.cfg.ADJACENT_LANE_OFFSET_M
        )
        self._prev_lat_offset_m = self.cfg.EGO_LANE_OFFSET_M
        self._prev_lat_speed_mps = 0.0
        self._lat_speed_mps = 0.0
        self._lat_accel_mps2 = 0.0
        self.spatiotemporal_traj = None
        self._planner_v_target_mps = None
        self._changing_lane_now = False
        self._maneuver_traj = None
        self._maneuver_t = 0.0
        self._maneuver_replanned_for_abort = False
        self.target_speed_mps = None
        self._joint_plan_unavailable = False
        self._maneuver_friction_mu = None
        self._motion_beta_sample = None
        self._sensor_loss_lateral_hold = False
        self.safety_monitor.reset()
        self.sensor_guard.reset()

    def on_scenario_reset(self, lateral_offset_m: float) -> None:
        """A scenario ``reset_state`` (route kept): the lighter reset the
        former inline code did -- target lane, reasons, friction flags,
        shield. Does NOT rebuild the FSM or zero the lateral smoothing."""
        self.lateral_target_d_m = lateral_offset_m
        self._last_map_limit = None
        self.junctions = JunctionBehavior()
        self.speed_limit_reason = "cruise"
        self._joint_plan_unavailable = False
        self._maneuver_friction_mu = None
        self._motion_beta_sample = None
        self._sensor_loss_lateral_hold = False
        self.friction_speed_derate_active = False
        self.friction_curve_cap_kmh = None
        self.friction_derate_kmh = 0.0
        self.shield_verdict = safety_shield.ShieldVerdict(
            approved=True, risk_level=safety_shield.RISK_NONE
        )
        self.safety_monitor.reset()
        self.sensor_guard.reset()

    # ------------------------------------------------------------------
    # Lateral decision -- was PhysicsEngine.update() L641..L891
    # ------------------------------------------------------------------
    def cruise_cap(self, obs: SensorObservation):
        limit = obs.road_rules.speed_limit_kmh if obs.road_rules else None
        reason = "mapped_speed_limit"
        if limit is not None:
            self._last_map_limit = (limit, obs.ego.station_m, obs.clock.sim_time_s)
        elif self._last_map_limit is not None:
            previous, station, time_s = self._last_map_limit
            if (0 <= obs.ego.station_m - station <= 50 and
                    0 <= obs.clock.sim_time_s - time_s <= 2):
                limit, reason = previous, "map_uncertainty_hold"
        if limit is None:
            # Modeled-road fallback is a simulation policy, never a mapped
            # legal limit. Unknown coverage cannot release highway cruise.
            limit, reason = 50., "map_unknown_conservative"
        cap = min(self.cruise_speed_kmh, limit)
        return cap, reason if cap < self.cruise_speed_kmh else "cruise"

    def _modeled_lane_turn_feasible(self, frame, station_m, lane_d_m, horizon_m):
        """Steering feasibility of a modeled parallel path, not surveyed lanes.

        The FSM may retain an overtaking lane after a pass. Before a sharp
        route bend, that offset can fold or exceed the physical wheel angle.
        Use the same unclipped turn reference as longitudinal control and
        require a collision-checked joint return, rather than enter that bend.
        """
        references=getattr(self,"_lane_turn_references",None)
        if references is None or any(ref.frame is not frame for ref in references.values()):
            references={};self._lane_turn_references=references
        try:
            reference=references.get(lane_d_m)
            if reference is None:
                reference=TurnSpeedReference(frame,lane_d_m,self.cfg.WHEELBASE_M,self.cfg.MAX_STEER_RAD)
                references[lane_d_m]=reference
            lo=bisect_left(reference.stations,max(0.,station_m-1.))
            hi=bisect_left(reference.stations,min(frame.total_length_m,station_m+horizon_m))+1
            return not bool(reference.infeasible[lo:hi].any())
        except ValueError:
            return False

    def _hold_lateral_during_sensor_loss(self, ctx: DriverContext) -> None:
        # Brake on missing evidence while retaining the mapped lane reference.
        # Updating the target to every measured offset turns tracking error into
        # desired motion and ratchets the car outward through a curved road.
        # A lane change is different: stop advancing its trajectory and latch
        # the measured offset once, rather than following it on every tick.
        if not self._sensor_loss_lateral_hold:
            if self._maneuver_traj is None and self._lc_sm.state not in (LaneChangeState.EXECUTE_LANE_CHANGE,LaneChangeState.ABORT_LANE_CHANGE):
                # Preserve an existing road/parking reference, including a
                # parking aisle's centreline0, when no joint maneuver exists.
                self._sensor_loss_lateral_hold = True
                return
            lanes=(self._lc_sm.origin_lane_d_m,self._lc_sm.adjacent_lane_d_m)
            nearest=min(lanes,key=lambda d:abs(d-ctx.lateral_offset_m))
            # An offset-hold launch is temporary motion, not a change of the
            # mandatory route destination. A second camera outage during that
            # launch must not classify its current d as the unsafe old lane.
            terminal=self._lc_sm.required_destination_d_m
            if terminal is None:
                terminal=getattr(self._maneuver_traj,"d1_m",None)
            if terminal is None:
                terminal=(self._lc_sm.adjacent_lane_d_m if self._lc_sm.state is LaneChangeState.EXECUTE_LANE_CHANGE
                          else self._lc_sm.origin_lane_d_m)
            intended=min(lanes,key=lambda d:abs(d-terminal))
            if intended == nearest:
                # A joint return/tracking correction is already in its intended
                # lane; retain that lane rather than ratcheting its drift.
                self.lateral_target_d_m = intended
            elif self._lc_sm.state in (LaneChangeState.EXECUTE_LANE_CHANGE,LaneChangeState.ABORT_LANE_CHANGE):
                self.lateral_target_d_m = ctx.lateral_offset_m
            self._sensor_loss_lateral_hold = True

    def decide_lateral(self, obs: SensorObservation, ctx: DriverContext) -> LateralDecision:
        cfg = self.cfg
        self._joint_plan_unavailable = False
        health = self.sensor_guard.evaluate(obs.clock.sim_time_s, obs.sensor_health)
        if health.monitored and not health.ready:
            self._hold_lateral_during_sensor_loss(ctx)
            self._planner_v_target_mps = None
            self.spatiotemporal_traj = None
            self._maneuver_traj = None
            self.planner_candidates = []
            self._changing_lane_now = False
            return LateralDecision(ctx.steering_angle_rad, 0., ctx.heading_deg)
        self._sensor_loss_lateral_hold = False
        if obs.parking and obs.parking['active']:
            # A parking aisle has no passing lane. Track the planned vehicle
            # centre path rather than the modeled highway lane offset.
            self.lateral_target_d_m = 0.
            self._planner_v_target_mps = None
            self.spatiotemporal_traj = None
            self.planner_candidates = []
            self._changing_lane_now = False
            return LateralDecision(ctx.steering_angle_rad, 0., ctx.heading_deg)
        heading = ctx.heading_deg
        steering_angle_rad = ctx.steering_angle_rad
        steering_angle = 0.0
        frenet_frame = obs.frenet_frame

        if ctx.waypoint_dist_m > 5.0:
            target_heading = ctx.target_heading_deg
            diff = (target_heading - heading + 180) % 360 - 180

            if ctx.controller == "legacy":
                turn_rate = max(0, min(1, ctx.speed_kmh / 20.0)) * 45.0 * ctx.dt_s
                if abs(diff) > turn_rate:
                    turn = turn_rate if diff > 0 else -turn_rate
                else:
                    turn = diff
                heading = (heading + turn + 360) % 360
                steering_angle = turn / max(0.01, turn_rate) if turn_rate > 0 else 0
                # legacy never sets a physical wheel angle
                return LateralDecision(
                    steering_angle_rad=steering_angle_rad,
                    steering_angle=steering_angle,
                    heading_deg=heading,
                )

            v_mps = ctx.speed_kmh / 3.6
            beta = sideslip(steering_angle_rad)
            sample_time = obs.clock.sim_time_s
            beta_rate = 0.
            if self._motion_beta_sample is not None:
                previous_time, previous_beta = self._motion_beta_sample
                elapsed = sample_time-previous_time
                if elapsed > 1e-6:
                    beta_rate = (beta-previous_beta)/elapsed
            self._motion_beta_sample = (sample_time, beta)
            if v_mps > 0.5:
                eff_a_lat = cfg.A_LAT_MAX_MPS2 * ctx.friction_mu
                grip_steer_limit = math.atan(eff_a_lat * cfg.WHEELBASE_M / (v_mps ** 2))
                steer_limit = min(cfg.MAX_STEER_RAD, grip_steer_limit)
            else:
                steer_limit = cfg.MAX_STEER_RAD

            if frenet_frame is not None:
                lead_gap_m = obs.lead.gap_m if obs.lead is not None else None
                _cur_lane_d = self._lc_sm.origin_lane_d_m
                _adj_lane_d = self._lc_sm.adjacent_lane_d_m
                adjacent_lane_clear = (
                    ctx.lane_clear_fn(ctx.station_m, _adj_lane_d)
                    if ctx.lane_clear_fn is not None else False
                )

                d0 = ctx.lateral_offset_m
                _raw_dd = (d0 - self._prev_lat_offset_m) / ctx.dt_s if ctx.dt_s > 1e-6 else 0.0
                self._lat_speed_mps = 0.5 * self._lat_speed_mps + 0.5 * max(-2.0, min(2.0, _raw_dd))
                _raw_ddd = (self._lat_speed_mps - self._prev_lat_speed_mps) / ctx.dt_s if ctx.dt_s > 1e-6 else 0.0
                self._lat_accel_mps2 = 0.5 * self._lat_accel_mps2 + 0.5 * max(-3.0, min(3.0, _raw_ddd))
                self._prev_lat_offset_m = d0
                self._prev_lat_speed_mps = self._lat_speed_mps

                # Ten seconds/minimum120m is a modeled planning horizon, not
                # a lane survey or a continuous footprint certificate. It
                # gives the existing6s joint maneuver room to return before
                # an offset path becomes physically impossible.
                turn_horizon=max(120.,v_mps*10.)
                current_lane_feasible=self._modeled_lane_turn_feasible(frenet_frame,ctx.station_m,_cur_lane_d,turn_horizon)
                adjacent_lane_feasible=self._modeled_lane_turn_feasible(frenet_frame,ctx.station_m,_adj_lane_d,turn_horizon)
                adjacent_lane_clear=adjacent_lane_clear and adjacent_lane_feasible
                if not current_lane_feasible and adjacent_lane_feasible:
                    self._lc_sm.require_adjacent_destination()
                lane_blocked = ((lead_gap_m is not None and lead_gap_m < LANE_CHANGE_TRIGGER_GAP_M)
                                or not current_lane_feasible)
                allowed_lanes = self._lc_sm.step(
                    current_d_m=d0, lane_blocked=lane_blocked, adjacent_clear=adjacent_lane_clear,
                    lateral_speed_mps=self._lat_speed_mps,
                )
                if (not current_lane_feasible and adjacent_lane_feasible
                        and self._lc_sm.origin_lane_d_m == _cur_lane_d):
                    # PREPARE cancellation clears ordinary maneuver state;
                    # geometry still requires the same destination.
                    self._lc_sm.require_adjacent_destination()
                maneuvering = self._lc_sm.state in (
                    LaneChangeState.EXECUTE_LANE_CHANGE,
                    LaneChangeState.ABORT_LANE_CHANGE,
                )

                traj = None
                is_abort = self._lc_sm.state is LaneChangeState.ABORT_LANE_CHANGE
                required_return_blocked=(
                    self._lc_sm.required_destination_d_m == self._lc_sm.adjacent_lane_d_m
                    and not adjacent_lane_clear)
                if required_return_blocked:
                    # Clearance veto applies before FSM commit too. Following
                    # the known-infeasible origin into the bend can remove all
                    # checked return/launch options before traffic clears.
                    self._joint_plan_unavailable=True
                if maneuvering and not required_return_blocked:
                    _cruise_mps = self.cruise_cap(obs)[0] / 3.6
                    _eff_a_lat = cfg.A_LAT_MAX_MPS2 * ctx.friction_mu
                    _curve_cap_mps = (
                        math.sqrt(_eff_a_lat / ctx.max_curvature_1pm)
                        if ctx.max_curvature_1pm > 1e-6 else _cruise_mps * 2.0
                    )
                    # Physical speed is not Frenet station velocity on
                    # a curved parallel path or while moving laterally.
                    _, _, tx, tz = frenet_to_local_xz(frenet_frame, ctx.station_m, 0.)
                    road_yaw = math.atan2(tx, -tz)
                    motion_yaw = math.radians(heading) + sideslip(steering_angle_rad)
                    k, k_s = reference_curvature_and_derivative(frenet_frame, ctx.station_m)
                    scale = 1.0 + k*d0
                    motion = None
                    try:
                        yaw_rate = v_mps*math.cos(beta)*math.tan(steering_angle_rad)/cfg.WHEELBASE_M
                        motion = project_motion(speed_mps=v_mps, acceleration_mps2=ctx.acceleration_mps2,
                            motion_heading_rad=motion_yaw, road_heading_rad=road_yaw,
                            motion_yaw_rate_radps=yaw_rate+beta_rate,
                            curvature_1pm=k, curvature_derivative_1pm2=k_s, lateral_offset_m=d0)
                    except ValueError:
                        pass
                    station_speed = motion.station_speed_mps if motion is not None else 0.
                    plan_ctx = PlannerContext(
                        target_speed_mps=_cruise_mps, lane_center_d_m=allowed_lanes[0],
                        max_speed_mps=min(_cruise_mps, _curve_cap_mps), dt=ctx.dt_s,
                        frenet_frame=frenet_frame,
                        risk_field=(ctx.prediction_result.risk_field if ctx.prediction_result is not None else None),
                        allowed_lane_centers_m=allowed_lanes,
                        friction_mu=max(0.1, min(1.0, ctx.friction_mu)), actual_steer_rad=steering_angle_rad)
                    _mt = self._maneuver_traj
                    long_deviation = False
                    remaining_feasible = True
                    if _mt is not None:
                        te = min(self._maneuver_t, _mt.t_lon_s)
                        expected_s = _mt.lon_poly.pos(te) + max(0., self._maneuver_t-_mt.t_lon_s) * _mt.lon_poly.vel(_mt.t_lon_s)
                        expected_sd = _mt.lon_poly.vel(te)
                        long_deviation = (abs(ctx.station_m-expected_s) > max(2., .25*v_mps) or
                                          abs(station_speed-expected_sd) > max(.75, .15*station_speed))
                        remaining_feasible = revalidate_trajectory_remaining(_mt, self._maneuver_t, plan_ctx, self.planner_cfg)
                    held_offset_launch=(_mt is not None and
                        abs(_mt.d1_m-_mt.lat_poly.pos(0.))<1e-9)
                    # The offset-hold lateral profile can finish before the checked
                    # longitudinal launch. Keep validating/tracking that full
                    # launch instead of prematurely replanning at low speed,
                    # where a remaining lateral return is still unreachable.
                    committed_horizon=(max(_mt.t_lat_s,_mt.t_lon_s) if held_offset_launch
                                       else _mt.t_lat_s) if _mt is not None else 0.
                    _need_replan = (
                        _mt is None or motion is None
                        or long_deviation or not remaining_feasible
                        or (self._maneuver_friction_mu is not None and
                            ctx.friction_mu < self._maneuver_friction_mu - 1e-9)
                        or (is_abort and not self._maneuver_replanned_for_abort)
                        or self._maneuver_t >= committed_horizon - 1e-6
                        or abs(d0 - _mt.lat_poly.pos(min(self._maneuver_t, _mt.t_lat_s))) > 1.2
                    )
                    if _need_replan:
                        # Replanning always starts from current estimated motion,
                        # not the abandoned polynomial's nominal momentum.
                        _mt = None if motion is None else plan_spatiotemporal(
                            PlannerStart(s0=ctx.station_m, sd0=motion.station_speed_mps,
                                sdd0=motion.station_accel_mps2, d0=d0,
                                dd0=motion.lateral_speed_mps, ddd0=motion.lateral_accel_mps2),
                            plan_ctx, self.planner_cfg,
                        )
                        self._maneuver_traj = _mt
                        self._maneuver_friction_mu = ctx.friction_mu if _mt is not None else None
                        self._maneuver_t = 0.0
                        self._maneuver_replanned_for_abort = is_abort
                    if _mt is not None:
                        self._maneuver_t += ctx.dt_s
                        _te_lat = min(self._maneuver_t + 0.8, _mt.t_lat_s)
                        _te_lon = min(self._maneuver_t, _mt.t_lon_s)
                        traj = _mt
                        self._committed_d_target = _mt.lat_poly.pos(_te_lat)
                        sample_s, sample_d, sample_sd, sample_dd = sample_trajectory_state(_mt, _te_lon)
                        sample_scale = 1. + reference_curvature(frenet_frame, sample_s)*sample_d
                        self._committed_v_target = math.hypot(sample_scale*sample_sd, sample_dd)

                if traj is not None:
                    self._lc_sm.confirm_joint_plan()
                    self.spatiotemporal_traj = traj
                    self.lateral_target_d_m = self._committed_d_target
                    self._planner_v_target_mps = self._committed_v_target
                    self.planner_chosen_d_m = traj.d1_m
                    # Joint plans also represent lane tracking and a held-
                    # offset launch. Describe an actual lane transition only,
                    # using the committed trajectory's semantic endpoints.
                    lanes=(self._lc_sm.origin_lane_d_m,self._lc_sm.adjacent_lane_d_m)
                    start_lane=min(lanes,key=lambda d:abs(d-traj.lat_poly.pos(0.)))
                    end_lane=min(lanes,key=lambda d:abs(d-traj.d1_m))
                    self._changing_lane_now = start_lane != end_lane
                    self.planner_candidates = generate_candidates(
                        current_d=d0, lead_gap_m=lead_gap_m,
                        adjacent_lane_clear=adjacent_lane_clear,
                    )
                    _pick = min(self.planner_candidates, key=lambda c: abs(c.d_target - traj.d1_m))
                    for _c in self.planner_candidates:
                        _c.is_chosen = _c is _pick
                    _pick.is_lane_change = self._changing_lane_now
                    _tx0, _tz0, _mt_x, _mt_z = frenet_to_local_xz(
                        frenet_frame, ctx.station_m, 0.0
                    )
                    _road_h = math.degrees(math.atan2(_mt_x, -_mt_z))
                    _road_k = signed_curvature_at(frenet_frame, ctx.station_m)
                    desired_steer = ff_stanley_steer(
                        current_d_m=ctx.lateral_offset_m,
                        target_d_m=self.lateral_target_d_m,
                        ego_heading_deg=heading,
                        road_heading_deg=_road_h,
                        road_curvature_signed_1pm=_road_k,
                        v_mps=v_mps,
                        wheelbase_m=cfg.WHEELBASE_M,
                        steer_limit_rad=steer_limit,
                    )
                    if (v_mps < .05 and self._lc_sm.required_destination_d_m is not None
                            and abs(traj.lon_poly.vel(traj.t_lon_s)) < 1e-6):
                        # Stationary hold: align the rate-limited wheel with a
                        # possible held-offset launch. No motion is authorized
                        # until the next full joint plan passes every check.
                        k,_=reference_curvature_and_derivative(frenet_frame,ctx.station_m)
                        scale=1.+k*d0
                        q=cfg.WHEELBASE_M*k/scale if scale>0. else float("inf")
                        if abs(q)<2.:
                            launch_steer=math.atan(q/math.sqrt(1.-.25*q*q))
                            launch_extent=lateral_half_extent(math.radians(heading-_road_h),
                                launch_steer,cfg.WHEELBASE_M,
                                self.planner_cfg.vehicle_chassis_half_width_m,
                                self.planner_cfg.vehicle_half_length_m)
                            if (abs(launch_steer)<=steer_limit and abs(d0)+launch_extent <=
                                    self.planner_cfg.road_half_width_m-self.planner_cfg.edge_margin_m):
                                desired_steer=launch_steer
                                self._maneuver_traj=None
                elif maneuvering:
                    self._lc_sm.reject_joint_plan(current_d_m=d0)
                    # A rejected joint maneuver must not escape through the
                    # unchecked decoupled passing planner. Hold the measured
                    # lateral reference and ask longitudinal control to stop.
                    self._joint_plan_unavailable = True
                    self.spatiotemporal_traj = None
                    self._planner_v_target_mps = None
                    self._changing_lane_now = False
                    self._maneuver_traj = None
                    self._maneuver_t = 0.0
                    self._maneuver_friction_mu = None
                    self.planner_candidates = []
                    self.planner_chosen_d_m = d0
                    self.lateral_target_d_m = d0
                    _, _, tx, tz = frenet_to_local_xz(frenet_frame, ctx.station_m, 0.)
                    desired_steer = ff_stanley_steer(
                        current_d_m=d0, target_d_m=d0, ego_heading_deg=heading,
                        road_heading_deg=math.degrees(math.atan2(tx, -tz)),
                        road_curvature_signed_1pm=signed_curvature_at(frenet_frame, ctx.station_m),
                        v_mps=v_mps, wheelbase_m=cfg.WHEELBASE_M, steer_limit_rad=steer_limit)
                else:
                    lateral_plan = plan_lateral_offset(
                        current_lateral_offset_m=ctx.lateral_offset_m,
                        lead_gap_m=lead_gap_m,
                        adjacent_lane_clear=False,
                        lateral_target_d_m=self.lateral_target_d_m,
                        frenet_frame=frenet_frame,
                        current_station_m=ctx.station_m,
                        ego_lat=ctx.ego_lat,
                        ego_lng=ctx.ego_lng,
                        heading_deg=heading,
                        v_mps=v_mps,
                        dt=ctx.dt_s,
                        steer_limit_rad=steer_limit,
                        lateral_target_rate_mps=cfg.LATERAL_TARGET_RATE_MPS,
                        pp_lookahead_k=cfg.PP_LOOKAHEAD_K,
                        pp_lookahead_min_m=cfg.PP_LOOKAHEAD_MIN_M,
                        wheelbase_m=cfg.WHEELBASE_M,
                        lane_center_d_m=_cur_lane_d,
                        adjacent_lane_d_m=_adj_lane_d,
                    )
                    self.spatiotemporal_traj = None
                    self._planner_v_target_mps = None
                    self._changing_lane_now = False
                    self._maneuver_traj = None
                    self._maneuver_t = 0.0
                    self._maneuver_replanned_for_abort = False
                    self.planner_candidates = lateral_plan.candidates
                    self.planner_chosen_d_m = lateral_plan.chosen_d_m
                    self.lateral_target_d_m = lateral_plan.lateral_target_d_m
                    desired_steer = lateral_plan.desired_steer_rad
            else:
                desired_steer = max(-steer_limit,
                                    min(steer_limit, cfg.STEER_KP * math.radians(diff)))

            max_step = cfg.STEER_RATE_MAX_RADPS * ctx.dt_s
            steer_error = desired_steer - steering_angle_rad
            steering_angle_rad += max(-max_step, min(max_step, steer_error))
            steering_angle_rad = max(-steer_limit, min(steer_limit, steering_angle_rad))

            yaw_rate = v_mps * math.cos(sideslip(steering_angle_rad)) * math.tan(steering_angle_rad) / cfg.WHEELBASE_M
            heading = (heading + math.degrees(yaw_rate * ctx.dt_s) + 360) % 360
            steering_angle = steering_angle_rad / cfg.MAX_STEER_RAD
        else:
            steering_angle = 0.0
            if ctx.controller != "legacy":
                max_step = cfg.STEER_RATE_MAX_RADPS * ctx.dt_s
                steering_angle_rad -= max(-max_step, min(max_step, steering_angle_rad))

        return LateralDecision(
            steering_angle_rad=steering_angle_rad,
            steering_angle=steering_angle,
            heading_deg=heading,
        )

    # ------------------------------------------------------------------
    # Longitudinal decision -- was PhysicsEngine.update() L915..L1148
    # ------------------------------------------------------------------
    def decide_longitudinal(self, obs: SensorObservation, ctx: DriverContext, *, steering_demand_rad: Optional[float] = None) -> LongitudinalDecision:
        cfg = self.cfg
        health = self.sensor_guard.evaluate(obs.clock.sim_time_s, obs.sensor_health)
        CRUISE_SPEED, cruise_reason = self.cruise_cap(obs)
        dist = ctx.dist_to_dest_m

        if ctx.has_arrived:
            base_target_speed = 0.0
        elif dist < max(40.0, (ctx.speed_kmh / 3.6) ** 2 / 3.0 + 2.0 * ctx.speed_kmh / 3.6):
            # Comfortable stopping envelope grows with speed, unlike the old
            # fixed 40m approach which is insufficient at highway speeds.
            stop_offset = .7 if obs.parking else 10.
            base_target_speed = min(CRUISE_SPEED, math.sqrt((2.0 if obs.parking else 3.0) * max(0.0, dist-stop_offset)) * 3.6)
        else:
            base_target_speed = CRUISE_SPEED

        target_speed = 0.0 if ctx.has_arrived else base_target_speed
        if obs.parking:
            # Slow before entering each phase, accounting for jerk-limited
            # actuator response and proportional speed tracking lag.
            for distance, entry_speed in ((obs.parking['distance_to_approach_m'], 10.),
                                          (obs.parking['distance_to_maneuver_m'], 5.)):
                envelope = math.sqrt((entry_speed/3.6)**2 + max(0., distance-20.))*3.6
                target_speed = min(target_speed, envelope)
        junction_cap = self.junctions.speed_cap(obs,
            braking_mps2=cfg.A_MAX_BRAKE_MPS2*max(.1,min(1.,ctx.friction_mu)),
            jerk_mps3=cfg.JERK_MAX_MPS3)
        if junction_cap is not None and junction_cap[0] < target_speed:
            target_speed = junction_cap[0]
            cruise_reason = junction_cap[1]

        self.speed_limit_reason = cruise_reason
        if obs.parking and obs.parking['active']:
            target_speed = min(target_speed, 5. if obs.parking['maneuvering'] else 10.)
            self.speed_limit_reason = obs.parking['status']
        self.friction_speed_derate_active = False

        if ctx.controller == "legacy":
            if health.monitored and not health.ready:
                self.target_speed_mps = 0.
                self.speed_limit_reason = health.reason
                braking = min(2., cfg.A_MAX_BRAKE_MPS2 * max(.1, ctx.friction_mu))
                return LongitudinalDecision(
                    accel_mps2=-braking,
                    speed_kmh_override=max(0., ctx.speed_kmh - braking * ctx.dt_s * 3.6),
                    speed_limit_reason=health.reason,
                )
            if ctx.corner_turn_deg > 0:
                corner_factor = 1.0 - min(ctx.corner_turn_deg, _CORNER_MAX_TURN_DEG) / _CORNER_MAX_TURN_DEG
                corner_speed_cap = _CORNER_MIN_SPEED_KMH + (CRUISE_SPEED - _CORNER_MIN_SPEED_KMH) * corner_factor
                target_speed = min(target_speed, corner_speed_cap)
            speed_diff = target_speed - ctx.speed_kmh
            new_speed = ctx.speed_kmh + speed_diff * ctx.dt_s * 2.0
            new_speed = max(0.0, min(new_speed, 160.0))
            return LongitudinalDecision(
                accel_mps2=0.0,
                speed_kmh_override=new_speed,
                speed_limit_reason=self.speed_limit_reason,
            )

        max_curvature = ctx.max_curvature_1pm
        if max_curvature > 1e-6:
            _mu = max(0.1, min(1.0, ctx.friction_mu))
            eff_a_lat = cfg.A_LAT_MAX_MPS2 * _mu
            v_max_mps = math.sqrt(eff_a_lat / max_curvature)
            curve_cap_kmh = v_max_mps * 3.6
            dry_cap_kmh = math.sqrt(cfg.A_LAT_MAX_MPS2 / max_curvature) * 3.6
            if _mu < 0.99 and curve_cap_kmh < min(target_speed, dry_cap_kmh) - 0.05:
                self.friction_speed_derate_active = True
                self.friction_curve_cap_kmh = curve_cap_kmh
                self.friction_derate_kmh = max(0.0, dry_cap_kmh - curve_cap_kmh)
            if curve_cap_kmh < target_speed:
                target_speed = curve_cap_kmh
                self.speed_limit_reason = "low_friction_curvature" if _mu < 0.99 else "lateral_accel_limit"

        steering_restore_brake=0.
        if steering_demand_rad is not None:
            try:
                if abs(steering_demand_rad)>cfg.MAX_STEER_RAD+1e-9:
                    raise ValueError("steering demand outside geometry")
                demand=steering_demand_constraint(desired_steer_rad=steering_demand_rad,
                    wheelbase_m=cfg.WHEELBASE_M,speed_mps=ctx.speed_kmh/3.6,
                    actual_steer_rad=ctx.steering_angle_rad,lateral_accel_mps2=ctx.last_lateral_accel_mps2,
                    mu=ctx.friction_mu,lateral_limit_mps2=cfg.A_LAT_MAX_MPS2,
                    brake_limit_mps2=cfg.A_MAX_BRAKE_MPS2,steer_rate_radps=cfg.STEER_RATE_MAX_RADPS,dt_s=ctx.dt_s)
            except ValueError:
                self.target_speed_mps=0.
                self.speed_limit_reason="turn_reference_infeasible"
                return LongitudinalDecision(-cfg.A_MAX_BRAKE_MPS2,None,self.speed_limit_reason)
            if demand.speed_cap_mps*3.6<target_speed:
                target_speed=demand.speed_cap_mps*3.6
                steering_restore_brake=demand.restore_brake_mps2
                self.speed_limit_reason="steering_demand_friction_limit"

        turn_constraint = None
        if obs.frenet_frame is not None:
            try:
                cached = getattr(self,"_turn_speed_reference",None)
                if cached is None or cached.frame is not obs.frenet_frame or cached.offset_m != self.lateral_target_d_m:
                    cached = TurnSpeedReference(obs.frenet_frame,self.lateral_target_d_m,cfg.WHEELBASE_M,cfg.MAX_STEER_RAD)
                    self._turn_speed_reference = cached
                turn_constraint = cached.constraint(ctx.station_m,ctx.speed_kmh/3.6,ctx.acceleration_mps2,
                    max(.1,min(1.,ctx.friction_mu)),cfg.A_LAT_MAX_MPS2,cfg.STEER_RATE_MAX_RADPS,
                    braking=min(1.,cfg.A_MAX_BRAKE_MPS2*max(.1,min(1.,ctx.friction_mu))),jerk=cfg.JERK_MAX_MPS3)
            except ValueError:
                self.target_speed_mps = 0.0
                self.speed_limit_reason = "turn_reference_infeasible"
                return LongitudinalDecision(-cfg.A_MAX_BRAKE_MPS2,None,self.speed_limit_reason)
            if turn_constraint.speed_cap_mps*3.6 < target_speed:
                target_speed = turn_constraint.speed_cap_mps*3.6
                self.speed_limit_reason = turn_constraint.reason
        v_mps_for_cap = ctx.speed_kmh / 3.6
        if v_mps_for_cap > 0.5:
            _eff_a_lat_track = cfg.A_LAT_MAX_MPS2 * max(0.1, min(1.0, ctx.friction_mu))
            current_steer_limit = min(
                cfg.MAX_STEER_RAD,
                math.atan(_eff_a_lat_track * cfg.WHEELBASE_M / (v_mps_for_cap ** 2)),
            )
        else:
            current_steer_limit = cfg.MAX_STEER_RAD

        if current_steer_limit > 1e-6:
            steer_fraction = abs(ctx.steering_angle_rad) / current_steer_limit
            if steer_fraction > cfg.TRACKING_ERROR_STEER_FRACTION_THRESHOLD:
                severity = (steer_fraction - cfg.TRACKING_ERROR_STEER_FRACTION_THRESHOLD) / \
                           (1.0 - cfg.TRACKING_ERROR_STEER_FRACTION_THRESHOLD)
                severity = min(1.0, severity)
                tracking_cap_kmh = max(cfg.MIN_CORNER_SPEED_KMH,
                                       CRUISE_SPEED * (1.0 - severity) ** 2)
                if tracking_cap_kmh < target_speed:
                    target_speed = tracking_cap_kmh
                    self.speed_limit_reason = "tracking_correction"

        if self._planner_v_target_mps is not None:
            planner_cap_kmh = self._planner_v_target_mps * 3.6
            if planner_cap_kmh < target_speed - 0.1:
                target_speed = planner_cap_kmh
                if self.speed_limit_reason == "cruise":
                    self.speed_limit_reason = "spatiotemporal_plan"

        if self._joint_plan_unavailable and not ctx.has_arrived:
            target_speed = 0.0
            self.speed_limit_reason = "joint_trajectory_infeasible"

        v_mps = ctx.speed_kmh / 3.6
        target_mps = target_speed / 3.6
        self.target_speed_mps = target_mps
        desired_accel = cfg.SPEED_KP * (target_mps - v_mps)
        if (ctx.has_arrived or self._joint_plan_unavailable or
                (turn_constraint is not None and turn_constraint.geometric_infeasibility
                 and turn_constraint.speed_cap_mps <= .01)):
            # Maintain a bounded brake hold, rather than asymptotically creep
            # toward zero under proportional speed control. The plant retains
            # its normal jerk limiter and clamps reverse motion at standstill.
            desired_accel = min(desired_accel, -min(.2, cfg.A_MAX_BRAKE_MPS2 * ctx.friction_mu))
        if steering_restore_brake>0.:desired_accel=min(desired_accel,-steering_restore_brake)
        if turn_constraint is not None and v_mps >= turn_constraint.speed_cap_mps:
            desired_accel = min(desired_accel,-turn_constraint.braking_mps2)

        lead = obs.lead
        if lead is not None:
            idm_accel = idm_acceleration(
                v_mps=v_mps,
                v0_mps=target_mps,
                gap_m=lead.gap_m,
                lead_speed_mps=lead.lead_speed_mps,
                a_max_mps2=cfg.A_MAX_ACCEL_MPS2,
            )
            if idm_accel is not None:
                desired_accel = min(desired_accel, idm_accel)
                if desired_accel < 0 and self.speed_limit_reason == "cruise":
                    self.speed_limit_reason = "car_following"

        pr = ctx.prediction_result
        if pr is not None and pr.proactive_decel_mps2 > 0.0:
            _proactive_floor_kmh = max(
                0.5 * target_speed,
                target_speed - cfg.PROACTIVE_CUT_IN_MAX_DIP_KMH,
                cfg.MIN_CORNER_SPEED_KMH,
            )
            if ctx.speed_kmh > _proactive_floor_kmh:
                desired_accel = min(desired_accel, -pr.proactive_decel_mps2)
                if self.speed_limit_reason in ("cruise", "car_following"):
                    self.speed_limit_reason = "predictive_cut_in"

        road_heading_deg = ctx.target_heading_deg
        junction_brake = self.junctions.stopping_accel(v_mps)
        if junction_brake is not None:
            desired_accel = min(desired_accel, junction_brake)
        if obs.frenet_frame is not None:
            _, _, tx, tz = frenet_to_local_xz(obs.frenet_frame, ctx.station_m, 0.0)
            road_heading_deg = math.degrees(math.atan2(tx, -tz))
        heading_error_rad = math.radians((ctx.heading_deg - road_heading_deg + 180) % 360 - 180)
        half_width = lateral_half_extent(0., ctx.steering_angle_rad, cfg.WHEELBASE_M)
        motion_heading_error_rad = heading_error_rad + sideslip(ctx.steering_angle_rad)
        footprint_risk = bool(obs.parking and obs.parking['maneuvering'] and not obs.parking['contained'])
        rear_conflict = False
        if obs.frenet_frame is not None:
            ex, ez = latlng_to_local(ctx.ego_lat, ctx.ego_lng,
                                    obs.frenet_frame.origin_lat, obs.frenet_frame.origin_lng)
            yaw = math.radians(ctx.heading_deg)
            # Conservative body-axis rectangle enclosing modeled chassis
            # and all Ackermann tires. The center moves along yaw + beta,
            # whereas the physical footprint remains oriented at body yaw.
            motion_yaw = yaw + sideslip(ctx.steering_angle_rad)
            ego_body = Footprint(ex, ez, yaw, 2*CHASSIS_HALF_LENGTH_M, 2*half_width,
                                 v_mps*math.sin(motion_yaw), -v_mps*math.cos(motion_yaw))
            detections = obs.safety_detections
            if detections is None:
                detections = tuple(t for t in obs.surround_tracks if t.status in ("CONFIRMED", "COASTED"))
            raw_age = obs.safety_measurement_age_s
            age = min(.5, max(0., raw_age)) if math.isfinite(raw_age) else .5
            for track in detections:
                tyaw = (math.atan2(track.vx, -track.vz) if math.hypot(track.vx, track.vz) > 0.2
                        else math.radians(road_heading_deg))
                body = Footprint(track.x + track.vx*age, track.z + track.vz*age, tyaw, track.length_m, track.width_m, track.vx, track.vz)
                if intersects_during(ego_body, body):
                    forward = (body.x-ex)*math.sin(yaw) - (body.z-ez)*math.cos(yaw)
                    if intersects_during(ego_body, body, 0.0) or forward >= 0.0:
                        footprint_risk = True
                        break
                    # Braking for a predicted rear strike increases closing
                    # speed. Report it; this veto-only supervisor cannot
                    # command the acceleration/escape maneuver it may need.
                    rear_conflict = True
        desired_accel, shield_reason = self.safety_monitor.step(
            ego_speed_mps=v_mps,
            heading_error_rad=heading_error_rad,
            footprint_risk=footprint_risk,
            rear_conflict=rear_conflict,
            vehicle_width_m=2*half_width, vehicle_length_m=2*CHASSIS_HALF_LENGTH_M,
            motion_heading_error_rad=motion_heading_error_rad,
            lateral_offset_m=ctx.lateral_offset_m,
            lateral_accel_mps2=ctx.last_lateral_accel_mps2,
            sensed_lead_gap_m=lead.gap_m if lead is not None else None,
            sensed_lead_speed_mps=lead.lead_speed_mps if lead is not None else None,
            desired_accel=desired_accel,
            a_max_brake_mps2=cfg.A_MAX_BRAKE_MPS2,
            recovery_speed_kmh=cfg.MIN_CORNER_SPEED_KMH,
            speed_kp=cfg.SPEED_KP,
        )
        self.shield_verdict = self.safety_monitor.verdict
        if shield_reason is not None:
            self.speed_limit_reason = shield_reason
            self.target_speed_mps = min(self.target_speed_mps,
                0.0 if self.shield_verdict.override_action == safety_shield.OVERRIDE_EMERGENCY_BRAKE
                else cfg.MIN_CORNER_SPEED_KMH / 3.6)

        if health.monitored and not health.ready:
            # Normal jerk-limited controlled stop. An independent emergency
            # override can brake harder and always retains precedence.
            desired_accel = min(desired_accel, -min(2., cfg.A_MAX_BRAKE_MPS2 * max(.1, ctx.friction_mu)))
            self.target_speed_mps = 0.
            if self.shield_verdict.approved:
                self.speed_limit_reason = health.reason

        desired_accel = max(-cfg.A_MAX_BRAKE_MPS2, min(cfg.A_MAX_ACCEL_MPS2, desired_accel))

        return LongitudinalDecision(
            accel_mps2=desired_accel,
            speed_kmh_override=None,
            speed_limit_reason=self.speed_limit_reason,
        )

    # ------------------------------------------------------------------
    # Convenience: the single-call boundary form (world backends / CARLA).
    # ------------------------------------------------------------------
    def decide(self, obs: SensorObservation, ctx: DriverContext) -> ActuatorCommand:
        """Full lateral + longitudinal decision as one ``ActuatorCommand``.

        NOTE: the world still owns integration. This form is what a
        ``WorldBackend`` (CARLA) drives; ``PhysicsEngine`` calls the two
        halves separately so it can keep its arrival latch + driving-state
        classifier interleaved exactly where they were.
        """
        lat = self.decide_lateral(obs, ctx)
        lon = self.decide_longitudinal(obs, ctx)
        source = (
            SOURCE_SAFETY_OVERRIDE
            if self.speed_limit_reason == "safety_shield_override" or self.speed_limit_reason.startswith("sensor_")
            else SOURCE_PLANNER
        )
        return ActuatorCommand(
            steer_rad=lat.steering_angle_rad,
            accel_mps2=lon.accel_mps2,
            source=source,
            speed_limit_reason=lon.speed_limit_reason,
        )

    def control(self, obs: SensorObservation, ctx: DriverContext) -> ActuatorCommand:
        """Track the committed lateral target; never re-run the planner here."""
        cfg = self.cfg
        health = self.sensor_guard.evaluate(obs.clock.sim_time_s, obs.sensor_health)
        if health.monitored and not health.ready:
            self._hold_lateral_during_sensor_loss(ctx)
            self._maneuver_traj = None
            self.spatiotemporal_traj = None
            self._planner_v_target_mps = None
            self._changing_lane_now = False
        else:
            self._sensor_loss_lateral_hold = False
        v = obs.ego.speed_mps
        if health.monitored and not health.ready and v < .05:
            # At standstill there is no road motion to track. Camera-loss
            # lane-reference retention must not wind the wheels toward an
            # incomplete lane centre and invalidate the next checked launch.
            lon=self.decide_longitudinal(obs,ctx)
            return ActuatorCommand(steer_rad=ctx.steering_angle_rad,
                accel_mps2=lon.accel_mps2,source=SOURCE_SAFETY_OVERRIDE,
                speed_limit_reason=lon.speed_limit_reason)
        limit = min(cfg.MAX_STEER_RAD, math.atan(cfg.A_LAT_MAX_MPS2 * ctx.friction_mu * cfg.WHEELBASE_M / max(v*v, .01)))
        desired = 0.0
        if obs.frenet_frame is not None:
            frame = obs.frenet_frame
            # Physical front axle plus bounded actuator preview, with
            # center-bicycle feedforward and measured sideslip correction.
            x, z = latlng_to_local(ctx.ego_lat, ctx.ego_lng, frame.origin_lat, frame.origin_lng)
            yaw = math.radians(ctx.heading_deg)
            preview_m = min(0.7, 0.2 * v)
            preview_x, preview_z, future_yaw = predict_front_axle(
                x,z,yaw,v,ctx.steering_angle_rad,cfg.WHEELBASE_M,preview_m)
            lat, lng = local_to_latlng(preview_x,preview_z,frame.origin_lat,frame.origin_lng)
            index = max(0, bisect_left(frame.station, ctx.station_m) - 4)
            preview_s, preview_d, _ = project_to_frenet(frame, lat, lng, index, 16)
            try:
                curvature = offset_path_curvature(raw_route_curvature(frame, preview_s), self.lateral_target_d_m)
                desired = center_tracking_steer(
                    curvature_1pm=curvature, wheelbase_m=cfg.WHEELBASE_M,
                    heading_error_rad=math.radians(continuous_path_heading(frame, preview_s, self.lateral_target_d_m,half_window_m=TURN_REFERENCE_HALF_WINDOW_M) - math.degrees(future_yaw))-front_axle_heading_advance(curvature,cfg.WHEELBASE_M),
                    cross_track_m=preview_d-self.lateral_target_d_m-front_axle_lateral_bias(curvature,cfg.WHEELBASE_M),
                    speed_mps=v, steer_limit_rad=cfg.MAX_STEER_RAD, actual_steer_rad=ctx.steering_angle_rad)
            except ValueError:
                self.speed_limit_reason = "turn_reference_infeasible"
                self.target_speed_mps = 0.0
                safe_steer = ctx.steering_angle_rad if math.isfinite(ctx.steering_angle_rad) else 0.0
                safe_steer = max(-limit, min(limit, safe_steer))
                return ActuatorCommand(steer_rad=safe_steer,
                    accel_mps2=-cfg.A_MAX_BRAKE_MPS2, source=SOURCE_SAFETY_OVERRIDE,
                    speed_limit_reason="turn_reference_infeasible")
        else:
            diff = (ctx.target_heading_deg - ctx.heading_deg + 180) % 360 - 180
            desired = max(-limit, min(limit, cfg.STEER_KP * math.radians(diff)))
        max_step = cfg.STEER_RATE_MAX_RADPS * ctx.dt_s
        steer = ctx.steering_angle_rad + max(-max_step, min(max_step, desired-ctx.steering_angle_rad))
        steer = max(-limit, min(limit, steer))
        lon = self.decide_longitudinal(obs, ctx,steering_demand_rad=desired)
        return ActuatorCommand(steer_rad=steer, accel_mps2=lon.accel_mps2,
            source=SOURCE_SAFETY_OVERRIDE if (not self.shield_verdict.approved or lon.speed_limit_reason=="turn_reference_infeasible" or lon.speed_limit_reason.startswith("sensor_")) else SOURCE_PLANNER,
            speed_limit_reason=lon.speed_limit_reason)
