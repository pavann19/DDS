"""Synthetic isolated fixtures; not claims of real-road corridor feasibility."""
import math
from dataclasses import replace
import pytest
from app.services.frenet import FrenetFrame
from app.services.planner.joint_physics import joint_lateral_acceleration, reference_curvature
from app.services.planner.spatiotemporal import _feasible, _lat_state, _lon_state, PlannerConfig, PlannerContext
from app.services.planner.polynomials import QuinticPolynomial, QuarticPolynomial


def circle_frame(radius=100.,direction=1.):
    # Begin at north; compass-right circle travels east.
    angles=[i*.002 for i in range(800)]
    pts=[(direction*radius*math.sin(a),-radius*math.cos(a)) for a in angles]
    station=[0.]
    for p,q in zip(pts,pts[1:]):station.append(station[-1]+math.hypot(q[0]-p[0],q[1]-p[1]))
    return FrenetFrame(points_xz=pts,station=station,origin_lat=37.,origin_lng=-122.)


@pytest.mark.parametrize('direction',[-1.,1.])
def test_constant_parallel_path_matches_cartesian_circle(direction):
    k=direction/100
    d=2.
    sd=10.
    speed=(1+k*d)*sd
    path_k=k/(1+k*d)
    assert joint_lateral_acceleration(k,0.,d,0.,0.,sd,0.)==pytest.approx(-speed*speed*path_k)


def test_reinforcement_and_cancellation_use_signed_total_demand():
    assert joint_lateral_acceleration(.01,0.,0.,0.,-1.4,10.,0.)==pytest.approx(-2.4)
    assert joint_lateral_acceleration(.01,0.,0.,0.,1.4,10.,0.)==pytest.approx(.4)


def test_lateral_velocity_and_longitudinal_acceleration_project_into_actual_velocity_normal():
    assert joint_lateral_acceleration(0.,0.,0.,2.,1.,10.,2.)==pytest.approx(6/math.sqrt(104))


def test_folded_reverse_and_nonfinite_joint_states_rejected():
    with pytest.raises(ValueError):joint_lateral_acceleration(-.1,0.,10.,0.,0.,2.,0.)
    with pytest.raises(ValueError):joint_lateral_acceleration(0.,0.,0.,0.,0.,-2.,0.)
    with pytest.raises(ValueError):joint_lateral_acceleration(0.,0.,float('nan'),0.,0.,2.,0.)


def test_joint_filter_rejects_two_individually_legal_acceleration_demands():
    lat=QuinticPolynomial(0.,0.,-1.4,-1.,0.,0.,6.)
    lon=QuarticPolynomial(30.,10.,0.,10.,0.,6.)
    cfg=replace(PlannerConfig(),max_lat_jerk_mps3=100.)
    ctx=PlannerContext(10.,0.,10.,.1,frenet_frame=circle_frame())
    assert _feasible(lat,lon,-1.,.2,9,cfg) is not None
    assert _feasible(lat,lon,-1.,.2,9,cfg,context=ctx) is None


def test_full_horizon_rejects_violation_after_shorter_polynomial_finishes():
    lat=QuinticPolynomial(0.,0.,0.,3.5,0.,0.,6.)
    lon=QuarticPolynomial(0.,5.,0.,5.,0.,.3)
    cfg=replace(PlannerConfig(),max_lat_accel_mps2=.3,max_lat_jerk_mps3=100.)
    assert _feasible(lat,lon,3.5,.3,9,cfg) is not None
    assert _feasible(lat,lon,3.5,6.,50,cfg,t_lat_s=6.,t_lon_s=.3) is None


def test_terminal_states_hold_without_polynomial_extrapolation():
    lat=QuinticPolynomial(0.,0.,0.,3.5,0.,0.,2.)
    lon=QuarticPolynomial(0.,5.,0.,10.,0.,2.)
    assert _lat_state(lat,6.,2.)==pytest.approx((3.5,0.,0.,0.))
    assert _lon_state(lon,6.,2.)==pytest.approx((lon.pos(2.)+40,10.,0.))


def test_raw_window_curvature_does_not_hide_tight_turns():
    assert reference_curvature(circle_frame(4.),3.)>.125


def test_remaining_committed_plan_rejects_friction_drop():
    from app.services.planner.spatiotemporal import plan, PlannerStart, revalidate_trajectory_remaining
    cfg=replace(PlannerConfig(),d_lattice_offsets_m=(0.,),t_lat_options_s=(6.,),
                t_lon_options_s=(6.,),v_lattice_frac=(1.,))
    ctx=PlannerContext(10.,0.,10.,.1,frenet_frame=circle_frame())
    traj=plan(PlannerStart(30.,10.,0.,0.,0.,0.),ctx,cfg)
    assert traj is not None
    assert traj.checked_horizon_s==6.
    assert traj.peak_reference_curvature_1pm>0
    assert revalidate_trajectory_remaining(traj,.2,ctx,cfg)
    assert not revalidate_trajectory_remaining(traj,.2,replace(ctx,friction_mu=.2),cfg)


def test_cartesian_long_acceleration_accounts_for_curvature_and_lateral_velocity():
    from app.services.planner.joint_physics import joint_acceleration_components
    longitudinal,lateral,speed=joint_acceleration_components(.1,0.,0.,2.,0.,5.,0.)
    assert longitudinal==pytest.approx((5*2 + 2*(-2.5))/math.sqrt(29))
    assert lateral==pytest.approx((5*(-2.5)-2*2)/math.sqrt(29))
    assert speed==pytest.approx(math.sqrt(29))

def test_mapped_planning_physics_sampling_bounds_time_and_station_spacing():
    from app.services.planner.spatiotemporal import plan,PlannerStart
    cfg=replace(PlannerConfig(),d_lattice_offsets_m=(0.,),t_lat_options_s=(6.,),
                t_lon_options_s=(6.,),v_lattice_frac=(1.,))
    ctx=PlannerContext(10.,0.,10.,.1,frenet_frame=circle_frame())
    traj=plan(PlannerStart(30.,10.,0.,0.,0.,0.),ctx,cfg)
    assert traj is not None
    assert max(b[0]-a[0] for a,b in zip(traj.s_samples,traj.s_samples[1:])) <= .10000001
    assert max(b[1]-a[1] for a,b in zip(traj.s_samples,traj.s_samples[1:])) <= 2.0000001

@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('mu,speed',[(1.,14.),(.5,9.8)])
def test_selected_curve_lane_change_stays_within_combined_sampled_grip_budget(direction,mu,speed):
    from app.services.planner.spatiotemporal import plan,PlannerStart
    from app.services.planner.joint_physics import reference_curvature_and_derivative
    ctx=PlannerContext(speed,direction*5.25,speed,.1,frenet_frame=circle_frame(direction=direction),
                       allowed_lane_centers_m=[direction*5.25],friction_mu=mu)
    traj=plan(PlannerStart(30.,speed/1.0175,0.,direction*1.75,0.,0.),ctx)
    assert traj is not None
    assert traj.v1_mps<speed
    for i in range(1001):
        t=traj.checked_horizon_s*i/1000
        d,dv,da,_=_lat_state(traj.lat_poly,t,traj.t_lat_s)
        s,sv,sa=_lon_state(traj.lon_poly,t,traj.t_lon_s)
        k,ks=reference_curvature_and_derivative(ctx.frenet_frame,s)
        demand=abs(joint_lateral_acceleration(k,ks,d,dv,da,sv,sa))
        assert demand <= PlannerConfig().max_lat_accel_mps2*mu+1e-6

def test_candidate_exiting_recorded_route_is_rejected_but_endpoint_rest_is_valid():
    frame=FrenetFrame(points_xz=[(0.,0.),(0.,-20.)],station=[0.,20.],origin_lat=37.,origin_lng=-122.)
    ctx=PlannerContext(5.,0.,5.,.1,frenet_frame=frame)
    lat=QuinticPolynomial(0.,0.,0.,0.,0.,0.,2.)
    exiting=QuarticPolynomial(19.,5.,0.,5.,0.,2.)
    stationary=QuarticPolynomial(20.,0.,0.,0.,0.,2.)
    assert _feasible(lat,exiting,0.,2.,20,PlannerConfig(),context=ctx) is None
    assert _feasible(lat,stationary,0.,2.,20,PlannerConfig(),context=ctx) is not None


def test_exact_velocity_extrema_detect_reverse_motion_between_samples():
    from app.services.planner.spatiotemporal import _longitudinal_velocity_extrema
    # v(t)=1-A*t*(1-t)^2 has its minimum exactly at t=1/3,
    # between the 8-interval sample stations. All samples remain positive.
    A=6.751
    lon=QuarticPolynomial(0.,1.,-A,1.,0.,1.)
    sampled=[lon.vel(i/8) for i in range(9)]
    assert min(sampled)>0
    assert _longitudinal_velocity_extrema(lon,1.)[0]<0
    lat=QuinticPolynomial(0.,0.,0.,0.,0.,0.,1.)
    cfg=replace(PlannerConfig(),min_long_accel_mps2=-10.,max_long_accel_mps2=10.)
    assert _feasible(lat,lon,0.,1.,8,cfg) is None

def test_public_trajectory_sampler_holds_shorter_polynomial_terminal_state():
    from types import SimpleNamespace
    from app.services.planner.spatiotemporal import sample_trajectory_state
    lat=QuinticPolynomial(0.,0.,0.,3.5,0.,0.,6.)
    lon=QuarticPolynomial(0.,5.,0.,10.,0.,2.)
    traj=SimpleNamespace(lat_poly=lat,lon_poly=lon,t_lat_s=6.,t_lon_s=2.)
    s,d,sd,dd=sample_trajectory_state(traj,4.)
    assert s==pytest.approx(lon.pos(2.)+20.)
    assert sd==pytest.approx(10.)
    assert d==pytest.approx(lat.pos(4.))
    assert dd==pytest.approx(lat.vel(4.))
    with pytest.raises(ValueError):sample_trajectory_state(traj,float('nan'))

def test_physical_speed_cap_counts_parallel_path_metric_and_lateral_motion():
    lat=QuinticPolynomial(2.,0.,0.,2.,0.,0.,2.)
    lon=QuarticPolynomial(30.,10.,0.,10.,0.,2.)
    ctx=PlannerContext(10.,2.,10.,.1,frenet_frame=circle_frame())
    assert _feasible(lat,lon,2.,2.,30,PlannerConfig(),context=ctx) is None
    # Persistent overspeed is rejected; a recovery may never stay above the cap at the terminal time.
    straight=replace(ctx,frenet_frame=None)
    moving_lat=QuinticPolynomial(0.,1.,0.,1.,0.,0.,6.)
    assert _feasible(moving_lat,lon,1.,6.,60,PlannerConfig(),context=straight,t_lat_s=6.,t_lon_s=2.) is None

def test_lateral_maneuver_cannot_continue_after_longitudinal_stop():
    lat=QuinticPolynomial(1.75,.45,.64,2.05,0.,0.,6.)
    lon=QuarticPolynomial(0.,10.,-4.25,0.,0.,4.)
    ctx=PlannerContext(14.,1.75,14.,.1)
    assert _feasible(lat,lon,2.05,6.,60,PlannerConfig(),context=ctx,t_lat_s=6.,t_lon_s=4.) is None


def test_stopped_car_does_not_gain_sideways_motion_from_a_polynomial():
    lat=QuinticPolynomial(0.,0.,0.,3.5,0.,0.,6.)
    lon=QuarticPolynomial(0.,0.,0.,0.,0.,2.)
    ctx=PlannerContext(10.,3.5,10.,.1)
    assert _feasible(lat,lon,3.5,6.,60,PlannerConfig(),context=ctx,t_lat_s=6.,t_lon_s=2.) is None


def test_viable_moving_abort_exists_without_relaxing_acceleration_or_jerk():
    from app.services.planner.spatiotemporal import plan,PlannerStart
    ctx=PlannerContext(14.,1.75,14.,.1,allowed_lane_centers_m=[1.75])
    traj=plan(PlannerStart(0.,10.,-4.25,2.25,.45,.64),ctx)
    assert traj is not None
    assert traj.v1_mps>0
    assert traj.peak_lat_jerk_mps3<=PlannerConfig().max_lat_jerk_mps3

@pytest.mark.parametrize('seed',range(12))
def test_straight_batch_matches_scalar_hard_constraints(seed):
    import random
    from app.services.planner.flat_physics_batch import flat_feasible_lattice
    from app.services.planner.spatiotemporal import _longitudinal_velocity_extrema
    rng=random.Random(seed)
    d0=rng.uniform(-1.,1.);dd0=rng.uniform(-.4,.4);da0=rng.uniform(-.3,.3)
    v=rng.uniform(4.,18.);a=rng.uniform(-1.,1.)
    ctx=PlannerContext(10.,0.,rng.choice([8.,14.,20.]),.1,friction_mu=rng.choice([.2,.5,1.]))
    cfg=PlannerConfig()
    laterals=[(QuinticPolynomial(d0,dd0,da0,d1,0.,0.,T),T,d1,0.) for d1 in (-.3,0.,.3) for T in (3.,6.)]
    longs=[(QuarticPolynomial(0.,v,a,v1,0.,T),T,v1) for v1 in (0.,5.,min(10.,ctx.max_speed_mps)) for T in (2.,4.)]
    mu=ctx.friction_mu
    lat_cap=cfg.max_lat_accel_mps2*mu;jerk_cap=cfg.max_lat_jerk_mps3*math.sqrt(mu)
    result=flat_feasible_lattice(laterals,longs,ctx,cfg,lat_cap,jerk_cap,
        {id(p):_longitudinal_velocity_extrema(p,T) for p,T,_ in longs})
    for lat,Tlat,d1,_ in laterals:
        for lon,Tlon,_ in longs:
            scalar=_feasible(lat,lon,d1,max(Tlat,Tlon),cfg.sample_n,cfg,
                max_lat_accel=lat_cap,max_lat_jerk=jerk_cap,context=ctx,t_lat_s=Tlat,t_lon_s=Tlon)
            batch=result[(id(lat),id(lon))]
            assert (batch is None)==(scalar is None)
            if scalar is not None:assert batch==pytest.approx(scalar,abs=1e-8)


def test_overspeed_recovery_is_explicit_and_monotone_until_terminal_cap():
    from app.services.planner.spatiotemporal import plan,PlannerStart
    ctx=PlannerContext(13.9,0.,8.,.1)
    traj=plan(PlannerStart(0.,13.9,0.,0.,0.,0.),ctx)
    assert traj is not None and traj.initial_speed_recovery
    assert traj.v1_mps<=8.
    speeds=[_lon_state(traj.lon_poly,traj.t_lon_s*i/100,traj.t_lon_s)[1] for i in range(101)]
    assert all(b<=a+1e-9 for a,b in zip(speeds,speeds[1:]))

def test_reference_steering_angle_rejects_acceleration_safe_too_tight_curve():
    lat=QuinticPolynomial(0.,0.,0.,0.,0.,0.,1.)
    lon=QuarticPolynomial(2.,1.,0.,1.,0.,1.)
    ctx=PlannerContext(2.,0.,2.,.1,frenet_frame=circle_frame(4.))
    assert _feasible(lat,lon,0.,1.,20,PlannerConfig(),context=ctx) is None


def test_reference_steering_rate_is_a_separate_hard_constraint():
    lat=QuinticPolynomial(0.,0.,0.,.05,0.,0.,1.)
    lon=QuarticPolynomial(0.,1.,0.,1.,0.,1.)
    ctx=PlannerContext(10.,.05,10.,.1)
    cfg=replace(PlannerConfig(),max_lat_jerk_mps3=100.,max_steer_rad=math.radians(89.),max_steer_rate_radps=100.)
    assert _feasible(lat,lon,.05,1.,100,cfg,context=ctx) is not None
    assert _feasible(lat,lon,.05,1.,100,replace(cfg,max_steer_rate_radps=.01),context=ctx) is None
