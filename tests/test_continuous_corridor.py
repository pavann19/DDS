"""Modeled polynomial corridor proofs and initial dynamic actuator checks."""
import math
from dataclasses import replace
import pytest
from app.services.planner.polynomials import QuinticPolynomial,QuarticPolynomial
from app.services.planner.spatiotemporal import PlannerConfig,PlannerContext,PlannerStart,plan,_feasible,revalidate_trajectory_remaining
from app.services.planner.corridor_proof import polynomial_within,lateral_corridor_within,lateral_corridor_batch,longitudinal_corridor_within
from app.services.planner.actuator_reachability import initial_actuator_reachable,required_steering_rate


def test_continuous_proof_detects_between_sample_corridor_excursion():
    base=QuinticPolynomial(0.,1.,.4,0.,0.,0.,2.)
    sampled=max(base.pos(2*i/9) for i in range(10))
    offset=6.5-sampled-1e-5
    lat=QuinticPolynomial(offset,1.,.4,offset,0.,0.,2.)
    assert max(lat.pos(2*i/9) for i in range(10))<6.5
    assert max(lat.pos(2*i/10000) for i in range(10001))>6.5
    assert not polynomial_within(lat,-6.5,6.5)


def test_continuous_body_half_width_margin_rejects_outer_lattice_but_retains_adjacent_lane():
    cfg=PlannerConfig()
    outer=QuinticPolynomial(1.75,0.,0.,5.85,0.,0.,6.)
    lane=QuinticPolynomial(1.75,0.,0.,5.25,0.,0.,6.)
    assert not lateral_corridor_within(outer,5.85,cfg)
    assert lateral_corridor_within(lane,5.25,cfg)


@pytest.mark.parametrize('seed',range(8))
def test_batched_interval_proof_matches_scalar_subdivision(seed):
    import random
    rng=random.Random(seed)
    laterals=[]
    for _ in range(20):
        d0=rng.uniform(-5.,5.);d1=rng.uniform(-6.,6.);T=rng.choice([2.,4.,6.])
        p=QuinticPolynomial(d0,rng.uniform(-1.,1.),rng.uniform(-1.,1.),d1,0.,0.,T)
        laterals.append((p,T,d1,0.))
    batch=lateral_corridor_batch(laterals,PlannerConfig())
    for p,_,d1,_ in laterals:assert batch[id(p)]==lateral_corridor_within(p,d1,PlannerConfig())


def test_proof_rejects_invalid_coefficients_and_holds_longitudinal_terminal_state():
    lon=QuarticPolynomial(0.,5.,0.,10.,0.,2.)
    assert longitudinal_corridor_within(lon,6.,100.,PlannerConfig())
    assert not longitudinal_corridor_within(lon,6.,40.,PlannerConfig())
    assert not polynomial_within(lon,0.,100.,start=float('nan'))
    assert not polynomial_within(lon,0.,100.,max_depth=-1)


def test_straight_reference_cannot_instantly_replace_actual_steering():
    lat=QuinticPolynomial(0.,0.,0.,0.,0.,0.,2.)
    lon=QuarticPolynomial(0.,10.,0.,10.,0.,2.)
    ctx=PlannerContext(10.,0.,10.,.1,actual_steer_rad=.3)
    assert not initial_actuator_reachable(lat,lon,ctx,PlannerConfig())
    assert _feasible(lat,lon,0.,2.,20,PlannerConfig(),context=ctx) is None
    assert initial_actuator_reachable(lat,lon,replace(ctx,actual_steer_rad=0.),PlannerConfig())


def test_dynamic_beta_rate_accepts_feasible_transient_rejected_by_steady_angle_jump():
    delta=.1;desired_rate=.3;v=4.;L=2.8
    beta=math.atan(.5*math.tan(delta))
    beta_prime=.5/(math.cos(delta)**2*(1+.25*math.tan(delta)**2))
    curvature=(v*math.cos(beta)*math.tan(delta)/L+beta_prime*desired_rate)/v
    assert required_steering_rate(delta,v,curvature,L)==pytest.approx(desired_rate)
    lat=QuinticPolynomial(0.,0.,-curvature*v*v,-.5,0.,0.,6.)
    lon=QuarticPolynomial(0.,v,0.,v,0.,6.)
    ctx=PlannerContext(10.,0.,10.,.1,actual_steer_rad=delta)
    assert initial_actuator_reachable(lat,lon,ctx,PlannerConfig())


def test_stopped_vehicle_may_hold_steering_without_yaw_impulse():
    lat=QuinticPolynomial(0.,0.,0.,0.,0.,0.,2.)
    lon=QuarticPolynomial(0.,0.,0.,0.,0.,2.)
    ctx=PlannerContext(10.,0.,10.,.1,actual_steer_rad=.3)
    assert initial_actuator_reachable(lat,lon,ctx,PlannerConfig())


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-.9])
def test_invalid_actual_steering_fails_closed(bad):
    ctx=PlannerContext(10.,0.,10.,.1,actual_steer_rad=bad)
    assert plan(PlannerStart(0.,10.,0.,0.,0.,0.),ctx) is None


def test_revalidation_checks_current_actual_steering_and_exposes_honest_scope():
    ctx=PlannerContext(10.,1.75,10.,.1,actual_steer_rad=0.)
    traj=plan(PlannerStart(0.,10.,0.,1.75,0.,0.),ctx)
    assert traj is not None
    assert traj.continuous_modeled_corridor_verified
    assert traj.initial_actual_steering_checked
    assert not traj.full_cartesian_swept_corridor_verified
    assert revalidate_trajectory_remaining(traj,.2,ctx)
    assert not revalidate_trajectory_remaining(traj,.2,replace(ctx,actual_steer_rad=.3))

@pytest.mark.parametrize('steer',[0.,math.radians(15.),-math.radians(15.),math.radians(35.),-math.radians(35.)])
def test_nominal_chassis_and_tire_envelope_covers_rendered_planar_points(steer):
    from app.services.vehicle_geometry import lateral_half_extent,lateral_half_extent_array,MIN_LATERAL_HALF_EXTENT_M
    from app.services.vehicle_kinematics import wheel_geometry
    import numpy as np
    angles=wheel_geometry(steer,2.8)[0]
    points=[(x,y) for x in (-.99,.99) for y in (-2.4,2.4)]
    for (x,y),angle in zip([(-.98,1.4),(.98,1.4),(-.98,-1.4),(.98,-1.4)],angles):
        for w in (-.11,.11):
            for r in (-.34,.34):points.append((x+w*math.cos(angle)+r*math.sin(angle),y-w*math.sin(angle)+r*math.cos(angle)))
    rectangle_half_width=lateral_half_extent(0.,steer)
    assert all(abs(x)<=rectangle_half_width+1e-12 and abs(y)<=2.4 for x,y in points)
    for yaw in (-1.5,-.7,-.1,0.,.1,.7,1.5):
        extent=lateral_half_extent(yaw,steer)
        assert extent>=MIN_LATERAL_HALF_EXTENT_M-1e-12
        assert extent==pytest.approx(float(lateral_half_extent_array(np.array(yaw),np.array(steer))))
        assert all(abs(x*math.cos(yaw)+y*math.sin(yaw))<=extent+1e-12 for x,y in points)


def test_vehicle_width_matches_straight_tire_envelope_and_widens_with_steering():
    from app.services.vehicle_geometry import lateral_half_extent
    assert 2*lateral_half_extent(0.,0.)==pytest.approx(2.18)
    assert 2*lateral_half_extent(0.,math.radians(35.))>2.5

def test_initial_actuator_check_includes_rising_rate_at_final_endpoint():
    from scipy.integrate import solve_ivp
    # Independent high-accuracy integration demonstrates a rate rise only after
    # the old last midpoint (90 ms); the final 100 ms sample must reject it.
    lat=QuinticPolynomial(0.,0.,-.1475,-1.1277,0.,0.,6.)
    lon=QuarticPolynomial(0.,.5,0.,.5,0.,6.)
    def physical_derivative(t,state):
        dv,da=lat.vel(t),lat.acc(t)
        speed=math.hypot(.5,dv)
        motion_curvature=-.5*da/(speed**3)
        return [required_steering_rate(state[0],speed,motion_curvature,2.8)]
    solution=solve_ivp(physical_derivative,[0.,.1],[0.],rtol=1e-10,atol=1e-12,dense_output=True)
    assert physical_derivative(.09,solution.sol(.09))[0]<.6
    assert physical_derivative(.1,solution.sol(.1))[0]>.60001
    ctx=PlannerContext(10.,0.,10.,.1,actual_steer_rad=0.)
    assert not initial_actuator_reachable(lat,lon,ctx,PlannerConfig())
