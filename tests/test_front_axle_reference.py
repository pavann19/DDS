"""Physical-geometry acceptance; synthetic motion is explicitly a test fixture."""
import math
import pytest
from app.services.driver.turn_reference import front_axle_lateral_bias

@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('radius',[4.5,8.,12.,25.,60.])
def test_physical_front_axle_radius_matches_independent_position(direction,radius):
    k=direction/radius
    a=1.4
    beta=math.asin(a*k)
    # Reference center is at origin with motion tangent north.
    front=(a*math.sin(-beta),-a*math.cos(-beta))
    measured=math.hypot(front[0]-1/k,front[1])-radius
    assert front_axle_lateral_bias(k,2.8)==pytest.approx(math.copysign(measured,k),abs=1e-12)

@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
@pytest.mark.parametrize('field',[0,1])
def test_nonfinite_geometry_fails_closed(bad,field):
    args=[.1,2.8];args[field]=bad
    with pytest.raises(ValueError):front_axle_lateral_bias(*args)

@pytest.mark.parametrize('args',[(.1,0.),(.1,-2.8),(1.,2.8)])
def test_unreachable_geometry_rejected(args):
    with pytest.raises(ValueError):front_axle_lateral_bias(*args)


def test_straight_reference_is_unbiased():
    assert front_axle_lateral_bias(0.,2.8)==0.

@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('radius',[8.,12.,25.])
@pytest.mark.parametrize('speed_mu',[(1.5,.3),(2.5,.3),(4.,1.)])
def test_actual_world_front_reference_improves_be37_baseline(direction,radius,speed_mu,monkeypatch):
    from tests.test_turn_reference import _drive_circle
    from app.services.driver import driver as module
    corrected=_drive_circle(direction,radius,speed_mu)
    with monkeypatch.context() as patch:
        patch.setattr(module,'front_axle_lateral_bias',lambda *args:0.)
        patch.setattr(module,'front_axle_heading_advance',lambda *args:0.)
        from app.services.driver.turn_tracking import continuous_path_heading as old_heading
        patch.setattr(module,'continuous_path_heading',lambda frame,station,offset,**kwargs:old_heading(frame,station,offset,half_window_m=1.))
        from app.services.driver.turn_reference import SteeringDemandConstraint
        patch.setattr(module,'steering_demand_constraint',lambda **kwargs:SteeringDemandConstraint(float('inf'),0.,0.))
        baseline=_drive_circle(direction,radius,speed_mu)
    assert corrected<baseline
    if radius==25.:
        assert corrected<.5


@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('radius',[4.5,8.,12.,25.])
def test_front_heading_advance_matches_rigid_body_front_velocity(direction,radius):
    from app.services.driver.turn_reference import front_axle_heading_advance
    k=direction/radius
    speed=3.;theta=.4;a=1.4
    beta=math.asin(a*k);body=theta-beta
    center_velocity=(speed*math.sin(theta),-speed*math.cos(theta))
    front_velocity=(center_velocity[0]+speed*k*a*math.cos(body),center_velocity[1]+speed*k*a*math.sin(body))
    measured=math.atan2(front_velocity[0],-front_velocity[1])-theta
    assert front_axle_heading_advance(k,2.8)==pytest.approx(measured,abs=1e-12)


def demand(**updates):
    from app.services.driver.turn_reference import steering_demand_constraint
    values=dict(desired_steer_rad=.6,wheelbase_m=2.8,speed_mps=10.,actual_steer_rad=.1,
        lateral_accel_mps2=.2,mu=.3,lateral_limit_mps2=3.,brake_limit_mps2=4.5,steer_rate_radps=.6,dt_s=.02)
    values.update(updates)
    return steering_demand_constraint(**values)

@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('mu',[.1,.3,1.])
@pytest.mark.parametrize('steer',[.05,.2,.6])
def test_demand_speed_cap_preserves_both_physical_and_existing_actuator_grip(direction,mu,steer):
    d=demand(desired_steer_rad=direction*steer,mu=mu)
    physical=d.speed_cap_mps**2*d.center_curvature_1pm
    assert physical<=3.*mu+1e-12
    actuator_limit=math.atan(3.*mu*2.8/d.speed_cap_mps**2)
    assert steer<=actuator_limit+1e-12

@pytest.mark.parametrize('mu',[.1,.3,1.])
def test_restoration_contribution_respects_available_modeled_ellipse(mu):
    lat=3.*mu*.7
    d=demand(mu=mu,lateral_accel_mps2=lat)
    assert 0.<d.restore_brake_mps2<=1.
    assert (lat/(3.*mu))**2+(d.restore_brake_mps2/(4.5*mu))**2<=1.+1e-12


def test_saturated_lateral_budget_provides_no_additional_restoration_brake():
    # This does not certify/clamp the aggregate PID/IDM/safety acceleration.
    assert demand(lateral_accel_mps2=.9).restore_brake_mps2==0.


def test_straight_cruise_and_already_feasible_turn_need_no_restore_braking():
    assert math.isinf(demand(desired_steer_rad=0.).speed_cap_mps)
    assert demand(desired_steer_rad=0.).restore_brake_mps2==0.
    assert demand(speed_mps=.5).restore_brake_mps2==0.

@pytest.mark.parametrize('field',['desired_steer_rad','speed_mps','lateral_accel_mps2','mu','dt_s'])
@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_nonfinite_demand_readings_are_rejected(field,bad):
    with pytest.raises(ValueError):demand(**{field:bad})
