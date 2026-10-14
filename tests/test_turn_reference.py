"""Synthetic fixtures are explicitly isolated from the real-road mission."""
import math
import numpy as np
import pytest
from app.services.frenet import FrenetFrame,signed_curvature_at
from app.services.driver.turn_reference import TurnSpeedReference,raw_route_curvature,predict_front_axle


def corner(angle=math.pi/2):
    return FrenetFrame([(0.,0.),(0.,-50.),(50*math.sin(angle),-50-50*math.cos(angle))],[0.,50.,100.],37.,-122.)


def circle(radius=25.,direction=1.):
    points=[(direction*radius*math.sin(i/radius),-radius*math.cos(i/radius)) for i in range(401)]
    station=[0.]
    for a,b in zip(points,points[1:]):station.append(station[-1]+math.dist(a,b))
    return FrenetFrame(points,station,37.,-122.)

@pytest.mark.parametrize('direction',[-1.,1.])
def test_raw_turn_demand_is_not_silently_radius_clipped(direction):
    frame=corner(direction*2*math.pi/3)
    k=raw_route_curvature(frame,50.)
    assert k==pytest.approx(direction*math.pi/18)
    assert abs(k)>abs(signed_curvature_at(frame,50.))
    reference=TurnSpeedReference(frame,0.,2.8,.61)
    assert reference.curvature[50]==pytest.approx(k)

@pytest.mark.parametrize('direction',[-1.,1.])
def test_mirrored_profile_preserves_geometry_and_constraints(direction):
    frame=corner(direction*math.pi/2)
    before=frame.points_xz
    reference=TurnSpeedReference(frame,0.,2.8,.61)
    other=TurnSpeedReference(corner(-direction*math.pi/2),0.,2.8,.61)
    assert reference.curvature==pytest.approx(-other.curvature)
    assert reference.steer==pytest.approx(-other.steer)
    assert reference.steer_gradient==pytest.approx(other.steer_gradient)
    assert frame.points_xz==before
    assert reference.stations[0]==0.
    assert reference.stations[-1]==frame.total_length_m
    with pytest.raises(ValueError):reference.steer[0]=1.

@pytest.mark.parametrize('direction',[-1.,1.])
def test_parallel_offset_uses_existing_left_positive_convention(direction):
    reference=TurnSpeedReference(corner(direction*math.pi/2),1.75,2.8,.61)
    k=direction*math.pi/24
    assert reference.curvature[50]==pytest.approx(k/(1+k*1.75))


def test_wet_constant_curve_envelope_matches_friction_scaling():
    reference=TurnSpeedReference(circle(),0.,2.8,.61)
    dry=reference.constraint(150.,2.,0.,1.,3.,.6)
    wet=reference.constraint(150.,2.,0.,.3,3.,.6)
    assert wet.speed_cap_mps/dry.speed_cap_mps==pytest.approx(math.sqrt(.3),rel=1e-10)


def test_turn_steering_slew_limits_future_speed():
    reference=TurnSpeedReference(corner(),0.,2.8,.61)
    c=reference.constraint(44.,10.,0.,1.,3.,.1)
    assert c.reason=='turn_steering_rate'
    assert c.speed_cap_mps<=.1/reference.steer_gradient[44]+1e-10


def test_jerk_ramp_requires_earlier_braking_for_positive_acceleration():
    reference=TurnSpeedReference(corner(),0.,2.8,.61)
    level=reference.constraint(10.,10.,0.,1.,3.,.6)
    accelerating=reference.constraint(10.,10.,2.,1.,3.,.6)
    assert accelerating.speed_cap_mps<level.speed_cap_mps
    gentler_jerk=reference.constraint(10.,10.,0.,1.,3.,.6,jerk=.5)
    assert gentler_jerk.speed_cap_mps<level.speed_cap_mps


def test_geometrically_infeasible_offset_is_not_declared_drivable():
    reference=TurnSpeedReference(corner(-2*math.pi/3),4.,2.8,.61)
    c=reference.constraint(50.,2.,0.,1.,3.,.6)
    assert c.geometric_infeasibility
    assert c.speed_cap_mps==0.
    assert c.reason=='turn_geometry_infeasible'


def test_straight_road_has_no_artificial_low_speed_cap():
    frame=FrenetFrame([(0.,0.),(0.,-200.)],[0.,200.],37.,-122.)
    c=TurnSpeedReference(frame,1.75,2.8,.61).constraint(10.,30.,0.,1.,3.,.6)
    assert c.speed_cap_mps>30.
    assert c.braking_mps2==0.

@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('speed',[0.,1.5,4.,15.])
@pytest.mark.parametrize('steer',[.0,.2,.6])
def test_front_preview_agrees_with_independently_integrated_motion(direction,speed,steer):
    steer*=direction
    x,z,yaw=0.,0.,.2
    beta=math.atan(.5*math.tan(steer));yr=speed*math.cos(beta)*math.tan(steer)/2.8
    length=min(.7,.2*speed);time=length/max(speed,.01)
    # High-resolution independent integration of physical center/body motion.
    px=pz=0.;angle=yaw
    for _ in range(2000):
        dt=time/2000
        px+=speed*dt*math.sin(angle+beta+yr*dt/2)
        pz-=speed*dt*math.cos(angle+beta+yr*dt/2)
        angle+=yr*dt
    expected=(px+1.4*math.sin(angle),pz-1.4*math.cos(angle),angle)
    assert predict_front_axle(x,z,yaw,speed,steer,2.8,length)==pytest.approx(expected,abs=1e-8)

@pytest.mark.parametrize('field',range(7))
@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_preview_invalid_state_rejected(field,bad):
    args=[0.,0.,0.,4.,.2,2.8,.7];args[field]=bad
    with pytest.raises(ValueError):predict_front_axle(*args)

@pytest.mark.parametrize('args',[(0.,0.,0.,-1.,0.,2.8,.7),(0.,0.,0.,1.,0.,0.,.7),(0.,0.,0.,1.,2.,2.8,.7),(0.,0.,0.,1.,0.,2.8,-.7)])
def test_preview_impossible_state_rejected(args):
    with pytest.raises(ValueError):predict_front_axle(*args)


def test_bad_reference_longitudinal_control_brakes(monkeypatch):
    from app.services.physics_engine import PhysicsEngine
    from app.services.interfaces import SensorObservation,EgoProprioception
    from app.services.driver import driver as module
    p=PhysicsEngine(seed=42)
    p.set_route([(37.,-122.),(37.001,-122.)])
    p.step_world_sensors(0.,perceive=False,predict=False)
    obs=SensorObservation(clock=p.clock,ego=EgoProprioception(0.,0.,0.,0.,0.,0.),frenet_frame=p.frenet_frame)
    def invalid(*args):raise ValueError('invalid raw geometry')
    monkeypatch.setattr(module,'TurnSpeedReference',invalid)
    decision=p._driver.decide_longitudinal(obs,p.build_driver_context(.02))
    assert decision.accel_mps2==-p.A_MAX_BRAKE_MPS2
    assert decision.speed_limit_reason=='turn_reference_infeasible'
    assert p._driver.target_speed_mps==0.


def _drive_circle(direction,radius,speed_mu):
    from app.services.physics_engine import PhysicsEngine
    from app.services.frenet import local_to_latlng
    from app.services.interfaces import SensorObservation,EgoProprioception
    speed,mu=speed_mu
    p=PhysicsEngine(seed=42)
    points=[local_to_latlng(radius*math.sin(direction*a),-radius*math.cos(a),37.,-122.)
            for a in [i*.01 for i in range(601)]]
    p.set_destination(*points[-1]);p.set_route(points)
    p.reset_state(station_m=5.,speed_kmh=speed*3.6,lateral_offset_m=.3)
    p.friction_mu=mu
    p._driver.lateral_target_d_m=0.
    errors=[]
    previous=p.steering_angle_rad
    for i in range(450):
        p.step_world_sensors(.02,perceive=False,predict=False)
        obs=SensorObservation(clock=p.clock,ego=EgoProprioception(
            speed_mps=p.speed_kmh/3.6,heading_deg=p.heading,accel_mps2=0.,
            steer_rad=p.steering_angle_rad,station_m=p.current_station_m,
            lateral_offset_m=p.current_lateral_offset_m),frenet_frame=p.frenet_frame,
            safety_detections=())
        command=p._driver.control(obs,p.build_driver_context(.02))
        assert abs(command.steer_rad)<=p.MAX_STEER_RAD+1e-10
        assert abs(command.steer_rad-previous)<=p.STEER_RATE_MAX_RADPS*.02+1e-10
        previous=command.steer_rad
        p.step_dynamics(steer_rad=command.steer_rad,accel_mps2=0.,dt=.02)
        assert p.lateral_accel_mps2<=p.A_LAT_MAX_MPS2*mu+1e-6
        if i>100:errors.append(abs(p.current_lateral_offset_m))
    return max(errors)



def old_front(x,z,heading,speed,steer,wheelbase,preview):
    beta=math.atan(.5*math.tan(steer))
    return x+wheelbase/2*math.sin(heading)+preview*math.sin(heading+beta),z-wheelbase/2*math.cos(heading)-preview*math.cos(heading+beta),heading

@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('radius',[8.,12.,25.])
@pytest.mark.parametrize('speed_mu',[(1.5,.3),(2.5,.3),(4.,1.)])
def test_actual_steering_improves_paired_baseline(direction,radius,speed_mu,monkeypatch):
    from app.services.driver import driver as module
    from app.services.driver.turn_reference import TurnSpeedConstraint
    corrected=_drive_circle(direction,radius,speed_mu)
    with monkeypatch.context() as patch:
        patch.setattr(module,'predict_front_axle',old_front)
        patch.setattr(module,'front_axle_lateral_bias',lambda *args:0.)
        patch.setattr(module,'front_axle_heading_advance',lambda *args:0.)
        from app.services.driver.turn_tracking import continuous_path_heading as old_heading
        patch.setattr(module,'continuous_path_heading',lambda frame,station,offset,**kwargs:old_heading(frame,station,offset,half_window_m=1.))
        from app.services.driver.turn_reference import SteeringDemandConstraint
        patch.setattr(module,'steering_demand_constraint',lambda **kwargs:SteeringDemandConstraint(float('inf'),0.,0.))
        patch.setattr(module,'raw_route_curvature',signed_curvature_at)
        patch.setattr(module.TurnSpeedReference,'constraint',lambda *args,**kwargs:TurnSpeedConstraint(float('inf'),0.,'cruise',0.,False))
        baseline=_drive_circle(direction,radius,speed_mu)
    assert corrected<baseline
    if radius==25.:
        assert corrected<.5

@pytest.mark.parametrize('offset',[-2.,0.,2.])
def test_parallel_arc_length_and_rate_have_correct_physical_units(offset):
    reference=TurnSpeedReference(circle(),offset,2.8,.61)
    raw=reference.curvature[150]/(1-reference.curvature[150]*offset)
    scale=1+raw*offset
    assert reference.arc_scale[150]==pytest.approx(scale)
    assert reference.arc[151]-reference.arc[150]==pytest.approx(scale,rel=.05)
    # Cartesian velocity is ds/dt multiplied by the parallel-path metric.
    gradient=reference.steer_gradient[150]
    if gradient>1e-9:
        cartesian_cap=.6*scale/gradient
        assert gradient*cartesian_cap/scale==pytest.approx(.6)


def test_profile_rejects_queries_past_recorded_road_end():
    reference=TurnSpeedReference(corner(),0.,2.8,.61)
    for station in [-1.,101.]:
        with pytest.raises(ValueError):reference.constraint(station,10.,0.,1.,3.,.6)


def test_unwrapped_hairpin_cannot_reverse_the_demand_sign():
    # Three successive right rotations of 90 degrees inside the window.
    frame=FrenetFrame([(0.,0.),(0.,-1.),(1.,-1.),(1.,0.),(0.,0.)],[0.,1.,2.,3.,4.],37.,-122.)
    with pytest.raises(ValueError,match='heading domain'):raw_route_curvature(frame,2.)
    reference=TurnSpeedReference(frame,0.,2.8,.61)
    assert reference.infeasible.any()


def test_positive_acceleration_jerk_ramp_stops_actual_world_before_infeasible_region():
    from app.services.physics_engine import PhysicsEngine
    reference=TurnSpeedReference(corner(-2*math.pi/3),4.,2.8,.61)
    entry=int(np.flatnonzero(reference.infeasible)[0])
    available=float(reference.arc[entry])
    boundary=reference.constraint(0.,2.,3.,1.,3.,.6,braking=1.,jerk=2.5)
    p=PhysicsEngine(seed=42)
    p.set_route([(37.,-122.),(37.002,-122.)])
    p.reset_state(station_m=0.,speed_kmh=boundary.speed_cap_mps*3.6,lateral_offset_m=0.)
    p.acceleration_mps2=3.
    previous=p.acceleration_mps2
    for _ in range(4000):
        p.step_dynamics(steer_rad=0.,accel_mps2=-1.,dt=.02)
        assert abs(p.acceleration_mps2-previous)<=p.JERK_MAX_MPS3*.02+1e-10
        previous=p.acceleration_mps2
        p.step_world_sensors(.02,perceive=False,predict=False)
        if p.speed_kmh<.001:break
    assert p.speed_kmh<.001
    assert p.current_station_m<=available+.05


def test_isolated_overlapping_backtrack_is_not_a_real_turn_reference():
    points=[(0.,0.),(10.,0.),(9.,0.),(8.,0.),(20.,0.)]
    frame=FrenetFrame(points,[0.,10.,11.,12.,24.],37.,-122.)
    reference=TurnSpeedReference(frame,1.75,2.8,.61)
    assert np.max(abs(reference.curvature))<1e-12
    assert raw_route_curvature(frame,10.)==pytest.approx(0.)
    assert frame.points_xz==tuple(points)
    actual_reversal=FrenetFrame([(0.,0.),(10.,0.),(0.,0.),(-10.,0.)],[0.,10.,20.,30.],37.,-122.)
    # A real continuation back west does not return to the incoming direction.
    assert TurnSpeedReference(actual_reversal,0.,2.8,.61).infeasible.any()
