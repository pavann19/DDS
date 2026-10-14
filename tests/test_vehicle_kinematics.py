"""Physical invariants for steering, wheel travel and center-referenced motion."""
import math
import pytest
from app.services.vehicle_kinematics import wheel_geometry, sideslip, TRACK_WIDTH_M, WHEEL_RADIUS_M
from app.services.physics_engine import PhysicsEngine
from app.services.frenet import latlng_to_local


@pytest.mark.parametrize("steer", [-.6, -.2, .2, .6])
def test_ackermann_wheels_share_one_instantaneous_turn_center(steer):
    left, right, _, _ = wheel_geometry(steer, 2.8)[0]
    assert 2.8 / math.tan(right) + TRACK_WIDTH_M / 2 == pytest.approx(2.8 / math.tan(left) - TRACK_WIDTH_M / 2)
    inner, outer = (right, left) if steer > 0 else (left, right)
    assert abs(inner) > abs(outer)
    mirrored = wheel_geometry(-steer, 2.8)[0]
    assert mirrored[:2] == pytest.approx((-right, -left))


def test_wheel_travel_is_distance_based_and_stops_at_zero_speed():
    p = PhysicsEngine(seed=42)
    p.heading = 0
    p.speed_kmh = 36
    p.step_dynamics(0, 0, .02)
    assert p.wheel_rotation_rad == pytest.approx([.2 / WHEEL_RADIUS_M] * 4)
    p.speed_kmh = 0
    previous = list(p.wheel_rotation_rad)
    p.step_dynamics(.4, 0, .02)
    assert p.wheel_rotation_rad == previous
    p.reset_state()
    assert p.wheel_rotation_rad == [0] * 4


@pytest.mark.parametrize("steer", [-.3, .3])
def test_center_motion_matches_no_slip_bicycle_arc(steer):
    p = PhysicsEngine(seed=42)
    p.heading = 0
    p.speed_kmh = 18
    # This is the steady-angle arc invariant, not an instantaneous actuator jump.
    p.steering_angle_rad = steer
    p.steering_angle = steer / p.MAX_STEER_RAD
    p.kinematic_sideslip_rad = sideslip(steer)
    origin = p.lat, p.lng
    dt = .02
    p.step_dynamics(steer, 0, dt)
    beta = sideslip(steer)
    yaw = 5 * math.cos(beta) * math.tan(steer) / p.WHEELBASE_M * dt
    assert p.heading == pytest.approx(math.degrees(yaw) % 360)
    x, z = latlng_to_local(p.lat, p.lng, *origin)
    assert x == pytest.approx(5 * dt * math.sin(beta + yaw / 2), abs=1e-6)
    assert z == pytest.approx(-5 * dt * math.cos(beta + yaw / 2), abs=1e-6)
    fl, fr, rl, rr = p.wheel_rotation_rad
    assert (fl > fr and rl > rr) if steer > 0 else (fl < fr and rl < rr)


def test_authoritative_pose_does_not_snap_to_route_endpoint():
    from app.services.frenet import local_to_latlng
    p = PhysicsEngine(seed=42)
    route = [(37., -122.), (37.001, -122.)]
    p.set_route(route)
    f = p.frenet_frame
    p.lat, p.lng = local_to_latlng(2., -140., f.origin_lat, f.origin_lng)
    p.current_station_m, p.current_lateral_offset_m = f.station[-1], -2.
    pose = p.get_navigation_state()['world_pos']
    assert pose['x'] == pytest.approx(2.)
    assert pose['z'] == pytest.approx(-140.)


@pytest.mark.parametrize("requested_steer",[-.3,.3])
def test_transient_center_arc_uses_applied_rate_limited_wheels(requested_steer):
    p=PhysicsEngine(seed=42)
    p.heading=0.
    p.speed_kmh=18.
    origin=p.lat,p.lng
    dt=.02
    p.step_dynamics(requested_steer,0.,dt)
    applied=p.steering_angle_rad
    assert applied==pytest.approx(math.copysign(p.STEER_RATE_MAX_RADPS*dt,requested_steer))
    assert abs(applied)<abs(requested_steer)
    beta=sideslip(applied)
    yaw=5*math.cos(beta)*math.tan(applied)/p.WHEELBASE_M*dt
    assert p.heading==pytest.approx(math.degrees(yaw)%360)
    x,z=latlng_to_local(p.lat,p.lng,*origin)
    assert x==pytest.approx(5*dt*math.sin(beta+yaw/2),abs=1e-6)
    assert z==pytest.approx(-5*dt*math.cos(beta+yaw/2),abs=1e-6)
