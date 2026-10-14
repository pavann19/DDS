from types import SimpleNamespace as NS
import pytest
from app.services.frenet import FrenetFrame
from app.services.vehicle_signals import geometry_turns, route_turns, vehicle_signals


def frame(points):
    import math
    stations = [0.]
    for a, b in zip(points, points[1:]):
        stations.append(stations[-1] + math.dist(a, b))
    return FrenetFrame(points, stations, 37., -122.)


@pytest.mark.parametrize('x,direction', [(100., 'right'), (-100., 'left')])
def test_route_corner_sign_and_straight(x, direction):
    turns = geometry_turns(frame([(0., 0.), (0., -100.), (x, -100.)]))
    assert turns == [(100., direction)]
    assert geometry_turns(frame([(0., 0.), (0., -200.)])) == []


def test_s_bends_remain_two_distinct_intents():
    turns = geometry_turns(frame([(0., 0.), (0., -100.), (100., -100.), (100., -200.)]))
    assert turns == [(100., 'right'), (200., 'left')]


def test_intent_window_heading_jitter_pause_arrival_and_fault():
    physics = NS(is_paused=False, has_arrived=False, current_station_m=65., speed_kmh=50., acceleration_mps2=0., heading=0.)
    driver = NS(spatiotemporal_traj=None, _changing_lane_now=False)
    turns = [(100., 'right')]
    assert vehicle_signals(physics, driver, turns)['right']
    for heading in [-90., 3., 270.]:
        physics.heading = heading
        assert vehicle_signals(physics, driver, turns)['right']
    physics.current_station_m = 113.
    assert not vehicle_signals(physics, driver, turns)['right']
    physics.current_station_m = float('nan')
    assert not vehicle_signals(physics, driver, turns)['right']
    physics.current_station_m = 90.
    physics.is_paused = True
    signals = vehicle_signals(physics, driver, turns)
    assert signals['hazards'] and not signals['left'] and not signals['right']
    physics.is_paused = False
    physics.has_arrived = True
    assert not vehicle_signals(physics, driver, turns)['right']


def test_committed_lane_direction_precedes_route_and_is_not_tracking_error():
    physics = NS(is_paused=False, has_arrived=False, current_station_m=90., speed_kmh=50., acceleration_mps2=-1.)
    driver = NS(_changing_lane_now=True, spatiotemporal_traj=NS(d1_m=-1.75, lat_poly=NS(pos=lambda t: 1.75)))
    signals = vehicle_signals(physics, driver, [(100., 'right')])
    assert signals['left'] and not signals['right'] and signals['brake']


def test_malformed_maneuvers_fallback_and_empty_frame():
    straight = frame([(0., 0.), (0., -200.)])
    assert route_turns(straight, [None, {}, {'type': 'turn', 'modifier': 'left', 'location': [float('nan'), 1.]}]) == []
    assert route_turns(None, []) == []


def test_explicit_map_turn_precedes_geometry_and_rejects_remote_location():
    from app.services.frenet import frenet_to_latlng
    corner = frame([(0., 0.), (0., -100.), (100., -100.)])
    location = frenet_to_latlng(corner, 80., 0.)
    turns = route_turns(corner, [{'type': 'turn', 'modifier': 'left', 'location': location}])
    assert len(turns) == 1 and turns[0][1] == 'left'
    assert turns[0][0] == pytest.approx(80., abs=.01)
    straight = frame([(0., 0.), (0., -200.)])
    assert route_turns(straight, [{'type': 'turn', 'modifier': 'left', 'location': (38., -122.)}]) == []


@pytest.mark.parametrize('density,count', [('low', 4), ('medium', 8), ('high', 14)])
def test_density_preference_is_retained(density, count):
    from app.services.simulation_session import SimulationSession
    session = SimulationSession(seed=42, real_time=False)
    session._handle_command({'type':'load_scenario','scenario_id':'driving_environment','traffic_density':density,'params':{}})
    assert session.scenario_engine.active_density == density
    assert len(session.world.physics.traffic.npcs) == count


def test_short_route_density_cannot_overlap_actors():
    from app.services.physics_engine import PhysicsEngine
    from app.services.scenario_engine import configure_driving_environment_traffic
    physics = PhysicsEngine(seed=42)
    physics.set_route([(37., -122.), (37.0004, -122.)])
    configure_driving_environment_traffic(physics, 'high')
    traffic = physics.traffic
    stations = sorted(n.station_m for n in traffic.npcs)
    assert len(stations) < 14
    assert all(b-a >= 12. for a,b in zip(stations, stations[1:]))
    assert all(0. < s < traffic.total_length_m for s in stations)


def test_combined_density_and_initial_actor_spacing():
    from app.services.simulation_session import SimulationSession
    session = SimulationSession(seed=42, real_time=False)
    session._handle_command({'type':'load_scenario','scenario_id':'driving_environment','params':{}})
    physics = session.world.physics
    assert session.scenario_engine.active_density == 'medium'
    assert len(physics.traffic.npcs) == 8
    stations = sorted(n.station_m for n in physics.traffic.npcs)
    assert stations[:4] == [40., 70., 100., 130.]
    assert all(b-a >= 12. for a,b in zip(stations, stations[1:]))
    assert all(0. <= s < physics.traffic.total_length_m for s in stations)
    session._handle_command({'type':'reset_simulation'})
    assert sorted(n.station_m for n in session.world.physics.traffic.npcs) == stations
