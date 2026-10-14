import json
import math

import pytest

from app.services import road_routes
from app.services.carla_adapter.route import DeterministicRouteProvider
from app.services.path_smoothing import smooth_route
from app.services.simulation_session import SimulationSession


def test_default_session_and_route_provider_use_recorded_us101():
    recorded = road_routes.real_road_route()
    assert recorded[0] == (37.615968, -122.3984)
    assert recorded[-1] == (37.592646, -122.365383)
    assert DeterministicRouteProvider().waypoints() == recorded
    session = SimulationSession(seed=42, real_time=False)
    from app.services.parking import parking_plan
    plan = parking_plan()
    assert list(plan.waypoints[:len(recorded)]) == recorded
    assert session.world.physics.route == smooth_route(plan.waypoints)
    assert session.world.physics.parking_mission['lot_way_id'] == plan.lot_way_id
    assert 'mapped parking area' in session.route_steps[0]['instruction']


@pytest.mark.parametrize('scenario', [
    'normal_cruising', 'traffic_overtake', 'emergency_cut_in',
    'queue_stop_and_go', 'erratic_cut_in', 'highway_merge',
    'lane_straddler', 'low_friction_curve',
])
def test_every_scenario_and_reset_restore_recorded_roads(scenario):
    session = SimulationSession(seed=42, real_time=False)
    session._handle_command({'type': 'load_scenario', 'scenario_id': scenario})
    profile = 'wet_curve' if scenario == 'low_friction_curve' else 'highway'
    expected = smooth_route(road_routes.real_road_route(profile))
    assert session.world.physics.route == expected
    initial = session.world.physics.observe()
    for _ in range(20):
        session.step_pipeline()
    session.reset()
    assert session.world.physics.route == expected
    assert session.world.physics.observe() == initial
    # Switching away from the wet curve must restore highway geometry.
    session._handle_command({'type': 'load_scenario', 'scenario_id': 'normal_cruising'})
    assert session.world.physics.route == smooth_route(road_routes.real_road_route())


def test_corrupt_recorded_geometry_fails_without_synthetic_fallback(tmp_path, monkeypatch):
    original = json.loads(road_routes.ROUTE_DATA.read_text())
    original['profiles']['highway']['waypoints'][0][0] += 0.01
    corrupt = tmp_path / 'routes.json'
    corrupt.write_text(json.dumps(original))
    road_routes._profile.cache_clear()
    monkeypatch.setattr(road_routes, 'ROUTE_DATA', corrupt)
    try:
        with pytest.raises(ValueError, match='integrity'):
            SimulationSession(real_time=False)
    finally:
        road_routes._profile.cache_clear()


@pytest.mark.parametrize('profile', ['highway', 'wet_curve'])
def test_smoothing_stays_within_one_metre_of_recorded_road(profile):
    raw = road_routes.real_road_route(profile)
    lat, lng = raw[0]
    def project(point):
        return ((point[1] - lng) * 111320 * math.cos(math.radians(lat)),
                (point[0] - lat) * 111320)
    def distance(point, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0, min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (dx*dx + dy*dy)))
        return math.hypot(point[0] - a[0] - t*dx, point[1] - a[1] - t*dy)
    source = list(map(project, raw))
    for point in smooth_route(raw):
        assert min(distance(project(point), a, b) for a, b in zip(source, source[1:])) < 1.0
