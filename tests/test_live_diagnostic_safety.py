import pytest
from scripts.capture_live_diagnostics import safety_failure


def test_overlap_fails_acceptance_even_inside_road():
    assert safety_failure({'body_contained': True, 'swept_body_contained': True,
                           'overlap_detected': True}) == 'occupied_space_overlap'


def test_boundary_failure_retains_priority():
    assert safety_failure({'body_contained': False, 'overlap_detected': True}) == 'road_boundary_failure'


def test_predicted_risk_is_not_reported_as_observed_overlap():
    assert safety_failure({'body_contained': True, 'swept_body_contained': True,
                           'overlap_detected': False, 'footprint_collision_risk': True}) is None


@pytest.mark.parametrize('parked',[False,True])
def test_parking_state_cannot_mask_or_mislabel_overlap_failure(monkeypatch, tmp_path, capsys, parked):
    import asyncio
    import json
    from scripts import capture_live_diagnostics as recorder

    message = {'type': 'state', 'simulation_id': 'recorded-test', 'epoch': 1,
               'tick': 42, 'simulation_time_s': 4.2,
               'channels': {'pose': {'ego': {'pose': {'z': -10.},
                   'parking': {'parked': parked}, 'sensor_health': {'ready': True}}},
                   'semantic': {'safety_shield': {'body_contained': True,
                       'swept_body_contained': True, 'overlap_detected': True}}}}

    class Socket:
        async def __aenter__(self):
            return self

        async def __aexit__(self, *_):
            return False

        async def recv(self):
            return json.dumps(message)

    monkeypatch.setattr(recorder.websockets, 'connect', lambda *args, **kwargs: Socket())
    target = tmp_path / 'live.json'
    assert not asyncio.run(recorder.capture(1., target, until_parked=True))
    summary = json.loads(capsys.readouterr().out)
    assert summary['stop_reason'] == 'occupied_space_overlap'
    assert summary['overlap_samples'] == 1
    assert summary['goal_timeout'] is False
    assert json.loads(target.read_text(encoding='utf-8'))[0]['shield']['overlap_detected']
