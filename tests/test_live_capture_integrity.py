import asyncio
import json
import pytest
from scripts import capture_live_diagnostics as diagnostics


def state(tick, *, epoch=1, parking=None, contained=True):
    return dict(type='state',simulation_id='live',epoch=epoch,tick=tick,simulation_time_s=tick/10,
        channels=dict(pose=dict(ego=dict(pose=dict(z=-tick),parking=parking,sensor_health=dict(ready=True))),
                      semantic=dict(safety_shield=dict(body_contained=contained,swept_body_contained=contained))))


class Connection:
    def __init__(self,messages):self.messages=iter(messages)
    async def __aenter__(self):return self
    async def __aexit__(self,*args):return False
    async def recv(self):
        message=next(self.messages)
        if isinstance(message,Exception):raise message
        return json.dumps(message)


def setup(monkeypatch,messages):
    monkeypatch.setattr(diagnostics.websockets,'connect',lambda url, **options:Connection(messages))


def test_long_route_transport_has_explicit_finite_message_bound(monkeypatch,tmp_path):
    options={}
    def connect(url, **kwargs):
        options.update(kwargs)
        return Connection([{'type':'route','waypoints':[[1,2]]*140000},state(1,parking={'parked':True})])
    monkeypatch.setattr(diagnostics.websockets,'connect',connect)
    assert asyncio.run(diagnostics.capture(10,tmp_path/'capture.json',True))
    assert options['max_size']==16*1024*1024


def test_null_parking_is_valid_until_actual_parked_sample(monkeypatch,tmp_path):
    setup(monkeypatch,[state(1),state(2,parking={'parked':True})])
    output=tmp_path/'capture.json'
    assert asyncio.run(diagnostics.capture(10,output,True))
    assert len(json.loads(output.read_text()))==2


def test_new_epoch_cannot_be_counted_as_previous_mission_success(monkeypatch,tmp_path,capsys):
    setup(monkeypatch,[state(1),state(2,epoch=2,parking={'parked':True})])
    output=tmp_path/'capture.json'
    assert not asyncio.run(diagnostics.capture(10,output,True))
    assert len(json.loads(output.read_text()))==1
    assert json.loads(capsys.readouterr().out)['stop_reason']=='simulation_or_epoch_changed'


def test_boundary_failure_is_saved_and_stops_goal_capture(monkeypatch,tmp_path,capsys):
    setup(monkeypatch,[state(1,contained=False)])
    output=tmp_path/'capture.json'
    assert not asyncio.run(diagnostics.capture(10,output,True))
    assert len(json.loads(output.read_text()))==1
    assert json.loads(capsys.readouterr().out)['stop_reason']=='road_boundary_failure'


def test_disconnect_preserves_final_trace_without_passing(monkeypatch,tmp_path):
    setup(monkeypatch,[state(1),RuntimeError('connection lost')])
    output=tmp_path/'capture.json'
    with pytest.raises(RuntimeError,match='connection lost'):
        asyncio.run(diagnostics.capture(10,output,True))
    assert len(json.loads(output.read_text()))==1
    assert output.with_suffix('.jsonl').read_text().count('\n')==1
