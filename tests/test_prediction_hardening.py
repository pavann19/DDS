"""Defensive-hardening tests for the Phase 6.5 / 7 code -- degenerate
inputs must degrade gracefully, never throw mid-tick.
"""
import math

import pytest

from app.services.executor import MultiRateExecutor
from app.services.frenet import build_frenet_frame
from app.services.interfaces import SimClock
from app.services.prediction import PredictionEngine, build_risk_field
from app.services.interfaces import AgentPrediction, PredictedState

_ROUTE = [(37.7749 + i * 0.000045, -122.4194) for i in range(120)]
_FRAME = build_frenet_frame(_ROUTE)


class _Track:
    def __init__(self, tid, x, z, vx, vz, status="CONFIRMED"):
        self.track_id, self.x, self.z, self.vx, self.vz = tid, x, z, vx, vz
        self.status = status
        self.entity_class = "SEDAN"


def _step(engine, tracks, dt=0.1):
    return engine.step(clock=SimClock(dt_s=0.1).advance(),
                       surround_tracks=tracks, frenet_frame=_FRAME,
                       ego_lateral_offset_m=1.75, dt=dt)


# --- PredictionEngine ------------------------------------------------------
def test_nan_track_is_dropped_and_good_tracks_still_forecast():
    engine = PredictionEngine()
    tracks = [
        _Track(1, 1.75, -30.0, 0.0, -12.0),
        _Track(2, float("nan"), -40.0, 0.0, -12.0),   # bad position
        _Track(3, 5.25, -50.0, float("inf"), -12.0),   # bad velocity
    ]
    res = _step(engine, tracks)
    ids = {a.track_id for a in res.output.agents}
    assert ids == {1}
    assert res.proactive_decel_mps2 == 0.0


def test_zero_and_negative_dt_do_not_crash():
    engine = PredictionEngine()
    tracks = [_Track(1, 1.75, -30.0, 0.0, -12.0)]
    for dt in (0.0, -0.1):
        res = _step(engine, tracks, dt=dt)
        assert len(res.output.agents) == 1


def test_track_missing_attributes_is_skipped_not_fatal():
    engine = PredictionEngine()

    class _Broken:
        track_id = 9
        status = "CONFIRMED"
        x = 1.0
        z = -10.0
        # no vx / vz

    res = _step(engine, [_Broken(), _Track(1, 1.75, -30.0, 0.0, -12.0)])
    assert {a.track_id for a in res.output.agents} == {1}


def test_empty_track_list_yields_empty_prediction_and_zero_risk():
    engine = PredictionEngine()
    res = _step(engine, [])
    assert res.output.agents == ()
    assert res.risk_field.risk_at(0.0, 0.0, 1.0) == 0.0
    assert res.proactive_decel_mps2 == 0.0


# --- MultiRateExecutor ---------------------------------------------------
def test_executor_rejects_non_positive_stage_rate():
    ex = MultiRateExecutor()
    with pytest.raises(ValueError):
        ex.add_stage("bad", 0.0, lambda clk: None)
    with pytest.raises(ValueError):
        ex.add_stage("bad", -10.0, lambda clk: None)


# --- risk field --------------------------------------------------------
def test_risk_field_stays_finite_with_a_nan_carrying_prediction():
    bad = AgentPrediction(
        track_id=1,
        states=tuple(
            PredictedState(t_s=round((i + 1) * 0.1, 2),
                           x=float("nan"), z=-20.0 - i, vx=0.0, vz=-10.0)
            for i in range(30)
        ),
    )
    field = build_risk_field([bad])
    r = field.risk_at(1.0, -25.0, 1.0)
    assert math.isfinite(r) and 0.0 <= r <= 1.0


# --- forecaster degenerate frame -------------------------------------------
def test_forecast_on_a_degenerate_frame_does_not_crash():
    from app.services.frenet import build_frenet_frame as _bff
    from app.services.prediction import forecast_agent, AgentKinematics

    # Two near-identical waypoints -> a valid but zero-length frame.
    frame = _bff([(37.7749, -122.4194), (37.77490001, -122.4194)])
    k = AgentKinematics(track_id=1, x=1.0, z=-5.0, vx=0.0, vz=-10.0)
    pred = forecast_agent(k, frame=frame)
    assert len(pred.states) == 30
    assert all(math.isfinite(s.x) and math.isfinite(s.z) for s in pred.states)
