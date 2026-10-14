import math
import pandas as pd
import pytest
from app.services.physics_engine import PhysicsEngine
from app.services import safety_shield
from app.services.simulation_session import SimulationSession
from app.services.inference import pipeline


def test_emergency_bypasses_comfort_jerk_but_normal_control_does_not():
    p = PhysicsEngine(seed=42)
    p.speed_kmh = 48
    p.step_dynamics(steer_rad=0, accel_mps2=-4.5, dt=.02)
    assert p.acceleration_mps2 == pytest.approx(-p.JERK_MAX_MPS3 * .02)
    p.shield_verdict = safety_shield.ShieldVerdict(
        approved=False, risk_level=safety_shield.RISK_CRITICAL,
        override_action=safety_shield.OVERRIDE_EMERGENCY_BRAKE)
    before = p.speed_kmh
    p.step_dynamics(steer_rad=0, accel_mps2=2, dt=.02)
    assert p.acceleration_mps2 == -p.A_MAX_BRAKE_MPS2
    assert p.speed_kmh < before


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_invalid_dynamics_rejected_before_mutation(value):
    p = PhysicsEngine(seed=42)
    before = (p.heading, p.speed_kmh, p.lat, p.lng)
    with pytest.raises(ValueError):
        p.step_dynamics(steer_rad=value, accel_mps2=1, dt=.02)
    assert (p.heading, p.speed_kmh, p.lat, p.lng) == before


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('field', range(5))
def test_shield_never_approves_nonfinite_state(value, field):
    args = [10., 1.75, 0., 30., 5.]
    args[field] = value
    verdict = safety_shield.evaluate(*args)
    assert not verdict.approved
    assert verdict.risk_level == safety_shield.RISK_CRITICAL
    assert verdict.override_action == safety_shield.OVERRIDE_EMERGENCY_BRAKE


def test_reset_after_pause_replays_physics_and_resets_clocks():
    s = SimulationSession(seed=42, real_time=False)
    s._handle_command({'type': 'load_scenario', 'scenario_id': 'normal_cruising'})
    def trace():
        result = []
        for _ in range(10):
            s.step_pipeline()
            p = s.world.physics
            result.append((p.speed_kmh, p.lat, p.lng, p.rpm, p.altitude))
        return result
    first = trace()
    s._handle_command({'type': 'pause_simulation'})
    s.reset()
    assert not s.is_paused and not s.scenario_engine.is_paused
    assert s.executor.clock.sim_time_s == 0
    assert s.world.physics.clock.sim_time_s == 0
    assert trace() == first


def test_paused_step_advances_scenario_exactly_once_and_remains_paused():
    s = SimulationSession(seed=42, real_time=False)
    s._handle_command({'type': 'load_scenario', 'scenario_id': 'normal_cruising'})
    s._handle_command({'type': 'pause_simulation'})
    s._handle_command({'type': 'step_simulation'})
    assert s.executor.clock.sim_time_s == pytest.approx(.1)
    assert s.scenario_engine.elapsed_time_s == pytest.approx(.1)
    assert s.is_paused and s.scenario_engine.is_paused
    assert s.latest_payload['channels']['semantic']['scenario']['is_paused']


def test_new_scenario_does_not_reuse_previous_lead_sensor():
    from app.services.traffic import SensedLeadVehicle
    s = SimulationSession(seed=42, real_time=False)
    s.world.physics.sensed_lead = SensedLeadVehicle(gap_m=1., lead_speed_kmh=0.)
    s._handle_command({'type': 'load_scenario', 'scenario_id': 'normal_cruising'})
    assert s.latest_observation.lead is None or s.latest_observation.lead.gap_m > 40
    s.step_pipeline()
    assert abs(s.world.physics.acceleration_mps2) <= s.world.physics.JERK_MAX_MPS3 * .1


def test_shap_matches_model_margin_in_scaled_space():
    reading = {f: 0.0 for f in pipeline.features}
    reading.update({'RPM': 2104.8, 'Coolant': 81.1, 'Altitude': 162.5,
                    'Litre per 100km(Instant)': 7.8})
    result = pipeline.predict(reading)
    cls = list(pipeline.label_encoder.classes_).index(result['prediction'])
    scaled = pipeline.scaler.transform(pd.DataFrame([result['clean_input']]))
    margin = pipeline.model.predict(scaled, output_margin=True)[0][cls]
    explanation = result['shap_result']
    assert explanation['contributions']
    total = explanation['base_value'] + sum(i['contribution'] for i in explanation['contributions'])
    assert total == pytest.approx(float(margin), abs=1e-5)
    for i in explanation['contributions']:
        assert i['value'] == reading[i['feature']]


@pytest.mark.asyncio
async def test_reconnect_resumes_existing_database_session(test_db_session, monkeypatch):
    from app.core import database
    maker, _ = test_db_session
    monkeypatch.setattr(database, 'AsyncSessionLocal', maker)
    await database.create_stream_session('reconnect-regression')
    await database.finalize_stream_session('reconnect-regression', 'completed', 2, 180.)
    await database.create_stream_session('reconnect-regression')
    async with maker() as db:
        row = await db.get(database.SessionRecord, 'reconnect-regression')
        assert row.status == 'active'
        assert row.end_time is None
        assert row.total_predictions == 0  # No telemetry rows were committed.
        assert row.avg_score is None
