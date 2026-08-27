"""Contract tests for app/services/interfaces.py (ADR-001, Action Item 1).

These lock the *shape* of the World/Driver contracts -- the types are pure
data, so the only things worth asserting are that they construct with the
documented fields, that ``SimClock`` advances deterministically and stays
immutable, and that the frozen dataclasses actually reject mutation.
"""
import dataclasses

import pytest

from app.services.interfaces import (
    ActuatorCommand,
    AgentPrediction,
    EgoProprioception,
    LeadObservation,
    PerceptionOutput,
    PlannedTrajectory,
    PredictedState,
    PredictionOutput,
    SensorObservation,
    SimClock,
    SOURCE_PLANNER,
    SOURCE_SAFETY_OVERRIDE,
    TrackEstimate,
    TrajectoryPoint,
)


def test_simclock_advances_deterministically():
    clock = SimClock(dt_s=0.02)
    assert (clock.tick, clock.sim_time_s) == (0, 0.0)

    one = clock.advance()
    assert one.tick == 1
    assert one.sim_time_s == pytest.approx(0.02)

    # Same start + same substeps => identical value, every time.
    a = SimClock(dt_s=0.02).advance(50)
    b = SimClock(dt_s=0.02).advance(50)
    assert a == b
    assert a.sim_time_s == pytest.approx(1.0)

    # advance() does not mutate the receiver.
    assert clock.tick == 0


def test_simclock_is_frozen():
    clock = SimClock()
    with pytest.raises(dataclasses.FrozenInstanceError):
        clock.tick = 5  # type: ignore[misc]


def test_simclock_rate_tick_predicate():
    base = SimClock(dt_s=0.02)  # 50 Hz base
    # 10 Hz stage runs once every 5 base ticks.
    ticks_that_fire = [t for t in range(20) if base.advance(t).is_rate_tick(10.0)]
    assert ticks_that_fire == [0, 5, 10, 15]
    # 50 Hz stage fires every base tick.
    assert all(base.advance(t).is_rate_tick(50.0) for t in range(10))


def test_sensor_observation_carries_only_permitted_fields():
    obs = SensorObservation(
        clock=SimClock(),
        ego=EgoProprioception(
            speed_mps=12.0, heading_deg=45.0, accel_mps2=0.3, steer_rad=0.01,
            station_m=100.0, lateral_offset_m=1.75,
        ),
        lead=LeadObservation(gap_m=30.0, lead_speed_mps=10.0),
        adjacent_lane_clear=True,
    )
    field_names = {f.name for f in dataclasses.fields(obs)}
    # The boundary: no NPC list, no traffic model handle, ever.
    assert "npcs" not in field_names
    assert "traffic" not in field_names
    assert field_names == {
        "clock", "ego", "lead", "adjacent_lane_clear",
        "surround_tracks", "occupancy", "frenet_frame",
    }
    assert obs.surround_tracks == ()
    assert obs.occupancy is None


def test_sensor_observation_is_frozen():
    obs = SensorObservation(
        clock=SimClock(),
        ego=EgoProprioception(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    with pytest.raises(dataclasses.FrozenInstanceError):
        obs.adjacent_lane_clear = True  # type: ignore[misc]


def test_driver_stage_outputs_construct():
    clock = SimClock()
    perception = PerceptionOutput(
        clock=clock,
        tracks=(TrackEstimate(1, "SEDAN", "CONFIRMED", 1.0, -20.0, 0.0, 8.0, 4.5, 1.8, 1.5),),
    )
    assert perception.tracks[0].track_id == 1

    prediction = PredictionOutput(
        clock=clock,
        agents=(AgentPrediction(track_id=1, states=(PredictedState(0.1, 1.0, -19.0, 0.0, 10.0),)),),
    )
    assert prediction.agents[0].states[0].t_s == pytest.approx(0.1)

    traj = PlannedTrajectory(
        clock=clock,
        points=(TrajectoryPoint(t_s=0.0, s_m=100.0, d_m=1.75, speed_mps=13.9),),
        chosen_d_m=1.75,
        target_speed_mps=13.9,
    )
    assert traj.points[0].d_m == pytest.approx(1.75)
    assert traj.is_lane_change is False


def test_actuator_command_defaults_and_provenance():
    cmd = ActuatorCommand(steer_rad=0.02, accel_mps2=-1.0)
    assert cmd.source == SOURCE_PLANNER
    assert cmd.speed_limit_reason == "cruise"

    overridden = ActuatorCommand(
        steer_rad=0.0, accel_mps2=-4.5,
        source=SOURCE_SAFETY_OVERRIDE, speed_limit_reason="safety_shield_override",
    )
    assert overridden.source == "safety_override"
