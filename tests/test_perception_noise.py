"""
Measurement-boundary imperfection model for the surround perception sensor
(PerceptionNoiseConfig). Requirement: default config is a NO-OP -- the
perception output must be byte-for-byte identical to the pre-noise pipeline
-- and each imperfection, when enabled, must have the intended, reproducible
effect on the DETECTIONS (before the EKF), never a privileged-state leak.
"""
import math
import pytest

from app.services.frenet import build_frenet_frame
from app.services.traffic import NpcVehicle
from app.services.perception.perception_engine import (
    SurroundPerceptionEngine,
    PerceptionNoiseConfig,
)


def _frame():
    pts = [(37.7749, -122.4194 + i * 0.0005) for i in range(60)]  # ~straight east
    return build_frenet_frame(pts)


def _npcs(n=6):
    return [
        NpcVehicle(id=f"n{i}", lane_offset=1.75 if i % 2 else -1.75,
                   speed_kmh=40.0, station_m=20.0 + 12.0 * i, desired_speed_kmh=40.0)
        for i in range(n)
    ]


def _run(engine, frame, npcs, ticks=25, dt=0.1):
    out = []
    ego_s = 0.0
    for _ in range(ticks):
        ego_s += 11.0 * dt
        tracks = engine.step(frame, ego_s, 1.75, npcs, dt)
        out.append([(t.track_id, round(t.x, 6), round(t.z, 6)) for t in tracks])
    return out


def test_default_config_is_a_noop_byte_for_byte():
    frame, npcs = _frame(), _npcs()
    a = _run(SurroundPerceptionEngine(), frame, npcs)
    b = _run(SurroundPerceptionEngine(noise=PerceptionNoiseConfig(), seed=123), frame, npcs)
    assert a == b


def test_position_noise_perturbs_tracks_and_is_reproducible():
    frame, npcs = _frame(), _npcs()
    clean = _run(SurroundPerceptionEngine(), frame, npcs)
    cfg = PerceptionNoiseConfig(position_sigma_m=0.4)
    n1 = _run(SurroundPerceptionEngine(noise=cfg, seed=7), frame, npcs)
    n2 = _run(SurroundPerceptionEngine(noise=cfg, seed=7), frame, npcs)
    assert n1 == n2                      # reproducible for a fixed seed
    assert n1 != clean                   # and it actually moved the tracks
    # different seed -> different realisation
    n3 = _run(SurroundPerceptionEngine(noise=cfg, seed=8), frame, npcs)
    assert n3 != n1


def test_dropout_reduces_the_confirmed_track_count():
    frame, npcs = _frame(), _npcs(6)
    base = _run(SurroundPerceptionEngine(), frame, npcs, ticks=30)
    heavy = _run(SurroundPerceptionEngine(noise=PerceptionNoiseConfig(dropout_prob=0.6), seed=3),
                 frame, npcs, ticks=30)
    base_max = max(len(f) for f in base)
    heavy_max = max(len(f) for f in heavy)
    assert heavy_max <= base_max
    assert sum(len(f) for f in heavy) < sum(len(f) for f in base)


def test_latency_delays_first_detection():
    frame, npcs = _frame(), _npcs(3)
    base = _run(SurroundPerceptionEngine(), frame, npcs, ticks=20)
    lag = _run(SurroundPerceptionEngine(noise=PerceptionNoiseConfig(latency_ticks=5), seed=1),
               frame, npcs, ticks=20)
    first_base = next((i for i, f in enumerate(base) if f), None)
    first_lag = next((i for i, f in enumerate(lag) if f), None)
    assert first_base is not None and first_lag is not None
    assert first_lag >= first_base + 4


def test_occlusion_can_hide_a_far_vehicle_behind_a_near_one():
    frame = _frame()
    # two NPCs in the same lane, nearly collinear from the ego -> the far one
    # is occluded by the near one
    npcs = [
        NpcVehicle(id="near", lane_offset=1.75, speed_kmh=30.0, station_m=25.0, desired_speed_kmh=30.0),
        NpcVehicle(id="far", lane_offset=1.75, speed_kmh=30.0, station_m=45.0, desired_speed_kmh=30.0),
    ]
    base = _run(SurroundPerceptionEngine(), frame, npcs, ticks=15)
    occ = _run(SurroundPerceptionEngine(noise=PerceptionNoiseConfig(occlusion_enabled=True), seed=1),
               frame, npcs, ticks=15)
    assert max(len(f) for f in base) == 2
    assert max(len(f) for f in occ) == 1


def test_config_is_noop_predicate():
    assert PerceptionNoiseConfig().is_noop()
    assert not PerceptionNoiseConfig(position_sigma_m=0.1).is_noop()
    assert not PerceptionNoiseConfig(dropout_prob=0.01).is_noop()
    assert not PerceptionNoiseConfig(latency_ticks=1).is_noop()
    assert not PerceptionNoiseConfig(occlusion_enabled=True).is_noop()


def test_physics_engine_default_perception_is_noop_and_scenario_param_wires_it():
    from app.services.physics_engine import PhysicsEngine
    from app.services.scenario_engine import ScenarioEngine

    eng = PhysicsEngine(seed=1)
    assert eng.perception_noise.is_noop()

    se = ScenarioEngine()
    phys = PhysicsEngine(seed=1)
    phys.set_route([(37.7749 + i * 0.00005, -122.4194) for i in range(120)])
    se.load_scenario("normal_cruising", phys, params={
        "position_sigma_m": 0.35, "dropout_prob": 0.05, "latency_ticks": 2,
    })
    assert not phys.perception_noise.is_noop()
    assert phys.perception_noise.position_sigma_m == pytest.approx(0.35)
    assert phys.perception_noise.latency_ticks == 2
