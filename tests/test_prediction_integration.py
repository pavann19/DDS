"""Phase 7 -- prediction wired into PhysicsEngine (Gate 7.2).

Gate 7.2: on a high-confidence cut-in the ego sheds speed at < 1.5 m/s^2,
proactively, so the Safety Shield's critical-TTC path never has to fire.

The traffic model's NPCs only move longitudinally, so the test scripts a
gradual lane-offset change on one NPC (0.4 m/s toward the ego lane) -- the
surround tracker then sees real lateral motion and the prediction stage
picks it up.
"""
import time

import pytest

from app.services.physics_engine import PhysicsEngine
from app.services.traffic import NpcVehicle, ADJACENT_LANE_OFFSET_M, EGO_LANE_OFFSET_M

_STRAIGHT_ROUTE = [(37.7749 + i * 0.000045, -122.4194) for i in range(260)]
DT = 0.1


def _engine_on_route(seed=7):
    eng = PhysicsEngine(seed=seed)
    eng.set_destination(*_STRAIGHT_ROUTE[-1])
    eng.set_route(_STRAIGHT_ROUTE)
    eng.traffic.npcs = []  # start clean; the test injects its own actor
    return eng


def _run_cut_in(engine, *, drift_mps=0.4, start_gap_m=32.0, ticks=60):
    """Drive the ego up to speed, then release one NPC from the adjacent lane
    that drifts toward the ego lane at ``drift_mps``. Returns per-tick traces.
    """
    # spin up to cruising speed with an empty road
    for _ in range(80):
        engine.last_update_time = time.time() - DT
        engine.update("Maintain Speed", dt=DT)

    npc = NpcVehicle(
        id="cutin", lane_offset=ADJACENT_LANE_OFFSET_M,
        speed_kmh=engine.speed_kmh * 0.9, station_m=engine.current_station_m + start_gap_m,
        desired_speed_kmh=engine.speed_kmh * 0.9, prevent_recycle=True,
    )
    engine.traffic.npcs = [npc]

    per_step = drift_mps * DT
    accels, reasons, overrides, speeds = [], [], [], []
    for _ in range(ticks):
        if npc.lane_offset > EGO_LANE_OFFSET_M:
            npc.lane_offset = max(EGO_LANE_OFFSET_M, npc.lane_offset - per_step)
        engine.last_update_time = time.time() - DT
        engine.update("Maintain Speed", dt=DT)
        accels.append(engine.acceleration_mps2)
        reasons.append(engine.get_navigation_state()["speed_limit_reason"])
        overrides.append(engine.shield_verdict.override_action)
        speeds.append(engine.speed_kmh)
    return accels, reasons, overrides, speeds


def test_gate_7_2_proactive_slowdown_is_gentle_and_prevents_critical_ttc():
    engine = _engine_on_route()
    accels, reasons, overrides, speeds = _run_cut_in(engine)

    assert "predictive_cut_in" in reasons, "prediction stage should engage a proactive slowdown"

    # While the proactive response is the binding constraint, deceleration
    # stays inside the comfort bound the gate allows.
    proactive_accels = [a for a, r in zip(accels, reasons) if r == "predictive_cut_in"]
    assert proactive_accels
    assert min(proactive_accels) > -1.5

    # The Safety Shield's emergency path never has to fire -- the early
    # response kept TTC out of the critical band.
    assert "EMERGENCY_BRAKE" not in overrides

    # And the ego actually slowed down.
    assert speeds[-1] < speeds[0]


def test_proactive_response_starts_before_the_forward_sensor_would_react():
    """The cut-in vehicle is still a full lane away when prediction engages;
    IDM (forward sensor, +/- half a lane) could not have seen it yet."""
    engine = _engine_on_route()
    accels, reasons, _, _ = _run_cut_in(engine, ticks=45)

    first_predictive = next((i for i, r in enumerate(reasons) if r == "predictive_cut_in"), None)
    first_carfollow = next((i for i, r in enumerate(reasons) if r == "car_following"), None)
    assert first_predictive is not None
    if first_carfollow is not None:
        assert first_predictive < first_carfollow


def test_no_cut_in_no_proactive_slowdown():
    """A vehicle holding its adjacent lane must not trigger the response."""
    engine = _engine_on_route()
    for _ in range(80):
        engine.last_update_time = time.time() - DT
        engine.update("Maintain Speed", dt=DT)
    engine.traffic.npcs = [NpcVehicle(
        id="steady", lane_offset=ADJACENT_LANE_OFFSET_M, speed_kmh=45.0,
        station_m=engine.current_station_m + 30.0, desired_speed_kmh=45.0,
        prevent_recycle=True,
    )]
    reasons = []
    for _ in range(50):
        engine.last_update_time = time.time() - DT
        engine.update("Maintain Speed", dt=DT)
        reasons.append(engine.get_navigation_state()["speed_limit_reason"])
    assert "predictive_cut_in" not in reasons


def test_prediction_state_is_exposed():
    engine = _engine_on_route()
    _run_cut_in(engine, ticks=40)
    state = engine.get_prediction_state()
    assert "agents" in state and "cut_in" in state
    assert isinstance(state["proactive_decel_mps2"], float)
    if state["agents"]:
        a = state["agents"][0]
        assert "trail" in a and "intent" in a
        assert len(a["trail"]) == 30  # 3.0 s / 0.1 s


# ---------------------------------------------------------------------------
# Training-observation boundary: the curved-road phantom cut-in must be
# QUANTITATIVELY BOUNDED. With only same-direction, lane-keeping ambient
# traffic on a curve, the proactive cut-in response must fire on a small
# fraction of ticks and cost almost no speed. The cut-in signal is now the
# agent's position-differenced lane-relative lateral RATE with the
# Frenet-frame geometry term (v_s * kappa * d) subtracted; oncoming traffic
# is excluded from the gate.
# ---------------------------------------------------------------------------
import math as _m
from app.services.physics_engine import PhysicsEngine as _PE, EGO_LANE_OFFSET_M as _ELO


def _phantom_arc(R, lead=40, step=3.0):
    lat0, lng0 = 37.7749, -122.4194
    ML, MG = 111_320.0, 111_320.0 * _m.cos(_m.radians(lat0))
    pts = [(lat0, lng0 + (i * step) / MG) for i in range(lead)]
    x0 = (lead - 1) * step
    ang = 0.0
    while ang < _m.pi / 2:
        ang += step / R
        pts.append((lat0 + R * (1 - _m.cos(ang)) / ML, lng0 + (x0 + R * _m.sin(ang)) / MG))
    ln, lg = pts[-1]
    for i in range(1, 60):
        pts.append((ln + (i * step) / ML, lg))
    return pts


def test_curved_road_phantom_cut_in_is_quantitatively_bounded():
    for R in (80.0, 120.0, 200.0, 300.0):
        for seed in (1, 2, 3):
            eng = _PE(seed=seed)
            route = _phantom_arc(R)
            eng.set_destination(*route[-1])
            eng.set_route(route)
            eng.reset_state(station_m=0.0, speed_kmh=45.0, lateral_offset_m=_ELO)
            total = eng.station_distances[-1]

            # same route, no traffic -> the pure curve-cap speed floor
            ref = _PE(seed=seed)
            ref.set_destination(*route[-1])
            ref.set_route(route)
            ref.reset_state(station_m=0.0, speed_kmh=45.0, lateral_offset_m=_ELO)
            ref.traffic = None

            n = 0
            cutin_ticks = 0
            min_traffic = 1e9
            min_ref = 1e9
            for _ in range(1200):
                eng.update("Maintain Speed", dt=0.1)
                ref.update("Maintain Speed", dt=0.1)
                st = eng.current_station_m
                if 15.0 < st < total - 70.0:
                    n += 1
                    pr = eng.prediction_result
                    if pr is not None and pr.proactive_decel_mps2 > 0.0:
                        cutin_ticks += 1
                    min_traffic = min(min_traffic, eng.speed_kmh)
                    min_ref = min(min_ref, ref.speed_kmh)

            assert n > 100
            frac = cutin_ticks / n
            phantom_speed_loss = min_ref - min_traffic  # traffic-only speed cost
            # BOUND: <= 15% of mid-curve ticks, <= 2 km/h of speed beyond the
            # legitimate curve cap. Realistic radii (R >= 120) should be ~0.
            assert frac <= 0.15, f"R={R} seed={seed}: phantom cut-in on {100*frac:.0f}% of ticks"
            assert phantom_speed_loss <= 2.0, (
                f"R={R} seed={seed}: phantom cut-in cost {phantom_speed_loss:.1f} km/h of speed"
            )
            if R >= 120.0:
                assert frac <= 0.05, f"R={R} seed={seed}: {100*frac:.0f}% on a realistic curve"
