"""Enforce the learned-pipeline observation boundary.

The learned driving intelligence (ADR-003) may ONLY receive sensor-derived
state. This suite proves, statically and at runtime, that
``app.services.learning`` cannot see privileged simulator state:

* the module does not import ``traffic`` / ``NpcVehicle`` / ``physics_engine``;
* ``build_observation`` rejects a ground-truth traffic object;
* the observation depends only on the perception/prediction outputs -- mutating
  the ground-truth NPCs after perception has run does not change it.
"""
import ast
import math
import pathlib

import pytest

from app.services.learning import build_observation, Observation, MAX_AGENTS
from app.services.learning import observation as observation_mod

_REPO = pathlib.Path(__file__).resolve().parents[1]
_FORBIDDEN_IMPORTS = {
    "app.services.traffic",
    "app.services.physics_engine",
    "app.services.scenario_engine",
}


def _imported_modules(py_path: pathlib.Path) -> set:
    tree = ast.parse(py_path.read_text(encoding="utf-8"))
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods.add(node.module)
    return mods


def test_learning_package_does_not_import_privileged_modules():
    for py in (_REPO / "app" / "services" / "learning").glob("*.py"):
        mods = _imported_modules(py)
        leaked = mods & _FORBIDDEN_IMPORTS
        assert not leaked, f"{py.name} imports privileged modules: {leaked}"
        # nothing named *traffic* / *physics_engine* at all
        assert not any("traffic" in m or "physics_engine" in m for m in mods), (
            f"{py.name} imports a suspicious module: {mods}"
        )


def test_build_observation_rejects_a_traffic_model():
    from app.services.traffic import TrafficModel, NpcVehicle

    tm = TrafficModel(total_length_m=500.0, seed=1)
    with pytest.raises(TypeError):
        build_observation(
            ego_speed_mps=12.0, ego_accel_mps2=0.0, ego_lane_offset_m=1.75,
            ego_lateral_rate_mps=0.0, ego_heading_err_rad=0.0,
            road_curvature_1pm=0.0, friction_estimate=1.0, dist_to_goal_m=500.0,
            lead_gap_m=None, lead_speed_mps=None,
            surround_tracks=tm,  # <-- privileged
            intent_by_track={}, left_lane_clear=True, right_lane_clear=True,
        )
    with pytest.raises(TypeError):
        build_observation(
            ego_speed_mps=12.0, ego_accel_mps2=0.0, ego_lane_offset_m=1.75,
            ego_lateral_rate_mps=0.0, ego_heading_err_rad=0.0,
            road_curvature_1pm=0.0, friction_estimate=1.0, dist_to_goal_m=500.0,
            lead_gap_m=None, lead_speed_mps=None,
            surround_tracks=[NpcVehicle(id="n", lane_offset=1.75, speed_kmh=30.0,
                                        station_m=40.0, desired_speed_kmh=30.0)],
            intent_by_track={}, left_lane_clear=True, right_lane_clear=True,
        )


def test_observation_is_fixed_width_and_sensor_typed():
    obs = build_observation(
        ego_speed_mps=12.0, ego_accel_mps2=-0.3, ego_lane_offset_m=1.75,
        ego_lateral_rate_mps=0.1, ego_heading_err_rad=0.02,
        road_curvature_1pm=-0.005, friction_estimate=0.9, dist_to_goal_m=430.0,
        lead_gap_m=18.0, lead_speed_mps=8.0,
        surround_tracks=[], intent_by_track={},
        left_lane_clear=True, right_lane_clear=False,
    )
    assert isinstance(obs, Observation)
    assert len(obs.agents) == MAX_AGENTS
    assert all(not a.valid for a in obs.agents)  # empty scene -> all masked
    assert obs.lead_closing_mps == pytest.approx(4.0)


def test_observation_depends_only_on_perception_not_ground_truth():
    """Mutating the ground-truth NPCs AFTER perception has produced its
    tracks must not change the observation the learned pipeline sees."""
    from app.services.physics_engine import PhysicsEngine, EGO_LANE_OFFSET_M
    from app.services.traffic import TrafficModel, NpcVehicle

    eng = PhysicsEngine(seed=1)
    eng.set_route([(37.7749 + i * 0.00005, -122.4194) for i in range(160)])
    eng.reset_state(station_m=0.0, speed_kmh=45.0, lateral_offset_m=EGO_LANE_OFFSET_M)
    tm = TrafficModel(total_length_m=eng.station_distances[-1], seed=1)
    tm.spawn_scripted_npcs([
        NpcVehicle(id="lead", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=25.0,
                   station_m=55.0, desired_speed_kmh=25.0, prevent_recycle=True),
    ])
    eng.traffic = tm
    for _ in range(40):
        eng.update("Maintain Speed", dt=0.1)

    obs_before = eng.build_learning_observation()
    # teleport the ground-truth NPC somewhere absurd WITHOUT running perception
    for npc in eng.traffic.npcs:
        npc.station_m += 999.0
        npc.lane_offset = -5.25
        npc.speed_kmh = 200.0
    obs_after = eng.build_learning_observation()

    # identical -- the observation is a function of the (unchanged) perception
    # tracks + prediction + ego state, not of traffic.npcs
    assert obs_before == obs_after


def test_agent_slots_carry_only_relative_sensor_terms():
    from app.services.physics_engine import PhysicsEngine, EGO_LANE_OFFSET_M
    from app.services.traffic import TrafficModel, NpcVehicle

    eng = PhysicsEngine(seed=2)
    eng.set_route([(37.7749 + i * 0.00005, -122.4194) for i in range(160)])
    eng.reset_state(station_m=0.0, speed_kmh=50.0, lateral_offset_m=EGO_LANE_OFFSET_M)
    tm = TrafficModel(total_length_m=eng.station_distances[-1], seed=2)
    tm.spawn_scripted_npcs([
        NpcVehicle(id="lead", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=30.0,
                   station_m=50.0, desired_speed_kmh=30.0, prevent_recycle=True),
    ])
    eng.traffic = tm
    for _ in range(35):
        eng.update("Maintain Speed", dt=0.1)
    obs = eng.build_learning_observation()
    valid = [a for a in obs.agents if a.valid]
    assert valid, "expected at least one tracked agent"
    a = valid[0]
    # relative longitudinal gap is positive (ahead) and finite; probabilities in [0,1]
    assert -5.0 < a.ds_m < 120.0
    assert 0.0 <= a.p_cut_in <= 1.0 and 0.0 <= a.p_lane_keep <= 1.0
    assert 0.0 <= a.track_confidence <= 1.0
