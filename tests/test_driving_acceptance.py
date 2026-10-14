"""Whole-session acceptance fixtures and independent evaluation-oracle checks."""
import copy
import pytest
from scripts.evaluate_driving_acceptance import (
    evaluate, evaluate_abort, evaluate_recorded_intersection, load, physical_sample, physical_digest, rectangle, rectangles_overlap, summarize,
)
from app.services.simulation_session import SimulationSession


def test_independent_overlap_oracle_checks_rotated_bodies_not_center_distance():
    ego=rectangle(0.,0.,0.,4.5,1.96)
    assert rectangles_overlap(ego,rectangle(1.5,0.,.3,4.8,1.8))
    assert not rectangles_overlap(ego,rectangle(8.,0.,.3,4.8,1.8))
    assert not rectangles_overlap(ego,rectangle(1.96,0.,0.,4.5,1.96))


@pytest.mark.parametrize("scenario", ["normal_cruising", "traffic_overtake", "emergency_cut_in", "highway_merge"])
def test_actual_session_early_scenario_physical_envelopes(scenario):
    summary,_=evaluate(scenario, seconds=8.)
    assert summary["passed"], summary["first_failures"]
    assert not summary["browser_verified"]
    if scenario=="traffic_overtake":assert summary["coverage"]["observed_following"]
    if scenario=="emergency_cut_in":assert summary["coverage"]["observed_emergency"]


def test_actual_session_reset_replays_physical_trajectory_with_epoch_change():
    session=SimulationSession(seed=42,real_time=False)
    load(session,"traffic_overtake")
    first_epoch=session.epoch
    def trace():
        rows=[]
        for _ in range(150):
            session.step_pipeline(2);rows.append(physical_sample(session))
        return physical_digest(rows)
    a=trace()
    session._handle_command({"type":"reset_simulation"})
    assert session.epoch>first_epoch
    assert trace()==a


@pytest.mark.parametrize("invalid", [float("nan"),float("inf"),0.,1.1])
def test_invalid_scenario_change_preserves_paused_world_and_command_boundary(invalid):
    session=SimulationSession(seed=42,real_time=False)
    load(session,"normal_cruising")
    session._handle_command({"type":"pause_simulation"})
    before=copy.deepcopy(physical_sample(session))
    epoch=session.epoch
    with pytest.raises(ValueError):
        load(session,"traffic_overtake",{"friction_mu":invalid})
    after=physical_sample(session)
    assert after==before
    assert session.epoch==epoch
    assert session.is_paused


def test_ml_analytics_labels_do_not_change_actual_session_actuators():
    _,accelerate=evaluate("highway_merge",seconds=3.,analytics_decision="Accelerate")
    _,brake=evaluate("highway_merge",seconds=3.,analytics_decision="Decelerate")
    assert physical_digest(accelerate)==physical_digest(brake)
    assert accelerate[0]["analytics_decision"]!=brake[0]["analytics_decision"]


def test_actual_sensed_adversarial_npc_causes_abort_without_collision():
    summary,_=evaluate_abort()
    assert summary["intervention"]["actor_id"]=="npc-0"
    assert summary["observed_abort"]
    assert summary["observed_abort_return"]
    assert summary["abort_return_time_s"]<=25.
    assert summary["passed"],summary["first_failures"]


def test_acceptance_oracle_does_not_excuse_unbounded_jerk_for_planner_rejection():
    session=SimulationSession(seed=42,real_time=False)
    load(session,"normal_cruising")
    row=physical_sample(session)
    second={**row,"time":row["time"]+.02,"acceleration_mps2":-4.5,"reason":"joint_trajectory_infeasible"}
    limits=dict(steering_rad=.61,steering_rate_radps=.6,jerk_mps3=2.5,braking_mps2=4.5,lateral_acceleration_mps2=3.)
    ordinary=summarize([row,second],limits)
    assert ordinary["failure_counts"]["normal_applied_jerk"]==1
    emergency=summarize([row,{**second,"safety_override":"EMERGENCY_BRAKE"}],limits)
    assert "normal_applied_jerk" not in emergency["failure_counts"]
    unsafe=summarize([row,{**second,"safety_override":"EMERGENCY_BRAKE","acceleration_mps2":-9.}],limits)
    assert unsafe["failure_counts"]["bounded_emergency_braking"]==1


def test_recorded_osm_stop_has_observed_dwell_before_release_and_crossing():
    summary,rows=evaluate_recorded_intersection()
    assert summary["source_node_id"]==65424265
    assert summary["source_way_id"]==130716952
    assert summary["passed"],summary["first_failures"]
    assert summary["behavior_passed"]
    assert summary["observed_stopped_duration_before_release_s"]>=1.98
    assert any(r["reason"]=="stop_sign" and r["speed_mps"]<.15 for r in rows)


def test_recorded_osm_yield_waits_for_simulated_conflict_and_fresh_clearance():
    summary,rows=evaluate_recorded_intersection("give_way",True)
    assert summary["source_node_id"]==6383816258
    assert summary["control_direction_verified"]
    assert summary["passed"],summary["first_failures"]
    assert summary["behavior_passed"]
    assert not summary["crossed_before_conflict_clear"]
    assert summary["first_stopped_s"]<summary["conflict_removed_s"]
    assert summary["released_s"]>=summary["conflict_removed_s"]+.98


def test_render_oracle_includes_tire_outside_chassis_rectangle(monkeypatch):
    from scripts import evaluate_driving_acceptance as module
    monkeypatch.setattr(module,"CHASSIS_HALF_LENGTH_M",2.4)
    monkeypatch.setattr(module,"CHASSIS_HALF_WIDTH_M",.99)
    monkeypatch.setattr(module,"TIRE_WIDTH_M",.22)
    shapes=module.ego_rectangles(0.,0.,0.,0.,2.8)
    assert len(shapes)==5
    assert max(abs(x) for shape in shapes for x,z in shape)==pytest.approx(1.09)
    assert max(abs(z) for x,z in shapes[0])==pytest.approx(2.4)
    # Tiny object touching the tire's occupied side is missed by chassis-only
    # checking. This independently exercises the union/SAT oracle.
    object_body=rectangle(1.07,-1.4,0.,.2,.06)
    assert not rectangles_overlap(shapes[0],object_body)
    assert any(rectangles_overlap(shape,object_body) for shape in shapes[1:])


def test_actual_seeded_erratic_mission_recovers_without_excusing_physical_failures():
    from scripts.evaluate_driving_acceptance import fixture_liveness
    summary, rows = evaluate("erratic_cut_in", seconds=25.)
    assert summary["passed"], summary["first_failures"]
    assert summary["behavior_passed"], summary
    rejection = next(i for i,r in enumerate(rows) if r["reason"] == "joint_trajectory_infeasible")
    abort = next(i for i,r in enumerate(rows[rejection:],rejection) if r["lane_change_state"] == "ABORT_LANE_CHANGE")
    assert any(r["lane_change_state"] == "LANE_KEEP" for r in rows[abort+1:])
    assert rows[-1]["station_m"] - rows[-151]["station_m"] > 6.
    assert rows[-1]["speed_mps"] > 2.
    stalled = [{**r, "speed_mps": 0., "reason": "joint_trajectory_infeasible"} for r in rows]
    assert not fixture_liveness(stalled)["behavior_passed"]
