from app.services.simulation_session import SimulationSession


def test_planner_telemetry_reports_semantic_lane_without_changing_it():
    session=SimulationSession(seed=42,real_time=False)
    sm=session.driver._lc_sm
    sm.origin_lane_d_m=5.25;sm.adjacent_lane_d_m=1.75
    sm.require_adjacent_destination()
    before=(sm.state,sm.origin_lane_d_m,sm.adjacent_lane_d_m,sm.required_destination_d_m)
    planner=session.build_telemetry_payload()["channels"]["semantic"]["planner"]
    assert planner["lane_state"]==sm.state.value
    assert planner["semantic_origin_m"]==5.25
    assert planner["semantic_adjacent_m"]==1.75
    assert planner["required_destination_m"]==1.75
    assert before==(sm.state,sm.origin_lane_d_m,sm.adjacent_lane_d_m,sm.required_destination_d_m)
