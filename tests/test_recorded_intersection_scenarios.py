"""Selectable recorded intersections use the application command/pipeline path."""
import pytest
from app.services.simulation_session import SimulationSession


def command_load(s,scenario,params=None):
    s.enqueue_command({"type":"load_scenario","scenario_id":scenario,
        "params":params or {},"command_id":"recorded-intersection-load"})
    s.process_pending_commands()
    assert s.scenario_engine.active_scenario_id==scenario
    return s.world.physics


def trace_control(s,node_id,seconds):
    p=s.world.physics
    control=next(c for c in p.route_controls if c.node_id==node_id)
    line=control.station_m-6.
    rows=[]
    for _ in range(round(seconds/.02)):
        s.step_pipeline(2)
        rows.append((s.executor.clock.sim_time_s,p.current_station_m,p.speed_kmh/3.6,
            node_id in s.driver.junctions.released,
            tuple(n.id for n in p.traffic.npcs),s.driver.speed_limit_reason))
    return line,rows


def test_selectable_recorded_stop_dwell_and_route_label():
    s=SimulationSession(seed=42,real_time=False)
    p=command_load(s,"recorded_stop")
    assert p.speed_kmh==18.
    assert s.scenario_engine.get_state()["route_name"]=="Ray Drive"
    assert s.route_steps[0]["instruction"]=="Ray Drive"
    assert not p.traffic.npcs
    line,rows=trace_control(s,65424265,40.)
    release=next(i for i,r in enumerate(rows) if r[3])
    # Independent observed dwell immediately before release; no use of the
    # driver's internal stopped_since timer as the acceptance oracle.
    start=release
    while start>0 and abs(rows[start-1][1]-line)<=1.5 and rows[start-1][2]<.15:start-=1
    assert rows[release][0]-rows[start][0]>=1.98
    assert all(r[1]<=line+.05 for r in rows[:release])
    assert any(r[1]>line+.5 for r in rows[release:])


def test_selectable_recorded_yield_waits_until_scheduled_simulated_conflict_clears():
    s=SimulationSession(seed=42,real_time=False)
    p=command_load(s,"recorded_yield_conflict")
    state=s.scenario_engine.get_state()
    assert state["conflict_source"]=="explicit_simulated_stationary_actor"
    assert state["route_name"] is None  # Do not invent a name for unnamed OSM way.
    assert s.route_steps[0]["instruction"]=="Mapped yield approach"
    line,rows=trace_control(s,6383816258,40.)
    assert all(r[1]<=line+.05 for r in rows if r[0]<20.)
    assert any(r[2]<.15 and abs(r[1]-line)<=1.5 for r in rows if r[0]<20.)
    cleared=next(r[0] for r in rows if "fixture-yield-conflict" not in r[4])
    assert 20.<=cleared<=20.12  # 10Hz scenario tick, authoritative simulator clock.
    release=next(r[0] for r in rows if r[3])
    assert release>=cleared+.98
    assert any(r[1]>line+.5 for r in rows if r[0]>release)


@pytest.mark.parametrize("scenario",["recorded_stop","recorded_yield_conflict"])
def test_selectable_recorded_intersection_reset_replays_initial_motion(scenario):
    s=SimulationSession(seed=42,real_time=False)
    command_load(s,scenario,{"friction_mu":.3})
    assert s.world.physics.friction_mu==.3  # Params applied after scenario setup.
    def trace():
        rows=[]
        for _ in range(150):
            s.step_pipeline(2);p=s.world.physics
            rows.append((p.lat,p.lng,p.speed_kmh,p.steering_angle_rad,p.acceleration_mps2))
        return rows
    initial=trace();epoch=s.epoch
    s.enqueue_command({"type":"reset_simulation","command_id":"reset-recorded-intersection"})
    s.process_pending_commands()
    assert s.epoch>epoch
    assert trace()==initial
    if scenario=="recorded_yield_conflict":
        assert any(n.id=="fixture-yield-conflict" for n in s.world.physics.traffic.npcs)
