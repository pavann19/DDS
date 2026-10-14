"""Rejected maneuvers recover through the joint planner, never a fallback."""
import pytest
from app.services.planner.state_machine import LaneChangeStateMachine,LaneChangeState


def machine(origin=1.75,adjacent=5.25):
    sm=LaneChangeStateMachine(origin,adjacent)
    sm.state=LaneChangeState.EXECUTE_LANE_CHANGE
    return sm

@pytest.mark.parametrize('origin,adjacent',[(1.75,5.25),(5.25,1.75)])
def test_early_rejection_enters_abort_and_requires_validated_return(origin,adjacent):
    sm=machine(origin,adjacent)
    d=origin+.1*(adjacent-origin)
    assert sm.reject_joint_plan(current_d_m=d)
    assert sm.state is LaneChangeState.ABORT_LANE_CHANGE
    assert sm.step(current_d_m=origin,lane_blocked=False,adjacent_clear=True)==[origin]
    assert sm.state is LaneChangeState.ABORT_LANE_CHANGE
    sm.confirm_joint_plan()
    sm.step(current_d_m=origin,lane_blocked=False,adjacent_clear=True,lateral_speed_mps=.3)
    assert sm.state is LaneChangeState.ABORT_LANE_CHANGE
    sm.step(current_d_m=origin,lane_blocked=False,adjacent_clear=True,lateral_speed_mps=.05)
    assert sm.state is LaneChangeState.LANE_KEEP

@pytest.mark.parametrize('origin,adjacent',[(1.75,5.25),(5.25,1.75)])
def test_late_rejection_never_reverses_or_completes_without_a_valid_plan(origin,adjacent):
    sm=machine(origin,adjacent)
    d=origin+.99*(adjacent-origin)
    assert not sm.reject_joint_plan(current_d_m=d)
    assert sm.state is LaneChangeState.EXECUTE_LANE_CHANGE
    sm.step(current_d_m=adjacent,lane_blocked=False,adjacent_clear=True)
    assert sm.state is LaneChangeState.EXECUTE_LANE_CHANGE
    sm.confirm_joint_plan()
    sm.step(current_d_m=adjacent,lane_blocked=False,adjacent_clear=True)
    assert sm.state is LaneChangeState.LANE_KEEP
    assert sm.origin_lane_d_m==adjacent


def test_a_second_abort_rejection_requires_revalidation_again():
    sm=machine()
    sm.reject_joint_plan(current_d_m=2.)
    sm.confirm_joint_plan()
    sm.reject_joint_plan(current_d_m=1.8)
    sm.step(current_d_m=1.75,lane_blocked=False,adjacent_clear=True)
    assert sm.state is LaneChangeState.ABORT_LANE_CHANGE


def test_reset_discards_pending_rejection():
    sm=machine();sm.reject_joint_plan(current_d_m=2.);sm.reset()
    assert sm.state is LaneChangeState.LANE_KEEP
    assert not sm._rejected_plan_pending


def test_rejected_driver_never_calls_unchecked_decoupled_passing(monkeypatch):
    from app.services.simulation_session import SimulationSession
    from app.services.driver import driver as module
    session=SimulationSession(seed=42,real_time=False)
    session.step_pipeline(20)
    p=session.world.physics
    driver=p._driver;driver._lc_sm.state=LaneChangeState.EXECUTE_LANE_CHANGE
    driver._maneuver_traj=None
    monkeypatch.setattr(module,'plan_spatiotemporal',lambda *args:None)
    def unchecked(*args,**kwargs):raise AssertionError('unchecked passing fallback')
    monkeypatch.setattr(module,'plan_lateral_offset',unchecked)
    from dataclasses import replace
    ctx=replace(p.build_driver_context(.1),lane_clear_fn=lambda *args:True)
    obs=p.observe()
    driver.decide_lateral(obs,ctx)
    assert driver._joint_plan_unavailable
    assert driver._lc_sm.state is LaneChangeState.ABORT_LANE_CHANGE
    assert driver.lateral_target_d_m==pytest.approx(ctx.lateral_offset_m)
    assert driver.decide_longitudinal(obs,ctx).speed_limit_reason=='joint_trajectory_infeasible'


def test_existing_positional_config_contract_is_preserved():
    from app.services.planner.state_machine import StateMachineConfig
    cfg=StateMachineConfig(3,.2,.6)
    assert cfg.abort_progress_max==.6
    assert cfg.complete_lateral_speed_mps==.15


def test_actual_session_late_rejection_holds_without_an_unchecked_return(monkeypatch):
    from app.services.simulation_session import SimulationSession
    from app.services.driver import driver as module
    session=SimulationSession(seed=42,real_time=False)
    session._handle_command(dict(type='load_scenario',scenario_id='traffic_overtake',params={}))
    p=session.world.physics;driver=session.driver
    reached=False
    for _ in range(1000):
        session.step_pipeline(2)
        sm=driver._lc_sm
        progress=abs(p.current_lateral_offset_m-sm.origin_lane_d_m)/sm._span()
        if sm.state is LaneChangeState.EXECUTE_LANE_CHANGE and .9<=progress<.96:
            reached=True;break
    assert reached,'actual maneuver never reached the late-rejection fixture'
    driver._maneuver_traj=None
    monkeypatch.setattr(module,'plan_spatiotemporal',lambda *args:None)
    def unchecked(*args,**kwargs):raise AssertionError('unchecked late fallback')
    monkeypatch.setattr(module,'plan_lateral_offset',unchecked)
    driver.decide_lateral(p.observe(),p.build_driver_context(.1))
    assert driver._joint_plan_unavailable
    assert driver._lc_sm.state is LaneChangeState.EXECUTE_LANE_CHANGE
    assert driver._lc_sm._rejected_plan_pending
    assert driver.lateral_target_d_m==pytest.approx(p.current_lateral_offset_m)
    decision=driver.decide_longitudinal(p.observe(),p.build_driver_context(.1))
    assert decision.speed_limit_reason=='joint_trajectory_infeasible'
    assert driver.target_speed_mps==0.
    # Continue the real plant while every joint retry is deliberately rejected.
    # The held late maneuver must settle at exactly zero without bypassing validation.
    from scripts.evaluate_driving_acceptance import physical_sample, summarize
    rows=[]
    for _ in range(500):
        session.step_pipeline(2)
        rows.append(physical_sample(session))
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report["passed"],report["first_failures"]
    assert p.speed_kmh == 0.
    assert all(r["speed_mps"] == 0. for r in rows[-50:])
    assert driver._lc_sm._rejected_plan_pending

def test_recorded_zero_speed_abort_recovers_after_camera_outage(monkeypatch):
    # Recorded live blocked pose from2026-10-14, with injected capture loss.
    # Full controller/plant/safety/planner pipeline; no unchecked fallback.
    from dataclasses import replace
    from types import SimpleNamespace
    from app.services.simulation_session import SimulationSession
    from app.services.interfaces import SensorHealth
    from scripts.evaluate_driving_acceptance import physical_sample,summarize
    session=SimulationSession(seed=42)
    p=session.world.physics
    p.reset_state(station_m=1521.701914967645,lateral_offset_m=2.2413633441276914,
        heading_deg=129.47566541222807)
    p.acceleration_mps2=-.15
    p.steering_angle_rad=-.0028949014955719295
    p.traffic.npcs=[]
    driver=session.driver
    driver._lc_sm.state=LaneChangeState.ABORT_LANE_CHANGE
    driver._lc_sm._rejected_plan_pending=True
    driver.lateral_target_d_m=1.75
    # A return/tracking joint polynomial is not proof of a new lane transition.
    driver._maneuver_traj=SimpleNamespace(d1_m=1.75)
    driver._changing_lane_now=True
    original=session.world.observe
    def observed():
        obs=original();now=p.clock.sim_time_s
        camera=None if now<1. else now
        return replace(obs,sensor_health=SensorHealth(now,now,True,camera))
    monkeypatch.setattr(session.world,'observe',observed)
    rows=[]
    for _ in range(700):
        session.step_pipeline(2)
        if p.clock.sim_time_s<1.:
            assert driver.lateral_target_d_m == 1.75
            assert p.speed_kmh == 0.
        if driver._maneuver_traj is not None:
            traj=driver._maneuver_traj
            lanes=(driver._lc_sm.origin_lane_d_m,driver._lc_sm.adjacent_lane_d_m)
            initial=min(lanes,key=lambda d:abs(d-traj.lat_poly.pos(0.)))
            terminal=min(lanes,key=lambda d:abs(d-traj.d1_m))
            if initial==terminal:
                assert not driver._changing_lane_now
                assert not any(c.is_chosen and c.is_lane_change for c in driver.planner_candidates)
        rows.append(physical_sample(session))
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report['passed'],report['first_failures']
    assert p.current_station_m>1535.
    assert p.speed_kmh>10.
    assert driver._lc_sm.state is LaneChangeState.LANE_KEEP
    assert abs(p.current_lateral_offset_m-1.75)<.2


def test_actual_overtake_joint_plan_remains_labeled_lane_transition():
    from app.services.simulation_session import SimulationSession
    session=SimulationSession(seed=42,real_time=False)
    session._handle_command(dict(type='load_scenario',scenario_id='traffic_overtake',params={}))
    driver=session.driver
    for _ in range(1000):
        session.step_pipeline(2)
        traj=driver._maneuver_traj
        if traj is not None and driver._lc_sm.state is LaneChangeState.EXECUTE_LANE_CHANGE:
            assert driver._changing_lane_now
            assert any(c.is_chosen and c.is_lane_change for c in driver.planner_candidates)
            return
    pytest.fail('actual overtake never committed a lane transition')


def test_required_return_rejection_and_clearance_loss_preserve_destination():
    sm=machine(origin=5.25,adjacent=1.75)
    sm.require_adjacent_destination()
    assert not sm.reject_joint_plan(current_d_m=4.55)
    assert sm.state is LaneChangeState.EXECUTE_LANE_CHANGE
    assert sm._rejected_plan_pending
    assert sm.step(current_d_m=4.55,lane_blocked=True,adjacent_clear=False)==[1.75]
    assert sm.state is LaneChangeState.EXECUTE_LANE_CHANGE
    assert sm.required_destination_d_m == 1.75


@pytest.mark.parametrize('second_loss',[False,True])
def test_geometry_required_return_survives_camera_loss_and_joint_rejection(monkeypatch,second_loss):
    # Actual recorded corner; eight producer views have controlled dropouts.
    # This is pipeline fault injection, not live browser camera acceptance.
    from dataclasses import replace
    from app.services.simulation_session import SimulationSession
    from app.services.interfaces import SensorHealth
    from app.services.camera_rig import CAMERA_IDS
    from app.services.driver import driver as module
    from scripts.evaluate_driving_acceptance import physical_sample,summarize
    session=SimulationSession(seed=42)
    p=session.world.physics
    p.reset_state(station_m=4200.,speed_kmh=50.,lateral_offset_m=5.25)
    p.traffic.npcs=[]
    driver=session.driver
    driver._lc_sm.origin_lane_d_m=5.25
    driver._lc_sm.adjacent_lane_d_m=1.75
    driver.lateral_target_d_m=5.25
    original_observe=session.world.observe
    loss_start=None
    loss_duration=8. if second_loss else 5.
    second_loss_start=None
    second_loss_offset=None
    rejected=False
    original_plan=module.plan_spatiotemporal
    def observed():
        obs=original_observe();now=p.clock.sim_time_s
        lost=(loss_start is not None and now<loss_start+loss_duration) or (
            second_loss_start is not None and now<second_loss_start+1.5)
        stamp=None if lost else now
        return replace(obs,sensor_health=SensorHealth(now,now,True,stamp,
            CAMERA_IDS,tuple((name,stamp) for name in CAMERA_IDS)))
    def planning(*args):
        nonlocal rejected
        if loss_start is not None and p.clock.sim_time_s>=loss_start+loss_duration and not rejected:
            rejected=True
            return None
        return original_plan(*args)
    monkeypatch.setattr(session.world,'observe',observed)
    monkeypatch.setattr(module,'plan_spatiotemporal',planning)
    rows=[]
    for _ in range(3000 if second_loss else 2500):
        session.step_pipeline(2)
        if (loss_start is None and driver._lc_sm.required_destination_d_m == 1.75
                and .18<=driver._lc_sm.progress<=.3):
            loss_start=p.clock.sim_time_s
        traj=driver._maneuver_traj
        if (second_loss and second_loss_start is None and loss_start is not None
                and p.clock.sim_time_s>loss_start+loss_duration and traj is not None
                and abs(traj.d1_m-traj.lat_poly.pos(0.))<1e-9):
            second_loss_start=p.clock.sim_time_s
            second_loss_offset=p.current_lateral_offset_m
        if second_loss_start is not None and second_loss_start<p.clock.sim_time_s<second_loss_start+1.5:
            assert driver.lateral_target_d_m<=second_loss_offset+.01
        if loss_start is not None and driver._lc_sm.required_destination_d_m is not None:
            assert driver._lc_sm.state is not LaneChangeState.ABORT_LANE_CHANGE
            traj=driver._maneuver_traj
            if traj is not None:
                # A checked held-offset launch is allowed; steering back
                # toward the infeasible original lane is not.
                lanes=(driver._lc_sm.origin_lane_d_m,driver._lc_sm.adjacent_lane_d_m)
                terminal_lane=min(lanes,key=lambda d:abs(d-traj.d1_m))
                assert (terminal_lane==driver._lc_sm.required_destination_d_m
                        or abs(traj.d1_m-traj.lat_poly.pos(0.))<1e-9)
        rows.append(physical_sample(session))
        if p.current_station_m>4430.:break
    assert loss_start is not None
    assert rejected
    if second_loss:assert second_loss_start is not None
    assert p.current_station_m>4430.
    assert abs(p.current_lateral_offset_m-1.75)<.5
    assert driver._lc_sm.origin_lane_d_m == 1.75
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report['passed'],report['first_failures']


def test_required_return_occupied_adjacent_lane_holds_without_checked_launch(monkeypatch):
    from dataclasses import replace
    from app.services.physics_engine import PhysicsEngine
    from app.services.parking import parking_plan
    from app.services.driver import driver as module
    p=PhysicsEngine(seed=42);p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4260.,speed_kmh=0.,lateral_offset_m=4.55)
    p.step_world_sensors(.02,perceive=False,predict=False)
    driver=p._driver
    driver._lc_sm.origin_lane_d_m=5.25
    driver._lc_sm.adjacent_lane_d_m=1.75
    driver._lc_sm.state=LaneChangeState.EXECUTE_LANE_CHANGE
    driver._lc_sm.require_adjacent_destination()
    ctx=replace(p.build_driver_context(.1),lane_clear_fn=lambda *_:False)
    def forbidden(*args):raise AssertionError('planning into occupied required-return lane')
    monkeypatch.setattr(module,'plan_spatiotemporal',forbidden)
    # Isolate the adjacent-clear veto from producer startup recovery.
    observed=replace(p.observe(),sensor_health=None)
    driver.decide_lateral(observed,ctx)
    lon=driver.decide_longitudinal(observed,ctx)
    assert driver._lc_sm.state is LaneChangeState.EXECUTE_LANE_CHANGE
    assert driver._lc_sm.required_destination_d_m == 1.75
    assert driver._joint_plan_unavailable
    assert driver._maneuver_traj is None
    assert lon.accel_mps2<0.
    assert driver.target_speed_mps == 0.


@pytest.mark.parametrize('speed',[.3,.8,1.7])
def test_low_speed_replan_can_continue_checked_launch_without_erasing_motion(speed):
    from app.services.planner.spatiotemporal import plan,PlannerStart,PlannerContext
    start=PlannerStart(0.,speed,-.2,4.5,-.0003,.00003)
    context=PlannerContext(13.9,1.75,13.9,.1,allowed_lane_centers_m=[1.75],actual_steer_rad=-.003)
    trajectory=plan(start,context)
    assert trajectory is not None
    assert trajectory.v1_mps>speed
    assert trajectory.lat_poly.vel(0.) == start.dd0
    assert trajectory.lat_poly.acc(0.) == start.ddd0
    assert trajectory.initial_actual_steering_checked
    assert trajectory.sampled_aggregate_grip_verified
    assert trajectory.continuous_modeled_corridor_verified


def test_actual_second_camera_outage_during_required_offset_launch_keeps_measured_hold():
    from dataclasses import replace
    from types import SimpleNamespace
    from app.services.physics_engine import PhysicsEngine
    from app.services.parking import parking_plan
    p=PhysicsEngine(seed=42);p.set_route(parking_plan().waypoints)
    p.reset_state(station_m=4352.928,lateral_offset_m=4.86458)
    p.step_world_sensors(.02,perceive=False,predict=False)
    driver=p._driver
    driver._lc_sm.origin_lane_d_m=5.25
    driver._lc_sm.adjacent_lane_d_m=1.75
    driver._lc_sm.state=LaneChangeState.EXECUTE_LANE_CHANGE
    driver._lc_sm.require_adjacent_destination()
    ctx=p.build_driver_context(.02)
    driver._maneuver_traj=SimpleNamespace(d1_m=ctx.lateral_offset_m)
    driver.lateral_target_d_m=ctx.lateral_offset_m
    driver._hold_lateral_during_sensor_loss(ctx)
    assert driver.lateral_target_d_m==ctx.lateral_offset_m
    assert driver._lc_sm.required_destination_d_m == 1.75
    driver._hold_lateral_during_sensor_loss(replace(ctx,lateral_offset_m=ctx.lateral_offset_m+.1))
    assert driver.lateral_target_d_m==ctx.lateral_offset_m
    assert driver.lateral_target_d_m<5.25


@pytest.mark.parametrize('initial_state',[LaneChangeState.LANE_KEEP,LaneChangeState.PREPARE_LANE_CHANGE,LaneChangeState.EXECUTE_LANE_CHANGE])
def test_geometry_required_return_brakes_before_commit_when_clearance_blocked(monkeypatch,initial_state):
    # Recorded moving approach; clearance producer fault injection stays
    # blocked until the car has stopped, then the real empty-lane query resumes.
    from dataclasses import replace
    from app.services.simulation_session import SimulationSession
    from app.services.interfaces import SensorHealth
    from app.services.camera_rig import CAMERA_IDS
    from scripts.evaluate_driving_acceptance import physical_sample,summarize
    session=SimulationSession(seed=42);p=session.world.physics
    p.reset_state(station_m=4200.,speed_kmh=50.,lateral_offset_m=5.25)
    p.traffic.npcs=[];driver=session.driver
    sm=driver._lc_sm;sm.origin_lane_d_m=5.25;sm.adjacent_lane_d_m=1.75
    sm.state=initial_state
    if initial_state is not LaneChangeState.LANE_KEEP:sm.require_adjacent_destination()
    driver.lateral_target_d_m=5.25
    original_observe=session.world.observe
    real_clear=p._sense_lane_clear
    def observe():
        obs=original_observe();now=p.clock.sim_time_s
        return replace(obs,sensor_health=SensorHealth(now,now,True,now,
            CAMERA_IDS,tuple((name,now) for name in CAMERA_IDS)))
    def clearance(station,offset):
        return False if p.clock.sim_time_s<20. else real_clear(station,offset)
    monkeypatch.setattr(session.world,'observe',observe)
    monkeypatch.setattr(p,'_sense_lane_clear',clearance)
    rows=[];stopped=False;saw_return=False
    for _ in range(3500):
        session.step_pipeline(2);now=p.clock.sim_time_s
        if now<20.:
            assert sm.state is not LaneChangeState.EXECUTE_LANE_CHANGE or initial_state is LaneChangeState.EXECUTE_LANE_CHANGE
            assert not driver._changing_lane_now
            if driver._joint_plan_unavailable:
                assert driver.target_speed_mps==0.
                assert driver._maneuver_traj is None
            if now>18.:
                assert p.speed_kmh<.1
                assert p.current_station_m<4300.
                stopped=True
        elif driver.spatiotemporal_traj is not None:
            saw_return=True
            assert driver.spatiotemporal_traj.initial_actual_steering_checked
            assert driver.spatiotemporal_traj.continuous_modeled_corridor_verified
        rows.append(physical_sample(session))
        if p.current_station_m>4430.:break
    assert stopped and saw_return
    assert p.current_station_m>4430.
    assert abs(p.current_lateral_offset_m-1.75)<.5
    assert sm.origin_lane_d_m==1.75
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report['passed'],report['first_failures']


def test_curved_rest_launch_uses_measured_wheel_angle_without_relaxing_rate():
    import math
    from app.services.simulation_session import SimulationSession
    from app.services.planner.spatiotemporal import plan,PlannerStart,PlannerContext
    from app.services.planner.joint_physics import reference_curvature_and_derivative
    p=SimulationSession(seed=42).world.physics;cfg=p._driver.planner_cfg
    station=4355.5556;d=1.75
    k,_=reference_curvature_and_derivative(p.frenet_frame,station)
    q=cfg.wheelbase_m*k/(1+k*d)
    aligned=math.atan(q/math.sqrt(1-.25*q*q))
    start=PlannerStart(station,0.,0.,d,0.,0.)
    def launch(steer):
        return plan(start,PlannerContext(target_speed_mps=1.,lane_center_d_m=d,
            max_speed_mps=1.,dt=.02,frenet_frame=p.frenet_frame,
            allowed_lane_centers_m=[d],actual_steer_rad=steer),cfg)
    unaligned=launch(0.)
    assert unaligned is not None and abs(unaligned.lon_poly.vel(unaligned.t_lon_s))<1e-9
    checked=launch(aligned)
    assert checked is not None and checked.lon_poly.vel(checked.t_lon_s)>.5
    assert checked.initial_actual_steering_checked
    assert checked.sampled_modeled_vehicle_envelope_verified
    assert checked.continuous_modeled_corridor_verified


@pytest.mark.parametrize('state',[LaneChangeState.ABORT_LANE_CHANGE,LaneChangeState.EXECUTE_LANE_CHANGE])
def test_actual_incomplete_abort_pose_holds_stationary_wheels_during_camera_loss(state):
    import math
    from dataclasses import replace
    from app.services.simulation_session import SimulationSession
    from app.services.interfaces import SensorHealth
    from app.services.camera_rig import CAMERA_IDS
    session=SimulationSession(seed=42);p=session.world.physics
    p.reset_state(station_m=2394.9912894665263,lateral_offset_m=2.088447042575271,
        speed_kmh=0.,heading_deg=131.0741518492842)
    p.steering_angle_rad=.15214288782813096
    p.kinematic_sideslip_rad=math.atan(.5*math.tan(p.steering_angle_rad))
    p.step_world_sensors(.02,perceive=False,predict=False)
    driver=session.driver;driver._lc_sm.state=state
    driver._lc_sm._rejected_plan_pending=True
    driver.lateral_target_d_m=1.75
    ctx=p.build_driver_context(.02)
    observed=replace(p.observe(),sensor_health=SensorHealth(0.,0.,True,None,
        CAMERA_IDS,tuple((name,None) for name in CAMERA_IDS)))
    cmd=driver.control(observed,ctx)
    assert cmd.steer_rad==ctx.steering_angle_rad
    assert cmd.accel_mps2<0.
    assert driver._lc_sm.required_destination_d_m is None
    assert driver._lc_sm.state is state


def test_actual_ordinary_abort_recovers_after_repeated_camera_interruption(monkeypatch):
    import math
    from dataclasses import replace
    from app.services.simulation_session import SimulationSession
    from app.services.interfaces import SensorHealth
    from app.services.camera_rig import CAMERA_IDS
    from scripts.evaluate_driving_acceptance import physical_sample,summarize
    session=SimulationSession(seed=42);p=session.world.physics
    start=2394.9912894665263
    p.reset_state(station_m=start,lateral_offset_m=2.088447042575271,
        speed_kmh=0.,heading_deg=131.0741518492842)
    p.steering_angle_rad=.15214288782813096
    p.kinematic_sideslip_rad=math.atan(.5*math.tan(p.steering_angle_rad))
    p.acceleration_mps2=-.15;p.traffic.npcs=[]
    driver=session.driver;driver._lc_sm.state=LaneChangeState.ABORT_LANE_CHANGE
    driver._lc_sm._rejected_plan_pending=True;driver.lateral_target_d_m=1.75
    original_observe=session.world.observe
    def observe():
        obs=original_observe();now=p.clock.sim_time_s
        lost=now<1. or (now<8. and now%2.<.3)
        stamp=None if lost else now
        return replace(obs,sensor_health=SensorHealth(now,now,True,stamp,
            CAMERA_IDS,tuple((name,stamp) for name in CAMERA_IDS)))
    monkeypatch.setattr(session.world,'observe',observe)
    rows=[]
    for _ in range(1500):
        previous_steer=p.steering_angle_rad
        session.step_pipeline(2)
        if p.clock.sim_time_s<1.:
            assert p.speed_kmh==0.
            assert p.steering_angle_rad==previous_steer
        assert driver._lc_sm.required_destination_d_m is None
        rows.append(physical_sample(session))
    assert driver._lc_sm.state is LaneChangeState.LANE_KEEP
    assert p.current_station_m>start+50.
    assert abs(p.current_lateral_offset_m-1.75)<.3
    assert p.speed_kmh>20.
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    report=summarize(rows,limits)
    assert report['passed'],report['first_failures']
