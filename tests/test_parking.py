from app.services.parking import parking_plan, footprint_inside, point_inside, install_parking
from app.services.physics_engine import PhysicsEngine
from app.services.frenet import latlng_to_local


def test_parking_pose_is_inside_real_area_and_off_aisle():
    plan = parking_plan()
    polygon = tuple(latlng_to_local(*p, *plan.maneuver_start) for p in plan.polygon_latlng)
    goal = latlng_to_local(*plan.goal, *plan.maneuver_start)
    assert plan.lot_way_id == 1356718570
    assert footprint_inside(*goal, plan.goal_heading_deg, polygon, margin=.5)
    assert plan.pose_source == 'planner_selected_pose_inside_recorded_parking_area'


def test_full_footprint_rejects_boundary_and_concave_notch():
    polygon = ((-10.,-10.),(10.,-10.),(10.,10.),(1.,10.),(1.,0.),(-1.,0.),(-1.,10.),(-10.,10.))
    assert point_inside((0.,-1.),polygon)
    assert not footprint_inside(0.,-1.,0.,polygon)
    assert footprint_inside(5.,-5.,0.,polygon)


def test_parking_state_requires_containment_heading_and_stop():
    plan = parking_plan()
    physics = PhysicsEngine(seed=42)
    physics.set_destination(*plan.goal)
    physics.set_route(plan.waypoints)
    install_parking(physics,plan)
    physics.lat,physics.lng = plan.goal
    physics.heading = plan.goal_heading_deg
    physics.current_station_m = physics.parking_mission['maneuver_start_m']+30.
    physics.speed_kmh = 0.
    assert physics.observe_parking()['parked']
    assert not physics.observe_parking()['maneuvering']
    physics.speed_kmh = 5.
    assert not physics.observe_parking()['parked']
    assert physics.observe_parking()['maneuvering']
    for speed in (.49,.01,.000001):
        physics.speed_kmh=speed
        observation=physics.observe_parking()
        assert not observation['parked']
        assert observation['status']!='parked_in_mapped_area'
    physics.speed_kmh = 0.
    physics.heading += 90
    assert not physics.observe_parking()['parked']
    physics.set_route(plan.waypoints[:10])
    assert physics.observe_parking() is None


def test_complete_mission_slows_before_parking_phases_and_stays_contained():
    from app.services.simulation_session import SimulationSession
    session = SimulationSession(seed=42, real_time=False)
    session.scenario_engine.load_scenario('mapped_parking', session.world.physics)
    physics = session.world.physics
    reached = False
    for _ in range(2000):
        session.step_pipeline(10)
        parking = physics.observe_parking()
        if not parking['active']:
            # Actual center position, including both public-road turns.
            assert abs(physics.current_lateral_offset_m - session.driver.lateral_target_d_m) < 1.3
        if parking['active']:
            assert physics.speed_kmh <= 10.1
        if parking['maneuvering']:
            assert physics.speed_kmh <= 5.1
            assert parking['contained']
        junction = session.driver.junctions.status
        if junction and junction['phase'] == 'RED':
            assert junction['distance_to_stop_m'] >= 0
        if parking['parked'] and physics.speed_kmh == 0.:
            reached = True
            break
    assert reached
    for _ in range(20):
        session.step_pipeline(10)
        assert physics.speed_kmh == 0.
        assert physics.observe_parking()['contained']
