"""
Unit tests for app/services/traffic.py : server-side NPC traffic +
the forward range-sensor model that gives PhysicsEngine's IDM controller
 real sensing instead of oracle access to NPC state.
"""
import pytest

from app.services.traffic import (
    TrafficModel, NpcVehicle, SensedLeadVehicle,
    EGO_LANE_OFFSET_M, ADJACENT_LANE_OFFSET_M, NPC_COUNT, SENSOR_MAX_RANGE_M, VISIBILITY_WINDOW_M,
)


def test_spawns_the_expected_npc_count():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    assert len(model.npcs) == NPC_COUNT


def test_no_traffic_when_route_has_no_length():
    model = TrafficModel(total_length_m=0.0)
    assert model.npcs == []
    assert model.sense_lead_vehicle(ego_station_m=0.0) is None


def test_seeded_spawn_is_deterministic():
    a = TrafficModel(total_length_m=1000.0, seed=7)
    b = TrafficModel(total_length_m=1000.0, seed=7)
    a_state = [(n.lane_offset, round(n.speed_kmh, 6), round(n.station_m, 6)) for n in a.npcs]
    b_state = [(n.lane_offset, round(n.speed_kmh, 6), round(n.station_m, 6)) for n in b.npcs]
    assert a_state == b_state


def test_sensor_ignores_npc_in_a_different_lane():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="npc-0", lane_offset=-1.75, speed_kmh=40.0, station_m=50.0)]  # oncoming lane
    assert model.sense_lead_vehicle(ego_station_m=0.0, ego_lane_offset=EGO_LANE_OFFSET_M) is None


def test_sensor_ignores_npc_behind_the_ego():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="npc-0", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=40.0, station_m=10.0)]
    assert model.sense_lead_vehicle(ego_station_m=50.0, ego_lane_offset=EGO_LANE_OFFSET_M) is None


def test_sensor_ignores_npc_beyond_max_range():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(
        id="npc-0", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=40.0,
        station_m=SENSOR_MAX_RANGE_M + 50.0,
    )]
    assert model.sense_lead_vehicle(ego_station_m=0.0, ego_lane_offset=EGO_LANE_OFFSET_M) is None


def test_sensor_detects_a_same_lane_vehicle_ahead_within_range():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="npc-0", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=35.0, station_m=40.0)]
    sensed = model.sense_lead_vehicle(ego_station_m=10.0, ego_lane_offset=EGO_LANE_OFFSET_M)
    assert isinstance(sensed, SensedLeadVehicle)
    assert sensed.gap_m == pytest.approx(30.0)
    assert sensed.lead_speed_kmh == pytest.approx(35.0)


def test_sensor_returns_the_nearest_of_several_candidates():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [
        NpcVehicle(id="npc-far", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=40.0, station_m=90.0),
        NpcVehicle(id="npc-near", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=20.0, station_m=25.0),
    ]
    sensed = model.sense_lead_vehicle(ego_station_m=0.0, ego_lane_offset=EGO_LANE_OFFSET_M)
    assert sensed.gap_m == pytest.approx(25.0)
    assert sensed.lead_speed_kmh == pytest.approx(20.0)


def test_sensor_never_exposes_the_npc_list_itself():
    """The perception/control boundary: sense_lead_vehicle() must return a
    SensedLeadVehicle (gap + speed only), never an NpcVehicle or the list."""
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="npc-0", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=35.0, station_m=40.0)]
    sensed = model.sense_lead_vehicle(ego_station_m=0.0, ego_lane_offset=EGO_LANE_OFFSET_M)
    assert not hasattr(sensed, "id")
    assert not hasattr(sensed, "lane_offset")


def test_npcs_advance_along_the_route_each_tick():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="npc-0", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=36.0, station_m=100.0)]
    model.update(dt=1.0, ego_station_m=100.0)
    assert model.npcs[0].station_m == pytest.approx(110.0)  # 36 km/h = 10 m/s


def test_npc_recycles_when_it_drifts_out_of_the_visibility_window():
    model = TrafficModel(total_length_m=10000.0, seed=1)
    model.npcs = [NpcVehicle(id="npc-0", lane_offset=EGO_LANE_OFFSET_M, speed_kmh=36.0, station_m=0.0)]
    model.update(dt=1.0, ego_station_m=5000.0)  # far outside VISIBILITY_WINDOW_M
    assert abs(model.npcs[0].station_m - 5000.0) <= VISIBILITY_WINDOW_M


def test_get_npc_states_returns_full_state_for_rendering():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    states = model.get_npc_states()
    assert len(states) == NPC_COUNT
    for s in states:
        assert set(s.keys()) == {"id", "lane_offset", "speed_kmh", "station_m"}


# ---------------------------------------------------------------------------
# NPC-to-NPC IDM car-following (P6-4): traffic now forms queues instead of
# driving at an independently-scripted constant speed forever.
# ---------------------------------------------------------------------------

def test_npc_slows_down_behind_a_slower_same_lane_npc_ahead():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [
        NpcVehicle(id="lead", lane_offset=ADJACENT_LANE_OFFSET_M, speed_kmh=20.0,
                  station_m=40.0, desired_speed_kmh=20.0),
        NpcVehicle(id="follower", lane_offset=ADJACENT_LANE_OFFSET_M, speed_kmh=50.0,
                  station_m=10.0, desired_speed_kmh=50.0),
    ]
    for _ in range(50):
        model.update(dt=0.2, ego_station_m=0.0)
    follower = next(n for n in model.npcs if n.id == "follower")
    assert follower.speed_kmh < 50.0, "must slow down closing on a slower NPC ahead in the same lane"
    assert follower.speed_kmh >= 0.0


def test_npc_never_overlaps_a_slower_lead_npc():
    """The core IDM safety property: with a stopped lead vehicle, the
    follower must brake to a stop before reaching it, not drive through it."""
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [
        NpcVehicle(id="lead", lane_offset=ADJACENT_LANE_OFFSET_M, speed_kmh=0.0,
                  station_m=30.0, desired_speed_kmh=0.0),
        NpcVehicle(id="follower", lane_offset=ADJACENT_LANE_OFFSET_M, speed_kmh=50.0,
                  station_m=0.0, desired_speed_kmh=50.0),
    ]
    for _ in range(300):
        model.update(dt=0.1, ego_station_m=0.0)
        follower = next(n for n in model.npcs if n.id == "follower")
        lead = next(n for n in model.npcs if n.id == "lead")
        assert follower.station_m <= lead.station_m, "follower must never overlap/pass through the lead vehicle"


def test_npc_with_no_lead_vehicle_eases_back_to_its_own_cruise_speed():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [
        NpcVehicle(id="solo", lane_offset=ADJACENT_LANE_OFFSET_M, speed_kmh=20.0,
                  station_m=0.0, desired_speed_kmh=50.0),
    ]
    for _ in range(100):
        model.update(dt=0.1, ego_station_m=0.0)
    assert model.npcs[0].speed_kmh == pytest.approx(50.0, abs=1.0)


def test_oncoming_npcs_are_unaffected_by_npc_to_npc_idm():
    """Scoped explicitly to same-direction traffic -- oncoming NPCs keep
    their old constant-speed behaviour regardless of what's "ahead" of them
    in their own direction of travel."""
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [
        NpcVehicle(id="oncoming-lead", lane_offset=-1.75, speed_kmh=-40.0,
                  station_m=10.0, desired_speed_kmh=-40.0),
        NpcVehicle(id="oncoming-follower", lane_offset=-1.75, speed_kmh=-40.0,
                  station_m=50.0, desired_speed_kmh=-40.0),
    ]
    model.update(dt=1.0, ego_station_m=0.0)
    follower = next(n for n in model.npcs if n.id == "oncoming-follower")
    assert follower.speed_kmh == pytest.approx(-40.0)


# ---------------------------------------------------------------------------
# Lane-change safety sensor (used by app/services/planner.py's real
# lane-change candidate): a second sensor query, distinct from
# sense_lead_vehicle, that only ever answers "is this lane clear?" -- never
# exposes which NPC (if any) is blocking it.
# ---------------------------------------------------------------------------

def test_sense_lane_clear_true_when_target_lane_is_empty():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = []
    assert model.sense_lane_clear(ego_station_m=100.0, target_lane_offset=ADJACENT_LANE_OFFSET_M) is True


def test_sense_lane_clear_false_when_npc_is_ahead_within_clearance():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="blocker", lane_offset=ADJACENT_LANE_OFFSET_M,
                             speed_kmh=40.0, station_m=110.0)]
    assert model.sense_lane_clear(ego_station_m=100.0, target_lane_offset=ADJACENT_LANE_OFFSET_M) is False


def test_sense_lane_clear_false_when_npc_is_behind_within_clearance():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="blocker", lane_offset=ADJACENT_LANE_OFFSET_M,
                             speed_kmh=40.0, station_m=90.0)]
    assert model.sense_lane_clear(ego_station_m=100.0, target_lane_offset=ADJACENT_LANE_OFFSET_M) is False


def test_sense_lane_clear_ignores_npc_far_outside_the_clearance_envelope():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="far", lane_offset=ADJACENT_LANE_OFFSET_M,
                             speed_kmh=40.0, station_m=500.0)]
    assert model.sense_lane_clear(ego_station_m=100.0, target_lane_offset=ADJACENT_LANE_OFFSET_M) is True


def test_sense_lane_clear_ignores_npc_in_a_different_lane():
    model = TrafficModel(total_length_m=1000.0, seed=1)
    model.npcs = [NpcVehicle(id="other-lane", lane_offset=EGO_LANE_OFFSET_M,
                             speed_kmh=40.0, station_m=105.0)]
    assert model.sense_lane_clear(ego_station_m=100.0, target_lane_offset=ADJACENT_LANE_OFFSET_M) is True
