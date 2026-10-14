"""The World -> Driver -> World boundary, exercised through
``DeterministicSimBackend`` (the existing sim behind the ``WorldBackend``
contract). No CARLA.

Verifies: route following, ``observe()`` returns a sensor-only
``SensorObservation`` (no NPC identity / absolute pose), the contract is
honoured, and the deterministic sim still works unchanged through the new
object.
"""
import math

import pytest

from app.services.interfaces import SensorObservation, EgoProprioception
from app.services.carla_adapter import (
    DeterministicSimBackend,
    WorldBackend,
    ControlCommand,
)
from app.services.carla_adapter.route import DeterministicRouteProvider
from app.services.carla_adapter.carla_backend import CarlaBackend


def _straight(n=200):
    return [(37.7749 + i * 0.00005, -122.4194) for i in range(n)]


def test_backend_reset_step_observe_snapshot_cycle():
    b = DeterministicSimBackend(seed=1)
    assert isinstance(b, WorldBackend)
    b.reset(route_waypoints=_straight(), start_speed_mps=12.5)

    for _ in range(50):
        obs = b.observe()
        assert isinstance(obs, SensorObservation)
        assert isinstance(obs.ego, EgoProprioception)
        b.apply_control(ControlCommand(steer_rad=0.0, accel_mps2=0.0))
        b.step(0.1)

    obs = b.observe()
    # the ego actually progressed along the route and held its lane
    assert obs.ego.station_m > 40.0
    assert abs(obs.ego.lateral_offset_m - 1.75) < 0.5
    assert obs.frenet_frame is not None
    snap = b.snapshot()
    assert snap.ego_speed_mps > 0.0
    assert "station_m" in snap.nav_state


def test_observe_is_sensor_only_no_npc_identity_or_pose():
    from app.services.traffic import TrafficModel, NpcVehicle
    b = DeterministicSimBackend(seed=1)
    b.reset(route_waypoints=_straight(), start_speed_mps=13.0)
    b.physics.traffic = TrafficModel(total_length_m=b.physics.station_distances[-1], seed=1)
    b.physics.traffic.spawn_scripted_npcs([NpcVehicle(
        id="lead", lane_offset=1.75, speed_kmh=20.0, station_m=45.0,
        desired_speed_kmh=20.0, prevent_recycle=True)])
    for _ in range(30):
        b.step(0.1)
    obs = b.observe()

    # SensorObservation has no npc list / dict of NpcVehicle
    assert not hasattr(obs, "npcs")
    # lead is a bare gap + speed, no id / absolute position
    if obs.lead is not None:
        assert set(vars(obs.lead)) <= {"gap_m", "lead_speed_mps"}
    # surround tracks are the tracker's SurroundTrack shape (ego-local x/z,
    # track_id) -- never the source NpcVehicle object
    for t in obs.surround_tracks:
        assert type(t).__name__ != "NpcVehicle"
        assert hasattr(t, "track_id") and hasattr(t, "x") and hasattr(t, "z")

    # mutate ground truth without re-running perception -> observe() unchanged
    before = b.observe()
    for npc in b.physics.traffic.npcs:
        npc.station_m += 500.0
        npc.lane_offset = -5.25
    after = b.observe()
    assert before == after


def test_deterministic_backend_is_seed_reproducible():
    def run():
        b = DeterministicSimBackend(seed=7)
        b.reset(route_waypoints=_straight(), start_speed_mps=13.0)
        trace = []
        for _ in range(80):
            b.step(0.1)
            o = b.observe()
            trace.append((round(o.ego.speed_mps, 9), round(o.ego.station_m, 9),
                          round(o.ego.steer_rad, 9)))
        return trace
    assert run() == run()


def test_external_driver_mode_drives_vehicle_dynamics():
    b1 = DeterministicSimBackend(mode="external", seed=42)
    b1.reset(route_waypoints=_straight(), start_speed_mps=10.0)

    # Command positive acceleration and right steering
    cmd_accel = ControlCommand(steer_rad=0.15, accel_mps2=2.0)
    b1.apply_control(cmd_accel)
    b1.step(0.1)
    snap1 = b1.snapshot()

    # Second instance with different command (braking, left steering)
    b2 = DeterministicSimBackend(mode="external", seed=42)
    b2.reset(route_waypoints=_straight(), start_speed_mps=10.0)
    cmd_brake = ControlCommand(steer_rad=-0.15, accel_mps2=-3.0)
    b2.apply_control(cmd_brake)
    b2.step(0.1)
    snap2 = b2.snapshot()

    # Changing actuator command changes the next world state directly
    assert snap1.ego_speed_mps > snap2.ego_speed_mps
    assert snap1.ego_accel_mps2 > snap2.ego_accel_mps2
    assert snap1.ego_steer_rad != snap2.ego_steer_rad
    assert snap1.ego_pose["heading_deg"] != snap2.ego_pose["heading_deg"]
    assert snap1.ego_pose["lat"] != snap2.ego_pose["lat"]


def test_carla_backend_is_a_skeleton_that_refuses_cleanly():
    assert issubclass(CarlaBackend, WorldBackend)
    with pytest.raises(NotImplementedError):
        CarlaBackend()


def test_route_provider_builds_a_frenet_frame():
    rp = DeterministicRouteProvider(waypoints=_straight(60))
    assert len(rp.waypoints()) >= 2
    assert rp.frenet_frame() is not None
