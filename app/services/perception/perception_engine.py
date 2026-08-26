"""
Surround perception orchestrator (Phase 6, P6-5).

Ties sensor_rig.py (frustum detection), entities.py (classification),
tracker.py (EKF + GNN association), and occupancy_grid.py (log-odds grid)
into one per-tick pipeline PhysicsEngine can call. This is the boundary
between "raw simulated NPC truth" (traffic.py's TrafficModel, which NPCs
have oracle access to each other through) and "what the ego's own sensors
actually perceive" -- an NPC not inside any frustum this tick simply never
becomes a DetectedEntity, the same perception/control boundary
traffic.py's sense_lead_vehicle() already established for the forward
sensor.
"""
from dataclasses import dataclass
from typing import List

from app.services.frenet import FrenetFrame, frenet_to_local_xz, frenet_to_local_xz_batch
from app.services.perception.entities import DetectedEntity, EntityClass, dimensions_for
from app.services.perception.occupancy_grid import OccupancyGrid, OccupiedFootprint
from app.services.perception.sensor_rig import (
    SENSOR_RIG,
    RelativeObservation,
    batch_detecting_mask,
    batch_relative_observations,
    detecting_frustums,
)
from app.services.perception.tracker import MultiTargetTracker, Track
from app.services.traffic import NpcVehicle


@dataclass
class SurroundTrack:
    """One tracked, currently-or-recently-detected actor, serialisable for
    the WebSocket protocol / HMI."""
    track_id: int
    entity_class: str
    status: str
    x: float
    z: float
    vx: float
    vz: float
    range_m: float
    azimuth_deg: float
    detecting_sensors: List[str]
    length_m: float
    width_m: float
    height_m: float


class SurroundPerceptionEngine:
    def __init__(self):
        self.tracker = MultiTargetTracker()
        self.grid = OccupancyGrid()
        self.last_tracks: List[SurroundTrack] = []

    def step(
        self,
        frame: FrenetFrame,
        ego_s: float,
        ego_d: float,
        npcs: List[NpcVehicle],
        dt: float,
    ) -> List[SurroundTrack]:
        # Ego's own projection is fixed for the whole tick -- computed once
        # and reused for every actor. Every actor's own projection (world
        # position + tangent) is computed in ONE batched numpy call instead
        # of a Python loop calling frenet_to_local_xz once per actor
        # (frenet_to_local_xz_batch), and range/azimuth/frustum-detection
        # are likewise batched (batch_relative_observations,
        # batch_detecting_mask) rather than looped per actor per frustum.
        # This per-actor Python loop was a measurable share of Gate 6.3's
        # 2ms/tick @ 30-actor budget.
        ego_x, ego_z, ego_dir_x, ego_dir_z = frenet_to_local_xz(frame, ego_s, ego_d)

        detections: List[DetectedEntity] = []
        observation_by_npc = {}
        if npcs:
            stations = [npc.station_m for npc in npcs]
            offsets = [npc.lane_offset for npc in npcs]
            actor_xs, actor_zs, dir_xs, dir_zs = frenet_to_local_xz_batch(frame, stations, offsets)
            range_m, azimuth_deg = batch_relative_observations(ego_x, ego_z, ego_dir_x, ego_dir_z, actor_xs, actor_zs)
            detected_mask = batch_detecting_mask(range_m, azimuth_deg, SENSOR_RIG)

            for i, npc in enumerate(npcs):
                if not detected_mask[i]:
                    continue
                obs = RelativeObservation(range_m=float(range_m[i]), azimuth_deg=float(azimuth_deg[i]),
                                           x=float(actor_xs[i]), z=float(actor_zs[i]))
                # Per-actor sensor-name list only computed for actors that
                # ARE detected (the common case is most actors aren't, at
                # any given tick, inside every frustum) -- this is the one
                # remaining per-frustum loop, now bounded by the detected
                # subset rather than every actor.
                sensors = detecting_frustums(obs, SENSOR_RIG)
                speed_mps = npc.speed_kmh / 3.6
                try:
                    entity_class = EntityClass(npc.entity_class)
                except ValueError:
                    entity_class = EntityClass.SEDAN
                det = DetectedEntity(
                    entity_class=entity_class,
                    x=obs.x,
                    z=obs.z,
                    vx=speed_mps * float(dir_xs[i]),
                    vz=speed_mps * float(dir_zs[i]),
                    source_id=npc.id,
                )
                detections.append(det)
                observation_by_npc[npc.id] = (obs, sensors)

        tracks = self.tracker.step(dt, detections)

        self.grid.reset(ego_x, ego_z)
        footprints = []
        for t in tracks:
            length_m, width_m, _ = dimensions_for(t.entity_class)
            footprints.append(OccupiedFootprint(x=t.x, z=t.z, length_m=length_m, width_m=width_m, heading_rad=t.heading_rad))
        self.grid.update(footprints)

        surround_tracks: List[SurroundTrack] = []
        for t in tracks:
            obs_entry = observation_by_npc.get(t.source_id)
            if obs_entry:
                obs, sensors = obs_entry
                range_m, azimuth_deg = obs.range_m, obs.azimuth_deg
            else:
                # Coasted track with no detection this tick -- report a
                # zeroed range/azimuth rather than dropping it, matching a
                # real tracker's behaviour through a brief dropout (the
                # track itself, t.x/t.z, still reflects the filter's own
                # predicted position).
                range_m, azimuth_deg, sensors = 0.0, 0.0, []
            length_m, width_m, height_m = dimensions_for(t.entity_class)
            surround_tracks.append(SurroundTrack(
                track_id=t.track_id,
                entity_class=t.entity_class.value,
                status=t.status.value,
                x=t.x,
                z=t.z,
                vx=t.vx,
                vz=t.vz,
                range_m=range_m,
                azimuth_deg=azimuth_deg,
                detecting_sensors=sensors,
                length_m=length_m,
                width_m=width_m,
                height_m=height_m,
            ))

        self.last_tracks = surround_tracks
        return surround_tracks

    def get_state(self) -> List[dict]:
        return [
            {
                "id": f"track-{t.track_id}",
                "class": t.entity_class,
                "status": t.status,
                "x": round(t.x, 2),
                "z": round(t.z, 2),
                "vx": round(t.vx, 2),
                "vz": round(t.vz, 2),
                "range_m": round(t.range_m, 1),
                "azimuth_deg": round(t.azimuth_deg, 1),
                "sensors": t.detecting_sensors,
                "dims": [t.length_m, t.width_m, t.height_m],
            }
            for t in self.last_tracks
            if t.status == "CONFIRMED"
        ]
