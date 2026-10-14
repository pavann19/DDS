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
import math
import random
from collections import deque
from dataclasses import dataclass
from typing import List, Optional

from app.services.frenet import FrenetFrame, frenet_to_local_xz, frenet_to_local_xz_batch
from app.services.perception.entities import DetectedEntity, EntityClass, dimensions_for
from app.services.perception.occupancy_grid import OccupancyGrid, OccupiedFootprint
from app.services.perception.sensor_rig import (
    SENSOR_RIG,
    RelativeObservation,
    batch_frustum_membership,
    batch_relative_observations,
    detecting_frustums,
)
from app.services.perception.tracker import MultiTargetTracker, Track
from app.services.traffic import NpcVehicle


@dataclass
class PerceptionNoiseConfig:
    """Configurable *measurement*-boundary imperfections applied to the raw
    frustum detections BEFORE they reach the EKF tracker -- so any learned
    representation is built from something a real sensor could produce, not
    from privileged ground truth. Every field defaults to a no-op; with the
    defaults, SurroundPerceptionEngine.step() is byte-for-byte identical to
    the pre-noise pipeline. A dedicated RNG (seeded from the engine seed)
    keeps runs reproducible.
    """
    # Gaussian position error, metres. Total sigma at range R is
    #   position_sigma_m + position_sigma_per_100m * (R / 100).
    position_sigma_m: float = 0.0
    position_sigma_per_100m: float = 0.0
    # Gaussian error on the detection's reported velocity components, m/s.
    velocity_sigma_mps: float = 0.0
    # Per-detection Bernoulli miss probability (independent each tick).
    dropout_prob: float = 0.0
    # If enabled, a detection whose bearing is within occlusion_half_angle_deg
    # of a NEARER detection this tick (and is farther away) is dropped.
    occlusion_enabled: bool = False
    occlusion_half_angle_deg: float = 4.0
    # Whole-pipeline detection latency, in ticks (a K-tick ring buffer: the
    # tracker this tick sees the detection set from K ticks ago).
    latency_ticks: int = 0
    # Poisson-rate of spurious detections per tick (ghost targets). Placed at
    # a random in-frustum range/azimuth. Default 0.
    false_positive_rate: float = 0.0

    def is_noop(self) -> bool:
        return (
            self.position_sigma_m == 0.0
            and self.position_sigma_per_100m == 0.0
            and self.velocity_sigma_mps == 0.0
            and self.dropout_prob == 0.0
            and not self.occlusion_enabled
            and self.latency_ticks == 0
            and self.false_positive_rate == 0.0
        )


@dataclass(slots=True)
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
    # I2 observation-substrate additions (sensor-derived only): how long this
    # track has been alive, and an EKF position-uncertainty proxy
    # (sqrt of the mean position-covariance diagonal, metres). Both come from
    # the tracker's own state -- never from ground truth.
    age_ticks: int = 0
    pos_sigma_m: float = 0.0


class SurroundPerceptionEngine:
    def __init__(
        self,
        noise: Optional[PerceptionNoiseConfig] = None,
        seed: Optional[int] = None,
    ):
        self.tracker = MultiTargetTracker()
        self.grid = OccupancyGrid()
        self.last_tracks: List[SurroundTrack] = []
        self.last_detections = ()
        self.last_capture_time_s = None
        self._capture_clock_s = 0.
        self.noise = noise or PerceptionNoiseConfig()
        # Dedicated RNG so measurement noise is reproducible for a fixed
        # seed and never perturbs any other stream.
        self._rng = random.Random(seed)
        self._latency_buf: deque = deque()

    def step(
        self,
        frame: FrenetFrame,
        ego_s: float,
        ego_d: float,
        npcs: List[NpcVehicle],
        dt: float,
        *, capture_time_s: Optional[float] = None,
    ) -> List[SurroundTrack]:
        self._capture_clock_s = self._capture_clock_s + dt if capture_time_s is None else capture_time_s
        self.last_capture_time_s = self._capture_clock_s
        # Ego's own projection is fixed for the whole tick -- computed once
        # and reused for every actor. Every actor's own projection (world
        # position + tangent) is computed in ONE batched numpy call instead
        # of a Python loop calling frenet_to_local_xz once per actor
        # (frenet_to_local_xz_batch), and range/azimuth/frustum-detection
        # are likewise batched (batch_relative_observations,
        # batch_detecting_mask) rather than looped per actor per frustum.
        # This per-actor Python loop was a measurable share of Gate 6.3's
        # 2ms/tick @ 30-actor budget.
        if frame is None:
            if npcs:
                raise ValueError('Actor sensing needs an available coordinate frame')
            # With no world actors there are no coordinates to transform.
            # Still run the actual empty acquisition, tracker and latency
            # buffer: absence of road geometry is not sensor-feed failure.
            ego_x, ego_z, ego_dir_x, ego_dir_z = 0., 0., 0., -1.
        else:
            ego_x, ego_z, ego_dir_x, ego_dir_z = frenet_to_local_xz(frame, ego_s, ego_d)

        detections: List[DetectedEntity] = []
        observation_by_npc = {}
        if npcs:
            stations = [npc.station_m for npc in npcs]
            offsets = [npc.lane_offset for npc in npcs]
            actor_xs, actor_zs, dir_xs, dir_zs = frenet_to_local_xz_batch(frame, stations, offsets)
            range_m, azimuth_deg = batch_relative_observations(ego_x, ego_z, ego_dir_x, ego_dir_z, actor_xs, actor_zs)
            membership = batch_frustum_membership(range_m, azimuth_deg, SENSOR_RIG)
            detected_mask = membership.any(axis=1)

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
                sensors = [f.name for f, visible in zip(SENSOR_RIG, membership[i]) if visible]
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
                    heading_rad=math.atan2(float(dir_zs[i]), float(dir_xs[i])),
                    source_id=npc.id,
                )
                detections.append(det)
                observation_by_npc[npc.id] = (obs, sensors)

        # ---- measurement-boundary imperfections (no-op by default) ----
        if not self.noise.is_noop():
            detections, observation_by_npc = self._apply_measurement_noise(
                detections, observation_by_npc, ego_x, ego_z, ego_dir_x, ego_dir_z
            )

        self.last_detections = tuple(detections)
        tracks = self.tracker.step(dt, detections)

        footprints = []
        surround_tracks: List[SurroundTrack] = []
        for t in tracks:
            x, z, vx, vz = map(float, t.state[:4])
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
            footprints.append(OccupiedFootprint(x=x, z=z, length_m=length_m,
                                               width_m=width_m, heading_rad=t.heading_rad))
            try:
                _pos_sigma = float(math.sqrt(max(
                    0.0, 0.5 * (float(t.covariance[0][0]) + float(t.covariance[1][1]))
                )))
            except Exception:
                _pos_sigma = 0.0
            surround_tracks.append(SurroundTrack(
                track_id=t.track_id,
                entity_class=t.entity_class.value,
                status=t.status.value,
                x=x,
                z=z,
                vx=vx,
                vz=vz,
                range_m=range_m,
                azimuth_deg=azimuth_deg,
                detecting_sensors=sensors,
                length_m=length_m,
                width_m=width_m,
                height_m=height_m,
                age_ticks=int(getattr(t, "age_ticks", 0)),
                pos_sigma_m=_pos_sigma,
            ))

        self.grid.reset(ego_x, ego_z)
        self.grid.update(footprints)
        self.last_tracks = surround_tracks
        return surround_tracks

    def _apply_measurement_noise(self, detections, observation_by_npc,
                                ego_x, ego_z, ego_dir_x, ego_dir_z):
        """Occlusion -> dropout -> position/velocity noise -> latency ->
        false positives. Operates on the raw detection list before the EKF.
        Returns (detections, observation_by_npc) both filtered/updated to
        match. Only called when self.noise is not a no-op."""
        cfg = self.noise
        rng = self._rng

        def _range_az(x, z):
            fwd = (x - ego_x) * ego_dir_x + (z - ego_z) * ego_dir_z
            right = (x - ego_x) * (-ego_dir_z) + (z - ego_z) * ego_dir_x
            rng_m = math.hypot(fwd, right)
            az = math.degrees(math.atan2(right, fwd))
            return rng_m, az

        # 1. occlusion -- drop a detection hidden behind a nearer one at ~same bearing
        if cfg.occlusion_enabled and len(detections) > 1:
            enriched = []
            for det in detections:
                r, az = _range_az(det.x, det.z)
                enriched.append((r, az, det))
            enriched.sort(key=lambda t: t[0])
            kept = []
            for r, az, det in enriched:
                hidden = any(
                    r > kr and abs((az - kaz + 180.0) % 360.0 - 180.0) <= cfg.occlusion_half_angle_deg
                    for kr, kaz, _ in kept
                )
                if not hidden:
                    kept.append((r, az, det))
            detections = [det for _, _, det in kept]

        # 2. per-detection dropout
        if cfg.dropout_prob > 0.0:
            detections = [d for d in detections if rng.random() >= cfg.dropout_prob]

        # 3. gaussian position + velocity noise
        noisy = []
        for det in detections:
            r, _az = _range_az(det.x, det.z)
            pos_sigma = cfg.position_sigma_m + cfg.position_sigma_per_100m * (r / 100.0)
            nx = det.x + (rng.gauss(0.0, pos_sigma) if pos_sigma > 0 else 0.0)
            nz = det.z + (rng.gauss(0.0, pos_sigma) if pos_sigma > 0 else 0.0)
            nvx = det.vx + (rng.gauss(0.0, cfg.velocity_sigma_mps) if cfg.velocity_sigma_mps > 0 else 0.0)
            nvz = det.vz + (rng.gauss(0.0, cfg.velocity_sigma_mps) if cfg.velocity_sigma_mps > 0 else 0.0)
            noisy.append(DetectedEntity(
                entity_class=det.entity_class, x=nx, z=nz, vx=nvx, vz=nvz, source_id=det.source_id,
                heading_rad=(math.atan2(nvz, nvx) if cfg.velocity_sigma_mps > 0 and math.hypot(nvx, nvz) > 1e-6
                             else det.heading_rad),
            ))
        detections = noisy

        # 4. whole-pipeline latency: buffer this tick's set, emit K ticks ago
        if cfg.latency_ticks > 0:
            self._latency_buf.append((detections, self._capture_clock_s, observation_by_npc))
            while len(self._latency_buf) > cfg.latency_ticks + 1:
                self._latency_buf.popleft()
            if len(self._latency_buf) > cfg.latency_ticks:
                delivered, capture_time, captured_observations = self._latency_buf[0]
                detections = list(delivered)
                self.last_capture_time_s = capture_time
                observation_by_npc = captured_observations
            else:
                detections = []
                observation_by_npc = {}
                self.last_capture_time_s = None

        # 5. spurious detections (ghost targets)
        if cfg.false_positive_rate > 0.0:
            k = 0
            while rng.random() < cfg.false_positive_rate and k < 3:
                k += 1
                gr = rng.uniform(8.0, 120.0)
                ga = math.radians(rng.uniform(-90.0, 90.0))
                gx = ego_x + ego_dir_x * gr * math.cos(ga) + (-ego_dir_z) * gr * math.sin(ga)
                gz = ego_z + ego_dir_z * gr * math.cos(ga) + (ego_dir_x) * gr * math.sin(ga)
                detections.append(DetectedEntity(
                    entity_class=EntityClass.SEDAN, x=gx, z=gz, vx=0.0, vz=0.0, source_id="",
                ))

        # rebuild observation_by_npc to match surviving/perturbed detections
        new_obs = {}
        for det in detections:
            if not det.source_id:
                continue
            r, az = _range_az(det.x, det.z)
            prev = observation_by_npc.get(det.source_id)
            sensors = prev[1] if prev else []
            new_obs[det.source_id] = (RelativeObservation(range_m=r, azimuth_deg=az, x=det.x, z=det.z), sensors)
        return detections, new_obs

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
