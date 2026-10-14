"""``CarlaBackend`` -- CARLA behind the same ``WorldBackend`` contract.

SKELETON. CARLA cannot run on the current machine (docs/CARLA_FEASIBILITY.md:
AMD 4 GB GPU / 11 GB RAM / Python 3.13 -- all below spec). Every method here
raises with a pointer to the plan. The class exists so:

* the boundary is symmetric -- swapping ``DeterministicSimBackend`` for
  ``CarlaBackend`` is the only change the rest of DDS ever sees;
* the responsibilities are pinned in code, not just prose;
* the pure ``conversions`` it will use are already implemented and tested.

Target: CARLA 0.9.15 (UE4), Python 3.10 client venv, synchronous mode,
``fixed_delta_seconds`` matching the DDS control tick. See
docs/CARLA_MIGRATION_PLAN.md for the phased implementation
(CarlaWorld -> CarlaObservationSource -> CarlaControlSink -> CarlaRoute ->
CarlaScenarioRunner) and the actor-state vs real-sensor decision.
"""
from __future__ import annotations

from typing import Any, Optional, Sequence, Tuple

from app.services.carla_adapter.contracts import (
    ControlCommand,
    SensorObservation,
    WorldBackend,
    WorldSnapshot,
)

_NOT_YET = (
    "CarlaBackend is a skeleton -- CARLA is not runnable on this machine "
    "(docs/CARLA_FEASIBILITY.md). Implement per docs/CARLA_MIGRATION_PLAN.md "
    "on an NVIDIA/16GB+ host with a Python 3.10 CARLA client venv."
)


class CarlaBackend(WorldBackend):
    name = "carla-0.9.15"

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 2000,
        town: str = "Town04",              # a highway-heavy map, matches DDS's highway prior
        fixed_delta_seconds: float = 0.05,  # 20 Hz sim; DDS control decimates to 10 Hz
        sensor_mode: str = "actor_state",   # "actor_state" (stage 1) | "real_sensors" (stage 2)
        seed: Optional[int] = None,
    ) -> None:
        self.host, self.port, self.town = host, port, town
        self.fixed_delta_seconds = fixed_delta_seconds
        self.sensor_mode = sensor_mode
        self.seed = seed
        self._client = None   # carla.Client
        self._world = None    # carla.World
        self._ego = None      # carla.Vehicle
        raise NotImplementedError(_NOT_YET)

    # -- lifecycle --------------------------------------------------------
    def reset(self, *, route_waypoints: Sequence[Tuple[float, float]],
              start_speed_mps: float = 0.0, seed: Optional[int] = None,
              **kwargs: Any) -> None:
        # 1. connect carla.Client(host, port); load `town`
        # 2. world.apply_settings(synchronous_mode=True,
        #        fixed_delta_seconds=self.fixed_delta_seconds)
        # 3. spawn ego at route_waypoints[0]; set initial velocity
        # 4. TrafficManager(port+6000); synchronous; deterministic seed
        raise NotImplementedError(_NOT_YET)

    def apply_control(self, cmd: ControlCommand) -> None:
        # conversions.dds_command_to_carla_control(cmd.steer_rad, cmd.accel_mps2)
        #   -> carla.VehicleControl; self._ego.apply_control(...)
        raise NotImplementedError(_NOT_YET)

    def step(self, dt_s: float) -> None:
        # self._world.tick()  (dt_s must equal fixed_delta_seconds, or
        # accumulate sub-ticks); pump sensor queues.
        raise NotImplementedError(_NOT_YET)

    def observe(self) -> SensorObservation:
        # sensor_mode == "actor_state":
        #   * ego: self._ego.get_transform()/get_velocity()/get_control()
        #   * surround_tracks: nearby carla actors -> TrackEstimate (ego-local
        #     x=East,z=South via conversions.carla_xy_to_dds_local_xz), fed
        #     through the SAME PerceptionNoiseConfig + MultiTargetTracker DDS
        #     already uses -- so this is not privileged.
        #   * lead: forward corridor query over those actors -> LeadObservation
        #   * frenet_frame: build_frenet_frame(CarlaRoute waypoints)
        # sensor_mode == "real_sensors":
        #   * attach carla RGB/Depth/LiDAR/Radar/IMU; run a detector; same
        #     tracker; identical SensorObservation shape (no DDS change).
        raise NotImplementedError(_NOT_YET)

    def snapshot(self) -> WorldSnapshot:
        raise NotImplementedError(_NOT_YET)

    # -- optional -------------------------------------------------------
    def set_weather(self, **kwargs: Any) -> None:
        raise NotImplementedError(_NOT_YET)

    def set_friction(self, mu: float) -> None:
        # carla: per-wheel PhysicsControl.tire_friction, or a friction
        # trigger volume over the curve section.
        raise NotImplementedError(_NOT_YET)

    def close(self) -> None:
        if self._world is not None:  # pragma: no cover
            settings = self._world.get_settings()
            settings.synchronous_mode = False
            self._world.apply_settings(settings)
