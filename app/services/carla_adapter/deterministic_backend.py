"""``DeterministicSimBackend`` -- the existing DDS simulator behind the
``WorldBackend`` contract.

Phase A (current): ``step()`` runs ``PhysicsEngine.update()``, whose Driver
decision loop is still INLINE. ``apply_control`` is accepted for API
symmetry but the internal loop is authoritative -- so every existing seeded
regression test keeps passing bit-for-bit. This proves the deterministic sim
works *through the new boundary object* without a risky god-object rewrite.

Phase B (docs/CARLA_MIGRATION_PLAN.md): extract the decision loop out of
``PhysicsEngine.update()`` into a ``Driver`` that consumes
``SensorObservation`` and emits ``ActuatorCommand``; then this backend's
``step()`` becomes ``physics.apply_actuator_command(cmd); physics.integrate(dt)``
and ``mode="external"`` becomes the default. The full suite validates the
extraction is behaviour-preserving.
"""
from __future__ import annotations

import math

from typing import Any, Optional, Sequence, Tuple

from app.services.carla_adapter.contracts import (
    ControlCommand,
    SensorObservation,
    WorldBackend,
    WorldSnapshot,
)
from app.services.physics_engine import PhysicsEngine, EGO_LANE_OFFSET_M


class DeterministicSimBackend(WorldBackend):
    name = "deterministic-sim"

    def __init__(self, *, mode: str = "internal", seed: Optional[int] = None):
        if mode not in ("internal", "external"):
            raise ValueError("mode must be 'internal' (Phase A) or 'external' (Phase B)")
        self.mode = mode
        self._seed = seed
        self.physics = PhysicsEngine(seed=seed)
        self._last_cmd: Optional[ControlCommand] = None

    # -- lifecycle ----------------------------------------------------------
    def reset(
        self,
        *,
        route_waypoints: Sequence[Tuple[float, float]],
        start_speed_mps: float = 0.0,
        seed: Optional[int] = None,
        lateral_offset_m: float = EGO_LANE_OFFSET_M,
        **kwargs: Any,
    ) -> None:
        if seed is not None and seed != self._seed:
            self._seed = seed
            self.physics = PhysicsEngine(seed=seed)
        wps = list(route_waypoints)
        if len(wps) < 2:
            raise ValueError("route_waypoints needs >= 2 (lat, lng) points")
        # set_destination BEFORE set_route -- the latter's predecessor clears
        # the route (a real gotcha the codebase carries).
        self.physics.set_destination(wps[-1][0], wps[-1][1])
        self.physics.set_route(wps)
        self.physics.reset_state(
            station_m=0.0,
            speed_kmh=start_speed_mps * 3.6,
            lateral_offset_m=lateral_offset_m,
        )
        self.physics.step_world_sensors(0.0)
        self._last_cmd = None

    # -- driver <-> world -------------------------------------------------
    def apply_control(self, cmd: ControlCommand) -> None:
        if not all(math.isfinite(v) for v in (cmd.steer_rad, cmd.accel_mps2)):
            raise ValueError("Control command must be finite")
        self._last_cmd = cmd

    def step(self, dt_s: float, *, clock=None, update_sensors: bool = True) -> None:
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        if self.mode == "internal":
            self.physics.update("Maintain Speed", dt=dt_s)
        else:
            # External driver mode: WorldBackend.apply_control() drives vehicle dynamics!
            # Zero hidden internal driver decisions.
            cmd = self._last_cmd or ControlCommand(steer_rad=0.0, accel_mps2=0.0, source="default")
            self.physics.clock = clock or self.physics.clock.advance(max(1, round(dt_s / self.physics.clock.dt_s)))
            heading_before = self.physics.heading
            self.physics.step_dynamics(
                steer_rad=cmd.steer_rad,
                accel_mps2=cmd.accel_mps2,
                dt=dt_s,
                heading_before=heading_before,
                update_heading=True,
            )
            self.physics.step_world_sensors(dt=dt_s, perceive=update_sensors, predict=update_sensors)

    def observe(self) -> SensorObservation:
        return self.physics.observe()

    def snapshot(self) -> WorldSnapshot:
        nav = self.physics.get_navigation_state()
        return WorldSnapshot(
            clock=self.physics.clock,
            ego_pose={
                "lat": self.physics.lat,
                "lng": self.physics.lng,
                "heading_deg": self.physics.heading,
                "station_m": self.physics.current_station_m,
                "lateral_offset_m": self.physics.current_lateral_offset_m,
            },
            ego_speed_mps=self.physics.speed_kmh / 3.6,
            ego_accel_mps2=self.physics.acceleration_mps2,
            ego_steer_rad=self.physics.steering_angle_rad,
            actors=tuple(self.physics.get_npc_states()),
            nav_state=nav,
            extra={"last_command": self._last_cmd},
        )

    # -- optional capabilities -------------------------------------------
    def set_friction(self, mu: float) -> None:
        self.physics.friction_mu = max(0.1, min(1.0, mu))

    def spawn_npc(self, **kwargs: Any) -> Any:
        if self.physics.traffic is None:
            raise NotImplementedError("no TrafficModel on this route")
        from app.services.traffic import NpcVehicle
        npc = NpcVehicle(**kwargs)
        self.physics.traffic.spawn_scripted_npcs([npc])
        return npc

    def close(self) -> None:
        pass
