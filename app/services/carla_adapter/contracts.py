"""The world-backend contract every simulator must satisfy.

A ``WorldBackend`` owns the world, the map, vehicle physics, NPC traffic,
weather and sensor generation. The DDS Driver reads it only through
``observe() -> SensorObservation`` and drives it only through
``apply_control(ControlCommand)``. It exposes ``snapshot()`` for the
telemetry / HMI layer (protocol v3) -- that is NOT a Driver input.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from app.services.interfaces import (  # re-export -- callers import from here
    ActuatorCommand,
    SensorObservation,
    SimClock,
)

__all__ = ["WorldBackend", "WorldSnapshot", "ControlCommand", "ActuatorCommand",
           "SensorObservation"]

# ``ControlCommand`` is the Driver -> World message. ``ActuatorCommand``
# already IS this shape (steer_rad, accel_mps2, source, speed_limit_reason);
# alias it so adapter code and docs use one name.
ControlCommand = ActuatorCommand


@dataclass(frozen=True)
class WorldSnapshot:
    """Everything the telemetry / HMI layer needs from a tick -- a superset
    of the Driver's ``SensorObservation`` that MAY include ground truth (NPC
    poses for rendering, etc.). It is never passed to the Driver or to the
    learning-observation builder.
    """
    clock: SimClock
    # ego, in whatever absolute frame the backend uses natively
    ego_pose: Dict[str, float]              # {x, y, z, heading_deg, ...} or {lat, lng, ...}
    ego_speed_mps: float
    ego_accel_mps2: float
    ego_steer_rad: float
    # NPC / actor render state (ground truth -- HMI only)
    actors: Tuple[Dict[str, Any], ...] = ()
    # passthrough of the backend-native nav/diagnostic dict (protocol v3)
    nav_state: Dict[str, Any] = field(default_factory=dict)
    extra: Dict[str, Any] = field(default_factory=dict)


class WorldBackend(ABC):
    """Simulator-agnostic world. Two implementations: the deterministic DDS
    sim (CI/regression) and CARLA (realistic experiments)."""

    #: human-readable id, e.g. "deterministic-sim", "carla-0.9.15"
    name: str = "world"

    @abstractmethod
    def reset(
        self,
        *,
        route_waypoints: Sequence[Tuple[float, float]],
        start_speed_mps: float = 0.0,
        seed: Optional[int] = None,
        **kwargs: Any,
    ) -> None:
        """Load a route and place the ego at its start. ``route_waypoints``
        are ``(x, y)`` pairs in the backend's own route frame (for the
        deterministic sim: lat/lng; for CARLA: a metric world frame). The
        adapter's ``RouteProvider`` produces them."""

    @abstractmethod
    def apply_control(self, cmd: ControlCommand) -> None:
        """Queue the Driver's command for the next ``step()``. The World owns
        the jerk limit and the integration; ``accel_mps2`` is a *demand*."""

    @abstractmethod
    def step(self, dt_s: float) -> None:
        """Advance the world by ``dt_s`` using the last ``apply_control``."""

    @abstractmethod
    def observe(self) -> SensorObservation:
        """The ONLY thing the Driver may read. Sensor-derived + ego
        proprioception + route geometry. No NPC list / identity / absolute
        pose beyond what the modelled sensors resolved."""

    @abstractmethod
    def snapshot(self) -> WorldSnapshot:
        """Telemetry / HMI view. May contain ground truth. NOT a Driver
        input and NOT passed to the learning-observation builder."""

    # -- optional capabilities ------------------------------------------------
    def spawn_npc(self, **kwargs: Any) -> Any:  # pragma: no cover - optional
        raise NotImplementedError(f"{self.name} does not support scripted NPC spawn")

    def set_weather(self, **kwargs: Any) -> None:  # pragma: no cover - optional
        raise NotImplementedError(f"{self.name} does not model weather")

    def set_friction(self, mu: float) -> None:  # pragma: no cover - optional
        raise NotImplementedError(f"{self.name} does not model road friction")

    def close(self) -> None:  # pragma: no cover - optional
        pass
