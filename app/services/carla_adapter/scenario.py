"""``ScenarioRunner`` -- drives a named scenario against a ``WorldBackend``.

The 8 built-in DDS scenarios (following / cut-in / merge / straddle /
wet-curve / queue / ...) are choreographed today by ``scenario_engine`` in
Frenet-station terms against ``PhysicsEngine``/``TrafficModel``. Their
*intent* is what must be preserved, not the implementation.

* ``DeterministicScenarioRunner`` -- thin pass-through to the existing
  ``ScenarioEngine`` when the backend is ``DeterministicSimBackend``. Keeps
  the current scenario suite working unchanged.
* ``CarlaScenarioRunner`` -- (skeleton) re-expresses each scenario's intent
  with CARLA actors + Traffic Manager + weather (per-tick ``set_transform``
  for scripted actors; TM for ambient). Built when a CARLA host exists.

Intent map (for the CARLA re-implementation):

| scenario           | intent to preserve                                            |
|--------------------|--------------------------------------------------------------- |
| normal_cruising    | free-flow lane keep + cruise tracking                          |
| traffic_overtake   | slow lead in ego lane, clear adjacent -> IDM brake then pass   |
| emergency_cut_in   | adjacent vehicle crosses in close -> Safety Shield emerg. brake|
| queue_stop_and_go  | stop-and-go column ahead -> smooth queueing, no overlap        |
| erratic_cut_in     | high-lat-rate aggressive cut-in + heavy decel                  |
| highway_merge      | on-ramp vehicle seeks gap -> ego yields, no shield trip        |
| lane_straddler     | lead straddles the lane line -> defensive spacing              |
| low_friction_curve | wet mu on a real curve -> lateral de-rate binds                |
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Optional

from app.services.carla_adapter.contracts import WorldBackend
from app.services.carla_adapter.deterministic_backend import DeterministicSimBackend


class ScenarioRunner(ABC):
    @abstractmethod
    def load(self, name: str, *, seed: Optional[int] = None, **params: Any) -> dict:
        ...

    @abstractmethod
    def tick(self, dt_s: float) -> Optional[dict]:
        """Advance the scenario one step; return a scenario event dict or None."""


class DeterministicScenarioRunner(ScenarioRunner):
    """Pass-through to the existing ScenarioEngine."""

    def __init__(self, backend: DeterministicSimBackend):
        from app.services.scenario_engine import ScenarioEngine
        if not isinstance(backend, DeterministicSimBackend):
            raise TypeError("DeterministicScenarioRunner requires DeterministicSimBackend")
        self._backend = backend
        self._engine = ScenarioEngine()

    def load(self, name: str, *, seed: Optional[int] = None, **params: Any) -> dict:
        return self._engine.load_scenario(
            name, self._backend.physics,
            initial_speed_kmh=params.pop("initial_speed_kmh", 50.0),
            **({"params": params} if params else {}),
        )

    def tick(self, dt_s: float) -> Optional[dict]:
        return self._engine.update(self._backend.physics, dt_s)


class CarlaScenarioRunner(ScenarioRunner):  # pragma: no cover - skeleton
    """SKELETON -- re-express each scenario's intent with CARLA actors + TM."""

    def __init__(self, backend: WorldBackend):
        self._backend = backend
        raise NotImplementedError(
            "CarlaScenarioRunner needs a CARLA host -- see docs/CARLA_FEASIBILITY.md "
            "and the intent map in this module's docstring"
        )

    def load(self, name: str, *, seed: Optional[int] = None, **params: Any) -> dict:
        raise NotImplementedError

    def tick(self, dt_s: float) -> Optional[dict]:
        raise NotImplementedError
