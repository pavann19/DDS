"""DDS <-> simulator boundary layer.

The migration boundary (docs/CARLA_MIGRATION_PLAN.md):

    world backend  --observe()-->  SensorObservation
                                        |
                                   DDS Driver / intelligence
                                        |
    world backend  <--apply()--  ActuatorCommand (steer_rad, accel_mps2)

Anything left of the boundary (world, roads/map, vehicle physics, NPC
traffic, weather, sensor generation) is a ``WorldBackend``. Anything right of
it (perception interface, tracking, prediction, planner, intent/risk, Safety
Shield, controller, learning-observation boundary, future learned policy) is
unchanged DDS and never imports this package.

Two backends implement the same contract:

* ``DeterministicSimBackend`` -- wraps the existing ``PhysicsEngine``. Kept
  for CI / unit / regression tests (ADR-003 invariant V; the seeded suites).
* ``CarlaBackend`` -- a skeleton until a CARLA host is available
  (docs/CARLA_FEASIBILITY.md: not runnable on the current machine).

``conversions`` holds the pure coordinate / heading / control-unit maps
between DDS and CARLA, tested without a running CARLA.
"""
from app.services.carla_adapter.contracts import (  # noqa: F401
    WorldBackend,
    WorldSnapshot,
    ControlCommand,
)
from app.services.carla_adapter.deterministic_backend import (  # noqa: F401
    DeterministicSimBackend,
)

__all__ = [
    "WorldBackend",
    "WorldSnapshot",
    "ControlCommand",
    "DeterministicSimBackend",
]
