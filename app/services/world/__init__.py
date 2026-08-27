"""World-side of the DDS autonomy pipeline (ADR-001, Phase 6.5).

The *world* is ground truth: it integrates the physical vehicle forward,
owns NPC traffic, and (later) the environment/map. The *driver* (see
``app/services/driver/``) may only observe it through a ``SensorObservation``
(``app/services/interfaces.py``).

Phase 6.5 does a **hybrid** extraction: the world's math is moved here as
pure, independently-testable functions, but ``PhysicsEngine`` stays the
public object and keeps delegating to them (thin facade). ``TrafficModel``
ownership is re-exported here so callers can migrate imports incrementally;
the full decoupling of ``scenario_engine`` from ``physics.traffic`` is
deferred to when Phase 11 (swappable/failable driver) forces it.
"""
from app.services.traffic import NpcVehicle, SensedLeadVehicle, TrafficModel
from app.services.world.vehicle_dynamics import (
    PowertrainState,
    advance_position,
    step_powertrain,
)

__all__ = [
    "NpcVehicle",
    "SensedLeadVehicle",
    "TrafficModel",
    "PowertrainState",
    "advance_position",
    "step_powertrain",
]
