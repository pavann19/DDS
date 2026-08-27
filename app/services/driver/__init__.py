"""Driver-side of the DDS autonomy pipeline (ADR-001, Phase 6.5).

The *driver* is the autonomy stack: perception, prediction, behaviour,
planning, control and an independent safety monitor. It may only observe the
world through a ``SensorObservation`` (``app/services/interfaces.py``) -- it
never holds a ``TrafficModel`` or an ``NpcVehicle``.

Phase 6.5 does a **hybrid** extraction: the driver's decision math moves here
as pure functions, but ``PhysicsEngine`` stays the public object and keeps
calling them (thin facade). The functions currently take the scalar subset of
``SensorObservation`` they need; Action Item 4 (the multi-rate executor) is
what switches them to the full typed object.
"""
from app.services.driver.lateral_planner import LateralPlan, plan_lateral_offset

__all__ = ["LateralPlan", "plan_lateral_offset"]
