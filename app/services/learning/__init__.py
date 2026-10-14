"""Learned-component boundary layer.

Nothing in this package may consume privileged simulator state. The ONLY
inputs a learned driving component is permitted to see are assembled by
``observation.build_observation`` from:

* ego proprioception (the vehicle's own integrated state),
* the route Frenet frame (map geometry the vehicle is localised against),
* sensor-derived surround tracks (``SurroundPerceptionEngine`` output --
  frustum-gated, EKF-filtered, optionally noised/dropped/occluded/delayed),
* the forward collision-corridor reading (``sense_lead_vehicle`` -- a gap and
  a speed, optionally noised),
* the adjacent-lane-clear booleans,
* the prediction stage's intent distributions (themselves computed only from
  the sensor tracks).

FORBIDDEN as inputs: ``TrafficModel``, ``NpcVehicle`` / ``traffic.npcs``,
exact NPC positions/velocities, exact future NPC trajectories, oracle intent,
or any other quantity the ego's own sensors could not produce. The
``test_observation_boundary`` suite enforces this statically and at runtime.
"""
from app.services.learning.observation import (  # noqa: F401
    Observation,
    AgentSlot,
    MAX_AGENTS,
    build_observation,
)

__all__ = ["Observation", "AgentSlot", "MAX_AGENTS", "build_observation"]
