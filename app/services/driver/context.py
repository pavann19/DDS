"""``DriverContext`` -- the per-tick input the :class:`~app.services.driver.driver.Driver`
consumes alongside a :class:`~app.services.interfaces.SensorObservation`.

The Driver is **simulator-independent**: it never imports ``PhysicsEngine``,
``TrafficModel`` or ``NpcVehicle``. It reads two things each tick:

* a ``SensorObservation`` -- the sensor picture (ego proprioception, forward
  range return, surround tracks, active Frenet frame), and
* a ``DriverContext`` -- map / navigation-derived scalars that a real AV
  planner gets from its map + localisation stack, not from perception:
  upcoming path curvature, distance-to-goal, the route tangent heading, the
  friction estimate, and a bare ``lane_clear`` bool query (same privacy class
  as ``SensorObservation.adjacent_lane_clear``).

Nothing here carries NPC identity or absolute NPC pose.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class DriverContext:
    # --- control condition -------------------------------------------------
    controller: str                       # "bicycle" | "legacy"
    dt_s: float

    # --- ego / localisation (SI + the km/h the legacy path still uses) -----
    speed_kmh: float
    heading_deg: float
    steering_angle_rad: float
    acceleration_mps2: float
    ego_lat: float
    ego_lng: float
    station_m: float
    lateral_offset_m: float
    last_lateral_accel_mps2: float        # previous tick's realised a_lat (shield input)

    # --- navigation / map geometry --------------------------------------
    dist_to_dest_m: float
    waypoint_dist_m: float
    target_heading_deg: float             # bearing to the look-ahead route point
    max_curvature_1pm: float              # sharpest curvature in the look-ahead window
    corner_turn_deg: float                # sharpest single bearing change (legacy cap + FSM)
    friction_mu: float
    has_arrived: bool                     # latched arrival flag (post dist<10 latch)

    # --- world query (bare bool, no NPC state) --------------------------
    # sense_lane_clear(station_m, lane_offset_d_m) -> bool. Optional: None when
    # there is no traffic model on the route.
    lane_clear_fn: Optional[Callable[[float, float], bool]] = None

    # Prediction-stage result (forecasts / intents / risk field), derived from
    # the ego's own surround tracks -- sensor-derived, not privileged world
    # state. Opaque here; the planner reads ``.risk_field`` and
    # ``.proactive_decel_mps2``.
    prediction_result: Optional[object] = None
