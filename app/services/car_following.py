"""
Intelligent Driver Model (IDM) car-following -- Treiber, Hennecke & Helbing
(2000).

Why this module exists: `traffic.py`'s forward range sensor (P6-1b) has
computed a real gap/relative-speed to the nearest same-lane lead vehicle
since it was built, and `planner.py` (P6-2) already reads it -- but only to
nudge the LATERAL candidate cost function. Nothing in `physics_engine.py`'s
longitudinal control ever consumed it: the ego's speed controller only ever
chased a cruise/AI-decision target, so the car never actually slowed down
for traffic ahead, regardless of gap. That is the literal absence of the
"intelligence layer" -- a sensed gap that gets computed and then discarded
is not a driving decision.

This module is deliberately just the IDM formula as a pure function, tested
in isolation from PhysicsEngine's much larger integration surface. Used by
BOTH the ego's own longitudinal control (physics_engine.py, fed from
traffic.py's perception-boundary-respecting sensor) and NPC-to-NPC
following (traffic.py, fed from ground-truth NPC state -- NPCs are allowed
oracle access to each other since there is no perception/control boundary
between two simulated vehicles, only between the sensor and the planner).
"""
import math
from typing import Optional

DESIRED_TIME_GAP_S = 1.5      # T: desired time headway to the lead vehicle
MIN_GAP_M = 2.0                # s0: minimum bumper-to-bumper gap at a standstill
COMFORTABLE_DECEL_MPS2 = 2.0   # b: comfortable braking deceleration used in the desired-gap formula
ACCEL_EXPONENT = 4             # delta: shape of the free-road acceleration term


def idm_acceleration(
    v_mps: float,
    v0_mps: float,
    gap_m: Optional[float],
    lead_speed_mps: Optional[float],
    a_max_mps2: float,
    comfortable_decel_mps2: float = COMFORTABLE_DECEL_MPS2,
) -> Optional[float]:
    """The IDM-desired longitudinal acceleration given a sensed lead vehicle,
    or None if there is no lead vehicle in range -- an explicit, testable
    "free-road driving, IDM does not apply" case, rather than passing a huge
    gap value through the interaction term and trusting it decays to
    ~zero. Callers combine this with their own free-road/cruise controller
    via min(), per the standard IDM/ACC composition: the more conservative
    (smaller) of "what I'd do anyway" and "what I must do to not run into
    the car ahead" wins.

    v0_mps <= 0 degenerates the free-road term to 0 (never accelerate to
    reach a zero/negative desired speed) rather than dividing by zero.
    """
    if gap_m is None:
        return None

    lead_v = lead_speed_mps if lead_speed_mps is not None else v_mps
    delta_v = v_mps - lead_v

    desired_gap = MIN_GAP_M + max(
        0.0,
        v_mps * DESIRED_TIME_GAP_S
        + (v_mps * delta_v) / (2.0 * math.sqrt(a_max_mps2 * comfortable_decel_mps2)),
    )

    free_road_term = 1.0 - (v_mps / v0_mps) ** ACCEL_EXPONENT if v0_mps > 0 else 0.0
    interaction_term = (desired_gap / max(gap_m, 0.1)) ** 2

    return a_max_mps2 * (free_road_term - interaction_term)
