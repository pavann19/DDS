"""
Safety Shield: an INDEPENDENT runtime check of the ego's actual physical
state, run AFTER the Frenet planner (planner.py) and IDM car-following
(car_following.py) have already decided -- not folded into either one's own
cost function, but a genuinely separate, adversarial check that can
override their output.

Why this is a separate module, not more logic inside the planner: a planner
bug can make a bad decision look locally "optimal" to the planner itself
and nothing downstream would ever question it -- exactly what happened with
the forward-sensor bug fixed alongside this (physics_engine.py hardcoded
the sensed lane instead of using the ego's real position, so IDM silently
saw nothing was wrong while the car drove through traffic). A shield that
re-derives risk from raw physical quantities (gap, closing speed, lateral
offset, realized lateral acceleration) rather than trusting the planner's
own bookkeeping is what actually catches that class of failure, not just a
second copy of the same logic that would share the same blind spot.

Three concrete, physically-grounded checks -- not a general policy engine:
  1. Time-to-collision (TTC) against the sensed lead vehicle.
  2. Road-boundary violation -- has the car actually left the modelled road.
  3. Vehicle-dynamics feasibility -- is REALIZED lateral acceleration within
     a hard safety limit, distinct from the planner's own comfort limit
     (A_LAT_MAX_MPS2 in physics_engine.py is a comfort target the planner
     steers toward; this is the "you are now unsafe" line, strictly beyond
     it, matching how a real ESC/stability system has a harder threshold
     than a comfort-tuned cruise/planning layer).
"""
from dataclasses import dataclass, field
from typing import List, Optional

from app.services.planner import ROAD_HALF_WIDTH_M

# TTC thresholds. Below CRITICAL, a collision is imminent enough that the
# shield overrides with maximum braking regardless of what the planner/IDM
# already decided. Between CRITICAL and WARNING, the situation is flagged
# but not overridden -- IDM is already the primary response to a closing
# lead vehicle; the shield's role there is visibility, not double-braking
# a car that's already braking correctly.
TTC_CRITICAL_S = 2.0
TTC_WARNING_S = 4.0

# Distinct from planner.py's ROAD_HALF_WIDTH_M usage as a candidate-scoring
# input -- this is "has the car actually left the road", checked against
# the SAME real half-width so the two layers can never disagree about
# where the road edge is.
ROAD_BOUNDARY_HARD_LIMIT_M = ROAD_HALF_WIDTH_M

# Strictly greater than physics_engine.py's A_LAT_MAX_MPS2 (3.0, a COMFORT
# target the cornering-speed cap steers toward) -- this is the harder
# "you are now unsafe" line. Chosen well below what would read as an
# arbitrary number: real passenger-car lateral grip limits before loss of
# control are commonly cited around 0.4-0.5g (~4-5 m/s^2) on dry pavement;
# 4.5 sits inside that range with margin below outright loss of traction.
HARD_LATERAL_ACCEL_LIMIT_MPS2 = 4.5

RISK_NONE = "NONE"
RISK_LOW = "LOW"
RISK_MEDIUM = "MEDIUM"
RISK_HIGH = "HIGH"
RISK_CRITICAL = "CRITICAL"

# Real bug found live (2026-08): an earlier version used EMERGENCY_BRAKE
# (forcing acceleration all the way to -A_MAX_BRAKE_MPS2, i.e. a full stop)
# for EVERY override, including road-boundary violations. That created a
# livelock -- yaw_rate = v*tan(delta)/L means the car cannot steer back
# onto the road once braked to a genuine standstill, so it froze off-road,
# permanently re-triggering the same override every tick, forever.
# EMERGENCY_BRAKE is still correct for an imminent collision (TTC
# critical) -- stopping IS the right response there. A road-boundary
# violation or an already-excessive lateral acceleration needs the
# opposite: enough speed preserved to actually regain control, not zero.
OVERRIDE_EMERGENCY_BRAKE = "EMERGENCY_BRAKE"
OVERRIDE_RECOVER_LOW_SPEED = "RECOVER_LOW_SPEED"


@dataclass
class ShieldVerdict:
    approved: bool
    risk_level: str
    reasons: List[str] = field(default_factory=list)
    override_action: Optional[str] = None
    ttc_s: Optional[float] = None


def _risk_rank(level: str) -> int:
    return [RISK_NONE, RISK_LOW, RISK_MEDIUM, RISK_HIGH, RISK_CRITICAL].index(level)


def _worse(a: str, b: str) -> str:
    return a if _risk_rank(a) >= _risk_rank(b) else b


def compute_ttc_s(gap_m: Optional[float], ego_speed_mps: float, lead_speed_mps: Optional[float]) -> Optional[float]:
    """Time-to-collision: gap / closing_speed, or None if there's no lead
    vehicle or the ego isn't actually closing on it (closing_speed <= 0 --
    matching or pulling away is not a collision trajectory, however small
    the gap)."""
    if gap_m is None:
        return None
    closing_speed = ego_speed_mps - (lead_speed_mps if lead_speed_mps is not None else ego_speed_mps)
    if closing_speed <= 0:
        return None
    return gap_m / closing_speed


def evaluate(
    ego_speed_mps: float,
    lateral_offset_m: float,
    lateral_accel_mps2: float,
    sensed_lead_gap_m: Optional[float],
    sensed_lead_speed_mps: Optional[float],
) -> ShieldVerdict:
    """The independent check. Called every tick, after the planner and IDM
    have already produced their decision -- this never influences THEIR
    inputs, it only evaluates the resulting state and may override the
    final commanded acceleration (physics_engine.py composes this the same
    way it already composes cruise/IDM: min() with whatever else was
    decided, so the shield can only ever make the car brake harder than
    planned, never accelerate harder)."""
    reasons: List[str] = []
    risk = RISK_NONE
    override: Optional[str] = None

    def _request_override(action: str) -> None:
        # EMERGENCY_BRAKE (an imminent-collision full stop) always wins
        # over RECOVER_LOW_SPEED if both are requested in the same tick --
        # collision risk trumps recovery-speed preservation. Once
        # EMERGENCY_BRAKE is set, nothing downgrades it.
        nonlocal override
        if override == OVERRIDE_EMERGENCY_BRAKE:
            return
        override = action

    ttc_s = compute_ttc_s(sensed_lead_gap_m, ego_speed_mps, sensed_lead_speed_mps)
    if ttc_s is not None:
        if ttc_s < TTC_CRITICAL_S:
            risk = _worse(risk, RISK_CRITICAL)
            reasons.append(f"Time-to-collision {ttc_s:.1f}s below critical threshold ({TTC_CRITICAL_S}s)")
            _request_override(OVERRIDE_EMERGENCY_BRAKE)
        elif ttc_s < TTC_WARNING_S:
            risk = _worse(risk, RISK_MEDIUM)
            reasons.append(f"Time-to-collision {ttc_s:.1f}s below warning threshold ({TTC_WARNING_S}s)")

    if abs(lateral_offset_m) > ROAD_BOUNDARY_HARD_LIMIT_M:
        risk = _worse(risk, RISK_CRITICAL)
        reasons.append(
            f"Lateral offset {lateral_offset_m:.1f}m exceeds the modelled road half-width "
            f"({ROAD_BOUNDARY_HARD_LIMIT_M}m) -- the car has left the road"
        )
        # RECOVER_LOW_SPEED, not EMERGENCY_BRAKE: braking to a full stop
        # here would remove the only thing that lets the car steer back
        # onto the road (yaw_rate = v*tan(delta)/L needs forward speed) --
        # a livelock found live during testing. A low but nonzero recovery
        # speed keeps enough authority to actually correct.
        _request_override(OVERRIDE_RECOVER_LOW_SPEED)

    if abs(lateral_accel_mps2) > HARD_LATERAL_ACCEL_LIMIT_MPS2:
        risk = _worse(risk, RISK_HIGH)
        reasons.append(
            f"Realized lateral acceleration {lateral_accel_mps2:.1f} m/s^2 exceeds the hard safety "
            f"limit ({HARD_LATERAL_ACCEL_LIMIT_MPS2} m/s^2)"
        )
        _request_override(OVERRIDE_RECOVER_LOW_SPEED)

    approved = override is None
    return ShieldVerdict(approved=approved, risk_level=risk, reasons=reasons,
                         override_action=override, ttc_s=ttc_s)
