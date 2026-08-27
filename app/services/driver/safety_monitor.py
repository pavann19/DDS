"""Independent safety supervisor node (ADR-001, Phase 6.5, Action Item 6).

Lifts the Safety Shield out of ``PhysicsEngine.update()``'s longitudinal
block into a parallel node with **veto-only authority**: it evaluates the
ego's raw physical state after the planner/IDM have decided, and may only
ever make the car brake *harder* -- never accelerate, never steer.

``SafetyMonitor`` wraps :func:`app.services.safety_shield.evaluate` and the
override-composition that previously lived inline. Behaviour is byte-for-byte
what it was: same ``evaluate`` call, same ``min()`` compositions, same
``speed_limit_reason`` string.

What is *structural prep*, not yet real (stated honestly, per the ADR's
"what we'll need to revisit"):

* **Own sensor feed.** Today the monitor is still handed the same
  ``sensed_lead`` the planner used. Phase 11 (RSS/MRM) gives it a genuinely
  separate feed so a planner-side perception bug cannot blind it too.
* **MRM.** The two override actions here (emergency brake / recover-low-
  speed) are the seed of Phase 11's three-tier Minimum Risk Maneuver.

It is independent *by construction* now -- own object, own verdict state,
own logic, veto-only -- while still sharing a process/failure domain with
the driver it supervises. That limitation is acknowledged, not papered over.
"""
from __future__ import annotations

from typing import Optional, Tuple

from app.services import safety_shield
from app.services.safety_shield import ShieldVerdict

OVERRIDE_SPEED_LIMIT_REASON = "safety_shield_override"


class SafetyMonitor:
    def __init__(self) -> None:
        self.verdict: ShieldVerdict = ShieldVerdict(
            approved=True, risk_level=safety_shield.RISK_NONE
        )

    def reset(self) -> None:
        """Return to the 'nothing to report yet' verdict (scenario reset)."""
        self.verdict = ShieldVerdict(approved=True, risk_level=safety_shield.RISK_NONE)

    def step(
        self,
        *,
        ego_speed_mps: float,
        lateral_offset_m: float,
        lateral_accel_mps2: float,
        sensed_lead_gap_m: Optional[float],
        sensed_lead_speed_mps: Optional[float],
        desired_accel: float,
        a_max_brake_mps2: float,
        recovery_speed_kmh: float,
        speed_kp: float,
    ) -> Tuple[float, Optional[str]]:
        """Evaluate the shield and apply its veto to ``desired_accel``.

        Returns ``(possibly_reduced_accel, speed_limit_reason_or_None)``.
        The caller keeps its own ``[-a_brake, +a_accel]`` clamp and jerk
        limiter downstream, exactly as before -- this method does not clamp.
        """
        self.verdict = safety_shield.evaluate(
            ego_speed_mps=ego_speed_mps,
            lateral_offset_m=lateral_offset_m,
            lateral_accel_mps2=lateral_accel_mps2,
            sensed_lead_gap_m=sensed_lead_gap_m,
            sensed_lead_speed_mps=sensed_lead_speed_mps,
        )

        reason: Optional[str] = None
        if self.verdict.override_action == safety_shield.OVERRIDE_EMERGENCY_BRAKE:
            # Imminent collision (TTC critical): force maximum physical
            # braking regardless of what cruise/IDM computed. min() so the
            # monitor can only ever brake harder.
            desired_accel = min(desired_accel, -a_max_brake_mps2)
            reason = OVERRIDE_SPEED_LIMIT_REASON
        elif self.verdict.override_action == safety_shield.OVERRIDE_RECOVER_LOW_SPEED:
            # Road-boundary / hard-lateral-accel: proportional control toward
            # a low but nonzero recovery floor -- braking to a dead stop
            # removes the steering authority needed to get back on the road
            # (yaw_rate = v*tan(delta)/L). Same min() composition.
            recovery_target_mps = recovery_speed_kmh / 3.6
            recovery_accel = speed_kp * (recovery_target_mps - ego_speed_mps)
            desired_accel = min(desired_accel, recovery_accel)
            reason = OVERRIDE_SPEED_LIMIT_REASON

        return desired_accel, reason
