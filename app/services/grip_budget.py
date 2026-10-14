"""Conservative acceleration budget for the existing no-slip bicycle simulator.

μg is an explicit model envelope, not measured tire performance. Changing-wheel
sideslip is reserved by a signed interval bound; this does not simulate tire slip.
"""
import math
from dataclasses import dataclass
from app.services.vehicle_kinematics import sideslip

GRAVITY_MPS2 = 9.81


def grip_acceleration_limit(friction_mu):
    if not math.isfinite(friction_mu):
        raise ValueError("Friction must be finite")
    return max(.1, min(1., friction_mu))*GRAVITY_MPS2


@dataclass(frozen=True)
class GripAllocation:
    steer_rad: float
    acceleration_mps2: float
    lateral_bound_mps2: float
    limit_mps2: float
    already_infeasible: bool
    steering_limited: bool
    acceleration_limited: bool


def allocate_grip(*, speed_mps, previous_steer_rad, requested_steer_rad,
                  requested_acceleration_mps2, friction_mu, dt_s, wheelbase_m,
                  max_steer_rad, steer_rate_radps, emergency, max_braking_mps2):
    values=(speed_mps,previous_steer_rad,requested_steer_rad,
            requested_acceleration_mps2,dt_s,wheelbase_m,max_steer_rad,
            steer_rate_radps,max_braking_mps2)
    if not all(math.isfinite(v) for v in values) or min(dt_s,wheelbase_m)<=0 or min(speed_mps,max_steer_rad,steer_rate_radps,max_braking_mps2)<0:
        raise ValueError("Invalid grip allocation state/geometry")
    if abs(previous_steer_rad)>max_steer_rad+1e-9 or max_steer_rad>=math.pi/2:
        raise ValueError("Previous steering exceeds mechanical domain")
    limit=grip_acceleration_limit(friction_mu)
    def curvature(steer):
        return math.cos(sideslip(steer))*math.tan(steer)/wheelbase_m
    old_curvature=abs(curvature(previous_steer_rad))
    already_infeasible=speed_mps*speed_mps*old_curvature>limit+1e-9
    target=max(-max_steer_rad,min(max_steer_rad,requested_steer_rad))
    if emergency and not already_infeasible:
        # Keep the actual wheels stable: prioritize the largest remaining brake
        # force without an instantaneous wheel unwind or invented yaw saturation.
        target=0. if speed_mps*speed_mps*old_curvature>=limit*(1.-1e-6) else previous_steer_rad
    if already_infeasible:
        # No instantaneous command can certify the initial state. Unwind at the
        # actual actuator rate and brake to reduce speed; report the violation.
        target=0.
    step=steer_rate_radps*dt_s
    steer=previous_steer_rad+max(-step,min(step,target-previous_steer_rad))
    def lateral_bound(angle, end_speed):
        # Assume bounded constant virtual steering slew across this tick. The
        # center velocity turns at v*k(delta)+beta_prime(delta)*delta_dot.
        # Bound its SIGNED normal acceleration over the complete steering and
        # speed intervals. Cancellation while unwinding is physically relevant:
        # an unsigned triangle reservation can trap an emergency at full grip.
        low_speed=max(0.,min(speed_mps,end_speed))
        high_speed=max(speed_mps,end_speed)
        k0,k1=curvature(previous_steer_rad),curvature(angle)
        def beta_prime(steer):
            tangent=math.tan(steer)
            return .5*(1.+tangent*tangent)/(1.+.25*tangent*tangent)
        closest=0. if previous_steer_rad*angle<=0. else min((previous_steer_rad,angle),key=abs)
        bp_low=beta_prime(closest)
        bp_high=max(beta_prime(previous_steer_rad),beta_prime(angle))
        rate=(angle-previous_steer_rad)/dt_s
        yaw_terms=[v*v*k for v in (low_speed,high_speed) for k in (k0,k1)]
        slew_terms=[v*bp*rate for v in (low_speed,high_speed) for bp in (bp_low,bp_high)]
        return max(abs(min(yaw_terms)+min(slew_terms)),abs(max(yaw_terms)+max(slew_terms)))
    if not already_infeasible and lateral_bound(steer,speed_mps)>limit:
        low,high=0.,1.
        for _ in range(30):
            fraction=(low+high)/2
            angle=previous_steer_rad+fraction*(steer-previous_steer_rad)
            if lateral_bound(angle,speed_mps)<=limit:low=fraction
            else:high=fraction
        steer=previous_steer_rad+low*(steer-previous_steer_rad)
    requested=(-max_braking_mps2 if emergency else requested_acceleration_mps2)
    desired=requested
    def feasible(acceleration):
        bound=lateral_bound(steer,speed_mps+acceleration*dt_s)
        return math.hypot(acceleration,bound)<=limit
    if already_infeasible and lateral_bound(steer,speed_mps)>limit:
        acceleration=-min(max_braking_mps2,limit)
    else:
        if already_infeasible:desired=-min(max_braking_mps2,limit)
        acceleration=desired
        if not feasible(desired):
            low,high=0.,abs(desired)
            sign=-1. if desired<0. else 1.
            for _ in range(30):
                middle=(low+high)/2
                if feasible(sign*middle):low=middle
                else:high=middle
            acceleration=sign*low
    bound=lateral_bound(steer,speed_mps+acceleration*dt_s)
    return GripAllocation(steer,acceleration,bound,limit,already_infeasible,
        abs(steer-requested_steer_rad)>1e-9,abs(acceleration-requested)>1e-9)
