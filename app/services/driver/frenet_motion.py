"""Project measured center-bicycle motion into the local Frenet frame.

The map curvature is a window estimate; derivatives are model estimates, not
measured tire forces or a continuous trajectory certificate.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FrenetMotion:
    station_speed_mps: float
    station_accel_mps2: float
    lateral_speed_mps: float
    lateral_accel_mps2: float


def project_motion(*, speed_mps, acceleration_mps2, motion_heading_rad,
                   road_heading_rad, motion_yaw_rate_radps,
                   curvature_1pm, curvature_derivative_1pm2, lateral_offset_m):
    values = (speed_mps, acceleration_mps2, motion_heading_rad, road_heading_rad,
              motion_yaw_rate_radps, curvature_1pm, curvature_derivative_1pm2, lateral_offset_m)
    if not all(math.isfinite(v) for v in values) or speed_mps < 0:
        raise ValueError('Invalid center-motion state')
    scale = 1. + curvature_1pm*lateral_offset_m
    if scale <= 1e-6:
        raise ValueError('Folded local Frenet frame')
    # The plant enforces non-reversing speed. At exactly zero speed its
    # retained brake-actuator demand is not a negative body acceleration:
    # the unilateral velocity constraint cancels it. Initializing a new
    # polynomial with that demand invents reverse motion and deadlocks every
    # otherwise feasible departure. Positive launch acceleration and all
    # moving-state braking remain unchanged.
    if speed_mps == 0.0 and acceleration_mps2 < 0.0:
        acceleration_mps2 = 0.0
    relative = motion_heading_rad-road_heading_rad
    station_speed = speed_mps*math.cos(relative)/scale
    if station_speed < -1e-6:
        raise ValueError('Reverse route-relative motion is unsupported')
    station_speed = max(0., station_speed)
    lateral_speed = -speed_mps*math.sin(relative)
    right_accel = speed_mps*motion_yaw_rate_radps
    tangent_accel = acceleration_mps2*math.cos(relative)-right_accel*math.sin(relative)
    normal_accel = -acceleration_mps2*math.sin(relative)-right_accel*math.cos(relative)
    station_accel = (tangent_accel-curvature_derivative_1pm2*lateral_offset_m*station_speed**2
                     -2*curvature_1pm*station_speed*lateral_speed)/scale
    lateral_accel = normal_accel+curvature_1pm*scale*station_speed**2
    return FrenetMotion(station_speed, station_accel, lateral_speed, lateral_accel)
