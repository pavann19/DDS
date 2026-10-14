"""Ideal no-slip, center-referenced bicycle and Ackermann wheel geometry.

This is simulated kinematics, not a tire-force or suspension model.
Compass-positive steering turns right; wheel order is FL, FR, RL, RR.
"""
import math

TRACK_WIDTH_M = 1.96
WHEEL_RADIUS_M = 0.34


def sideslip(steer_rad: float) -> float:
    return math.atan(0.5 * math.tan(steer_rad))


def wheel_geometry(steer_rad: float, wheelbase_m: float):
    k = math.tan(steer_rad) / wheelbase_m
    left, right = 1 + TRACK_WIDTH_M * k / 2, 1 - TRACK_WIDTH_M * k / 2
    angles = (math.atan2(wheelbase_m * k, left), math.atan2(wheelbase_m * k, right), 0., 0.)
    # Rear axle velocity is center velocity times cos(beta).
    c = math.cos(sideslip(steer_rad))
    factors = (math.hypot(left, wheelbase_m*k)*c,
               math.hypot(right, wheelbase_m*k)*c, left*c, right*c)
    return angles, factors
