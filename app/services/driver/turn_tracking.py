"""Center-bicycle turn tracking without altering map geometry or actual pose."""
import math
from app.services.frenet import frenet_to_local_xz


def continuous_path_heading(frame, station_m, offset_m=0.0, half_window_m=1.0):
    """Chord tangent avoids steering jumps at individual polyline vertices."""
    if not all(math.isfinite(v) for v in (station_m, offset_m, half_window_m)) or half_window_m <= 0:
        raise ValueError("invalid tangent window")
    x0, z0, _, _ = frenet_to_local_xz(frame, station_m-half_window_m, offset_m)
    x1, z1, _, _ = frenet_to_local_xz(frame, station_m+half_window_m, offset_m)
    dx, dz = x1-x0, z1-z0
    if math.hypot(dx, dz) < 1e-9:
        _, _, dx, dz = frenet_to_local_xz(frame, station_m, offset_m)
    if not all(math.isfinite(v) for v in (dx, dz)):
        raise ValueError("non-finite tangent")
    return math.degrees(math.atan2(dx, -dz))


def center_steer_for_curvature(curvature_1pm, wheelbase_m):
    """Invert k=cos(beta)*tan(delta)/L; reject impossible center curvature."""
    if not math.isfinite(wheelbase_m) or wheelbase_m <= 0:
        raise ValueError("invalid wheelbase")
    q = wheelbase_m*curvature_1pm
    if not math.isfinite(q) or abs(q) >= 2:
        raise ValueError("center bicycle curvature outside geometric domain")
    return math.atan(q/math.sqrt(1-.25*q*q))


def offset_path_curvature(curvature_1pm, offset_m):
    """Positive d is LEFT; positive compass curvature is RIGHT."""
    if not math.isfinite(curvature_1pm) or not math.isfinite(offset_m):
        raise ValueError("non-finite parallel path geometry")
    denominator = 1+curvature_1pm*offset_m
    if denominator <= 0:
        raise ValueError("parallel path folds through curvature center")
    return curvature_1pm/denominator


def center_tracking_steer(*, curvature_1pm, wheelbase_m, heading_error_rad,
                          cross_track_m, speed_mps, steer_limit_rad, actual_steer_rad=0.0):
    """Center geometry plus bounded feedback; nominal gains remain unchanged."""
    values = (heading_error_rad, cross_track_m, speed_mps, steer_limit_rad, actual_steer_rad)
    if not all(math.isfinite(v) for v in values) or speed_mps < 0 or steer_limit_rad < 0:
        raise ValueError("invalid turn tracking inputs")
    feedforward = center_steer_for_curvature(curvature_1pm, wheelbase_m)
    beta = math.atan(.5*math.tan(actual_steer_rad))
    error = (heading_error_rad-beta+math.pi)%(2*math.pi)-math.pi
    cte = max(-3., min(3., cross_track_m))
    desired = feedforward+error+math.atan2(2.2*cte, speed_mps+1.5)
    return max(-steer_limit_rad, min(steer_limit_rad, desired))
