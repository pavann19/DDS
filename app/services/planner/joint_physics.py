"""Sampled Frenet trajectory physics (positive left d, compass-right curvature).

This is local map-geometry kinematics, not a tire-force or corridor certificate.
"""
import math
from app.services.frenet import frenet_to_local_xz


def joint_acceleration_components(k, k_s, d, d_dot, d_ddot, s_dot, s_ddot):
    """Return (longitudinal, left-normal acceleration, Cartesian speed)."""
    if not all(math.isfinite(v) for v in (k,k_s,d,d_dot,d_ddot,s_dot,s_ddot)):
        raise ValueError('non-finite joint trajectory')
    return _joint_acceleration_components(k,k_s,d,d_dot,d_ddot,s_dot,s_ddot)


def _joint_acceleration_components(k,k_s,d,d_dot,d_ddot,s_dot,s_ddot):
    """Internal fast path; caller has validated finite trajectory samples."""
    scale = 1+k*d
    if scale <= 0 or s_dot < -1e-6:
        raise ValueError('folded or reversing Frenet trajectory')
    vt, vn = scale*s_dot, d_dot
    at = scale*s_ddot + k_s*d*s_dot*s_dot + 2*k*s_dot*d_dot
    an = d_ddot - k*scale*s_dot*s_dot
    speed = math.hypot(vt,vn)
    # At rest, velocity normal is undefined; conservatively retain road-normal demand.
    if speed < 1e-6:
        return at, an, speed
    return (vt*at+vn*an)/speed, (vt*an-vn*at)/speed, speed


def joint_lateral_acceleration(k, k_s, d, d_dot, d_ddot, s_dot, s_ddot):
    """Signed acceleration normal to Cartesian velocity, positive toward left."""
    return joint_acceleration_components(k, k_s, d, d_dot, d_ddot, s_dot, s_ddot)[1]


def reference_curvature(frame, station_m, half_window_m=6.0):
    """Unclipped window estimate; never conceal tight reference turns via a cap."""
    if not math.isfinite(station_m) or not math.isfinite(half_window_m) or half_window_m <= 0:
        raise ValueError("invalid curvature window")
    lo,hi=max(0.,station_m-half_window_m),min(frame.station[-1],station_m+half_window_m)
    if hi-lo < 1e-6:return 0.
    _,_,x0,z0=frenet_to_local_xz(frame,lo,0.)
    _,_,x1,z1=frenet_to_local_xz(frame,hi,0.)
    if not all(math.isfinite(v) for v in (x0,z0,x1,z1)):
        raise ValueError("non-finite map tangent")
    delta=(math.atan2(x1,-z1)-math.atan2(x0,-z0)+math.pi)%(2*math.pi)-math.pi
    return delta/(hi-lo)


def reference_curvature_and_derivative(frame, station_m):
    k=reference_curvature(frame,station_m)
    lo,hi=max(0.,station_m-3.),min(frame.station[-1],station_m+3.)
    derivative=0. if hi-lo<1e-6 else (reference_curvature(frame,hi)-reference_curvature(frame,lo))/(hi-lo)
    return k,derivative
