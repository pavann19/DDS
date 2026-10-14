"""Nominal 2D chassis and tire footprint matching the simulator's rendered geometry.

This is a modeled planar envelope, not a measured physical vehicle or 3D swept
suspension certificate. Wheel order and virtual steer follow vehicle_kinematics.
"""
import math
from app.services.vehicle_kinematics import TRACK_WIDTH_M,WHEEL_RADIUS_M

TIRE_WIDTH_M=.22
CHASSIS_HALF_WIDTH_M=.99
CHASSIS_HALF_LENGTH_M=2.4
MIN_LATERAL_HALF_EXTENT_M=TRACK_WIDTH_M/2+TIRE_WIDTH_M/2


def lateral_half_extent(body_yaw_rad,steer_rad,wheelbase_m=2.8,chassis_half_width_m=CHASSIS_HALF_WIDTH_M,half_length_m=CHASSIS_HALF_LENGTH_M):
    if not all(math.isfinite(v) for v in (body_yaw_rad,steer_rad,wheelbase_m,chassis_half_width_m,half_length_m)) or wheelbase_m<=0:
        raise ValueError('invalid vehicle footprint geometry')
    c,s=math.cos(body_yaw_rad),math.sin(body_yaw_rad)
    extent=chassis_half_width_m*abs(c)+half_length_m*abs(s)
    # Rear wheels alone enclose the minimum 1.09 m support in every body yaw.
    rear=(TRACK_WIDTH_M/2+TIRE_WIDTH_M/2)*abs(c)+(wheelbase_m/2+WHEEL_RADIUS_M)*abs(s)
    extent=max(extent,rear)
    k=math.tan(steer_rad)/wheelbase_m
    for x,denominator in ((-TRACK_WIDTH_M/2,1+TRACK_WIDTH_M*k/2),(TRACK_WIDTH_M/2,1-TRACK_WIDTH_M*k/2)):
        wheel_steer=math.atan2(wheelbase_m*k,denominator)
        wheel_yaw=body_yaw_rad+wheel_steer
        wheel_extent=abs(x*c+wheelbase_m/2*s)+TIRE_WIDTH_M/2*abs(math.cos(wheel_yaw))+WHEEL_RADIUS_M*abs(math.sin(wheel_yaw))
        extent=max(extent,wheel_extent)
    return extent


def lateral_half_extent_array(body_yaw_rad,steer_rad,wheelbase_m=2.8,chassis_half_width_m=CHASSIS_HALF_WIDTH_M,half_length_m=CHASSIS_HALF_LENGTH_M):
    import numpy as np
    c,s=np.cos(body_yaw_rad),np.sin(body_yaw_rad)
    extent=np.maximum(chassis_half_width_m*np.abs(c)+half_length_m*np.abs(s),
        (TRACK_WIDTH_M/2+TIRE_WIDTH_M/2)*np.abs(c)+(wheelbase_m/2+WHEEL_RADIUS_M)*np.abs(s))
    k=np.tan(steer_rad)/wheelbase_m
    for x,denominator in ((-TRACK_WIDTH_M/2,1+TRACK_WIDTH_M*k/2),(TRACK_WIDTH_M/2,1-TRACK_WIDTH_M*k/2)):
        wheel_steer=np.arctan2(wheelbase_m*k,denominator)
        wheel_yaw=body_yaw_rad+wheel_steer
        wheel_extent=np.abs(x*c+wheelbase_m/2*s)+TIRE_WIDTH_M/2*np.abs(np.cos(wheel_yaw))+WHEEL_RADIUS_M*np.abs(np.sin(wheel_yaw))
        extent=np.maximum(extent,wheel_extent)
    return extent
