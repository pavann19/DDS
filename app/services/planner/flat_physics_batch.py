"""Batched straight-reference sampled checks with the scalar physics limits."""
import math
from functools import lru_cache
import numpy as np
from app.services.planner.corridor_proof import lateral_corridor_batch
from app.services.planner.actuator_reachability import initial_actuator_reachable
from app.services.grip_budget import grip_acceleration_limit
from app.services.vehicle_geometry import lateral_half_extent_array


@lru_cache(maxsize=32)
def _sampling_layout(lat_durations,lon_durations,sample_n):
    """Bounded cache of immutable duration-only sample grids, never motion/risk."""
    li=np.repeat(np.arange(len(lat_durations)),len(lon_durations))
    si=np.tile(np.arange(len(lon_durations)),len(lat_durations))
    lat_unique,lat_inverse=np.unique(lat_durations,return_inverse=True)
    lon_unique,lon_inverse=np.unique(lon_durations,return_inverse=True)
    temporal_lat=np.repeat(lat_unique,len(lon_unique))[:,None]
    temporal_lon=np.tile(lon_unique,len(lat_unique))[:,None]
    horizon=np.maximum(temporal_lat,temporal_lon)
    uniform=horizon*np.arange(sample_n+1)[None,:]/sample_n
    temporal_grid=np.sort(np.concatenate([uniform,temporal_lat,temporal_lon],axis=1),axis=1)
    pair_times=lat_inverse[li]*len(lon_unique)+lon_inverse[si]
    times=temporal_grid[pair_times]
    lat_indices=np.repeat(np.arange(len(lat_durations)),len(lon_unique))
    lat_grid=lat_inverse[lat_indices]*len(lon_unique)+np.tile(np.arange(len(lon_unique)),len(lat_durations))
    lat_gather=li*len(lon_unique)+lon_inverse[si]
    lon_indices=np.repeat(np.arange(len(lon_durations)),len(lat_unique))
    lon_grid=np.tile(np.arange(len(lat_unique)),len(lon_durations))*len(lon_unique)+lon_inverse[lon_indices]
    lon_gather=si*len(lat_unique)+lat_inverse[li]
    result=(times,lat_indices,lat_grid,lat_gather,lon_indices,lon_grid,lon_gather,temporal_grid)
    for array in result:array.flags.writeable=False
    return result


def _states(polys, times, lateral, indices=None):
    coefficients=np.array([p.coef for p in polys],dtype=float)
    durations=np.array([p.T for p in polys])[:,None]
    if indices is not None:
        coefficients=coefficients[indices]
        durations=durations[indices]
    t=np.minimum(times,durations)
    states=[]
    for derivative in range(4 if lateral else 3):
        factors=np.array([math.factorial(i)/math.factorial(i-derivative)
                          for i in range(derivative,coefficients.shape[1])])
        c=coefficients[:,derivative:]*factors[None,:]
        value=np.zeros_like(t)
        for i in range(c.shape[1]-1,-1,-1):
            np.multiply(value,t,out=value)
            value+=c[:,i,None]
        states.append(value)
    beyond=times>durations
    if lateral:
        for derivative in range(1,4):states[derivative]=np.where(beyond,0.,states[derivative])
    else:
        states[0]=states[0]+states[1]*np.maximum(0.,times-durations)
        states[2]=np.where(beyond,0.,states[2])
    return states


def flat_feasible_lattice(laterals,longs,ctx,cfg,lat_cap,jerk_cap,velocity_bounds):
    """Evaluate the unchanged complete lattice in a single vector batch."""
    pairs=[(lat,lon) for lat in laterals for lon in longs]
    if ctx.actual_steer_rad is not None and (not math.isfinite(ctx.actual_steer_rad) or abs(ctx.actual_steer_rad)>cfg.max_steer_rad+1e-9):
        return {(id(lat[0]),id(lon[0])):None for lat,lon in pairs}
    try:
        aggregate_grip_cap=grip_acceleration_limit(ctx.friction_mu)
    except ValueError:
        return {(id(lat[0]),id(lon[0])):None for lat,lon in pairs}
    # Extract each polynomial once; the Cartesian lattice repeats it many
    # times but does not change its coefficients or duration.
    lp=[item[0] for item in laterals]
    sp=[item[0] for item in longs]
    li=np.repeat(np.arange(len(lp)),len(sp))
    si=np.tile(np.arange(len(sp)),len(lp))
    lat_duration=np.array([item[1] for item in laterals])[li,None]
    lon_duration=np.array([item[1] for item in longs])[si,None]
    times,lat_indices,lat_grid,lat_gather,lon_indices,lon_grid,lon_gather,temporal_grid = _sampling_layout(
        tuple(item[1] for item in laterals),tuple(item[1] for item in longs),cfg.sample_n)
    lat_states=_states(lp,temporal_grid[lat_grid],True,lat_indices)
    d,dv,da,dj=(state[lat_gather] for state in lat_states)
    lon_states=_states(sp,temporal_grid[lon_grid],False,lon_indices)
    station,sv,sa=(state[lon_gather] for state in lon_states)
    d0=np.array([p.coef[0] for p in lp])[li,None]
    d1=np.array([item[2] for item in laterals])[li,None]
    valid=(np.isfinite(d)&np.isfinite(dv)&np.isfinite(da)&np.isfinite(dj)&
        np.isfinite(station)&np.isfinite(sv)&np.isfinite(sa)&(sv>=-1e-6)&
        (np.abs(d)<=cfg.road_half_width_m-cfg.edge_margin_m)&
        (d>=np.minimum(d0,d1)-cfg.max_corridor_excursion_m)&
        (d<=np.maximum(d0,d1)+cfg.max_corridor_excursion_m)&
        (np.abs(da)<=lat_cap)&(np.abs(dj)<=jerk_cap)&
        (sa>=cfg.min_long_accel_mps2)&(sa<=cfg.max_long_accel_mps2))
    speed=np.hypot(sv,dv)
    divisor=np.where(speed<1e-6,1.,speed)
    normal=np.where(speed<1e-6,da,(sv*da-dv*sa)/divisor)
    longitudinal=np.where(speed<1e-6,sa,(sv*sa+dv*da)/divisor)
    curvature=np.where(speed<1e-6,0.,-normal/np.maximum(speed*speed,1e-12))
    q=cfg.wheelbase_m*curvature
    steer=np.arctan(q/np.sqrt(np.maximum(1.-.25*q*q,1e-300)))
    for i in range(1,times.shape[1]):
        steer[:,i]=np.where(speed[:,i]<1e-6,steer[:,i-1],steer[:,i])
    valid&=(np.hypot(longitudinal,normal)<=aggregate_grip_cap+1e-9)
    valid&=((np.abs(normal)<=lat_cap)&(longitudinal>=cfg.min_long_accel_mps2)&
        (longitudinal<=cfg.max_long_accel_mps2)&~((np.abs(sv)<1e-6)&(np.abs(dv)>1e-6))&
        (np.abs(q)<2.)&(np.abs(steer)<=cfg.max_steer_rad+1e-9))
    body_steer=steer.copy()
    if ctx.actual_steer_rad is not None:body_steer[:,0]=ctx.actual_steer_rad
    body_heading=-np.arctan2(dv,sv)-np.arctan(.5*np.tan(body_steer))
    body_extent=lateral_half_extent_array(body_heading,body_steer,cfg.wheelbase_m,cfg.vehicle_chassis_half_width_m,cfg.vehicle_half_length_m)
    valid&=np.abs(d)+body_extent <= cfg.road_half_width_m-cfg.edge_margin_m+cfg.corridor_numeric_tolerance_m
    over=speed>ctx.max_speed_mps+1e-6
    nonincreasing=np.concatenate([np.ones_like(speed[:,:1],dtype=bool),np.diff(speed,axis=1)<=1e-6],axis=1)
    valid&=(~over)|(over[:,0,None]&(longitudinal<=1e-6)&nonincreasing&(times<lon_duration-1e-9))
    rate=np.abs(np.diff(steer,axis=1))/np.maximum(np.diff(times,axis=1),1e-300)
    accepted=np.all(valid,axis=1)&np.all(rate<=cfg.max_steer_rate_radps+1e-9,axis=1)
    accepted&=np.array([velocity_bounds[id(p)][0]>=-1e-6 for p in sp])[si]
    continuous=lateral_corridor_batch(laterals,cfg)
    accepted&=np.array([continuous[id(p)] for p in lp])[li]
    if ctx.actual_steer_rad is not None:
        for i,(lat,lon) in enumerate(pairs):
            if accepted[i] and not initial_actuator_reachable(lat[0],lon[0],ctx,cfg):accepted[i]=False
    peak_lat=np.maximum(np.max(np.abs(normal),axis=1),np.max(np.abs(da),axis=1))
    peak_jerk=np.max(np.abs(dj),axis=1)
    peak_long=np.maximum(np.max(np.abs(longitudinal),axis=1),np.max(np.abs(sa),axis=1))
    return {(id(lat[0]),id(lon[0])):(float(peak_lat[i]),float(peak_jerk[i]),float(peak_long[i])) if accepted[i] else None
            for i,(lat,lon) in enumerate(pairs)}
