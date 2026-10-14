"""Initial sampled actuator reachability for the center bicycle model.

The velocity heading includes beta-rate; a steady inverse curvature alone is
not the actual steering angle during transients. This first-interval check is
not continuous swept-footprint or full-horizon actuator certification.
"""
import math
from app.services.planner.joint_physics import joint_acceleration_components,reference_curvature_and_derivative


def required_steering_rate(steer,speed,motion_curvature,wheelbase):
    beta=math.atan(.5*math.tan(steer))
    beta_derivative=.5/(math.cos(steer)**2*(1+.25*math.tan(steer)**2))
    body_yaw_rate=speed*math.cos(beta)*math.tan(steer)/wheelbase
    return (speed*motion_curvature-body_yaw_rate)/beta_derivative


def initial_actuator_reachable(lat,lon,ctx,cfg,start=0.):
    actual=ctx.actual_steer_rad
    if actual is None:return True
    if not math.isfinite(actual) or abs(actual)>cfg.max_steer_rad+1e-9 or not math.isfinite(ctx.dt) or ctx.dt<=0:return False
    end=min(max(lat.T,lon.T),start+ctx.dt)
    if end<=start:return False
    def reference(t):
        tl=min(t,lat.T);ts=min(t,lon.T)
        d=lat.pos(tl);dv=0. if t>lat.T else lat.vel(tl);da=0. if t>lat.T else lat.acc(tl)
        station=lon.pos(ts)+max(0.,t-lon.T)*lon.vel(lon.T)
        sv=lon.vel(ts);sa=0. if t>lon.T else lon.acc(ts)
        k,ks=(0.,0.) if ctx.frenet_frame is None else reference_curvature_and_derivative(ctx.frenet_frame,station)
        _,normal,speed=joint_acceleration_components(k,ks,d,dv,da,sv,sa)
        return speed,0. if speed<1e-6 else -normal/(speed*speed)
    steps=max(1,math.ceil((end-start)/.02));dt=(end-start)/steps
    steer=actual
    try:
        for i in range(steps):
            t=start+i*dt
            speed,k=reference(t);rate=required_steering_rate(steer,speed,k,cfg.wheelbase_m)
            mid=steer+dt*rate/2
            speed,k=reference(t+dt/2);mid_rate=required_steering_rate(mid,speed,k,cfg.wheelbase_m)
            if not all(math.isfinite(value) for value in (rate,mid,mid_rate)):return False
            if max(abs(rate),abs(mid_rate))>cfg.max_steer_rate_radps+1e-6:return False
            steer+=dt*mid_rate
            if not math.isfinite(steer) or max(abs(mid),abs(steer))>cfg.max_steer_rad+1e-9:return False
        speed,k=reference(end)
        endpoint_rate=required_steering_rate(steer,speed,k,cfg.wheelbase_m)
        if not math.isfinite(endpoint_rate) or abs(endpoint_rate)>cfg.max_steer_rate_radps+1e-6:return False
    except (ValueError,OverflowError):return False
    return True
