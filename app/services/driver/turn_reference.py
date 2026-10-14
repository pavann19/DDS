"""Unclipped, simulated turn reference and anticipatory physical speed limits.

Recorded route points stay unchanged. Curvature is a 12 m heading-change
reference, not surveyed lane geometry or a certified trajectory.
"""
from dataclasses import dataclass
import math
import numpy as np

TURN_REFERENCE_HALF_WINDOW_M = 6.0

@dataclass(frozen=True)
class TurnSpeedConstraint:
    speed_cap_mps: float
    braking_mps2: float
    reason: str
    peak_curvature_1pm: float
    geometric_infeasibility: bool

class TurnSpeedReference:
    def __init__(self,frame,offset_m,wheelbase_m,max_steer_rad,step_m=1.,window_m=TURN_REFERENCE_HALF_WINDOW_M):
        if not all(math.isfinite(v) for v in (offset_m,wheelbase_m,max_steer_rad,step_m,window_m)) or min(wheelbase_m,max_steer_rad,step_m,window_m)<=0:
            raise ValueError('invalid turn reference configuration')
        if len(frame.station)<2 or not np.all(np.isfinite(frame._points_array)) or not np.all(np.isfinite(frame._station_array)) or not np.all(np.diff(frame._station_array)>0):
            raise ValueError("invalid recorded route geometry")
        self.frame=frame;self.offset_m=offset_m
        self.stations=np.arange(0.,frame.total_length_m+step_m,step_m)
        self.stations=np.minimum(self.stations,frame.total_length_m)
        headings=_route_headings(frame)
        def heading(s):
            index=np.clip(np.searchsorted(frame._station_array,s,side='left')-1,0,len(headings)-1)
            return headings[index]
        lo=np.maximum(0.,self.stations-window_m);hi=np.minimum(frame.total_length_m,self.stations+window_m)
        raw=(heading(hi)-heading(lo))/np.maximum(hi-lo,1e-9)
        denominator=1+raw*offset_m
        self.folded=denominator<=0
        self.arc_scale=np.maximum(denominator,0.)
        self.arc=np.r_[0.,np.cumsum(.5*(self.arc_scale[1:]+self.arc_scale[:-1])*np.diff(self.stations))]
        self.curvature=raw/np.maximum(denominator,1e-12)
        sine=wheelbase_m*self.curvature/2
        valid=np.abs(sine)<1
        self.steer=np.arctan(wheelbase_m*self.curvature/np.sqrt(np.maximum(1-sine*sine,1e-12)))
        self.infeasible=self.folded|(~valid)|(np.abs(self.steer)>max_steer_rad+1e-9)
        gradient=np.abs(np.diff(self.steer)/np.diff(self.stations))
        self.steer_gradient=np.maximum(np.r_[gradient[0],gradient],np.r_[gradient,gradient[-1]])
        for array in (self.stations,self.curvature,self.steer,self.infeasible,self.steer_gradient,self.arc_scale,self.arc):
            array.flags.writeable=False

    def constraint(self,station_m,speed_mps,accel_mps2,mu,a_lat_max,steer_rate_max,braking=1.,jerk=2.5):
        values=(station_m,speed_mps,accel_mps2,mu,a_lat_max,steer_rate_max,braking,jerk)
        if not all(math.isfinite(v) for v in values) or speed_mps<0 or not .1<=mu<=1 or min(a_lat_max,steer_rate_max,braking,jerk)<=0:
            raise ValueError('invalid turn envelope state')
        if station_m < -1e-6 or station_m > self.frame.total_length_m+1e-6:
            raise ValueError('turn query outside recorded route')
        lookahead=max(60.,speed_mps*speed_mps/(2*braking)+speed_mps*(max(0.,accel_mps2)+braking)/jerk+10.)
        current_arc=float(np.interp(station_m,self.stations,self.arc))
        mask=(self.stations>=max(0.,station_m-1.))&(self.arc<=current_arc+lookahead)
        s=self.stations[mask];k=self.curvature[mask]
        if not len(s):return TurnSpeedConstraint(float('inf'),0.,'cruise',0.,False)
        lateral=np.sqrt(a_lat_max*mu/np.maximum(np.abs(k),1e-12))
        rate=steer_rate_max*self.arc_scale[mask]/np.maximum(self.steer_gradient[mask],1e-12)
        caps=np.minimum(lateral,rate)
        caps=np.where(self.infeasible[mask],0.,caps)
        distances=np.maximum(0.,self.arc[mask]-current_arc)
        initial=max(0.,accel_mps2)
        ramp=(initial+braking)/jerk
        dv=initial*ramp-.5*jerk*ramp*ramp
        c=.5*initial*ramp*ramp-jerk*ramp**3/6
        b=dv+braking*ramp
        allowed=np.maximum(caps,-b+np.sqrt(np.maximum(0.,b*b-dv*dv-2*braking*c+caps*caps+2*braking*distances)))
        allowed=np.where(distances<=1e-9,caps,allowed)
        idx=int(np.argmin(allowed));v_cap=float(allowed[idx])
        margin=speed_mps*ramp+c
        remaining=max(.1,float(distances[idx])-margin)
        required=max(0.,((speed_mps+dv)**2-float(caps[idx])**2)/(2*remaining))
        reason='turn_geometry_infeasible' if self.infeasible[mask][idx] else ('turn_steering_rate' if rate[idx]<lateral[idx] else 'turn_geometry_speed_limit')
        return TurnSpeedConstraint(v_cap,min(braking,required),reason,float(np.max(np.abs(k))),bool(np.any(self.infeasible[mask])))


def raw_route_curvature(frame,station_m,window_m=TURN_REFERENCE_HALF_WINDOW_M):
    """Heading-change curvature with no silent steering-radius clipping."""
    from app.services.frenet import frenet_to_local_xz
    if not math.isfinite(station_m) or not math.isfinite(window_m) or window_m<=0:
        raise ValueError('invalid route curvature query')
    lo=max(0.,station_m-window_m);hi=min(frame.total_length_m,station_m+window_m)
    if hi-lo<1e-9:return 0.
    index_lo=int(np.clip(np.searchsorted(frame._station_array,lo,side="left")-1,0,len(frame.points_xz)-2))
    index_hi=int(np.clip(np.searchsorted(frame._station_array,hi,side="left")-1,0,len(frame.points_xz)-2))
    heading=_route_headings(frame)[index_lo:index_hi+1]
    difference=float(heading[-1]-heading[0])
    if abs(difference)>=math.pi:
        raise ValueError('turn reference exceeds unambiguous heading domain')
    return difference/(hi-lo)



def predict_front_axle(x,z,heading_rad,speed_mps,steer_rad,wheelbase_m,preview_m):
    """Constant actual-steer bicycle preview, including body rotation.

    The old straight translation left the front axle at its current body
    angle while comparing to a future road tangent. No real pose is changed.
    """
    values=(x,z,heading_rad,speed_mps,steer_rad,wheelbase_m,preview_m)
    if not all(math.isfinite(v) for v in values) or speed_mps<0 or wheelbase_m<=0 or preview_m<0 or abs(steer_rad)>=math.pi/2:
        raise ValueError('invalid physical preview state')
    beta=math.atan(.5*math.tan(steer_rad))
    time=preview_m/max(speed_mps,.01)
    yaw_rate=speed_mps*math.cos(beta)*math.tan(steer_rad)/wheelbase_m
    turn=yaw_rate*time
    half=turn/2
    sinc=math.sin(half)/half if abs(half)>1e-9 else 1-half*half/6
    travel=speed_mps*time*sinc
    middle=heading_rad+beta+half
    future_yaw=heading_rad+turn
    return (x+travel*math.sin(middle)+wheelbase_m/2*math.sin(future_yaw),
            z-travel*math.cos(middle)-wheelbase_m/2*math.cos(future_yaw),future_yaw)



def _route_headings(frame):
    """Retain the existing isolated-collinear-backtrack artifact contract.

    Only a short reversed run fully overlapping its incoming segment and
    immediately returning to that same forward direction is rejected as
    a reference artifact. Real curved hairpins remain infeasible; the
    recorded positions and route projection are never edited.
    """
    vectors=np.diff(frame._points_array,axis=0)
    lengths=np.hypot(vectors[:,0],vectors[:,1])
    units=vectors/np.maximum(lengths[:,None],1e-12)
    headings=np.arctan2(vectors[:,0],-vectors[:,1])
    candidates=np.flatnonzero(np.sum(units[:-1]*units[1:],axis=1)<-1+1e-8)+1
    for start in candidates:
        base=units[start-1]
        end=start
        reversed_length=0.
        while end<len(units) and float(np.dot(base,units[end]))<-1+1e-8:
            reversed_length+=float(lengths[end]);end+=1
        if end<len(units) and float(np.dot(base,units[end]))>1-1e-8 and reversed_length<=min(6.,float(lengths[start-1]))+1e-6:
            headings[start:end]=headings[start-1]
    return np.unwrap(headings)



def front_axle_lateral_bias(curvature_1pm,wheelbase_m):
    """Physical front axle's concentric radius relative to center-reference.

    Rear radius is sqrt(Rcenter^2-(L/2)^2), front radius is
    sqrt(Rcenter^2+3*(L/2)^2). Positive Frenet d is LEFT.
    """
    if not all(math.isfinite(v) for v in (curvature_1pm,wheelbase_m)) or wheelbase_m<=0 or abs(curvature_1pm*wheelbase_m)>=2:
        raise ValueError('invalid physical axle reference')
    if abs(curvature_1pm)<1e-12:return 0.
    radius=1/abs(curvature_1pm)
    square=3*(wheelbase_m/2)**2
    return math.copysign(square/(math.sqrt(radius*radius+square)+radius),curvature_1pm)



def front_axle_heading_advance(curvature_1pm,wheelbase_m):
    """Front path tangent minus center motion tangent, equal delta-beta."""
    front_axle_lateral_bias(curvature_1pm,wheelbase_m)
    q=wheelbase_m*curvature_1pm
    delta=math.atan(q/math.sqrt(1-.25*q*q))
    beta=math.asin(.5*q)
    return delta-beta


@dataclass(frozen=True)
class SteeringDemandConstraint:
    speed_cap_mps: float
    restore_brake_mps2: float
    center_curvature_1pm: float


def steering_demand_constraint(*,desired_steer_rad,wheelbase_m,speed_mps,
                               actual_steer_rad,lateral_accel_mps2,mu,
                               lateral_limit_mps2,brake_limit_mps2,
                               steer_rate_radps,dt_s):
    """Restore an achievable requested turn without changing actuator limits.

    The center curvature cap is paired with the existing conservative
    rear-bicycle actuator bound. Restore braking is limited by the remaining
    modeled friction ellipse and the existing 1 m/s² comfort policy; the
    world still applies jerk and the existing separate grip/slew bounds.
    This bounds the additional restoration request only; aggregate PID,
    IDM, junction and emergency braking are not globally bounded by this
    ellipse. The simulator is kinematic and has no tire-force model.
    """
    values=(desired_steer_rad,wheelbase_m,speed_mps,actual_steer_rad,
            lateral_accel_mps2,mu,lateral_limit_mps2,brake_limit_mps2,
            steer_rate_radps,dt_s)
    if not all(math.isfinite(v) for v in values) or speed_mps<0 or not .1<=mu<=1. or min(wheelbase_m,lateral_limit_mps2,brake_limit_mps2,steer_rate_radps)<=0 or dt_s<0 or max(abs(desired_steer_rad),abs(actual_steer_rad))>=math.pi/2:
        raise ValueError('invalid steering demand state')
    if abs(desired_steer_rad)<1e-8:
        return SteeringDemandConstraint(float('inf'),0.,0.)
    beta=math.atan(.5*math.tan(desired_steer_rad))
    curvature=abs(math.cos(beta)*math.tan(desired_steer_rad)/wheelbase_m)
    physical_cap=math.sqrt(lateral_limit_mps2*mu/curvature)
    actuator_cap=math.sqrt(lateral_limit_mps2*mu*wheelbase_m/abs(math.tan(desired_steer_rad)))
    cap=min(physical_cap,actuator_cap)
    brake=0.
    if speed_mps>cap:
        restore_time=max(1e-6,dt_s,abs(desired_steer_rad-actual_steer_rad)/steer_rate_radps)
        requested=(speed_mps-cap)/restore_time
        lateral_fraction=min(1.,abs(lateral_accel_mps2)/(lateral_limit_mps2*mu))
        available=brake_limit_mps2*mu*math.sqrt(max(0.,1-lateral_fraction*lateral_fraction))
        brake=min(1.,available,requested)
    return SteeringDemandConstraint(cap,brake,curvature)
