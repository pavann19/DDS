"""Synthetic world camera boundary. Renderer owns truth; detector accepts pixels only.
Map expectations retain location/kind priors, never an unseen signal phase.
"""
import math
from bisect import bisect_left
from dataclasses import replace
from app.services.camera_rig import CAMERA_YAWS_DEG
from app.services.junctions import ControlObservation, simulated_signal_phase
from app.services.frenet import frenet_to_local_xz, latlng_to_local
from app.services.control_camera import CameraCalibration, ControlPrimitive, render_control_camera, detect_controls

class ControlCameraPipeline:
    def __init__(self, *, source="synthetic_RGB_camera_classical_CV"):
        self.source=source
        self.ttl_s=.35 if source=="scene_RGB_camera_classical_CV" else .15
        self.frame=None
        self.camera_frames={}
        self.camera_detections={}
        self.detections=()
        self.readings={}
        self.confirmations={}
        self.fault=None

    def projected_control(self,p,c):
        x,z,tx,tz=frenet_to_local_xz(p.frenet_frame,c.station_m,0.)
        heading=math.atan2(tx,-tz)
        # Simulator roadside placement; map node is not a surveyed sign pose.
        if self.source == "scene_RGB_camera_classical_CV":
            x,z = scene_roadside_position(p.frenet_frame,c.station_m)
        else:
            x+=10.*math.cos(heading);z+=10.*math.sin(heading)
        ex,ez=latlng_to_local(p.lat,p.lng,p.frenet_frame.origin_lat,p.frenet_frame.origin_lng)
        theta=math.radians(p.heading);dx,dz=x-ex,z-ez
        return dx*math.sin(theta)-dz*math.cos(theta),dx*math.cos(theta)+dz*math.sin(theta)

    def acquire(self,p):
        now=p.clock.sim_time_s
        try:
            primitives=[]
            for c in p.route_controls:
                forward,right=self.projected_control(p,c)
                if forward<=0:continue
                kind={'traffic_signals':'signal','give_way':'yield'}.get(c.kind,c.kind)
                phase={'RED':'red','YELLOW':'amber','GREEN':'green'}[simulated_signal_phase(c.node_id,now)] if kind=='signal' else 'unknown'
                primitives.append(ControlPrimitive(kind=kind,forward_m=forward,right_m=right,phase=phase))
            self.frame=render_control_camera(tuple(primitives),now,CameraCalibration(hfov_deg=140.),
                occlusions=((0,0,640,360),) if p.camera_occluded else (),drop=p.camera_dropout)
            self.detections=detect_controls(self.frame)
            self.fault=None
        except Exception as error:
            self.frame=None;self.detections=();self.fault=type(error).__name__
        self._associate(p)

    def acquire_scene(self,p):
        pending=p.scene_camera_pending
        if pending is None or p.camera_dropout:
            return
        p.scene_camera_pending=None
        age=p.clock.sim_time_s-pending.frame.capture_time_s
        if not math.isfinite(age) or not 0<=age<=self.ttl_s:
            self.confirmations={};self.readings={};self.fault='stale_scene_frame'
            return
        if self.frame is not None and pending.frame.capture_time_s<=self.frame.capture_time_s:
            return
        if self.frame is not None and pending.frame.capture_time_s-self.frame.capture_time_s>self.ttl_s:
            self.confirmations={}
        self.frame=pending.frame
        self.camera_frames={"front":pending.frame,**dict(pending.auxiliary_frames)}
        self.camera_detections={"front":pending.detections,**dict(pending.auxiliary_detections)}
        self.detections=pending.detections
        self.fault=None
        self._associate(p)

    def _associate(self,p):
        self.readings={}
        current={}
        # Associate by observed bearing/range and map priors. Ambiguous pairings
        # are UNKNOWN; no world identifier is present in a detection.
        candidates=[]
        views=self.camera_detections if self.source=='scene_RGB_camera_classical_CV' and self.camera_detections else {'front':self.detections}
        for camera_id,hits in views.items():
            yaw=CAMERA_YAWS_DEG.get(camera_id,0.)
            for i,d in enumerate(hits):
                if d.range_m is None or d.range_uncertainty_m is None: continue
                bearing=(d.bearing_deg+yaw+180.)%360.-180.
                factor=math.cos(math.radians(bearing))/max(.01,math.cos(math.radians(d.bearing_deg)))
                if factor<=0: continue
                candidates.append(((camera_id,i),replace(d,bearing_deg=bearing,
                    range_m=d.range_m*factor,range_uncertainty_m=d.range_uncertainty_m*factor)))
        pairs=[]
        for c in p.route_controls:
            f,r=self.projected_control(p,c)
            if f<=0:continue
            expected=math.degrees(math.atan2(r,f))
            kind={'traffic_signals':'signal','give_way':'yield'}.get(c.kind,c.kind)
            for i,d in candidates:
                if d.kind==kind and d.confidence_heuristic>=.3 and abs(d.bearing_deg-expected)<=4. and abs(d.range_m-f)<=max(8.,d.range_uncertainty_m):
                    pairs.append((c,i,d))
        for c in p.route_controls:
            matched=[(i,d) for x,i,d in pairs if x.node_id==c.node_id]
            if not matched: continue
            # Each view must identify a unique control; multiple cameras may
            # corroborate it but cannot count as multiple temporal captures.
            if len({i[0] for i,d in matched})!=len(matched): continue
            if any(sum(j==i for _,j,_ in pairs)!=1 for i,d in matched): continue
            if any(d.confidence_heuristic<.7 for i,d in matched): continue
            phases={d.phase for i,d in matched} if c.kind=='traffic_signals' else {None}
            if len(phases)!=1: continue
            phase={'red':'RED','amber':'YELLOW','green':'GREEN'}.get(next(iter(phases))) if c.kind=='traffic_signals' else None
            if c.kind=='traffic_signals' and phase is None: continue
            key=(c.node_id,phase)
            current[key]=self.confirmations.get(key,0)+1
            if current[key]>=3 and self.frame is not None:
                self.readings[c.node_id]=(phase,self.frame.capture_time_s)
        self.confirmations=current

    def observations(self,p):
        now=p.clock.sim_time_s
        out=[]
        for c in p.route_controls:
            if not -20<=c.station_m-p.current_station_m<=200:continue
            match=self.readings.get(c.node_id)
            stamp=match[1] if match else (self.frame.capture_time_s if self.frame else -math.inf)
            observed=match is not None and math.isfinite(stamp) and 0<=now-stamp<=self.ttl_s
            out.append(ControlObservation(c,match[0] if observed else None,stamp,
                source=self.source,identity_observed=observed))
        return tuple(out)

    def state(self,now):
        age=now-self.frame.capture_time_s if self.frame else None
        return {'mode':'scene_camera_prototype' if self.source=='scene_RGB_camera_classical_CV' else 'camera_prototype','source':self.source,
            'capture_time_s':self.frame.capture_time_s if self.frame else None,
            'age_s':age,'fresh':age is not None and 0<=age<=self.ttl_s,
            'camera_views':tuple({'camera_id':name,'capture_time_s':frame.capture_time_s,'width':frame.calibration.width,'height':frame.calibration.height,'hfov_deg':frame.calibration.hfov_deg,'detections':len(self.camera_detections.get(name,()))} for name,frame in self.camera_frames.items()),
            'detections':len(self.detections),'confirmed_controls':sum(math.isfinite(stamp) and 0<=now-stamp<=self.ttl_s for _,stamp in self.readings.values()),
            'fault':self.fault,'scope':'actual rendered scene; authoritative actor poses; not physical camera' if self.source=='scene_RGB_camera_classical_CV' else 'controlled synthetic imagery; not real-world recognition'}


def scene_roadside_position(frame, station_m, half_width_m=8.8, clearance_m=1.2):
    """Match routeGeometry.ts roadsidePosition's displayed mitered asphalt.

    Only a modeled location/kind prior; phase remains pixel-derived. A curved
    junction's edge normal differs from its individual segment tangent, so a
    fixed tangent offset can point outside the real camera association gate.
    """
    station=max(0.,min(station_m,frame.total_length_m))
    points=frame.points_xz
    i=max(0,min(len(points)-2,bisect_left(frame.station,station)-1))
    t=(station-frame.station[i])/max(1e-9,frame.station[i+1]-frame.station[i])
    def edge(j):
        previous=points[max(0,j-1)];current=points[j];following=points[min(len(points)-1,j+1)]
        incoming=(current[0]-previous[0],current[1]-previous[1]) if j else (following[0]-current[0],following[1]-current[1])
        outgoing=(following[0]-current[0],following[1]-current[1]) if j<len(points)-1 else incoming
        def normal(vector):
            length=math.hypot(*vector)
            return (-vector[1]/length,vector[0]/length) if length>1e-9 else (1.,0.)
        a,b=normal(incoming),normal(outgoing)
        nx,nz=a[0]+b[0],a[1]+b[1]
        length=math.hypot(nx,nz)
        nx,nz=(nx/length,nz/length) if length>1e-9 else a
        scale=min(3.,1/max(.01,nx*a[0]+nz*a[1]))
        return current[0]+nx*half_width_m*scale,current[1]+nz*half_width_m*scale
    a,b=edge(i),edge(i+1)
    x,z=a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])
    cx=points[i][0]+t*(points[i+1][0]-points[i][0])
    cz=points[i][1]+t*(points[i+1][1]-points[i][1])
    nx,nz=x-cx,z-cz;length=math.hypot(nx,nz)
    return x+clearance_m*nx/length,z+clearance_m*nz/length
