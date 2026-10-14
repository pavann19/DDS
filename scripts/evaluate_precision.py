"""Reproducible real-road simulated mission precision and sampled body audit.

Raw body/polyline heading stays reported. Motion heading uses measured
center-bicycle beta against a +/-6 m chord (the turn reference span).
The 7 m corridor is the existing generated road model, not surveyed lanes.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch
from contextlib import ExitStack

for key in ("OMP_NUM_THREADS","OPENBLAS_NUM_THREADS","MKL_NUM_THREADS","NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(key,"1")
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from app.services.simulation_session import SimulationSession
from app.services.driver.turn_tracking import continuous_path_heading
from app.services.driver import driver as driver_module
from app.services.planner.lateral import ROAD_HALF_WIDTH_M
from app.services.frenet import latlng_to_local
from app.services.vehicle_kinematics import sideslip


def sampled_narrow_body_distance(frame,station,x,z,heading):
    # Existing safety body dimensions, four edges sampled at <=0.2 m.
    corners=np.array([[-.9,-2.4],[.9,-2.4],[.9,2.4],[-.9,2.4]])
    local=[]
    for a,b in zip(corners,np.roll(corners,-1,axis=0)):
        count=math.ceil(float(np.linalg.norm(b-a))/.2)
        local.extend(a+t*(b-a) for t in np.linspace(0.,1.,count+1))
    local=np.asarray(local)
    right=np.array([math.cos(heading),math.sin(heading)])
    forward=np.array([math.sin(heading),-math.cos(heading)])
    world=np.array([x,z])+local[:,0,None]*right+local[:,1,None]*forward
    start=max(0,int(np.searchsorted(frame._station_array,station))-5)
    end=min(len(frame.points_xz)-1,start+20)
    a=frame._points_array[start:end];u=frame._points_array[start+1:end+1]-a
    q=world[:,None,:]-a[None,:,:]
    t=np.clip(np.sum(q*u[None,:,:],axis=2)/np.maximum(np.sum(u*u,axis=1)[None,:],1e-12),0.,1.)
    squared=np.sum((q-t[:,:,None]*u[None,:,:])**2,axis=2)
    return float(np.sqrt(np.max(np.min(squared,axis=1))))


def evaluate(friction=1.,baseline=False):
    s=SimulationSession(seed=42,real_time=False)
    s.scenario_engine.load_scenario("mapped_parking",s.world.physics)
    p=s.world.physics;p.friction_mu=friction
    rows=[]
    with ExitStack() as context:
        if baseline:
            context.enter_context(patch.object(driver_module,"front_axle_lateral_bias",lambda *args:0.))
            context.enter_context(patch.object(driver_module,"front_axle_heading_advance",lambda *args:0.))
            context.enter_context(patch.object(driver_module,"continuous_path_heading",lambda frame,station,offset,**kwargs:continuous_path_heading(frame,station,offset,half_window_m=1.)))
            from app.services.driver.turn_reference import SteeringDemandConstraint
            context.enter_context(patch.object(driver_module,"steering_demand_constraint",lambda **kwargs:SteeringDemandConstraint(float("inf"),0.,0.)))
        for _ in range(3200):
            s.step_pipeline(10)
            nav=p.get_navigation_state();parking=p.observe_parking();junction=s.driver.junctions.status
            heading=math.radians(nav["world_heading_deg"])
            motion=heading+sideslip(p.steering_angle_rad)
            reference=math.radians(continuous_path_heading(p.frenet_frame,p.current_station_m,s.driver.lateral_target_d_m,half_window_m=6.))
            error=(motion-reference+math.pi)%(2*math.pi)-math.pi
            x,z=nav["world_pos"]["x"],nav["world_pos"]["z"]
            px,pz=latlng_to_local(p.lat,p.lng,p.frenet_frame.origin_lat,p.frenet_frame.origin_lng)
            body=sampled_narrow_body_distance(p.frenet_frame,p.current_station_m,x,z,heading) if not parking["active"] else None
            row=dict(time=s.executor.clock.sim_time_s,station=p.current_station_m,
                center_track_error_m=p.current_lateral_offset_m-s.driver.lateral_target_d_m,
                raw_body_heading_error_deg=nav["relative_heading_deg"],
                reference_motion_heading_error_deg=math.degrees(error),
                acceleration_mps2=p.acceleration_mps2,lateral_acceleration_mps2=p.lateral_accel_mps2,
                steer_rad=p.steering_angle_rad,speed_kmh=p.speed_kmh,
                nominal_combined_budget_index=(p.lateral_accel_mps2/(3.*friction))**2+(p.acceleration_mps2/((3. if p.acceleration_mps2>=0 else 4.5)*friction))**2,
                model_body_distance_m=body,pose_error_m=math.hypot(x-px,z-pz),
                parking_active=parking["active"],maneuvering=parking["maneuvering"],
                contained=parking["contained"],parked=parking["parked"],
                red_stop_distance_m=junction["distance_to_stop_m"] if junction and junction["phase"]=="RED" else None)
            if not all(math.isfinite(v) for v in row.values() if isinstance(v,(float,int))):raise ValueError("nonfinite actual mission")
            rows.append(row)
            if parking["parked"]:break
    road=[r for r in rows if not r["parking_active"]]
    active=[r for r in rows if r["parking_active"]];maneuver=[r for r in rows if r["maneuvering"]]
    red=[r["red_stop_distance_m"] for r in rows if r["red_stop_distance_m"] is not None]
    body=max(r["model_body_distance_m"] for r in road)
    summary=dict(friction_mu=friction,baseline_reconstructed=baseline,seed=42,samples=len(rows),
        parked=rows[-1]["parked"],max_center_track_error_m=max(abs(r["center_track_error_m"]) for r in road),
        max_raw_body_heading_error_deg=max(abs(r["raw_body_heading_error_deg"]) for r in road),
        max_reference_motion_heading_error_deg=max(abs(r["reference_motion_heading_error_deg"]) for r in road),
        max_road_lateral_acceleration_mps2=max(r["lateral_acceleration_mps2"] for r in road),
        max_nominal_combined_budget_index=max(r["nominal_combined_budget_index"] for r in rows),
        nominal_combined_budget_exceedance_samples=sum(r["nominal_combined_budget_index"]>1.+1e-8 for r in rows),
        combined_budget_scope="existing conservative nominal ellipse only; aggregate actual commands not guaranteed by new restoration contribution; no tire-force model",
        max_sampled_narrow_body_distance_m=body,model_road_half_width_m=ROAD_HALF_WIDTH_M,
        minimum_sampled_narrow_body_model_road_margin_m=ROAD_HALF_WIDTH_M-body,
        footprint_scope="sampled perimeter <=0.2m at0.1s; narrow 4.8x1.8m chassis diagnostic, excludes rendered wheels; not full-vehicle, continuous or surveyed corridor certificate",
        max_jerk_mps3=max(abs(b["acceleration_mps2"]-a["acceleration_mps2"])/(b["time"]-a["time"]) for a,b in zip(rows,rows[1:])),
        max_steer_rate_radps=max(abs(b["steer_rad"]-a["steer_rad"])/(b["time"]-a["time"]) for a,b in zip(rows,rows[1:])),
        max_steer_rad=max(abs(r["steer_rad"]) for r in rows),
        max_pose_error_m=max(r["pose_error_m"] for r in rows),
        minimum_red_stop_distance_m=min(red) if red else None,
        all_maneuver_samples_contained=all(r["contained"] for r in maneuver),
        max_active_speed_kmh=max(r["speed_kmh"] for r in active),
        max_maneuver_speed_kmh=max(r["speed_kmh"] for r in maneuver))
    return summary,rows


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--friction",type=float,default=1.);parser.add_argument("--baseline",action="store_true");parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if not math.isfinite(args.friction) or not .1<=args.friction<=1.:parser.error("friction must be finite in[.1,1]")
    summary,rows=evaluate(args.friction,args.baseline)
    summary["revision"]=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip()
    summary["worktree_dirty"]=bool(subprocess.check_output(["git","status","--porcelain"],text=True).strip())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(summary=summary,samples=rows),indent=2),encoding="utf-8")
    print(json.dumps(summary,indent=2))
