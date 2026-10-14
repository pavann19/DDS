"""Actual fixed-step session evaluation; simulated actors/sensors, recorded roads.

This supplies reproducible evidence for a separate live browser/WebSocket audit.
It neither fabricates telemetry nor certifies real-car or continuous safety.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from collections import Counter

for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(key, "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app.services.simulation_session import SimulationSession
from app.services.frenet import frenet_to_local_xz, latlng_to_local
from app.services.safety_shield import ROAD_BOUNDARY_HARD_LIMIT_M
from app.services.perception.entities import EntityClass, dimensions_for
from app.services.vehicle_kinematics import sideslip,wheel_geometry,TRACK_WIDTH_M,WHEEL_RADIUS_M
try:
    from app.services.vehicle_geometry import CHASSIS_HALF_LENGTH_M,CHASSIS_HALF_WIDTH_M,TIRE_WIDTH_M
    from app.services.vehicle_kinematics import TRACK_WIDTH_M,WHEEL_RADIUS_M
    ENVELOPE_VERSION="render-aligned chassis and four Ackermann tires"
except ImportError:
    CHASSIS_HALF_LENGTH_M,CHASSIS_HALF_WIDTH_M=2.25,.98
    TIRE_WIDTH_M=None
    ENVELOPE_VERSION="baseline 4.5x1.96m safety rectangle; no separate tire envelope"


def select_ego_envelope(mode):
    """Explicit comparison selection; auto follows available model constants."""
    global CHASSIS_HALF_LENGTH_M,CHASSIS_HALF_WIDTH_M,TIRE_WIDTH_M,ENVELOPE_VERSION
    if mode=="auto":return
    if mode=="baseline":
        CHASSIS_HALF_LENGTH_M,CHASSIS_HALF_WIDTH_M,TIRE_WIDTH_M=2.25,.98,None
        ENVELOPE_VERSION="explicit baseline 4.5x1.96m safety rectangle"
    elif mode=="render":
        CHASSIS_HALF_LENGTH_M,CHASSIS_HALF_WIDTH_M,TIRE_WIDTH_M=2.4,.99,.22
        ENVELOPE_VERSION="explicit render chassis4.8x1.98m plus four Ackermann tires"
    else:raise ValueError("Unsupported ego envelope selection")


def rectangle(x, z, heading, length, width):
    forward=(math.sin(heading), -math.cos(heading))
    right=(math.cos(heading), math.sin(heading))
    return [(x+sf*length/2*forward[0]+sr*width/2*right[0],
             z+sf*length/2*forward[1]+sr*width/2*right[1])
            for sf,sr in ((-1,-1),(-1,1),(1,1),(1,-1))]


def rectangles_overlap(a, b):
    """Independent vertex-projection SAT; strict overlap excludes mere contact."""
    for shape in (a,b):
        for p,q in zip(shape, shape[1:]+shape[:1]):
            axis=(q[1]-p[1], p[0]-q[0])
            pa=[x*axis[0]+z*axis[1] for x,z in a]
            pb=[x*axis[0]+z*axis[1] for x,z in b]
            if min(max(pa),max(pb))-max(min(pa),min(pb)) <= 1e-9:
                return False
    return True


def ego_rectangles(x,z,heading,steer,wheelbase):
    """Independent vertices from the version's explicit nominal geometry."""
    shapes=[rectangle(x,z,heading,CHASSIS_HALF_LENGTH_M*2,CHASSIS_HALF_WIDTH_M*2)]
    if TIRE_WIDTH_M is not None:
        angles,_=wheel_geometry(steer,wheelbase)
        for side in (-1.,1.):
            for front in (False,True):
                sx=side*TRACK_WIDTH_M/2
                forward=wheelbase/2*(1 if front else -1)
                wx=x+sx*math.cos(heading)+forward*math.sin(heading)
                wz=z+sx*math.sin(heading)-forward*math.cos(heading)
                index=(0 if side<0 else 1) if front else (2 if side<0 else 3)
                wheel_angle=angles[index]
                shapes.append(rectangle(wx,wz,heading+wheel_angle,2*WHEEL_RADIUS_M,TIRE_WIDTH_M))
    return shapes


def ground_truth_overlap_ids(p, x, z, heading):
    """Read-only offline oracle, never provided to the driving system."""
    ego=ego_rectangles(x,z,heading,p.steering_angle_rad,p.WHEELBASE_M)
    ids=[]
    for actor in (p.traffic.npcs if p.traffic else []):
        if abs(actor.station_m-p.current_station_m)>25.:continue
        nx,nz,tx,tz=frenet_to_local_xz(p.frenet_frame,actor.station_m,actor.lane_offset)
        length,width,_=dimensions_for(EntityClass(actor.entity_class))
        yaw=math.atan2(tx,-tz)+math.radians(actor.relative_heading_deg)
        actor_shape=rectangle(nx,nz,yaw,length,width)
        if any(rectangles_overlap(shape,actor_shape) for shape in ego):ids.append(actor.id)
    return ids


def load(session, scenario_id, params=None):
    """Use the same atomic command boundary as the WebSocket application."""
    session._handle_command(dict(type="load_scenario", scenario_id=scenario_id, params=params or {}))


def physical_sample(session):
    p = session.world.physics
    f = p.frenet_frame
    x, z = latlng_to_local(p.lat, p.lng, f.origin_lat, f.origin_lng)
    _, _, tx, tz = frenet_to_local_xz(f, p.current_station_m, 0.)
    road_heading = math.atan2(tx, -tz)
    heading = math.radians(p.heading)
    relative = heading-road_heading
    cx0,cz0,_,_=frenet_to_local_xz(f,p.current_station_m-2.5,session.driver.lateral_target_d_m)
    cx1,cz1,_,_=frenet_to_local_xz(f,p.current_station_m+2.5,session.driver.lateral_target_d_m)
    chord_heading=math.atan2(cx1-cx0,-(cz1-cz0))
    def degrees_error(angle):return math.degrees((angle+math.pi)%(2*math.pi)-math.pi)
    # Constant local route normal: modeled ribbon support, not surveyed lane
    # containment or a continuous footprint sweep across polyline vertices.
    right=(math.cos(road_heading),math.sin(road_heading))
    support=max(abs((vx-x)*right[0]+(vz-z)*right[1])
        for shape in ego_rectangles(x,z,heading,p.steering_angle_rad,p.WHEELBASE_M) for vx,vz in shape)
    parking = p.observe_parking() or {}
    junction = session.driver.junctions.status or {}
    command = session.latest_command
    return dict(time=session.executor.clock.sim_time_s, tick=session.executor.clock.tick,
        x=x, z=z, heading_deg=p.heading, speed_mps=p.speed_kmh/3.6,
        station_m=p.current_station_m, lateral_m=p.current_lateral_offset_m,
        target_lateral_m=session.driver.lateral_target_d_m,
        steering_rad=p.steering_angle_rad, acceleration_mps2=p.acceleration_mps2,
        raw_polyline_body_heading_error_deg=degrees_error(heading-road_heading),
        five_meter_chord_motion_heading_error_deg=degrees_error(heading+sideslip(p.steering_angle_rad)-chord_heading),
        lateral_acceleration_mps2=p.lateral_accel_mps2, friction_mu=p.friction_mu,
        modeled_road_margin_m=ROAD_BOUNDARY_HARD_LIMIT_M-abs(p.current_lateral_offset_m)-support,
        safety_override=p.shield_verdict.override_action,
        shield_overlap=p.shield_verdict.overlap_detected,
        shield_footprint_risk=p.shield_verdict.footprint_collision_risk,
        simulated_truth_overlap_ids=ground_truth_overlap_ids(p,x,z,heading),
        reason=session.driver.speed_limit_reason,
        actuator_source=command.source if command else None,
        lane_change_state=session.driver._lc_sm.state.value,
        junction_node_id=junction.get("node_id"),
        junction_kind=junction.get("kind"), junction_phase=junction.get("phase"),
        junction_distance_m=junction.get("distance_to_stop_m"),
        parking_active=bool(parking.get("active")), maneuvering=bool(parking.get("maneuvering")),
        parked=bool(parking.get("parked")), parking_contained=parking.get("contained"),
        analytics_decision=session.latest_analytics.get("decision"))


def summarize(rows, limits, require_parked=False):
    """Evaluate physical envelopes without exempting arbitrary planner reasons."""
    if not rows:raise ValueError("At least one actual physical sample is required")
    failures = []
    maximum_jerk = maximum_rate = 0.
    emergency_jerk = 0.
    for i, row in enumerate(rows):
        if any(not math.isfinite(v) for v in row.values() if isinstance(v, (int, float))):
            failures.append(dict(sample=i, invariant="finite_physical_state"))
        if abs(row["steering_rad"]) > limits["steering_rad"]+1e-9:
            failures.append(dict(sample=i, invariant="steering_angle"))
        if row["lateral_acceleration_mps2"] > limits["lateral_acceleration_mps2"]*row["friction_mu"]+1e-6:
            failures.append(dict(sample=i, invariant="friction_lateral_acceleration"))
        if not row["parking_active"] and row["modeled_road_margin_m"] < -1e-6:
            failures.append(dict(sample=i, invariant="modeled_full_body_road_bound"))
        if row["shield_overlap"]:
            failures.append(dict(sample=i, invariant="sensor_reported_overlap"))
        if row.get("simulated_truth_overlap_ids"):
            failures.append(dict(sample=i, invariant="simulated_truth_rectangle_overlap"))
        if row["junction_phase"] == "RED" and row["junction_distance_m"] < -1e-6:
            failures.append(dict(sample=i, invariant="active_red_stop_margin"))
        if row["maneuvering"] and not row["parking_contained"]:
            failures.append(dict(sample=i, invariant="mapped_parking_full_body_containment"))
        if i:
            old = rows[i-1]
            dt = row["time"]-old["time"]
            if dt <= 0:
                failures.append(dict(sample=i, invariant="monotonic_clock")); continue
            rate = abs(row["steering_rad"]-old["steering_rad"])/dt
            jerk = abs(row["acceleration_mps2"]-old["acceleration_mps2"])/dt
            maximum_rate = max(maximum_rate, rate)
            if rate > limits["steering_rate_radps"]+1e-8:
                failures.append(dict(sample=i, invariant="applied_steering_rate"))
            if row["safety_override"] == "EMERGENCY_BRAKE":
                emergency_jerk = max(emergency_jerk, jerk)
                if row["acceleration_mps2"] < -limits["braking_mps2"]-1e-8:
                    failures.append(dict(sample=i, invariant="bounded_emergency_braking"))
            else:
                maximum_jerk = max(maximum_jerk, jerk)
                if jerk > limits["jerk_mps3"]+1e-8:
                    failures.append(dict(sample=i, invariant="normal_applied_jerk"))
    if require_parked and not rows[-1]["parked"]:
        failures.append(dict(sample=len(rows)-1, invariant="mission_parked_before_timeout"))
    red = [r["junction_distance_m"] for r in rows if r["junction_phase"] == "RED"]
    road=[r for r in rows if not r["parking_active"]]
    return dict(samples=len(rows), simulated_seconds=rows[-1]["time"]-rows[0]["time"],
        passed=not failures, acceptance_scope="sampled physical envelopes only; behavioral coverage and liveness reported separately",
        ego_envelope_version=ENVELOPE_VERSION,
        collision_oracle_scope="50Hz independent nominal rectangle union SAT; NPC perception-class body dimensions; not swept/physical collision certification",
        failure_counts=dict(Counter(f["invariant"] for f in failures)),
        first_failures=failures[:30], parked=rows[-1]["parked"],
        max_normal_jerk_mps3=maximum_jerk, max_emergency_jerk_mps3=emergency_jerk,
        max_steering_rate_radps=maximum_rate,
        max_road_center_tracking_error_m=max((abs(r["lateral_m"]-r["target_lateral_m"]) for r in road),default=None),
        max_raw_polyline_body_heading_error_deg=max((abs(r["raw_polyline_body_heading_error_deg"]) for r in road),default=None),
        max_five_meter_chord_motion_heading_error_deg=max((abs(r["five_meter_chord_motion_heading_error_deg"]) for r in road),default=None),
        heading_metric_scope="body vs raw polyline tangent; velocity vs 5m offset-path chord; chord is model reference, not surveyed heading",
        minimum_red_stop_distance_m=min(red) if red else None,
        minimum_modeled_road_margin_m=min(r["modeled_road_margin_m"] for r in rows if not r["parking_active"]),
        safety_overrides=dict(Counter(r["safety_override"] for r in rows if r["safety_override"])),
        reasons=dict(Counter(r["reason"] for r in rows)),
        lane_states=dict(Counter(r["lane_change_state"] for r in rows)),
        junction_kinds=dict(Counter(r["junction_kind"] for r in rows if r["junction_kind"])))


def fixture_liveness(rows):
    """Declared seeded-fixture gate, separate from sampled physical safety."""
    longest = current = 0
    for row in rows:
        current = current + 1 if row["reason"] == "joint_trajectory_infeasible" else 0
        longest = max(longest, current)
    hold_s = longest * .02  # authoritative 50Hz evaluator cadence
    progress = rows[-1]["station_m"] - rows[0]["station_m"]
    return dict(behavior_scope="25s seeded erratic fixture: <=5s continuous plan hold, >20m station progress, final speed >1m/s; not general liveness",
        longest_plan_hold_s=hold_s, station_progress_m=progress,
        behavior_passed=bool(hold_s <= 5. and progress > 20. and rows[-1]["speed_mps"] > 1.))

def evaluate(scenario_id, seconds=30., params=None, seed=42, analytics_decision=None):
    if not math.isfinite(seconds) or seconds<=0:raise ValueError("Evaluation duration must be finite and positive")
    session = SimulationSession(seed=seed, real_time=False)
    load(session, scenario_id, params)
    if analytics_decision is not None:
        session.latest_analytics["decision"] = analytics_decision
    p = session.world.physics
    limits = dict(steering_rad=p.MAX_STEER_RAD, steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3, braking_mps2=p.A_MAX_BRAKE_MPS2,
        lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    rows = []
    for _ in range(math.ceil(seconds/.02)):
        session.step_pipeline(2)
        rows.append(physical_sample(session))
        if rows[-1]["parked"] and rows[-1]["speed_mps"] == 0.:
            break
    summary = summarize(rows, limits, require_parked=scenario_id == "mapped_parking")
    summary.update(scenario_id=scenario_id, params=params or {}, seed=seed,
        scenario_seed=session.scenario_engine.scenarios[scenario_id].seed, limits=limits,
        scope="recorded roads; simulated vehicle, actors, sensors and signals",
        collision_oracle="independent rectangle SAT over simulated actor truth; NPC perception class dimensions; discrete 50Hz, not continuous",
        ego_envelope_version=ENVELOPE_VERSION,
        road_bounds="modeled road half-width; not surveyed real lanes",
        browser_verified=False)
    summary["final_speed_mps"]=rows[-1]["speed_mps"]
    summary["scenario_events"]=session.scenario_engine.events_log
    summary["coverage"]={"observed_following":any(r["reason"]=="car_following" for r in rows),
        "observed_lane_change":any(r["lane_change_state"]=="EXECUTE_LANE_CHANGE" for r in rows),
        "observed_abort":any(r["lane_change_state"]=="ABORT_LANE_CHANGE" for r in rows),
        "observed_emergency":any(r["safety_override"]=="EMERGENCY_BRAKE" for r in rows),
        "observed_stop_sign":any(r["junction_kind"]=="stop" for r in rows),
        "observed_signal":any(r["junction_kind"]=="traffic_signals" for r in rows)}
    if scenario_id == "mapped_parking":
        summary["behavior_scope"] = "mapped parking fixture reaches exact standstill with modeled containment"
        summary["behavior_passed"] = bool(rows[-1]["parked"] and rows[-1]["speed_mps"] == 0. and rows[-1]["parking_contained"])
    if scenario_id == "erratic_cut_in":
        summary.update(fixture_liveness(rows))
    return summary, rows


def evaluate_abort(seconds=25.):
    """Controlled simulated intervention using an existing built-in NPC.

    This is explicitly an adversarial fixture, not a claimed real-road event.
    No sensor return or planner result is injected; sensors read the changed NPC.
    The bounded window includes physically sized background following gaps;
    the former centre-gap world returned at 10.3 s, corrected traffic at 20.9 s.
    """
    session=SimulationSession(seed=42,real_time=False)
    load(session,"traffic_overtake")
    p=session.world.physics
    rows=[];intervention=None
    for _ in range(math.ceil(seconds/.02)):
        if intervention is None and session.driver._lc_sm.state.value=="EXECUTE_LANE_CHANGE" and p.current_lateral_offset_m>2.2:
            actor=next(n for n in p.traffic.npcs if n.id!="npc-slow-lead")
            intervention=dict(actor_id=actor.id,time=session.executor.clock.sim_time_s,
                description="Existing simulated NPC relocated to block passing lane 12m ahead; synthetic adversarial fixture")
            actor.lane_offset=actor.base_lane_offset=session.driver._lc_sm.adjacent_lane_d_m
            actor.station_m=p.current_station_m+12.
            actor.speed_kmh=actor.desired_speed_kmh=0.
            actor.prevent_recycle=True
            actor.wander_amplitude_m=0.
        session.step_pipeline(2);rows.append(physical_sample(session))
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    summary=summarize(rows,limits)
    first_abort=next((i for i,r in enumerate(rows) if r["lane_change_state"]=="ABORT_LANE_CHANGE"),None)
    returned_row=next((r for r in rows[first_abort:] if r["lane_change_state"]=="LANE_KEEP" and abs(r["lateral_m"]-1.75)<=.3),None) if first_abort is not None else None
    returned=returned_row is not None
    summary.update(scenario_id="traffic_overtake_adversarial_abort",scope="Explicit synthetic intervention in recorded-road simulated session",
        intervention=intervention,observed_abort=any(r["lane_change_state"]=="ABORT_LANE_CHANGE" for r in rows),
        observed_abort_return=returned,abort_return_time_s=returned_row["time"] if returned else None,
        final_speed_mps=rows[-1]["speed_mps"],final_lateral_m=rows[-1]["lateral_m"],browser_verified=False)
    return summary,rows


def evaluate_recorded_intersection(kind="stop", conflict=False, seconds=40.):
    """Recorded OSM road/control, simulated movement and conflict actor.

    Stop line remains a modeled 6m offset. Direction-unknown tags are reported.
    Yield conflict is an explicitly synthetic actor fixture on real geometry.
    """
    if kind not in {"stop","give_way"}:raise ValueError("Unsupported recorded intersection kind")
    from app.services.traffic import TrafficModel,NpcVehicle
    data=json.loads((Path(__file__).resolve().parents[1]/"app/data/road_semantics.json").read_text(encoding="utf-8"))
    way_id,node_id=(130716952,65424265) if kind=="stop" else (370279592,6383816258)
    way=next(w for w in data["features"] if w["id"]==way_id)
    session=SimulationSession(seed=42,real_time=False)
    session.world.reset(route_waypoints=way["geometry"],start_speed_mps=5.,seed=42,lateral_offset_m=0.)
    p=session.world.physics;session.driver=p._driver
    # An available simulated producer reporting no actors is distinct from
    # removing the producer entirely (traffic=None on older adapter versions).
    p.traffic=TrafficModel(total_length_m=p.frenet_frame.total_length_m,seed=42,density="low")
    p.traffic.spawn_scripted_npcs([])
    control=next(c for c in p.route_controls if c.node_id==node_id)
    stop_station=control.station_m-session.driver.junctions.STOP_OFFSET_M
    session._reset_run_handoffs()
    session.driver.cruise_speed_kmh=18.
    conflict_removed=None
    if conflict:
        p.traffic=TrafficModel(total_length_m=p.frenet_frame.total_length_m,seed=42,density="low")
        p.traffic.spawn_scripted_npcs([NpcVehicle("fixture-yield-conflict",-3.5,0.,control.station_m,
            desired_speed_kmh=0.,prevent_recycle=True)])
    rows=[];first_stopped=None;release_time=None;cross_before_clear=False
    for _ in range(math.ceil(seconds/.02)):
        if conflict and conflict_removed is None and session.executor.clock.sim_time_s>=20.:
            p.traffic.npcs=[];conflict_removed=session.executor.clock.sim_time_s
        session.step_pipeline(2);row=physical_sample(session);rows.append(row)
        stopped=abs(stop_station-row["station_m"])<=1.5 and row["speed_mps"]<.15
        if stopped and first_stopped is None:first_stopped=row["time"]
        if not stopped and release_time is None:first_stopped=None
        if node_id in session.driver.junctions.released and release_time is None:release_time=row["time"]
        if conflict_removed is None and conflict and row["station_m"]>stop_station:cross_before_clear=True
    limits=dict(steering_rad=p.MAX_STEER_RAD,steering_rate_radps=p.STEER_RATE_MAX_RADPS,
        jerk_mps3=p.JERK_MAX_MPS3,braking_mps2=p.A_MAX_BRAKE_MPS2,lateral_acceleration_mps2=p.A_LAT_MAX_MPS2)
    summary=summarize(rows,limits)
    summary.update(scenario_id="recorded_stop" if kind=="stop" else "recorded_yield_synthetic_conflict",
        source_way_id=way_id,source_node_id=node_id,map_timestamp=control.map_timestamp,
        control_direction_verified=control.direction_verified,
        stop_line_source="modeled_6m_offset_from_recorded_OSM_node",
        road_name=way["tags"].get("name"),scope="Recorded OSM geometry/control; simulated vehicle/sensors",
        conflict_actor_source="Explicit stationary simulated NPC fixture" if conflict else None,
        conflict_removed_s=conflict_removed,crossed_before_conflict_clear=cross_before_clear,
        first_stopped_s=first_stopped,released_s=release_time,
        observed_stopped_duration_before_release_s=release_time-first_stopped if release_time is not None and first_stopped is not None else None,
        crossed_modeled_stop_station=any(r["station_m"]>stop_station+.5 for r in rows),
        browser_verified=False)
    dwell=summary["observed_stopped_duration_before_release_s"]
    summary["behavior_passed"]=(release_time is not None and summary["crossed_modeled_stop_station"] and
        (dwell is not None and dwell>=2.-.02 if kind=="stop" else
         first_stopped is not None and (not conflict or (not cross_before_clear and
         first_stopped<conflict_removed and release_time>=conflict_removed+1.-.02))))
    return summary,rows


def physical_digest(rows):
    keys=("time", "x", "z", "heading_deg", "speed_mps", "steering_rad", "acceleration_mps2", "reason")
    return hashlib.sha256(json.dumps([[r[k] for k in keys] for r in rows], separators=(",", ":"), allow_nan=False).encode()).hexdigest()


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenarios", nargs="*")
    parser.add_argument("--seconds", type=float, default=30.)
    parser.add_argument("--mapped-seconds", type=float, default=300.)
    parser.add_argument("--abort-fixture",action="store_true")
    parser.add_argument("--recorded-intersections",action="store_true")
    parser.add_argument("--ego-envelope",choices=("auto","baseline","render"),default="auto")
    options=parser.parse_args()
    select_ego_envelope(options.ego_envelope)
    available=SimulationSession(seed=42, real_time=False).scenario_engine.scenarios
    ids=options.scenarios or list(available)
    cases=[(s, {}, options.mapped_seconds if s=="mapped_parking" else options.seconds) for s in ids]
    if "mapped_parking" in ids:cases.append(("mapped_parking", {"friction_mu":.3}, options.mapped_seconds))
    results=[]
    for scenario, params, seconds in cases:
        summary, samples=evaluate(scenario, seconds, params)
        summary["physical_trace_sha256"]=physical_digest(samples)
        results.append(dict(summary=summary, samples=samples))
        print(json.dumps(summary), flush=True)
    if options.abort_fixture:
        summary,samples=evaluate_abort()
        summary["physical_trace_sha256"]=physical_digest(samples)
        results.append(dict(summary=summary,samples=samples))
        print(json.dumps(summary),flush=True)
    if options.recorded_intersections:
        for kind,conflict in (("stop",False),("give_way",True)):
            summary,samples=evaluate_recorded_intersection(kind,conflict)
            summary["physical_trace_sha256"]=physical_digest(samples)
            results.append(dict(summary=summary,samples=samples))
            print(json.dumps(summary),flush=True)
    report=dict(revision=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest()
            for folder in ("app","scripts") for path in Path(folder).rglob("*.py")},
        scope="offline actual session pipeline; live browser/WebSocket acceptance remains required", cases=results)
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    raise SystemExit(0 if all(r["summary"]["passed"] and r["summary"].get("behavior_passed",True) for r in results) else 1)
