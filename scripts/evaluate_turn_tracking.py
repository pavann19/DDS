"""Run the actual session pipeline and emit reproducible turning evidence.

Synthetic fixtures belong in tests; this evaluator uses the recorded road mission.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(key, "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app.services.simulation_session import SimulationSession
from app.services.frenet import latlng_to_local


def evaluate(seed=42, max_samples=3200, friction=None):
    session = SimulationSession(seed=seed, real_time=False)
    session.scenario_engine.load_scenario("mapped_parking", session.world.physics)
    p = session.world.physics
    if friction is not None:
        if not math.isfinite(friction) or not .1 <= friction <= 1.:
            raise ValueError("Friction must be finite and in [0.1, 1.0]")
        p.friction_mu = friction
    rows = []
    for _ in range(max_samples):
        session.step_pipeline(10)
        nav = p.get_navigation_state()
        parking = p.observe_parking()
        junction = session.driver.junctions.status
        ex, ez = latlng_to_local(p.lat, p.lng, p.frenet_frame.origin_lat, p.frenet_frame.origin_lng)
        row = dict(time=session.executor.clock.sim_time_s, station=p.current_station_m,
                   d=p.current_lateral_offset_m, target_d=session.driver.lateral_target_d_m,
                   speed_kmh=p.speed_kmh, steer_rad=p.steering_angle_rad,
                   heading_error_deg=nav["relative_heading_deg"],
                   acceleration_mps2=p.acceleration_mps2,
                   lateral_acceleration_mps2=p.lateral_accel_mps2,
                   pose_error_m=math.hypot(nav["world_pos"]["x"]-ex, nav["world_pos"]["z"]-ez),
                   parking_active=parking["active"], maneuvering=parking["maneuvering"],
                   contained=parking["contained"], parked=parking["parked"],
                   red_stop_distance_m=junction["distance_to_stop_m"] if junction and junction["phase"]=="RED" else None,
                   speed_limit_reason=session.driver.speed_limit_reason)
        if not all(math.isfinite(v) for v in row.values() if isinstance(v, (int, float))):
            raise ValueError("Non-finite live session state")
        rows.append(row)
        if parking["parked"]:
            break
    road = [r for r in rows if not r["parking_active"]]
    active = [r for r in rows if r["parking_active"]]
    maneuver = [r for r in rows if r["maneuvering"]]
    red = [r["red_stop_distance_m"] for r in rows if r["red_stop_distance_m"] is not None]
    summary = dict(seed=seed, friction_mu=p.friction_mu, samples=len(rows), parked=rows[-1]["parked"],
                   max_road_tracking_error_m=max(abs(r["d"]-r["target_d"]) for r in road),
                   max_road_heading_error_deg=max(abs(r["heading_error_deg"]) for r in road),
                   max_pose_error_m=max(r["pose_error_m"] for r in rows),
                   max_road_lateral_acceleration_mps2=max(r["lateral_acceleration_mps2"] for r in road),
                   max_steer_rad=max(abs(r["steer_rad"]) for r in rows),
                   max_steer_rate_radps=max(abs(b["steer_rad"]-a["steer_rad"])/(b["time"]-a["time"]) for a,b in zip(rows,rows[1:])),
                   max_active_speed_kmh=max((r["speed_kmh"] for r in active), default=None),
                   max_maneuver_speed_kmh=max((r["speed_kmh"] for r in maneuver), default=None),
                   all_maneuver_samples_contained=all(r["contained"] for r in maneuver),
                   minimum_red_stop_distance_m=min(red) if red else None)
    return summary, rows


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--output", type=Path, required=True)
    args.add_argument("--seed", type=int, default=42)
    args.add_argument("--friction", type=float)
    options = args.parse_args()
    summary, rows = evaluate(options.seed, friction=options.friction)
    summary["revision"] = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    summary["worktree_dirty"] = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], text=True).strip())
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(json.dumps(dict(summary=summary, samples=rows), indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
