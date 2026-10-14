"""I1 -> I2 Experiment A: is the current sensor-only Observation representation
sufficient (temporal / multi-agent / uncertainty / road-context), and how
clean / covered / repeatable is it?

NO neural network is trained. This harness:

1. runs the full built-in scenario matrix + randomised traffic/routes at
   several seeds, calling PhysicsEngine.build_learning_observation() every
   tick and recording, alongside it, the GROUND-TRUTH scene it must NOT be
   able to see (traffic.npcs exact state);
2. quantifies, per the I2 questions:
     * privileged-information LEAKAGE  -- can a cheap linear probe recover
       exact ground-truth NPC state from the Observation better than a
       sensor could? (R^2; a high R^2 on quantities the sensor blurs =
       leak);
     * COVERAGE          -- fraction of scenario "situations" (cut-in,
       curve, low-friction, lane-change, queue, ...) that actually occur;
     * DISTRIBUTION      -- per-field range / mean / std / % constant;
     * MISSINGNESS       -- per-field None / masked / zero fraction;
     * REPEATABILITY     -- identical (scenario, seed) -> identical
       Observation stream?
     * TEMPORAL / MULTI-AGENT / UNCERTAINTY / ROAD-CONTEXT sufficiency --
       explicit probes (below).

Run:  python -m experiments.i2_experiment_a  [--fast]
Writes a JSON summary to experiments/_out/experiment_a_summary.json (local,
gitignored) and prints the report.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import random
import statistics
from dataclasses import asdict
from typing import Dict, List

from app.services.physics_engine import PhysicsEngine, EGO_LANE_OFFSET_M
from app.services.scenario_engine import ScenarioEngine
from app.services.traffic import TrafficModel, NpcVehicle

_OUT = pathlib.Path(__file__).resolve().parent / "_out"
_OUT.mkdir(exist_ok=True)

SCENARIOS = [
    "normal_cruising", "traffic_overtake", "emergency_cut_in",
    "queue_stop_and_go", "erratic_cut_in", "highway_merge",
    "lane_straddler", "low_friction_curve",
]


# --------------------------------------------------------------------------
# route generators for the randomised (non-scenario) runs
# --------------------------------------------------------------------------
def _rand_route(rng: random.Random):
    """A straight-ish or curved highway route, dense waypoints."""
    lat0, lng0 = 37.7749, -122.4194
    m_lat = 111_320.0
    m_lng = 111_320.0 * math.cos(math.radians(lat0))
    pts = [(lat0, lng0 + (i * 3.0) / m_lng) for i in range(40)]
    if rng.random() < 0.6:  # add a curve
        R = rng.uniform(70.0, 400.0)
        sign = rng.choice((-1.0, 1.0))
        x0 = 39 * 3.0
        ang, d_ang = 0.0, 3.0 / R
        while ang < rng.uniform(0.4, math.pi / 2):
            ang += d_ang
            pts.append((lat0 + sign * R * (1 - math.cos(ang)) / m_lat,
                        lng0 + (x0 + R * math.sin(ang)) / m_lng))
    lat_n, lng_n = pts[-1]
    for i in range(1, 60):
        pts.append((lat_n + (i * 3.0) / m_lat, lng_n))
    return pts


# --------------------------------------------------------------------------
# one run -> list of (observation_dict, ground_truth_dict) per tick
# --------------------------------------------------------------------------
def _ground_truth(phys: PhysicsEngine) -> dict:
    """The exact scene the learned pipeline must NOT see."""
    npcs = phys.traffic.npcs if phys.traffic is not None else []
    same_dir = [n for n in npcs if n.speed_kmh >= 0]
    ahead = sorted(
        (n for n in same_dir if n.station_m > phys.current_station_m),
        key=lambda n: n.station_m,
    )
    nearest = ahead[0] if ahead else None
    return {
        "n_npcs": len(npcs),
        "nearest_ahead_gap_m": (nearest.station_m - phys.current_station_m) if nearest else None,
        "nearest_ahead_speed_mps": (nearest.speed_kmh / 3.6) if nearest else None,
        "nearest_ahead_lane_offset": nearest.lane_offset if nearest else None,
        "any_npc_lat_speed": max((abs(getattr(n, "lateral_speed_mps", 0.0)) for n in npcs), default=0.0),
        "ego_true_d": phys.current_lateral_offset_m,
        "friction_mu": phys.friction_mu,
        "path_curvature_true": phys.path_curvature,
        "speed_limit_reason": phys.speed_limit_reason,
        "shield_approved": phys.shield_verdict.approved,
        "is_changing_lane": phys._lc_sm.state.name != "LANE_KEEP",
    }


_NOISE = None  # set in main() when --noise


def _run(kind: str, name: str, seed: int, ticks: int) -> List[dict]:
    phys = PhysicsEngine(seed=seed)
    if _NOISE is not None:
        phys.set_perception_noise(_NOISE)
    rows: List[dict] = []
    if kind == "scenario":
        phys.set_route([(37.7749 + i * 0.00005, -122.4194) for i in range(200)])
        ScenarioEngine().load_scenario(name, phys, initial_speed_kmh=50.0) \
            if name != "low_friction_curve" else \
            ScenarioEngine().load_scenario(name, phys, initial_speed_kmh=55.0)
        eng = ScenarioEngine()
        eng.load_scenario(name, phys, initial_speed_kmh=50.0 if name != "low_friction_curve" else 55.0)
        for _ in range(ticks):
            eng.update(phys, 0.1)
            phys.update("Maintain Speed", dt=0.1)
            obs = phys.build_learning_observation()
            if obs is not None:
                rows.append({"obs": asdict(obs), "gt": _ground_truth(phys)})
    else:  # randomised
        rng = random.Random(seed)
        route = _rand_route(rng)
        phys.set_destination(*route[-1])
        phys.set_route(route)
        phys.reset_state(station_m=0.0, speed_kmh=rng.uniform(35.0, 55.0),
                         lateral_offset_m=EGO_LANE_OFFSET_M)
        tm = TrafficModel(total_length_m=phys.station_distances[-1], seed=seed,
                          density=rng.choice(("low", "medium", "high")))
        # a scripted slow lead sometimes
        if rng.random() < 0.5:
            tm.spawn_scripted_npcs([NpcVehicle(
                id="lead", lane_offset=EGO_LANE_OFFSET_M,
                speed_kmh=rng.uniform(0.0, 35.0), station_m=rng.uniform(30.0, 90.0),
                desired_speed_kmh=rng.uniform(0.0, 35.0), prevent_recycle=True,
            )])
        phys.traffic = tm
        for _ in range(ticks):
            phys.update("Maintain Speed", dt=0.1)
            obs = phys.build_learning_observation()
            if obs is not None:
                rows.append({"obs": asdict(obs), "gt": _ground_truth(phys)})
    return rows


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------
def _flatten_obs(o: dict) -> Dict[str, float]:
    flat: Dict[str, float] = {}
    for k, v in o.items():
        if k == "agents":
            valid = [a for a in v if a["valid"]]
            flat["agents_valid_count"] = float(len(valid))
            flat["agents_slot_saturation"] = 1.0 if len(valid) >= len(v) else 0.0
            if valid:
                flat["agent_min_ds_m"] = min(a["ds_m"] for a in valid)
                flat["agent_max_pcut"] = max(a["p_cut_in"] for a in valid)
                flat["agent_mean_conf"] = statistics.mean(a["track_confidence"] for a in valid)
                flat["agent_lateral_rate_nonzero_frac"] = statistics.mean(
                    1.0 if abs(a["lateral_rate_mps"]) > 1e-9 else 0.0 for a in valid)
            continue
        if v is None:
            flat[k + "__is_none"] = 1.0
            continue
        if isinstance(v, bool):
            flat[k] = 1.0 if v else 0.0
        elif isinstance(v, (int, float)):
            flat[k] = float(v)
        elif isinstance(v, str):
            flat[k + "::" + v] = 1.0
    return flat


def _linear_probe_r2(X: List[List[float]], y: List[float]) -> float:
    """Cheap least-squares R^2 with a bias term. No sklearn."""
    n = len(y)
    if n < 8 or len(set(round(v, 6) for v in y)) < 3:
        return float("nan")
    d = len(X[0])
    # normal equations (d small)
    XtX = [[0.0] * (d + 1) for _ in range(d + 1)]
    Xty = [0.0] * (d + 1)
    for xi, yi in zip(X, y):
        row = xi + [1.0]
        for a in range(d + 1):
            Xty[a] += row[a] * yi
            for b in range(d + 1):
                XtX[a][b] += row[a] * row[b]
    # ridge for stability
    for a in range(d + 1):
        XtX[a][a] += 1e-6
    beta = _solve(XtX, Xty)
    if beta is None:
        return float("nan")
    ybar = sum(y) / n
    ss_tot = sum((yi - ybar) ** 2 for yi in y) or 1e-12
    ss_res = 0.0
    for xi, yi in zip(X, y):
        pred = sum(b * v for b, v in zip(beta, xi + [1.0]))
        ss_res += (yi - pred) ** 2
    return max(0.0, 1.0 - ss_res / ss_tot)


def _solve(A, b):
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            return None
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        M[col] = [x / pv for x in M[col]]
        for r in range(n):
            if r != col and abs(M[r][col]) > 1e-15:
                f = M[r][col]
                M[r] = [x - f * y for x, y in zip(M[r], M[col])]
    return [M[i][n] for i in range(n)]


def analyse(all_rows: List[dict], repeat_pairs: List) -> dict:
    flats = [_flatten_obs(r["obs"]) for r in all_rows]
    keys = sorted({k for f in flats for k in f})
    N = len(flats)

    # DISTRIBUTION + MISSINGNESS
    dist = {}
    for k in keys:
        vals = [f[k] for f in flats if k in f]
        present = len(vals) / N
        col = {
            "present_frac": round(present, 4),
            "min": round(min(vals), 4) if vals else None,
            "max": round(max(vals), 4) if vals else None,
            "mean": round(statistics.mean(vals), 4) if vals else None,
            "std": round(statistics.pstdev(vals), 4) if len(vals) > 1 else 0.0,
        }
        col["constant"] = bool(vals and col["std"] == 0.0 and present > 0.99)
        dist[k] = col

    # explicit missingness for the raw Observation fields
    miss = {}
    for f in ["lead_gap_m", "lead_closing_mps"]:
        miss[f] = round(statistics.mean(1.0 if (f + "__is_none") in fl else 0.0 for fl in flats), 4)
    miss["agent_slots_all_full_frac"] = round(
        statistics.mean(fl.get("agents_slot_saturation", 0.0) for fl in flats), 4)
    miss["agent_slots_empty_frac"] = round(
        statistics.mean(1.0 if fl.get("agents_valid_count", 0.0) == 0.0 else 0.0 for fl in flats), 4)
    miss["agent_lateral_rate_always_zero"] = round(
        1.0 - statistics.mean(fl.get("agent_lateral_rate_nonzero_frac", 0.0) for fl in flats), 4)

    # COVERAGE -- situations actually seen
    def frac(pred):
        return round(statistics.mean(1.0 if pred(r) else 0.0 for r in all_rows), 4)
    coverage = {
        "curve_present(kappa>0.002)": frac(lambda r: abs(r["obs"]["road_curvature_1pm"]) > 0.002),
        "low_friction(<0.9)": frac(lambda r: r["obs"]["friction_estimate"] < 0.9),
        "has_lead(gap not None)": frac(lambda r: r["obs"]["lead_gap_m"] is not None),
        "cut_in_signalled(any agent p_cut>0.5)": frac(
            lambda r: any(a["valid"] and a["p_cut_in"] > 0.5 for a in r["obs"]["agents"])),
        "lane_change(gt)": frac(lambda r: r["gt"]["is_changing_lane"]),
        "shield_active(gt)": frac(lambda r: not r["gt"]["shield_approved"]),
        "queue/near_stop(speed<3)": frac(lambda r: r["obs"]["speed_mps"] < 3.0),
        ">=1 agent": frac(lambda r: any(a["valid"] for a in r["obs"]["agents"])),
        ">=4 agents": frac(lambda r: sum(a["valid"] for a in r["obs"]["agents"]) >= 4),
        "8/8 agents (saturated)": frac(lambda r: sum(a["valid"] for a in r["obs"]["agents"]) >= 8),
    }

    # LEAKAGE -- linear probe from the Observation numeric vector to
    # ground-truth quantities the SENSOR should blur/limit.
    num_keys = [k for k in keys if "::" not in k and "__is_none" not in k
                and not dist[k]["constant"]]
    X = [[fl.get(k, 0.0) for k in num_keys] for fl in flats]
    leak = {}
    for gt_key in ["nearest_ahead_gap_m", "nearest_ahead_speed_mps",
                   "nearest_ahead_lane_offset", "n_npcs", "ego_true_d",
                   "friction_mu", "path_curvature_true", "any_npc_lat_speed"]:
        pairs = [(x, r["gt"][gt_key]) for x, r in zip(X, all_rows) if r["gt"][gt_key] is not None]
        if len(pairs) < 30:
            leak[gt_key] = None
            continue
        Xk = [p[0] for p in pairs]
        yk = [float(p[1]) for p in pairs]
        leak[gt_key] = round(_linear_probe_r2(Xk, yk), 4)

    # REPEATABILITY
    rep = {"identical_pairs": 0, "total_pairs": len(repeat_pairs), "first_divergence": []}
    for a, b in repeat_pairs:
        same = (a == b)
        rep["identical_pairs"] += 1 if same else 0
        if not same:
            idx = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), None)
            rep["first_divergence"].append(idx)

    return {
        "n_ticks": N,
        "distribution": dist,
        "missingness": miss,
        "coverage": coverage,
        "leakage_probe_r2": leak,
        "repeatability": rep,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--noise", action="store_true",
                    help="enable a realistic PerceptionNoiseConfig on every run")
    args = ap.parse_args()
    global _NOISE
    if args.noise:
        from app.services.perception.perception_engine import PerceptionNoiseConfig
        _NOISE = PerceptionNoiseConfig(
            position_sigma_m=0.25, position_sigma_per_100m=0.6,
            velocity_sigma_mps=0.5, dropout_prob=0.06,
            occlusion_enabled=True, latency_ticks=1,
        )
    seeds = [1, 2, 3] if not args.fast else [1]
    rand_runs = 12 if not args.fast else 3
    ticks = 260 if not args.fast else 140

    all_rows: List[dict] = []
    repeat_pairs = []

    for name in SCENARIOS:
        for seed in seeds:
            rows = _run("scenario", name, seed, ticks)
            all_rows += rows
        # repeatability: same scenario+seed twice
        a = _run("scenario", name, seeds[0], ticks)
        b = _run("scenario", name, seeds[0], ticks)
        repeat_pairs.append(([r["obs"] for r in a], [r["obs"] for r in b]))

    for i in range(rand_runs):
        all_rows += _run("random", f"rand{i}", 1000 + i, ticks)

    summary = analyse(all_rows, repeat_pairs)
    (_OUT / "experiment_a_summary.json").write_text(json.dumps(summary, indent=2))

    print("=" * 72)
    print(f"I2 EXPERIMENT A  --  {summary['n_ticks']} ticks logged")
    print("=" * 72)
    print("\n[REPEATABILITY]")
    r = summary["repeatability"]
    print(f"  identical (scenario,seed) observation streams: "
          f"{r['identical_pairs']}/{r['total_pairs']}")
    if r["first_divergence"]:
        print(f"  first-divergence tick indices: {r['first_divergence']}")

    print("\n[COVERAGE]  (fraction of logged ticks)")
    for k, v in summary["coverage"].items():
        print(f"  {k:38s} {v:.3f}")

    print("\n[MISSINGNESS]")
    for k, v in summary["missingness"].items():
        print(f"  {k:38s} {v:.3f}")

    print("\n[LEAKAGE]  linear-probe R^2  Observation -> ground truth")
    print("  (high on a quantity the sensor should NOT reveal cleanly = leak)")
    for k, v in summary["leakage_probe_r2"].items():
        tag = "" if v is None else ("  <-- HIGH" if v > 0.9 else "")
        print(f"  {k:30s} {('n/a' if v is None else f'{v:.3f}'):>7}{tag}")

    print("\n[DISTRIBUTION]  constant / degenerate fields")
    consts = [k for k, c in summary["distribution"].items() if c["constant"]]
    for k in consts:
        print(f"  CONSTANT: {k} = {summary['distribution'][k]['mean']}")
    print(f"  ({len(consts)} constant of {len(summary['distribution'])} flattened features)")

    print(f"\nfull summary -> {(_OUT / 'experiment_a_summary.json')}")


if __name__ == "__main__":
    main()
