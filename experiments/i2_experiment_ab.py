"""I2 Experiment A -- proper: Rep A vs Rep B on the *revised* (post-I1) sensor
observation substrate.

NO driving policy and NO neural network is trained. The only thing fitted is a
closed-form ridge **analysis probe** (per docs/I2_EXPERIMENT_A_FINDINGS.md §3:
"train each on two probe tasks ... record CPU inference latency"). Rep B's
DeepSets encoder uses a **fixed random** projection (an untrained random-feature
set encoder) -- it measures whether permutation-invariant pooling of the same
per-agent features helps a linear readout, without training a network.

Two representations of the K-frame history stack (K=10 @ 10 Hz = 1 s):

  Rep A -- flat hand-features. Fixed-role agent slots concatenated; no
           permutation handling (roles are fixed). Cheapest.
  Rep B -- fixed random ReLU per-agent projection -> sum+mean pool
           (permutation-invariant) -> concat ego history + road profile.

Two probe tasks, on a (scenario, seed)-held-out split, measurement noise ON:
  1. deterministic-planner next-lane choice (3-class: keep / left / right, 1 s
     horizon) -- accuracy + macro-F1.
  2. true next-3 s NPC displacement (ego-frame) -- ADE (mean over 5 horizons)
     and FDE (at 3 s), metres. Ground-truth future is a LABEL only, never a
     representation input (OBSERVATION_BOUNDARY.md).

Plus per-call feature-construction latency (median µs).

Run:  python -m experiments.i2_experiment_ab [--fast] [--noise]
Writes experiments/_out/experiment_ab_summary.json (gitignored).
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import random
import time
from collections import deque
from dataclasses import asdict
from typing import Dict, List, Tuple

import numpy as np

from app.services.physics_engine import PhysicsEngine, EGO_LANE_OFFSET_M
from app.services.scenario_engine import ScenarioEngine
from app.services.traffic import TrafficModel, NpcVehicle
from app.services.learning.observation import ROLE_SLOTS

_OUT = pathlib.Path(__file__).resolve().parent / "_out"
_OUT.mkdir(exist_ok=True)

SCENARIOS = [
    "normal_cruising", "traffic_overtake", "emergency_cut_in",
    "queue_stop_and_go", "erratic_cut_in", "highway_merge",
    "lane_straddler", "low_friction_curve",
]

K_HIST = 10                       # history frames (1 s @ 10 Hz)
FUT_HORIZON_TICKS = 30            # 3 s
FUT_CHECKPOINTS = (6, 12, 18, 24, 30)   # 0.6 .. 3.0 s
LANE_HORIZON_TICKS = 10          # 1 s -- "next" lane choice
LANE_CENTRES = (1.75, 5.25)
_PHI_DIM = 16
_AGENT_FEATS = 9                 # ds, dd, closing, d_rate, d_accel, p_cut_in, conf, age, pos_sigma


# --------------------------------------------------------------------------
# rollout: per tick -> (obs_dict, planner_target_d, {npc_id: (x_ego, z_ego)})
# --------------------------------------------------------------------------
def _npc_rel_frenet(phys: PhysicsEngine) -> Dict[str, Tuple[float, float]]:
    """Each NPC's (ds, dd) = (station - ego_station, lane_offset - ego_d) in
    the road frame. LABEL ONLY -- never enters a representation."""
    out: Dict[str, Tuple[float, float]] = {}
    if phys.traffic is None or phys.frenet_frame is None:
        return out
    es, ed = phys.current_station_m, phys.current_lateral_offset_m
    for n in phys.traffic.npcs:
        out[str(n.id)] = (n.station_m - es, n.lane_offset - ed)
    return out


def _rand_route(rng: random.Random):
    lat0, lng0 = 37.7749, -122.4194
    m_lat = 111_320.0
    m_lng = 111_320.0 * math.cos(math.radians(lat0))
    pts = [(lat0, lng0 + (i * 3.0) / m_lng) for i in range(40)]
    if rng.random() < 0.6:
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


_NOISE = None


def _run(kind: str, name: str, seed: int, ticks: int) -> List[dict]:
    phys = PhysicsEngine(seed=seed)
    if _NOISE is not None:
        phys.set_perception_noise(_NOISE)
    rows: List[dict] = []
    if kind == "scenario":
        phys.set_route([(37.7749 + i * 0.00005, -122.4194) for i in range(200)])
        eng = ScenarioEngine()
        eng.load_scenario(name, phys,
                          initial_speed_kmh=55.0 if name == "low_friction_curve" else 50.0)
        for _ in range(ticks):
            eng.update(phys, 0.1)
            phys.update("Maintain Speed", dt=0.1)
            obs = phys.build_learning_observation()
            if obs is not None:
                rows.append({
                    "obs": asdict(obs),
                    "target_d": float(phys.lateral_target_d_m),
                    "ego_d": float(phys.current_lateral_offset_m),
                    "npc": _npc_rel_frenet(phys),
                })
    else:
        rng = random.Random(seed)
        route = _rand_route(rng)
        phys.set_destination(*route[-1])
        phys.set_route(route)
        phys.reset_state(station_m=0.0, speed_kmh=rng.uniform(35.0, 55.0),
                         lateral_offset_m=EGO_LANE_OFFSET_M)
        tm = TrafficModel(total_length_m=phys.station_distances[-1], seed=seed,
                          density=rng.choice(("low", "medium", "high")))
        if rng.random() < 0.5:
            tm.spawn_scripted_npcs([NpcVehicle(
                id="lead", lane_offset=EGO_LANE_OFFSET_M,
                speed_kmh=rng.uniform(0.0, 35.0), station_m=rng.uniform(30.0, 90.0),
                desired_speed_kmh=rng.uniform(0.0, 35.0), prevent_recycle=True)])
        phys.traffic = tm
        for _ in range(ticks):
            phys.update("Maintain Speed", dt=0.1)
            obs = phys.build_learning_observation()
            if obs is not None:
                rows.append({
                    "obs": asdict(obs),
                    "target_d": float(phys.lateral_target_d_m),
                    "ego_d": float(phys.current_lateral_offset_m),
                    "npc": _npc_rel_frenet(phys),
                })
    return rows


# --------------------------------------------------------------------------
# labels
# --------------------------------------------------------------------------
def _lane_idx(d: float) -> int:
    return min(range(len(LANE_CENTRES)), key=lambda i: abs(LANE_CENTRES[i] - d))


def _lane_label(run: List[dict], t: int) -> int:
    """0 keep / 1 left / 2 right, from the planner target LANE_HORIZON_TICKS ahead."""
    j = min(t + LANE_HORIZON_TICKS, len(run) - 1)
    cur = _lane_idx(run[t]["ego_d"])
    fut = _lane_idx(run[j]["target_d"])
    if fut == cur:
        return 0
    # +d is LEFT in DDS Frenet -> higher lane index sits further left
    return 1 if fut > cur else 2


def _npc_future(run: List[dict], t: int):
    """Target = the (ds, dd) displacement of the SINGLE nearest-ahead NPC
    (0 < ds < 120 m) at each checkpoint, in the road frame at t. It must
    survive the whole 3 s horizon. Returns [10] = 5 checkpoints x (Δds, Δdd),
    or None. Ground-truth future -- LABEL ONLY."""
    here = run[t]["npc"]
    if not here:
        return None
    ahead = sorted(((k, v) for k, v in here.items() if 0.0 < v[0] < 120.0),
                   key=lambda kv: kv[1][0])
    if not ahead:
        return None
    key, (ds0, dd0) = ahead[0]
    disp = []
    for cp in FUT_CHECKPOINTS:
        j = t + cp
        if j >= len(run) or key not in run[j]["npc"]:
            return None
        ds1, dd1 = run[j]["npc"][key]
        d_ds, d_dd = ds1 - ds0, dd1 - dd0
        # reject NPC recycle / id-reuse teleports (a wrapped ambient NPC can
        # jump hundreds of metres) -- physically, 3 s of relative motion is
        # well under 60 m longitudinal / 8 m lateral here.
        if abs(d_ds) > 60.0 or abs(d_dd) > 8.0:
            return None
        disp += [d_ds, d_dd]
    return disp


# --------------------------------------------------------------------------
# representations
# --------------------------------------------------------------------------
_EGO_KEYS = ("speed_mps", "accel_mps2", "lane_offset_m", "lateral_rate_mps",
             "heading_err_rad", "road_curvature_1pm")


def _ego_hist_vec(hist: List[dict]) -> List[float]:
    v: List[float] = []
    for o in hist:
        v += [float(o[k]) for k in _EGO_KEYS]
    return v


def _road_vec(o: dict) -> List[float]:
    cp = list(o.get("curvature_profile_1pm") or [])
    cp = (cp + [0.0] * 5)[:5]
    return cp + [
        float(o.get("dist_to_next_maneuver_m", 0.0)) / 200.0,
        float(o["ego_lane_index"]),
        1.0 if o["left_lane_clear"] else 0.0,
        1.0 if o["right_lane_clear"] else 0.0,
    ]


def _agent_row(a: dict) -> List[float]:
    return [
        float(a["ds_m"]), float(a["dd_m"]), float(a["closing_mps"]),
        float(a["d_rate_mps"]), float(a["d_accel_mps2"]), float(a["p_cut_in"]),
        float(a["track_confidence"]), float(a["track_age_ticks"]),
        float(a["pos_sigma_m"]),
    ]


def rep_a(hist: List[dict]) -> List[float]:
    o = hist[-1]
    v = _ego_hist_vec(hist) + _road_vec(o)
    for a in o["agents"]:                       # fixed 8 slots, fixed order
        v.append(1.0 if a["valid"] else 0.0)
        v += _agent_row(a) if a["valid"] else [0.0] * _AGENT_FEATS
    return v


_PHI_W = None
_PHI_B = None


def _phi_init():
    global _PHI_W, _PHI_B
    rng = np.random.default_rng(20260910)
    _PHI_W = rng.standard_normal((_AGENT_FEATS, _PHI_DIM)) / math.sqrt(_AGENT_FEATS)
    _PHI_B = rng.standard_normal(_PHI_DIM) * 0.1


def rep_b(hist: List[dict]) -> List[float]:
    o = hist[-1]
    v = _ego_hist_vec(hist) + _road_vec(o)
    valid = [a for a in o["agents"] if a["valid"]]
    if valid:
        G = np.array([_agent_row(a) for a in valid])          # (n, 9)
        H = np.maximum(0.0, G @ _PHI_W + _PHI_B)              # (n, 16) random ReLU
        v += list(H.sum(axis=0)) + list(H.mean(axis=0))
    else:
        v += [0.0] * (2 * _PHI_DIM)
    v.append(float(len(valid)))
    return v


# --------------------------------------------------------------------------
# closed-form ridge probe
# --------------------------------------------------------------------------
def _ridge_fit(X, Y, lam=1.0):
    X = np.asarray(X, float)
    Y = np.asarray(Y, float)
    mu = X.mean(0)
    sd = X.std(0)
    sd[sd < 1e-9] = 1.0
    Xn = (X - mu) / sd
    Xb = np.hstack([Xn, np.ones((len(Xn), 1))])
    d = Xb.shape[1]
    A = Xb.T @ Xb + lam * np.eye(d)
    A[-1, -1] -= lam
    W = np.linalg.solve(A, Xb.T @ Y)
    return (mu, sd, W)


def _ridge_pred(model, X):
    mu, sd, W = model
    Xn = (np.asarray(X, float) - mu) / sd
    Xb = np.hstack([Xn, np.ones((len(Xn), 1))])
    return Xb @ W


def _lane_probe(Xtr, ytr, Xte, yte):
    Ytr = np.zeros((len(ytr), 3))
    Ytr[np.arange(len(ytr)), ytr] = 1.0
    m = _ridge_fit(Xtr, Ytr, lam=5.0)
    pred = _ridge_pred(m, Xte).argmax(1)
    acc = float((pred == np.asarray(yte)).mean())
    f1s = []
    for c in range(3):
        tp = int(((pred == c) & (np.asarray(yte) == c)).sum())
        fp = int(((pred == c) & (np.asarray(yte) != c)).sum())
        fn = int(((pred != c) & (np.asarray(yte) == c)).sum())
        prec = tp / (tp + fp) if tp + fp else 0.0
        rec = tp / (tp + fn) if tp + fn else 0.0
        f1s.append(2 * prec * rec / (prec + rec) if prec + rec else 0.0)
    return {"accuracy": round(acc, 4), "macro_f1": round(sum(f1s) / 3, 4),
            "n_test": len(yte),
            "base_rate": round(float(np.bincount(yte, minlength=3).max() / len(yte)), 4)}


def _ade_fde(pred, Yte):
    Yte = np.asarray(Yte, float)
    pred = np.asarray(pred, float)
    ade, fde = [], []
    for i in range(len(Yte)):
        errs = [math.hypot(pred[i, 2 * c] - Yte[i, 2 * c], pred[i, 2 * c + 1] - Yte[i, 2 * c + 1])
                for c in range(len(FUT_CHECKPOINTS))]
        ade.append(sum(errs) / len(errs))
        fde.append(errs[-1])
    return {"ADE_m": round(float(np.mean(ade)), 4), "FDE_m": round(float(np.mean(fde)), 4),
            "n_test": len(Yte)}


def _npc_probe(Xtr, Ytr, Xte, Yte):
    m = _ridge_fit(Xtr, Ytr, lam=2.0)
    return _ade_fde(_ridge_pred(m, Xte), Yte)


def _cv_baseline_pred(cv_rows):
    """Constant-velocity: Δds = closing*cp*dt (ego closes the gap), Δdd =
    d_rate*cp*dt, from the ego_lead slot. cv_rows: list of (closing, d_rate)."""
    out = []
    for closing, d_rate in cv_rows:
        row = []
        for cp in FUT_CHECKPOINTS:
            dt = cp * 0.1
            row += [-closing * dt, d_rate * dt]
        out.append(row)
    return out


# --------------------------------------------------------------------------
def _assemble(runs: List[List[dict]]):
    """-> per-representation feature matrices + labels, plus latency samples."""
    lat = {"rep_a": [], "rep_b": []}
    XA, XB, y_lane, Y_npc, has_npc, cv = [], [], [], [], [], []
    for run in runs:
        hist: deque = deque(maxlen=K_HIST)
        for t, row in enumerate(run):
            hist.append(row["obs"])
            if len(hist) < K_HIST:
                continue
            if t + LANE_HORIZON_TICKS >= len(run):
                continue
            hl = list(hist)
            t0 = time.perf_counter(); va = rep_a(hl); lat["rep_a"].append(time.perf_counter() - t0)
            t0 = time.perf_counter(); vb = rep_b(hl); lat["rep_b"].append(time.perf_counter() - t0)
            XA.append(va); XB.append(vb)
            y_lane.append(_lane_label(run, t))
            _el = hl[-1]["agents"][0]   # ego_lead slot
            cv.append((float(_el["closing_mps"]) if _el["valid"] else 0.0,
                       float(_el["d_rate_mps"]) if _el["valid"] else 0.0))
            fut = _npc_future(run, t)
            if fut is None:
                Y_npc.append([0.0] * (2 * len(FUT_CHECKPOINTS))); has_npc.append(False)
            else:
                Y_npc.append(fut); has_npc.append(True)
    return XA, XB, y_lane, Y_npc, has_npc, cv, lat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--noise", action="store_true")
    args = ap.parse_args()
    global _NOISE
    if args.noise:
        from app.services.perception.perception_engine import PerceptionNoiseConfig
        _NOISE = PerceptionNoiseConfig(
            position_sigma_m=0.25, position_sigma_per_100m=0.6,
            velocity_sigma_mps=0.5, dropout_prob=0.06,
            occlusion_enabled=True, latency_ticks=1,
        )
    _phi_init()
    ticks = 140 if args.fast else 260
    train_seeds = [1, 2] if not args.fast else [1]
    test_seed = 3
    n_rand = 12 if not args.fast else 3
    rand_split = (n_rand * 2) // 3

    train_runs, test_runs = [], []
    for name in SCENARIOS:
        for s in train_seeds:
            train_runs.append(_run("scenario", name, s, ticks))
        test_runs.append(_run("scenario", name, test_seed, ticks))
    for i in range(n_rand):
        r = _run("random", f"r{i}", 5000 + i, ticks)
        (train_runs if i < rand_split else test_runs).append(r)

    XA_tr, XB_tr, yl_tr, Yn_tr, hn_tr, cv_tr, _ = _assemble(train_runs)
    XA_te, XB_te, yl_te, Yn_te, hn_te, cv_te, lat = _assemble(test_runs)

    def _mask(rows, keep):
        return [r for r, k in zip(rows, keep) if k]

    out = {
        "config": {"noise": bool(args.noise), "K_hist": K_HIST,
                   "fut_checkpoints_s": [c / 10 for c in FUT_CHECKPOINTS],
                   "n_train": len(yl_tr), "n_test": len(yl_te),
                   "dim_rep_a": len(XA_tr[0]), "dim_rep_b": len(XB_tr[0])},
        "lane_choice_probe": {
            "rep_a": _lane_probe(XA_tr, yl_tr, XA_te, yl_te),
            "rep_b": _lane_probe(XB_tr, yl_tr, XB_te, yl_te),
        },
        "npc_next_3s_probe": {
            "constant_velocity_baseline": _ade_fde(
                _cv_baseline_pred(_mask(cv_te, hn_te)), _mask(Yn_te, hn_te)),
            "rep_a": _npc_probe(_mask(XA_tr, hn_tr), _mask(Yn_tr, hn_tr),
                                _mask(XA_te, hn_te), _mask(Yn_te, hn_te)),
            "rep_b": _npc_probe(_mask(XB_tr, hn_tr), _mask(Yn_tr, hn_tr),
                                _mask(XB_te, hn_te), _mask(Yn_te, hn_te)),
        },
        "feature_build_latency_us": {
            "rep_a_median": round(1e6 * float(np.median(lat["rep_a"])), 2),
            "rep_b_median": round(1e6 * float(np.median(lat["rep_b"])), 2),
            "rep_a_p95": round(1e6 * float(np.percentile(lat["rep_a"], 95)), 2),
            "rep_b_p95": round(1e6 * float(np.percentile(lat["rep_b"], 95)), 2),
        },
    }

    la, lb = out["lane_choice_probe"]["rep_a"], out["lane_choice_probe"]["rep_b"]
    na, nb = out["npc_next_3s_probe"]["rep_a"], out["npc_next_3s_probe"]["rep_b"]
    lane_margin = lb["macro_f1"] - la["macro_f1"]
    ade_gain = (na["ADE_m"] - nb["ADE_m"]) / max(na["ADE_m"], 1e-6)
    lat_ratio = out["feature_build_latency_us"]["rep_b_median"] / max(
        out["feature_build_latency_us"]["rep_a_median"], 1e-6)
    # Decision rule: Rep A (flat, fixed-role) is the default -- cheaper and no
    # permutation handling. Rep B is only recommended if it (a) does NOT
    # regress the lane probe (macro-F1 within -0.01), (b) wins at least one
    # probe materially (lane macro-F1 >= +0.03, or NPC ADE cut >= 10%), and
    # (c) stays within 3x feature-build latency.
    rep_b_ok = (lane_margin >= -0.01
                and (lane_margin >= 0.03 or ade_gain >= 0.10)
                and lat_ratio <= 3.0)
    out["verdict"] = {
        "lane_macro_f1_margin_b_minus_a": round(lane_margin, 4),
        "npc_ade_relative_gain_b_over_a": round(ade_gain, 4),
        "latency_ratio_b_over_a": round(lat_ratio, 3),
        "rule": ("default Rep A; pick Rep B only if it does not regress the "
                 "lane probe (>= -0.01 macro-F1), wins one probe materially "
                 "(lane +0.03 or NPC ADE -10%), and stays within 3x latency"),
        "recommended": "rep_b" if rep_b_ok else "rep_a",
        "note": "analysis-probe evidence only; no policy/network trained. "
                "Rep B here is an UNTRAINED random-feature set encoder -- a "
                "trained DeepSets could do better and must be re-evaluated at "
                "I3 with the actual learned head.",
    }

    (_OUT / "experiment_ab_summary.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))
    print(f"\nsummary -> {_OUT / 'experiment_ab_summary.json'}")


if __name__ == "__main__":
    main()
