"""Prediction stage orchestrator (Phase 7).

Per tick: take the ego's confirmed surround tracks, forecast each agent,
estimate its intent, build the risk field, and derive a single proactive
control hint -- "a cut-in is developing, start easing off now" -- so the ego
sheds speed *before* TTC becomes critical (Gate 7.2) instead of slamming
the Safety Shield later.

Keeps a small per-track history so ``forecaster.kinematics_from_track`` can
finite-difference a yaw rate / longitudinal accel, and EMA-smooths the
noisy lateral-drift and accel estimates before they reach intent scoring
(the "yaw history" the roadmap calls for).
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from app.services.frenet import FrenetFrame, signed_curvature_at
from app.services.interfaces import AgentPrediction, PredictionOutput, SimClock
from app.services.prediction.forecaster import (
    AgentKinematics,
    forecast_agent,
    kinematics_from_track,
    project_agent_frenet,
)
from app.services.prediction.intent import (
    CUT_IN_ACTION_THRESHOLD,
    IntentEstimate,
    estimate_intent,
)
from app.services.prediction.risk_field import RiskField, RiskFieldConfig, build_risk_field

logger = logging.getLogger(__name__)


def _finite_track(t) -> bool:
    """Reject a track whose position/velocity isn't finite -- a NaN from a
    misbehaving tracker frame must not poison the forecast or the risk
    field."""
    try:
        return all(
            math.isfinite(float(v))
            for v in (t.x, t.z, t.vx, t.vz)
        )
    except (TypeError, ValueError, AttributeError):
        return False


# EMA factor for the per-track drift/accel estimates (0 = frozen, 1 = raw).
SMOOTHING_ALPHA = 0.4

# Proactive response (Gate 7.2): a gentle, comfort-bounded deceleration --
# strictly below the 1.5 m/s^2 the gate allows.
PROACTIVE_DECEL_MPS2 = 1.2
# Only respond while the projected lane crossing is still at least
# MIN_LEAD_S away (so the response is genuinely proactive) and no more than
# MAX_LEAD_S away (ignore a barely-drifting car many seconds out).
MIN_LEAD_S = 1.2
MAX_LEAD_S = 5.0
# An agent travelling clearly AGAINST the route direction is oncoming
# traffic in a physically separated opposing lane -- it is not a
# lane-merge / cut-in threat model (a genuine head-on crossing is the
# Safety Shield's / RSS's concern, not proactive lane-merge easing).
# Excluding it removes a sustained false-positive cut-in the intent
# estimator produced from small lateral components of a fast oncoming
# velocity vector.
OPPOSING_TRAFFIC_VS_MPS = -2.0

# Cut-in intent is scored from the agent's LANE-RELATIVE lateral rate
# d(d)/dt, position-differenced from its own tracked Frenet offset over a
# short window -- NOT from projecting the EKF velocity vector onto the
# lane normal. On a curve the EKF velocity estimate lags the true heading
# and that projection shows a large phantom inward drift (measured up to
# ~2 m/s on an R=80 m bend) for an agent that is simply following its
# lane; its lateral OFFSET, by contrast, is ~constant. The velocity
# projection (v_s) is still used for the opposing-traffic gate.
LATERAL_RATE_WINDOW_S = 0.5


# Windowed Frenet re-projection: a tracked agent moves at most a few metres
# per tick, so search only +/- this many route segments around last tick's
# match instead of the whole polyline.
FRENET_SEARCH_WINDOW = 25


@dataclass
class _TrackHistory:
    kin: AgentKinematics
    drift_ema: float = 0.0        # EMA of the position-differenced lane-relative lateral rate
    accel_ema: float = 0.0
    seg_idx: int = 0
    v_s_ema: float = 0.0
    d_hist: tuple = ()           # recent (agent_d) samples, newest last


@dataclass
class PredictionResult:
    """What the prediction stage hands the rest of the tick."""

    output: PredictionOutput
    risk_field: RiskField
    intents: Dict[int, IntentEstimate] = field(default_factory=dict)
    # Highest cut-in probability among agents that will actually intrude the
    # ego lane within the horizon (0 if none).
    cut_in_probability: float = 0.0
    cut_in_track_id: Optional[int] = None
    time_to_cross_s: Optional[float] = None
    # Comfort-bounded deceleration the longitudinal controller should
    # compose in (0.0 = no proactive action this tick).
    proactive_decel_mps2: float = 0.0
    # I2 observation substrate: per-track de-biased lane-relative lateral
    # rate / longitudinal accel (the EMA-smoothed, Frenet-geometry-corrected
    # quantities the intent heuristic already computes internally). Exposed
    # so AgentSlot.d_rate_mps / d_accel_mps2 can be populated from a
    # noise-robust source instead of the fragile pre-integrated p_cut_in.
    lateral_rate_by_track: Dict[int, float] = field(default_factory=dict)
    long_accel_by_track: Dict[int, float] = field(default_factory=dict)


class PredictionEngine:
    def __init__(self, risk_config: RiskFieldConfig = RiskFieldConfig()) -> None:
        self._risk_config = risk_config
        self._history: Dict[int, _TrackHistory] = {}

    def reset(self) -> None:
        self._history.clear()

    def step(
        self,
        *,
        clock: SimClock,
        surround_tracks: List,
        frenet_frame: Optional[FrenetFrame],
        ego_lateral_offset_m: float,
        dt: float,
    ) -> PredictionResult:
        confirmed = [
            t for t in surround_tracks
            if getattr(t, "status", "CONFIRMED") in ("CONFIRMED", "COASTED")
            and _finite_track(t)
        ]
        live_ids = {int(t.track_id) for t in confirmed}
        for stale in [tid for tid in self._history if tid not in live_ids]:
            del self._history[stale]

        predictions: List[AgentPrediction] = []
        intents: Dict[int, IntentEstimate] = {}

        best_p = 0.0
        best_id: Optional[int] = None
        best_ttc: Optional[float] = None

        for track in confirmed:
            try:
                self._process_track(
                    track, frenet_frame, ego_lateral_offset_m, dt, predictions, intents,
                )
            except Exception:  # pragma: no cover - defensive: one bad track must not kill the tick
                logger.exception("prediction: skipping track %r after error", getattr(track, "track_id", "?"))

        for est_tid, est in intents.items():
            hist = self._history.get(est_tid)
            is_opposing = hist is not None and hist.v_s_ema < OPPOSING_TRAFFIC_VS_MPS
            if (
                not is_opposing
                and est.p_cut_in > CUT_IN_ACTION_THRESHOLD
                and est.time_to_cross_s is not None
                and MIN_LEAD_S <= est.time_to_cross_s <= MAX_LEAD_S
                and est.p_cut_in > best_p
            ):
                best_p, best_id, best_ttc = est.p_cut_in, est_tid, est.time_to_cross_s

        risk_field = build_risk_field(predictions, self._risk_config)
        output = PredictionOutput(clock=clock, agents=tuple(predictions))

        proactive = PROACTIVE_DECEL_MPS2 if best_id is not None else 0.0
        return PredictionResult(
            output=output,
            risk_field=risk_field,
            intents=intents,
            cut_in_probability=best_p,
            cut_in_track_id=best_id,
            time_to_cross_s=best_ttc,
            proactive_decel_mps2=proactive,
            lateral_rate_by_track={tid: h.drift_ema for tid, h in self._history.items()},
            long_accel_by_track={tid: h.accel_ema for tid, h in self._history.items()},
        )

    def _process_track(self, track, frenet_frame, ego_lateral_offset_m, dt, predictions, intents):
        """Forecast + intent for one track. Appends to ``predictions`` /
        ``intents``. Raises on bad input -- the caller isolates each track so
        one failure does not kill the whole prediction stage."""
        tid = int(track.track_id)
        prev = self._history.get(tid)
        kin = kinematics_from_track(track, prev.kin if prev else None, dt=dt)

        # One windowed Frenet projection per agent per tick, reused for both
        # intent (lane-relative drift) and the lane-following model.
        frenet0 = None
        seg_idx = prev.seg_idx if prev else 0
        if frenet_frame is not None:
            s0, agent_d, v_s, raw_drift, seg_idx = project_agent_frenet(
                frenet_frame, kin.x, kin.z, kin.vx, kin.vz,
                hint_idx=seg_idx, window=(FRENET_SEARCH_WINDOW if prev else 0),
            )
            frenet0 = (s0, agent_d, v_s, raw_drift)
        else:
            agent_d, raw_drift = kin.x, kin.vx
        raw_accel = kin.a_long_mps2

        raw_v_s = v_s if frenet_frame is not None else kin.vz

        # Lane-relative lateral rate from the agent's OWN tracked offset,
        # position-differenced over ~LATERAL_RATE_WINDOW_S. Immune to the
        # EKF velocity-vector lag that fakes an inward drift on a curve.
        _n_win = max(1, int(round(LATERAL_RATE_WINDOW_S / max(dt, 1e-3))))
        _d_hist = ((prev.d_hist if prev else ()) + (agent_d,))[-(_n_win + 1):]
        if len(_d_hist) >= 2:
            _span_s = (len(_d_hist) - 1) * dt
            lane_rel_rate = (_d_hist[-1] - _d_hist[0]) / _span_s if _span_s > 1e-6 else 0.0
        else:
            lane_rel_rate = 0.0

        # Subtract the drift attributable purely to FRENET-FRAME GEOMETRY: an
        # agent advancing along a road of signed curvature kappa at constant
        # true lateral offset d, projected back to Frenet, has its projected
        # d change at ~ -v_s * kappa * d (the offset point sits at a different
        # radius than the centreline). Measured to match the observed phantom
        # drift within ~10% on R=80 m. What remains is genuine lane-relative
        # motion.
        if frenet_frame is not None and abs(v_s) > 0.5:
            _kappa_road = signed_curvature_at(frenet_frame, s0)
            lane_rel_rate -= v_s * _kappa_road * agent_d
        

        if prev is None:
            drift_ema, accel_ema, v_s_ema = lane_rel_rate, raw_accel, raw_v_s
        else:
            drift_ema = (1 - SMOOTHING_ALPHA) * prev.drift_ema + SMOOTHING_ALPHA * lane_rel_rate
            accel_ema = (1 - SMOOTHING_ALPHA) * prev.accel_ema + SMOOTHING_ALPHA * raw_accel
            v_s_ema = (1 - SMOOTHING_ALPHA) * prev.v_s_ema + SMOOTHING_ALPHA * raw_v_s

        self._history[tid] = _TrackHistory(
            kin=kin, drift_ema=drift_ema, accel_ema=accel_ema, seg_idx=seg_idx,
            v_s_ema=v_s_ema, d_hist=tuple(_d_hist),
        )

        est = estimate_intent(
            agent_d=agent_d,
            agent_v_d=drift_ema,
            agent_a_long_mps2=accel_ema,
            agent_speed_mps=kin.speed_mps,
            ego_d=ego_lateral_offset_m,
        )
        intents[tid] = est

        pred = forecast_agent(kin, frame=frenet_frame, frenet0=frenet0)
        predictions.append(AgentPrediction(
            track_id=pred.track_id,
            states=pred.states,
            intent=tuple(sorted(est.distribution.items(), key=lambda kv: -kv[1])),
        ))
