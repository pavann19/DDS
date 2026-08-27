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

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from app.services.frenet import FrenetFrame
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


# Windowed Frenet re-projection: a tracked agent moves at most a few metres
# per tick, so search only +/- this many route segments around last tick's
# match instead of the whole polyline.
FRENET_SEARCH_WINDOW = 25


@dataclass
class _TrackHistory:
    kin: AgentKinematics
    drift_ema: float = 0.0
    accel_ema: float = 0.0
    seg_idx: int = 0


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
            tid = int(track.track_id)
            prev = self._history.get(tid)
            kin = kinematics_from_track(track, prev.kin if prev else None, dt=dt)

            # One windowed Frenet projection per agent per tick, reused for
            # both intent (lane-relative drift) and the lane-following model.
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

            if prev is None:
                drift_ema, accel_ema = raw_drift, raw_accel
            else:
                drift_ema = (1 - SMOOTHING_ALPHA) * prev.drift_ema + SMOOTHING_ALPHA * raw_drift
                accel_ema = (1 - SMOOTHING_ALPHA) * prev.accel_ema + SMOOTHING_ALPHA * raw_accel

            self._history[tid] = _TrackHistory(
                kin=kin, drift_ema=drift_ema, accel_ema=accel_ema, seg_idx=seg_idx,
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
            pred = AgentPrediction(
                track_id=pred.track_id,
                states=pred.states,
                intent=tuple(sorted(est.distribution.items(), key=lambda kv: -kv[1])),
            )
            predictions.append(pred)

            if (
                est.p_cut_in > CUT_IN_ACTION_THRESHOLD
                and est.time_to_cross_s is not None
                and MIN_LEAD_S <= est.time_to_cross_s <= MAX_LEAD_S
                and est.p_cut_in > best_p
            ):
                best_p, best_id, best_ttc = est.p_cut_in, tid, est.time_to_cross_s

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
        )
