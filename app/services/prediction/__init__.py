"""Multi-agent trajectory forecasting & intent (Phase 7).

The planner up to Phase 6 treats every obstacle as fixed-velocity, so it
brakes late on merges and cut-ins. This package adds a real prediction
stage between perception and planning:

* ``forecaster``  -- forward trajectory per tracked agent (CTRA + Frenet
  lane-following), 3.0 s horizon at 0.1 s steps.
* ``intent``      -- maneuver-intent probability distribution
  (LANE_KEEP / MERGE_LEFT / MERGE_RIGHT / DECELERATING / STOPPING) and a
  single ``p_cut_in``.
* ``risk_field``  -- Gaussian spatiotemporal risk field around the
  forecast, covariance growing with horizon time.

It consumes the ego's own sensor-resolved picture (``SurroundTrack`` /
``TrackEstimate``) -- never raw NPC ground truth -- and emits
``interfaces.AgentPrediction`` / ``PredictionOutput``.
"""
from app.services.prediction.forecaster import (
    AgentKinematics,
    forecast_agent,
    forecast_ctra,
    forecast_lane_following,
    kinematics_from_track,
    project_agent_frenet,
)
from app.services.prediction.intent import (
    CUT_IN_ACTION_THRESHOLD,
    Intent,
    IntentEstimate,
    estimate_intent,
    estimate_intent_from_track,
)
from app.services.prediction.risk_field import (
    DEFAULT_CONFIG as RISK_FIELD_DEFAULT_CONFIG,
    RiskField,
    RiskFieldConfig,
    build_risk_field,
)

__all__ = [
    "AgentKinematics",
    "forecast_agent",
    "forecast_ctra",
    "forecast_lane_following",
    "kinematics_from_track",
    "project_agent_frenet",
    "Intent",
    "IntentEstimate",
    "estimate_intent",
    "estimate_intent_from_track",
    "CUT_IN_ACTION_THRESHOLD",
    "RiskField",
    "RiskFieldConfig",
    "build_risk_field",
    "RISK_FIELD_DEFAULT_CONFIG",
]
