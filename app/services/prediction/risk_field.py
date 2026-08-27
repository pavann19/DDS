"""Spatiotemporal risk field (Phase 7).

Given the per-agent forecasts, this builds a scalar risk in ``[0, 1]`` over
``(x, z, t)``: near 1 right on a predicted agent position, decaying with a
Gaussian whose spread **grows with horizon time** -- a forecast 3 s out is
far less certain than one 0.3 s out, so its footprint is correspondingly
wider.

Each agent contributes an oriented Gaussian (wider along its direction of
travel than across it, since along-track position error dominates). Agents
combine as a probabilistic OR -- ``1 - prod(1 - risk_i)`` -- so the field
stays bounded in ``[0, 1]`` no matter how many agents overlap.

The planner queries this along its candidate trajectories
(``sample_along`` / ``max_risk``) to shed speed *before* a merge closes,
instead of reacting once TTC is already critical.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from app.services.interfaces import AgentPrediction


@dataclass(frozen=True)
class RiskFieldConfig:
    # Gaussian half-widths at t = 0 ...
    sigma0_lon_m: float = 2.0
    sigma0_lat_m: float = 1.0
    # ... and how fast they grow per second of forecast horizon.
    k_lon_m_per_s: float = 0.7
    k_lat_m_per_s: float = 0.3
    step_s: float = 0.1


DEFAULT_CONFIG = RiskFieldConfig()


class RiskField:
    def __init__(
        self,
        predictions: Sequence[AgentPrediction],
        config: RiskFieldConfig = DEFAULT_CONFIG,
    ) -> None:
        self.config = config
        # Keep only agents that actually carry states.
        self._preds: List[AgentPrediction] = [p for p in predictions if p.states]

    # -- core query -----------------------------------------------------
    def risk_at(self, x: float, z: float, t_s: float) -> float:
        if not self._preds:
            return 0.0
        survive = 1.0
        for pred in self._preds:
            survive *= 1.0 - self._agent_risk(pred, x, z, t_s)
            if survive <= 0.0:
                return 1.0
        return 1.0 - survive

    def sample_along(self, points: Iterable[Tuple[float, float, float]]) -> List[float]:
        """``points`` is an iterable of ``(x, z, t_s)``; returns the risk at
        each (e.g. one per ego trajectory point)."""
        return [self.risk_at(x, z, t) for x, z, t in points]

    def max_risk(self, points: Iterable[Tuple[float, float, float]]) -> float:
        vals = self.sample_along(points)
        return max(vals) if vals else 0.0

    # -- internals ----------------------------------------------------
    def _state_at(self, pred: AgentPrediction, t_s: float):
        """Nearest forecasted state to ``t_s`` (states are evenly spaced)."""
        states = pred.states
        if t_s <= states[0].t_s:
            return states[0]
        if t_s >= states[-1].t_s:
            return states[-1]
        step = self.config.step_s
        idx = int(round((t_s - states[0].t_s) / step))
        idx = max(0, min(len(states) - 1, idx))
        return states[idx]

    def _agent_risk(self, pred: AgentPrediction, x: float, z: float, t_s: float) -> float:
        st = self._state_at(pred, t_s)
        cfg = self.config
        t_eff = max(0.0, t_s)
        sig_lon = cfg.sigma0_lon_m + cfg.k_lon_m_per_s * t_eff
        sig_lat = cfg.sigma0_lat_m + cfg.k_lat_m_per_s * t_eff

        dx, dz = x - st.x, z - st.z
        speed = math.hypot(st.vx, st.vz)
        if speed > 1e-6:
            fwd_x, fwd_z = st.vx / speed, st.vz / speed
            right_x, right_z = fwd_z, -fwd_x
            lon = dx * fwd_x + dz * fwd_z
            lat = dx * right_x + dz * right_z
        else:
            lon, lat = dz, dx  # axis-aligned fallback for a stopped agent

        exponent = 0.5 * ((lon * lon) / (sig_lon * sig_lon) + (lat * lat) / (sig_lat * sig_lat))
        if not math.isfinite(exponent) or exponent > 60.0:
            return 0.0
        return math.exp(-exponent)


def build_risk_field(
    predictions: Sequence[AgentPrediction],
    config: RiskFieldConfig = DEFAULT_CONFIG,
) -> RiskField:
    return RiskField(predictions, config)
