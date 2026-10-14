"""Multi-rate deterministic executor (ADR-001, Phase 6.5, Action Item 4).

Owns a single :class:`~app.services.interfaces.SimClock` and dispatches
registered stages at their own rates off **fixed-step simulation time** --
never the wall clock. This is the structure the ADR's

    perception 20 Hz / prediction + behaviour + planner 10 Hz /
    controller + safety monitor 50 Hz

split plugs into. The full driver-stage separation is deferred to Phase 7
(hybrid-extraction decision), so today a caller registers one stage that
advances ``PhysicsEngine.update()`` at the control rate; the rate-dispatch
machinery lives here and is unit-tested so Phase 7 only has to register the
finer-grained stages.

Determinism contract: given the same registered stages and the same starting
clock, ``run_for`` / repeated ``step`` calls invoke those stages in exactly
the same order at exactly the same tick indices, every run, on any machine.
The executor reads no clock and draws no randomness of its own; a seeded
:class:`random.Random` is exposed as :attr:`rng` for stages that need one.
"""
from __future__ import annotations

import random
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional

from app.services.interfaces import SimClock

# Base substep rate. 100 Hz is chosen so every ADR rate divides it as an
# integer (100/50 = 2, 100/20 = 5, 100/10 = 10) -- a 50 Hz base cannot
# represent 20 Hz cleanly (100/20 vs 50/20 = 2.5).
BASE_HZ = 100.0

PERCEPTION_HZ = 20.0
PLANNING_HZ = 10.0
CONTROL_HZ = 50.0
SAFETY_HZ = 50.0

StageFn = Callable[[SimClock], None]


@dataclass(frozen=True)
class Stage:
    name: str
    hz: float
    fn: StageFn


class MultiRateExecutor:
    """Fixed-step, multi-rate stage scheduler.

    Parameters
    ----------
    base_hz:
        Substep rate. Every registered stage's ``hz`` must divide this.
    seed:
        Seeds :attr:`rng`; pass an int for a reproducible run.
    """

    def __init__(self, base_hz: float = BASE_HZ, seed: Optional[int] = None):
        if isinstance(base_hz, bool) or not isinstance(base_hz, (int, float)) or not math.isfinite(base_hz) or base_hz <= 0:
            raise ValueError('base rate must be finite and positive')
        self.clock = SimClock(dt_s=1.0 / base_hz)
        self.base_hz = base_hz
        self.rng = random.Random(seed)
        self._stages: List[Stage] = []
        self.metrics = {}

    # -- registration ------------------------------------------------------
    def add_stage(self, name: str, hz: float, fn: StageFn) -> None:
        if isinstance(hz, bool) or not isinstance(hz, (int, float)) or not math.isfinite(hz) or hz <= 0:
            raise ValueError(f"stage '{name}' rate must be positive, got {hz}")
        period = self.base_hz / hz
        if abs(period - round(period)) > 1e-9:
            raise ValueError(
                f"stage '{name}' rate {hz} Hz does not divide base {self.base_hz} Hz "
                f"(period {period})"
            )
        if any(stage.name == name for stage in self._stages):
            raise ValueError(f"duplicate stage '{name}'")
        self._stages.append(Stage(name=name, hz=hz, fn=fn))
        self.metrics[name] = {"count": 0, "deadline_misses": 0, "durations_ms": deque(maxlen=2000)}

    @property
    def stage_names(self) -> List[str]:
        return [s.name for s in self._stages]

    # -- execution -------------------------------------------------------
    def step(self) -> List[str]:
        """Advance exactly one base substep; run every stage due on the new
        tick, in registration order. Returns the names that ran (for tests
        / introspection)."""
        self.clock = self.clock.advance()
        ran: List[str] = []
        for stage in self._stages:
            if self.clock.is_rate_tick(stage.hz):
                started = time.perf_counter()
                stage.fn(self.clock)
                duration = (time.perf_counter() - started) * 1000
                metric = self.metrics[stage.name]
                metric["count"] += 1
                metric["deadline_misses"] += int(duration > 1000 / stage.hz)
                metric["durations_ms"].append(duration)
                ran.append(stage.name)
        return ran

    def record_external_timing(self, name, duration_ms, budget_ms):
        """Measure non-stage work without changing fixed-step dispatch."""
        metric = self.metrics.setdefault(name, {"count": 0, "deadline_misses": 0,
            "durations_ms": deque(maxlen=2000)})
        metric["count"] += 1
        metric["deadline_misses"] += int(duration_ms > budget_ms)
        metric["durations_ms"].append(duration_ms)

    def timing_summary(self):
        result = {}
        for name, metric in self.metrics.items():
            values = sorted(metric["durations_ms"])
            result[name] = {"count": metric["count"], "deadline_misses": metric["deadline_misses"],
                "sample_count": len(values), "window_capacity": 2000,
                "percentile_scope": "last_2000_stage_calls",
                "p50_ms": values[min(len(values)-1, int(len(values)*.50))] if values else 0.,
                "p95_ms": values[min(len(values)-1, int(len(values)*.95))] if values else 0.,
                "p99_ms": values[min(len(values)-1, int(len(values)*.99))] if values else 0.,
                "max_ms": values[-1] if values else 0.}
        return result

    def run_for(self, sim_seconds: float) -> int:
        """Run whole base substeps covering ``sim_seconds``. Returns the
        number of substeps executed."""
        if isinstance(sim_seconds, bool) or not isinstance(sim_seconds, (int, float)) or not math.isfinite(sim_seconds) or sim_seconds < 0:
            raise ValueError('simulation duration must be finite and nonnegative')
        n = round(sim_seconds * self.base_hz)
        for _ in range(n):
            self.step()
        return n
