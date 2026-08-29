"""Lane-change state machine for the Phase 8 joint planner.

The joint planner searches Frenet trajectories to a *lane centre*; this
state machine decides **which** lane centre(s) it is allowed to aim at on
a given tick, and tracks a lane change in progress so it can be aborted
cleanly:

    LANE_KEEP ──(current lane blocked & adjacent verified clear)──▶ PREPARE_LANE_CHANGE
    PREPARE_LANE_CHANGE ──(held COMMIT_TICKS)──▶ EXECUTE_LANE_CHANGE
    PREPARE_LANE_CHANGE ──(condition lost)──▶ LANE_KEEP
    EXECUTE_LANE_CHANGE ──(reached adjacent lane)──▶ LANE_KEEP  (origin lane := adjacent)
    EXECUTE_LANE_CHANGE ──(adjacent no longer clear & progress < ABORT_MAX)──▶ ABORT_LANE_CHANGE
    ABORT_LANE_CHANGE ──(back at origin lane)──▶ LANE_KEEP

The abort is not a special trajectory type: the planner simply plans a
fresh quintic from the ego's *current* ``(d, ḋ, d̈)`` back to the origin
lane centre, so it inherits the same ``|d''| ≤ 2.0`` / ``|d'''| ≤ 1.5``
feasibility envelope as any other maneuver (roadmap gate 8.1).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List


class LaneChangeState(str, Enum):
    LANE_KEEP = "LANE_KEEP"
    PREPARE_LANE_CHANGE = "PREPARE_LANE_CHANGE"
    EXECUTE_LANE_CHANGE = "EXECUTE_LANE_CHANGE"
    ABORT_LANE_CHANGE = "ABORT_LANE_CHANGE"


@dataclass(frozen=True)
class StateMachineConfig:
    commit_ticks: int = 5           # debounce in PREPARE before committing (0.5 s @ 10 Hz)
    complete_tol_m: float = 0.25    # within this of a lane centre = "arrived"
    abort_progress_max: float = 0.85  # past this fraction of the change, finish rather than abort


@dataclass
class LaneChangeStateMachine:
    origin_lane_d_m: float
    adjacent_lane_d_m: float
    cfg: StateMachineConfig = field(default_factory=StateMachineConfig)

    state: LaneChangeState = LaneChangeState.LANE_KEEP
    _prepare_count: int = 0
    progress: float = 0.0           # 0..1 fraction of the current EXECUTE/ABORT move

    def reset(self) -> None:
        self.state = LaneChangeState.LANE_KEEP
        self._prepare_count = 0
        self.progress = 0.0

    def _span(self) -> float:
        span = abs(self.adjacent_lane_d_m - self.origin_lane_d_m)
        return span if span > 1e-6 else 1.0

    def step(
        self,
        *,
        current_d_m: float,
        lane_blocked: bool,
        adjacent_clear: bool,
    ) -> List[float]:
        """Advance the machine one tick; return the lane centre(s) the
        planner may target this tick (always length 1 in the current
        design — the joint planner still fans out its own d-offset lattice
        around whichever centre is returned)."""
        want_change = lane_blocked and adjacent_clear

        if self.state is LaneChangeState.LANE_KEEP:
            if want_change:
                self.state = LaneChangeState.PREPARE_LANE_CHANGE
                self._prepare_count = 1
            return [self.origin_lane_d_m]

        if self.state is LaneChangeState.PREPARE_LANE_CHANGE:
            if not want_change:
                self.reset()
                return [self.origin_lane_d_m]
            self._prepare_count += 1
            if self._prepare_count >= self.cfg.commit_ticks:
                self.state = LaneChangeState.EXECUTE_LANE_CHANGE
                self.progress = 0.0
                return [self.adjacent_lane_d_m]
            return [self.origin_lane_d_m]

        if self.state is LaneChangeState.EXECUTE_LANE_CHANGE:
            self.progress = min(
                1.0, abs(current_d_m - self.origin_lane_d_m) / self._span()
            )
            if abs(current_d_m - self.adjacent_lane_d_m) <= self.cfg.complete_tol_m:
                # Arrived: the adjacent lane is now "the lane we're in".
                self.origin_lane_d_m, self.adjacent_lane_d_m = (
                    self.adjacent_lane_d_m,
                    self.origin_lane_d_m,
                )
                self.reset()
                return [self.origin_lane_d_m]
            if (not adjacent_clear) and self.progress < self.cfg.abort_progress_max:
                self.state = LaneChangeState.ABORT_LANE_CHANGE
                return [self.origin_lane_d_m]
            return [self.adjacent_lane_d_m]

        # ABORT_LANE_CHANGE
        self.progress = max(
            0.0, 1.0 - abs(current_d_m - self.origin_lane_d_m) / self._span()
        )
        if abs(current_d_m - self.origin_lane_d_m) <= self.cfg.complete_tol_m:
            self.reset()
        return [self.origin_lane_d_m]
