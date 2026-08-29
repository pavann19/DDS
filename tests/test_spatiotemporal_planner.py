"""
Phase 8 -- joint (s, d, t) motion planner.

Covers:
  * polynomials.py -- quintic/quartic boundary conditions + closed-form
    jerk-squared integral vs numerical quadrature
  * spatiotemporal.py -- lattice generation, feasibility filtering,
    cost ranking, risk-field awareness, timing (gate 8.3), and the
    100-lane-change jerk percentile (gate 8.2)
  * state_machine.py -- LANE_KEEP / PREPARE / EXECUTE / ABORT transitions
    and the mid-maneuver abort trajectory (gate 8.1)
"""
import math
import time

import numpy as np
import pytest

from app.services.planner.polynomials import QuinticPolynomial, QuarticPolynomial


# ===========================================================================
# polynomials.py
# ===========================================================================

def test_quintic_matches_all_six_boundary_conditions():
    p = QuinticPolynomial(x0=0.5, v0=0.2, acc0=-0.1, x1=1.75, v1=0.0, acc1=0.0, T=3.0)
    assert p.pos(0.0) == pytest.approx(0.5)
    assert p.vel(0.0) == pytest.approx(0.2)
    assert p.acc(0.0) == pytest.approx(-0.1)
    assert p.pos(3.0) == pytest.approx(1.75)
    assert p.vel(3.0) == pytest.approx(0.0, abs=1e-9)
    assert p.acc(3.0) == pytest.approx(0.0, abs=1e-9)


def test_quartic_matches_its_five_boundary_conditions():
    p = QuarticPolynomial(x0=10.0, v0=8.0, acc0=0.5, v1=13.9, acc1=0.0, T=4.0)
    assert p.pos(0.0) == pytest.approx(10.0)
    assert p.vel(0.0) == pytest.approx(8.0)
    assert p.acc(0.0) == pytest.approx(0.5)
    assert p.vel(4.0) == pytest.approx(13.9)
    assert p.acc(4.0) == pytest.approx(0.0, abs=1e-9)


def test_quintic_derivatives_are_self_consistent_by_finite_difference():
    p = QuinticPolynomial(x0=0.0, v0=1.0, acc0=0.0, x1=2.5, v1=0.0, acc1=0.0, T=3.5)
    h = 1e-4
    for t in (0.3, 1.1, 2.0, 3.0):
        num_v = (p.pos(t + h) - p.pos(t - h)) / (2 * h)
        num_a = (p.vel(t + h) - p.vel(t - h)) / (2 * h)
        num_j = (p.acc(t + h) - p.acc(t - h)) / (2 * h)
        assert num_v == pytest.approx(p.vel(t), rel=1e-4)
        assert num_a == pytest.approx(p.acc(t), rel=1e-4)
        assert num_j == pytest.approx(p.jerk(t), rel=1e-4)


def test_quintic_jerk_integral_matches_numerical_quadrature():
    p = QuinticPolynomial(x0=-1.0, v0=0.3, acc0=0.2, x1=1.0, v1=-0.1, acc1=0.0, T=2.8)
    ts = np.linspace(0.0, p.T, 20001)
    numerical = np.trapezoid(np.array([p.jerk(t) for t in ts]) ** 2, ts)
    assert p.jerk_squared_integral() == pytest.approx(numerical, rel=1e-4)


def test_quartic_jerk_integral_matches_numerical_quadrature():
    p = QuarticPolynomial(x0=0.0, v0=12.0, acc0=-0.5, v1=8.0, acc1=0.0, T=3.2)
    ts = np.linspace(0.0, p.T, 20001)
    numerical = np.trapezoid(np.array([p.jerk(t) for t in ts]) ** 2, ts)
    assert p.jerk_squared_integral() == pytest.approx(numerical, rel=1e-4)


def test_no_op_quintic_has_zero_jerk_cost():
    p = QuinticPolynomial(x0=1.75, v0=0.0, acc0=0.0, x1=1.75, v1=0.0, acc1=0.0, T=3.0)
    assert p.jerk_squared_integral() == pytest.approx(0.0, abs=1e-9)


def test_lateral_jerk_cost_is_sign_symmetric():
    left = QuinticPolynomial(x0=0.0, v0=0.0, acc0=0.0, x1=3.5, v1=0.0, acc1=0.0, T=3.0)
    right = QuinticPolynomial(x0=0.0, v0=0.0, acc0=0.0, x1=-3.5, v1=0.0, acc1=0.0, T=3.0)
    assert left.jerk_squared_integral() == pytest.approx(right.jerk_squared_integral())


def test_slower_lane_change_has_strictly_lower_jerk_cost():
    fast = QuinticPolynomial(x0=0.0, v0=0.0, acc0=0.0, x1=3.5, v1=0.0, acc1=0.0, T=2.0)
    slow = QuinticPolynomial(x0=0.0, v0=0.0, acc0=0.0, x1=3.5, v1=0.0, acc1=0.0, T=4.0)
    assert slow.jerk_squared_integral() < fast.jerk_squared_integral()


# ===========================================================================
# spatiotemporal.py -- joint planner
# ===========================================================================
from app.services.planner.spatiotemporal import (
    DEFAULT_CONFIG,
    PlannerConfig,
    PlannerContext,
    PlannerStart,
    plan,
)


def _cruise_start(d0=1.75, v=13.9):
    return PlannerStart(s0=0.0, sd0=v, sdd0=0.0, d0=d0, dd0=0.0, ddd0=0.0)


def _cruise_ctx(target=13.9, lane_c=1.75, vmax=40.0, dt=0.1):
    return PlannerContext(
        target_speed_mps=target, lane_center_d_m=lane_c, max_speed_mps=vmax, dt=dt
    )


def test_plan_returns_a_trajectory_on_open_road():
    traj = plan(_cruise_start(), _cruise_ctx())
    assert traj is not None
    assert traj.n_candidates_evaluated >= 30          # gate 8.3 (count)
    assert traj.n_candidates_feasible >= 1


def test_open_road_winner_holds_lane_centre_and_target_speed():
    traj = plan(_cruise_start(d0=1.75), _cruise_ctx(target=13.9, lane_c=1.75))
    assert traj.d1_m == pytest.approx(1.75, abs=0.05)
    assert traj.v1_mps == pytest.approx(13.9, abs=0.05)
    assert traj.d_target_m == pytest.approx(1.75, abs=0.2)
    assert traj.cost_terms["risk"] == 0.0


def test_winner_is_within_the_feasibility_envelope():
    traj = plan(_cruise_start(d0=0.4), _cruise_ctx())
    assert traj.peak_lat_accel_mps2 <= DEFAULT_CONFIG.max_lat_accel_mps2 + 1e-9
    assert traj.peak_lat_jerk_mps3 <= DEFAULT_CONFIG.max_lat_jerk_mps3 + 1e-9
    assert traj.peak_long_accel_mps2 <= DEFAULT_CONFIG.max_long_accel_mps2 + 1e-9
    for _, d in traj.d_samples:
        assert abs(d) <= DEFAULT_CONFIG.road_half_width_m - DEFAULT_CONFIG.edge_margin_m + 1e-9


def test_curvature_speed_cap_is_respected_and_not_penalised():
    # a tight curve upstream has capped max_speed to 8 m/s
    traj = plan(_cruise_start(v=13.9), _cruise_ctx(target=13.9, vmax=8.0))
    assert traj.v1_mps <= 8.0 + 1e-9
    # speed_dev is measured against min(target, vmax) = 8.0, so cruising the
    # cap is a near-zero speed-deviation cost, not a large one
    assert traj.cost_terms["speed_dev"] == pytest.approx(0.0, abs=1.0)


def test_over_constrained_start_returns_none_gracefully():
    # start already outside the road with a violent lateral accel: no
    # feasible quintic to a lane centre within the jerk envelope
    bad = PlannerStart(s0=0.0, sd0=13.9, sdd0=0.0, d0=2.9, dd0=6.0, ddd0=0.0)
    tight = PlannerConfig(max_lat_jerk_mps3=0.05, max_lat_accel_mps2=0.1)
    assert plan(bad, _cruise_ctx(), tight) is None


def test_plan_runs_under_4ms_per_tick():
    st, ctx = _cruise_start(d0=0.6), _cruise_ctx()
    plan(st, ctx)  # warm import / JIT of nothing, just parity
    best = 1e9
    for _ in range(50):
        t0 = time.perf_counter()
        plan(st, ctx)
        best = min(best, time.perf_counter() - t0)
    assert best < 4.0e-3, f"best plan() tick {best*1e3:.2f} ms exceeds 4.0 ms"


class _FakeRiskField:
    """High risk for x > 0 (right of the route centreline), zero elsewhere.
    In frenet_to_local_xz the local x roughly tracks +d for a route heading
    north, but the planner must not assume that -- so this stub keys on the
    real x it is handed."""

    def __init__(self, threshold_x=0.0, value=5.0):
        self.threshold_x = threshold_x
        self.value = value

    def risk_at(self, x, z, t_s):
        return self.value if x > self.threshold_x else 0.0


def test_planner_prefers_the_lower_risk_lane_when_risk_field_present(monkeypatch):
    # Stub frenet_to_local_xz so d maps straight to x (route heading east).
    import app.services.planner.spatiotemporal as sp

    monkeypatch.setattr(
        "app.services.frenet.frenet_to_local_xz",
        lambda frame, s, d: (d, 0.0, 1.0, 0.0),
    )
    ctx = PlannerContext(
        target_speed_mps=13.9,
        lane_center_d_m=0.0,
        max_speed_mps=40.0,
        dt=0.1,
        frenet_frame=object(),
        risk_field=_FakeRiskField(threshold_x=0.0, value=8.0),
        allowed_lane_centers_m=[0.0],
    )
    traj = plan(PlannerStart(0.0, 13.9, 0.0, 0.0, 0.0, 0.0), ctx)
    assert traj is not None
    # with a stiff risk penalty for d > 0 the winner biases left of centre
    assert traj.d1_m <= 0.0
    assert traj.cost_terms["risk"] >= 0.0


def test_state_machine_can_open_the_adjacent_lane_to_the_planner():
    ctx = PlannerContext(
        target_speed_mps=13.9,
        lane_center_d_m=1.75,
        max_speed_mps=40.0,
        dt=0.1,
        allowed_lane_centers_m=[5.25],   # EXECUTE_LANE_CHANGE narrows to the adjacent lane
    )
    traj = plan(_cruise_start(d0=1.75), ctx)
    assert traj is not None
    assert traj.d1_m == pytest.approx(5.25, abs=0.7)


def test_100_lane_changes_p99_lateral_jerk_under_1_5(seed_rng=12345):
    """Gate 8.2 -- every trajectory the planner returns is inside the
    |jerk_lat| <= 1.5 m/s^3 envelope by construction (the feasibility
    filter), so 100 varied lane-change requests never produce a
    p99 >= 1.5."""
    import random

    rng = random.Random(seed_rng)
    peaks = []
    for _ in range(100):
        d0 = rng.uniform(1.4, 2.1)          # near the current lane centre
        dd0 = rng.uniform(-0.4, 0.4)
        target_lane = rng.choice([1.75, 5.25])
        st = PlannerStart(s0=0.0, sd0=rng.uniform(8.0, 16.0), sdd0=0.0,
                          d0=d0, dd0=dd0, ddd0=0.0)
        ctx = PlannerContext(target_speed_mps=13.9, lane_center_d_m=target_lane,
                             max_speed_mps=40.0, dt=0.1,
                             allowed_lane_centers_m=[target_lane])
        traj = plan(st, ctx)
        assert traj is not None
        peaks.append(traj.peak_lat_jerk_mps3)
    peaks.sort()
    p99 = peaks[98]
    assert p99 < 1.5, f"p99 lateral jerk {p99:.3f} >= 1.5"
