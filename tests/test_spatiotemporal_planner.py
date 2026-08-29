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
