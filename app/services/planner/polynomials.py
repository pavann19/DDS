"""Quintic / quartic polynomial trajectory primitives for the Phase 8
joint ``(s, d, t)`` planner.

Follows Werling, Ziegler, Kammel & Thrun, "Optimal Trajectory Generation
for Dynamic Street Scenarios in a Frenet Frame" (ICRA 2010):

- A **quintic** ``x(t)`` is uniquely determined by position / velocity /
  acceleration at both ends ``(x0, x0', x0'')`` and ``(x1, x1', x1'')``.
  Used for the lateral profile ``d(t)`` (all three terminal conditions
  matter: arrive at the lane centre with zero lateral speed and zero
  lateral acceleration) and for the longitudinal "stopping / following"
  mode where a terminal station is specified.

- A **quartic** ``x(t)`` drops the terminal-position constraint and is
  determined by ``(x0, x0', x0'')`` and ``(x1', x1'')``. Used for the
  longitudinal "velocity keeping" (cruise) mode: reach a target speed with
  a target acceleration, wherever that happens to leave the station.

Both expose the closed-form jerk-squared integral ``∫₀ᵀ x'''(t)² dt`` used
directly as the comfort term of the trajectory cost — no numerical
quadrature per candidate per tick.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple


def _solve3(m: List[List[float]], b: List[float]) -> Tuple[float, float, float]:
    """3x3 linear solve by Cramer's rule. The planner only calls this with
    the Werling terminal-condition matrix for ``T > 0``, which is always
    non-singular (``det = 12 T^9``), so no pivoting / degeneracy handling
    is needed."""
    def det3(a: List[List[float]]) -> float:
        return (
            a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
        )

    d = det3(m)
    cols = []
    for c in range(3):
        mc = [[b[r] if k == c else m[r][k] for k in range(3)] for r in range(3)]
        cols.append(det3(mc) / d)
    return cols[0], cols[1], cols[2]


@dataclass
class QuinticPolynomial:
    """``x(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5`` fitted to
    ``(x0, v0, acc0)`` at ``t = 0`` and ``(x1, v1, acc1)`` at ``t = T``."""

    x0: float
    v0: float
    acc0: float
    x1: float
    v1: float
    acc1: float
    T: float
    coef: Tuple[float, float, float, float, float, float] = field(init=False)

    def __post_init__(self) -> None:
        T = self.T
        a0, a1, a2 = self.x0, self.v0, self.acc0 / 2.0
        # Terminal conditions minus the part fixed by (a0, a1, a2).
        c0 = self.x1 - (a0 + a1 * T + a2 * T * T)
        c1 = self.v1 - (a1 + 2.0 * a2 * T)
        c2 = self.acc1 - 2.0 * a2
        m = [
            [T ** 3, T ** 4, T ** 5],
            [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
            [6 * T, 12 * T ** 2, 20 * T ** 3],
        ]
        a3, a4, a5 = _solve3(m, [c0, c1, c2])
        self.coef = (a0, a1, a2, a3, a4, a5)

    def pos(self, t: float) -> float:
        a0, a1, a2, a3, a4, a5 = self.coef
        return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

    def vel(self, t: float) -> float:
        _, a1, a2, a3, a4, a5 = self.coef
        return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4

    def acc(self, t: float) -> float:
        _, _, a2, a3, a4, a5 = self.coef
        return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3

    def jerk(self, t: float) -> float:
        _, _, _, a3, a4, a5 = self.coef
        return 6 * a3 + 24 * a4 * t + 60 * a5 * t ** 2

    def jerk_squared_integral(self, T: float | None = None) -> float:
        """``∫₀ᵀ x'''(t)^2 dt`` in closed form (``x''' = 6a3 + 24a4 t +
        60a5 t^2``)."""
        _, _, _, a3, a4, a5 = self.coef
        t = self.T if T is None else T
        return (
            36 * a3 ** 2 * t
            + 144 * a3 * a4 * t ** 2
            + (192 * a4 ** 2 + 240 * a3 * a5) * t ** 3
            + 720 * a4 * a5 * t ** 4
            + 720 * a5 ** 2 * t ** 5
        )


@dataclass
class QuarticPolynomial:
    """``x(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4`` fitted to
    ``(x0, v0, acc0)`` at ``t = 0`` and ``(v1, acc1)`` at ``t = T`` — no
    terminal position constraint (longitudinal velocity-keeping mode)."""

    x0: float
    v0: float
    acc0: float
    v1: float
    acc1: float
    T: float
    coef: Tuple[float, float, float, float, float] = field(init=False)

    def __post_init__(self) -> None:
        T = self.T
        a0, a1, a2 = self.x0, self.v0, self.acc0 / 2.0
        c1 = self.v1 - (a1 + 2.0 * a2 * T)
        c2 = self.acc1 - 2.0 * a2
        # [3T^2  4T^3 ] [a3]   [c1]        det = 12 T^4
        # [6T    12T^2] [a4] = [c2]
        det = 12.0 * T ** 4
        a3 = (c1 * 12.0 * T ** 2 - 4.0 * T ** 3 * c2) / det
        a4 = (3.0 * T ** 2 * c2 - 6.0 * T * c1) / det
        self.coef = (a0, a1, a2, a3, a4)

    def pos(self, t: float) -> float:
        a0, a1, a2, a3, a4 = self.coef
        return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4

    def vel(self, t: float) -> float:
        _, a1, a2, a3, a4 = self.coef
        return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3

    def acc(self, t: float) -> float:
        _, _, a2, a3, a4 = self.coef
        return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2

    def jerk(self, t: float) -> float:
        _, _, _, a3, a4 = self.coef
        return 6 * a3 + 24 * a4 * t

    def jerk_squared_integral(self, T: float | None = None) -> float:
        """``∫₀ᵀ x'''(t)^2 dt`` in closed form (``x''' = 6a3 + 24a4 t``)."""
        _, _, _, a3, a4 = self.coef
        t = self.T if T is None else T
        return 36 * a3 ** 2 * t + 144 * a3 * a4 * t ** 2 + 192 * a4 ** 2 * t ** 3
