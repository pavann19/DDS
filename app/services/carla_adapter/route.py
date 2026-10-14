"""``RouteProvider`` -- produces the ``(x, y)`` route polyline a
``WorldBackend.reset`` consumes, and the ``FrenetFrame`` the Driver projects
against.

* ``DeterministicRouteProvider`` -- recorded real-road / explicit routes
  (lat/lng), used with ``DeterministicSimBackend``.
* ``CarlaRouteProvider`` -- (skeleton) wraps CARLA
  ``agents.navigation.GlobalRoutePlanner``; its waypoints are already a
  dense per-lane centreline in the CARLA metric frame -- no spline smoothing
  needed. Built when a CARLA host exists.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional, Sequence, Tuple

from app.services.frenet import build_frenet_frame
from app.services.road_routes import real_road_route


class RouteProvider(ABC):
    @abstractmethod
    def waypoints(self) -> List[Tuple[float, float]]:
        """Route polyline in the backend's route frame."""

    def frenet_frame(self):
        """FrenetFrame the Driver projects candidates against (map data)."""
        return build_frenet_frame(self.waypoints())


class DeterministicRouteProvider(RouteProvider):
    """Recorded US 101 road geometry or an explicit lat/lng polyline."""

    def __init__(
        self,
        *,
        waypoints: Optional[Sequence[Tuple[float, float]]] = None,
    ) -> None:
        if waypoints is not None:
            self._wps = [tuple(p) for p in waypoints]
        else:
            self._wps = real_road_route()

    def waypoints(self) -> List[Tuple[float, float]]:
        return list(self._wps)


class CarlaRouteProvider(RouteProvider):  # pragma: no cover - skeleton
    """SKELETON -- CARLA GlobalRoutePlanner. See docs/CARLA_MIGRATION_PLAN.md."""

    def __init__(self, carla_map, start, dest, sampling_resolution_m: float = 2.0):
        self._map = carla_map
        self._start = start
        self._dest = dest
        self._res = sampling_resolution_m
        raise NotImplementedError(
            "CarlaRouteProvider needs a CARLA host -- see docs/CARLA_FEASIBILITY.md"
        )

    def waypoints(self) -> List[Tuple[float, float]]:
        # GlobalRoutePlanner(self._map, self._res).trace_route(self._start, self._dest)
        # -> [(wp.transform.location.x, wp.transform.location.y), ...]
        raise NotImplementedError
