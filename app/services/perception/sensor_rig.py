"""
360-degree virtual sensor rig (Phase 6, P6-1).

Replaces traffic.py's 1D forward-only range cone (sense_lead_vehicle) with
5 named frustums covering the full azimuth circle around the ego, so
blind-spot/rear/side traffic becomes visible to perception for the first
time. traffic.py's forward sensor is untouched -- it's still what IDM/the
planner consume for car-following; this module is the new, separate
360-degree awareness layer Phase 6-7-8 build on.

Geometry: both the ego and every actor share ONE Frenet frame (the route
frame already built by app/services/frenet.py), so both are projected into
the SAME local (x, z) plane via frenet_to_local_xz before any frustum test
runs. Azimuth is measured relative to the ego's own forward tangent at its
station, using the exact same "right = (dir_z, -dir_x)" convention
frenet.py/routeGeometry.ts/traffic.py all already agree on -- a frustum
azimuth here has to mean the same "which side" as everywhere else in this
project, or detections would silently point the wrong way on screen.

Azimuth convention: 0 deg = straight ahead (ego forward), positive = right
of forward, range (-180, 180], matching the project's existing right-handed
d/right convention.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from app.services.frenet import FrenetFrame, frenet_to_local_xz


@dataclass(frozen=True)
class SensorFrustum:
    name: str
    range_m: float
    fov_deg: float          # total angular width
    center_azimuth_deg: float  # 0 = forward, 180/-180 = directly behind


# Five frustums approximating a real AV sensor suite. Deliberately does not
# claim full contiguous 360-degree coverage -- there are narrow gaps near
# the pure left/right beam (~60-90 deg either side), the same kind of
# realistic coverage gap real sensor suites have between overlapping
# cameras/radars, not a bug to "fix" by inventing a 6th sensor with no
# real-world analogue.
SENSOR_RIG: Tuple[SensorFrustum, ...] = (
    SensorFrustum("forward_long_range", range_m=150.0, fov_deg=30.0, center_azimuth_deg=0.0),
    SensorFrustum("forward_wide", range_m=60.0, fov_deg=120.0, center_azimuth_deg=0.0),
    SensorFrustum("left_blind_spot", range_m=80.0, fov_deg=90.0, center_azimuth_deg=-135.0),
    SensorFrustum("right_blind_spot", range_m=80.0, fov_deg=90.0, center_azimuth_deg=135.0),
    SensorFrustum("rear_center", range_m=100.0, fov_deg=60.0, center_azimuth_deg=180.0),
)


def _wrap_deg(angle_deg: float) -> float:
    """Wrap to (-180, 180]."""
    a = (angle_deg + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


@dataclass
class RelativeObservation:
    range_m: float
    azimuth_deg: float  # relative to ego forward, see module docstring
    x: float             # world-local x of the observed actor
    z: float             # world-local z of the observed actor


def relative_observation_from_ego_frame(
    ego_x: float,
    ego_z: float,
    dir_x: float,
    dir_z: float,
    actor_x: float,
    actor_z: float,
) -> RelativeObservation:
    """Same math as ego_relative_observation, taking the ego's own
    projection (x, z, forward tangent) already computed rather than
    re-deriving it. The ego's projection is fixed for an entire tick --
    recomputing it once per actor (as ego_relative_observation does) is
    pure repeated work in a per-tick hot loop over many actors; this is the
    fast path perception_engine.py's step() uses for that reason. See
    Gate 6.3's 2ms/tick @ 30-actor performance budget."""
    dx, dz = actor_x - ego_x, actor_z - ego_z
    # Same right-vector convention as frenet.py's frenet_to_local_xz.
    right_x, right_z = dir_z, -dir_x

    forward_component = dx * dir_x + dz * dir_z
    right_component = dx * right_x + dz * right_z
    range_m = math.hypot(forward_component, right_component)
    azimuth_deg = math.degrees(math.atan2(right_component, forward_component))
    return RelativeObservation(range_m=range_m, azimuth_deg=azimuth_deg, x=actor_x, z=actor_z)


def ego_relative_observation(
    frame: FrenetFrame,
    ego_s: float,
    ego_d: float,
    actor_s: float,
    actor_d: float,
) -> RelativeObservation:
    """Project both ego and actor into the shared local plane and return the
    actor's range/azimuth relative to the ego's own forward tangent."""
    ego_x, ego_z, dir_x, dir_z = frenet_to_local_xz(frame, ego_s, ego_d)
    actor_x, actor_z, _, _ = frenet_to_local_xz(frame, actor_s, actor_d)
    return relative_observation_from_ego_frame(ego_x, ego_z, dir_x, dir_z, actor_x, actor_z)


def frustum_contains(frustum: SensorFrustum, observation: RelativeObservation) -> bool:
    if observation.range_m > frustum.range_m:
        return False
    half_fov = frustum.fov_deg / 2.0
    delta = abs(_wrap_deg(observation.azimuth_deg - frustum.center_azimuth_deg))
    return delta <= half_fov


def detecting_frustums(observation: RelativeObservation, rig: Tuple[SensorFrustum, ...] = SENSOR_RIG) -> List[str]:
    """Names of every frustum in the rig that currently sees this
    observation (an actor near a frustum boundary can be seen by more than
    one, same as overlapping camera/radar coverage on a real car)."""
    return [f.name for f in rig if frustum_contains(f, observation)]


def is_detected(observation: RelativeObservation, rig: Tuple[SensorFrustum, ...] = SENSOR_RIG) -> bool:
    return any(frustum_contains(f, observation) for f in rig)


def batch_relative_observations(
    ego_x: float,
    ego_z: float,
    dir_x: float,
    dir_z: float,
    actor_xs,
    actor_zs,
):
    """Vectorized relative_observation_from_ego_frame for many actors
    against the same ego projection at once -- same formula, just computed
    with numpy arrays instead of a Python loop calling the scalar version
    once per actor. Returns (range_m, azimuth_deg) arrays. See
    Gate 6.3's 2ms/tick @ 30-actor performance budget."""
    ax = np.asarray(actor_xs, dtype=float)
    az = np.asarray(actor_zs, dtype=float)
    dx, dz = ax - ego_x, az - ego_z
    right_x, right_z = dir_z, -dir_x

    forward_component = dx * dir_x + dz * dir_z
    right_component = dx * right_x + dz * right_z
    range_m = np.hypot(forward_component, right_component)
    azimuth_deg = np.degrees(np.arctan2(right_component, forward_component))
    return range_m, azimuth_deg


def batch_detecting_mask(range_m, azimuth_deg, rig: Tuple[SensorFrustum, ...] = SENSOR_RIG):
    """Vectorized is_detected for many actors at once: a boolean array,
    True where ANY frustum in the rig sees that actor. Same rule as
    frustum_contains (range cutoff + wrapped-azimuth half-FOV check), just
    evaluated once per frustum across every actor instead of once per
    (actor, frustum) pair via nested Python-level calls."""
    range_m = np.asarray(range_m, dtype=float)
    azimuth_deg = np.asarray(azimuth_deg, dtype=float)
    detected = np.zeros(range_m.shape, dtype=bool)
    for f in rig:
        diff = azimuth_deg - f.center_azimuth_deg
        wrapped = (diff + 180.0) % 360.0 - 180.0
        in_fov = np.abs(wrapped) <= (f.fov_deg / 2.0)
        in_range = range_m <= f.range_m
        detected |= (in_fov & in_range)
    return detected
