"""Pure DDS <-> CARLA coordinate / heading / control conversions.

Nothing here imports ``carla`` -- these are the maps the ``CarlaBackend`` will
use, and they are fully unit-tested without a running CARLA
(``tests/test_carla_adapter_conversions.py``). A sign error in any of these
silently steers the vehicle the wrong way, so each is spelled out and pinned.

Frame conventions
-----------------
* **DDS local metric frame** (``app/services/frenet.py``): right-handed with
  ``x = East``, ``z = South`` (``z = -(lat - origin_lat) * R``). "Forward at
  heading h" is ``(sin h, 0, -cos h)`` and ``heading = atan2(dir_x, -dir_z)``
  measured clockwise from North (0 deg = North, 90 deg = East).
* **CARLA world frame** (Unreal): left-handed, ``x = forward/North``,
  ``y = East/right``, ``z = up``, **units = metres**, ``rotation.yaw`` in
  **degrees, clockwise from the +x axis** (so yaw 0 = +x, yaw +90 = +y).

* **DDS steering** ``steering_angle_rad`` -- front-wheel angle; **positive =
  turn toward -d = the vehicle's right** (from the kinematic-bicycle
  integration ``yaw_rate = v * tan(delta) / L`` with the heading convention
  above). Range +/- ``MAX_STEER_RAD`` (35 deg).
* **CARLA steering** ``VehicleControl.steer`` -- normalised **[-1, 1]**,
  **positive = right**. So DDS->CARLA steering is a same-sign scale by
  ``1 / max_steer_rad``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# DDS front-wheel limit (radians) -- mirrors PhysicsEngine.MAX_STEER_RAD.
DDS_MAX_STEER_RAD = math.radians(35.0)
# Comfort accel / brake envelope (m/s^2) -- mirrors PhysicsEngine.
DDS_MAX_ACCEL_MPS2 = 3.0
DDS_MAX_BRAKE_MPS2 = 4.5


# ---------------------------------------------------------------------------
# heading
# ---------------------------------------------------------------------------
def dds_heading_to_carla_yaw_deg(dds_heading_deg: float) -> float:
    """DDS heading (deg, clockwise from North, 0 = North) -> CARLA yaw
    (deg, clockwise from +x). DDS North (+ x=East, z=South) maps to CARLA +x
    (North), and DDS East to CARLA +y, i.e. the two are the SAME rotation
    sense with the SAME zero once axes are matched: yaw = heading.
    Normalised to (-180, 180]."""
    y = dds_heading_deg % 360.0
    return y - 360.0 if y > 180.0 else y


def carla_yaw_deg_to_dds_heading(carla_yaw_deg: float) -> float:
    """Inverse of :func:`dds_heading_to_carla_yaw_deg`. -> [0, 360)."""
    return carla_yaw_deg % 360.0


# ---------------------------------------------------------------------------
# position  (DDS local metric  <->  CARLA world metric)
# ---------------------------------------------------------------------------
def dds_local_xz_to_carla_xy(x_east_m: float, z_south_m: float) -> Tuple[float, float]:
    """DDS ``(x = East, z = South)`` metres -> CARLA ``(x = North, y = East)``
    metres. North = -z_south, East = x_east."""
    carla_x_north = -z_south_m
    carla_y_east = x_east_m
    return carla_x_north, carla_y_east


def carla_xy_to_dds_local_xz(carla_x_north: float, carla_y_east: float) -> Tuple[float, float]:
    """Inverse: CARLA ``(x=North, y=East)`` -> DDS ``(x=East, z=South)``."""
    x_east_m = carla_y_east
    z_south_m = -carla_x_north
    return x_east_m, z_south_m


def dds_forward_vector(heading_deg: float) -> Tuple[float, float, float]:
    """DDS forward unit vector for a heading: ``(sin h, 0, -cos h)`` in the
    ``(x=East, y=up, z=South)`` frame -- matches ``frenet``/``pure_pursuit``
    and the frontend ``headingToForward``."""
    h = math.radians(heading_deg)
    return math.sin(h), 0.0, -math.cos(h)


# ---------------------------------------------------------------------------
# control
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CarlaControl:
    """A plain mirror of ``carla.VehicleControl`` (so this module needs no
    ``carla`` import). The backend converts this to the real type."""
    throttle: float          # [0, 1]
    steer: float             # [-1, 1], + = right
    brake: float             # [0, 1]
    hand_brake: bool = False
    reverse: bool = False


def dds_steer_rad_to_carla(steer_rad: float,
                           max_steer_rad: float = DDS_MAX_STEER_RAD) -> float:
    """DDS front-wheel angle (rad, + = right) -> CARLA ``steer`` in [-1, 1]
    (+ = right). Same sign; scaled by the geometric limit; clamped."""
    if max_steer_rad <= 0.0:
        return 0.0
    return max(-1.0, min(1.0, steer_rad / max_steer_rad))


def carla_steer_to_dds_rad(carla_steer: float,
                           max_steer_rad: float = DDS_MAX_STEER_RAD) -> float:
    """Inverse -- e.g. to read back what the vehicle is actually doing."""
    return max(-1.0, min(1.0, carla_steer)) * max_steer_rad


def dds_accel_to_carla_throttle_brake(
    accel_mps2: float,
    *,
    max_accel_mps2: float = DDS_MAX_ACCEL_MPS2,
    max_brake_mps2: float = DDS_MAX_BRAKE_MPS2,
    deadband_mps2: float = 0.05,
) -> Tuple[float, float]:
    """DDS demanded longitudinal acceleration (m/s^2, + = speed up) ->
    ``(throttle, brake)`` each in [0, 1].

    This is an OPEN-LOOP first cut: CARLA's actual throttle->accel map is
    vehicle- and speed-dependent, so a closed-loop PI on measured accel
    should replace this once CARLA is available (flagged in
    CARLA_MIGRATION_PLAN.md). It is monotone and correctly signed, which is
    what the pre-CARLA tests pin.
    """
    if accel_mps2 > deadband_mps2:
        return min(1.0, accel_mps2 / max_accel_mps2), 0.0
    if accel_mps2 < -deadband_mps2:
        return 0.0, min(1.0, -accel_mps2 / max_brake_mps2)
    return 0.0, 0.0


def dds_command_to_carla_control(
    steer_rad: float,
    accel_mps2: float,
    *,
    max_steer_rad: float = DDS_MAX_STEER_RAD,
    max_accel_mps2: float = DDS_MAX_ACCEL_MPS2,
    max_brake_mps2: float = DDS_MAX_BRAKE_MPS2,
) -> CarlaControl:
    """Full DDS ``ActuatorCommand`` (steer + accel) -> ``CarlaControl``."""
    throttle, brake = dds_accel_to_carla_throttle_brake(
        accel_mps2, max_accel_mps2=max_accel_mps2, max_brake_mps2=max_brake_mps2
    )
    return CarlaControl(
        throttle=throttle,
        steer=dds_steer_rad_to_carla(steer_rad, max_steer_rad),
        brake=brake,
    )
