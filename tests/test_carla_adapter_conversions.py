"""Coordinate-frame and steering-direction conversion tests -- MUST pass
before any CARLA result is trusted (a sign error here silently steers the
vehicle the wrong way). Pure; no ``carla`` import.
"""
import math

import pytest

from app.services.carla_adapter import conversions as C


# --- heading -------------------------------------------------------------
def test_heading_roundtrip_and_cardinals():
    for h in (0.0, 45.0, 90.0, 179.0, 270.0, 359.0):
        assert C.carla_yaw_deg_to_dds_heading(C.dds_heading_to_carla_yaw_deg(h)) == pytest.approx(h % 360.0)
    # DDS North (0) -> CARLA yaw 0 ; DDS East (90) -> CARLA yaw +90 (same sense)
    assert C.dds_heading_to_carla_yaw_deg(0.0) == pytest.approx(0.0)
    assert C.dds_heading_to_carla_yaw_deg(90.0) == pytest.approx(90.0)
    assert C.dds_heading_to_carla_yaw_deg(270.0) == pytest.approx(-90.0)


# --- position ----------------------------------------------------------
def test_position_roundtrip_and_axes():
    # DDS (x=East, z=South) -> CARLA (x=North, y=East)
    x_north, y_east = C.dds_local_xz_to_carla_xy(x_east_m=10.0, z_south_m=4.0)
    assert x_north == pytest.approx(-4.0)   # North = -South
    assert y_east == pytest.approx(10.0)    # East = East
    back = C.carla_xy_to_dds_local_xz(x_north, y_east)
    assert back == pytest.approx((10.0, 4.0))


def test_forward_vector_matches_frenet_convention():
    # heading 0 (North) -> (0,0,-1) ; heading 90 (East) -> (1,0,0)
    assert C.dds_forward_vector(0.0) == pytest.approx((0.0, 0.0, -1.0))
    assert C.dds_forward_vector(90.0) == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)


# --- steering: SAME sign, + = right ----------------------------------
def test_steer_sign_is_preserved_right_is_right():
    assert C.dds_steer_rad_to_carla(+0.3) > 0.0   # DDS right -> CARLA right
    assert C.dds_steer_rad_to_carla(-0.3) < 0.0   # DDS left  -> CARLA left
    assert C.dds_steer_rad_to_carla(0.0) == 0.0


def test_steer_scale_and_clamp():
    m = C.DDS_MAX_STEER_RAD
    assert C.dds_steer_rad_to_carla(m) == pytest.approx(1.0)
    assert C.dds_steer_rad_to_carla(-m) == pytest.approx(-1.0)
    assert C.dds_steer_rad_to_carla(3 * m) == 1.0     # clamped
    assert C.dds_steer_rad_to_carla(-3 * m) == -1.0
    # mid-range roundtrip
    assert C.carla_steer_to_dds_rad(C.dds_steer_rad_to_carla(0.2)) == pytest.approx(0.2)


# --- longitudinal: accel -> throttle/brake, correctly signed ---------
def test_accel_maps_to_throttle_positive_and_brake_negative():
    t, b = C.dds_accel_to_carla_throttle_brake(2.0)
    assert t > 0.0 and b == 0.0
    t, b = C.dds_accel_to_carla_throttle_brake(-3.0)
    assert t == 0.0 and b > 0.0
    t, b = C.dds_accel_to_carla_throttle_brake(0.0)
    assert t == 0.0 and b == 0.0
    # monotone + clamped to [0,1]
    assert C.dds_accel_to_carla_throttle_brake(100.0)[0] == 1.0
    assert C.dds_accel_to_carla_throttle_brake(-100.0)[1] == 1.0
    assert C.dds_accel_to_carla_throttle_brake(3.0)[0] > C.dds_accel_to_carla_throttle_brake(1.5)[0]


def test_full_command_conversion():
    ctl = C.dds_command_to_carla_control(steer_rad=-0.2, accel_mps2=1.5)
    assert isinstance(ctl, C.CarlaControl)
    assert ctl.steer < 0.0            # left
    assert ctl.throttle > 0.0 and ctl.brake == 0.0
    ctl = C.dds_command_to_carla_control(steer_rad=0.4, accel_mps2=-2.0)
    assert ctl.steer > 0.0 and ctl.brake > 0.0 and ctl.throttle == 0.0
