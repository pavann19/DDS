"""Velocity/acceleration projection uses the left-positive coordinate contract."""
import math
import pytest
from app.services.driver.frenet_motion import project_motion


def test_constant_offset_circle_preserves_physical_speed_and_acceleration():
    k, d, v, a = .01, 2., 10., 1.5
    scale = 1+k*d
    motion = project_motion(speed_mps=v, acceleration_mps2=a, motion_heading_rad=0., road_heading_rad=0.,
                            motion_yaw_rate_radps=v*k/scale, curvature_1pm=k, curvature_derivative_1pm2=0., lateral_offset_m=d)
    assert motion.station_speed_mps == pytest.approx(v/scale)
    assert motion.station_accel_mps2 == pytest.approx(a/scale)
    assert motion.lateral_speed_mps == 0.
    assert motion.lateral_accel_mps2 == pytest.approx(0., abs=1e-12)


def test_rightward_motion_has_negative_left_positive_lateral_velocity():
    motion = project_motion(speed_mps=10., acceleration_mps2=2., motion_heading_rad=math.pi/4, road_heading_rad=0.,
                            motion_yaw_rate_radps=.1, curvature_1pm=0., curvature_derivative_1pm2=0., lateral_offset_m=0.)
    assert motion.station_speed_mps == pytest.approx(10/math.sqrt(2))
    assert motion.lateral_speed_mps == pytest.approx(-10/math.sqrt(2))
    assert motion.station_accel_mps2 == pytest.approx(1/math.sqrt(2))
    assert motion.lateral_accel_mps2 == pytest.approx(-3/math.sqrt(2))


@pytest.mark.parametrize('offset', [-10., -11.])
def test_folded_reference_is_rejected(offset):
    with pytest.raises(ValueError):
        project_motion(speed_mps=1., acceleration_mps2=0., motion_heading_rad=0., road_heading_rad=0.,
                       motion_yaw_rate_radps=0., curvature_1pm=.1, curvature_derivative_1pm2=0., lateral_offset_m=offset)


@pytest.mark.parametrize('heading',[0.,.1,-.1])
def test_stationary_brake_demand_is_not_reverse_body_motion(heading):
    motion=project_motion(speed_mps=0.,acceleration_mps2=-.15,
        motion_heading_rad=heading,road_heading_rad=0.,motion_yaw_rate_radps=0.,
        curvature_1pm=.01,curvature_derivative_1pm2=0.,lateral_offset_m=2.24)
    assert motion.station_speed_mps == 0.
    assert motion.station_accel_mps2 == 0.
    assert motion.lateral_speed_mps == 0.
    assert motion.lateral_accel_mps2 == 0.


def test_positive_launch_and_moving_braking_remain_motion_inputs():
    def motion(v,a):
        return project_motion(speed_mps=v,acceleration_mps2=a,motion_heading_rad=0.,
            road_heading_rad=0.,motion_yaw_rate_radps=0.,curvature_1pm=0.,
            curvature_derivative_1pm2=0.,lateral_offset_m=0.)
    assert motion(0.,1.).station_accel_mps2 == 1.
    assert motion(.001,-.15).station_accel_mps2 == -.15
