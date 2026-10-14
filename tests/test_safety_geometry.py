import pytest
from app.services.safety_geometry import Footprint, intersects_during

@pytest.mark.parametrize("x,z", [(0,0),(1.5,0),(0,3),(0,-3)])
def test_overlap_from_any_side_or_stopped(x,z):
    assert intersects_during(Footprint(0,0,0,4.8,1.8), Footprint(x,z,0,4.8,1.8),0)

def test_swept_collision_between_endpoints():
    assert intersects_during(Footprint(0,0,0,4.8,1.8), Footprint(-5,0,0,4.8,1.8,100,0),.1)

def test_separated_adjacent_traffic_does_not_brake():
    assert not intersects_during(Footprint(0,0,0,4.8,1.8), Footprint(3.5,0,0,4.8,1.8))


def test_monitor_applies_swept_footprint_veto():
    from app.services.driver.safety_monitor import SafetyMonitor
    monitor = SafetyMonitor()
    accel, reason = monitor.step(ego_speed_mps=10, lateral_offset_m=1.75,
        lateral_accel_mps2=0, sensed_lead_gap_m=None, sensed_lead_speed_mps=None,
        desired_accel=2, a_max_brake_mps2=4.5, recovery_speed_kmh=15, speed_kp=.6,
        footprint_risk=True)
    assert accel == -4.5
    assert reason == "safety_shield_override"


def test_rear_forecast_reports_risk_without_counterproductive_braking():
    from app.services.driver.safety_monitor import SafetyMonitor
    monitor = SafetyMonitor()
    accel, reason = monitor.step(ego_speed_mps=10, lateral_offset_m=1.75,
        lateral_accel_mps2=0, sensed_lead_gap_m=None, sensed_lead_speed_mps=None,
        desired_accel=0, a_max_brake_mps2=4.5, recovery_speed_kmh=15, speed_kp=.6,
        rear_conflict=True)
    assert accel == 0
    assert reason is None
    assert monitor.verdict.risk_level == "MEDIUM"
    assert monitor.verdict.reasons
