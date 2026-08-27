"""Phase 7 -- multi-agent trajectory forecasting & intent.

Gates:
  7.1  lateral drift 0.4 m/s => P(cut-in) > 0.70, >= 1.2 s before crossing
  7.2  ego sheds speed < 1.5 m/s^2 on high-confidence cut-in, no critical TTC
  7.3  stable in-lane traffic keeps P(cut-in) < 0.15 through curves
  7.4  >= 15 new tests here; suite total >= 201
"""
import math

import pytest

from app.services.frenet import build_frenet_frame, local_to_latlng
from app.services.prediction.forecaster import (
    DEFAULT_HORIZON_S,
    DEFAULT_STEP_S,
    AgentKinematics,
    forecast_agent,
    forecast_ctra,
    forecast_lane_following,
    kinematics_from_track,
    project_agent_frenet,
)
from app.services.prediction.intent import (
    Intent,
    estimate_intent,
    estimate_intent_from_track,
)

# Straight route due north, ~5 m spacing (smoothed-OSRM-like).
_ROUTE = [(37.7749 + i * 0.000045, -122.4194) for i in range(240)]
_FRAME = build_frenet_frame(_ROUTE)


def _curved_route(n=200, step_m=5.0, dphi_deg=0.9):
    """A route that curves steadily left: heading rotates dphi_deg per step."""
    x = z = 0.0
    phi = 0.0
    pts_xz = []
    for _ in range(n):
        pts_xz.append((x, z))
        x += step_m * math.sin(phi)
        z += -step_m * math.cos(phi)
        phi += math.radians(dphi_deg)
    return [local_to_latlng(px, pz, 37.7749, -122.4194) for px, pz in pts_xz]


_CURVED_FRAME = build_frenet_frame(_curved_route())


class _Track:
    def __init__(self, track_id, x, z, vx, vz):
        self.track_id, self.x, self.z, self.vx, self.vz = track_id, x, z, vx, vz


# --------------------------------------------------------------------------
# CTRA
# --------------------------------------------------------------------------
def test_ctra_straight_line_advances_along_heading():
    # Heading north-ish in the frenet frame: forward = (sin h, -cos h);
    # moving with vz < 0 is "north" (z = south).
    k = AgentKinematics(track_id=1, x=0.0, z=0.0, vx=0.0, vz=-10.0)
    states = forecast_ctra(k, horizon_s=3.0, step_s=0.1)
    assert len(states) == 30
    assert states[-1].t_s == pytest.approx(3.0)
    # 10 m/s for 3 s => ~30 m north (z decreases by ~30).
    assert states[-1].z == pytest.approx(-30.0, abs=0.5)
    assert states[-1].x == pytest.approx(0.0, abs=1e-6)
    assert states[-1].vz == pytest.approx(-10.0, abs=1e-6)


def test_ctra_constant_acceleration_matches_kinematics():
    k = AgentKinematics(track_id=1, x=0.0, z=0.0, vx=0.0, vz=-10.0, a_long_mps2=2.0)
    states = forecast_ctra(k, horizon_s=2.0, step_s=0.1)
    # v(2) = 10 + 2*2 = 14 m/s ; distance = 10*2 + 0.5*2*4 = 24 m.
    assert math.hypot(states[-1].vx, states[-1].vz) == pytest.approx(14.0, abs=0.1)
    assert abs(states[-1].z) == pytest.approx(24.0, abs=0.3)


def test_ctra_decelerating_agent_stops_and_does_not_reverse():
    k = AgentKinematics(track_id=1, x=0.0, z=0.0, vx=0.0, vz=-10.0, a_long_mps2=-4.0)
    states = forecast_ctra(k, horizon_s=3.0, step_s=0.1)
    speeds = [math.hypot(s.vx, s.vz) for s in states]
    assert min(speeds) == pytest.approx(0.0, abs=1e-9)
    assert all(s >= -1e-9 for s in speeds)
    # Stops at ~1.25 m/s / s => t ~ 2.5 s, ~12.5 m travelled, then holds.
    assert abs(states[-1].z) == pytest.approx(12.5, abs=0.5)
    assert abs(states[-1].z - states[-6].z) < 0.05  # not moving in the last 0.5 s


def test_ctra_turning_agent_rotates_heading_by_yaw_rate_times_time():
    k = AgentKinematics(track_id=1, x=0.0, z=0.0, vx=10.0, vz=0.0, yaw_rate_radps=0.2)
    states = forecast_ctra(k, horizon_s=2.0, step_s=0.1)
    h0 = math.atan2(10.0, -0.0)
    h_end = math.atan2(states[-1].vx, -states[-1].vz)
    assert ((h_end - h0 - 0.4) + math.pi) % (2 * math.pi) - math.pi == pytest.approx(0.0, abs=0.02)
    # Path actually curves (x and z both change).
    assert abs(states[-1].x) > 1.0 and abs(states[-1].z) > 1.0


def test_ctra_state_count_and_spacing_follow_horizon_and_step():
    k = AgentKinematics(track_id=1, x=0.0, z=0.0, vx=0.0, vz=-5.0)
    states = forecast_ctra(k, horizon_s=3.0, step_s=0.2)
    assert len(states) == 15
    assert [round(s.t_s, 4) for s in states][:3] == [0.2, 0.4, 0.6]


# --------------------------------------------------------------------------
# kinematics_from_track
# --------------------------------------------------------------------------
def test_kinematics_from_track_without_prev_has_zero_derived_terms():
    k = kinematics_from_track(_Track(3, 1.0, -20.0, 0.0, -12.0))
    assert k.yaw_rate_radps == 0.0
    assert k.a_long_mps2 == 0.0
    assert k.speed_mps == pytest.approx(12.0)


def test_kinematics_from_track_estimates_and_clamps_yaw_rate():
    prev = AgentKinematics(track_id=3, x=0.0, z=0.0, vx=0.0, vz=-10.0)      # heading 0
    cur = _Track(3, 0.0, -1.0, 10.0, 0.0)                                   # heading +pi/2
    k = kinematics_from_track(cur, prev=prev, dt=0.1)
    # (pi/2)/0.1 = ~15.7 rad/s, clamped to the 1.0 rad/s hard limit.
    assert k.yaw_rate_radps == pytest.approx(1.0)


def test_kinematics_from_track_estimates_longitudinal_acceleration():
    prev = AgentKinematics(track_id=3, x=0.0, z=0.0, vx=0.0, vz=-10.0)
    cur = _Track(3, 0.0, -1.0, 0.0, -10.2)
    k = kinematics_from_track(cur, prev=prev, dt=0.1)
    assert k.a_long_mps2 == pytest.approx(2.0, abs=1e-6)


# --------------------------------------------------------------------------
# Frenet lane-following
# --------------------------------------------------------------------------
def test_lane_following_relaxes_toward_nearest_lane_centre():
    # Agent at station ~100 m, d = 3.0 (between the 1.75 and 5.25 lanes,
    # closer to 5.25 is 2.25 away vs 1.25 to 1.75 -> nearest is 1.75).
    # Place it in local xz for station 100, d = 3.0 on a due-north route:
    # x = East = d (route heads north, right = +East), z = -100.
    k = AgentKinematics(track_id=5, x=3.0, z=-100.0, vx=0.0, vz=-12.0)
    states = forecast_lane_following(k, _FRAME, horizon_s=3.0, step_s=0.1)
    assert len(states) == 30
    # Ends near the 1.75 lane centre (x ~ 1.75), monotonically approaching.
    assert states[-1].x == pytest.approx(1.75, abs=0.25)
    assert abs(states[-1].x - 1.75) < abs(states[0].x - 1.75)
    # Advances ~ along track: 12 m/s * 3 s ~ 36 m north.
    assert states[-1].z == pytest.approx(-136.0, abs=2.0)


def test_lane_following_agent_already_centred_stays_centred():
    k = AgentKinematics(track_id=5, x=1.75, z=-100.0, vx=0.0, vz=-12.0)
    states = forecast_lane_following(k, _FRAME, horizon_s=3.0, step_s=0.1)
    assert max(abs(s.x - 1.75) for s in states) < 0.15


def test_lane_following_velocities_are_populated():
    k = AgentKinematics(track_id=5, x=3.0, z=-100.0, vx=0.0, vz=-12.0)
    states = forecast_lane_following(k, _FRAME, horizon_s=2.0, step_s=0.1)
    assert all(math.hypot(s.vx, s.vz) > 1.0 for s in states)
    # Dominant component is along -z (north), ~12 m/s.
    assert states[10].vz == pytest.approx(-12.0, abs=1.5)


# --------------------------------------------------------------------------
# Model selection
# --------------------------------------------------------------------------
def test_forecast_agent_uses_ctra_when_maneuvering():
    k = AgentKinematics(track_id=7, x=2.0, z=-50.0, vx=1.0, vz=-12.0, yaw_rate_radps=0.3)
    pred = forecast_agent(k, frame=_FRAME)
    assert pred.track_id == 7
    assert len(pred.states) == int(round(DEFAULT_HORIZON_S / DEFAULT_STEP_S))
    # A CTRA turn keeps curving away; lane-following would pull x toward 1.75.
    assert pred.states[-1].x > 2.0


def test_forecast_agent_uses_lane_following_when_tracking_road():
    k = AgentKinematics(track_id=8, x=3.0, z=-50.0, vx=0.0, vz=-12.0, yaw_rate_radps=0.0)
    pred = forecast_agent(k, frame=_FRAME)
    assert pred.states[-1].x == pytest.approx(1.75, abs=0.3)


def test_forecast_agent_without_frame_falls_back_to_ctra():
    k = AgentKinematics(track_id=9, x=3.0, z=-50.0, vx=0.0, vz=-12.0)
    pred = forecast_agent(k, frame=None)
    # Pure CTRA straight line: x unchanged.
    assert pred.states[-1].x == pytest.approx(3.0, abs=1e-6)


# --------------------------------------------------------------------------
# Intent -- Gate 7.1 / 7.3
# --------------------------------------------------------------------------
def test_intent_lane_keep_dominates_for_stable_in_lane_agent():
    est = estimate_intent(
        agent_d=1.75, agent_v_d=0.0, agent_a_long_mps2=0.0,
        agent_speed_mps=12.0, ego_d=1.75,
    )
    assert est.dominant == Intent.LANE_KEEP.value
    assert est.p_cut_in < 0.15
    assert sum(est.distribution.values()) == pytest.approx(1.0)


def test_intent_gate_7_1_sustained_drift_toward_ego_is_a_cut_in():
    # Agent one lane to the right of the ego (d=5.25 vs ego 1.75), drifting
    # left toward the ego at 0.4 m/s, still mid-lane (>1.2 s from the divider).
    est = estimate_intent(
        agent_d=5.25, agent_v_d=-0.4, agent_a_long_mps2=0.0,
        agent_speed_mps=13.0, ego_d=1.75,
    )
    assert est.p_cut_in > 0.70
    assert est.dominant == Intent.MERGE_LEFT.value
    assert est.time_to_cross_s is not None and est.time_to_cross_s > 1.2


def test_intent_cut_in_probability_climbs_as_the_agent_nears_the_divider():
    far = estimate_intent(agent_d=5.25, agent_v_d=-0.4, agent_a_long_mps2=0.0,
                          agent_speed_mps=13.0, ego_d=1.75)
    near = estimate_intent(agent_d=4.0, agent_v_d=-0.4, agent_a_long_mps2=0.0,
                           agent_speed_mps=13.0, ego_d=1.75)
    assert near.p_cut_in > far.p_cut_in
    assert near.time_to_cross_s < far.time_to_cross_s


def test_intent_drift_away_from_ego_is_not_a_cut_in():
    est = estimate_intent(
        agent_d=5.25, agent_v_d=+0.4, agent_a_long_mps2=0.0,
        agent_speed_mps=13.0, ego_d=1.75,
    )
    assert est.p_cut_in < 0.15
    assert est.dominant == Intent.MERGE_RIGHT.value  # merging, just not toward ego


def test_intent_agent_left_of_ego_cut_in_is_merge_right():
    est = estimate_intent(
        agent_d=-1.75, agent_v_d=+0.4, agent_a_long_mps2=0.0,
        agent_speed_mps=13.0, ego_d=1.75,
    )
    assert est.dominant == Intent.MERGE_RIGHT.value
    assert est.p_cut_in > 0.70


def test_intent_small_wander_stays_below_the_action_threshold():
    est = estimate_intent(
        agent_d=5.25, agent_v_d=-0.09, agent_a_long_mps2=0.0,
        agent_speed_mps=13.0, ego_d=1.75,
    )
    assert not est.is_cut_in()
    assert est.p_cut_in < 0.65


def test_intent_decelerating_agent():
    est = estimate_intent(
        agent_d=1.75, agent_v_d=0.0, agent_a_long_mps2=-3.0,
        agent_speed_mps=15.0, ego_d=1.75,
    )
    assert est.dominant == Intent.DECELERATING.value
    assert est.p_cut_in < 0.15


def test_intent_stopping_agent():
    est = estimate_intent(
        agent_d=1.75, agent_v_d=0.0, agent_a_long_mps2=-3.0,
        agent_speed_mps=2.0, ego_d=1.75,
    )
    assert est.dominant == Intent.STOPPING.value


def test_intent_gate_7_3_curve_follower_is_not_a_cut_in():
    # Agent sits on the curved route at station ~150 m, in the ego lane
    # (d = 1.75), moving exactly along the local tangent -- its Cartesian
    # velocity has a large lateral component, but its *Frenet* v_d is ~0.
    from app.services.frenet import frenet_to_local_xz

    s = 150.0
    x0, z0, dir_x, dir_z = frenet_to_local_xz(_CURVED_FRAME, s, 1.75)
    speed = 13.0
    vx, vz = speed * dir_x, speed * dir_z

    _, agent_d, _, agent_v_d = project_agent_frenet(_CURVED_FRAME, x0, z0, vx, vz)
    assert abs(agent_v_d) < 0.05, "curve follower should have ~zero Frenet lateral drift"

    est = estimate_intent_from_track(
        frame=_CURVED_FRAME, x=x0, z=z0, vx=vx, vz=vz,
        a_long_mps2=0.0, ego_d=1.75,
    )
    assert est.p_cut_in < 0.15
    assert est.dominant == Intent.LANE_KEEP.value


def test_intent_from_track_without_frame_uses_local_lateral_velocity():
    est = estimate_intent_from_track(
        frame=None, x=5.25, z=-40.0, vx=-0.4, vz=-13.0,
        a_long_mps2=0.0, ego_d=1.75,
    )
    assert est.p_cut_in > 0.70


# --------------------------------------------------------------------------
# Risk field
# --------------------------------------------------------------------------
from app.services.interfaces import AgentPrediction, PredictedState  # noqa: E402
from app.services.prediction.risk_field import RiskFieldConfig, build_risk_field  # noqa: E402


def _straight_pred(track_id, x0, z0, vx, vz, horizon_s=3.0, step_s=0.1):
    n = int(round(horizon_s / step_s))
    states = tuple(
        PredictedState(t_s=round((i + 1) * step_s, 4),
                       x=x0 + vx * (i + 1) * step_s,
                       z=z0 + vz * (i + 1) * step_s,
                       vx=vx, vz=vz)
        for i in range(n)
    )
    return AgentPrediction(track_id=track_id, states=states)


def test_risk_is_near_one_on_a_predicted_position_and_near_zero_far_away():
    pred = _straight_pred(1, x0=1.75, z0=-20.0, vx=0.0, vz=-12.0)
    field = build_risk_field([pred])
    # its own predicted point at t = 1.0 s
    st = pred.states[9]
    assert field.risk_at(st.x, st.z, 1.0) == pytest.approx(1.0, abs=1e-6)
    # 40 m to the side
    assert field.risk_at(st.x + 40.0, st.z, 1.0) < 0.01


def test_risk_footprint_widens_with_horizon_time():
    pred = _straight_pred(1, x0=1.75, z0=-20.0, vx=0.0, vz=-12.0)
    field = build_risk_field([pred])
    # Same lateral offset from the centroid, sampled early vs late.
    early = pred.states[2]   # t = 0.3
    late = pred.states[27]   # t = 2.8
    off = 2.0
    r_early = field.risk_at(early.x + off, early.z, early.t_s)
    r_late = field.risk_at(late.x + off, late.z, late.t_s)
    assert r_late > r_early


def test_risk_combines_multiple_agents_as_bounded_or():
    a = _straight_pred(1, x0=1.75, z0=-20.0, vx=0.0, vz=-12.0)
    b = _straight_pred(2, x0=1.9, z0=-20.6, vx=0.0, vz=-12.0)
    fa = build_risk_field([a])
    fab = build_risk_field([a, b])
    st = a.states[9]
    assert fab.risk_at(st.x, st.z, 1.0) >= fa.risk_at(st.x, st.z, 1.0)
    assert fab.risk_at(st.x, st.z, 1.0) <= 1.0


def test_empty_prediction_set_is_zero_risk_everywhere():
    field = build_risk_field([])
    assert field.max_risk([(0.0, 0.0, 0.0), (5.0, -30.0, 1.5)]) == 0.0


def test_max_risk_along_an_intersecting_ego_path_is_high_and_a_clear_path_is_low():
    # Agent cutting across from the right lane toward the ego lane.
    pred = _straight_pred(1, x0=5.25, z0=-30.0, vx=-1.2, vz=-11.0)
    field = build_risk_field([pred])

    # An ego path whose (x, z, t) coincides with the agent's forecast for the
    # second half of the horizon -> should read as high risk.
    hit = [(s.x, s.z, s.t_s) for s in pred.states[15:]]
    # An ego path 25 m to the left of the agent throughout -> clear.
    clear = [(s.x - 25.0, s.z, s.t_s) for s in pred.states]

    assert field.max_risk(hit) > 0.9
    assert field.max_risk(clear) < 0.05


def test_risk_field_config_growth_rates_are_honoured():
    pred = _straight_pred(1, x0=0.0, z0=0.0, vx=0.0, vz=-10.0)
    tight = build_risk_field([pred], RiskFieldConfig(k_lon_m_per_s=0.0, k_lat_m_per_s=0.0))
    loose = build_risk_field([pred], RiskFieldConfig(k_lon_m_per_s=3.0, k_lat_m_per_s=3.0))
    st = pred.states[27]  # t = 2.8
    off = 3.0
    assert loose.risk_at(st.x + off, st.z, st.t_s) > tight.risk_at(st.x + off, st.z, st.t_s)
