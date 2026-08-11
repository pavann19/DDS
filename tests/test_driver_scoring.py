"""Unit tests for app/services/driver_scoring.py."""
from app.services.driver_scoring import DriverScorer


def _reading(fuel_rate=5.0, anomaly=None, confidence=0.9):
    return {
        "telemetry": {"Litre per 100km(Instant)": fuel_rate},
        "anomaly": anomaly or {"is_anomaly": False},
        "confidence": confidence,
    }


def test_empty_history_scores_perfect():
    scorer = DriverScorer()
    result = scorer.calculate_score()
    assert result["score"] == 100
    assert result["rating"] == "A+"


def test_steady_driving_keeps_high_score():
    scorer = DriverScorer()
    for _ in range(10):
        scorer.add_reading({"Litre per 100km(Instant)": 5.0}, "Maintain Speed", 0.95, {"is_anomaly": False})
    result = scorer.calculate_score()
    assert result["score"] >= 90
    assert result["rating"] == "A+"


def test_frequent_action_switching_penalizes_smoothness():
    scorer = DriverScorer()
    actions = ["Accelerate", "Decelerate"] * 10
    for action in actions:
        scorer.add_reading({"Litre per 100km(Instant)": 5.0}, action, 0.95, {"is_anomaly": False})
    result = scorer.calculate_score()
    assert result["score"] < 100
    assert result["breakdown"]["smoothness"] < 100


def test_high_fuel_consumption_penalizes_efficiency():
    scorer = DriverScorer()
    for _ in range(10):
        scorer.add_reading({"Litre per 100km(Instant)": 20.0}, "Maintain Speed", 0.95, {"is_anomaly": False})
    result = scorer.calculate_score()
    assert result["breakdown"]["efficiency"] < 100


def test_high_severity_anomalies_penalize_safety_heavily():
    scorer = DriverScorer()
    for _ in range(5):
        scorer.add_reading(
            {"Litre per 100km(Instant)": 5.0}, "Maintain Speed", 0.95,
            {"is_anomaly": True, "severity": "HIGH"},
        )
    result = scorer.calculate_score()
    assert result["breakdown"]["safety"] < 100
    assert result["score"] < 100


def test_score_is_bounded_zero_to_hundred():
    scorer = DriverScorer()
    for _ in range(60):
        scorer.add_reading(
            {"Litre per 100km(Instant)": 30.0}, "Accelerate", 0.1,
            {"is_anomaly": True, "severity": "HIGH"},
        )
    result = scorer.calculate_score()
    assert 0 <= result["score"] <= 100


def test_window_size_caps_history_length():
    scorer = DriverScorer(window_size=5)
    for i in range(20):
        scorer.add_reading({"Litre per 100km(Instant)": 5.0}, "Maintain Speed", 0.9, {"is_anomaly": False})
    assert len(scorer.history) == 5


def test_rating_bands_match_score_thresholds():
    scorer = DriverScorer()
    result = scorer.calculate_score()
    score = result["score"]
    rating = result["rating"]
    if score >= 90:
        assert rating == "A+"
    elif score >= 80:
        assert rating == "A"
    elif score >= 70:
        assert rating == "B"
    elif score >= 60:
        assert rating == "C"
    elif score >= 50:
        assert rating == "D"
    else:
        assert rating == "F"
