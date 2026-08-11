"""
Unit tests for app/services/inference.py, against the real trained
artifacts checked into the repo (best_model.pkl, scaler.pkl,
label_encoder.pkl, optimal_features.json, anomaly_model.pkl).
"""
import pytest
from app.services.inference import InferencePipeline


@pytest.fixture(scope="module")
def pipeline():
    p = InferencePipeline()
    assert p.is_ready(), f"pipeline not ready: {p.get_errors()}"
    return p


# Realistic, jointly-typical reading: column means from processed_telemetry.csv
# (the same data the classifier/anomaly detector were trained on). Using
# per-feature range midpoints instead would produce a reading that's
# univariate-plausible per feature but jointly atypical (e.g. delta features
# cluster near 0 in real data, not at their range midpoint).
_TYPICAL_READING = {
    "RPM": 2104.8,
    "Coolant": 81.1,
    "CO2": 188.9,
    "Litre per 100km(Instant)": 7.8,
    "RPM_Delta": 2.4,
    "CO2_Delta": 0.3,
    "Fuel_Rate_Delta": 0.01,
    "Altitude": 162.5,
}


def _plausible_input(pipeline):
    return dict(_TYPICAL_READING)


def test_predict_returns_a_known_class_label(pipeline):
    result = pipeline.predict(_plausible_input(pipeline))
    assert result["prediction"] in {"Accelerate", "Decelerate", "Maintain Speed"}


def test_predict_confidence_dict_sums_to_one(pipeline):
    result = pipeline.predict(_plausible_input(pipeline))
    total = sum(result["confidence_dict"].values())
    assert total == pytest.approx(1.0, abs=1e-4)


def test_predict_confidence_matches_max_of_confidence_dict(pipeline):
    result = pipeline.predict(_plausible_input(pipeline))
    assert result["confidence"] == pytest.approx(max(result["confidence_dict"].values()))


def test_predict_missing_fields_flagged_incomplete_input_not_silently_defaulted(pipeline):
    """P1-3 finding: missing telemetry used to silently default to 0.0 with
    no anomaly flag. It must now surface as INCOMPLETE_INPUT."""
    result = pipeline.predict({})
    assert result["anomaly_result"]["type"] == "INCOMPLETE_INPUT"
    assert result["anomaly_result"]["is_anomaly"] is True


def test_predict_extreme_single_feature_is_caught_by_anomaly_detector(pipeline):
    reading = _plausible_input(pipeline)
    bounds = pipeline.anomaly_detector.feature_bounds["Coolant"]
    rng = bounds["max"] - bounds["min"]
    reading["Coolant"] = bounds["max"] + rng * 5
    result = pipeline.predict(reading)
    assert result["anomaly_result"]["is_anomaly"] is True
    assert result["anomaly_result"]["type"] == "OUT_OF_RANGE"


def test_predict_clean_input_only_contains_classifier_features(pipeline):
    result = pipeline.predict(_plausible_input(pipeline))
    assert set(result["clean_input"].keys()) == set(pipeline.features)


def test_unready_pipeline_raises_on_predict():
    p = InferencePipeline.__new__(InferencePipeline)
    p.model = None
    p.scaler = None
    p.label_encoder = None
    p.features = []
    with pytest.raises(RuntimeError):
        p.predict({})
