"""
Unit tests for app/services/anomaly_detector.py.

Uses the real trained anomaly_model.pkl / anomaly_feature_bounds.json
artifacts checked into the repo -- these tests exercise the actual
deployed detector, not a mock, matching P1-3's real-pipeline-not-mock
approach to robustness testing.
"""
import pytest
from app.services.anomaly_detector import AnomalyDetector


@pytest.fixture(scope="module")
def detector():
    d = AnomalyDetector()
    assert d.model is not None, "anomaly_model.pkl must be present for these tests"
    assert d.feature_bounds is not None, "anomaly_feature_bounds.json must be present for these tests"
    return d


def _mid_range_reading(detector):
    """A reading with every feature at the midpoint of its observed train range.

    Note this is NOT necessarily a jointly-typical reading -- e.g. delta
    features cluster near 0 in real data, so their range midpoint can itself
    look unusual to the Isolation Forest. Use this only where the test cares
    about the univariate range-bounds logic, not "is this normal driving."""
    return {
        f: (b["min"] + b["max"]) / 2.0
        for f, b in detector.feature_bounds.items()
    }


# Realistic, jointly-typical reading: column means from processed_telemetry.csv
# (the same data the detector/classifier were trained on), not range midpoints.
# Matches AnomalyDetector's actual feature set (self.features, driven by the
# trained model's feature_names_in_) -- NOT the classifier's optimal_features,
# which is a different subset (see inference.py's comment on why the two
# feature sets are independent).
_TYPICAL_READING = {
    "Altitude": 162.5,
    "RPM": 2104.8,
    "Coolant": 81.1,
    "Litre per 100km(Instant)": 7.8,
    "RPM_Delta": 2.4,
    "CO2_Delta": 0.3,
    "Fuel_Rate_Delta": 0.01,
}


def test_normal_reading_is_not_flagged(detector):
    result = detector.detect(dict(_TYPICAL_READING))
    assert result["is_anomaly"] is False
    assert result["type"] == "NONE"


def test_single_feature_far_out_of_range_is_caught(detector):
    """P1-3's core finding: Isolation Forest alone missed single-feature
    extremes. The hard range check (added in P1-3) must catch this."""
    reading = dict(_TYPICAL_READING)
    feat = "Coolant"
    bounds = detector.feature_bounds[feat]
    rng = bounds["max"] - bounds["min"]
    reading[feat] = bounds["max"] + rng * 5  # 5x the observed range beyond max

    result = detector.detect(reading)
    assert result["is_anomaly"] is True
    assert result["type"] == "OUT_OF_RANGE"
    assert result["severity"] == "HIGH"


def test_slightly_beyond_bounds_is_not_a_hard_violation(detector):
    """range_margin=1.0 means a value has to be more than 1x the observed
    range beyond min/max to trigger OUT_OF_RANGE -- a value just past the
    boundary should not immediately trip it."""
    reading = _mid_range_reading(detector)
    feat = "RPM"
    bounds = detector.feature_bounds[feat]
    reading[feat] = bounds["max"] + 1.0  # trivially past max, not 1x-range past it

    result = detector.detect(reading)
    assert result["type"] != "OUT_OF_RANGE"


def test_missing_features_return_feature_mismatch(detector):
    result = detector.detect({"RPM": 2000.0})
    assert result["is_anomaly"] is False
    assert result["type"] == "FEATURE_MISMATCH"


def test_detect_accepts_dict_list_and_dataframe_input(detector):
    import pandas as pd

    reading = _mid_range_reading(detector)
    as_dict = detector.detect(reading)
    as_list = detector.detect([reading[f] for f in detector.features])
    as_df = detector.detect(pd.DataFrame([reading]))

    assert as_dict["type"] == as_list["type"] == as_df["type"]


def test_unloaded_model_reports_not_loaded():
    d = AnomalyDetector(model_path="nonexistent_path_for_test.pkl")
    result = d.detect({"RPM": 2000.0})
    assert result["is_anomaly"] is False
    assert result["message"] == "Model not loaded"
