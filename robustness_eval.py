"""
Robustness / Out-of-Distribution (OOD) evaluation for the DDS live inference
pipeline (task P1-3).

Question this answers: when the input telemetry is nonsense -- outside
anything the classifier or anomaly detector ever saw in training -- does the
system fail safely (low classifier confidence, and/or the anomaly detector
flags it), or does it fail dangerously (a confident, wrong prediction with no
anomaly flag, which a downstream driving-decision system would act on as if
it were trustworthy)?

Methodology: per-feature train-split min/max/mean/std (see
_evidence/P1-3/train_distribution_stats.json) define "in-distribution."
Test cases are constructed to be unambiguously outside that range, then run
through the real, deployed app.services.inference.pipeline -- not a mock --
so results reflect the actual shipped behavior.

This is NOT a security/adversarial-robustness study (no gradient-based
attacks, no search for minimal adversarial perturbations) -- it's a basic
OOD sanity check appropriate for this system's scope, honestly scoped as
such rather than oversold.
"""
import sys
import json
import os
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from app.services.inference import pipeline

RAW_FEATURES = ['Altitude', 'CO2', 'Coolant', 'Litre per 100km(Instant)',
                'RPM', 'RPM_Delta', 'CO2_Delta', 'Fuel_Rate_Delta']

def get_train_distribution_stats():
    """Same held-out split as model_pipeline.py -- stats computed on TRAIN
    only, so this eval doesn't even look at the test rows."""
    df = pd.read_csv("processed_telemetry.csv")
    le = LabelEncoder()
    y = le.fit_transform(df['Driving_Decision'])
    X = df[RAW_FEATURES]
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, stratify=y, random_state=42
    )
    stats = {}
    for f in RAW_FEATURES:
        stats[f] = {
            "min": float(X_train[f].min()),
            "max": float(X_train[f].max()),
            "mean": float(X_train[f].mean()),
            "std": float(X_train[f].std()),
        }
    return stats

def run_case(name, features_dict, category):
    result = pipeline.predict(features_dict)
    conf_values = list(result["confidence_dict"].values())
    return {
        "name": name,
        "category": category,
        "input": features_dict,
        "prediction": result["prediction"],
        "confidence": round(result["confidence"], 4),
        "confidence_spread": round(max(conf_values) - sorted(conf_values)[-2], 4) if len(conf_values) > 1 else None,
        "shap_contributions": len(result["shap_result"]["contributions"]),
        "anomaly": result["anomaly_result"],
    }

def main():
    stats = get_train_distribution_stats()
    os.makedirs("_evidence/P1-3", exist_ok=True)
    with open("_evidence/P1-3/train_distribution_stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    mean_input = {f: stats[f]["mean"] for f in RAW_FEATURES}
    results = []

    # --- Category 1: single-feature extreme perturbation (others at mean) ---
    for f in RAW_FEATURES:
        rng = stats[f]["max"] - stats[f]["min"]
        for k, label in [(2, "2x_range_beyond_max"), (5, "5x_range_beyond_max"), (10, "10x_range_beyond_max")]:
            case = dict(mean_input)
            case[f] = stats[f]["max"] + k * rng
            results.append(run_case(f"{f}_{label}", case, "single_feature_extreme"))

    # --- Category 2: all features simultaneously extreme (stress test) ---
    all_extreme = {f: stats[f]["max"] + 5 * (stats[f]["max"] - stats[f]["min"]) for f in RAW_FEATURES}
    results.append(run_case("all_features_5x_beyond_max", all_extreme, "all_extreme"))

    all_extreme_negative = {f: stats[f]["min"] - 5 * (stats[f]["max"] - stats[f]["min"]) for f in RAW_FEATURES}
    results.append(run_case("all_features_5x_below_min", all_extreme_negative, "all_extreme"))

    # --- Category 3: physically implausible combinations ---
    results.append(run_case(
        "idle_rpm_huge_fuel_rate",
        {**mean_input, "RPM": 800.0, "Litre per 100km(Instant)": 80.0},
        "implausible_combo"
    ))
    results.append(run_case(
        "engine_off_but_moving_fast_co2",
        {**mean_input, "RPM": 0.0, "CO2": 1000.0, "Coolant": 20.0},
        "implausible_combo"
    ))
    results.append(run_case(
        "boiling_coolant_cold_rpm",
        {**mean_input, "Coolant": 150.0, "RPM": 800.0},
        "implausible_combo"
    ))

    # --- Category 4: all-zero edge case ---
    results.append(run_case("all_zero", {f: 0.0 for f in RAW_FEATURES}, "edge_case"))

    # --- Category 5: extreme deltas (implausible instantaneous jumps) ---
    results.append(run_case(
        "extreme_rpm_delta",
        {**mean_input, "RPM_Delta": stats["RPM_Delta"]["min"] - 10 * stats["RPM_Delta"]["std"]},
        "extreme_delta"
    ))

    # --- Category 6: missing features (pipeline should default via .get(f, 0.0)) ---
    partial_input = {"RPM": 2000.0}  # everything else missing
    results.append(run_case("missing_most_features", partial_input, "missing_features"))

    # --- Category 7: random Gaussian OOD sampling (statistical picture) ---
    rng = np.random.default_rng(42)
    ood_random_results = []
    for i in range(50):
        case = {}
        for f in RAW_FEATURES:
            # Sample far into the tail: mean +/- (15 to 30) std, random sign
            direction = rng.choice([-1, 1])
            magnitude = rng.uniform(15, 30)
            case[f] = stats[f]["mean"] + direction * magnitude * stats[f]["std"]
        r = run_case(f"random_ood_{i}", case, "random_ood_batch")
        ood_random_results.append(r)
    results.extend(ood_random_results)

    # --- Analysis ---
    flagged = sum(1 for r in ood_random_results if r["anomaly"]["is_anomaly"])
    high_conf_unflagged = sum(
        1 for r in ood_random_results
        if not r["anomaly"]["is_anomaly"] and r["confidence"] > 0.9
    )
    confidences = [r["confidence"] for r in ood_random_results]

    summary = {
        "train_distribution_stats_file": "_evidence/P1-3/train_distribution_stats.json",
        "n_random_ood_samples": len(ood_random_results),
        "random_ood_anomaly_flag_rate": flagged / len(ood_random_results),
        "random_ood_dangerous_rate_high_conf_unflagged": high_conf_unflagged / len(ood_random_results),
        "random_ood_confidence_mean": float(np.mean(confidences)),
        "random_ood_confidence_median": float(np.median(confidences)),
        "random_ood_confidence_min": float(np.min(confidences)),
        "random_ood_confidence_max": float(np.max(confidences)),
        "all_cases": results,
    }

    with open("_evidence/P1-3/robustness_results.json", "w") as f:
        json.dump(summary, f, indent=2)

    print(f"Random OOD batch (n={len(ood_random_results)}):")
    print(f"  Anomaly-flagged: {flagged}/{len(ood_random_results)} ({flagged/len(ood_random_results)*100:.1f}%)")
    print(f"  Dangerous (high-confidence, NOT flagged): {high_conf_unflagged}/{len(ood_random_results)} "
          f"({high_conf_unflagged/len(ood_random_results)*100:.1f}%)")
    print(f"  Confidence: mean={np.mean(confidences):.3f}, median={np.median(confidences):.3f}, "
          f"min={np.min(confidences):.3f}, max={np.max(confidences):.3f}")
    print(f"\nFull results: _evidence/P1-3/robustness_results.json")

    return summary

if __name__ == "__main__":
    main()
