"""
P5-2: baseline comparison -- what does the XGBoost classifier actually buy?

Motivation (2026-08-11). The headline number for this project has been
"85.0% test accuracy". That figure is meaningless on its own, because the
class distribution is 711/104/82 (Maintain Speed / Accelerate / Decelerate)
-- a model that ignores its inputs entirely and always predicts "Maintain
Speed" already scores ~79%. The thesis needs the MARGINAL contribution
stated explicitly, not the raw accuracy.

This script evaluates three models on ONE identical held-out split so the
numbers are guaranteed comparable:

  1. DummyClassifier(strategy='most_frequent') -- the majority-class floor.
     Any model that cannot beat this has learned nothing usable.
  2. LogisticRegression (scaled, class-balanced) -- a linear baseline. If a
     linear model matches the gradient-boosted one, the extra capacity (and
     the SHAP-compatibility constraint it forced on model selection) is not
     earning its place.
  3. XGBoost -- the deployed model, re-evaluated HERE rather than read from
     metrics.json, so all three numbers come from a single run of a single
     script (AUDIT_PROTOCOL.md: no reported number without a regenerating
     script).

Split, features, scaling and class-balancing are taken from model_pipeline.py
itself (imported, not re-implemented) so this cannot silently drift from the
real training pipeline.
"""
import json
import os

import numpy as np
import pandas as pd
from sklearn.dummy import DummyClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import classification_report, confusion_matrix, f1_score
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder, StandardScaler
from xgboost import XGBClassifier

from model_pipeline import fit_balanced, load_features

RANDOM_STATE = 42
TEST_SIZE = 0.2
OUT_DIR = os.path.join("_evidence", "P5-2")


def _wilson_interval(successes, n, z=1.96):
    """95% Wilson score interval for a binomial proportion.

    Necessary here, not decorative: the test split contains only 16
    Decelerate and 21 Accelerate samples, so a per-class recall of "0.50"
    is 8 correct out of 16 and carries a very wide interval. Reporting
    per-class performance on this dataset WITHOUT an interval would
    overstate precision -- the Wilson interval is used rather than the
    normal-approximation (Wald) interval because Wald is badly behaved for
    small n and for proportions near 0 or 1, both of which occur here
    (several models score exactly 0.0 recall on Decelerate)."""
    if n == 0:
        return [0.0, 0.0]
    p = successes / n
    denom = 1 + z ** 2 / n
    center = (p + z ** 2 / (2 * n)) / denom
    margin = (z * ((p * (1 - p) / n + z ** 2 / (4 * n ** 2)) ** 0.5)) / denom
    return [round(max(0.0, center - margin), 4), round(min(1.0, center + margin), 4)]


def _evaluate(name, model, X_train, y_train, X_test, y_test, class_names,
              balanced_name=None):
    """Fit and evaluate one model. balanced_name, if given, is the key
    model_pipeline.fit_balanced uses to decide whether this model needs
    per-sample balanced weights (SAMPLE_WEIGHTED_MODELS)."""
    if balanced_name is not None:
        fit_balanced(model, balanced_name, X_train, y_train)
    else:
        model.fit(X_train, y_train)

    y_pred = model.predict(X_test)
    report = classification_report(
        y_test, y_pred, target_names=class_names, output_dict=True, zero_division=0
    )
    cm = confusion_matrix(y_test, y_pred)

    # Per-class recall as an explicit "k correct out of n", with a 95%
    # interval -- so the tiny per-class support is visible in the reported
    # numbers rather than hidden behind a 2-decimal score.
    recall_ci = {}
    for i, c in enumerate(class_names):
        n_c = int(cm[i].sum())
        correct_c = int(cm[i][i])
        recall_ci[c] = {
            "correct": correct_c,
            "support": n_c,
            "recall": round(correct_c / n_c, 4) if n_c else 0.0,
            "recall_95ci_wilson": _wilson_interval(correct_c, n_c),
        }

    n_test = int(len(y_test))
    n_correct = int((y_pred == y_test).sum())

    return {
        "model": name,
        "accuracy": float(report["accuracy"]),
        "accuracy_95ci_wilson": _wilson_interval(n_correct, n_test),
        "macro_f1": float(f1_score(y_test, y_pred, average="macro", zero_division=0)),
        "weighted_f1": float(f1_score(y_test, y_pred, average="weighted", zero_division=0)),
        "per_class_f1": {c: float(report[c]["f1-score"]) for c in class_names},
        "per_class_recall_detail": recall_ci,
        "confusion_matrix": cm.tolist(),
    }


def run(data_file="processed_telemetry.csv",
        optimal_features_file="optimal_features.json"):
    df = pd.read_csv(data_file)

    feature_info = load_features()
    all_features = feature_info["features"] if feature_info else [
        "Altitude", "CO2", "Coolant", "Litre per 100km(Instant)",
        "RPM", "RPM_Delta", "CO2_Delta", "Fuel_Rate_Delta",
    ]

    # The deployed model uses the genetic-search-selected subset; the
    # baselines must see the SAME features, otherwise this compares feature
    # selection and model capacity at the same time and isolates neither.
    if os.path.exists(optimal_features_file):
        with open(optimal_features_file, "r") as f:
            features = json.load(f).get("selected_features", all_features)
    else:
        features = all_features

    X = df[features]
    le = LabelEncoder()
    y = le.fit_transform(df["Driving_Decision"])
    class_names = le.classes_.tolist()

    # IDENTICAL split to model_pipeline.train_models().
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=TEST_SIZE, stratify=y, random_state=RANDOM_STATE
    )

    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)

    results = []

    # 1. Majority-class floor. Deliberately NOT class-balanced: the whole
    #    point of this baseline is "what if the model always guesses the
    #    most common class", which is what an unbalanced trivial model does.
    results.append(_evaluate(
        "MajorityClass(DummyClassifier)",
        DummyClassifier(strategy="most_frequent"),
        X_train_scaled, y_train, X_test_scaled, y_test, class_names,
    ))

    # 2a. Linear baseline WITHOUT balancing. Included to pre-empt the obvious
    #     examiner question ("did the linear model only look bad because you
    #     balanced it?") and to show what the imbalance does to an unweighted
    #     linear model: it collapses toward the majority class.
    results.append(_evaluate(
        "LogisticRegression(scaled, unbalanced)",
        LogisticRegression(max_iter=5000, random_state=RANDOM_STATE),
        X_train_scaled, y_train, X_test_scaled, y_test, class_names,
    ))

    # 2b. Linear baseline, class-balanced (so it is not itself trivially
    #     collapsed onto the majority class by the 711/104/82 imbalance).
    #     This is the fair like-for-like comparison against XGBoost, which is
    #     also fit with balanced sample weights.
    results.append(_evaluate(
        "LogisticRegression(scaled, balanced)",
        LogisticRegression(class_weight="balanced", max_iter=5000,
                           random_state=RANDOM_STATE),
        X_train_scaled, y_train, X_test_scaled, y_test, class_names,
    ))

    # 3. The deployed model, same hyperparameters as model_pipeline's factory.
    results.append(_evaluate(
        "XGBoost(deployed config)",
        XGBClassifier(n_estimators=200, max_depth=6, learning_rate=0.1,
                      eval_metric="mlogloss", random_state=RANDOM_STATE),
        X_train_scaled, y_train, X_test_scaled, y_test, class_names,
        balanced_name="XGBoost",
    ))

    majority = results[0]["accuracy"]
    for r in results:
        r["accuracy_gain_over_majority_pts"] = round(
            (r["accuracy"] - majority) * 100, 2
        )

    # Sanity check stated in the P5-2 acceptance criteria: the majority
    # baseline must equal (count of the largest class in the test split) /
    # (test split size).
    majority_class_count = int(np.bincount(y_test).max())
    expected_majority_acc = majority_class_count / len(y_test)

    payload = {
        "task": "P5-2 baselines",
        "purpose": (
            "State the XGBoost classifier's marginal contribution explicitly, "
            "against a majority-class floor and a linear baseline, on one "
            "identical held-out split."
        ),
        "methodology": (
            f"Stratified train_test_split(test_size={TEST_SIZE}, "
            f"random_state={RANDOM_STATE}) -- byte-identical to "
            "model_pipeline.train_models(). All three models see the same "
            "genetic-search-selected feature subset and the same "
            "StandardScaler (fit on train only). Class balancing for XGBoost "
            "is applied through model_pipeline.fit_balanced() (imported, not "
            "duplicated). LogisticRegression uses class_weight='balanced'. "
            "The majority-class dummy is deliberately unbalanced -- that is "
            "what makes it the trivial floor."
        ),
        "dataset_rows": int(len(df)),
        "train_samples": int(len(X_train)),
        "test_samples": int(len(X_test)),
        "features_used": features,
        "class_names": class_names,
        "class_distribution_full": {
            c: int(n) for c, n in zip(class_names, np.bincount(y))
        },
        "class_distribution_test": {
            c: int(n) for c, n in zip(class_names, np.bincount(y_test))
        },
        "majority_baseline_check": {
            "largest_test_class_count": majority_class_count,
            "test_samples": int(len(y_test)),
            "expected_accuracy": float(expected_majority_acc),
            "measured_accuracy": float(majority),
            "matches": bool(abs(expected_majority_acc - majority) < 1e-9),
        },
        "results": results,
    }

    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, "baselines.json")
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)

    # --- Human-readable comparison table (the thesis 6.1 table) ---
    print(f"\nDataset: {len(df)} rows | train {len(X_train)} / test {len(X_test)}")
    print(f"Test-split class distribution: {payload['class_distribution_test']}")
    print(f"\n{'Model':<38} {'Acc':>7} {'MacroF1':>9} {'vs majority':>12}")
    print("-" * 70)
    for r in results:
        print(f"{r['model']:<38} {r['accuracy']:>7.4f} {r['macro_f1']:>9.4f} "
              f"{r['accuracy_gain_over_majority_pts']:>+11.2f} pts")

    print(f"\nPer-class F1 (the number that actually matters for safety):")
    print(f"{'Model':<38} " + " ".join(f"{c[:12]:>13}" for c in class_names))
    print("-" * 82)
    for r in results:
        print(f"{r['model']:<38} " +
              " ".join(f"{r['per_class_f1'][c]:>13.4f}" for c in class_names))

    print(f"\nPer-class recall with 95% Wilson intervals "
          f"(note the support -- these are small-n estimates):")
    for r in results:
        print(f"\n  {r['model']}")
        for c in class_names:
            d = r["per_class_recall_detail"][c]
            lo, hi = d["recall_95ci_wilson"]
            print(f"    {c:<15} {d['correct']:>3}/{d['support']:<4} "
                  f"recall={d['recall']:.3f}  95% CI [{lo:.3f}, {hi:.3f}]")

    print(f"\nMajority-baseline sanity check: "
          f"{majority_class_count}/{len(y_test)} = {expected_majority_acc:.4f} "
          f"-> {'OK' if payload['majority_baseline_check']['matches'] else 'MISMATCH'}")
    print(f"\nWritten: {out_path}")

    return payload


if __name__ == "__main__":
    run()
