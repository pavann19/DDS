"""
Exhaustive Feature Subset Search for DDS Optimization.

Why exhaustive search instead of a Genetic Algorithm?
- With N features (typically 7-8), there are only 2^N = 128-256 possible subsets.
- Exhaustive search evaluates ALL of them and guarantees the global optimum.
- A GA with population=20 and 5 generations would only sample ~100 combinations
  and might miss the best one. For this problem size, brute force IS the best algorithm.
- This approach is honest, deterministic, and reproducible.
(Filename kept as genetic_optimizer.py for historical/doc continuity; the
algorithm itself is exhaustive search, not a GA -- see above.)

Leakage note (fixed as part of a previous fix): this search used to run its
cross-validation on the FULL dataset, including rows that model_pipeline.py
later held out as its test split -- so the feature *selection* was
optimistically biased by data the final model's reported test accuracy was
supposed to be blind to. This version splits off the identical held-out test
set FIRST (same test_size/random_state/stratify as model_pipeline.py, which
produces the exact same row split since sklearn's train_test_split depends
only on sample count and the y labels, not which X columns are passed) and
searches using only the train partition.
"""

import numpy as np
import pandas as pd
import json
import itertools
from sklearn.model_selection import StratifiedKFold, cross_val_score, train_test_split
from sklearn.preprocessing import StandardScaler, LabelEncoder
from xgboost import XGBClassifier
from sklearn.ensemble import RandomForestClassifier

def load_data(data_file="processed_telemetry.csv", feature_file="feature_info.json", test_size=0.2):
    """Load dataset, feature info, and hold out the SAME test split model_pipeline.py uses."""
    df = pd.read_csv(data_file)

    with open(feature_file, "r") as f:
        info = json.load(f)

    features = info["features"]
    X = df[features].values

    le = LabelEncoder()
    y = le.fit_transform(df['Driving_Decision'])

    # Hold out the identical test split model_pipeline.py uses (same
    # test_size/random_state/stratify -> identical row indices, since the
    # split depends only on sample count + y, not which columns are in X).
    # Feature selection below only ever sees X_train/y_train.
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=test_size, stratify=y, random_state=42
    )

    return X_train, y_train, features, len(X_test)

def evaluate_subset(X, y, feature_indices, cv):
    """Evaluate a feature subset using cross-validated accuracy (train split only)."""
    if len(feature_indices) == 0:
        return 0.0

    X_subset = X[:, list(feature_indices)]
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X_subset)

    # Use RandomForest as evaluator (fast, handles mixed features well)
    clf = RandomForestClassifier(
        n_estimators=100, max_depth=10,
        class_weight='balanced', random_state=42
    )
    scores = cross_val_score(clf, X_scaled, y, cv=cv, scoring='accuracy')
    return scores.mean()

def run_exhaustive_search():
    """
    Exhaustively evaluate all 2^N feature subsets (on the train split only)
    to find the optimal one without leaking the held-out test set.
    """
    print("Loading data...")
    X, y, features, held_out_count = load_data()
    n_features = len(features)
    total_subsets = 2**n_features - 1  # Exclude empty set

    print(f"Features ({n_features}): {features}")
    print(f"Total non-empty subsets to evaluate: {total_subsets}")
    print(f"Dataset: {len(y)} train samples ({held_out_count} held out as test, never seen here)")
    print(f"\nStarting exhaustive search...\n")

    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)

    best_score = 0.0
    best_indices = None
    results = []

    for size in range(1, n_features + 1):
        print(f"--- Evaluating subsets of size {size} ({len(list(itertools.combinations(range(n_features), size)))} combinations) ---")

        for combo in itertools.combinations(range(n_features), size):
            score = evaluate_subset(X, y, combo, cv)
            selected = [features[i] for i in combo]
            results.append({
                "features": selected,
                "indices": list(combo),
                "accuracy": float(score),
                "n_features": size
            })

            if score > best_score:
                best_score = score
                best_indices = combo
                print(f"  NEW BEST: {selected} -> {score:.4f}")

    # Sort all results by accuracy
    results.sort(key=lambda x: x["accuracy"], reverse=True)

    # Final report
    best_features = [features[i] for i in best_indices]
    print(f"\n{'='*60}")
    print(f"EXHAUSTIVE SEARCH COMPLETE (train split only, test split never touched)")
    print(f"{'='*60}")
    print(f"Total subsets evaluated: {total_subsets}")
    print(f"Optimal feature subset ({len(best_features)} features):")
    for f in best_features:
        print(f"  - {f}")
    print(f"Cross-validated accuracy (train split): {best_score:.4f}")

    # Top 5 subsets
    print(f"\nTop 5 feature subsets:")
    for i, r in enumerate(results[:5]):
        print(f"  {i+1}. {r['features']} -> {r['accuracy']:.4f}")

    # Save optimal features for model_pipeline.py to consume
    output = {
        "selected_features": best_features,
        "feature_indices": list(best_indices),
        "accuracy": float(best_score),
        "methodology": (
            "Exhaustive search over all non-empty feature subsets, evaluated by "
            "5-fold stratified CV on the TRAIN split only -- the same held-out "
            "test split model_pipeline.py uses (test_size=0.2, random_state=42, "
            "stratify=y) is excluded from this search entirely. Fixes a prior "
            "leakage bug (a previous fix) where this search ran on the full dataset "
            "including what later became the test split."
        ),
        "all_features": features,
        "total_subsets_evaluated": total_subsets,
        "top_10_subsets": results[:10]
    }
    with open("optimal_features.json", "w") as f:
        json.dump(output, f, indent=2)

    print(f"\nOptimal features saved to optimal_features.json")
    print(f"Run model_pipeline.py next to train with optimal features.")

if __name__ == "__main__":
    run_exhaustive_search()
