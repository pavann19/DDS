"""
Exhaustive Feature Subset Search for DDS Optimization.

Why exhaustive search instead of a Genetic Algorithm?
- With N features (typically 7-8), there are only 2^N = 128-256 possible subsets.
- Exhaustive search evaluates ALL of them and guarantees the global optimum.
- A GA with population=20 and 5 generations would only sample ~100 combinations
  and might miss the best one. For this problem size, brute force IS the best algorithm.
- This approach is honest, deterministic, and reproducible.
"""

import numpy as np
import pandas as pd
import json
import itertools
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.preprocessing import StandardScaler, LabelEncoder
from xgboost import XGBClassifier
from sklearn.ensemble import RandomForestClassifier

def load_data(data_file="processed_telemetry.csv", feature_file="feature_info.json"):
    """Load dataset and feature info. NOT called at module level."""
    df = pd.read_csv(data_file)
    
    with open(feature_file, "r") as f:
        info = json.load(f)
    
    features = info["features"]
    X = df[features].values
    
    le = LabelEncoder()
    y = le.fit_transform(df['Driving_Decision'])
    
    return X, y, features

def evaluate_subset(X, y, feature_indices, cv):
    """Evaluate a feature subset using cross-validated accuracy."""
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
    Exhaustively evaluate all 2^N feature subsets to find the optimal one.
    Guarantees finding the global optimum.
    """
    print("Loading data...")
    X, y, features = load_data()
    n_features = len(features)
    total_subsets = 2**n_features - 1  # Exclude empty set
    
    print(f"Features ({n_features}): {features}")
    print(f"Total non-empty subsets to evaluate: {total_subsets}")
    print(f"Dataset: {len(y)} samples")
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
    print(f"EXHAUSTIVE SEARCH COMPLETE")
    print(f"{'='*60}")
    print(f"Total subsets evaluated: {total_subsets}")
    print(f"Optimal feature subset ({len(best_features)} features):")
    for f in best_features:
        print(f"  - {f}")
    print(f"Cross-validated accuracy: {best_score:.4f}")
    
    # Top 5 subsets
    print(f"\nTop 5 feature subsets:")
    for i, r in enumerate(results[:5]):
        print(f"  {i+1}. {r['features']} -> {r['accuracy']:.4f}")
    
    # Save optimal features for model_pipeline.py to consume
    output = {
        "selected_features": best_features,
        "feature_indices": list(best_indices),
        "accuracy": float(best_score),
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
