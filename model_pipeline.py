import pandas as pd
import numpy as np
from sklearn.model_selection import StratifiedKFold, cross_val_score, train_test_split
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.neural_network import MLPClassifier
from xgboost import XGBClassifier
from sklearn.metrics import classification_report, confusion_matrix
import joblib
import json
import os

def load_features(feature_file="feature_info.json"):
    """Load the feature list saved by data_prep.py."""
    if os.path.exists(feature_file):
        with open(feature_file, "r") as f:
            return json.load(f)
    return None

def train_models(data_file="processed_telemetry.csv", model_dir=".",
                 optimal_features_file="optimal_features.json", test_size=0.2):
    """
    Trains multiple ML models with a genuine held-out test evaluation.

    Methodology (fixes a prior bug where the reported metrics were computed
    on the training set itself, giving a meaningless 100% accuracy):
    1. Split off a stratified held-out test set FIRST, before any model
       selection or fitting touches the data.
    2. Model selection (5-fold CV) runs on the TRAIN split only.
    3. The selected model type is fit on TRAIN and evaluated ONCE on the
       untouched TEST split -- this is the reported, honest metric.
    4. For the artifact actually shipped for live inference, the same
       model type is refit on the FULL dataset (train+test) to use all
       available data -- standard practice, but it means best_model.pkl's
       expected generalization is represented by the step-3 test metrics,
       not by re-evaluating it on its own training data.

    Known limitation (disclosed, not fixed here): the feature subset in
    optimal_features.json was selected via exhaustive search
    (genetic_optimizer.py) using cross-validation on the FULL dataset,
    including rows that are now in this script's test split. That makes
    the feature choice itself optimistically biased by test data, even
    though this script's own model fitting/evaluation split is clean.
    A fully leak-free pipeline would rerun feature selection on the train
    split only -- tracked as a follow-up in STATE.md, not hidden.
    """
    df = pd.read_csv(data_file)

    feature_info = load_features()
    all_features = feature_info["features"] if feature_info else [
        'Altitude', 'CO2', 'Coolant', 'Litre per 100km(Instant)',
        'RPM', 'RPM_Delta', 'CO2_Delta', 'Fuel_Rate_Delta'
    ]

    if os.path.exists(optimal_features_file):
        with open(optimal_features_file, "r") as f:
            opt = json.load(f)
        features = opt.get("selected_features", all_features)
        print(f"Using optimal feature subset from exhaustive search: {features}")
    else:
        features = all_features
        print(f"Using all features: {features}")

    X = df[features]
    le = LabelEncoder()
    y = le.fit_transform(df['Driving_Decision'])
    class_names = le.classes_.tolist()

    print(f"\nDataset: {len(X)} samples, {len(features)} features")
    print(f"Classes: {class_names}")
    print(f"Class distribution: {dict(zip(class_names, np.bincount(y)))}")

    # --- HELD-OUT TEST SPLIT (before anything else touches the data) ---
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=test_size, stratify=y, random_state=42
    )
    print(f"\nTrain: {len(X_train)} samples | Held-out test: {len(X_test)} samples")

    # --- SCALING (fit on train only, applied to test) ---
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)

    # --- MODEL DEFINITIONS ---
    models = {
        "RandomForest": RandomForestClassifier(
            n_estimators=200, max_depth=15, min_samples_split=5,
            class_weight='balanced', random_state=42
        ),
        "GradientBoosting": GradientBoostingClassifier(
            n_estimators=200, max_depth=5, learning_rate=0.1,
            random_state=42
        ),
        "MLP": MLPClassifier(
            hidden_layer_sizes=(128, 64, 32), max_iter=1000,
            early_stopping=True, validation_fraction=0.15,
            random_state=42
        ),
        "XGBoost": XGBClassifier(
            n_estimators=200, max_depth=6, learning_rate=0.1,
            eval_metric='mlogloss', random_state=42
        )
    }

    # --- STRATIFIED CROSS-VALIDATION ON TRAIN ONLY (model selection) ---
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)

    print("\n" + "="*60)
    print("STRATIFIED 5-FOLD CROSS-VALIDATION RESULTS (train split only)")
    print("="*60)

    cv_results = {}
    for name, model in models.items():
        scores = cross_val_score(model, X_train_scaled, y_train, cv=cv, scoring='accuracy')
        cv_results[name] = {
            "mean_accuracy": float(scores.mean()),
            "std_accuracy": float(scores.std()),
            "fold_scores": scores.tolist()
        }
        print(f"\n{name}:")
        print(f"  Accuracy: {scores.mean():.4f} +/- {scores.std():.4f}")
        print(f"  Folds: {[f'{s:.4f}' for s in scores]}")

    best_name = max(cv_results, key=lambda k: cv_results[k]["mean_accuracy"])
    print(f"\n{'='*60}")
    print(f"BEST MODEL (by train-split CV): {best_name} (CV Accuracy: {cv_results[best_name]['mean_accuracy']:.4f})")
    print(f"{'='*60}")

    # --- FIT ON TRAIN, EVALUATE ON HELD-OUT TEST (the honest metric) ---
    eval_model = models[best_name]
    eval_model.fit(X_train_scaled, y_train)
    y_test_pred = eval_model.predict(X_test_scaled)

    print(f"\nHeld-out TEST Classification Report ({best_name}):")
    test_report = classification_report(y_test, y_test_pred, target_names=class_names, output_dict=True)
    print(classification_report(y_test, y_test_pred, target_names=class_names))

    test_cm = confusion_matrix(y_test, y_test_pred)
    print("Held-out TEST Confusion Matrix:")
    print(f"  Classes: {class_names}")
    for i, row in enumerate(test_cm):
        print(f"  {class_names[i]:>15}: {row}")

    # --- REFIT ON FULL DATA FOR THE DEPLOYED ARTIFACT ---
    # Once the model type is chosen and honestly evaluated above, refit on
    # all available data to ship the best-informed model. Its expected
    # real-world accuracy is the TEST metrics above, not whatever it would
    # score if evaluated on its own training data.
    full_scaler = StandardScaler()
    X_full_scaled = full_scaler.fit_transform(X)
    deploy_model = models[best_name].__class__(**models[best_name].get_params())
    deploy_model.fit(X_full_scaled, y)

    # --- FEATURE IMPORTANCE (from the deployed model) ---
    importance_dict = {}
    if hasattr(deploy_model, 'feature_importances_'):
        importances = deploy_model.feature_importances_
        sorted_idx = np.argsort(importances)[::-1]
        print(f"\nFeature Importance ({best_name}, full-data fit):")
        for idx in sorted_idx:
            print(f"  {features[idx]:>30}: {importances[idx]:.4f}")
            importance_dict[features[idx]] = float(importances[idx])

    # --- SAVE DEPLOYMENT ARTIFACTS ---
    joblib.dump(deploy_model, f"{model_dir}/best_model.pkl")
    joblib.dump(full_scaler, f"{model_dir}/scaler.pkl")
    joblib.dump(le, f"{model_dir}/label_encoder.pkl")

    metrics = {
        "best_model": best_name,
        "methodology": (
            "Model selection via 5-fold stratified CV on an 80% train split. "
            "Reported test_* metrics are from ONE evaluation on a 20% held-out "
            "stratified split the model never trained on. The deployed "
            "best_model.pkl is refit on 100% of the data (standard practice for "
            "shipping) -- its expected real-world performance is represented by "
            "the test_* metrics below, not by evaluating it on its own training "
            "data."
        ),
        "known_limitation": (
            "Feature selection (optimal_features.json, genetic_optimizer.py) "
            "used cross-validation on the full dataset, including rows that are "
            "now in this script's test split -- the feature subset choice "
            "itself is optimistically biased by test data, even though this "
            "script's model fitting/evaluation split is clean. Not yet fixed; "
            "see STATE.md."
        ),
        "test_split_size": test_size,
        "train_samples": int(len(X_train)),
        "test_samples": int(len(X_test)),
        "cv_results_on_train_split": cv_results,
        "features_used": features,
        "class_names": class_names,
        "test_classification_report": {k: v for k, v in test_report.items() if k != 'accuracy'},
        "test_overall_accuracy": float(test_report.get('accuracy', 0)),
        "test_confusion_matrix": test_cm.tolist(),
        "feature_importance": importance_dict
    }
    with open(f"{model_dir}/metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)

    print(f"\nArtifacts saved:")
    print(f"  best_model.pkl ({best_name}, refit on full data)")
    print(f"  scaler.pkl (fit on full data)")
    print(f"  label_encoder.pkl")
    print(f"  metrics.json (honest held-out test metrics)")

    return metrics

if __name__ == "__main__":
    train_models()
