import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from sklearn.model_selection import StratifiedKFold, train_test_split
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.neural_network import MLPClassifier
from xgboost import XGBClassifier
from sklearn.metrics import classification_report, confusion_matrix, brier_score_loss
from sklearn.calibration import calibration_curve
from sklearn.utils.class_weight import compute_sample_weight
import joblib
import json
import os

def load_features(feature_file="feature_info.json"):
    """Load the feature list saved by data_prep.py."""
    if os.path.exists(feature_file):
        with open(feature_file, "r") as f:
            return json.load(f)
    return None

# Models with no native class-weighting get a per-sample balanced weight
# instead (see train_models's cross_validate_balanced / fit calls).
# RandomForest keeps its native class_weight='balanced' (computed fresh per
# fold from that fold's own y, same effect, no need to double up).
SAMPLE_WEIGHTED_MODELS = {"GradientBoosting", "MLP", "XGBoost"}

def cross_validate_balanced(model_factory, model_name, X, y, cv):
    """Manual CV loop (not cross_val_score) so we control exactly when
    class-imbalance-balanced sample_weight is applied per fold, per model."""
    scores = []
    for train_idx, val_idx in cv.split(X, y):
        X_tr, X_val = X[train_idx], X[val_idx]
        y_tr, y_val = y[train_idx], y[val_idx]
        model = model_factory()
        if model_name in SAMPLE_WEIGHTED_MODELS:
            sw = compute_sample_weight('balanced', y_tr)
            model.fit(X_tr, y_tr, sample_weight=sw)
        else:
            model.fit(X_tr, y_tr)
        scores.append(model.score(X_val, y_val))
    return np.array(scores)

def fit_balanced(model, model_name, X, y):
    """Fit a model with balanced sample_weight if it needs it (see
    SAMPLE_WEIGHTED_MODELS); RandomForest already balances natively."""
    if model_name in SAMPLE_WEIGHTED_MODELS:
        sw = compute_sample_weight('balanced', y)
        model.fit(X, y, sample_weight=sw)
    else:
        model.fit(X, y)
    return model

def compute_calibration(model, X_test_scaled, y_test, class_names, out_path):
    """
    Top-label (overall) Expected Calibration Error + per-class one-vs-rest
    reliability curves, plotted to out_path. Answers: when the model says
    it's 90% confident, is it actually right ~90% of the time?
    """
    proba = model.predict_proba(X_test_scaled)
    pred_idx = np.argmax(proba, axis=1)
    confidence = np.max(proba, axis=1)
    correct = (pred_idx == y_test).astype(int)

    # Top-label ECE via 5 equal-width confidence bins (n=180 test samples
    # is too small for finer binning to be meaningful).
    n_bins = 5
    bin_edges = np.linspace(0, 1, n_bins + 1)
    bin_ids = np.digitize(confidence, bin_edges[1:-1])
    ece = 0.0
    bin_stats = []
    for b in range(n_bins):
        mask = bin_ids == b
        if mask.sum() == 0:
            continue
        bin_acc = correct[mask].mean()
        bin_conf = confidence[mask].mean()
        weight = mask.sum() / len(confidence)
        ece += weight * abs(bin_acc - bin_conf)
        bin_stats.append({
            "bin_range": [float(bin_edges[b]), float(bin_edges[b + 1])],
            "n_samples": int(mask.sum()),
            "mean_confidence": float(bin_conf),
            "accuracy": float(bin_acc)
        })

    # Per-class one-vs-rest reliability curves + Brier scores
    fig, axes = plt.subplots(1, len(class_names) + 1, figsize=(5 * (len(class_names) + 1), 4.5))

    ax0 = axes[0]
    if bin_stats:
        confs = [b["mean_confidence"] for b in bin_stats]
        accs = [b["accuracy"] for b in bin_stats]
        ax0.plot([0, 1], [0, 1], "k--", label="Perfect calibration")
        ax0.plot(confs, accs, "o-", color="#3b82f6", label="Model")
    ax0.set_xlabel("Mean predicted confidence")
    ax0.set_ylabel("Empirical accuracy")
    ax0.set_title(f"Top-label calibration (ECE={ece:.4f})")
    ax0.legend()
    ax0.set_xlim(0, 1)
    ax0.set_ylim(0, 1)

    brier_scores = {}
    for i, cls in enumerate(class_names):
        y_binary = (y_test == i).astype(int)
        prob_true, prob_pred = calibration_curve(y_binary, proba[:, i], n_bins=5, strategy="uniform")
        brier = brier_score_loss(y_binary, proba[:, i])
        brier_scores[cls] = float(brier)

        ax = axes[i + 1]
        ax.plot([0, 1], [0, 1], "k--", label="Perfect calibration")
        ax.plot(prob_pred, prob_true, "o-", color="#22c55e", label="Model")
        ax.set_xlabel("Mean predicted probability")
        ax.set_ylabel("Fraction positive")
        ax.set_title(f"{cls} (Brier={brier:.4f})")
        ax.legend()
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)

    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close(fig)

    return {
        "top_label_ece": float(ece),
        "top_label_bins": bin_stats,
        "per_class_brier_score": brier_scores,
        "plot_path": out_path
    }

def train_models(data_file="processed_telemetry.csv", model_dir=".",
                 optimal_features_file="optimal_features.json", test_size=0.2):
    """
    Trains multiple ML models with a genuine held-out test evaluation,
    class-imbalance-balanced fitting, and a calibration check.

    Methodology (fixes a prior bug where the reported metrics were computed
    on the training set itself, giving a meaningless 100% accuracy):
    1. Split off a stratified held-out test set FIRST, before any model
       selection or fitting touches the data.
    2. Model selection (5-fold CV) runs on the TRAIN split only, with
       class-imbalance-balanced fitting (native class_weight for
       RandomForest, sample_weight='balanced' for the others -- see
       SAMPLE_WEIGHTED_MODELS).
    3. The selected model type is fit on TRAIN (balanced) and evaluated
       ONCE on the untouched TEST split -- this is the reported, honest
       metric, plus a calibration check (is model confidence trustworthy?).
    4. For the artifact actually shipped for live inference, the same
       model type is refit on the FULL dataset (train+test, balanced) to
       use all available data -- standard practice, but it means
       best_model.pkl's expected generalization is represented by the
       step-3 test metrics, not by re-evaluating it on its own training
       data.

    Feature selection (optimal_features.json, genetic_optimizer.py) and
    model selection (SHAP-multiclass-compatibility constraint) are both
    already leak-free / documented as of tasks P1-1b and P1-1.
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

    # --- MODEL FACTORIES (fresh instance per CV fold / fit call) ---
    model_factories = {
        "RandomForest": lambda: RandomForestClassifier(
            n_estimators=200, max_depth=15, min_samples_split=5,
            class_weight='balanced', random_state=42
        ),
        "GradientBoosting": lambda: GradientBoostingClassifier(
            n_estimators=200, max_depth=5, learning_rate=0.1,
            random_state=42
        ),
        "MLP": lambda: MLPClassifier(
            hidden_layer_sizes=(128, 64, 32), max_iter=1000,
            early_stopping=True, validation_fraction=0.15,
            random_state=42
        ),
        "XGBoost": lambda: XGBClassifier(
            n_estimators=200, max_depth=6, learning_rate=0.1,
            eval_metric='mlogloss', random_state=42
        )
    }

    # --- STRATIFIED CROSS-VALIDATION ON TRAIN ONLY (model selection),
    #     class-imbalance-balanced ---
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)

    print("\n" + "="*60)
    print("STRATIFIED 5-FOLD CV, CLASS-BALANCED (train split only)")
    print("="*60)

    cv_results = {}
    for name, factory in model_factories.items():
        scores = cross_validate_balanced(factory, name, X_train_scaled, y_train, cv)
        cv_results[name] = {
            "mean_accuracy": float(scores.mean()),
            "std_accuracy": float(scores.std()),
            "fold_scores": scores.tolist(),
            "balancing": "sample_weight='balanced'" if name in SAMPLE_WEIGHTED_MODELS else "class_weight='balanced' (native)"
        }
        print(f"\n{name} ({cv_results[name]['balancing']}):")
        print(f"  Accuracy: {scores.mean():.4f} +/- {scores.std():.4f}")
        print(f"  Folds: {[f'{s:.4f}' for s in scores]}")

    # Model selection is constrained to models compatible with shap.TreeExplainer
    # for MULTICLASS output (app/services/explainability.py, used live in the
    # 10Hz WebSocket loop -- too latency-sensitive for Kernel/Permutation SHAP
    # fallbacks). GradientBoostingClassifier concretely fails inside
    # shap.TreeExplainer for 3+ classes; MLPClassifier isn't tree-based at all.
    # Both are evaluated above for transparency but not selectable.
    SHAP_COMPATIBLE = {"RandomForest", "XGBoost"}
    eligible = {k: v for k, v in cv_results.items() if k in SHAP_COMPATIBLE}
    best_name = max(eligible, key=lambda k: eligible[k]["mean_accuracy"])
    print(f"\n{'='*60}")
    print(f"BEST MODEL (by balanced train-split CV, SHAP-compatible only): "
          f"{best_name} (CV Accuracy: {cv_results[best_name]['mean_accuracy']:.4f})")
    for name in cv_results:
        if name not in SHAP_COMPATIBLE:
            print(f"  [excluded from selection] {name}: {cv_results[name]['mean_accuracy']:.4f} "
                  f"(not shap.TreeExplainer multiclass-compatible)")
    print(f"{'='*60}")

    # --- FIT ON TRAIN (balanced), EVALUATE ON HELD-OUT TEST (the honest metric) ---
    eval_model = model_factories[best_name]()
    fit_balanced(eval_model, best_name, X_train_scaled, y_train)
    y_test_pred = eval_model.predict(X_test_scaled)

    print(f"\nHeld-out TEST Classification Report ({best_name}, balanced fit):")
    test_report = classification_report(y_test, y_test_pred, target_names=class_names, output_dict=True)
    print(classification_report(y_test, y_test_pred, target_names=class_names))

    test_cm = confusion_matrix(y_test, y_test_pred)
    print("Held-out TEST Confusion Matrix:")
    print(f"  Classes: {class_names}")
    for i, row in enumerate(test_cm):
        print(f"  {class_names[i]:>15}: {row}")

    # --- CALIBRATION CHECK: are eval_model's confidence scores trustworthy? ---
    calib_dir = os.path.join(model_dir, "_evidence", "P1-2")
    os.makedirs(calib_dir, exist_ok=True)
    calibration = compute_calibration(
        eval_model, X_test_scaled, y_test, class_names,
        os.path.join(calib_dir, "calibration_curves.png")
    )
    print(f"\nCalibration (held-out test, {best_name}):")
    print(f"  Top-label ECE: {calibration['top_label_ece']:.4f} (0 = perfectly calibrated)")
    print(f"  Per-class Brier scores: {calibration['per_class_brier_score']}")

    # --- REFIT ON FULL DATA (balanced) FOR THE DEPLOYED ARTIFACT ---
    full_scaler = StandardScaler()
    X_full_scaled = full_scaler.fit_transform(X)
    deploy_model = model_factories[best_name]()
    fit_balanced(deploy_model, best_name, X_full_scaled, y)

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
        "model_selection_constraint": (
            "Selection is restricted to models compatible with shap.TreeExplainer "
            "for multiclass output (RandomForest, XGBoost), since SHAP is computed "
            "live in the 10Hz WebSocket loop and slower model-agnostic explainers "
            "aren't viable at that latency. GradientBoosting and MLP are evaluated "
            "above for transparency but excluded from selection even when they "
            "score higher on raw CV accuracy -- GradientBoostingClassifier "
            "concretely fails inside shap.TreeExplainer for 3+ classes."
        ),
        "class_imbalance_handling": (
            "Class distribution is 711/104/82 (Maintain Speed/Accelerate/"
            "Decelerate). RandomForest uses native class_weight='balanced' "
            "(recomputed per CV fold from that fold's own y). GradientBoosting, "
            "MLP, and XGBoost -- none of which expose a class_weight parameter "
            "-- use per-sample compute_sample_weight('balanced', y) passed to "
            "fit(), applied consistently in CV, the held-out-test evaluation "
            "fit, and the final deployment fit."
        ),
        "calibration": calibration,
        "methodology": (
            "Model selection via 5-fold stratified CV on an 80% train split. "
            "Reported test_* metrics are from ONE evaluation on a 20% held-out "
            "stratified split the model never trained on. The deployed "
            "best_model.pkl is refit on 100% of the data (standard practice for "
            "shipping) -- its expected real-world performance is represented by "
            "the test_* metrics below, not by evaluating it on its own training "
            "data."
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
    print(f"  best_model.pkl ({best_name}, refit on full data, balanced)")
    print(f"  scaler.pkl (fit on full data)")
    print(f"  label_encoder.pkl")
    print(f"  metrics.json (honest held-out test metrics + calibration)")
    print(f"  {calib_dir}/calibration_curves.png")

    return metrics

if __name__ == "__main__":
    train_models()
