import pandas as pd
import numpy as np
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.neural_network import MLPClassifier
from xgboost import XGBClassifier
from sklearn.metrics import classification_report, confusion_matrix, accuracy_score
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
                 optimal_features_file="optimal_features.json"):
    """
    Trains multiple ML models with proper evaluation.
    
    Key improvements over the original:
    - Uses StratifiedKFold cross-validation (5-fold) for honest accuracy
    - Full classification report (precision, recall, F1 per class)
    - Confusion matrix saved
    - Actually selects the best model by CV score
    - Optionally uses optimal feature subset from exhaustive search
    - Feature importance analysis
    """
    df = pd.read_csv(data_file)
    
    # --- FEATURE SELECTION ---
    # Check if optimal features from exhaustive search exist
    feature_info = load_features()
    all_features = feature_info["features"] if feature_info else [
        'Altitude', 'CO2', 'Coolant', 'Litre per 100km(Instant)', 
        'RPM', 'RPM_Delta', 'CO2_Delta', 'Fuel_Rate_Delta'
    ]
    
    # Use optimal subset if available, otherwise use all features
    if os.path.exists(optimal_features_file):
        with open(optimal_features_file, "r") as f:
            opt = json.load(f)
        features = opt.get("selected_features", all_features)
        print(f"Using optimal feature subset from exhaustive search: {features}")
    else:
        features = all_features
        print(f"Using all features: {features}")
    
    X = df[features]
    
    # Encode target
    le = LabelEncoder()
    y = le.fit_transform(df['Driving_Decision'])
    class_names = le.classes_.tolist()
    
    print(f"\nDataset: {len(X)} samples, {len(features)} features")
    print(f"Classes: {class_names}")
    print(f"Class distribution: {dict(zip(class_names, np.bincount(y)))}")
    
    # --- SCALING ---
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    
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
    
    # --- STRATIFIED CROSS-VALIDATION ---
    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
    
    print("\n" + "="*60)
    print("STRATIFIED 5-FOLD CROSS-VALIDATION RESULTS")
    print("="*60)
    
    cv_results = {}
    for name, model in models.items():
        scores = cross_val_score(model, X_scaled, y, cv=cv, scoring='accuracy')
        cv_results[name] = {
            "mean_accuracy": float(scores.mean()),
            "std_accuracy": float(scores.std()),
            "fold_scores": scores.tolist()
        }
        print(f"\n{name}:")
        print(f"  Accuracy: {scores.mean():.4f} ± {scores.std():.4f}")
        print(f"  Folds: {[f'{s:.4f}' for s in scores]}")
    
    # --- SELECT BEST MODEL ---
    best_name = max(cv_results, key=lambda k: cv_results[k]["mean_accuracy"])
    best_model = models[best_name]
    print(f"\n{'='*60}")
    print(f"BEST MODEL: {best_name} (CV Accuracy: {cv_results[best_name]['mean_accuracy']:.4f})")
    print(f"{'='*60}")
    
    # --- FINAL TRAINING on full dataset for deployment ---
    best_model.fit(X_scaled, y)
    y_pred = best_model.predict(X_scaled)  # Training set predictions for report
    
    # Full classification report
    print(f"\nClassification Report ({best_name} on full training data):")
    report = classification_report(y, y_pred, target_names=class_names, output_dict=True)
    print(classification_report(y, y_pred, target_names=class_names))
    
    # Confusion matrix
    cm = confusion_matrix(y, y_pred)
    print("Confusion Matrix:")
    print(f"  Classes: {class_names}")
    for i, row in enumerate(cm):
        print(f"  {class_names[i]:>15}: {row}")
    
    # --- FEATURE IMPORTANCE ---
    importance_dict = {}
    if hasattr(best_model, 'feature_importances_'):
        importances = best_model.feature_importances_
        sorted_idx = np.argsort(importances)[::-1]
        print(f"\nFeature Importance ({best_name}):")
        for idx in sorted_idx:
            print(f"  {features[idx]:>30}: {importances[idx]:.4f}")
            importance_dict[features[idx]] = float(importances[idx])
    
    # --- SAVE ARTIFACTS ---
    joblib.dump(best_model, f"{model_dir}/best_model.pkl")
    joblib.dump(scaler, f"{model_dir}/scaler.pkl")
    joblib.dump(le, f"{model_dir}/label_encoder.pkl")
    
    # Comprehensive metrics
    metrics = {
        "best_model": best_name,
        "cv_results": cv_results,
        "features_used": features,
        "class_names": class_names,
        "classification_report": {k: v for k, v in report.items() if k != 'accuracy'},
        "overall_accuracy": float(report.get('accuracy', 0)),
        "confusion_matrix": cm.tolist(),
        "feature_importance": importance_dict
    }
    with open(f"{model_dir}/metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)
        
    print(f"\nArtifacts saved:")
    print(f"  best_model.pkl ({best_name})")
    print(f"  scaler.pkl")
    print(f"  label_encoder.pkl")
    print(f"  metrics.json")

if __name__ == "__main__":
    train_models()
