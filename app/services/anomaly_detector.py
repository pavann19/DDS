"""
Anomaly Detection Engine for DDS Autopilot
Uses Isolation Forest to detect abnormal driving patterns, plus a hard
per-feature range check (see task P1-3 -- Isolation Forest alone only
isolates JOINTLY unusual combinations; a single feature far outside the
training range, with everything else normal, mostly slips through it).
"""
import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest
from sklearn.model_selection import train_test_split
import joblib
import json
import os
import logging
from app.core.config import settings

logger = logging.getLogger(__name__)

class AnomalyDetector:
    def __init__(self, model_path=None, contamination=0.05, range_margin=1.0):
        self.model_path = model_path or os.path.join(settings.BASE_DIR, "anomaly_model.pkl")
        self.bounds_path = os.path.join(os.path.dirname(self.model_path), "anomaly_feature_bounds.json")
        self.contamination = contamination
        # A feature beyond [min - margin*range, max + margin*range] is a hard
        # range violation. margin=1.0 means "more than 1x the observed
        # train-split range beyond the min/max" -- deliberately looser than
        # the 2x/5x/10x cases task P1-3 tested, so this doesn't fire on
        # ordinary noisy-but-plausible readings.
        self.range_margin = range_margin
        self.model = None
        self.feature_bounds = None
        self.features = ['RPM', 'Coolant', 'CO2', 'Litre per 100km(Instant)', 'RPM_Delta', 'CO2_Delta', 'Fuel_Rate_Delta']

        if os.path.exists(self.model_path):
            try:
                # SECURITY NOTE: joblib.load is vulnerable to arbitrary code execution if the .pkl file is tampered with.
                # In this system, we assume models are generated locally and stored securely.
                self.model = joblib.load(self.model_path)
                if hasattr(self.model, "feature_names_in_"):
                    self.features = list(self.model.feature_names_in_)
                logger.info(f"Loaded AnomalyDetector model from {self.model_path}")
            except Exception as e:
                logger.error(f"Failed to load anomaly model: {e}", exc_info=True)
        else:
            logger.warning(f"Anomaly model not found at {self.model_path}")

        if os.path.exists(self.bounds_path):
            try:
                with open(self.bounds_path, "r") as f:
                    self.feature_bounds = json.load(f)
                logger.info(f"Loaded anomaly feature bounds from {self.bounds_path}")
            except Exception as e:
                logger.error(f"Failed to load anomaly feature bounds: {e}", exc_info=True)
        else:
            logger.warning(f"Anomaly feature bounds not found at {self.bounds_path} -- range check disabled")

    def train(self, data_path=None, features: list = None, test_size=0.2):
        """
        Trains the Isolation Forest on a stratified TRAIN split only (same
        test_size/random_state/stratify convention as model_pipeline.py, so
        this holds out data the model never sees -- consistent rigor with
        the classifier, even though IsolationForest is unsupervised and
        doesn't have the same train/test leakage concerns for its own
        accuracy). Also computes and saves per-feature (min, max) bounds
        from that same train split, used by detect()'s hard range check.
        """
        data_path = data_path or os.path.join(settings.BASE_DIR, "processed_telemetry.csv")

        if not os.path.exists(data_path):
            logger.error(f"Data file {data_path} not found.")
            return False

        df = pd.read_csv(data_path)
        target = df['Driving_Decision'] if 'Driving_Decision' in df.columns else None
        if 'Driving_Decision' in df.columns:
            df = df.drop(columns=['Driving_Decision'])

        selected_features = features or self.features
        missing = [feature for feature in selected_features if feature not in df.columns]
        if missing:
            logger.error(f"Cannot train anomaly detector; missing features: {missing}")
            return False

        df = df[selected_features]
        self.features = list(selected_features)

        if target is not None:
            df_train, _ = train_test_split(df, test_size=test_size, stratify=target, random_state=42)
        else:
            df_train, _ = train_test_split(df, test_size=test_size, random_state=42)

        logger.info(f"Training Anomaly Detector on {len(df_train)} train-split samples (of {len(df)} total) "
                    f"with features: {df_train.columns.tolist()}...")
        self.model = IsolationForest(contamination=self.contamination, random_state=42)
        self.model.fit(df_train)
        joblib.dump(self.model, self.model_path)
        logger.info(f"Anomaly Detector saved to {self.model_path}")

        self.feature_bounds = {
            f: {"min": float(df_train[f].min()), "max": float(df_train[f].max())}
            for f in self.features
        }
        with open(self.bounds_path, "w") as f:
            json.dump(self.feature_bounds, f, indent=2)
        logger.info(f"Anomaly feature bounds saved to {self.bounds_path}")

        return True

    def _check_range_violations(self, row):
        """Returns (feature_name, magnitude_beyond_range) for the worst hard
        range violation in this row, or None if none. magnitude is how many
        multiples of the observed train range the value sits beyond
        min/max -- used to grade severity."""
        if not self.feature_bounds:
            return None

        worst = None
        for f in self.features:
            if f not in self.feature_bounds:
                continue
            b = self.feature_bounds[f]
            rng = b["max"] - b["min"]
            if rng <= 0:
                continue
            lower = b["min"] - self.range_margin * rng
            upper = b["max"] + self.range_margin * rng
            value = row.get(f, 0.0)
            if value < lower:
                magnitude = (lower - value) / rng
            elif value > upper:
                magnitude = (value - upper) / rng
            else:
                continue
            if worst is None or magnitude > worst[1]:
                worst = (f, magnitude)
        return worst

    def detect(self, input_features, model_confidence=1.0):
        """
        Detects anomalies in a single reading.
        Returns a dictionary with anomaly status and details.
        """
        if self.model is None:
            return {"is_anomaly": False, "type": "NONE", "severity": "NONE", "message": "Model not loaded"}

        if isinstance(input_features, list):
            input_features = pd.DataFrame([input_features], columns=self.features)
        elif isinstance(input_features, np.ndarray):
            input_features = pd.DataFrame(input_features, columns=self.features)
        elif isinstance(input_features, dict):
            input_features = pd.DataFrame([input_features])

        missing = [feature for feature in self.features if feature not in input_features.columns]
        if missing:
            return {
                "is_anomaly": False,
                "type": "FEATURE_MISMATCH",
                "severity": "LOW",
                "message": f"Missing anomaly features: {', '.join(missing)}"
            }

        input_features = input_features[self.features]
        row = input_features.iloc[0]

        # --- Hard range check (task P1-3): catches single-feature extreme
        # values that Isolation Forest alone mostly misses, since IF isolates
        # JOINTLY unusual combinations, not univariate outliers when every
        # other feature looks ordinary. Checked BEFORE the IF-based logic so
        # a hard violation is never silently downgraded by it. ---
        violation = self._check_range_violations(row)
        if violation is not None:
            feat, magnitude = violation
            severity = "HIGH" if magnitude > 3 else "MEDIUM"
            return {
                "is_anomaly": True,
                "type": "OUT_OF_RANGE",
                "severity": severity,
                "message": f"{feat} is {magnitude:.1f}x the observed training range outside its normal bounds "
                           f"({row[feat]:.2f})"
            }

        # 1 means normal, -1 means anomaly in IsolationForest
        prediction = self.model.predict(input_features)[0]

        is_anomaly = (prediction == -1)
        anomaly_type = "NONE"
        severity = "NONE"
        message = ""

        if is_anomaly:
            if row.get('Coolant', 0) > 95:
                anomaly_type = "OVERHEAT"
                severity = "HIGH"
                message = f"Engine coolant temperature critically high ({row['Coolant']:.1f}°C)"
            elif abs(row.get('RPM_Delta', 0)) > 500:
                anomaly_type = "RPM_SPIKE"
                severity = "MEDIUM"
                message = f"Sudden RPM change detected ({row['RPM_Delta']:.0f} RPM)"
            elif row.get('CO2', 0) > 400:
                anomaly_type = "HIGH_EMISSION"
                severity = "MEDIUM"
                message = "Unusually high CO2 emissions detected"
            elif model_confidence < 0.6:
                anomaly_type = "ERRATIC"
                severity = "LOW"
                message = "Erratic driving pattern (low model confidence)"
            else:
                anomaly_type = "UNKNOWN"
                severity = "LOW"
                message = "Unusual telemetry reading detected"

        return {
            "is_anomaly": bool(is_anomaly),
            "type": anomaly_type,
            "severity": severity,
            "message": message
        }
