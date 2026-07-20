"""
Anomaly Detection Engine for DDS Autopilot
Uses Isolation Forest to detect abnormal driving patterns.
"""
import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest
import joblib
import os
import logging
from app.core.config import settings

logger = logging.getLogger(__name__)

class AnomalyDetector:
    def __init__(self, model_path=None, contamination=0.05):
        self.model_path = model_path or os.path.join(settings.BASE_DIR, "anomaly_model.pkl")
        self.contamination = contamination
        self.model = None
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
                
    def train(self, data_path=None, features: list = None):
        """Trains the Isolation Forest model on normal data"""
        data_path = data_path or os.path.join(settings.BASE_DIR, "processed_telemetry.csv")
        
        if not os.path.exists(data_path):
            logger.error(f"Data file {data_path} not found.")
            return False
            
        df = pd.read_csv(data_path)
        if 'Driving_Decision' in df.columns:
            df = df.drop(columns=['Driving_Decision'])
            
        selected_features = features or self.features
        missing = [feature for feature in selected_features if feature not in df.columns]
        if missing:
            logger.error(f"Cannot train anomaly detector; missing features: {missing}")
            return False

        df = df[selected_features]
        self.features = list(selected_features)
            
        logger.info(f"Training Anomaly Detector on {len(df)} samples with features: {df.columns.tolist()}...")
        self.model = IsolationForest(contamination=self.contamination, random_state=42)
        self.model.fit(df)
        joblib.dump(self.model, self.model_path)
        logger.info(f"Anomaly Detector saved to {self.model_path}")
        return True
        
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
            
        # 1 means normal, -1 means anomaly in IsolationForest
        prediction = self.model.predict(input_features)[0]
        
        is_anomaly = (prediction == -1)
        anomaly_type = "NONE"
        severity = "NONE"
        message = ""
        
        if is_anomaly:
            row = input_features.iloc[0]
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
