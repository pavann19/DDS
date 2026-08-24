import os
import json
import joblib
import pandas as pd
import numpy as np
import logging
from app.core.config import settings
from app.services.explainability import ExplainabilityEngine
from app.services.anomaly_detector import AnomalyDetector

logger = logging.getLogger(__name__)

class InferencePipeline:
    def __init__(self):
        self.model = None
        self.scaler = None
        self.label_encoder = None
        self.features = []
        
        self._load_artifacts()
        
        self.explainer = ExplainabilityEngine()
        self.anomaly_detector = AnomalyDetector()
        
    def _load_artifacts(self):
        try:
            # SECURITY NOTE: joblib.load on .pkl files is inherently unsafe against arbitrary code execution.
            # Assuming these artifacts are generated locally by the data pipeline and not externally supplied.
            self.model = joblib.load(os.path.join(settings.BASE_DIR, "best_model.pkl"))
            self.scaler = joblib.load(os.path.join(settings.BASE_DIR, "scaler.pkl"))
            self.label_encoder = joblib.load(os.path.join(settings.BASE_DIR, "label_encoder.pkl"))
            
            with open(os.path.join(settings.BASE_DIR, "optimal_features.json"), "r") as f:
                self.features = json.load(f)["selected_features"]
                
            logger.info("Successfully loaded ML artifacts for InferencePipeline.")
        except Exception as e:
            logger.error(f"Failed to load ML artifacts: {e}", exc_info=True)

    def is_ready(self) -> bool:
        return all([
            self.model is not None, 
            self.scaler is not None, 
            self.label_encoder is not None, 
            bool(self.features)
        ])

    def get_errors(self) -> list:
        errors = []
        if self.model is None: errors.append("best_model.pkl is missing or failed to load")
        if self.scaler is None: errors.append("scaler.pkl is missing or failed to load")
        if self.label_encoder is None: errors.append("label_encoder.pkl is missing or failed to load")
        if not self.features: errors.append("optimal_features.json is missing or invalid")
        return errors

    def predict(self, input_dict: dict):
        if not self.is_ready():
            raise RuntimeError(f"InferencePipeline is not ready: {self.get_errors()}")
            
        clean_input = {f: float(input_dict.get(f, 0.0)) for f in self.features}
        input_df = pd.DataFrame([clean_input])

        scaled_input = self.scaler.transform(input_df)

        pred_proba = self.model.predict_proba(scaled_input)[0]
        pred_idx = int(np.argmax(pred_proba))
        pred_label = self.label_encoder.inverse_transform([pred_idx])[0]
        confidence = float(np.max(pred_proba))
        
        confidence_override = False
        if confidence < 0.55:
            # Safety override: below this confidence, don't trust the raw
            # prediction, default to the safest action instead. This must
            # also repoint pred_idx at Decelerate's class index -- not just
            # relabel pred_label -- otherwise the SHAP explanation below
            # would still explain the ORIGINAL (discarded) prediction while
            # the car visibly does something else, a real inconsistency
            # between what the AI Prediction panel says it's doing and what
            # the SHAP panel says caused it.
            classes = list(self.label_encoder.classes_)
            if "Decelerate" in classes:
                pred_idx = classes.index("Decelerate")
            pred_label = "Decelerate"
            confidence_override = True

        conf_dict = {str(self.label_encoder.inverse_transform([i])[0]): float(prob) for i, prob in enumerate(pred_proba)}

        shap_result = self.explainer.explain_prediction(input_df, class_index=pred_idx)

        # The anomaly detector is an independently-trained model with its own
        # feature set (anomaly_model.pkl's feature_names_in_), which is not
        # guaranteed to match the classifier's feature subset (optimal_features.json)
        # -- they can and do diverge whenever either model is retrained separately.
        # Build its input from the raw input_dict directly rather than reusing
        # clean_input, so a classifier feature-set change can never silently
        # break anomaly detection again.
        # silently defaulting missing telemetry fields to
        # 0.0 (below) let a reading missing 6 of 7 fields predict at 97.7%
        # confidence with no anomaly flag at all -- a real risk if an
        # upstream sensor/physics-engine field ever drops out. Detect
        # genuine absence (key not present) BEFORE defaulting, and treat it
        # as its own anomaly category, distinct from a present-but-extreme
        # reading (which the range/Isolation-Forest checks in
        # AnomalyDetector.detect() already handle).
        anomaly_missing_keys = [f for f in self.anomaly_detector.features if f not in input_dict]
        anomaly_input = {f: float(input_dict.get(f, 0.0)) for f in self.anomaly_detector.features}
        anomaly_df = pd.DataFrame([anomaly_input])
        if anomaly_missing_keys:
            anomaly_result = {
                "is_anomaly": True,
                "type": "INCOMPLETE_INPUT",
                "severity": "MEDIUM",
                "message": f"Telemetry fields missing from input, defaulted to 0.0: {', '.join(anomaly_missing_keys)}"
            }
        else:
            anomaly_result = self.anomaly_detector.detect(anomaly_df, model_confidence=confidence)
        
        return {
            "prediction": pred_label,
            "confidence": confidence,
            "confidence_dict": conf_dict,
            "confidence_override": confidence_override,
            "shap_result": shap_result,
            "anomaly_result": anomaly_result,
            "clean_input": clean_input
        }

# Global instance
pipeline = InferencePipeline()
