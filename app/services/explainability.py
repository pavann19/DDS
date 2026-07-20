"""
SHAP Explainability Engine for DDS Autopilot
"""
import shap
import joblib
import pandas as pd
import json
import os
import numpy as np
import logging
from app.core.config import settings

logger = logging.getLogger(__name__)

class ExplainabilityEngine:
    def __init__(self, model_path=None, features_path=None):
        self.model_path = model_path or os.path.join(settings.BASE_DIR, "best_model.pkl")
        self.features_path = features_path or os.path.join(settings.BASE_DIR, "optimal_features.json")
        
        self.model = None
        self.explainer = None
        
        if os.path.exists(self.model_path):
            try:
                # SECURITY NOTE: Unsafe deserialization using joblib.
                self.model = joblib.load(self.model_path)
                self.explainer = shap.TreeExplainer(self.model)
                logger.info(f"Loaded Explainability model from {self.model_path}")
            except Exception as e:
                logger.error(f"Failed to initialize SHAP explainer: {e}", exc_info=True)
        else:
            logger.warning(f"Explainability model not found at {self.model_path}")
            
        if os.path.exists(self.features_path):
            try:
                with open(self.features_path, "r") as f:
                    opt = json.load(f)
                    self.features = opt.get("selected_features", [])
            except Exception as e:
                logger.error(f"Failed to load features: {e}")
                self.features = []
        else:
            self.features = ['Altitude', 'Coolant', 'Litre per 100km(Instant)', 'RPM', 'RPM_Delta', 'CO2_Delta', 'Fuel_Rate_Delta']
            
    def explain_prediction(self, input_features, class_index=None):
        if self.explainer is None:
            return {"base_value": 0, "contributions": []}
            
        if isinstance(input_features, list):
            input_features = pd.DataFrame([input_features], columns=self.features)
        elif isinstance(input_features, np.ndarray):
            input_features = pd.DataFrame(input_features, columns=self.features)
        elif isinstance(input_features, dict):
            input_features = pd.DataFrame([input_features])
            
        try:
            shap_values = self.explainer.shap_values(input_features)
            
            if isinstance(shap_values, list):
                vals = shap_values[class_index][0] if class_index is not None else shap_values[0][0]
            elif len(shap_values.shape) == 3:
                vals = shap_values[0, :, class_index] if class_index is not None else shap_values[0, :, 0]
            else:
                vals = shap_values[0]
                
            expected_value = self.explainer.expected_value
            if isinstance(expected_value, list) or isinstance(expected_value, np.ndarray):
                base = float(expected_value[class_index]) if class_index is not None else float(expected_value[0])
            else:
                base = float(expected_value)
                
            contributions = []
            for i, feat in enumerate(self.features):
                contributions.append({
                    "feature": feat,
                    "value": float(input_features.iloc[0][feat]),
                    "contribution": float(vals[i])
                })
                
            contributions.sort(key=lambda x: abs(x["contribution"]), reverse=True)
            
            return {
                "base_value": base,
                "contributions": contributions
            }
        except Exception as e:
            logger.error(f"Error calculating SHAP values: {e}", exc_info=True)
            return {"base_value": 0, "contributions": []}
