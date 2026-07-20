"""
Driver Scoring System for DDS Autopilot
Calculates a dynamic driver behavior score (0-100).
"""
import collections
import time
import logging

logger = logging.getLogger(__name__)

class DriverScorer:
    def __init__(self, window_size=60):
        self.window_size = window_size
        self.history = collections.deque(maxlen=window_size)
        
    def add_reading(self, telemetry, prediction, confidence, anomaly):
        self.history.append({
            "telemetry": telemetry,
            "prediction": prediction,
            "confidence": confidence,
            "anomaly": anomaly,
            "timestamp": time.time()
        })
        
    def calculate_score(self):
        if len(self.history) == 0:
            return {"score": 100, "rating": "A+", "breakdown": {}}
            
        score = 100.0
        transitions = 0
        
        for i in range(1, len(self.history)):
            prev = self.history[i-1]["prediction"]
            curr = self.history[i]["prediction"]
            if prev != curr and curr != "Maintain Speed":
                transitions += 1
                
        smoothness_penalty = min(20.0, transitions * 2.0)
        score -= smoothness_penalty
        
        high_fuel_readings = sum(1 for item in self.history if item["telemetry"].get("Litre per 100km(Instant)", 0) > 15)
        efficiency_penalty = min(20.0, (high_fuel_readings / len(self.history)) * 40.0)
        score -= efficiency_penalty
        
        safety_penalty = 0.0
        for item in self.history:
            if item["anomaly"].get("is_anomaly"):
                severity = item["anomaly"].get("severity", "LOW")
                if severity == "HIGH":
                    safety_penalty += 15.0
                elif severity == "MEDIUM":
                    safety_penalty += 5.0
                else:
                    safety_penalty += 2.0
                    
            if item["confidence"] < 0.6:
                safety_penalty += 0.5
                
        safety_penalty = min(60.0, safety_penalty)
        score -= safety_penalty
        
        score = max(0.0, min(100.0, score))
        
        if score >= 90: rating = "A+"
        elif score >= 80: rating = "A"
        elif score >= 70: rating = "B"
        elif score >= 60: rating = "C"
        elif score >= 50: rating = "D"
        else: rating = "F"
        
        return {
            "score": round(score),
            "rating": rating,
            "breakdown": {
                "smoothness": round(100 - (smoothness_penalty * 5)),
                "efficiency": round(100 - (efficiency_penalty * 5)),
                "safety": round(100 - (safety_penalty * 1.66)) 
            }
        }
