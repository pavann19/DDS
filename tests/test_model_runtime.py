from pathlib import Path

import joblib
import numpy as np

from app.services.model_runtime import bound_inference_threads


def test_inference_thread_budget_preserves_recorded_model_predictions():
    model = joblib.load(Path(__file__).resolve().parents[1] / "best_model.pkl")
    x = np.random.default_rng(514).normal(size=(32, model.n_features_in_))
    probabilities = model.predict_proba(x)
    trees = model.get_booster().get_dump()
    bound_inference_threads(model)
    assert model.get_params()["n_jobs"] == 1
    assert model.get_booster().get_dump() == trees
    np.testing.assert_array_equal(model.predict_proba(x), probabilities)
