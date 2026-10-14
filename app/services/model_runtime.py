"""Bound background single-reading analytics before workers start."""


def bound_inference_threads(model):
    if hasattr(model, "get_params") and "n_jobs" in model.get_params():
        model.set_params(n_jobs=1)
    return model
