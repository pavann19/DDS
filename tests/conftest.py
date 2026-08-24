"""
Shared pytest fixtures.

REST/DB tests use an isolated in-memory SQLite database via a
`get_db` dependency override, so they never touch the real
`dds_telemetry.db` file the running app uses.
"""
import pytest
import pytest_asyncio
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy.orm import sessionmaker
from httpx import AsyncClient, ASGITransport
import joblib
import json
import numpy as np
from pathlib import Path
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.ensemble import RandomForestClassifier
from sklearn.datasets import make_classification

from app.core import database as db_module
from app.core.config import settings


@pytest.fixture(scope="session", autouse=True)
def setup_ml_artifacts():
    """Generate minimal mock ML artifacts for testing.
    
    This fixture runs once per test session and creates the required
    ML model files that the InferencePipeline and AnomalyDetector
    expect to find at settings.BASE_DIR.
    """
    base_dir = Path(settings.BASE_DIR)
    
    # Generate synthetic data for training
    X, y = make_classification(
        n_samples=100,
        n_features=4,
        n_informative=3,
        n_redundant=1,
        n_classes=3,
        random_state=42,
        shuffle=False
    )
    
    # Create and save classifier model
    model = RandomForestClassifier(n_estimators=5, random_state=42, max_depth=3)
    model.fit(X, y)
    joblib.dump(model, base_dir / "best_model.pkl")
    
    # Create and save scaler
    scaler = StandardScaler()
    scaler.fit(X)
    joblib.dump(scaler, base_dir / "scaler.pkl")
    
    # Create and save label encoder with expected driving actions
    label_encoder = LabelEncoder()
    labels = ["Accelerate", "Maintain Speed", "Decelerate"]
    label_encoder.fit(labels)
    joblib.dump(label_encoder, base_dir / "label_encoder.pkl")
    
    # Create optimal_features.json
    optimal_features = {
        "selected_features": ["Feature1", "Feature2", "Feature3", "Feature4"]
    }
    with open(base_dir / "optimal_features.json", "w") as f:
        json.dump(optimal_features, f)
    
    # Create anomaly model with same feature space
    anomaly_X = X
    anomaly_y = np.zeros(len(X))  # All normal (not anomalies)
    anomaly_model = RandomForestClassifier(n_estimators=5, random_state=42, max_depth=3)
    anomaly_model.fit(anomaly_X, anomaly_y)
    joblib.dump(anomaly_model, base_dir / "anomaly_model.pkl")
    
    # Create anomaly_feature_bounds.json with realistic bounds for telemetry features
    # These match the features used in test_anomaly_detector.py
    anomaly_bounds = {
        "Altitude": {"min": 0.0, "max": 1000.0},
        "RPM": {"min": 0.0, "max": 7000.0},
        "Coolant": {"min": 50.0, "max": 120.0},
        "Litre per 100km(Instant)": {"min": 0.0, "max": 50.0},
        "RPM_Delta": {"min": -500.0, "max": 500.0},
        "CO2_Delta": {"min": -100.0, "max": 100.0},
        "Fuel_Rate_Delta": {"min": -10.0, "max": 10.0}
    }

    with open(base_dir / "anomaly_feature_bounds.json", "w") as f:
        json.dump(anomaly_bounds, f)

    # Reload the inference pipeline AFTER all ML artifacts exist.
    from app.services import inference as inference_module
    inference_module.pipeline = inference_module.InferencePipeline()

    # Confirm the test environment is actually ready.
    assert inference_module.pipeline.is_ready(), (
        f"Test ML pipeline failed to initialize: "
        f"{inference_module.pipeline.get_errors()}"
    )

    yield
    
    # Cleanup: remove generated artifacts after tests
    for artifact in ["best_model.pkl", "scaler.pkl", "label_encoder.pkl", 
                     "optimal_features.json", "anomaly_model.pkl", "anomaly_feature_bounds.json"]:
        artifact_path = base_dir / artifact
        if artifact_path.exists():
            artifact_path.unlink()


@pytest_asyncio.fixture
async def test_db_session():
    engine = create_async_engine("sqlite+aiosqlite:///:memory:", echo=False)
    async with engine.begin() as conn:
        await conn.run_sync(db_module.Base.metadata.create_all)

    session_maker = sessionmaker(engine, class_=db_module.AsyncSession, expire_on_commit=False)

    async def override_get_db():
        async with session_maker() as session:
            yield session

    yield session_maker, override_get_db
    await engine.dispose()


@pytest_asyncio.fixture
async def api_client(test_db_session):
    """An httpx AsyncClient wired to the real FastAPI app, with the DB
    dependency swapped for an isolated in-memory SQLite instance."""
    from app.main import app
    from app.core.database import get_db

    _, override_get_db = test_db_session
    app.dependency_overrides[get_db] = override_get_db

    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client

    app.dependency_overrides.clear()
