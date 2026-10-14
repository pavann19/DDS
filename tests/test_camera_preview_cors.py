"""Browser preview must read the capture timestamp across the API origin."""
from fastapi.testclient import TestClient
from app.main import app
from app.core.config import settings


def test_camera_timestamp_headers_exposed_to_browser():
    origin = settings.ALLOWED_ORIGINS[0]
    with TestClient(app) as client:
        response = client.get('/api/camera/scene/invalid.png', headers={'Origin': origin})
    assert response.status_code == 404
    exposed = {value.strip().lower() for value in response.headers['access-control-expose-headers'].split(',')}
    assert {'x-capture-time-s', 'x-camera-id'} <= exposed
