"""
Unit tests for app/services/routing.py, using a monkeypatched httpx client
so these don't depend on the real OSRM public demo service being up
(they'd be flaky/slow otherwise) -- the fail-soft behavior when the
service genuinely IS unavailable is exactly what these tests exist to
pin down.
"""
import httpx
import pytest

from app.services import routing


@pytest.fixture(autouse=True)
def _clear_route_cache():
    """The route cache is a module-level, process-lifetime dict -- without
    clearing it, a mocked test result cached under a rounded lat/lng here
    can leak into a LATER test (e.g. the WS smoke test, which defaults to
    routing between these same SF coordinates) when the full suite runs in
    one process, even though this test passes fine in isolation."""
    routing.clear_route_cache()
    yield
    routing.clear_route_cache()


class _FakeResponse:
    def __init__(self, json_data, status_code=200):
        self._json_data = json_data
        self.status_code = status_code

    def raise_for_status(self):
        if self.status_code >= 400:
            raise httpx.HTTPStatusError("error", request=None, response=self)

    def json(self):
        return self._json_data


class _FakeAsyncClient:
    def __init__(self, response=None, exc=None):
        self._response = response
        self._exc = exc

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        return False

    async def get(self, url, params=None):
        if self._exc:
            raise self._exc
        return self._response


def _osrm_ok_payload():
    return {
        "code": "Ok",
        "routes": [{
            "geometry": {
                "coordinates": [
                    [-122.4194, 37.7749],
                    [-122.4190, 37.7755],
                    [-122.4783, 37.8199],
                ]
            },
            "legs": [{
                "steps": [
                    {
                        "name": "Market Street",
                        "distance": 13.7,
                        "maneuver": {"type": "depart", "modifier": "right", "location": [-122.4194, 37.7749]},
                    },
                    {
                        "name": "South Van Ness Avenue",
                        "distance": 225.5,
                        "maneuver": {"type": "turn", "modifier": "sharp right", "location": [-122.4190, 37.7755]},
                    },
                ]
            }],
        }],
    }


@pytest.mark.asyncio
async def test_get_route_returns_lat_lng_waypoints_and_parsed_steps(monkeypatch):
    monkeypatch.setattr(
        routing.httpx, "AsyncClient",
        lambda *a, **k: _FakeAsyncClient(response=_FakeResponse(_osrm_ok_payload())),
    )
    result = await routing.get_route(37.7749, -122.4194, 37.8199, -122.4783)
    assert result is not None
    waypoints, steps = result
    assert waypoints == [
        (37.7749, -122.4194),
        (37.7755, -122.4190),
        (37.8199, -122.4783),
    ]
    assert len(steps) == 2
    assert steps[0]["type"] == "depart"
    assert steps[1]["modifier"] == "sharp right"
    assert steps[1]["instruction"] == "South Van Ness Avenue"
    assert steps[0]["location"] == (37.7749, -122.4194)


@pytest.mark.asyncio
async def test_get_route_missing_legs_still_returns_waypoints_with_empty_steps(monkeypatch):
    payload = _osrm_ok_payload()
    del payload["routes"][0]["legs"]
    monkeypatch.setattr(
        routing.httpx, "AsyncClient",
        lambda *a, **k: _FakeAsyncClient(response=_FakeResponse(payload)),
    )
    result = await routing.get_route(37.7749, -122.4194, 37.8199, -122.4783)
    assert result is not None
    waypoints, steps = result
    assert len(waypoints) == 3
    assert steps == []


@pytest.mark.asyncio
async def test_get_route_returns_none_on_network_error(monkeypatch):
    monkeypatch.setattr(
        routing.httpx, "AsyncClient",
        lambda *a, **k: _FakeAsyncClient(exc=httpx.ConnectError("boom")),
    )
    result = await routing.get_route(37.7749, -122.4194, 37.8199, -122.4783)
    assert result is None


@pytest.mark.asyncio
async def test_get_route_returns_none_on_non_ok_code(monkeypatch):
    monkeypatch.setattr(
        routing.httpx, "AsyncClient",
        lambda *a, **k: _FakeAsyncClient(response=_FakeResponse({"code": "NoRoute", "routes": []})),
    )
    result = await routing.get_route(37.7749, -122.4194, 0.0, 0.0)
    assert result is None


@pytest.mark.asyncio
async def test_get_route_returns_none_on_http_error_status(monkeypatch):
    monkeypatch.setattr(
        routing.httpx, "AsyncClient",
        lambda *a, **k: _FakeAsyncClient(response=_FakeResponse({}, status_code=500)),
    )
    result = await routing.get_route(37.7749, -122.4194, 37.8199, -122.4783)
    assert result is None
