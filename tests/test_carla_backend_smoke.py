"""CARLA smoke test -- SKIPS when the ``carla`` client package is not
installed (it cannot be, on this machine: Python 3.13 + AMD GPU, see
docs/CARLA_FEASIBILITY.md).

When run on a CARLA host (0.9.15, Python 3.10 client venv, server up on
:2000) these become the observe -> decide -> control -> feedback validation
the migration completion criterion requires.
"""
import pytest

carla = pytest.importorskip(
    "carla",
    reason="carla client not installed -- see docs/CARLA_FEASIBILITY.md",
)


@pytest.fixture(scope="module")
def carla_backend():
    from app.services.carla_adapter.carla_backend import CarlaBackend
    b = CarlaBackend(fixed_delta_seconds=0.05, sensor_mode="actor_state")
    yield b
    b.close()


def test_carla_connects_and_loads_a_town(carla_backend):
    from app.services.carla_adapter.route import CarlaRouteProvider  # noqa: F401
    # carla_backend.reset(route_waypoints=..., start_speed_mps=0.0)
    # assert carla_backend.observe().frenet_frame is not None
    pytest.skip("implement on a CARLA host -- docs/CARLA_MIGRATION_PLAN.md")


def test_ego_control_moves_the_vehicle(carla_backend):
    pytest.skip("implement on a CARLA host: apply throttle, step, assert speed rises")


def test_observation_is_sensor_only_from_carla(carla_backend):
    pytest.skip("implement on a CARLA host: teleport a CARLA actor, assert observe() unchanged")
