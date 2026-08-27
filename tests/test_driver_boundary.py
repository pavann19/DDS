"""ADR-001 Gate 6.5.4 -- the World/Driver boundary is a type signature,
not a comment.

The Driver (``app/services/driver/``) may only ever act on the
sensor-resolved picture. It must have no code path that names, imports, or
accepts a ``TrafficModel`` or an ``NpcVehicle`` -- those are ground truth
owned by the World. This test fails loudly if a future change reaches
across that line.
"""
import ast
import inspect
import pathlib

import app.services.driver as driver_pkg

_FORBIDDEN_NAMES = {"TrafficModel", "NpcVehicle", "sense_lead_vehicle", "get_npc_states"}
_DRIVER_DIR = pathlib.Path(driver_pkg.__file__).parent


def _driver_source_files():
    return sorted(_DRIVER_DIR.glob("*.py"))


def test_driver_package_has_source_files():
    files = _driver_source_files()
    assert files, "expected driver/ modules to exist"


def test_no_driver_module_references_world_ground_truth():
    offenders = {}
    for path in _driver_source_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        hits = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Name) and node.id in _FORBIDDEN_NAMES:
                hits.add(node.id)
            elif isinstance(node, ast.Attribute) and node.attr in _FORBIDDEN_NAMES:
                hits.add(node.attr)
            elif isinstance(node, (ast.Import, ast.ImportFrom)):
                mod = getattr(node, "module", "") or ""
                if mod.endswith("traffic"):
                    for alias in node.names:
                        if alias.name in _FORBIDDEN_NAMES:
                            hits.add(alias.name)
        if hits:
            offenders[path.name] = sorted(hits)
    assert not offenders, f"driver modules reach into World ground truth: {offenders}"


def test_no_driver_module_imports_the_traffic_module_at_all():
    for path in _driver_source_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and (node.module or "").endswith("services.traffic"):
                raise AssertionError(f"{path.name} imports app.services.traffic")
            if isinstance(node, ast.Import):
                for alias in node.names:
                    assert not alias.name.endswith("services.traffic"), path.name


def test_public_driver_entrypoints_take_no_traffic_typed_parameters():
    from app.services.driver import SafetyMonitor, plan_lateral_offset

    for fn in (plan_lateral_offset, SafetyMonitor.step):
        params = inspect.signature(fn).parameters
        bad = [p for p in params if "traffic" in p.lower() or "npc" in p.lower()]
        assert not bad, f"{fn.__qualname__} exposes traffic-shaped params: {bad}"
