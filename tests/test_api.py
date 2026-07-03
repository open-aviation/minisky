"""Smoke tests for the FastAPI endpoints (minisky-api.py).

Importing minisky-api.py calls minisky.init() at module import time, which
would clobber the singletons used by the rest of the suite. These tests are
therefore marked 'api' and excluded from the default run; execute them in a
separate process:

    uv run pytest -m api tests/test_api.py

The /stack/{cmd} endpoint requires the async runner loop and is not tested
here (flaky under TestClient).
"""

import importlib.util
from pathlib import Path

import pytest

pytestmark = pytest.mark.api


@pytest.fixture(scope="module")
def client():
    fastapi_testclient = pytest.importorskip("fastapi.testclient")

    # module filename contains a hyphen, so import it via importlib
    api_path = Path(__file__).parent.parent / "minisky-api.py"
    spec = importlib.util.spec_from_file_location("minisky_api", api_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    with fastapi_testclient.TestClient(module.app) as test_client:
        yield test_client


def test_root(client):
    resp = client.get("/")
    assert resp.status_code == 200
    assert "ready" in resp.json()["msg"].lower()


def test_simtime(client):
    resp = client.get("/simtime")
    assert resp.status_code == 200
    value = resp.json()["simulation time (seconds)"]
    assert isinstance(value, (int, float))


def test_all_empty_traffic(client):
    resp = client.get("/all")
    assert resp.status_code == 200
    assert isinstance(resp.json(), list)


def test_all_reflects_created_aircraft(client):
    import minisky

    minisky.traf.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
    resp = client.get("/all")
    assert resp.status_code == 200
    callsigns = [ac["callsign"] for ac in resp.json()]
    assert "KL001" in callsigns


def test_speed_endpoint(client):
    resp = client.get("/speed/10")
    assert resp.status_code == 200
    assert "10" in resp.json()["msg"]


def test_plugins_endpoint(client):
    resp = client.get("/plugins")
    assert resp.status_code == 200
