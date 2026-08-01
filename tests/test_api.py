"""Smoke tests for the FastAPI endpoints.

Use `just test-api`.

The `/stack/{cmd}` endpoint requires the async runner loop and is not tested
here because it is flaky under `TestClient`.
"""

from __future__ import annotations

from collections.abc import Iterator

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from minisky import MiniSky, MiniSkyConfig

pytestmark = pytest.mark.api


@pytest.fixture(scope="module")
def server_app(config: MiniSkyConfig) -> FastAPI:
    from minisky.server import create_app

    return create_app(MiniSky(config))


@pytest.fixture(scope="module")
def runtime(server_app: FastAPI) -> MiniSky:
    return server_app.state.runtime


@pytest.fixture(scope="module")
def client(server_app: FastAPI) -> Iterator[TestClient]:
    fastapi_testclient = pytest.importorskip("fastapi.testclient")

    with fastapi_testclient.TestClient(server_app) as test_client:
        yield test_client


def test_root(client: TestClient) -> None:
    resp = client.get("/")
    assert resp.status_code == 200
    assert "ready" in resp.json()["msg"].lower()


def test_simtime(client: TestClient) -> None:
    resp = client.get("/simtime")
    assert resp.status_code == 200
    value = resp.json()["simulation time (seconds)"]
    assert isinstance(value, (int, float))


def test_all_empty_traffic(client: TestClient) -> None:
    resp = client.get("/all")
    assert resp.status_code == 200
    assert isinstance(resp.json(), list)


def test_all_reflects_created_aircraft(client: TestClient, runtime: MiniSky) -> None:
    runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
    resp = client.get("/all")
    assert resp.status_code == 200
    callsigns = [ac["callsign"] for ac in resp.json()]
    assert "KL001" in callsigns


def test_speed_endpoint(client: TestClient) -> None:
    resp = client.get("/speed/10")
    assert resp.status_code == 200
    assert "10" in resp.json()["msg"]


def test_plugins_endpoint(client: TestClient) -> None:
    resp = client.get("/plugins")
    assert resp.status_code == 200
