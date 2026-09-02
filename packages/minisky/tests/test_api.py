"""Smoke tests for the FastAPI endpoints.

Use `just test-api`.

The `/stack/{cmd}` endpoint requires the async runner loop and is not tested
here because it is flaky under the in-process ASGI client.
"""

from __future__ import annotations

from collections.abc import AsyncIterator

import httpx2
import pytest
from fastapi import FastAPI
from minisky import MiniSky, MiniSkyConfig
from minisky.types import CasMps, StdPressureAltM

pytestmark = pytest.mark.api


@pytest.fixture(scope="module")
def server_app(config: MiniSkyConfig) -> FastAPI:
    from minisky._internal.server import create_app

    return create_app(MiniSky(config))


@pytest.fixture(scope="module")
def runtime(server_app: FastAPI) -> MiniSky:
    return server_app.state.runtime


@pytest.fixture
async def client(server_app: FastAPI) -> AsyncIterator[httpx2.AsyncClient]:
    transport = httpx2.ASGITransport(app=server_app)
    async with httpx2.AsyncClient(transport=transport, base_url="http://test") as test_client:
        yield test_client


@pytest.mark.anyio
async def test_commands_returns_structured_schema(client: httpx2.AsyncClient) -> None:
    resp = await client.get("/commands")

    assert resp.status_code == 200
    schema = resp.json()["minisky"]
    assert "commands" in schema
    assert "definitions" in schema
    assert "CRE" in schema["commands"]


@pytest.mark.anyio
async def test_all_reflects_created_aircraft(client: httpx2.AsyncClient, runtime: MiniSky) -> None:
    runtime.traffic.cre(
        "KL001",
        "A320",
        lat=52.0,
        lon=4.0,
        hdg=90,
        alt=StdPressureAltM(3000.0),
        airspeed=CasMps(150.0),
    )
    resp = await client.get("/all")
    assert resp.status_code == 200
    aircraft = next(ac for ac in resp.json() if ac["callsign"] == "KL001")
    assert aircraft["selected airspeed"] == {"kind": "CAS", "mps": 150.0}
