"""Smoke tests for the FastAPI endpoints.

Use `just test-api`.
"""

from __future__ import annotations

from collections.abc import AsyncIterator

import httpx2
import pytest
from fastapi import FastAPI
from minisky import MagneticDeclinationGrid, MiniSky, MiniSkyConfig, NavData
from minisky._internal.streaming import build_snapshot
from minisky.types import CasMps, StdPressureAltM
from starlette.testclient import TestClient

pytestmark = pytest.mark.api


@pytest.fixture(scope="module")
def server_app(config: MiniSkyConfig) -> FastAPI:
    from minisky._internal.server import create_app

    return create_app(
        MiniSky(
            config,
            navdata=NavData(),
            magnetic_declination=MagneticDeclinationGrid.load_default(),
        )
    )


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


@pytest.mark.anyio
async def test_traffic_arrays_lists_registered_names(client: httpx2.AsyncClient) -> None:
    resp = await client.get("/traffic/arrays")
    assert resp.status_code == 200
    names = resp.json()
    assert "lat" in names
    assert "tcpamax" in names


@pytest.mark.anyio
async def test_traffic_array_returns_full_array(
    client: httpx2.AsyncClient, runtime: MiniSky
) -> None:
    runtime.traffic.cre(
        "KL002",
        "A320",
        lat=52.0,
        lon=4.0,
        hdg=90,
        alt=StdPressureAltM(3000.0),
        airspeed=CasMps(150.0),
    )
    resp = await client.get("/traffic/arrays/lat")
    assert resp.status_code == 200
    idx = runtime.traffic.idx("KL002")
    assert resp.json()[idx] == pytest.approx(52.0)


@pytest.mark.anyio
async def test_traffic_array_unknown_name_returns_message(client: httpx2.AsyncClient) -> None:
    resp = await client.get("/traffic/arrays/not_a_real_array")
    assert resp.status_code == 200
    assert "msg" in resp.json()


@pytest.mark.anyio
async def test_traffic_array_value_for_callsign(
    client: httpx2.AsyncClient, runtime: MiniSky
) -> None:
    runtime.traffic.cre(
        "KL003",
        "A320",
        lat=53.0,
        lon=4.0,
        hdg=90,
        alt=StdPressureAltM(3000.0),
        airspeed=CasMps(150.0),
    )
    resp = await client.get("/traffic/arrays/lat/KL003")
    assert resp.status_code == 200
    assert resp.json() == pytest.approx(53.0)


@pytest.mark.anyio
async def test_traffic_array_value_unknown_callsign_returns_message(
    client: httpx2.AsyncClient,
) -> None:
    resp = await client.get("/traffic/arrays/lat/GHOST")
    assert resp.status_code == 200
    assert "msg" in resp.json()


@pytest.mark.anyio
async def test_traffic_route_reflects_added_waypoints(
    client: httpx2.AsyncClient, runtime: MiniSky
) -> None:
    runtime.traffic.cre(
        "KL004",
        "A320",
        lat=52.0,
        lon=4.0,
        hdg=90,
        alt=StdPressureAltM(3000.0),
        airspeed=CasMps(150.0),
    )
    invocation = runtime.commands.submit("ADDWPT KL004 52.5,5.0")
    runtime.simulation.step()
    await invocation

    resp = await client.get("/traffic/route/KL004")
    assert resp.status_code == 200
    route = resp.json()
    assert route["callsign"] == "KL004"
    assert len(route["waypoints"]) == 1
    assert route["waypoints"][0]["lat"] == pytest.approx(52.5)


@pytest.mark.anyio
async def test_traffic_route_unknown_callsign_returns_message(client: httpx2.AsyncClient) -> None:
    resp = await client.get("/traffic/route/GHOST")
    assert resp.status_code == 200
    assert "msg" in resp.json()


def test_stream_subscribe_and_unsubscribe_array_extras(
    server_app: FastAPI, runtime: MiniSky
) -> None:
    """A control frame on `/stream` adds/removes a per-connection extra topic.

    Ticks are published explicitly (bypassing the sim-step-driven rate cap)
    via the websocket's own portal, so the publish and the server's handler
    run on the same event loop/thread instead of racing across threads.
    """
    runtime.traffic.cre(
        "KL010",
        "A320",
        lat=52.0,
        lon=4.0,
        hdg=90,
        alt=StdPressureAltM(3000.0),
        airspeed=CasMps(150.0),
    )

    async def publish_tick() -> None:
        runtime.streaming.publish(
            build_snapshot(runtime.simulation, runtime.traffic, runtime.runner, runtime.commands)
        )

    def drain_until(ws, predicate, max_tries: int = 10):
        for _ in range(max_tries):
            ws.portal.call(publish_tick)
            tick = ws.receive_json()
            if predicate(tick):
                return tick
        pytest.fail("condition not met within retry budget")

    with TestClient(server_app).websocket_connect("/stream") as ws:
        first = drain_until(ws, lambda _tick: True, max_tries=1)
        assert "extras" not in first

        ws.send_json(
            {"action": "subscribe", "topic": {"kind": "array", "name": "lat", "callsign": "KL010"}}
        )
        tick = drain_until(ws, lambda t: "array:lat:KL010" in t.get("extras", {}))
        assert tick["extras"]["array:lat:KL010"] == pytest.approx(52.0)

        ws.send_json(
            {
                "action": "unsubscribe",
                "topic": {"kind": "array", "name": "lat", "callsign": "KL010"},
            }
        )
        tick = drain_until(ws, lambda t: "array:lat:KL010" not in t.get("extras", {}))
        assert "extras" not in tick
