"""MiniSky REST + streaming API server.

The FastAPI application wraps an explicit [`MiniSky`][minisky.MiniSky]
runtime and steps it continuously with its async runner while the server is
active. Endpoints expose aircraft state, conflict information, simulation-time
control, plugin management, a passthrough for stack commands, a per-tick push
stream (`GET /stream`, WebSocket), and active command schemas (`GET /commands`).

[`create_app`][.create_app] constructs the application and stores its runtime on
`app.state.runtime`. The supported CLI entry point is `minisky server`.

Run with:

```console
minisky server                 # CLI server command (uvicorn)
minisky server --reload        # development, auto-reload
```

Interactive OpenAPI docs are served at `/docs`.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager, suppress
from typing import Annotated, Literal, TypeAlias, TypedDict

from fastapi import (
    APIRouter,
    Depends,
    FastAPI,
    File,
    Request,
    Response,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)

from minisky import MiniSky
from minisky import quantities as q
from minisky._internal.command import CommandSchema
from minisky._internal.result import Err, Ok, Result
from minisky._internal.streaming import (
    ArrayTopic,
    ConnectionSubscriptions,
    RouteSnapshot,
    RouteTopic,
    Snapshot,
    Topic,
    build_extras,
    build_route_snapshot,
    serialize_array,
)
from minisky.types import AircraftTypeCode, AirspeedKind


def _get_runtime(request: Request) -> MiniSky:
    """Return the runtime owned by the current FastAPI application."""
    runtime: MiniSky = request.app.state.runtime
    return runtime


Runtime = Annotated[MiniSky, Depends(_get_runtime)]

# we are using adjacently tagged enums for compatability
# TODO(abraham): use externally tagged once we remove the html


class OkResultResponse(TypedDict):
    """JSON representation of a successful string result."""

    ok: Literal[True]
    value: str


class ErrResultResponse(TypedDict):
    """JSON representation of an unsuccessful string result."""

    ok: Literal[False]
    error: str


ResultResponse: TypeAlias = OkResultResponse | ErrResultResponse


class SelectedCasResponse(TypedDict):
    kind: Literal["CAS"]
    mps: q.CalibratedAirspeedMps[float]


class SelectedMachResponse(TypedDict):
    kind: Literal["MACH"]
    mach: q.MachNumber[float]


SelectedAirspeedResponse: TypeAlias = SelectedCasResponse | SelectedMachResponse


AircraftResponse = TypedDict(
    "AircraftResponse",
    {
        "callsign": str,
        "typecode": AircraftTypeCode,
        "latitude": q.LatitudeDeg[float],
        "longitude": q.LongitudeDeg[float],
        "altitude (feet)": q.PressureAltitudeFt[int],
        "heading (degrees)": q.TrueHeadingDegrees[int],
        "assigned heading (degrees)": q.TrueHeadingDegrees[int],
        "track (degrees)": q.GroundTrackDeg[float],
        "TAS (knots)": q.TrueAirspeedKt[int],
        "groundspeed (knots)": q.GroundSpeedKt[int],
        "CAS (knots)": q.CalibratedAirspeedKt[int],
        "mach": q.MachNumber[float],
        "vertical_rate (feet/minute)": q.VerticalRateFpm[int],
        "target altitude (feet)": q.PressureAltitudeFt[int],
        "selected airspeed": SelectedAirspeedResponse,
    },
)

ConflictResponse = TypedDict(
    "ConflictResponse",
    {
        "conflict pairs": tuple[str, str],
        "distance (nautical miles)": q.DistanceNM[float],
        "altitude difference (feet)": q.VerticalDistanceFt[float],
        "qdr (degrees)": q.BearingDeg[float],
        "tlos (seconds)": q.DurationS[float],
        "dcpa (meters)": q.DistanceM[float],
        "tcpa (seconds)": q.DurationS[float],
    },
)


def _result_response(result: Result[str, str]) -> ResultResponse:
    match result:
        case Ok(value):
            return {"ok": True, "value": value}
        case Err(error):
            return {"ok": False, "error": error}


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Run the app-owned simulator for the lifetime of the API server."""
    runtime: MiniSky = app.state.runtime
    await runtime.plugins.load_configured()
    runner_task = asyncio.create_task(runtime.run())
    try:
        yield
    finally:
        errors: list[Exception] = []
        runner_task.cancel()
        with suppress(asyncio.CancelledError):
            try:
                await runner_task
            except Exception as exc:  # ruff: ignore[BLE001] aggregate server cleanup failures
                errors.append(exc)
        try:
            await runtime.aclose()
        except Exception as exc:  # ruff: ignore[BLE001] aggregate server cleanup failures
            errors.append(exc)
        if len(errors) == 1:
            raise errors[0]
        if errors:
            raise ExceptionGroup("MiniSky server shutdown failed", errors)


def create_app(runtime: MiniSky) -> FastAPI:
    """Create a FastAPI application owning a simulator runtime."""
    runtime.runner.prevent_shutdown()

    app = FastAPI(lifespan=lifespan)
    app.state.runtime = runtime
    app.include_router(create_router())

    return app


def root() -> dict[str, str]:
    """Health check: confirm the API is up."""
    return {"msg": "MiniSky API endpoint ready"}


def all_aircraft(runtime: Runtime) -> list[AircraftResponse]:
    """Get all aircraft states in the aviation units used by the REST API."""
    traffic = runtime.traffic
    aircraft: list[AircraftResponse] = []
    for i, (callsign, typecode) in enumerate(zip(traffic.callsign, traffic.typecode, strict=True)):
        altitude_ft: q.PressureAltitudeFt[float] = q.m_to_ft(float(traffic.alt[i]))
        tas_kt: q.TrueAirspeedKt[float] = q.mps_to_kt(float(traffic.tas[i]))
        groundspeed_kt: q.GroundSpeedKt[float] = q.mps_to_kt(float(traffic.gs[i]))
        cas_kt: q.CalibratedAirspeedKt[float] = q.mps_to_kt(float(traffic.cas[i]))
        vertical_rate_fpm: q.VerticalRateFpm[float] = q.mps_to_fpm(float(traffic.vs[i]))
        target_altitude_ft: q.PressureAltitudeFt[float] = q.m_to_ft(float(traffic.selalt[i]))
        if traffic.selected_airspeed.kind[i] == AirspeedKind.CAS:
            selected_airspeed: SelectedAirspeedResponse = {
                "kind": "CAS",
                "mps": float(traffic.selected_airspeed.values[i]),
            }
        else:
            selected_airspeed = {
                "kind": "MACH",
                "mach": float(traffic.selected_airspeed.values[i]),
            }
        aircraft.append(
            {
                "callsign": callsign,
                "typecode": typecode,
                "latitude": float(traffic.lat[i]),
                "longitude": float(traffic.lon[i]),
                "altitude (feet)": int(altitude_ft),
                "heading (degrees)": int(traffic.hdg[i]),
                "assigned heading (degrees)": int(traffic.aporasas.hdg[i]),
                "track (degrees)": float(traffic.trk[i]),
                "TAS (knots)": int(tas_kt),
                "groundspeed (knots)": int(groundspeed_kt),
                "CAS (knots)": int(cas_kt),
                "mach": float(traffic.M[i]),
                "vertical_rate (feet/minute)": int(vertical_rate_fpm),
                "target altitude (feet)": int(target_altitude_ft),
                "selected airspeed": selected_airspeed,
            }
        )
    return aircraft


def simtime(runtime: Runtime) -> dict[str, q.SimulationTimeS[float]]:
    """Get the simulation time."""
    return {"simulation time (seconds)": runtime.simulation.simt}


def speedup(speed: float, runtime: Runtime) -> dict[str, str]:
    """Speed up the simulation."""
    runtime.runner.speed = speed
    return {"msg": f"simulation speed set to {speed}x"}


def forward(seconds: q.DurationS[float], runtime: Runtime) -> dict[str, str]:
    """Jump to a specific simulation time."""
    runtime.runner.forward(seconds)
    return {"msg": f"simulation time jump forward {seconds} seconds"}


def conflicts(runtime: Runtime) -> list[ConflictResponse] | dict[str, str]:
    """Get all detected conflicts with explicit aviation-unit output fields."""
    detection = runtime.traffic.cd
    if not detection.confpairs:
        return {"msg": "No conflicts detected"}

    if len(detection.tcpa) == 0:
        return {"msg": "No TCPA data available"}

    processed_pairs: list[set[str]] = []
    conflict_info: list[ConflictResponse] = []

    for i, pair in enumerate(detection.confpairs):
        if set(pair) in processed_pairs:
            continue

        processed_pairs.append(set(pair))
        distance_nm: q.DistanceNM[float] = q.m_to_nmi(float(detection.dist[i]))
        altitude_difference_ft: q.VerticalDistanceFt[float] = q.m_to_ft(float(detection.dalt[i]))
        conflict_info.append(
            {
                "conflict pairs": pair,
                "distance (nautical miles)": distance_nm,
                "altitude difference (feet)": altitude_difference_ft,
                "qdr (degrees)": float(detection.qdr[i]),
                "tlos (seconds)": float(detection.tLOS[i]),
                "dcpa (meters)": float(detection.dcpa[i]),
                "tcpa (seconds)": float(detection.tcpa[i]),
            }
        )
    return conflict_info


def traffic_arrays(runtime: Runtime) -> list[str]:
    """List every traffic array name currently registered, core or plugin-added."""
    return runtime.traffic.array_names()


def traffic_array(name: str, runtime: Runtime) -> list[float] | dict[str, str]:
    """Get the full values of one registered traffic array."""
    array = runtime.traffic.find_array(name)
    if array is None:
        return {"msg": f"no traffic array named {name!r}"}
    return serialize_array(array)


def traffic_array_value(name: str, callsign: str, runtime: Runtime) -> float | dict[str, str]:
    """Get one aircraft's value from a registered traffic array."""
    array = runtime.traffic.find_array(name)
    if array is None:
        return {"msg": f"no traffic array named {name!r}"}
    idx = runtime.traffic.idx(callsign)
    if idx is None:
        return {"msg": f"no aircraft with callsign {callsign!r}"}
    return serialize_array(array)[idx]


def traffic_route(callsign: str, runtime: Runtime) -> RouteSnapshot | dict[str, str]:
    """Get one aircraft's route (waypoints and the active-waypoint index)."""
    route = build_route_snapshot(runtime.traffic, callsign)
    if route is None:
        return {"msg": f"no aircraft with callsign {callsign!r}"}
    return route


async def stack(cmd: str, runtime: Runtime) -> ResultResponse:
    """Execute a stack command and return its completion result."""
    return _result_response(await runtime.commands.submit(cmd))


def commands(runtime: Runtime) -> dict[str, CommandSchema]:
    """Return command schemas for core and currently loaded plugins."""
    return runtime.commands.command_schemas()


def _parse_topic(data: object) -> Topic | None:
    """Parse a client-provided `topic` object, or `None` if it's malformed."""
    if not isinstance(data, dict):
        return None
    kind = data.get("kind")
    if kind == "route":
        callsign = data.get("callsign")
        if not isinstance(callsign, str):
            return None
        return RouteTopic(kind="route", callsign=callsign)
    if kind == "array":
        name = data.get("name")
        callsign = data.get("callsign")
        if not isinstance(name, str) or (callsign is not None and not isinstance(callsign, str)):
            return None
        return ArrayTopic(kind="array", name=name, callsign=callsign)
    return None


def _handle_control_message(subscriptions: ConnectionSubscriptions, message: object) -> None:
    """Apply a client control frame (subscribe/unsubscribe), ignoring malformed ones."""
    if not isinstance(message, dict):
        return
    topic = _parse_topic(message.get("topic"))
    if topic is None:
        return
    action = message.get("action")
    if action == "subscribe":
        ttl = message.get("ttl")
        subscriptions.subscribe(topic, ttl=float(ttl) if isinstance(ttl, (int, float)) else None)
    elif action == "unsubscribe":
        subscriptions.unsubscribe(topic)


async def stream(websocket: WebSocket) -> None:
    """Push a full simulation snapshot once per simulation step in SI units.

    Emits a JSON message per published tick, rate-capped by
    `STREAM_MAX_HZ`, containing the [`Snapshot`][minisky.Snapshot] `siminfo`
    and `acdata` fields.
    The most recent snapshot is sent immediately on connect so a new client is
    not left blank until the next tick.

    A connected client may also send small JSON control frames to layer
    per-connection extras onto its own outgoing ticks, under an `extras` key,
    until it sends a matching `unsubscribe` (or its optional `ttl` elapses):

    ```json
    {"action": "subscribe", "topic": {"kind": "route", "callsign": "KL001"}}
    {"action": "subscribe", "topic": {"kind": "array", "name": "tcpamax", "callsign": "KL001"}, "ttl": 30}
    {"action": "unsubscribe", "topic": {"kind": "route", "callsign": "KL001"}}
    ```

    Clients that never send a control frame see no `extras` key: the tick
    payload is unchanged from the base snapshot.
    """
    runtime: MiniSky = websocket.app.state.runtime
    hub = runtime.streaming
    subscriptions = ConnectionSubscriptions()
    await websocket.accept()
    hub.subscribe()

    def outgoing() -> Snapshot | dict[str, object]:
        assert hub.latest is not None
        extras = build_extras(subscriptions, runtime.traffic)
        return {**hub.latest, "extras": extras} if extras else hub.latest

    tick_task = asyncio.create_task(hub.wait())
    recv_task = asyncio.create_task(websocket.receive_json())
    try:
        if hub.latest is not None:
            await websocket.send_json(outgoing())
        while True:
            done, _pending = await asyncio.wait(
                {tick_task, recv_task}, return_when=asyncio.FIRST_COMPLETED
            )
            if tick_task in done:
                tick_task.result()  # re-raises RuntimeError once the hub is closed
                if hub.latest is not None:
                    await websocket.send_json(outgoing())
                tick_task = asyncio.create_task(hub.wait())
            if recv_task in done:
                try:
                    message = recv_task.result()
                except WebSocketDisconnect:
                    raise
                except Exception:  # ruff: ignore[BLE001] malformed control frame, not a disconnect
                    message = None
                if message is not None:
                    _handle_control_message(subscriptions, message)
                recv_task = asyncio.create_task(websocket.receive_json())
    except (WebSocketDisconnect, RuntimeError):
        # RuntimeError: the transport closed between the disconnect and our
        # next send (uvicorn raises it instead of WebSocketDisconnect).
        pass
    finally:
        # Deliberately not awaited: this coroutine may itself be getting torn
        # down under cancellation right now (e.g. a client disconnecting),
        # and awaiting here would re-raise whatever exception either task
        # already carries (which could be a stale WebSocketDisconnect, not a
        # fresh CancelledError) over the original exception. `cancel()` alone
        # is enough for asyncio to reap both tasks without complaint.
        tick_task.cancel()
        recv_task.cancel()
        hub.unsubscribe()


def upload_form() -> Response:
    """Serve a minimal HTML form for uploading a scenario file."""
    content = """
    upload a scenario file<hr>
    <form method="post" enctype="multipart/form-data">
        <input type="file" name="file">
        <input type="submit" value="submit">
    </form>
    """
    return Response(content=content, media_type="text/html")


async def scn(runtime: Runtime, file: Annotated[UploadFile, File()]) -> dict[str, str]:
    """Load an uploaded scenario file into the running simulation."""
    contents = await file.read()
    filename = file.filename or "uploaded.scn"
    runtime.commands.load_scenario_text(contents.decode("utf-8"), name=filename)
    return {"msg": f"scenario {filename} loaded"}


def list_plugins(runtime: Runtime) -> ResultResponse:
    """List available and loaded plugins."""
    result = runtime.plugins.listing()
    return _result_response(result)


async def load_plugin(name: str, runtime: Runtime) -> ResultResponse:
    """Load a plugin by name."""
    result = await runtime.plugins.load(name)
    return _result_response(result)


def create_router() -> APIRouter:
    """Create the API router for a FastAPI application."""
    router = APIRouter()
    router.add_api_route("/", root, methods=["GET"])
    router.add_api_route("/all", all_aircraft, methods=["GET"])
    router.add_api_route("/simtime", simtime, methods=["GET"])
    router.add_api_route("/speed/{speed}", speedup, methods=["GET"])
    router.add_api_route("/forward/{seconds}", forward, methods=["GET"])
    router.add_api_route("/conflicts", conflicts, methods=["GET"])
    router.add_api_route("/traffic/arrays", traffic_arrays, methods=["GET"])
    router.add_api_route("/traffic/arrays/{name}", traffic_array, methods=["GET"])
    router.add_api_route("/traffic/arrays/{name}/{callsign}", traffic_array_value, methods=["GET"])
    router.add_api_route("/traffic/route/{callsign}", traffic_route, methods=["GET"])
    router.add_api_route("/stack/{cmd:path}", stack, methods=["GET"])
    router.add_api_route("/commands", commands, methods=["GET"])
    router.add_api_websocket_route("/stream", stream)
    router.add_api_route("/scn", upload_form, methods=["GET"])
    router.add_api_route("/scn", scn, methods=["POST"])
    router.add_api_route("/plugins", list_plugins, methods=["GET"])
    router.add_api_route("/plugins/load/{name}", load_plugin, methods=["GET"])
    return router
