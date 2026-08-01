"""MiniSky REST + streaming API server.

The FastAPI application wraps an explicit [`MiniSky`][minisky.MiniSky]
runtime and steps it continuously with its async runner while the server is
active. Endpoints expose aircraft state, conflict information, simulation-time
control, plugin management, a passthrough for stack commands, a per-tick push
stream (`GET /stream`, WebSocket), and the command dictionary (`GET /commands`).

[`create_app`][] constructs the application and stores its runtime on
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
import os
from contextlib import asynccontextmanager, suppress
from io import StringIO
from typing import Annotated, Any, cast

import pandas as pd
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
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles

from minisky import MiniSky
from minisky.tools import aero


def _get_runtime(request: Request) -> MiniSky:
    """Return the runtime owned by the current FastAPI application."""
    return cast(MiniSky, request.app.state.runtime)


Runtime = Annotated[MiniSky, Depends(_get_runtime)]


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Run the app-owned simulator for the lifetime of the API server."""
    runtime = cast(MiniSky, app.state.runtime)
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


def create_app(runtime: MiniSky | None = None) -> FastAPI:
    """Create a FastAPI application owning a simulator runtime."""
    if runtime is None:
        runtime = MiniSky()

    app = FastAPI(lifespan=lifespan)
    app.state.runtime = runtime
    app.include_router(create_router())

    # TODO(abraham): package static assets inside minisky and resolve them
    # with importlib.resources for wheel installs
    static_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "static")
    os.makedirs(static_dir, exist_ok=True)
    app.mount("/static", StaticFiles(directory=static_dir), name="static")
    return app


def root() -> dict[str, str]:
    """Health check: confirm the API is up."""
    return {"msg": "MiniSky API endpoint ready"}


def all(runtime: Runtime) -> list[dict[str, Any]]:
    """Get all aircraft states."""
    traffic = runtime.traffic
    df = pd.DataFrame(
        {
            "callsign": traffic.callsign,
            "typecode": traffic.typecode,
            "latitude": traffic.lat,
            "longitude": traffic.lon,
            "altitude (feet)": (traffic.alt / aero.ft).astype(int),
            "heading (degrees)": traffic.hdg.astype(int),
            "assigned heading (degrees)": traffic.aporasas.hdg.astype(int),
            "track (degrees)": traffic.trk,
            "TAS (knots)": (traffic.tas / aero.kts).astype(int),
            "groundspeed (knots)": (traffic.gs / aero.kts).astype(int),
            "CAS (knots)": (traffic.cas / aero.kts).astype(int),
            "mach": traffic.M,
            "vertical_rate (feet/minute)": (traffic.vs / aero.fpm).astype(int),
            "target altitude (feet)": (traffic.selalt / aero.ft).astype(int),
            "assigned speed (knots)": (traffic.selspd / aero.kts).astype(int),
        }
    )

    return df.to_dict(orient="records")


def simtime(runtime: Runtime) -> dict[str, float]:
    """Get the simulation time."""
    return {"simulation time (seconds)": runtime.simulation.simt}


def speedup(speed: float, runtime: Runtime) -> dict[str, str]:
    """Speed up the simulation."""
    runtime.runner.speed = speed
    return {"msg": f"simulation speed set to {speed}x"}


def forward(seconds: float, runtime: Runtime) -> dict[str, str]:
    """Jump to a specific simulation time."""
    runtime.runner.forward(seconds)
    return {"msg": f"simulation time jump forward {seconds} seconds"}


def conflicts(runtime: Runtime) -> list[dict[str, Any]] | dict[str, str]:
    """Get all detected conflicts.

    Returns one record per unique aircraft pair with distance (NM), altitude
    difference (ft), bearing (deg), time to loss of separation (s), and distance and
    time to the closest point of approach (m, s).
    """
    detection = runtime.traffic.cd
    if not detection.confpairs:
        return {"msg": "No conflicts detected"}

    if len(detection.tcpa) == 0:
        return {"msg": "No TCPA data available"}

    processed_pairs = []
    conflict_info = []

    for i, pair in enumerate(detection.confpairs):
        if set(pair) in processed_pairs:
            continue

        processed_pairs.append(set(pair))

        conflict_info.append(
            {
                "conflict pairs": pair,
                "distance (nautical miles)": detection.dist[i] / aero.nm,
                "altitude difference (feet)": detection.dalt[i] / aero.ft,
                "qdr (degrees)": detection.qdr[i],
                "tlos (seconds)": detection.tLOS[i],
                "dcpa (meters)": detection.dcpa[i],
                "tcpa (seconds)": detection.tcpa[i],
            }
        )
    return conflict_info


async def stack(cmd: str, runtime: Runtime) -> dict[str, Any]:
    """Execute a stack command and return the output."""
    runtime.console.event.clear()
    runtime.commands.stack(cmd)
    await runtime.console.event.wait()
    msg = runtime.console.read_output_buffer()
    runtime.console.event.clear()
    return {"command to minisky": cmd, "message": msg}


def commands(runtime: Runtime) -> dict[str, str]:
    """Return the command dictionary as `{name: brief usage}`.

    Deduplicates aliases, which share a [`Command`][minisky.stack.Command]
    object, and reports each command under its canonical name so a console or
    autocomplete client can list the available commands and their usage.
    """
    seen: dict[str, str] = {}
    for cmdobj in dict.fromkeys(runtime.commands.cmddict.values()):
        seen[cmdobj.name] = cmdobj.brief
    return dict(sorted(seen.items()))


async def stream(websocket: WebSocket) -> None:
    """Push a full simulation snapshot once per simulation step in SI units.

    Emits a JSON message per published tick, rate-capped by
    [`STREAM_MAX_HZ`][minisky.streaming.STREAM_MAX_HZ], containing `siminfo`
    and `acdata` as built by [`build_snapshot`][minisky.streaming.build_snapshot].
    The most recent snapshot is sent immediately on connect so a new client is
    not left blank until the next tick.
    """
    runtime = cast(MiniSky, websocket.app.state.runtime)
    hub = runtime.streaming
    await websocket.accept()
    hub.subscribe()
    try:
        if hub.latest is not None:
            await websocket.send_json(hub.latest)
        while True:
            await hub.wait()
            if hub.latest is not None:
                await websocket.send_json(hub.latest)
    except (WebSocketDisconnect, RuntimeError):
        # RuntimeError: the transport closed between the disconnect and our
        # next send (uvicorn raises it instead of WebSocketDisconnect).
        pass
    finally:
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
    runtime.console.event.clear()
    contents = await file.read()
    scenario = StringIO(contents.decode("utf-8"))
    filename = file.filename or "uploaded.scn"
    runtime.commands.ic_StringIO(scenario, filename)
    return {"msg": f"scenario {filename} loaded"}


def show_map() -> RedirectResponse:
    """Display the aircraft map viewer."""
    return RedirectResponse(url="/static/display.html")


def list_plugins(runtime: Runtime) -> Any:
    """List available and loaded plugins."""
    return runtime.plugins.manage("LIST")


async def load_plugin(name: str, runtime: Runtime) -> Any:
    """Load a plugin by name."""
    return await runtime.plugins.load(name)


def create_router() -> APIRouter:
    """Create the API router for a FastAPI application."""
    router = APIRouter()
    router.add_api_route("/", root, methods=["GET"])
    router.add_api_route("/all", all, methods=["GET"])
    router.add_api_route("/simtime", simtime, methods=["GET"])
    router.add_api_route("/speed/{speed}", speedup, methods=["GET"])
    router.add_api_route("/forward/{seconds}", forward, methods=["GET"])
    router.add_api_route("/conflicts", conflicts, methods=["GET"])
    router.add_api_route("/stack/{cmd:path}", stack, methods=["GET"])
    router.add_api_route("/commands", commands, methods=["GET"])
    router.add_api_websocket_route("/stream", stream)
    router.add_api_route("/scn", upload_form, methods=["GET"])
    router.add_api_route("/scn", scn, methods=["POST"])
    router.add_api_route("/map", show_map, methods=["GET"])
    router.add_api_route("/plugins", list_plugins, methods=["GET"])
    router.add_api_route("/plugins/load/{name}", load_plugin, methods=["GET"])
    return router


def main() -> None:
    """Console-script entry point: serve the API with uvicorn.

    Host and port are read from `MINISKY_HOST` (default `0.0.0.0`) and
    `MINISKY_PORT` (default `8000`).
    """
    import uvicorn

    host = os.environ.get("MINISKY_HOST", "0.0.0.0")
    port = int(os.environ.get("MINISKY_PORT", "8000"))
    uvicorn.run("minisky.server:create_app", factory=True, host=host, port=port)


if __name__ == "__main__":
    main()
