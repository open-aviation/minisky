"""MiniSky REST + streaming API server.

The FastAPI application wraps an explicit [`MiniSky`][minisky.runtime.MiniSky]
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

from minisky import MiniSky, MiniSkySettings, filename_settings, plugin
from minisky.tools import aero

router = APIRouter()


def _get_runtime(request: Request) -> MiniSky:
    """Return the runtime owned by the current FastAPI application."""
    return cast(MiniSky, request.app.state.runtime)


Runtime = Annotated[MiniSky, Depends(_get_runtime)]


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Run the app-owned simulator for the lifetime of the API server."""
    runtime = cast(MiniSky, app.state.runtime)
    task = asyncio.create_task(runtime.run())
    try:
        yield
    finally:
        runtime.runner.running = False
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task


def create_app(runtime: MiniSky | None = None) -> FastAPI:
    """Create a FastAPI application owning a simulator runtime."""
    if runtime is None:
        settings = MiniSkySettings.from_file(filename_settings)
        runtime = MiniSky(settings)

    # TODO(abraham): migrate the plugin ownership
    plugin.discover()
    plugin.load_enabled()

    app = FastAPI(lifespan=lifespan)
    app.state.runtime = runtime
    app.include_router(router)

    # Static files live at the repository root (../static relative to this
    # package), which resolves correctly for both a source checkout and an
    # editable install.
    static_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "static")
    os.makedirs(static_dir, exist_ok=True)
    app.mount("/static", StaticFiles(directory=static_dir), name="static")
    return app


@router.get("/")
def root() -> dict[str, str]:
    """Health check: confirm the API is up."""
    return {"msg": "MiniSky API endpoint ready"}


@router.get("/all")
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


@router.get("/simtime")
def simtime(runtime: Runtime) -> dict[str, float]:
    """Get the simulation time."""
    return {"simulation time (seconds)": runtime.simulation.simt}


@router.get("/speed/{speed}")
def speedup(speed: float, runtime: Runtime) -> dict[str, str]:
    """Speed up the simulation."""
    runtime.runner.speed = speed
    return {"msg": f"simulation speed set to {speed}x"}


@router.get("/forward/{seconds}")
def forward(seconds: float, runtime: Runtime) -> dict[str, str]:
    """Jump to a specific simulation time."""
    runtime.runner.forward(seconds)
    return {"msg": f"simulation time jump forward {seconds} seconds"}


@router.get("/conflicts")
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


@router.get("/stack/{cmd:path}")
async def stack(cmd: str, runtime: Runtime) -> dict[str, Any]:
    """Execute a stack command and return the output."""
    runtime.console.event.clear()
    runtime.commands.stack(cmd)
    await runtime.console.event.wait()
    msg = runtime.console.read_output_buffer()
    runtime.console.event.clear()
    return {"command to minisky": cmd, "message": msg}


@router.get("/commands")
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


@router.websocket("/stream")
async def stream(websocket: WebSocket) -> None:
    """Push a full simulation snapshot once per simulation step in SI units.

    Emits one JSON message per published tick, rate-capped by
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


@router.get("/scn")
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


@router.post("/scn")
async def scn(runtime: Runtime, file: UploadFile = File(...)) -> dict[str, str]:
    """Load an uploaded scenario file into the running simulation."""
    runtime.console.event.clear()
    contents = await file.read()
    scenario = StringIO(contents.decode("utf-8"))
    filename = file.filename or "uploaded.scn"
    runtime.commands.ic_StringIO(scenario, filename)
    return {"msg": f"scenario {filename} loaded"}


@router.get("/map")
def show_map() -> RedirectResponse:
    """Display the aircraft map viewer."""
    return RedirectResponse(url="/static/display.html")


@router.get("/plugins")
def list_plugins() -> Any:
    """List available and loaded plugins."""
    return plugin.manage_plugins("LIST")


@router.get("/plugins/load/{name}")
def load_plugin(name: str) -> Any:
    """Load a plugin by name."""
    return plugin.manage_plugins("LOAD", name)


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
