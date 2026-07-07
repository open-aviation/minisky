"""MiniSky REST + streaming API server.

FastAPI application that wraps a live simulation: the simulator is initialised
at import time and stepped continuously by the async Runner once the server
starts. Endpoints expose aircraft state, conflict information, simulation-time
control, plugin management, a passthrough for any stack command, a per-tick
push stream (``GET /stream``, WebSocket), and the command dictionary
(``GET /commands``).

This module holds the FastAPI application object (``app``). The supported CLI entry
point is ``minisky server``.

Run with::

    minisky server                 # CLI server command (uvicorn)
    minisky server --reload        # development, auto-reload

Interactive OpenAPI docs are served at ``/docs``.
"""

import asyncio
import os
from contextlib import asynccontextmanager
from io import StringIO
from typing import Any

import pandas as pd
from fastapi import FastAPI, File, Response, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles

import minisky
from minisky.streaming import hub, register_stream_hook
from minisky.tools import aero


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start the simulation loop as a background task in the server's event loop."""
    task = asyncio.create_task(minisky.runner.run())
    yield
    task.cancel()


app = FastAPI(lifespan=lifespan)

# Static files live at the repository root (../static relative to this package),
# which resolves correctly for both a source checkout and an editable install.
static_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "static")
os.makedirs(static_dir, exist_ok=True)  # Create static directory if it doesn't exist
app.mount("/static", StaticFiles(directory=static_dir), name="static")

minisky.init()
minisky.load_plugins()
# Publish a snapshot on every simulation step for the /stream endpoint.
register_stream_hook()


@app.get("/")
def root() -> dict[str, str]:
    """Health check: confirm the API is up."""
    return {"msg": "MiniSky API endpoint ready"}


@app.get("/all")
def all() -> list[dict[str, Any]]:
    """Get all aircraft states"""
    df = pd.DataFrame(
        {
            "callsign": minisky.traf.callsign,
            "typecode": minisky.traf.typecode,
            "latitude": minisky.traf.lat,
            "longitude": minisky.traf.lon,
            "altitude (feet)": (minisky.traf.alt / aero.ft).astype(int),
            "heading (degrees)": minisky.traf.hdg.astype(int),
            "assigned heading (degrees)": minisky.traf.aporasas.hdg.astype(int),
            "track (degrees)": minisky.traf.trk,
            "TAS (knots)": (minisky.traf.tas / aero.kts).astype(int),
            "groundspeed (knots)": (minisky.traf.gs / aero.kts).astype(int),
            "CAS (knots)": (minisky.traf.cas / aero.kts).astype(int),
            "mach": minisky.traf.M,
            "vertical_rate (feet/minute)": (minisky.traf.vs / aero.fpm).astype(int),
            "target altitude (feet)": (minisky.traf.selalt / aero.ft).astype(int),
            "assigned speed (knots)": (minisky.traf.selspd / aero.kts).astype(int),
        }
    )

    return df.to_dict(orient="records")


@app.get("/simtime")
def simtime() -> dict[str, float]:
    """Get the simulation time"""
    return {"simulation time (seconds)": minisky.sim.simt}


@app.get("/speed/{speed}")
def speedup(speed: float) -> dict[str, str]:
    """Speed up the simulation"""
    minisky.runner.speed = speed
    return {"msg": f"simulation speed set to {speed}x"}


@app.get("/forward/{seconds}")
def forward(seconds: float) -> dict[str, str]:
    """Jump to a specific simulation time"""
    minisky.runner.forward(seconds)
    return {"msg": f"simulation time jump forward {seconds} seconds"}


@app.get("/conflicts")
def conflicts() -> list[dict[str, Any]] | dict[str, str]:
    """Get all detected conflicts.

    Returns one record per unique aircraft pair with distance (NM), altitude
    difference (ft), bearing (deg), time to loss of separation (s), and distance and
    time to the closest point of approach (m, s).
    """
    if not hasattr(minisky.traf.cd, "confpairs") or not len(minisky.traf.cd.confpairs):
        return {"msg": "No conflicts detected"}

    # Ensure there's a structure to hold TCPA for each conflict pair
    if not hasattr(minisky.traf.cd, "tcpa") or len(minisky.traf.cd.tcpa) == 0:
        return {"msg": "No TCPA data available"}

    processed_pairs = []
    conflict_info = []

    for i, pair in enumerate(minisky.traf.cd.confpairs):
        if set(pair) in processed_pairs:
            continue

        processed_pairs.append(set(pair))

        conflict_info.append(
            {
                "conflict pairs": pair,
                "distance (nautical miles)": (minisky.traf.cd.dist[i] / aero.nm),
                "altitude difference (feet)": (minisky.traf.cd.dalt[i] / aero.ft),
                "qdr (degrees)": minisky.traf.cd.qdr[i],
                "tlos (seconds)": minisky.traf.cd.tLOS[i],
                "dcpa (meters)": minisky.traf.cd.dcpa[i],
                "tcpa (seconds)": minisky.traf.cd.tcpa[i],
            }
        )
    return conflict_info


@app.get("/stack/{cmd}")
async def stack(cmd: str) -> dict[str, Any]:
    """Execute a stack command and return the output"""
    minisky.scr.event.clear()
    minisky.stack.stack(f"{cmd}")
    await minisky.scr.event.wait()
    msg = minisky.scr.read_output_buffer()
    minisky.scr.event.clear()
    return {"command to minisky": cmd, "message": msg}


@app.get("/commands")
def commands() -> dict[str, str]:
    """Return the command dictionary as ``{name: brief usage}``.

    Deduplicates aliases (which share a ``Command`` object) and reports each
    command under its canonical name, so a console/autocomplete client can list
    the available stack commands and their usage.
    """
    from minisky.stack import Command

    seen: dict[str, str] = {}
    for cmdobj in dict.fromkeys(Command.cmddict.values()):
        seen[cmdobj.name] = cmdobj.brief
    return dict(sorted(seen.items()))


@app.websocket("/stream")
async def stream(websocket: WebSocket) -> None:
    """Push a full simulation snapshot once per sim step (SI units).

    Emits one JSON message per published tick (rate-capped, see
    :data:`minisky.streaming.STREAM_MAX_HZ`) containing ``siminfo`` and
    ``acdata`` as built by :func:`minisky.streaming.build_snapshot`. The most
    recent snapshot is sent immediately on connect so a new client is not left
    blank until the next tick.
    """
    await websocket.accept()
    hub.subscribe()
    try:
        if hub.latest is not None:
            await websocket.send_json(hub.latest)
        while True:
            await hub.wait()
            if hub.latest is not None:
                await websocket.send_json(hub.latest)
    except WebSocketDisconnect:
        pass
    finally:
        hub.unsubscribe()


@app.get("/scn")
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


@app.post("/scn")
async def scn(file: UploadFile = File(...)) -> dict[str, str]:
    """Load an uploaded scenario file into the running simulation."""
    minisky.scr.event.clear()
    contents = await file.read()
    scenario = StringIO(contents.decode("utf-8"))
    filename = file.filename or "uploaded.scn"
    minisky.stack.ic_StringIO(scenario, filename)
    return {"msg": f"scenario {filename} loaded"}


@app.get("/map")
def show_map() -> RedirectResponse:
    """Display the aircraft map viewer"""
    return RedirectResponse(url="/static/display.html")


@app.get("/plugins")
def list_plugins() -> Any:
    """List available and loaded plugins"""
    return minisky.plugin.manage_plugins("LIST")


@app.get("/plugins/load/{name}")
def load_plugin(name: str) -> Any:
    """Load a plugin by name"""
    return minisky.plugin.manage_plugins("LOAD", name)


def main() -> None:
    """Console-script entry point: serve the API with uvicorn.

    Host and port are read from the ``MINISKY_HOST`` (default ``0.0.0.0``) and
    ``MINISKY_PORT`` (default ``8000``) environment variables. This is the
    retained for direct module execution.
    """
    import uvicorn

    host = os.environ.get("MINISKY_HOST", "0.0.0.0")
    port = int(os.environ.get("MINISKY_PORT", "8000"))
    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    main()
