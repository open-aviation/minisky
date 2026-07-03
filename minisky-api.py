"""MiniSky REST API server.

FastAPI application that wraps a live simulation: the simulator is initialised at import
time and stepped continuously by the async Runner once the server starts. Endpoints
expose aircraft state, conflict information, simulation-time control, plugin management,
and a passthrough for any stack command.

Run with::

    fastapi dev minisky-api.py    # development, auto-reload
    fastapi run minisky-api.py    # production

Interactive OpenAPI docs are served at ``/docs``.
"""
import asyncio
import os
from io import StringIO

import pandas as pd
from fastapi import FastAPI, File, HTTPException, Response, UploadFile
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

import minisky
from minisky.tools import aero

app = FastAPI()

# Mount static files directory
static_dir = os.path.join(os.path.dirname(__file__), "static")
os.makedirs(static_dir, exist_ok=True)  # Create static directory if it doesn't exist
app.mount("/static", StaticFiles(directory=static_dir), name="static")

minisky.init()
minisky.load_plugins()


async def start_simulation():
    """Start the simulation loop as a background task in the server's event loop."""
    asyncio.create_task(minisky.runner.run())


app.add_event_handler("startup", start_simulation)


@app.get("/")
def root():
    """Health check: confirm the API is up."""
    return {"msg": "MiniSky API endpoint ready"}


@app.get("/all")
def all():
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
def simtime():
    """Get the simulation time"""
    return {"simulation time (seconds)": minisky.sim.simt}


@app.get("/speed/{speed}")
def speedup(speed: float):
    """Speed up the simulation"""
    minisky.runner.speed = speed
    return {"msg": f"simulation speed set to {speed}x"}


@app.get("/forward/{seconds}")
def forward(seconds: float):
    """Jump to a specific simulation time"""
    minisky.runner.forward(seconds)
    return {"msg": f"simulation time jump forward {seconds} seconds"}


@app.get("/conflicts")
def conflicts():
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
async def stack(cmd: str):
    """Execute a stack command and return the output"""
    minisky.scr.event.clear()
    minisky.stack.stack(f"{cmd}")
    await minisky.scr.event.wait()
    msg = minisky.scr.read_output_buffer()
    minisky.scr.event.clear()
    return {"command to minisky": cmd, "message": msg}


@app.get("/scn")
def upload_form():
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
async def scn(file: UploadFile = File(...)):
    """Load an uploaded scenario file into the running simulation."""
    minisky.scr.event.clear()
    contents = await file.read()
    scenario = StringIO(contents.decode("utf-8"))
    minisky.stack.ic_StringIO(scenario, file.filename)
    return {"msg": f"scenario {file.filename} loaded"}


@app.get("/map")
def show_map():
    """Display the aircraft map viewer"""
    return RedirectResponse(url="/static/display.html")


@app.get("/plugins")
def list_plugins():
    """List available and loaded plugins"""
    return minisky.plugin.manage_plugins('LIST')


@app.get("/plugins/load/{name}")
def load_plugin(name: str):
    """Load a plugin by name"""
    return minisky.plugin.manage_plugins('LOAD', name)
