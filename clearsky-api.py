import asyncio
from io import StringIO

import clearsky as cs
import pandas as pd
from fastapi import FastAPI, File, Response, UploadFile

app = FastAPI()

cs.init()


async def start_simulation():
    asyncio.create_task(cs.runner.run())


app.add_event_handler("startup", start_simulation)


@app.get("/")
def root():
    return {"msg": "ClearSky API endpoint ready"}


@app.get("/all")
def all():
    """Get all aircraft states"""
    df = pd.DataFrame().assign(
        callsign=cs.traf.id,
        typecode=cs.traf.type,
        latitude=cs.traf.lat,
        longitude=cs.traf.lon,
        altitude=cs.traf.alt,
        heading=cs.traf.hdg,
        track=cs.traf.trk,
        TAS=cs.traf.tas,
        groundspeed=cs.traf.gs,
        CAS=cs.traf.cas,
        mach=cs.traf.M,
        vertical_rate=cs.traf.vs,
    )

    return df.to_dict(orient="records")


@app.get("/conflicts")
def conflicts():
    if not hasattr(cs.traf.cd, "confpairs") or not len(cs.traf.cd.confpairs):
        return {"msg": "No conflicts detected"}

    # Ensure there's a structure to hold TCPA for each conflict pair
    if not hasattr(cs.traf.cd, "tcpa") or len(cs.traf.cd.tcpa) == 0:
        return {"msg": "No TCPA data available"}

    print(cs.traf.cd.confpairs)
    print(cs.traf.cd.tcpa)
    print(cs.traf.cd.qdr)
    print(cs.traf.cd.dist)
    print(cs.traf.cd.dcpa)
    print(cs.traf.cd.tLOS)

    processed_pairs = []
    conflict_info = []

    for i, pair in enumerate(cs.traf.cd.confpairs):
        if set(pair) in processed_pairs:
            continue

        processed_pairs.append(set(pair))

        conflict_info.append(
            {
                "pair": pair,
                "distance": {"value": cs.traf.cd.dist[i], "unit": "meters"},
                "qdr": {"value": cs.traf.cd.qdr[i], "unit": "degrees"},
                "tlos": {
                    "value": cs.traf.cd.tLOS[i],
                    "unit": "seconds",
                },
                "dcpa": {"value": cs.traf.cd.dcpa[i], "unit": "meters"},
                "tcpa": {"value": cs.traf.cd.tcpa[i], "unit": "seconds"},
            }
        )
    return conflict_info


@app.get("/stack/{cmd}")
async def stack(cmd: str):
    """Execute a stack command and return the output"""
    cs.scr.event.clear()
    cs.stack.stack(f"{cmd}")
    await cs.scr.event.wait()
    msg = cs.scr.read_output_buffer()
    cs.scr.event.clear()
    return {"command_sent": cmd, "message": msg}


@app.get("/scn")
def upload_form():
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
    cs.scr.event.clear()
    contents = await file.read()
    scenario = StringIO(contents.decode("utf-8"))
    cs.stack.simstack.ic_from_string(scenario, file.filename)
    return {"msg": f"scenario {file.filename} loaded"}
