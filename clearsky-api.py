import asyncio
from io import StringIO

from fastapi import FastAPI, File, Response, UploadFile

import clearsky as cs
import pandas as pd

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


@app.get("/stack/{cmd}")
async def stack(cmd: str):
    """Execute a stack command and return the output"""
    cs.scr.event.clear()
    cs.stack.stack(f"{cmd}")
    await cs.scr.event.wait()
    msg = cs.scr.read_output_buffer()
    cs.scr.event.clear()
    return {"cmd": cmd, "msg": msg}


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
