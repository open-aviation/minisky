"""Minimal WebSocket client for the MiniSky ``GET /stream`` push feed.

Connects to a running MiniSky API server and prints one line per snapshot.
The stream only publishes while the simulation is stepping (state ``OP``) and
is skipped when no client is connected, so make sure a scenario is loaded and
running first, e.g.::

    fastapi dev minisky-api.py
    curl "http://localhost:8000/stack/IC%20scenarios/kl204.scn"
    curl "http://localhost:8000/stack/OP"

Then::

    uv run python scripts/stream_client.py
    uv run python scripts/stream_client.py --url ws://localhost:8000/stream --raw

Snapshots are SI units (metres, m/s, degrees); see ``minisky/streaming.py``.
"""

import argparse
import asyncio
import json

import websockets


async def stream(url: str, raw: bool) -> None:
    async with websockets.connect(url) as ws:
        print(f"connected to {url}")
        while True:
            snap = json.loads(await ws.recv())
            if raw:
                print(json.dumps(snap))
                continue
            info = snap["siminfo"]
            ac = snap["acdata"]
            print(
                f"t={info['simt']:8.1f}s  state={info['state']}  "
                f"ntraf={info['ntraf']}  speed={info['speed']}x  "
                f"callsigns={ac['callsign']}"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        default="ws://localhost:8000/stream",
        help="WebSocket URL of the MiniSky stream (default: %(default)s)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="print each snapshot as raw JSON instead of a summary line",
    )
    args = parser.parse_args()

    try:
        asyncio.run(stream(args.url, args.raw))
    except KeyboardInterrupt:
        print("\ndisconnected")


if __name__ == "__main__":
    main()
