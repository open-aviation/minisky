"""Tangram bridge: stream MiniSky state to a tangram map over Redis pub/sub.

This plugin makes a running MiniSky process act as an *external simulator*
for `tangram <https://github.com/open-aviation/tangram>`_. It talks to
tangram exclusively through Redis, using tangram's stable channel
convention (see ``docs/architecture/channel.md`` in the tangram repo):

- ``to:<channel>:<event>``   -- published by us, pushed to the browser by
  tangram's Channel service over WebSocket.
- ``from:<channel>:<event>`` -- pushed by the browser, re-published to
  Redis by the Channel service, consumed by us.

Wire contract (all published payloads are JSON):

- ``to:<channel>:new-data``: ``{"aircraft": [...], "count": n, "siminfo": {...}}``
  with per-aircraft fields in aviation units (altitude ft, speeds kt,
  vertical rate fpm) under jet1090-style names.
- ``to:<channel>:console``: ``{"lines": [...]}`` -- echoed simulator output.
- ``from:<channel>:command``: ``{"command": "OP"}`` -- a stack command to run.

All Redis I/O happens on a background thread so the simulation loop never
blocks on the network, and so commands are still received while the
simulation is paused (plugin update hooks only fire in the OP state; the
command stack itself is processed in every state).

Settings (optional, in ``settings.yml``):

- ``tangram_redis_url``: Redis connection URL (default
  ``redis://127.0.0.1:6379``).
- ``tangram_channel``: channel/topic name (default ``minisky``).
- ``tangram_max_hz``: wall-clock cap on snapshot publish rate (default 5).

Debug the transport without any frontend::

    redis-cli psubscribe "to:*"
    redis-cli publish "from:minisky:command" '{"command": "ECHO hello"}'
"""

import json
import queue
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any

import minisky
from minisky import stack
from minisky.core import settings
from minisky.streaming import build_snapshot
from minisky.tools.aero import fpm, ft, kts

# How often the background thread republishes state while the simulation is
# not advancing (paused/init), so the frontend still sees state changes.
HEARTBEAT_SECS = 1.0

SIM_STATE_NAMES = {0: "INIT", 1: "HOLD", 2: "OP", 3: "END"}

bridge = None


def convert_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Convert a MiniSky SI-unit snapshot into the tangram wire payload.

    Aircraft come out with jet1090-style field names and aviation units
    (altitude in ft, speeds in kt, vertical rate in fpm). Pure function so
    it can be unit-tested without a running simulator.
    """
    siminfo = snapshot.get("siminfo", {})
    acdata = snapshot.get("acdata", {})

    state = int(siminfo.get("state", 0))
    simutc = siminfo.get("simutc")
    try:
        timestamp = datetime.fromisoformat(simutc).timestamp() if simutc else None
    except ValueError:
        timestamp = None

    out_siminfo = {
        "simt": float(siminfo.get("simt", 0.0)),
        "simdt": float(siminfo.get("simdt", 0.0)),
        "simutc": simutc,
        "speed": float(siminfo.get("speed", 1.0)),
        "ntraf": int(siminfo.get("ntraf", 0)),
        "state": state,
        "state_name": SIM_STATE_NAMES.get(state, "?"),
        "scenname": siminfo.get("scenname"),
        "nconf_cur": acdata.get("nconf_cur", 0),
        "nlos_cur": acdata.get("nlos_cur", 0),
    }

    callsigns = acdata.get("callsign", [])
    n = len(callsigns)

    def col(name: str) -> list:
        values = acdata.get(name, [])
        return values if len(values) == n else [None] * n

    lat, lon = col("lat"), col("lon")
    alt, trk, vs = col("alt"), col("trk"), col("vs")
    tas, cas, gs = col("tas"), col("cas"), col("gs")
    typecode, inconf = col("typecode"), col("inconf")

    aircraft = []
    for i in range(n):
        aircraft.append(
            {
                "id": str(callsigns[i]),
                "callsign": str(callsigns[i]),
                "typecode": typecode[i],
                "latitude": lat[i],
                "longitude": lon[i],
                "altitude": round(alt[i] / ft) if alt[i] is not None else None,
                "groundspeed": round(gs[i] / kts, 1) if gs[i] is not None else None,
                "tas": round(tas[i] / kts, 1) if tas[i] is not None else None,
                "ias": round(cas[i] / kts, 1) if cas[i] is not None else None,
                "vertical_rate": round(vs[i] / fpm) if vs[i] is not None else None,
                "track": trk[i],
                "inconf": bool(inconf[i]) if inconf[i] is not None else False,
                "timestamp": timestamp,
            }
        )

    return {"aircraft": aircraft, "count": n, "siminfo": out_siminfo}


def extract_command(payload: str | bytes) -> str | None:
    """Extract the stack command from a ``from:<channel>:command`` payload.

    Accepts the JSON envelope pushed by the tangram frontend
    (``{"command": "..."}``) and, for convenience when testing with
    redis-cli, a bare string.
    """
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8", errors="replace")
    text = payload.strip()
    if not text:
        return None
    try:
        data = json.loads(text)
    except ValueError:
        return text
    if isinstance(data, dict):
        cmd = data.get("command")
        return str(cmd).strip() or None if cmd is not None else None
    if isinstance(data, str):
        return data.strip() or None
    return None


class TangramBridge:
    """Owns the Redis connection and shuttles data between it and the sim.

    The simulation thread only ever touches thread-safe queues/deques: the
    ``update`` hook enqueues converted snapshots, and a tee on ``scr.echo``
    enqueues console lines. A daemon thread does all Redis I/O: draining
    those queues, republishing a heartbeat while the sim is not advancing,
    and listening for browser commands on ``from:<channel>:*``.
    """

    def __init__(
        self,
        redis_url: str,
        channel: str,
        max_hz: float,
        redis_factory: Any | None = None,
    ) -> None:
        self.redis_url = redis_url
        self.channel = channel
        self.min_interval = 1.0 / max_hz if max_hz > 0 else 0.0
        self.redis_factory = redis_factory

        self.connected = False
        self.published = 0
        self.last_error = ""

        self._last_build = 0.0
        self._last_payload: dict[str, Any] | None = None
        self._snapshots: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=4)
        self._console: deque[str] = deque(maxlen=200)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.ready = threading.Event()
        """Set once the command subscription is live (commands published
        before this are lost -- Redis pub/sub has no replay)."""

    # -- simulation-thread side -------------------------------------------

    def start(self) -> tuple[bool, str]:
        """Install the console tee and start the Redis I/O thread."""
        try:
            if self.redis_factory is None:
                import redis

                self.redis_factory = redis.Redis.from_url
        except ImportError:
            return False, (
                "TANGRAM plugin needs the redis package: uv sync --extra tangram "
                "(or pip install redis)"
            )

        self._tee_console()
        self._thread = threading.Thread(target=self._run, name="tangram-bridge", daemon=True)
        self._thread.start()
        return True, f"Tangram bridge publishing to to:{self.channel}:* at {self.redis_url}"

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def tick(self) -> None:
        """Update hook: build and enqueue a snapshot (rate-capped). Runs in OP."""
        now = time.monotonic()
        if now - self._last_build < self.min_interval:
            return
        self._last_build = now
        self._enqueue(convert_snapshot(build_snapshot()))

    def reset(self) -> None:
        """Reset hook: push an empty payload so the frontend clears the map."""
        self._last_payload = None
        self._enqueue(convert_snapshot(build_snapshot()))

    def _enqueue(self, payload: dict[str, Any]) -> None:
        self._last_payload = payload
        try:
            self._snapshots.put_nowait(payload)
        except queue.Full:
            # Drop the oldest snapshot; each payload is a full state anyway.
            try:
                self._snapshots.get_nowait()
                self._snapshots.put_nowait(payload)
            except (queue.Empty, queue.Full):
                pass

    def _tee_console(self) -> None:
        """Also capture everything echoed to the console, without consuming it."""
        scr = minisky.scr
        original_echo = scr.echo

        def echo(text: str = "", flag: int = 0) -> None:
            original_echo(text, flag)
            if text:
                self._console.extend(text.splitlines())

        scr.echo = echo  # type: ignore[method-assign]

    # -- Redis-thread side -------------------------------------------------

    def _siminfo_heartbeat(self) -> dict[str, Any]:
        """Refresh the cheap scalar fields of the last payload.

        Only reads scalar attributes of the singletons (safe enough from a
        second thread); the aircraft list is reused from the last snapshot
        built on the simulation thread.
        """
        last = self._last_payload or {"aircraft": [], "count": 0, "siminfo": {}}
        sim = minisky.sim
        state = int(sim.state)
        siminfo: dict[str, Any] = {"nconf_cur": 0, "nlos_cur": 0}
        siminfo.update(last["siminfo"])
        siminfo.update(
            {
                "simt": float(sim.simt),
                "simdt": float(sim.simdt),
                "simutc": sim.utc.isoformat(),
                "speed": float(minisky.runner.speed) if minisky.runner else 1.0,
                "ntraf": int(minisky.traf.ntraf),
                "state": state,
                "state_name": SIM_STATE_NAMES.get(state, "?"),
                "scenname": stack.get_scenname(),
            }
        )
        return {"aircraft": last["aircraft"], "count": last["count"], "siminfo": siminfo}

    def _run(self) -> None:
        assert self.redis_factory is not None
        data_topic = f"to:{self.channel}:new-data"
        console_topic = f"to:{self.channel}:console"
        command_pattern = f"from:{self.channel}:*"
        command_topic = f"from:{self.channel}:command"

        while not self._stop.is_set():
            try:
                client = self.redis_factory(self.redis_url)
                pubsub = client.pubsub(ignore_subscribe_messages=True)
                pubsub.psubscribe(command_pattern)
                self.connected = True
                self.last_error = ""
                self.ready.set()
                last_publish = 0.0

                while not self._stop.is_set():
                    message = pubsub.get_message(timeout=0.05)
                    if message is not None and message.get("type") == "pmessage":
                        topic = message.get("channel", b"")
                        if isinstance(topic, bytes):
                            topic = topic.decode("utf-8", errors="replace")
                        if topic == command_topic:
                            cmd = extract_command(message.get("data", ""))
                            if cmd:
                                stack.stack(cmd)

                    published = False
                    while True:
                        try:
                            payload = self._snapshots.get_nowait()
                        except queue.Empty:
                            break
                        client.publish(data_topic, json.dumps(payload))
                        self.published += 1
                        published = True

                    if published:
                        last_publish = time.monotonic()
                    elif time.monotonic() - last_publish > HEARTBEAT_SECS:
                        # Publish even before the first snapshot (INIT state, no
                        # traffic yet) so the frontend sees the simulator at all.
                        client.publish(data_topic, json.dumps(self._siminfo_heartbeat()))
                        self.published += 1
                        last_publish = time.monotonic()

                    if self._console:
                        lines = []
                        while self._console:
                            lines.append(self._console.popleft())
                        client.publish(console_topic, json.dumps({"lines": lines}))
            except Exception as e:  # noqa: BLE001 - reconnect on any Redis failure
                self.connected = False
                self.ready.clear()
                self.last_error = str(e)
                if self._stop.wait(timeout=2.0):
                    return


@stack.command(name="TANGRAM")
def tangram_status() -> tuple[bool, str]:
    """Show the status of the tangram Redis bridge."""
    if bridge is None:
        return False, "Tangram bridge not initialised"
    status = "connected" if bridge.connected else "disconnected"
    text = (
        f"Tangram bridge: {status} to {bridge.redis_url}\n"
        f"Channel: to:{bridge.channel}:new-data ({bridge.published} messages published)"
    )
    if bridge.last_error:
        text += f"\nLast error: {bridge.last_error}"
    return True, text


def init_plugin():
    """Create the bridge and register its simulation hooks."""
    global bridge

    bridge = TangramBridge(
        redis_url=getattr(settings, "tangram_redis_url", "redis://127.0.0.1:6379"),
        channel=getattr(settings, "tangram_channel", "minisky"),
        max_hz=float(getattr(settings, "tangram_max_hz", 5.0)),
    )
    success, msg = bridge.start()
    minisky.scr.echo(msg)
    if not success:
        raise RuntimeError(msg)

    config = {
        "plugin_name": "TANGRAM",
        "update_interval": 0.0,
        "update": bridge.tick,
        "reset": bridge.reset,
    }
    return config
