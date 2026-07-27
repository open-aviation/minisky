"""Tangram bridge: stream MiniSky state to a tangram map over Redis pub/sub.

This plugin makes a running MiniSky process act as an *external simulator*
for [tangram](https://github.com/open-aviation/tangram). It talks to
tangram exclusively through Redis, using tangram's stable channel
convention (see `docs/architecture/channel.md` in the tangram repo):

- `to:<channel>:<event>`   -- published by us, pushed to the browser by
  tangram's Channel service over WebSocket.
- `from:<channel>:<event>` -- pushed by the browser, re-published to
  Redis by the Channel service, consumed by us.

Wire contract (all published payloads are JSON):

- `to:<channel>:new-data`: `{"aircraft": [...], "count": n, "siminfo": {...}}`
  with per-aircraft fields in aviation units (altitude ft, speeds kt,
  vertical rate fpm) under jet1090-style names.
- `to:<channel>:console`: `{"lines": [...]}` -- echoed simulator output.
- `from:<channel>:command`: `{"command": "OP"}` -- a stack command to run.

All Redis I/O happens on a background thread so the simulation loop never
blocks on the network, and so commands are still received while the
simulation is paused (plugin update hooks only fire in the OP state; the
command stack itself is processed in every state).

Settings (optional, under a `[tangram]` table in `settings.toml`):

- `redis_url`: Redis connection URL (default `redis://127.0.0.1:6379`).
- `channel`: channel/topic name (default `minisky`).
- `max_hz`: wall-clock cap on snapshot publish rate (default 5).

Debug the transport without any frontend:

```console
redis-cli psubscribe "to:*"
redis-cli publish "from:minisky:command" '{"command": "ECHO hello"}'
```
"""

from __future__ import annotations

import json
import queue
import threading
import time
from collections import deque
from collections.abc import Callable
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any, TypedDict, cast

from pydantic import BaseModel, ConfigDict, Field

import minisky
from minisky import stack
from minisky.core import settings
from minisky.streaming import Snapshot, build_snapshot
from minisky.tools.aero import fpm, ft, kts

if TYPE_CHECKING:
    from minisky.simulation import ConsoleIO, Runner, Simulation
    from minisky.traffic import Traffic


class TangramSettings(BaseModel):
    """Validated `[tangram]` config from `settings.toml`."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    redis_url: str = "redis://127.0.0.1:6379"
    channel: str = "minisky"
    max_hz: float = 5.0


class TangramPluginSettings(BaseModel):
    tangram: TangramSettings = Field(default_factory=TangramSettings)


# How often the background thread republishes state while the simulation is
# not advancing (paused/init), so the frontend still sees state changes.
HEARTBEAT_SECS = 1.0

SIM_STATE_NAMES = {0: "INIT", 1: "HOLD", 2: "OP", 3: "END"}

bridge = None


class TangramSimInfo(TypedDict):
    """Simulation status block of the `new-data` wire payload."""

    simt: float  # s
    simdt: float  # s
    simutc: str  # ISO-8601
    speed: float  # runner speed multiplier (x realtime)
    ntraf: int
    state: int  # 0=INIT, 1=HOLD, 2=OP, 3=END
    state_name: str
    scenname: str  # "" when no scenario is loaded
    nconf_cur: int
    nlos_cur: int


class TangramAircraft(TypedDict):
    """One aircraft in the `new-data` wire payload (jet1090-style fields)."""

    id: str
    callsign: str
    typecode: str
    latitude: float  # deg
    longitude: float  # deg
    altitude: int  # ft
    groundspeed: float  # kt
    tas: float  # kt
    ias: float  # kt
    vertical_rate: int  # fpm
    track: float  # deg
    inconf: bool
    timestamp: float | None  # epoch seconds; None when simutc fails to parse


class TangramPayload(TypedDict):
    """Full `to:<channel>:new-data` wire payload."""

    aircraft: list[TangramAircraft]
    count: int
    siminfo: TangramSimInfo


def convert_snapshot(snapshot: Snapshot) -> TangramPayload:
    """Convert a MiniSky SI-unit snapshot into the tangram wire payload.

    Aircraft come out with jet1090-style field names and aviation units
    (altitude in ft, speeds in kt, vertical rate in fpm). Pure function so
    it can be unit-tested without a running simulator.
    """
    siminfo = snapshot["siminfo"]
    acdata = snapshot["acdata"]

    state = siminfo["state"]
    simutc = siminfo["simutc"]
    try:
        utc = datetime.fromisoformat(simutc)
    except ValueError:
        utc = None
    if utc is not None and utc.tzinfo is None:
        # sim.utc is UTC by definition; never let a naive string be read as local time.
        utc = utc.replace(tzinfo=UTC)
    timestamp = utc.timestamp() if utc is not None else None

    out_siminfo: TangramSimInfo = {
        "simt": siminfo["simt"],
        "simdt": siminfo["simdt"],
        "simutc": simutc,
        "speed": siminfo["speed"],
        "ntraf": siminfo["ntraf"],
        "state": state,
        "state_name": SIM_STATE_NAMES.get(state, "?"),
        "scenname": siminfo["scenname"],
        "nconf_cur": acdata["nconf_cur"],
        "nlos_cur": acdata["nlos_cur"],
    }

    aircraft: list[TangramAircraft] = []
    for i, callsign in enumerate(acdata["callsign"]):
        aircraft.append(
            {
                "id": callsign,
                "callsign": callsign,
                "typecode": acdata["typecode"][i],
                "latitude": acdata["lat"][i],
                "longitude": acdata["lon"][i],
                "altitude": round(acdata["alt"][i] / ft),
                "groundspeed": round(acdata["gs"][i] / kts, 1),
                "tas": round(acdata["tas"][i] / kts, 1),
                "ias": round(acdata["cas"][i] / kts, 1),
                "vertical_rate": round(acdata["vs"][i] / fpm),
                "track": acdata["trk"][i],
                "inconf": acdata["inconf"][i],
                "timestamp": timestamp,
            }
        )

    return {"aircraft": aircraft, "count": len(aircraft), "siminfo": out_siminfo}


def extract_command(payload: str | bytes) -> str | None:
    """Extract the stack command from a `from:<channel>:command` payload.

    Accepts the JSON envelope pushed by the tangram frontend
    (`{"command": "..."}`) and, for convenience when testing with
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
        cmd = cast("dict[str, Any]", data).get("command")
        return str(cmd).strip() or None if cmd is not None else None
    if isinstance(data, str):
        return data.strip() or None
    return None


class TangramBridge:
    """Owns the Redis connection and shuttles data between it and the sim.

    The simulation thread only ever touches thread-safe queues/deques: the
    `update` hook enqueues converted snapshots, and a tee on `scr.echo`
    enqueues console lines. A daemon thread does all Redis I/O: draining
    those queues, republishing a heartbeat while the sim is not advancing,
    and listening for browser commands on `from:<channel>:*`.
    """

    def __init__(
        self,
        redis_url: str,
        channel: str,
        max_hz: float,
        snapshot_builder: Callable[[], Snapshot],
        console: ConsoleIO,
        simulation: Simulation,
        runner: Runner,
        traffic: Traffic,
        get_scenname: Callable[[], str],
        stack_command: Callable[[str], None],
        redis_factory: Callable[[str], Any] | None = None,
    ) -> None:
        self.redis_url = redis_url
        self.channel = channel
        self.min_interval = 1.0 / max_hz if max_hz > 0 else 0.0
        self.snapshot_builder = snapshot_builder
        self.console = console
        self.simulation = simulation
        self.runner = runner
        self.traffic = traffic
        self.get_scenname = get_scenname
        self.stack_command = stack_command
        self.redis_factory = redis_factory

        self.connected = False
        self.published = 0
        self.last_error = ""

        self._last_build = 0.0
        self._last_payload: TangramPayload | None = None
        self._snapshots: queue.Queue[TangramPayload] = queue.Queue(maxsize=4)
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

                # cast: from_url's untyped **kwargs would leak Unknown under strict mode.
                self.redis_factory = cast(
                    "Callable[[str], Any]",
                    redis.Redis.from_url,  # pyright: ignore[reportUnknownMemberType]
                )
        except ImportError:
            return False, (
                "TANGRAM plugin needs the redis package; run `just sync` from the "
                "MiniSky repository root"
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
        self._enqueue(convert_snapshot(self.snapshot_builder()))

    def reset(self) -> None:
        """Reset hook: push an empty payload so the frontend clears the map."""
        self._last_payload = None
        self._enqueue(convert_snapshot(self.snapshot_builder()))

    def _enqueue(self, payload: TangramPayload) -> None:
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
        original_echo = self.console.echo

        def echo(text: str = "", flag: int = 0) -> None:
            original_echo(text, flag)
            if text:
                self._console.extend(text.splitlines())

        self.console.echo = echo  # type: ignore[method-assign]

    # -- Redis-thread side -------------------------------------------------

    def _siminfo_heartbeat(self) -> TangramPayload:
        """Refresh the cheap scalar fields of the last payload.

        Only reads scalar attributes of the injected runtime components (safe
        enough from a second thread); the aircraft list and conflict counters are reused
        from the last snapshot built on the simulation thread.
        """
        last = self._last_payload
        sim = self.simulation
        state = int(sim.state)
        siminfo: TangramSimInfo = {
            "simt": float(sim.simt),
            "simdt": float(sim.simdt),
            "simutc": sim.utc.isoformat(),
            "speed": float(self.runner.speed),
            "ntraf": int(self.traffic.ntraf),
            "state": state,
            "state_name": SIM_STATE_NAMES.get(state, "?"),
            "scenname": self.get_scenname(),
            "nconf_cur": last["siminfo"]["nconf_cur"] if last is not None else 0,
            "nlos_cur": last["siminfo"]["nlos_cur"] if last is not None else 0,
        }
        if last is None:
            return {"aircraft": [], "count": 0, "siminfo": siminfo}
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
                                self.stack_command(cmd)

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
                        lines: list[str] = []
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


def init_plugin() -> dict[str, Any]:
    """Create the bridge and register its simulation hooks."""
    global bridge

    # TODO(abraham): we should namespace it under settings.plugins.tangram.
    cfg = TangramPluginSettings.model_validate(settings.default_settings).tangram
    command_stack = stack.current()
    bridge = TangramBridge(
        redis_url=cfg.redis_url,
        channel=cfg.channel,
        max_hz=cfg.max_hz,
        snapshot_builder=lambda: build_snapshot(
            minisky.sim, minisky.traf, minisky.runner, command_stack
        ),
        console=minisky.scr,
        simulation=minisky.sim,
        runner=minisky.runner,
        traffic=minisky.traf,
        get_scenname=command_stack.get_scenname,
        stack_command=command_stack.stack,
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
