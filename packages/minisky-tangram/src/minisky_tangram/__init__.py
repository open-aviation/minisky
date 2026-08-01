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

Config (optional, under `[plugins.tangram]` in the MiniSky user config file):

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
from collections.abc import AsyncGenerator, Callable
from contextlib import asynccontextmanager
from datetime import UTC, datetime
from typing import Any, TypedDict, cast

from pydantic import BaseModel, ConfigDict

from minisky import plugin as plugin_api
from minisky.simulation import SimulationState
from minisky.streaming import Snapshot
from minisky.tools.aero import fpm, ft, kts


# --8<-- [start:configuration]
class TangramConfig(BaseModel):
    """Validated `[plugins.tangram]` configuration."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    redis_url: str = "redis://127.0.0.1:6379"
    channel: str = "minisky"
    max_hz: float = 5.0


# --8<-- [end:configuration]


# How often the background thread republishes state while the simulation is
# not advancing (paused/init), so the frontend still sees state changes.
HEARTBEAT_SECS = 1.0


def _state_name(state: int) -> str:
    """Return the enum member name for a serialized simulation state."""
    try:
        return SimulationState(state).name
    except ValueError:
        return "?"


class TangramSimInfo(TypedDict):
    """Simulation status block of the `new-data` wire payload."""

    simt: float  # s
    simdt: float  # s
    simutc: str  # ISO-8601
    speed: float  # runner speed multiplier (x realtime)
    ntraf: int
    state: int  # Serialized SimulationState value.
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
        "state_name": _state_name(state),
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
    """Own Redis I/O and bridge it to a plugin runtime."""

    def __init__(
        self,
        # we assume the redis url has no password and is safe to log
        redis_url: str,
        channel: str,
        max_hz: float,
        redis_factory: Callable[[str], Any] | None = None,
    ) -> None:
        self.redis_url = redis_url
        self.channel = channel
        self.min_interval = 1.0 / max_hz if max_hz > 0 else 0.0
        self.redis_factory = redis_factory

        self.connected = False
        self.published = 0
        self.last_error = ""

        self._snapshot_builder: Callable[[], Snapshot] | None = None
        self._status_builder: Callable[[], plugin_api.PluginStatus] | None = None
        self._stack_command: Callable[[str], None] | None = None
        self._last_build = 0.0
        self._last_payload: TangramPayload | None = None
        self._snapshots: queue.Queue[TangramPayload] = queue.Queue(maxsize=4)
        self._console: deque[str] = deque(maxlen=200)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.ready = threading.Event()

    def start(self, runtime: plugin_api.PluginRuntime) -> tuple[bool, str]:
        """Bind runtime capabilities and start the Redis thread."""
        try:
            if self.redis_factory is None:
                import redis

                self.redis_factory = cast(
                    "Callable[[str], Any]",
                    redis.Redis.from_url,  # pyright: ignore[reportUnknownMemberType]
                )
        except ImportError:
            return False, (
                "TANGRAM plugin needs the redis package; run `just sync` from the "
                "MiniSky repository root"
            )

        self._snapshot_builder = runtime.snapshot
        self._status_builder = runtime.status
        self._stack_command = runtime.stack_command
        self._stop.clear()
        self.ready.clear()
        self._thread = threading.Thread(target=self._run, name="tangram-bridge", daemon=True)
        self._thread.start()
        return True, f"Tangram bridge publishing to to:{self.channel}:* at {self.redis_url}"

    def stop(self) -> None:
        """Stop Redis I/O and release runtime callbacks."""
        self._stop.set()
        # TODO(abraham): use finite redis timeouts, close client/pubsub resources,
        # and retain ownership when the thread does not stop
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.connected = False
        self.ready.clear()
        self._snapshot_builder = None
        self._status_builder = None
        self._stack_command = None

    @plugin_api.command(name="TANGRAM")
    def status(self) -> tuple[bool, str]:
        """Show the status of the tangram Redis bridge."""
        status = "connected" if self.connected else "disconnected"
        text = (
            f"Tangram bridge: {status} to {self.redis_url}\n"
            f"Channel: to:{self.channel}:new-data ({self.published} messages published)"
        )
        if self.last_error:
            text += f"\nLast error: {self.last_error}"
        return True, text

    @plugin_api.hook("update")
    def tick(self) -> None:
        """Build and enqueue a rate-capped snapshot while operating."""
        snapshot_builder = self._snapshot_builder
        if snapshot_builder is None:
            return
        now = time.monotonic()
        if now - self._last_build < self.min_interval:
            return
        self._last_build = now
        self._enqueue(convert_snapshot(snapshot_builder()))

    @plugin_api.hook("reset")
    def reset(self) -> None:
        """Push an empty payload so the frontend clears the map."""
        snapshot_builder = self._snapshot_builder
        if snapshot_builder is None:
            return
        self._last_payload = None
        self._enqueue(convert_snapshot(snapshot_builder()))

    def capture_console(self, text: str) -> None:
        if text:
            self._console.extend(text.splitlines())

    def _enqueue(self, payload: TangramPayload) -> None:
        self._last_payload = payload
        try:
            self._snapshots.put_nowait(payload)
        except queue.Full:
            try:
                self._snapshots.get_nowait()
                self._snapshots.put_nowait(payload)
            except (queue.Empty, queue.Full):
                pass

    def _siminfo_heartbeat(self) -> TangramPayload:
        status_builder = self._status_builder
        if status_builder is None:
            raise RuntimeError("Tangram bridge is stopped")
        status = status_builder()
        last = self._last_payload
        siminfo: TangramSimInfo = {
            "simt": status.simt,
            "simdt": status.simdt,
            "simutc": status.simutc.isoformat(),
            "speed": status.speed,
            "ntraf": status.ntraf,
            "state": status.state,
            "state_name": _state_name(status.state),
            "scenname": status.scenname,
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
                            command = extract_command(message.get("data", ""))
                            stack_command = self._stack_command
                            if command and stack_command is not None:
                                stack_command(command)

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
                        client.publish(data_topic, json.dumps(self._siminfo_heartbeat()))
                        self.published += 1
                        last_publish = time.monotonic()

                    if self._console:
                        lines: list[str] = []
                        while self._console:
                            lines.append(self._console.popleft())
                        client.publish(console_topic, json.dumps({"lines": lines}))
            except Exception as exc:  # noqa: BLE001 - reconnect after transport failure
                self.connected = False
                self.ready.clear()
                self.last_error = str(exc)
                if self._stop.wait(timeout=2.0):
                    return


# --8<-- [start:lifespan]
def build(context: plugin_api.PluginContext[TangramConfig]) -> plugin_api.PluginSpec:
    bridge = context.mount(
        TangramBridge(
            redis_url=context.config.redis_url,
            channel=context.config.channel,
            max_hz=context.config.max_hz,
        )
    )

    @asynccontextmanager
    async def lifespan(runtime: plugin_api.PluginRuntime) -> AsyncGenerator[None]:
        runtime.subscribe_console(bridge.capture_console)
        success, message = bridge.start(runtime)
        runtime.echo(message)
        if not success:
            raise RuntimeError(message)
        try:
            yield
        finally:
            bridge.stop()

    return context.finish(lifespan=lifespan)


plugin = plugin_api.Plugin(build=build, config_class=TangramConfig)
# --8<-- [end:lifespan]
