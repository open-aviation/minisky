"""Per-tick streaming of simulation state.

Provides a small, transport-agnostic mechanism to push a full snapshot of one
simulation runtime once per timestep. [`build_snapshot`][] receives the runtime
explicitly and returns a plain, JSON-serialisable dict in **SI units**;
[`StreamHub`][] fans that snapshot out to any number of awaiting consumers
(e.g. WebSocket connections in `minisky.server`).

This is a generic streaming API: it emits raw SI state and takes no position on
any particular client or wire contract. Unit conversion and field mapping to a
specific consumer's format happen downstream, in that consumer, not here.

The snapshot shape is defined by the [`Snapshot`][], [`SimInfo`][], and
[`AcData`][] TypedDicts below.

Units on the wire here are SI: positions in decimal degrees, `alt` in metres,
speeds (`tas`/`cas`/`gs`) in m/s, `vs` in m/s, `trk` in degrees,
`simt`/`simdt` in seconds. `state` is the numeric simulation state
(0=INIT, 1=HOLD, 2=OP, 3=END). Each tick is a full snapshot; aircraft are
identified by `callsign` for their lifetime.
"""

from __future__ import annotations

import asyncio
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, TypedDict, cast

import numpy as np

if TYPE_CHECKING:
    from minisky.simulation import Runner, Simulation
    from minisky.stack import CommandStack
    from minisky.traffic import Traffic

# Default upper bound on how often a snapshot is published, in Hz. The
# simulation may step much faster than this in fast-forward; publishing is
# gated to at most this wall-clock rate so consumers are not flooded.
STREAM_MAX_HZ = 10.0


class SimInfo(TypedDict):
    """Simulation-level snapshot fields."""

    speed: float  # runner speed multiplier (x realtime)
    simdt: float  # s
    simt: float  # s
    simutc: str  # ISO-8601, timezone-aware
    ntraf: int
    state: int  # 0=INIT, 1=HOLD, 2=OP, 3=END
    scenname: str  # "" when no scenario is loaded


class AcData(TypedDict):
    """Per-aircraft snapshot columns (one element per aircraft, SI units)."""

    callsign: list[str]
    lat: list[float]  # deg
    lon: list[float]  # deg
    alt: list[float]  # m
    trk: list[float]  # deg
    vs: list[float]  # m/s
    tas: list[float]  # m/s
    cas: list[float]  # m/s
    gs: list[float]  # m/s
    typecode: list[str]
    inconf: list[bool]
    tcpamax: list[float]  # s
    nconf_cur: int
    nconf_tot: int
    nlos_cur: int
    nlos_tot: int


class Snapshot(TypedDict):
    """Full per-tick simulation snapshot."""

    siminfo: SimInfo
    acdata: AcData


def _tolist(arr: Any) -> list[float]:
    """Convert a numpy array (or list) into a plain JSON-serialisable list."""
    if isinstance(arr, np.ndarray):
        return cast(list[float], arr.tolist())
    return list(arr)


def build_snapshot(
    simulation: Simulation,
    traffic: Traffic,
    runner: Runner,
    commands: CommandStack,
) -> Snapshot:
    """Build a full snapshot from explicit runtime components in SI units.

    Returns:
        A [`Snapshot`][] with `siminfo` and `acdata` keys.
    """
    sim = simulation
    traf = traffic
    cd = traf.cd

    siminfo: SimInfo = {
        "speed": float(runner.speed),
        "simdt": float(sim.simdt),
        "simt": float(sim.simt),
        "simutc": sim.utc.isoformat(),
        "ntraf": int(traf.ntraf),
        "state": int(sim.state),
        "scenname": commands.get_scenname(),
    }

    acdata: AcData = {
        "callsign": [str(c) for c in traf.callsign],
        "lat": _tolist(traf.lat),
        "lon": _tolist(traf.lon),
        "alt": _tolist(traf.alt),  # metres
        "trk": _tolist(traf.trk),
        "vs": _tolist(traf.vs),  # m/s
        "tas": _tolist(traf.tas),  # m/s
        "cas": _tolist(traf.cas),  # m/s
        "gs": _tolist(traf.gs),  # m/s
        "typecode": [str(t) for t in traf.typecode],
        # Conflict data (traf.cd). The per-pair counters are derived from the
        # detection object's current/cumulative unique-pair collections.
        "inconf": [bool(v) for v in cd.inconf],
        "tcpamax": _tolist(cd.tcpamax),
        "nconf_cur": len(cd.confpairs_unique),
        "nconf_tot": len(cd.confpairs_all),
        "nlos_cur": len(cd.lospairs_unique),
        "nlos_tot": len(cd.lospairs_all),
    }

    return {"siminfo": siminfo, "acdata": acdata}


class StreamHub:
    """Fan-out hub distributing per-tick snapshots to awaiting consumers.

    Each runtime owns a hub. Its simulation calls [`StreamHub.publish_tick`][]
    once per step; each connected consumer awaits [`StreamHub.wait`][] and then
    reads `latest`.

    Snapshot construction is skipped entirely when there are no subscribers,
    and gated to at most `max_hz` publications per wall-clock second so that a
    fast-forwarding simulation does not flood consumers.

    Attributes:
        latest: The most recently published snapshot (`None` until the first
            publish), used to seed newly connected consumers.
        generation: Monotonically increasing counter incremented on each
            publish; consumers may use it to detect missed ticks.
    """

    def __init__(
        self, build_snapshot: Callable[[], Snapshot], max_hz: float = STREAM_MAX_HZ
    ) -> None:
        self._build_snapshot = build_snapshot
        self._subscribers = 0
        self._event = asyncio.Event()
        self._min_interval = 1.0 / max_hz if max_hz > 0 else 0.0
        self._last_publish = 0.0
        self.latest: Snapshot | None = None
        self.generation = 0

    @property
    def active(self) -> bool:
        """True while at least one consumer is subscribed."""
        return self._subscribers > 0

    def subscribe(self) -> None:
        """Register a new consumer."""
        self._subscribers += 1

    def unsubscribe(self) -> None:
        """Deregister a consumer."""
        self._subscribers = max(0, self._subscribers - 1)

    def _ready(self) -> bool:
        """Whether enough wall-clock time has passed to publish another tick."""
        now = time.monotonic()
        if now - self._last_publish < self._min_interval:
            return False
        self._last_publish = now
        return True

    def publish_tick(self) -> None:
        """Build and publish a snapshot if warranted (called each sim step).

        No-op when there are no subscribers or when the rate cap has not yet
        elapsed, so the cost of [`build_snapshot`][] is only paid when a
        consumer will actually receive it.
        """
        if not self.active or not self._ready():
            return
        self.publish(self._build_snapshot())

    def publish(self, snapshot: Snapshot) -> None:
        """Store a snapshot as `latest` and wake awaiting consumers."""
        self.latest = snapshot
        self.generation += 1
        # set()+clear() wakes all consumers currently awaiting wait(); the flag
        # is immediately reset so the next wait() blocks until the next tick.
        self._event.set()
        self._event.clear()

    async def wait(self) -> None:
        """Block until the next snapshot is published."""
        await self._event.wait()
