"""Per-tick streaming of simulation state.

Provides a small, transport-agnostic mechanism to push a full snapshot of the
simulation once per timestep. :func:`build_snapshot` reads the singletons
(``minisky.sim``, ``minisky.traf``) and returns a plain, JSON-serialisable dict
in **SI units**; :class:`StreamHub` fans that snapshot out to any number of
awaiting consumers (e.g. WebSocket connections in :mod:`minisky.server`).

This is a generic streaming API: it emits raw SI state and takes no position on
any particular client or wire contract. Unit conversion and field mapping to a
specific consumer's format happen downstream, in that consumer, not here.

Snapshot shape::

    {
        "siminfo": {speed, simdt, simt, simutc, ntraf, state, scenname},
        "acdata":  {callsign, lat, lon, alt, trk, vs, tas, cas, gs, typecode,
                    inconf, tcpamax, nconf_cur, nconf_tot, nlos_cur, nlos_tot},
    }

Units on the wire here are SI: positions in decimal degrees, ``alt`` in metres,
speeds (``tas``/``cas``/``gs``) in m/s, ``vs`` in m/s, ``trk`` in degrees,
``simt``/``simdt`` in seconds. ``state`` is the numeric simulation state
(0=INIT, 1=HOLD, 2=OP, 3=END). Each tick is a full snapshot; aircraft are
identified by ``callsign`` for their lifetime.
"""

import asyncio
import time
from typing import Any, cast

import numpy as np

import minisky

# Default upper bound on how often a snapshot is published, in Hz. The
# simulation may step much faster than this in fast-forward; publishing is
# gated to at most this wall-clock rate so consumers are not flooded.
STREAM_MAX_HZ = 10.0


def _tolist(arr: Any) -> list:
    """Convert a numpy array (or list) into a plain JSON-serialisable list."""
    if isinstance(arr, np.ndarray):
        return cast(list, arr.tolist())
    return list(arr)


def build_snapshot() -> dict[str, Any]:
    """Build a full snapshot of the current simulation state (SI units).

    Reads ``minisky.sim`` and ``minisky.traf`` and returns a plain dict of
    Python scalars and lists (no numpy types), safe to serialise as JSON.

    Returns:
        A dict with ``siminfo`` and ``acdata`` keys as described in the module
        docstring.
    """
    sim = minisky.sim
    traf = minisky.traf
    cd = traf.cd

    siminfo = {
        "speed": float(minisky.runner.speed),
        "simdt": float(sim.simdt),
        "simt": float(sim.simt),
        "simutc": sim.utc.isoformat(),
        "ntraf": int(traf.ntraf),
        "state": int(sim.state),
        "scenname": minisky.stack.get_scenname(),
    }

    acdata = {
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
        "inconf": _tolist(cd.inconf),
        "tcpamax": _tolist(cd.tcpamax),
        "nconf_cur": len(cd.confpairs_unique),
        "nconf_tot": len(cd.confpairs_all),
        "nlos_cur": len(cd.lospairs_unique),
        "nlos_tot": len(cd.lospairs_all),
    }

    return {"siminfo": siminfo, "acdata": acdata}


class StreamHub:
    """Fan-out hub distributing per-tick snapshots to awaiting consumers.

    A single hub is shared by the streaming endpoint. The simulation loop calls
    :meth:`publish_tick` once per step (via a plugin ``update`` hook); each
    connected consumer awaits :meth:`wait` and then reads :attr:`latest`.

    Snapshot construction is skipped entirely when there are no subscribers,
    and gated to at most ``max_hz`` publications per wall-clock second so that a
    fast-forwarding simulation does not flood consumers.

    Attributes:
        latest: The most recently published snapshot (``None`` until the first
            publish), used to seed newly connected consumers.
        generation: Monotonically increasing counter incremented on each
            publish; consumers may use it to detect missed ticks.
    """

    def __init__(self, max_hz: float = STREAM_MAX_HZ) -> None:
        self._subscribers = 0
        self._event = asyncio.Event()
        self._min_interval = 1.0 / max_hz if max_hz > 0 else 0.0
        self._last_publish = 0.0
        self.latest: dict[str, Any] | None = None
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
        elapsed, so the cost of :func:`build_snapshot` is only paid when a
        consumer will actually receive it.
        """
        if not self.active or not self._ready():
            return
        self.publish(build_snapshot())

    def publish(self, snapshot: dict[str, Any]) -> None:
        """Store a snapshot as :attr:`latest` and wake awaiting consumers."""
        self.latest = snapshot
        self.generation += 1
        # set()+clear() wakes all consumers currently awaiting wait(); the flag
        # is immediately reset so the next wait() blocks until the next tick.
        self._event.set()
        self._event.clear()

    async def wait(self) -> None:
        """Block until the next snapshot is published."""
        await self._event.wait()


# Shared hub used by the streaming endpoint.
hub = StreamHub()


def register_stream_hook() -> None:
    """Register the per-step publish hook on the simulation's ``update`` cycle.

    Attaches :meth:`StreamHub.publish_tick` to the plugin ``update`` hook so a
    snapshot is published after every traffic update. Idempotent: registering
    twice keeps a single hook. Must be called after :func:`minisky.init`.
    """
    from minisky.plugin.timedfunction import hooks

    hooks.update.setdefault("stream_snapshot", hub.publish_tick)
