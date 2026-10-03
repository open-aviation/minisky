"""Per-tick streaming of simulation state.

Provides a small, transport-agnostic mechanism to push a full snapshot of a
simulation runtime once per timestep. [`build_snapshot`][.build_snapshot] receives the runtime
explicitly and returns a plain, JSON-serialisable dict in **SI units**;
[`StreamHub`][.StreamHub] fans that snapshot out to any number of awaiting consumers
(e.g. WebSocket connections in the HTTP server).

This is a generic streaming API: it emits raw SI state and takes no position on
any particular client or wire contract. Unit conversion and field mapping to a
specific consumer's format happen downstream, in that consumer, not here.

The snapshot shape is defined by the [`Snapshot`][minisky.Snapshot], [`SimInfo`][minisky.SimInfo], and
[`AcData`][minisky.AcData] TypedDicts below.

Units on the wire here are SI: positions in decimal degrees, `alt` in metres,
speeds (`tas`/`cas`/`gs`) in m/s, `vs` in m/s, `trk` in degrees,
`simt`/`simdt` in seconds. `state` is the integer value of
[`SimulationState`][minisky.SimulationState]. Each tick is a full snapshot; aircraft are
identified by `callsign` for their lifetime.
"""

from __future__ import annotations

import asyncio
import time
from collections.abc import Callable
from typing import TYPE_CHECKING, Literal, TypeAlias, TypedDict

import numpy as np

from minisky import quantities as q
from minisky._internal.traffic_arrays import OptionalArray, VariantArray
from minisky.types import AircraftCallsign, AircraftTypeCode

if TYPE_CHECKING:
    from minisky._internal.runner import Runner
    from minisky._internal.simulation import Simulation
    from minisky._internal.stack import CommandStack
    from minisky._internal.traffic import Traffic

# Default upper bound on how often a snapshot is published, in Hz. The
# simulation may step much faster than this in fast-forward; publishing is
# gated to at most this wall-clock rate so consumers are not flooded.
STREAM_MAX_HZ: q.FrequencyHz[float] = 10.0


class SimInfo(TypedDict):
    """Simulation-level snapshot fields."""

    speed: float  # runner speed multiplier (x realtime)
    simdt: q.DurationS[float]
    simt: q.SimulationTimeS[float]
    simutc: str  # ISO-8601, timezone-aware
    ntraf: int
    state: int  # Serialized SimulationState value.
    scenname: str  # "" when no scenario is loaded


class AcData(TypedDict):
    """Per-aircraft snapshot columns (one element per aircraft, SI units)."""

    callsign: list[AircraftCallsign]
    lat: list[q.LatitudeDeg[float]]
    lon: list[q.LongitudeDeg[float]]
    alt: list[q.PressureAltitudeM[float]]
    trk: list[q.GroundTrackDeg[float]]
    vs: list[q.VerticalRateMps[float]]
    tas: list[q.TrueAirspeedMps[float]]
    cas: list[q.CalibratedAirspeedMps[float]]
    gs: list[q.GroundSpeedMps[float]]
    typecode: list[AircraftTypeCode]
    inconf: list[bool]
    tcpamax: list[q.DurationS[float]]
    nconf_cur: int
    nconf_tot: int
    nlos_cur: int
    nlos_tot: int


class Snapshot(TypedDict):
    """Full per-tick simulation snapshot."""

    siminfo: SimInfo
    acdata: AcData


def _tolist(arr) -> list[float]:
    """Convert a numpy array (or list) into a plain JSON-serialisable list."""
    if isinstance(arr, np.ndarray):
        values: list[float] = arr.tolist()
        return values
    return list(arr)


def serialize_array(value: object) -> list:
    """Convert a registered traffic array (any of its supported kinds) to JSON.

    Public so REST endpoints exposing arbitrary traffic arrays (core or
    plugin-added) can reuse the same conversion as the stream extras below.
    """
    if isinstance(value, VariantArray):
        return _tolist(value.values)
    if isinstance(value, OptionalArray):
        return _tolist(value.values)
    return _tolist(value)  # type: ignore[arg-type]


class RouteWaypoint(TypedDict):
    """One waypoint on an aircraft's route."""

    name: str
    type: str
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]
    alt: q.PressureAltitudeM[float] | None
    flyby: bool


class RouteSnapshot(TypedDict):
    """Full route of a single aircraft, by callsign."""

    callsign: AircraftCallsign
    active_index: int | None
    waypoints: list[RouteWaypoint]


def build_route_snapshot(traffic: Traffic, callsign: AircraftCallsign) -> RouteSnapshot | None:
    """Serialize one aircraft's route, or `None` if `callsign` is unknown."""
    idx = traffic.idx(callsign)
    if idx is None:
        return None

    route = traffic.ap.route[idx]
    waypoints: list[RouteWaypoint] = [
        {
            "name": str(name),
            "type": str(wptype),
            "lat": float(lat),
            "lon": float(lon),
            "alt": float(alt) if alt is not None else None,
            "flyby": bool(flyby),
        }
        for name, wptype, lat, lon, alt, flyby in zip(
            route.wpname,
            route.wptype,
            route.wplat,
            route.wplon,
            route.wpalt,
            route.wpflyby,
            strict=True,
        )
    ]
    return {
        "callsign": callsign,
        "active_index": route.iactwp,
        "waypoints": waypoints,
    }


def build_snapshot(
    simulation: Simulation,
    traffic: Traffic,
    runner: Runner,
    commands: CommandStack,
) -> Snapshot:
    """Build a full snapshot from explicit runtime components in SI units."""
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
        "alt": _tolist(traf.alt),
        "trk": _tolist(traf.trk),
        "vs": _tolist(traf.vs),
        "tas": _tolist(traf.tas),
        "cas": _tolist(traf.cas),
        "gs": _tolist(traf.gs),
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

    Each runtime owns a hub. Its simulation calls [`publish_tick`][.publish_tick]
    once per step; each connected consumer awaits [`wait`][.wait] and then
    reads `latest`.

    Snapshot construction is skipped entirely when there are no subscribers,
    and gated to at most `max_hz` publications per wall-clock second so that a
    fast-forwarding simulation does not flood consumers.
    """

    def __init__(
        self,
        build_snapshot: Callable[[], Snapshot],
        max_hz: q.FrequencyHz[float] = STREAM_MAX_HZ,
    ) -> None:
        self._build_snapshot = build_snapshot
        self._subscribers = 0
        self._event = asyncio.Event()
        self._min_interval: q.DurationS[float] = 1.0 / max_hz if max_hz > 0 else 0.0  # pyright: ignore[reportGeneralTypeIssues]
        self._last_publish = 0.0
        self.latest: Snapshot | None = None
        self.generation = 0
        """Monotonically increases whenever a snapshot is published."""
        self._closed = False

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
        elapsed, so the cost of [`build_snapshot`][...build_snapshot] is only paid when a
        consumer will actually receive it.
        """
        if self._closed or not self.active or not self._ready():
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
        if self._closed:
            raise RuntimeError("Stream hub is closed")

    def close(self) -> None:
        """Close the hub and wake any waiting consumers."""
        self._closed = True
        self._subscribers = 0
        self._event.set()


class RouteTopic(TypedDict):
    """Subscribe to one aircraft's route, layered onto the shared stream."""

    kind: Literal["route"]
    callsign: AircraftCallsign


class ArrayTopic(TypedDict):
    """Subscribe to a registered traffic array, optionally for one aircraft."""

    kind: Literal["array"]
    name: str
    callsign: AircraftCallsign | None


Topic: TypeAlias = RouteTopic | ArrayTopic


def _topic_key(topic: Topic) -> str:
    """A stable string key identifying `topic`, used both to dedupe subscriptions
    and as the `extras` payload key so clients can tell topics apart."""
    if topic["kind"] == "route":
        return f"route:{topic['callsign']}"
    return f"array:{topic['name']}:{topic['callsign'] or ''}"


class ConnectionSubscriptions:
    """Per-connection extra topics layered onto the shared stream snapshot.

    Each topic lives until explicitly removed or, if given a `ttl`, until
    that many seconds have elapsed. Expired entries are pruned lazily, on the
    next `active_topics()` call.
    """

    def __init__(self) -> None:
        self._topics: dict[str, tuple[Topic, float | None]] = {}

    def subscribe(self, topic: Topic, ttl: q.DurationS[float] | None = None) -> None:
        """Add or replace a subscription, optionally expiring after `ttl` seconds."""
        expiry = time.monotonic() + ttl if ttl is not None else None
        self._topics[_topic_key(topic)] = (topic, expiry)

    def unsubscribe(self, topic: Topic) -> None:
        """Remove a subscription; a no-op if it isn't active."""
        self._topics.pop(_topic_key(topic), None)

    def active_topics(self) -> list[tuple[str, Topic]]:
        """Currently active (key, topic) pairs, pruning any that have expired."""
        now = time.monotonic()
        expired = [
            key for key, (_, expiry) in self._topics.items() if expiry is not None and expiry <= now
        ]
        for key in expired:
            del self._topics[key]
        return [(key, topic) for key, (topic, _) in self._topics.items()]


def build_extras(subscriptions: ConnectionSubscriptions, traffic: Traffic) -> dict[str, object]:
    """Build the per-connection `extras` payload for `subscriptions`' active topics.

    A topic that no longer resolves (deleted aircraft, unregistered array
    name) is silently omitted rather than raising.
    """
    extras: dict[str, object] = {}
    for key, topic in subscriptions.active_topics():
        if topic["kind"] == "route":
            route = build_route_snapshot(traffic, topic["callsign"])
            if route is not None:
                extras[key] = route
            continue

        array = traffic.find_array(topic["name"])
        if array is None:
            continue
        values = serialize_array(array)
        if topic["callsign"] is None:
            extras[key] = values
            continue
        idx = traffic.idx(topic["callsign"])
        if idx is not None:
            extras[key] = values[idx]
    return extras
