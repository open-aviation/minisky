"""Integration tests for the per-tick streaming API and DTMULT command."""

from __future__ import annotations

import json

import pytest
from minisky import MiniSky
from minisky._internal.simulation import Simulation, SimulationState
from minisky._internal.streaming import (
    STREAM_MAX_HZ,
    ArrayTopic,
    ConnectionSubscriptions,
    RouteTopic,
    StreamHub,
    build_extras,
    build_route_snapshot,
    build_snapshot,
)
from tests._types import RunCommand


def test_snapshot_structure_and_units(
    runtime: MiniSky, sim: Simulation, run_cmd: RunCommand
) -> None:
    # Two steps: the first creates the aircraft, the second flips INIT -> OP.
    run_cmd("CRE KL001 A320 52.0 4.0 90 FL100 250KT[CAS]", steps=2)

    snap = build_snapshot(runtime.simulation, runtime.traffic, runtime.runner, runtime.commands)
    assert set(snap) == {"siminfo", "acdata"}

    info = snap["siminfo"]
    assert set(info) == {
        "speed",
        "simdt",
        "simt",
        "simutc",
        "ntraf",
        "state",
        "scenname",
    }
    assert info["ntraf"] == 1
    assert info["state"] == SimulationState.OP  # running after a CRE
    assert isinstance(info["simutc"], str)

    ac = snap["acdata"]
    assert ac["callsign"] == ["KL001"]
    assert ac["typecode"] == ["A320"]
    # FL100 == 10000 ft == 3048 m, altitude stays SI (metres) on the wire here.
    assert ac["alt"][0] == pytest.approx(3048.0, abs=1.0)
    # Conflict counters are present and zero for a single aircraft.
    assert ac["nconf_cur"] == 0
    assert ac["nconf_tot"] == 0
    assert ac["nlos_cur"] == 0
    assert ac["nlos_tot"] == 0
    assert ac["inconf"] == [False]


def test_snapshot_is_json_serialisable(
    runtime: MiniSky, sim: Simulation, run_cmd: RunCommand
) -> None:
    run_cmd("CRE KL001 A320 52.0 4.0 90 FL100 250KT[CAS]")
    # Must not raise: no numpy scalars leak into the snapshot.
    json.dumps(
        build_snapshot(runtime.simulation, runtime.traffic, runtime.runner, runtime.commands)
    )


def test_snapshot_empty_when_no_traffic(runtime: MiniSky, sim: Simulation) -> None:
    snap = build_snapshot(runtime.simulation, runtime.traffic, runtime.runner, runtime.commands)
    assert snap["siminfo"]["ntraf"] == 0
    assert snap["acdata"]["callsign"] == []
    assert snap["acdata"]["alt"] == []


def test_dtmult_sets_runner_speed(runtime: MiniSky, sim: Simulation, run_cmd: RunCommand) -> None:
    run_cmd("DTMULT 8")
    assert runtime.runner.speed == 8.0


def test_hub_skips_publish_without_subscribers(runtime: MiniSky) -> None:
    hub = StreamHub(
        lambda: build_snapshot(
            runtime.simulation, runtime.traffic, runtime.runner, runtime.commands
        )
    )
    assert hub.active is False
    hub.publish_tick()  # no subscribers -> no snapshot built
    assert hub.latest is None

    hub.subscribe()
    assert hub.active is True


def test_hub_rate_cap_gates_publishing(runtime: MiniSky) -> None:
    # A very low cap means the second immediate tick is dropped.
    hub = StreamHub(
        lambda: build_snapshot(
            runtime.simulation, runtime.traffic, runtime.runner, runtime.commands
        ),
        max_hz=1.0,
    )
    hub.subscribe()
    hub.publish_tick()
    first_gen = hub.generation
    assert first_gen == 1
    hub.publish_tick()  # within the 1 s window -> skipped
    assert hub.generation == first_gen


def test_stream_max_hz_default_is_positive() -> None:
    assert STREAM_MAX_HZ > 0


def test_array_names_includes_core_and_nested_arrays(runtime: MiniSky) -> None:
    names = runtime.traffic.array_names()
    assert "lat" in names  # registered directly on Traffic (the tree root)
    assert "tcpamax" in names  # registered on the nested traf.cd child


def test_find_array_returns_none_for_unknown_name(runtime: MiniSky) -> None:
    assert runtime.traffic.find_array("not_a_real_array") is None


def test_find_array_resolves_core_and_nested_arrays(
    runtime: MiniSky, sim: Simulation, run_cmd: RunCommand
) -> None:
    run_cmd("CRE KL001 A320 52.0 4.0 90 FL100 250KT[CAS]")
    assert runtime.traffic.find_array("lat") is runtime.traffic.lat
    assert runtime.traffic.find_array("tcpamax") is runtime.traffic.cd.tcpamax


def test_build_route_snapshot_for_unknown_callsign(runtime: MiniSky, sim: Simulation) -> None:
    assert build_route_snapshot(runtime.traffic, "GHOST") is None


def test_build_route_snapshot_reflects_added_waypoints(
    runtime: MiniSky, sim: Simulation, run_cmd: RunCommand
) -> None:
    run_cmd("CRE KL001 A320 52.0 4.0 90 FL100 250KT[CAS]")
    run_cmd("ADDWPT KL001 52.5,5.0")

    route = build_route_snapshot(runtime.traffic, "KL001")
    assert route is not None
    assert route["callsign"] == "KL001"
    assert route["active_index"] == 0
    assert len(route["waypoints"]) == 1
    wpt = route["waypoints"][0]
    assert wpt["lat"] == pytest.approx(52.5)
    assert wpt["lon"] == pytest.approx(5.0)
    assert wpt["alt"] is None


def test_connection_subscriptions_unsubscribe(runtime: MiniSky, sim: Simulation) -> None:
    subscriptions = ConnectionSubscriptions()
    topic: RouteTopic = {"kind": "route", "callsign": "KL001"}
    subscriptions.subscribe(topic)
    assert dict(subscriptions.active_topics())["route:KL001"] == topic

    subscriptions.unsubscribe(topic)
    assert subscriptions.active_topics() == []


def test_connection_subscriptions_ttl_expiry(
    runtime: MiniSky, sim: Simulation, monkeypatch: pytest.MonkeyPatch
) -> None:
    now = 1000.0
    monkeypatch.setattr("minisky._internal.streaming.time.monotonic", lambda: now)

    subscriptions = ConnectionSubscriptions()
    subscriptions.subscribe({"kind": "route", "callsign": "KL001"}, ttl=30.0)
    assert len(subscriptions.active_topics()) == 1

    now += 31.0
    assert subscriptions.active_topics() == []


def test_build_extras_keys_route_and_array_topics(
    runtime: MiniSky, sim: Simulation, run_cmd: RunCommand
) -> None:
    run_cmd("CRE KL001 A320 52.0 4.0 90 FL100 250KT[CAS]")
    run_cmd("ADDWPT KL001 52.5,5.0")

    subscriptions = ConnectionSubscriptions()
    subscriptions.subscribe({"kind": "route", "callsign": "KL001"})
    subscriptions.subscribe({"kind": "array", "name": "tcpamax", "callsign": "KL001"})
    subscriptions.subscribe({"kind": "array", "name": "tcpamax", "callsign": None})

    extras = build_extras(subscriptions, runtime.traffic)
    assert set(extras) == {"route:KL001", "array:tcpamax:KL001", "array:tcpamax:"}
    assert extras["route:KL001"]["callsign"] == "KL001"  # type: ignore[index]
    assert extras["array:tcpamax:KL001"] == pytest.approx(0.0)
    assert extras["array:tcpamax:"] == [pytest.approx(0.0)]


def test_build_extras_omits_unresolvable_topics(runtime: MiniSky, sim: Simulation) -> None:
    subscriptions = ConnectionSubscriptions()
    subscriptions.subscribe({"kind": "route", "callsign": "GHOST"})
    subscriptions.subscribe({"kind": "array", "name": "not_a_real_array", "callsign": None})
    ghost_array_topic: ArrayTopic = {"kind": "array", "name": "tcpamax", "callsign": "GHOST"}
    subscriptions.subscribe(ghost_array_topic)

    assert build_extras(subscriptions, runtime.traffic) == {}
