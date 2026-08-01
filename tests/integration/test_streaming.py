"""Integration tests for the per-tick streaming API and DTMULT command.

Covers [`build_snapshot`][minisky.streaming.build_snapshot] against a live simulation and
the `DTMULT` stack command that sets the runner speed multiplier.
"""

from __future__ import annotations

import json

import pytest

from minisky import MiniSky
from minisky.simulation import Simulation, SimulationState
from minisky.streaming import STREAM_MAX_HZ, StreamHub, build_snapshot
from tests._types import RunCommand


def test_snapshot_structure_and_units(
    runtime: MiniSky, sim: Simulation, run_cmd: RunCommand
) -> None:
    # Two steps: the first creates the aircraft, the second flips INIT -> OP.
    run_cmd("CRE KL001 A320 52.0 4.0 90 FL100 250", steps=2)

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
    run_cmd("CRE KL001 A320 52.0 4.0 90 FL100 250")
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


def test_dtmult_rejects_non_positive(runtime: MiniSky, sim: Simulation) -> None:
    ok, msg = runtime.runner.setspeed(0)
    assert ok is False
    assert "positive" in msg.lower()


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
