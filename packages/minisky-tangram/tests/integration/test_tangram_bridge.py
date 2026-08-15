"""Integration tests for the tangram Redis bridge (against fakeredis)."""

import json
import time
from collections.abc import Callable, Iterator
from typing import Any

import fakeredis
import pytest
from minisky import MiniSky
from minisky.simulation import Simulation, SimulationState
from minisky_tangram import TangramBridge
from redis.client import PubSub

Observer = tuple[fakeredis.FakeRedis, PubSub]
StepUntil = Callable[[Callable[[], bool]], int]


@pytest.fixture
def anyio_backend() -> str:
    return "asyncio"


@pytest.fixture
def redis_server() -> fakeredis.FakeServer:
    return fakeredis.FakeServer()


@pytest.fixture
def bridge(
    runtime: MiniSky, sim: Simulation, redis_server: fakeredis.FakeServer
) -> Iterator[TangramBridge]:
    del sim
    bridge = TangramBridge(
        "redis://fake",
        "minisky",
        max_hz=1000,
        redis_factory=lambda url: fakeredis.FakeRedis(server=redis_server),
    )
    # TODO(abraham): load tangram through PluginManager when thread shutdown
    # ownership is hardened.
    plugin_runtime = runtime.plugins._plugin_runtime()
    plugin_runtime._activate()
    plugin_runtime.subscribe_console(bridge.capture_console)
    result = bridge.start(plugin_runtime)
    assert result.is_ok(), result.err()
    # The I/O thread subscribes asynchronously; commands published before the
    # subscription is live would be silently lost (pub/sub has no replay).
    assert bridge.ready.wait(timeout=5.0), "bridge did not subscribe in time"
    yield bridge
    plugin_runtime._revoke()
    bridge.stop()


@pytest.fixture
def observer(redis_server: fakeredis.FakeServer) -> Observer:
    """A second Redis client playing the role of tangram's Channel service."""
    client = fakeredis.FakeRedis(server=redis_server)
    pubsub = client.pubsub(ignore_subscribe_messages=True)
    pubsub.psubscribe("to:*")
    return client, pubsub


def wait_for(
    pubsub: PubSub,
    topic_suffix: str,
    pred: Callable[[dict[str, Any]], bool] = lambda payload: True,
    timeout: float = 5.0,
) -> dict[str, Any]:
    """Read pattern messages until a message on the given topic satisfies pred."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = pubsub.get_message(timeout=0.05)
        if message is None or message["type"] != "pmessage":
            continue
        if not message["channel"].decode().endswith(topic_suffix):
            continue
        payload = json.loads(message["data"])
        if pred(payload):
            return payload
    pytest.fail(f"no message on *{topic_suffix} satisfying predicate within {timeout}s")


def test_snapshot_published(
    runtime: MiniSky,
    sim: Simulation,
    bridge: TangramBridge,
    observer: Observer,
    step_until: StepUntil,
) -> None:
    _, pubsub = observer
    runtime.commands.stack("CRE KL204 B744 52 4 90 FL300 250KT[CAS]")
    step_until(lambda: runtime.traffic.ntraf == 1)
    bridge.tick()

    payload = wait_for(pubsub, ":new-data", lambda p: p["count"] == 1)
    (ac,) = payload["aircraft"]
    assert ac["callsign"] == "KL204"
    assert ac["altitude"] == 30000
    assert payload["siminfo"]["ntraf"] == 1


def test_command_roundtrip(
    runtime: MiniSky,
    sim: Simulation,
    bridge: TangramBridge,
    observer: Observer,
    step_until: StepUntil,
) -> None:
    client, _ = observer
    runtime.commands.stack("CRE KL204 B744 52 4 90 FL300 250KT[CAS]")
    step_until(lambda: runtime.simulation.state == SimulationState.OP)

    client.publish("from:minisky:command", json.dumps({"command": "HOLD"}))
    # The bridge thread stacks the command; the sim applies it on a step.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and runtime.simulation.state != SimulationState.HOLD:
        runtime.simulation.step()
        time.sleep(0.02)
    assert runtime.simulation.state == SimulationState.HOLD


def test_heartbeat_while_paused(
    runtime: MiniSky,
    sim: Simulation,
    bridge: TangramBridge,
    observer: Observer,
    step_until: StepUntil,
) -> None:
    _, pubsub = observer
    runtime.commands.stack("CRE KL204 B744 52 4 90 FL300 250KT[CAS]")
    step_until(lambda: runtime.traffic.ntraf == 1)
    bridge.tick()
    runtime.commands.stack("HOLD")
    runtime.simulation.step()

    # With no further ticks, the bridge must still republish state on its own,
    # and the refreshed siminfo must reflect the pause.
    payload = wait_for(
        pubsub, ":new-data", lambda p: p["siminfo"]["state_name"] == "HOLD", timeout=5.0
    )
    assert payload["aircraft"], "heartbeat should retain the last aircraft list"


def test_heartbeat_before_any_traffic(
    runtime: MiniSky, sim: Simulation, bridge: TangramBridge, observer: Observer
) -> None:
    """A freshly started, idle simulator (INIT, no aircraft, no ticks yet) must
    still announce itself, or the frontend shows 'simulator offline'."""
    _, pubsub = observer
    payload = wait_for(pubsub, ":new-data", timeout=5.0)
    assert payload["count"] == 0
    assert payload["aircraft"] == []
    assert payload["siminfo"]["state_name"] == "INIT"
    assert payload["siminfo"]["nconf_cur"] == 0


def test_console_relay(
    runtime: MiniSky, sim: Simulation, bridge: TangramBridge, observer: Observer
) -> None:
    _, pubsub = observer
    runtime.console.echo("hello tangram")
    payload = wait_for(pubsub, ":console", lambda p: "hello tangram" in p["lines"])
    assert payload["lines"]
