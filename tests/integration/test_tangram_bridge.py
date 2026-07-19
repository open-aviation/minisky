"""Integration tests for the tangram Redis bridge (against fakeredis)."""

import json
import time

import fakeredis
import pytest

from example_plugins.tangram import TangramBridge


@pytest.fixture
def redis_server():
    return fakeredis.FakeServer()


@pytest.fixture
def bridge(bs, sim, redis_server):
    bridge = TangramBridge(
        "redis://fake",
        "minisky",
        max_hz=1000,
        redis_factory=lambda url: fakeredis.FakeRedis(server=redis_server),
    )
    ok, msg = bridge.start()
    assert ok, msg
    # The I/O thread subscribes asynchronously; commands published before the
    # subscription is live would be silently lost (pub/sub has no replay).
    assert bridge.ready.wait(timeout=5.0), "bridge did not subscribe in time"
    yield bridge
    bridge.stop()


@pytest.fixture
def observer(redis_server):
    """A second Redis client playing the role of tangram's Channel service."""
    client = fakeredis.FakeRedis(server=redis_server)
    pubsub = client.pubsub(ignore_subscribe_messages=True)
    pubsub.psubscribe("to:*")
    return client, pubsub


def wait_for(pubsub, topic_suffix, pred=lambda payload: True, timeout=5.0):
    """Read pattern messages until one on the given topic satisfies pred."""
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


def test_snapshot_published(bs, sim, bridge, observer, step_until):
    _, pubsub = observer
    bs.stack.stack("CRE KL204 B744 52 4 90 FL300 250")
    step_until(lambda: bs.traf.ntraf == 1)
    bridge.tick()

    payload = wait_for(pubsub, ":new-data", lambda p: p["count"] == 1)
    (ac,) = payload["aircraft"]
    assert ac["callsign"] == "KL204"
    assert ac["altitude"] == 30000
    assert payload["siminfo"]["ntraf"] == 1


def test_command_roundtrip(bs, sim, bridge, observer, step_until):
    client, _ = observer
    bs.stack.stack("CRE KL204 B744 52 4 90 FL300 250")
    step_until(lambda: int(bs.sim.state) == bs.OP)

    client.publish("from:minisky:command", json.dumps({"command": "HOLD"}))
    # The bridge thread stacks the command; the sim applies it on a step.
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and int(bs.sim.state) != bs.HOLD:
        bs.sim.step()
        time.sleep(0.02)
    assert int(bs.sim.state) == bs.HOLD


def test_heartbeat_while_paused(bs, sim, bridge, observer, step_until):
    _, pubsub = observer
    bs.stack.stack("CRE KL204 B744 52 4 90 FL300 250")
    step_until(lambda: bs.traf.ntraf == 1)
    bridge.tick()
    bs.stack.stack("HOLD")
    bs.sim.step()

    # With no further ticks, the bridge must still republish state on its own,
    # and the refreshed siminfo must reflect the pause.
    payload = wait_for(
        pubsub, ":new-data", lambda p: p["siminfo"]["state_name"] == "HOLD", timeout=5.0
    )
    assert payload["aircraft"], "heartbeat should retain the last aircraft list"


def test_heartbeat_before_any_traffic(bs, sim, bridge, observer):
    """A freshly started, idle simulator (INIT, no aircraft, no ticks yet) must
    still announce itself, or the frontend shows 'simulator offline'."""
    _, pubsub = observer
    payload = wait_for(pubsub, ":new-data", timeout=5.0)
    assert payload["count"] == 0
    assert payload["aircraft"] == []
    assert payload["siminfo"]["state_name"] == "INIT"
    assert payload["siminfo"]["nconf_cur"] == 0


def test_console_relay(bs, sim, bridge, observer):
    _, pubsub = observer
    bs.scr.echo("hello tangram")
    payload = wait_for(pubsub, ":console", lambda p: "hello tangram" in p["lines"])
    assert payload["lines"]
