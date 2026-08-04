"""Integration tests owned by the example plugin package."""

from typing import cast

import pytest
from minisky import Err, MiniSky, MiniSkyConfig
from minisky_example import Example


def run_command(runtime: MiniSky, command: str) -> str:
    runtime.commands.stack(command)
    runtime.simulation.step()
    return runtime.console.read_output_buffer()


@pytest.mark.anyio
async def test_commands_and_entity_are_runtime_owned() -> None:
    runtime = MiniSky(MiniSkyConfig())
    try:
        result = await runtime.plugins.load("EXAMPLE")
        assert result.is_ok(), result.err()
        record = runtime.plugins.plugins["EXAMPLE"]
        assert record.loaded
        assert tuple(runtime.plugins.loaded_plugins) == ("EXAMPLE",)

        run_command(runtime, "CRE KL001,A320,52,4,90,FL100,250")
        assert "150" in run_command(runtime, "PASSENGERS KL001 150")
        assert "150" in run_command(runtime, "PASSENGERS KL001")
        assert "expected" in run_command(runtime, "PASSENGERS KL001 -1").lower()
        assert "150" in run_command(runtime, "PASSENGERS KL001")

        again = await runtime.plugins.load("EXAMPLE")
        assert again == Err("Plugin EXAMPLE already loaded")
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_entity_sizes_existing_traffic_and_retires() -> None:
    runtime = MiniSky(MiniSkyConfig())
    runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
    result = await runtime.plugins.load("EXAMPLE")
    assert result.is_ok(), result.err()
    record = runtime.plugins.plugins["EXAMPLE"]
    entity = cast(Example, record.entities[0])
    assert record.entities == (entity,)
    assert len(entity.npassengers) == 1
    assert entity._traffic is runtime.traffic

    await runtime.aclose()
    assert entity._retired
    assert entity._traffic is None
