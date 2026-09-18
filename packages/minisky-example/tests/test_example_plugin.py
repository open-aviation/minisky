"""Integration tests owned by the example plugin package."""

from collections.abc import AsyncIterator
from typing import cast

import pytest
from minisky import Err, MagneticDeclinationGrid, MiniSky, MiniSkyConfig, NavData, Ok
from minisky.types import CasMps, StdPressureAltM
from minisky_example import Example

_MAGNETIC_DECLINATION = MagneticDeclinationGrid.load_default()


@pytest.fixture
async def runtime() -> AsyncIterator[MiniSky]:
    runtime = MiniSky(
        MiniSkyConfig(),
        navdata=NavData(),
        magnetic_declination=_MAGNETIC_DECLINATION,
    )
    yield runtime
    await runtime.aclose()


async def run_command(runtime: MiniSky, command: str) -> str:
    invocation = runtime.commands.submit(command)
    runtime.simulation.step()
    match await invocation:
        case Ok(value):
            return value
        case Err(error):
            return error


@pytest.mark.anyio
async def test_commands_and_entity_are_runtime_owned(runtime: MiniSky) -> None:
    result = await runtime.plugins.load("EXAMPLE")
    assert result.is_ok(), result.err()
    assert tuple(runtime.plugins.loaded_plugins) == ("EXAMPLE",)

    await run_command(runtime, "CRE KL001,A320,52,4,90,FL100,250KT[CAS]")
    assert "150" in await run_command(runtime, "PASSENGERS KL001 150")
    assert "150" in await run_command(runtime, "PASSENGERS KL001")
    assert "expected" in (await run_command(runtime, "PASSENGERS KL001 -1")).lower()
    assert "150" in await run_command(runtime, "PASSENGERS KL001")

    again = await runtime.plugins.load("EXAMPLE")
    assert again == Err("Plugin EXAMPLE already loaded")


@pytest.mark.anyio
async def test_entity_sizes_existing_traffic_and_retires(runtime: MiniSky) -> None:
    runtime.traffic.cre(
        "KL001",
        "A320",
        lat=52.0,
        lon=4.0,
        hdg=90,
        alt=StdPressureAltM(3000.0),
        airspeed=CasMps(150.0),
    )
    result = await runtime.plugins.load("EXAMPLE")
    assert result.is_ok(), result.err()
    record = runtime.plugins.loaded_plugins["EXAMPLE"]
    entity = cast(Example, record.entities[0])
    assert record.entities == (entity,)
    assert len(entity.npassengers) == 1
    assert entity._traffic is runtime.traffic

    await runtime.aclose()
    assert entity._retired
    assert entity._traffic is None
