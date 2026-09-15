"""Shared fixtures for MiniSky integration tests.

One explicit runtime is constructed for the test session. Each test resets the
simulation state before use.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterator
from pathlib import Path

import pytest
from minisky import Err, MagneticDeclinationGrid, MiniSky, Ok
from minisky._internal.config import MiniSkyConfig
from minisky._internal.simulation import Simulation
from tests._types import RunCommand, StepUntil

_REPO_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="session")
def config() -> MiniSkyConfig:
    """Immutable default runtime configuration shared by tests."""
    return MiniSkyConfig()


@pytest.fixture(scope="session")
def runtime(config: MiniSkyConfig) -> Iterator[MiniSky]:
    from minisky_xplane_navdata import load

    instance = MiniSky(
        config,
        navdata=load(),
        magnetic_declination=MagneticDeclinationGrid.load_default(),
    )
    yield instance
    instance.close()


@pytest.fixture
def sim(runtime: MiniSky) -> Simulation:
    """Fresh simulation state for each test."""
    runtime.simulation.reset()
    runtime.console.read_output_buffer()  # drain "Simulation reset" echo
    return runtime.simulation


@pytest.fixture
def run_cmd(runtime: MiniSky, sim: Simulation) -> RunCommand:
    """Submit a command, step the sim, and return its direct response."""

    def _run(cmd: str, steps: int = 1) -> str:
        async def execute() -> str:
            invocation = runtime.commands.submit(cmd)
            for _ in range(steps):
                runtime.simulation.step()
            match await invocation:
                case Ok(value):
                    return value
                case Err(error):
                    return error

        return asyncio.run(execute())

    return _run


@pytest.fixture
def step_until(runtime: MiniSky) -> StepUntil:
    """Step the simulation until a predicate holds, failing after max_steps."""

    def _step(pred: Callable[[], bool], max_steps: int = 600) -> int:
        for i in range(max_steps):
            runtime.simulation.step()
            if pred():
                return i
        pytest.fail(f"condition not met within {max_steps} simulation steps")

    return _step
