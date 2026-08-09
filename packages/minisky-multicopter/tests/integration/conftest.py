"""Shared fixtures for the multicopter integration tests.

The plugin tests run on their own runtime with the MULTICOPTER plugin
loaded — the plugin registers replaceable implementations on load and
selects them from its hooks (first step after load, and after every reset),
so the shared session runtime of the other integration tests keeps the core
implementations.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterator

import pytest
from minisky import MiniSky
from minisky.core.config import MiniSkyConfig
from minisky.simulation import Simulation
from tests._types import RunCommand, StepUntil


@pytest.fixture(scope="module")
def mcruntime() -> Iterator[MiniSky]:
    """Module-wide MiniSky runtime with the MULTICOPTER plugin loaded."""
    instance = MiniSky(MiniSkyConfig())
    result = asyncio.run(instance.plugins.load("MULTICOPTER"))
    assert result.is_ok(), result.err()
    yield instance
    asyncio.run(instance.aclose())


@pytest.fixture
def mcsim(mcruntime: MiniSky) -> Simulation:
    """Fresh simulation state; the plugin reset hook re-selects the impls."""
    mcruntime.simulation.reset()
    mcruntime.console.read_output_buffer()  # drain "Simulation reset" echo
    return mcruntime.simulation


@pytest.fixture
def run_mc(mcruntime: MiniSky, mcsim: Simulation) -> RunCommand:
    """Queue a stack command, step the sim, and return the last echoed output."""

    def _run(cmd: str, steps: int = 1) -> str:
        mcruntime.commands.stack(cmd)
        for _ in range(steps):
            mcruntime.simulation.step()
        return mcruntime.console.read_output_buffer()

    return _run


@pytest.fixture
def step_mc(mcruntime: MiniSky) -> StepUntil:
    """Step the simulation until a predicate holds, failing after max_steps."""

    def _step(pred: Callable[[], bool], max_steps: int = 600) -> int:
        for i in range(max_steps):
            mcruntime.simulation.step()
            if pred():
                return i
        pytest.fail(f"condition not met within {max_steps} simulation steps")

    return _step
