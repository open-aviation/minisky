"""Shared fixtures for MiniSky integration tests.

One explicit runtime is constructed for the test session. Each test resets the
simulation state before use. Output from `ConsoleIO.echo()` is destructive:
`read_output_buffer()` returns only the most recently echoed message.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator

import pytest
from minisky import MiniSky
from minisky.core.config import MiniSkyConfig
from minisky.simulation.simulation import Simulation
from tests._types import RunCommand, StepUntil


@pytest.fixture(scope="session")
def config() -> MiniSkyConfig:
    """Immutable default runtime configuration shared by tests."""
    return MiniSkyConfig()


@pytest.fixture(scope="session")
def runtime(config: MiniSkyConfig) -> Iterator[MiniSky]:
    """Session-wide explicit MiniSky runtime."""
    instance = MiniSky(config)
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
    """Queue a stack command, step the sim, and return the last echoed output."""

    def _run(cmd: str, steps: int = 1) -> str:
        runtime.commands.stack(cmd)
        for _ in range(steps):
            runtime.simulation.step()
        return runtime.console.read_output_buffer()

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
