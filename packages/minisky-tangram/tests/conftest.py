"""Runtime fixtures for the MiniSky tangram bridge tests."""

from __future__ import annotations

from collections.abc import Callable, Iterator

import pytest
from minisky import MiniSky, MiniSkyConfig
from minisky.simulation import Simulation


@pytest.fixture(scope="session")
def runtime() -> Iterator[MiniSky]:
    instance = MiniSky(MiniSkyConfig())
    yield instance
    instance.close()


@pytest.fixture
def sim(runtime: MiniSky) -> Simulation:
    runtime.simulation.reset()
    runtime.console.read_output_buffer()
    return runtime.simulation


@pytest.fixture
def step_until(runtime: MiniSky) -> Callable[[Callable[[], bool]], int]:
    def _step(pred: Callable[[], bool], max_steps: int = 600) -> int:
        for index in range(max_steps):
            runtime.simulation.step()
            if pred():
                return index
        pytest.fail(f"condition not met within {max_steps} simulation steps")

    return _step
