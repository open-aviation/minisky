"""Runtime fixtures for the multicopter plugin tests.

The plugin tests build their own runtime with the MULTICOPTER plugin loaded
(see `mcruntime` in the test module); the plain `runtime`/`sim` fixtures here
provide a reference runtime with the core implementations for regression
comparisons.
"""

from __future__ import annotations

from collections.abc import Iterator

import pytest
from minisky import MiniSky, MiniSkyConfig
from minisky.simulation.simulation import Simulation


@pytest.fixture(scope="session")
def runtime() -> Iterator[MiniSky]:
    instance = MiniSky(MiniSkyConfig())
    yield instance
    instance.close()


@pytest.fixture
def sim(runtime: MiniSky) -> Simulation:
    runtime.simulation.reset()
    runtime.console.read_output_buffer()  # drain "Simulation reset" echo
    return runtime.simulation
