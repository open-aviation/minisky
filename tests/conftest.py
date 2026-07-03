"""Shared fixtures for MiniSky integration tests.

MiniSky uses module-level singletons (minisky.traf, minisky.sim, ...), so:
- minisky.init() is called exactly once per test session (re-initializing
  would leave modules holding references to stale objects);
- each test gets a clean state via minisky.sim.reset();
- always access singletons through the module (bs.traf), never via
  `from minisky import traf` (that binds None at import time).

Note on output: scr.echo() truncates the buffer on every call, so
scr.read_output_buffer() only ever returns the *last* echoed message.
"""

import pytest

import minisky


@pytest.fixture(scope="session")
def bs():
    """Session-wide initialized minisky module."""
    minisky.init()
    return minisky


@pytest.fixture
def sim(bs):
    """Fresh simulation state for each test."""
    bs.sim.reset()
    bs.scr.read_output_buffer()  # drain "Simulation reset" echo
    return bs.sim


@pytest.fixture
def run_cmd(bs, sim):
    """Queue a stack command, step the sim, and return the last echoed output."""

    def _run(cmd, steps=1):
        minisky.stack.stack(cmd)
        for _ in range(steps):
            bs.sim.step()
        return bs.scr.read_output_buffer()

    return _run


@pytest.fixture
def step_until(bs):
    """Step the simulation until a predicate holds, failing after max_steps."""

    def _step(pred, max_steps=600):
        for i in range(max_steps):
            bs.sim.step()
            if pred():
                return i
        pytest.fail(f"condition not met within {max_steps} simulation steps")

    return _step
