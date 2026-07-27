"""Shared fixtures for MiniSky integration tests.

Most existing integration tests still exercise the temporary module-level
compatibility aliases (`minisky.traf`, `minisky.sim`, ...), so:
- one explicit runtime is constructed for the test session and activates those
  aliases;
- each test gets a clean state via minisky.sim.reset();
- always access singletons through the module (bs.traf), never via
  `from minisky import traf` (that binds None at import time).

Note on output: scr.echo() truncates the buffer on every call, so
scr.read_output_buffer() only ever returns the *last* echoed message.
"""

import pytest

import minisky


@pytest.fixture(scope="session")
def runtime():
    """Session-wide explicit MiniSky runtime."""
    return minisky.init()


@pytest.fixture(scope="session")
def bs(runtime):
    """Compatibility module activated for the session runtime."""
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
