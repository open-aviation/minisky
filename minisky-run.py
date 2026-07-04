"""Run a scenario file without interaction.

Initialises the simulator with the given scenario, loads any plugins enabled in
``settings.yml``, and steps the simulation with the async Runner until the scenario
ends (e.g. via a ``QUIT`` command)::

    python minisky-run.py --scenario scenarios/kl204.scn [--speed 10]
"""

import asyncio
from collections.abc import Callable, Coroutine
from functools import wraps
from typing import Any, TypeVar

import click

import minisky

T = TypeVar("T")


def coroutine(f: Callable[..., Coroutine[Any, Any, T]]) -> Callable[..., T]:
    """Wrap an async click command so it runs inside asyncio.run()."""

    @wraps(f)
    def wrapper(*args: Any, **kwargs: Any) -> T:
        return asyncio.run(f(*args, **kwargs))

    return wrapper


@click.command()
@click.option("--scenario", required=True, help="scenario file for simulation")
@click.option("--speed", default=1, help="simulation speed")
@coroutine
async def main(scenario: str, speed: int) -> None:
    """Initialise the simulator with a scenario and run it to completion.

    Args:
        scenario: Path to the scenario (.scn) file to run.
        speed: Simulation speed multiplier relative to wall time (default 1).
    """
    minisky.init(scenario=scenario)
    minisky.load_plugins()
    minisky.runner.speed = speed

    await minisky.runner.run()


if __name__ == "__main__":
    asyncio.run(main())
