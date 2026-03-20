"""Main BlueSky start script"""

import asyncio
from functools import wraps

import click

import minisky


def coroutine(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        return asyncio.run(f(*args, **kwargs))

    return wrapper


@click.command()
@click.option("--scenario", required=True, help="scenario file for simulation")
@click.option("--speed", default=1, help="simulation speed")
@coroutine
async def main(scenario, speed):
    """Main function to start BlueSky"""
    minisky.init(scenario=scenario)
    minisky.load_plugins()
    minisky.runner.speed = speed

    await minisky.runner.run()


if __name__ == "__main__":
    asyncio.run(main())
