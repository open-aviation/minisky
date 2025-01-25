"""Main BlueSky start script"""

import asyncio
from functools import wraps

import click

import clearsky as cs


def coroutine(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        return asyncio.run(f(*args, **kwargs))

    return wrapper


@click.command()
@click.option("--scenario", required=True, help="scenario file for simulation")
@coroutine
async def main(scenario):
    """Main function to start BlueSky"""
    cs.init(scenario=scenario)
    await cs.runner.run()


if __name__ == "__main__":
    asyncio.run(main())
