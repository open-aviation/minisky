"""tangram frontend plugin rendering live traffic from a MiniSky simulator.

This package is deliberately frontend-only: it registers no API routes and
runs no background services inside tangram. All simulator-side logic (unit
conversion, snapshot publishing, command handling) lives in MiniSky's own
``TANGRAM`` plugin (``example_plugins/tangram.py`` in the MiniSky repo),
which talks to tangram exclusively over Redis pub/sub:

- ``to:<channel>:new-data``  full state snapshots (aviation units)
- ``to:<channel>:console``   echoed simulator console output
- ``from:<channel>:command`` stack commands pushed from the browser

The only thing tangram needs from this package is the compiled frontend
bundle in ``dist-frontend`` and the configuration schema below.
"""

from dataclasses import dataclass
from typing import Annotated

import tangram_core
from tangram_core.config import FrontendMutable


@dataclass(frozen=True)
class MiniskyConfig:
    channel: str = "minisky"
    """Redis channel name; must match ``tangram_channel`` in MiniSky's settings.yml."""
    topbar_order: int = 45
    sidebar_order: int = 45


@dataclass(frozen=True)
class MiniskyFrontendConfig:
    channel: str
    topbar_order: Annotated[int, FrontendMutable()]
    sidebar_order: Annotated[int, FrontendMutable()]


def into_frontend(config: MiniskyConfig) -> MiniskyFrontendConfig:
    return MiniskyFrontendConfig(
        channel=config.channel,
        topbar_order=config.topbar_order,
        sidebar_order=config.sidebar_order,
    )


plugin = tangram_core.Plugin(
    frontend_path="dist-frontend",
    config_class=MiniskyConfig,
    frontend_config_class=MiniskyFrontendConfig,
    into_frontend_config_function=into_frontend,
)
