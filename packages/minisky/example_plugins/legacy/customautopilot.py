from __future__ import annotations

from typing import TYPE_CHECKING

from minisky_example_customautopilot import CustomAutoPilot
from minisky_example_customautopilot import init_plugin as _init_plugin

if TYPE_CHECKING:
    from minisky import MiniSky


def init_plugin(runtime: MiniSky) -> dict[str, str]:
    return _init_plugin(runtime)


__all__ = ("CustomAutoPilot", "init_plugin")
