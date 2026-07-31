"""Public contracts for runtime-local MiniSky plugins.

Plugin packages can expose a [Plugin][minisky.plugin.Plugin] value through
the `minisky.plugins` entry-point group.
"""
# keeping init_plugin(runtime) for now

from __future__ import annotations

from minisky.plugin.entity import Entity
from minisky.plugin.plugin import Plugin, PluginContext, PluginError, PluginManager, PluginSpec
from minisky.plugin.plugin_decorators import command
from minisky.plugin.timedfunction import TimedFunctionManager, Timer

__all__ = (
    "Entity",
    "Plugin",
    "PluginContext",
    "PluginError",
    "PluginManager",
    "PluginSpec",
    "TimedFunctionManager",
    "Timer",
    "command",
)
