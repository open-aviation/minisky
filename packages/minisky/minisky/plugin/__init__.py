"""Public contracts for runtime-local MiniSky plugins.

Plugin packages can expose a [Plugin][minisky.plugin.Plugin] value through
the `minisky.plugins` entry-point group.
"""

from __future__ import annotations

from minisky.plugin.entity import Entity
from minisky.plugin.plugin import (
    Plugin,
    PluginContext,
    PluginError,
    PluginManager,
    PluginRuntime,
    PluginSpec,
    PluginStatus,
)
from minisky.plugin.plugin_decorators import HookName, command, hook, replacement

__all__ = (
    "Entity",
    "HookName",
    "Plugin",
    "PluginContext",
    "PluginError",
    "PluginManager",
    "PluginRuntime",
    "PluginSpec",
    "PluginStatus",
    "command",
    "hook",
    "replacement",
)
