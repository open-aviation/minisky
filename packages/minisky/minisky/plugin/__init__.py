"""Plugin system for MiniSky.

Plugins are installed Python packages that expose an `init_plugin(runtime)`
callable through the `minisky.plugins` entry-point group. Entry-point metadata
is discovered without importing plugin code and loaded on demand by the
runtime-owned [`PluginManager`][minisky.plugin.plugin.PluginManager]. Loading registers the
plugin's periodic update functions, lifecycle callbacks, variable-explorer
state, and stack commands with that runtime only.

This module provides the plugin infrastructure including:

- [`Entity`][minisky.plugin.entity.Entity]: Base class for plugin-owned
  per-aircraft data attached to a runtime's traffic tree.
- [`Plugin`][minisky.plugin.plugin.Plugin]: Discovery and loaded-state record
  for one plugin in one runtime.
- [`PluginManager`][minisky.plugin.plugin.PluginManager]: Runtime-owned plugin
  discovery, loading, and lifecycle management.
- [`TimedFunctionManager`][minisky.plugin.timedfunction.TimedFunctionManager]:
  Runtime-owned periodic callbacks and simulation lifecycle hooks.
- [`Timer`][minisky.plugin.timedfunction.Timer]: Simulation-time periodic
  trigger used by the timed-function manager.
- [`command`][minisky.plugin.plugin_decorators.command]: Decorator for declaring
  plugin stack commands without import-time registry mutation.
"""

from __future__ import annotations

from minisky.plugin.entity import Entity
from minisky.plugin.plugin import Plugin, PluginManager
from minisky.plugin.plugin_decorators import command
from minisky.plugin.timedfunction import TimedFunctionManager, Timer

__all__ = (
    "Entity",
    "Plugin",
    "PluginManager",
    "TimedFunctionManager",
    "Timer",
    "command",
)
