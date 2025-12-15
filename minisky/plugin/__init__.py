"""Plugin system for MiniSky.

This module provides the plugin infrastructure including:
- Entity: Base class for singleton plugins with TrafficArrays
- Plugin: Plugin discovery and loading
- timed_function: Decorator for periodic update functions
- PluginManager: Central manager for plugin lifecycle events
- command: Decorator for registering stack commands
"""
from minisky.plugin.entity import Entity, Proxy, isproxied, getproxied
from minisky.plugin.timedfunction import timed_function, PluginManager, Timer, hooks
from minisky.plugin.plugin import Plugin, discover, load_enabled, manage_plugins
from minisky.plugin.plugin_decorators import command, append_commands
