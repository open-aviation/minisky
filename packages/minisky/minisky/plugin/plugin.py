"""Runtime-owned discovery and loading for installed MiniSky plugins."""
# NOTE(abraham): discovery reads `minisky.plugins` entry point metadata
# without importing plugin code. loading currently uses the synchronous
# init_plugin(runtime), TODO replace the return protocol

from __future__ import annotations

import importlib
import traceback
from collections.abc import Callable
from dataclasses import dataclass, field
from importlib import metadata
from types import ModuleType
from typing import TYPE_CHECKING, Any, cast

from minisky.core.trafficarrays import TrafficArrays
from minisky.identifiers import validate_plugin_id
from minisky.plugin.plugin_decorators import append_commands, register_declared_commands
from minisky.plugin.timedfunction import TimedFunctionManager

if TYPE_CHECKING:
    from minisky.core.settings import MiniSkySettings
    from minisky.core.varexplorer import VariableExplorer
    from minisky.runtime import MiniSky
    from minisky.simulation import ConsoleIO, Simulation
    from minisky.stack import CommandStack


# TODO(abraham): replace this compatibility record with separate discovered and
# active plugin types when `init_plugin` is replaced by a typed declaration.
@dataclass
class Plugin:
    """Entry-point metadata and loaded state for a plugin in a runtime."""

    entry_point: metadata.EntryPoint
    plugin_name: str
    loaded: bool = False
    module: ModuleType | None = None
    config: dict[str, Any] = field(default_factory=dict)
    state: Any = None
    command_names: set[str] = field(default_factory=set)


class PluginManager:
    """Plugin discovery, loading, hooks, and state for one MiniSky runtime.

    Attributes:
        plugins: Dict mapping upper-case plugin names to all discovered plugin
            records for this runtime.
        loaded_plugins: Dict containing the plugins loaded into this runtime.
        timed: Runtime-owned timer and lifecycle-hook manager.
    """

    def __init__(
        self,
        settings: MiniSkySettings,
        console: ConsoleIO,
        variables: VariableExplorer,
        get_runtime: Callable[[], MiniSky],
        get_simulation: Callable[[], Simulation],
        get_command_stack: Callable[[], CommandStack],
    ) -> None:
        self.settings = settings
        self.console = console
        self.variables = variables
        self._get_runtime = get_runtime
        self._get_simulation = get_simulation
        self._get_command_stack = get_command_stack
        self.plugins: dict[str, Plugin] = {}
        self.loaded_plugins: dict[str, Plugin] = {}
        self.timed = TimedFunctionManager(lambda: self.simulation.simdt)

    @property
    def runtime(self) -> MiniSky:
        """Return the runtime that owns this manager."""
        return self._get_runtime()

    @property
    def simulation(self) -> Simulation:
        """Return the owning runtime's simulation."""
        return self._get_simulation()

    @property
    def commands(self) -> CommandStack:
        """Return the owning runtime's command stack."""
        return self._get_command_stack()

    def discover(self) -> None:
        """Discover installed plugins without importing their modules.

        Plugin packages register an `init_plugin` callable in the
        `minisky.plugins` entry-point group. The entry-point name is the plugin
        ID used by `PLUGINS LOAD` and `enabled_plugins`.
        """
        entries: dict[str, metadata.EntryPoint] = {}
        duplicates: set[str] = set()
        for entry_point in metadata.entry_points(group="minisky.plugins"):
            try:
                plugin_id = validate_plugin_id(entry_point.name)
            except ValueError as exc:
                self.console.echo(f"Ignoring plugin entry point {entry_point.name!r}: {exc}")
                continue

            plugin_name = plugin_id.upper()
            if plugin_name in duplicates:
                continue
            if plugin_name in entries:
                entries.pop(plugin_name)
                duplicates.add(plugin_name)
                self.console.echo(f"Ignoring duplicate plugin entry point: {plugin_id}")
                continue
            entries[plugin_name] = entry_point

        for plugin_name, entry_point in entries.items():
            existing = self.plugins.get(plugin_name)
            if existing is not None and existing.loaded:
                continue
            self.plugins[plugin_name] = Plugin(entry_point, plugin_name)

        for plugin_name in duplicates:
            existing = self.plugins.get(plugin_name)
            if existing is None or not existing.loaded:
                self.plugins.pop(plugin_name, None)

    def load(self, name: str) -> tuple[bool, str]:
        """Load a previously discovered plugin by name.

        Imports the module, calls `init_plugin(runtime)`, registers returned
        `preupdate`, `update`, `reset`, and `hold` hooks as timed functions,
        registers plugin data with the variable explorer, and appends declared
        stack commands to the owning runtime's command stack.

        Args:
            name: Plugin name, case-insensitive, as found during discovery.

        Returns:
            Tuple of `(success flag, status message)`. Loading fails when the
            plugin is unknown, already loaded, returns an invalid config, or
            raises during import or initialization.
        """
        plugin = self.plugins.get(name.upper())
        if plugin is None:
            return False, f"Error loading plugin: plugin {name} not found."
        if plugin.loaded:
            return False, f"Plugin {plugin.plugin_name} already loaded"

        # TODO(abraham): build and validate a registration plan before mutating
        # MiniSky-owned registries. legacy initialiser may already have external
        # side effects, so do not claim to provide general rollback.
        try:
            # Load and initialize the plugin for this runtime.
            # TODO(abraham): replace the callable and tuple/dict protocol with a
            # typed declaration after entry-point discovery has settled.
            loaded = plugin.entry_point.load()
            if not callable(loaded):
                return False, f"Plugin {plugin.plugin_name} entry point is not callable"
            initializer = cast("Callable[[MiniSky], Any]", loaded)
            module = importlib.import_module(plugin.entry_point.module)
            result = initializer(self.runtime)
            # TODO(abraham): replace dict and tuple returns with a typed plugin plan
            config = result if isinstance(result, dict) else result[0]
            stack_functions = (
                result[1] if isinstance(result, (tuple, list)) and len(result) > 1 else None
            )
            if not isinstance(config, dict):
                return False, f"Plugin {plugin.plugin_name} returned an invalid config"

            # Get update interval (minimum is simdt) and register hooks.
            interval = max(float(config.get("update_interval", 0.0)), self.simulation.simdt)
            for hook_name in ("preupdate", "update", "reset", "hold"):
                callback = config.get(hook_name)
                if callback is not None:
                    self.timed.register(
                        callback,
                        name=f"{plugin.plugin_name}.{callback.__name__}",
                        dt=interval,
                        hook=hook_name,
                    )

            # Register stack functions only on this runtime.
            command_names_before = set(self.commands.cmddict)
            register_declared_commands(self.commands, module)
            if stack_functions:
                append_commands(self.commands, stack_functions)
            command_names = set(self.commands.cmddict) - command_names_before

            # Register plugin state, or the module when no state object is returned.
            state = config.get("state")
            self.variables.register_data_parent(
                state if state is not None else module,
                plugin.plugin_name.lower(),
            )

            plugin.loaded = True
            plugin.module = module
            plugin.config = config
            plugin.state = state
            plugin.command_names = command_names
            self.loaded_plugins[plugin.plugin_name] = plugin
            return True, f"Successfully loaded plugin {plugin.plugin_name}"

        except ImportError as exc:
            return False, f"Failed to load {plugin.plugin_name}: {exc}"
        except Exception as exc:
            traceback.print_exc()
            return False, f"Error loading {plugin.plugin_name}: {exc}"

    def load_enabled(self) -> None:
        """Load plugins enabled in this runtime's settings."""
        for plugin_name in self.settings.enabled_plugins:
            _, message = self.load(plugin_name)
            self.console.echo(message)

    def manage(self, command: str = "LIST", plugin_name: str = "") -> tuple[bool, str]:
        """List available plugins or load a plugin.

        Arguments:
        - command: `LIST` to show plugins, or `LOAD` / `ENABLE` to load one.
        - plugin_name: Name of the plugin to load.
        """
        command = command.upper()

        if command == "LIST":
            running = set(self.loaded_plugins)
            available = set(self.plugins) - running
            text = f"\nLoaded plugins: {', '.join(sorted(running)) if running else '(none)'}"
            if available:
                text += f"\nAvailable plugins: {', '.join(sorted(available))}"
            else:
                text += "\nNo additional plugins available."
            return True, text

        if command in ("LOAD", "ENABLE") or not plugin_name:
            # If no command is given, assume loading a plugin.
            return self.load(plugin_name or command)

        return False, f"Unknown command: {command}"

    def preupdate(self) -> None:
        """Called before traffic update each simulation step."""
        self.timed.preupdate()

    def update(self) -> None:
        """Called after traffic update each simulation step."""
        self.timed.update()

    def reset(self) -> None:
        """Called on simulation reset."""
        self.timed.reset()

    def hold(self) -> None:
        """Called when simulation pauses."""
        self.timed.hold()

    def shutdown(self) -> None:
        """Run shutdown callbacks and release all runtime-owned plugin state."""
        errors: list[Exception] = []
        # TODO(abraham): own plugin tasks and handles through one async exit stack.
        for plugin in reversed(tuple(self.loaded_plugins.values())):
            try:
                callback = plugin.config.get("shutdown")
                if callback is not None:
                    callback()
            except Exception as exc:  # noqa: BLE001 - finish releasing every plugin
                errors.append(exc)
            finally:
                for command_name in plugin.command_names:
                    self.commands.cmddict.pop(command_name, None)

                self.variables.unregister_data_parent(plugin.plugin_name.lower())
                if isinstance(plugin.state, TrafficArrays):
                    plugin.state.detach()

                plugin.loaded = False
                plugin.module = None
                plugin.config.clear()
                plugin.state = None
                plugin.command_names.clear()

        self.timed.clear()
        self.loaded_plugins.clear()
        if errors:
            raise ExceptionGroup("Plugin shutdown failed", errors)
