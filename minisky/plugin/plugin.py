"""MiniSky plugin system.

Provides runtime-owned plugin discovery, loading, and management.

Discovery ([`PluginManager.discover`][]) scans the plugin directory and parses
each Python file's AST without importing it, looking for an `init_plugin`
function from which the plugin name, docstring, and stack commands are
extracted. Loading ([`PluginManager.load`][]) imports the module, calls
`init_plugin(runtime)`, and registers the returned update hooks and stack
commands with that runtime. [`PluginManager.manage`][] backs the in-simulator
`PLUGINS` stack command.
"""

from __future__ import annotations

import ast
import importlib
import sys
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING, Any

from minisky.plugin.plugin_decorators import append_commands, register_declared_commands
from minisky.plugin.timedfunction import TimedFunctionManager

if TYPE_CHECKING:
    from collections.abc import Callable

    from minisky.core.settings import MiniSkySettings
    from minisky.core.varexplorer import VariableExplorer
    from minisky.runtime import MiniSky
    from minisky.simulation import ConsoleIO, Simulation
    from minisky.stack import CommandStack


@dataclass
class Plugin:
    """Information about one plugin discovered for one runtime.

    Attributes:
        fullname: Importable module name of the plugin (dotted path).
        filepath: Path to the plugin's source file.
        plugin_doc: Module docstring, extracted during discovery.
        plugin_name: Name declared in the config dict, falling back to the
            upper-case file stem.
        plugin_stack: List of `(command name, help text)` tuples declared by
            the plugin.
        loaded: True once the plugin has been imported and initialized for the
            owning runtime.
        module: Imported plugin module, or None until loaded.
        config: Config dictionary returned by `init_plugin(runtime)`.
        state: Optional runtime-owned state object returned in the config.
    """

    fullname: str
    filepath: Path
    plugin_doc: str = ""
    plugin_name: str = ""
    plugin_stack: list[tuple[str, str]] = field(default_factory=list)
    loaded: bool = False
    module: ModuleType | None = None
    config: dict[str, Any] = field(default_factory=dict)
    state: Any = None


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
        """Discover plugins in the configured directory using AST parsing.

        Resolves `plugin_path` relative to the package root and then the current
        working directory, adds its parent to `sys.path`, and scans all `*.py`
        files except names starting with an underscore. Modules containing a
        top-level `init_plugin` function are registered without being imported.
        """
        # Get plugin path from settings.
        plugin_path = Path(self.settings.plugin_path)

        # Make the path absolute if it is relative.
        if not plugin_path.is_absolute():
            package_path = Path(__file__).parent.parent.parent / plugin_path
            working_path = Path.cwd() / plugin_path
            if package_path.exists():
                plugin_path = package_path
            elif working_path.exists():
                plugin_path = working_path
            else:
                self.console.echo(f"Plugin directory not found: {plugin_path}")
                return

        if not plugin_path.exists():
            self.console.echo(f"Plugin directory not found: {plugin_path}")
            return

        # TODO(abraham): replace this process-wide sys.path mutation with a
        # path-based importer that still supports plugin-local package imports.
        plugin_parent = str(plugin_path.parent)
        if plugin_parent not in sys.path:
            sys.path.insert(0, plugin_parent)

        # Scan Python files without importing them.
        for filepath in plugin_path.glob("**/*.py"):
            if filepath.name.startswith("_"):
                continue

            relative_path = filepath.relative_to(plugin_path.parent)
            fullname = ".".join(relative_path.with_suffix("").parts)
            try:
                tree = ast.parse(filepath.read_bytes())
            except Exception:
                continue

            # Find the required synchronous init_plugin function.
            init_node = next(
                (
                    item
                    for item in tree.body
                    if isinstance(item, ast.FunctionDef) and item.name == "init_plugin"
                ),
                None,
            )
            if init_node is None:
                continue

            plugin_info = self._parse_init_plugin(init_node)
            if plugin_info is None:
                continue

            plugin_name = str(
                plugin_info.get("plugin_name", filepath.stem.upper())
            ).upper()
            existing = self.plugins.get(plugin_name)
            if existing is not None and existing.loaded:
                continue

            self.plugins[plugin_name] = Plugin(
                fullname=fullname,
                filepath=filepath,
                plugin_doc=ast.get_docstring(tree) or "",
                plugin_name=plugin_name,
                plugin_stack=plugin_info.get("stack_functions", []),
            )

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

        try:
            # Load and initialize the plugin for this runtime.
            module = importlib.import_module(plugin.fullname)
            result = module.init_plugin(self.runtime)
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
            register_declared_commands(self.commands, module)
            if stack_functions:
                append_commands(self.commands, stack_functions)

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
        """Run shutdown callbacks and clear runtime-owned hook state."""
        # TODO(abraham): call this from the final `MiniSky` lifecycle/context
        # manager and unregister plugin variable-explorer parents at the same
        # time.
        for plugin in reversed(tuple(self.loaded_plugins.values())):
            callback = plugin.config.get("shutdown")
            if callback is not None:
                callback()
        self.timed.clear()
        self.loaded_plugins.clear()
        for plugin in self.plugins.values():
            plugin.loaded = False
            plugin.config.clear()
            plugin.state = None

    @staticmethod
    def _parse_init_plugin(func_node: ast.FunctionDef) -> dict[str, Any] | None:
        """Parse an `init_plugin` AST node to extract the plugin config.

        Walks the function body backwards from its return statement to find the
        returned config dict and optional stack-functions dict, reading literal
        keys and values without executing plugin code.

        Args:
            func_node: AST node of the `init_plugin` function.

        Returns:
            Dict with literal config values plus a `stack_functions` list of
            `(command name, help text)` tuples, or None if no return value can
            be found.
        """
        returned: list[ast.expr] = []
        return_names = ["", ""]

        for item in reversed(func_node.body):
            # Find return statement.
            if isinstance(item, ast.Return):
                if isinstance(item.value, ast.Tuple):
                    returned = list(item.value.elts)
                elif item.value is not None:
                    returned = [item.value]
                if returned:
                    return_names = [
                        value.id if isinstance(value, ast.Name) else "" for value in returned
                    ]

            # Resolve assignments of returned config dictionaries.
            if isinstance(item, ast.Assign) and isinstance(item.value, ast.Dict):
                target = item.targets[0]
                for index, name in enumerate(return_names):
                    if name and isinstance(target, ast.Name) and target.id == name:
                        returned[index] = item.value

        if not returned:
            return None

        # Parse the config dict.
        config: dict[str, Any] = {}
        if isinstance(returned[0], ast.Dict):
            for key, value in zip(returned[0].keys, returned[0].values, strict=False):
                if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                    config[str(key.value)] = value.value

        # Parse stack functions if present.
        if len(returned) > 1 and isinstance(returned[1], ast.Dict):
            stack_functions: list[tuple[str, str]] = []
            for key, value in zip(returned[1].keys, returned[1].values, strict=False):
                if not isinstance(key, ast.Constant):
                    continue
                help_text = ""
                if isinstance(value, (ast.List, ast.Tuple)) and value.elts:
                    last = value.elts[-1]
                    if isinstance(last, ast.Constant):
                        help_text = str(last.value)
                stack_functions.append((str(key.value), help_text))
            config["stack_functions"] = stack_functions

        return config
