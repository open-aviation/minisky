"""MiniSky plugin system.

Provides plugin discovery, loading, and management.

Discovery (:func:`discover`) scans the plugins directory and parses each
Python file's AST — without importing it — looking for an ``init_plugin()``
function, from which the plugin name, docstring and stack commands are
extracted. Loading (:meth:`Plugin.load`) then imports the module, calls its
``init_plugin()``, and registers the returned update hooks and stack
commands with the simulation. The :func:`manage_plugins` function backs the
in-simulator ``PLUGINS`` stack command.
"""

import ast
import importlib
import sys
from pathlib import Path
from typing import ClassVar

import minisky
from minisky.core import settings, varexplorer
from minisky.plugin import plugin_decorators
from minisky.plugin.timedfunction import timed_function


class Plugin:
    """MiniSky plugin class.

    Stores information about plugins found in the plugins directory. One
    instance is created per discovered plugin module; the class additionally
    keeps registries of all discovered and all loaded plugins.

    Attributes:
        plugins: Class-level dict mapping upper-case plugin name to its
            :class:`Plugin` instance, for all discovered plugins.
        loaded_plugins: Class-level dict of plugins that have been loaded.
        fullname: Importable module name of the plugin (dotted path).
        filepath: Path to the plugin's source file.
        plugin_doc: Module docstring of the plugin, extracted during discovery.
        plugin_name: Name of the plugin as declared in its config dict
            (falls back to the file stem in upper case).
        plugin_stack: List of (command name, help text) tuples of the stack
            commands the plugin declares.
        loaded: True once the plugin module has been imported and initialized.
        imp: The imported plugin module (None until loaded).
    """

    # Dictionary of all available plugins
    plugins: ClassVar[dict[str, "Plugin"]] = {}

    # Plugins that have been loaded
    loaded_plugins: ClassVar[dict[str, "Plugin"]] = {}

    def __init__(self, fullname: str, filepath: Path) -> None:
        """Create a plugin record for a discovered plugin module.

        Args:
            fullname: Importable (dotted) module name of the plugin.
            filepath: Path to the plugin's Python source file.
        """
        self.fullname = fullname
        self.filepath = filepath
        self.plugin_doc = ""
        self.plugin_name = ""
        self.plugin_stack = []
        self.loaded = False
        self.imp = None

    def _load(self) -> tuple[bool, str]:
        """Import and initialize this plugin.

        Imports the plugin module, calls its ``init_plugin()`` function, and
        registers the returned ``preupdate``/``update``/``reset`` hooks as
        timed functions (with the declared ``update_interval`` in seconds,
        never smaller than the simulation timestep), registers the module
        with the variable explorer, and appends any declared stack commands.

        Returns:
            Tuple of (success flag, status message).
        """
        if self.loaded:
            return False, f"Plugin {self.plugin_name} already loaded"

        try:
            # Load the plugin module
            self.imp = importlib.import_module(self.fullname)

            # Initialize the plugin
            result = self.imp.init_plugin()
            config = result if isinstance(result, dict) else result[0]

            # Get update interval (minimum is simdt)
            dt = max(config.get("update_interval", 0.0), minisky.sim.simdt)

            # Register timed functions if present
            for hook in ("preupdate", "update", "reset"):
                func = config.get(hook)
                if func:
                    timed_function(
                        func, name=f"{self.plugin_name}.{func.__name__}", dt=dt, hook=hook
                    )

            # Register with variable explorer
            varexplorer.register_data_parent(self.imp, self.plugin_name.lower())

            # Register stack functions if provided
            if isinstance(result, (tuple, list)) and len(result) > 1:
                stackfuns = result[1]
                plugin_decorators.append_commands(stackfuns)

            self.loaded = True
            return True, f"Successfully loaded plugin {self.plugin_name}"

        except ImportError as e:
            print(f"Plugin system failed to load {self.plugin_name}: {e}")
            return False, f"Failed to load {self.plugin_name}: {e}"
        except Exception as e:
            print(f"Plugin system error loading {self.plugin_name}: {e}")
            import traceback

            traceback.print_exc()
            return False, f"Error loading {self.plugin_name}: {e}"

    @classmethod
    def load(cls, name: str) -> tuple[bool, str]:
        """Load a previously discovered plugin by name.

        Args:
            name: Plugin name (case-insensitive) as found during discovery.

        Returns:
            Tuple of (success flag, status message). Fails when the plugin is
            unknown, already loaded, or raises during import/initialization.
        """
        plugin = cls.plugins.get(name.upper())
        if plugin is None:
            return False, f"Error loading plugin: plugin {name} not found."

        success, msg = plugin._load()
        if success:
            cls.loaded_plugins[name.upper()] = plugin
        return success, msg

    @classmethod
    def find_plugins(cls) -> None:
        """Discover plugins in the plugins directory using AST parsing.

        Resolves the plugin directory from the ``plugin_path`` setting
        (default ``plugins``, looked up relative to the package root and then
        the current working directory), adds it to ``sys.path``, and scans
        all ``*.py`` files (skipping names starting with an underscore). Any
        module whose AST contains a top-level ``init_plugin`` function is
        registered in :attr:`plugins` without being imported.
        """
        # Get plugin path from settings or use default
        plugin_path = Path(getattr(settings, "plugin_path", "plugins"))

        # Make path absolute if relative
        if not plugin_path.is_absolute():
            # Look relative to the minisky package first, then cwd
            pkg_path = Path(__file__).parent.parent.parent / plugin_path
            cwd_path = Path.cwd() / plugin_path

            if pkg_path.exists():
                plugin_path = pkg_path
            elif cwd_path.exists():
                plugin_path = cwd_path
            else:
                print(f"Plugin directory not found: {plugin_path}")
                return

        if not plugin_path.exists():
            print(f"Plugin directory not found: {plugin_path}")
            return

        # Add plugin path to sys.path for imports
        plugin_path_str = str(plugin_path.parent)
        if plugin_path_str not in sys.path:
            sys.path.insert(0, plugin_path_str)

        # Scan for Python files
        for filepath in plugin_path.glob("**/*.py"):
            if filepath.name.startswith("_"):
                continue

            # Construct module name
            rel_path = filepath.relative_to(plugin_path.parent)
            module_parts = list(rel_path.with_suffix("").parts)
            fullname = ".".join(module_parts)

            # Parse the source code using AST
            try:
                with open(filepath, "rb") as f:
                    source = f.read()
                tree = ast.parse(source)
            except Exception:
                continue

            # Look for init_plugin function
            for item in tree.body:
                if isinstance(item, ast.FunctionDef) and item.name == "init_plugin":
                    # Found a plugin, parse its config
                    plugin_info = cls._parse_init_plugin(item, tree)
                    if plugin_info:
                        plugin = Plugin(fullname, filepath)
                        plugin.plugin_doc = ast.get_docstring(tree) or ""
                        plugin.plugin_name = plugin_info.get("plugin_name", filepath.stem.upper())
                        plugin.plugin_stack = plugin_info.get("stack_functions", [])
                        cls.plugins[plugin.plugin_name.upper()] = plugin
                    break

    @classmethod
    def _parse_init_plugin(cls, func_node: ast.FunctionDef, tree: ast.Module) -> dict | None:
        """Parse an ``init_plugin`` AST node to extract the plugin config.

        Walks the function body backwards from its return statement to find
        the returned config dict (and optional stack-functions dict), reading
        literal keys and values without executing any plugin code.

        Args:
            func_node: ``ast.FunctionDef`` node of the ``init_plugin`` function.
            tree: Parsed AST of the full plugin module.

        Returns:
            Dict with the literal config values (e.g. ``plugin_name``,
            ``update_interval``) plus a ``stack_functions`` list of
            (command name, help text) tuples, or None if no return value
            could be found.
        """
        ret_dicts = []
        ret_names = ["", ""]

        for item in reversed(func_node.body):
            # Find return statement
            if isinstance(item, ast.Return):
                if isinstance(item.value, ast.Tuple):
                    ret_dicts = list(item.value.elts)
                elif item.value:
                    ret_dicts = [item.value]

                if not ret_dicts:
                    continue

                # Get variable names if return value is a Name
                ret_names = [el.id if isinstance(el, ast.Name) else "" for el in ret_dicts]

            # Check if this is assignment of a return value dict
            if isinstance(item, ast.Assign) and isinstance(item.value, ast.Dict):
                for i, name in enumerate(ret_names):
                    if name and hasattr(item.targets[0], "id") and item.targets[0].id == name:  # type: ignore[union-attr]
                        ret_dicts[i] = item.value

        if not ret_dicts:
            return None

        # Parse the config dict
        config = {}
        if isinstance(ret_dicts[0], ast.Dict):
            for key, value in zip(ret_dicts[0].keys, ret_dicts[0].values, strict=False):
                if hasattr(key, "s"):  # Python < 3.8 string constant
                    key_str = key.s  # type: ignore[union-attr]
                elif hasattr(key, "value"):  # Python 3.8+ Constant
                    key_str = key.value  # type: ignore[union-attr]
                else:
                    continue

                if hasattr(value, "s"):
                    config[key_str] = value.s  # type: ignore[attr-defined]
                elif hasattr(value, "value"):
                    config[key_str] = value.value  # type: ignore[attr-defined]

        # Parse stack functions if present
        if len(ret_dicts) > 1 and isinstance(ret_dicts[1], ast.Dict):
            stack_funcs = []
            for key, value in zip(ret_dicts[1].keys, ret_dicts[1].values, strict=False):
                if hasattr(key, "s"):
                    cmd_name = key.s  # type: ignore[union-attr]
                elif hasattr(key, "value"):
                    cmd_name = key.value  # type: ignore[union-attr]
                else:
                    continue

                # Extract help text (last element of the list/tuple)
                if isinstance(value, (ast.List, ast.Tuple)) and value.elts:
                    last = value.elts[-1]
                    if hasattr(last, "s"):
                        help_text = last.s  # type: ignore[attr-defined]
                    elif hasattr(last, "value"):
                        help_text = last.value  # type: ignore[attr-defined]
                    else:
                        help_text = ""
                    stack_funcs.append((cmd_name, help_text))
            config["stack_functions"] = stack_funcs

        return config


def discover() -> None:
    """Discover available plugins (AST parsing only, no imports).

    Convenience wrapper around :meth:`Plugin.find_plugins`; called once from
    :func:`minisky.init` so that the ``PLUGINS`` command can list what is
    available without importing anything.
    """
    Plugin.find_plugins()


def load_enabled() -> None:
    """Load enabled plugins from settings.

    Loads every plugin listed under ``enabled_plugins`` in the settings and
    prints the resulting status message for each. Called from
    :func:`minisky.load_plugins` after the simulator has been initialized.
    """
    enabled = getattr(settings, "enabled_plugins", [])
    for plugin_name in enabled:
        success, msg = Plugin.load(plugin_name)
        print(msg)


def manage_plugins(cmd: str = "LIST", plugin_name: str = "") -> tuple[bool, str]:
    """List available plugins or load/unload a plugin.

    Arguments:
    - cmd: 'LIST' to show plugins, 'LOAD' to load a plugin
    - plugin_name: Name of plugin to load
    """
    cmd = cmd.upper()

    if cmd == "LIST":
        running = set(Plugin.loaded_plugins.keys())
        available = set(Plugin.plugins.keys()) - running

        text = f"\nLoaded plugins: {', '.join(running) if running else '(none)'}"
        if available:
            text += f"\nAvailable plugins: {', '.join(available)}"
        else:
            text += "\nNo additional plugins available."
        return True, text

    if cmd in ("LOAD", "ENABLE") or not plugin_name:
        # If no command given, assume loading a plugin
        target = plugin_name or cmd
        return Plugin.load(target)

    return False, f"Unknown command: {cmd}"
