"""Runtime-owned declarations and loading for installed MiniSky plugins."""
# NOTE(abraham): entry points may export Plugin or the old init_plugin(runtime)
# callable. keep the compatibility path until the bundled plugins migrate.

from __future__ import annotations

import importlib
import inspect
import math
import traceback
from collections.abc import Callable, Iterable
from copy import deepcopy
from dataclasses import dataclass, field
from importlib import metadata
from random import Random
from types import MappingProxyType, ModuleType
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from pydantic import TypeAdapter

from minisky.core.trafficarrays import PreparedReplacement, TrafficArrays
from minisky.identifiers import validate_plugin_id
from minisky.plugin.entity import Entity
from minisky.plugin.plugin_decorators import (
    HookName,
    declared_commands,
    declared_hooks,
    declared_replacement,
    prepare_commands,
    prepare_declared_commands,
)
from minisky.plugin.timedfunction import TimedFunctionManager

if TYPE_CHECKING:
    from minisky.core.settings import MiniSkySettings
    from minisky.core.varexplorer import VariableExplorer
    from minisky.runtime import MiniSky
    from minisky.simulation import ConsoleIO, Simulation
    from minisky.stack import CommandStack, PreparedCommand


ConfigT = TypeVar("ConfigT")
ComponentT = TypeVar("ComponentT")


class PluginError(RuntimeError):
    """A plugin declaration or build operation failed."""


@dataclass(frozen=True, slots=True)
class PluginSpec:
    """Components and state built for aruntime."""

    components: tuple[object, ...]
    state: object | None = None
    replacements: tuple[type[TrafficArrays], ...] = ()


class PluginContext(Generic[ConfigT]):
    """Build fresh plugin components for aruntime."""

    def __init__(self, config: ConfigT, python_random: Random) -> None:
        self.config = config
        self.python_random = python_random
        self._components: list[object] = []
        self._state: object | None = None
        self._finished = False

    def mount(self, component: ComponentT, *, expose: bool = True) -> ComponentT:
        """Add a component and optionally expose it through variable lookup."""
        if self._finished:
            raise RuntimeError("plugin context has already been finished")
        if component is None:
            raise TypeError("plugin component must not be None")
        if any(existing is component for existing in self._components):
            raise PluginError("plugin component mounted more than once")
        if expose and self._state is not None:
            raise PluginError("a plugin may expose only a state component")
        self._components.append(component)
        if expose:
            self._state = component
        return component

    def finish(self, *, replacements: Iterable[type[TrafficArrays]] = ()) -> PluginSpec:
        """Finish this context and return its immutable specification."""
        if self._finished:
            raise RuntimeError("plugin context has already been finished")
        self._finished = True
        implementations = tuple(replacements)
        if not all(isinstance(implementation, type) for implementation in implementations):
            raise TypeError("plugin replacements must be classes")
        return PluginSpec(tuple(self._components), self._state, implementations)


PluginBuild = Callable[[PluginContext[Any]], PluginSpec]


def _empty_build(context: PluginContext[Any]) -> PluginSpec:
    return context.finish()


# TODO(abraham): preserve the config type relation through entry-point metadata.
@dataclass(frozen=True, slots=True)
class Plugin:
    """Declare a plugin build function and its optional configuration type."""

    build: PluginBuild = _empty_build
    config_class: type | None = None


@dataclass(frozen=True, slots=True)
class _PreparedHook:
    callback: Callable[..., Any]
    phase: HookName
    interval: float
    name: str


@dataclass
class _PluginRecord:
    """Entry-point metadata and loaded state for a runtime."""

    entry_point: metadata.EntryPoint
    plugin_name: str
    loaded: bool = False
    module: ModuleType | None = None
    config: dict[str, Any] = field(default_factory=dict)
    state: Any = None
    state_parent: object | None = None
    spec: PluginSpec | None = None
    commands: tuple[PreparedCommand, ...] = ()
    hooks: tuple[_PreparedHook, ...] = ()
    entities: tuple[Entity, ...] = ()
    replacements: tuple[PreparedReplacement, ...] = ()


class PluginManager:
    """Plugin discovery, loading, hooks, and state for a MiniSky runtime.

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
        self.plugins: dict[str, _PluginRecord] = {}
        self.loaded_plugins: dict[str, _PluginRecord] = {}
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

        Plugin packages register a declaration in
        the `minisky.plugins` entry-point group. The entry-point name is the
        plugin ID used by `PLUGINS LOAD` and `enabled_plugins`.
        """
        # or compatibility callable
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
            self.plugins[plugin_name] = _PluginRecord(entry_point, plugin_name)

        for plugin_name in duplicates:
            existing = self.plugins.get(plugin_name)
            if existing is None or not existing.loaded:
                self.plugins.pop(plugin_name, None)

    def load(self, name: str) -> tuple[bool, str]:
        """Load a discovered plugin by name."""
        plugin = self.plugins.get(name.upper())
        if plugin is None:
            return False, f"Error loading plugin: plugin {name} not found."
        if plugin.loaded:
            return False, f"Plugin {plugin.plugin_name} already loaded"

        entities: tuple[Entity, ...] = ()
        replacements: tuple[PreparedReplacement, ...] = ()
        try:
            loaded = plugin.entry_point.load()
            if isinstance(loaded, Plugin):
                key = plugin.plugin_name.lower()
                spec = self._build(key, loaded)
                commands, hooks, replacements, entities = self._prepare_typed(key, spec)
                module = None
                config: dict[str, Any] = {}
                state = spec.state
                state_parent = state
            else:
                module, config, state, state_parent, commands = self._load_legacy(
                    plugin.plugin_name, loaded, plugin.entry_point.module
                )
                spec = None
                hooks = ()

            state_name = plugin.plugin_name.lower()
            if state_parent is not None:
                self.variables.validate_data_parent(state_name)

            if spec is None:
                self._register_legacy_hooks(plugin.plugin_name, config)
            else:
                for entity in entities:
                    entity._publish()
                self._register_typed_hooks(hooks)

            self.commands.install_commands(commands)
            if state_parent is not None:
                self.variables.register_data_parent(state_parent, state_name)
            self.runtime.replaceables.install(replacements)

            plugin.loaded = True
            plugin.module = module
            plugin.config = config
            plugin.state = state
            plugin.state_parent = state_parent
            plugin.spec = spec
            plugin.commands = commands
            plugin.hooks = hooks
            plugin.entities = entities
            plugin.replacements = replacements
            self.loaded_plugins[plugin.plugin_name] = plugin
            return True, f"Successfully loaded plugin {plugin.plugin_name}"

        except ImportError as exc:
            for entity in reversed(entities):
                entity._abort()
            return False, f"Failed to load {plugin.plugin_name}: {exc}"
        except Exception as exc:
            for entity in reversed(entities):
                entity._abort()
            traceback.print_exc()
            return False, f"Error loading {plugin.plugin_name}: {exc}"

    def _build(self, key: str, declaration: Plugin) -> PluginSpec:
        raw = deepcopy(self.settings.plugins.get(key, {}))
        if declaration.config_class is None:
            if raw:
                raise PluginError(f"plugin {key} does not accept configuration")
            config: object = MappingProxyType({})
        else:
            try:
                config = TypeAdapter(declaration.config_class).validate_python(raw)
            except Exception as exc:
                raise PluginError(f"plugin {key} configuration is invalid: {exc}") from exc

        context = PluginContext(config, self.runtime.python_random)
        spec = declaration.build(context)
        if not isinstance(spec, PluginSpec):
            raise PluginError(f"plugin {key} build did not return PluginSpec")
        return spec

    def _prepare_typed(
        self, key: str, spec: PluginSpec
    ) -> tuple[
        tuple[PreparedCommand, ...],
        tuple[_PreparedHook, ...],
        tuple[PreparedReplacement, ...],
        tuple[Entity, ...],
    ]:
        commands: list[PreparedCommand] = []
        command_names: set[str] = set()
        for component in spec.components:
            try:
                for bound in declared_commands(component):
                    prepared = self.commands.prepare_command(
                        bound.callback,
                        name=bound.name,
                        aliases=bound.aliases,
                        arguments=bound.declaration.arguments,
                        brief=bound.brief,
                        help=bound.help,
                    )
                    overlap = command_names.intersection(prepared.names)
                    if overlap:
                        raise PluginError(f"plugin {key} repeats command name: {min(overlap)}")
                    command_names.update(prepared.names)
                    commands.append(prepared)
            except (TypeError, ValueError) as exc:
                raise PluginError(str(exc)) from exc

        hooks: list[_PreparedHook] = []
        hook_names: set[str] = set()
        for component in spec.components:
            try:
                for bound in declared_hooks(component):
                    prepared = self._prepare_hook(
                        key,
                        bound.callback,
                        bound.hook,
                        bound.declaration.interval,
                        bound.name,
                    )
                    if prepared.name in hook_names:
                        raise PluginError(f"plugin {key} repeats hook name: {bound.name}")
                    hook_names.add(prepared.name)
                    hooks.append(prepared)
            except (TypeError, ValueError) as exc:
                raise PluginError(str(exc)) from exc

        command_tuple = tuple(commands)
        self.commands.validate_commands(command_tuple)

        replacements: list[PreparedReplacement] = []
        for implementation in spec.replacements:
            declaration = declared_replacement(implementation)
            try:
                replacements.append(
                    self.runtime.replaceables.prepare(
                        implementation,
                        base=declaration.base,
                        name=declaration.name,
                    )
                )
            except (TypeError, ValueError) as exc:
                raise PluginError(str(exc)) from exc
        replacement_tuple = tuple(replacements)
        self.runtime.replaceables.validate(replacement_tuple)

        entities = tuple(
            component for component in spec.components if isinstance(component, Entity)
        )
        prepared_entities: list[Entity] = []
        try:
            for entity in entities:
                entity._prepare(self.runtime.traffic)
                prepared_entities.append(entity)
        except BaseException:
            for entity in reversed(prepared_entities):
                entity._abort()
            raise

        return command_tuple, tuple(hooks), replacement_tuple, entities

    @staticmethod
    def _prepare_hook(
        key: str,
        callback: Callable[..., Any],
        phase: HookName,
        interval: float,
        name: str,
    ) -> _PreparedHook:
        if inspect.iscoroutinefunction(callback):
            raise PluginError(f"plugin {key} hook {name} must be synchronous")
        if not math.isfinite(interval) or interval < 0:
            raise PluginError(f"plugin {key} hook {name} has invalid interval")
        if phase in ("reset", "hold") and interval:
            raise PluginError(f"plugin {key} gives interval to non-periodic {phase} hook")

        signature = inspect.signature(callback)
        accepts_dt = phase in ("preupdate", "update") and "dt" in signature.parameters
        try:
            signature.bind(dt=0.0) if accepts_dt else signature.bind()
        except TypeError as exc:
            raise PluginError(f"plugin {key} hook {name} has incompatible signature") from exc

        # NOTE(abraham): TimedFunctionManager keys hooks by name. include the
        # phase until typed hooks own their own runtime-local scheduler.
        return _PreparedHook(callback, phase, interval, f"{key}.{phase}.{name}")

    def _register_typed_hooks(self, hooks: tuple[_PreparedHook, ...]) -> None:
        for prepared in hooks:
            self.timed.register(
                prepared.callback,
                name=prepared.name,
                dt=prepared.interval,
                hook=prepared.phase,
            )

    def _load_legacy(
        self, plugin_name: str, loaded: object, module_name: str
    ) -> tuple[
        ModuleType,
        dict[str, Any],
        object | None,
        object,
        tuple[PreparedCommand, ...],
    ]:
        """Run the old initializer and prepare its MiniSky-owned registrations."""
        if not callable(loaded):
            raise PluginError(f"Plugin {plugin_name} entry point is not callable")
        initializer = cast("Callable[[MiniSky], Any]", loaded)
        module = importlib.import_module(module_name)
        result = initializer(self.runtime)
        config = result if isinstance(result, dict) else result[0]
        stack_functions = (
            result[1] if isinstance(result, (tuple, list)) and len(result) > 1 else None
        )
        if not isinstance(config, dict):
            raise PluginError(f"Plugin {plugin_name} returned an invalid config")

        # NOTE(abraham): init_plugin already ran, so this is not rollback.
        # we only make our command and variable registry writes predictable.
        prepared_commands = list(prepare_declared_commands(self.commands, module))
        if stack_functions:
            prepared_commands.extend(prepare_commands(self.commands, stack_functions))
        commands = tuple(prepared_commands)
        self.commands.validate_commands(commands)

        state = config.get("state")  # ew
        state_parent = state if state is not None else module
        return module, config, state, state_parent, commands

    def _register_legacy_hooks(self, plugin_name: str, config: dict[str, Any]) -> None:
        """Keep old hook registration separate from typed declarations."""
        # TODO(abraham): delete this with init_plugin compatibility.
        interval = max(float(config.get("update_interval", 0.0)), self.simulation.simdt)
        for hook_name in ("preupdate", "update", "reset", "hold"):
            callback = config.get(hook_name)
            if callback is not None:
                self.timed.register(
                    callback,
                    name=f"{plugin_name}.{callback.__name__}",
                    dt=interval,
                    hook=hook_name,
                )

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
        # TODO(abraham): own plugin tasks and handles through a async exit stack.
        for plugin in reversed(tuple(self.loaded_plugins.values())):
            try:
                callback = plugin.config.get("shutdown")
                if callback is not None:
                    callback()
            except Exception as exc:  # noqa: BLE001 - finish releasing every plugin
                errors.append(exc)
            finally:
                self.commands.remove_commands(plugin.commands)

                if plugin.state_parent is not None:
                    self.variables.unregister_data_parent(
                        plugin.plugin_name.lower(), expected=plugin.state_parent
                    )
                if plugin.spec is None and isinstance(plugin.state, TrafficArrays):
                    plugin.state.detach()
                self.runtime.replaceables.remove(plugin.replacements)
                for entity in reversed(plugin.entities):
                    entity._retire()

                plugin.loaded = False
                plugin.module = None
                plugin.config.clear()
                plugin.state = None
                plugin.state_parent = None
                plugin.spec = None
                plugin.commands = ()
                plugin.hooks = ()
                plugin.entities = ()
                plugin.replacements = ()

        self.timed.clear()
        self.loaded_plugins.clear()
        if errors:
            raise ExceptionGroup("Plugin shutdown failed", errors)
