"""Runtime-owned declarations and loading for installed MiniSky plugins."""

from __future__ import annotations

import asyncio
import inspect
import math
import traceback
from collections.abc import AsyncGenerator, Awaitable, Callable, Iterable
from contextlib import AbstractAsyncContextManager, asynccontextmanager
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
from enum import Enum, auto
from importlib import metadata
from random import Random
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from pydantic import TypeAdapter

from minisky.core.trafficarrays import PreparedReplacement, TrafficArrays
from minisky.identifiers import validate_plugin_id
from minisky.plugin.entity import Entity
from minisky.plugin.plugin_decorators import HookName, declared_hooks, declared_replacement
from minisky.result import Err, Ok, Result
from minisky.streaming import Snapshot, build_snapshot

if TYPE_CHECKING:
    from minisky.core.config import MiniSkyConfig
    from minisky.core.varexplorer import VariableExplorer
    from minisky.runtime import MiniSky
    from minisky.simulation import ConsoleIO, Simulation
    from minisky.simulation.console import ConsoleSubscription
    from minisky.stack import CommandStack, PreparedCommand

ConfigT = TypeVar("ConfigT")
ComponentT = TypeVar("ComponentT")
CommandReply = Result[str, str]


class PluginError(RuntimeError):
    """A plugin declaration or lifecycle operation failed."""


@dataclass(frozen=True, slots=True)
class PluginStatus:
    """Read-only scalar runtime status."""

    simt: float
    simdt: float
    simutc: datetime
    speed: float
    ntraf: int
    state: int
    scenname: str


class _PluginRuntimeState(Enum):
    STARTING = auto()
    PUBLISHED = auto()
    REVOKED = auto()


class PluginRuntime:
    """Restricted runtime capabilities available during a plugin lifespan."""

    def __init__(
        self,
        *,
        status: Callable[[], PluginStatus],
        snapshot: Callable[[], Snapshot],
        echo: Callable[[str], None],
        subscribe_console: Callable[[Callable[[str], None]], ConsoleSubscription],
        stack_command: Callable[[str], None],
    ) -> None:
        self._status = status
        self._snapshot = snapshot
        self._echo = echo
        self._subscribe_console = subscribe_console
        self._stack_command = stack_command
        self._subscriptions: list[ConsoleSubscription] = []
        self._state = _PluginRuntimeState.STARTING

    def status(self) -> PluginStatus:
        self._raise_if_revoked()
        return self._status()

    def snapshot(self) -> Snapshot:
        self._raise_if_revoked()
        return self._snapshot()

    def echo(self, text: str) -> None:
        self._raise_if_revoked()
        self._echo(text)

    def stack_command(self, command: str) -> None:
        self._raise_if_revoked()
        if self._state is not _PluginRuntimeState.PUBLISHED:
            raise RuntimeError("plugin runtime is not published")
        self._stack_command(command)

    def subscribe_console(self, callback: Callable[[str], None]) -> ConsoleSubscription:
        self._raise_if_revoked()
        subscription = self._subscribe_console(callback)
        self._subscriptions.append(subscription)
        return subscription

    def _activate(self) -> None:
        self._raise_if_revoked()
        self._state = _PluginRuntimeState.PUBLISHED

    def _revoke(self) -> None:
        if self._state is _PluginRuntimeState.REVOKED:
            return
        self._state = _PluginRuntimeState.REVOKED
        for subscription in reversed(self._subscriptions):
            subscription.close()
        self._subscriptions.clear()

    def _raise_if_revoked(self) -> None:
        if self._state is _PluginRuntimeState.REVOKED:
            raise RuntimeError("plugin runtime is revoked")


PluginLifespan = Callable[[PluginRuntime], AbstractAsyncContextManager[None]]


@asynccontextmanager
async def _noop_lifespan(_runtime: PluginRuntime) -> AsyncGenerator[None]:
    yield


@dataclass(frozen=True, slots=True)
class PluginSpec:
    """Components and resources built for a runtime."""

    components: tuple[object, ...]
    state: object | None = None
    replacements: tuple[type[TrafficArrays], ...] = ()
    lifespan: PluginLifespan = _noop_lifespan


class PluginContext(Generic[ConfigT]):
    """Build fresh plugin components for a runtime."""

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

    def finish(
        self,
        *,
        replacements: Iterable[type[TrafficArrays]] = (),
        lifespan: PluginLifespan = _noop_lifespan,
    ) -> PluginSpec:
        """Finish this context and return its immutable specification."""
        if self._finished:
            raise RuntimeError("plugin context has already been finished")
        self._finished = True
        implementations = tuple(replacements)
        if not all(isinstance(implementation, type) for implementation in implementations):
            raise TypeError("plugin replacements must be classes")
        return PluginSpec(tuple(self._components), self._state, implementations, lifespan)


PluginBuild = Callable[[PluginContext[Any]], PluginSpec]


def _empty_build(context: PluginContext[Any]) -> PluginSpec:
    return context.finish()


# TODO(abraham): preserve the config type relation through entry-point metadata.
@dataclass(frozen=True, slots=True)
class Plugin:
    """Declare a plugin build function and its optional configuration type."""

    build: PluginBuild = _empty_build
    config_class: type | None = None


@dataclass(slots=True)
class _Hook:
    callback: Callable[..., Any]
    phase: HookName
    interval: float
    name: str
    accepts_dt: bool
    elapsed: float = 0.0
    enabled: bool = True

    def due(self, simdt: float) -> tuple[bool, float]:
        if self.interval <= 0:
            return True, simdt
        self.elapsed += simdt
        if self.elapsed + 1e-12 < self.interval:
            return False, 0.0
        elapsed, self.elapsed = self.elapsed, 0.0
        return True, elapsed


@dataclass(frozen=True, slots=True)
class _PreparedPlugin:
    spec: PluginSpec
    commands: tuple[PreparedCommand, ...]
    hooks: tuple[_Hook, ...]
    entities: tuple[Entity, ...]
    replacements: tuple[PreparedReplacement, ...]

    def abort(self) -> None:
        for entity in reversed(self.entities):
            entity._abort()


@dataclass
class _PluginRecord:
    """Entry-point metadata and active state for a runtime."""

    entry_point: metadata.EntryPoint
    plugin_name: str
    loaded: bool = False
    spec: PluginSpec | None = None
    commands: tuple[PreparedCommand, ...] = ()
    hooks: tuple[_Hook, ...] = ()
    entities: tuple[Entity, ...] = ()
    replacements: tuple[PreparedReplacement, ...] = ()
    lifespan: AbstractAsyncContextManager[None] | None = None
    runtime: PluginRuntime | None = None


class _ManagerState(Enum):
    OPEN = auto()
    CLOSING = auto()
    CLOSED = auto()


class PluginManager:
    """Discover, load, run, and close plugins for a runtime."""

    def __init__(
        self,
        config: MiniSkyConfig,
        console: ConsoleIO,
        variables: VariableExplorer,
        get_runtime: Callable[[], MiniSky],
        get_simulation: Callable[[], Simulation],
        get_command_stack: Callable[[], CommandStack],
    ) -> None:
        self.config = config
        self.console = console
        self.variables = variables
        self._get_runtime = get_runtime
        self._get_simulation = get_simulation
        self._get_command_stack = get_command_stack
        self.plugins: dict[str, _PluginRecord] = {}
        self.loaded_plugins: dict[str, _PluginRecord] = {}
        self._lock = asyncio.Lock()
        self._state = _ManagerState.OPEN

    @property
    def runtime(self) -> MiniSky:
        return self._get_runtime()

    @property
    def simulation(self) -> Simulation:
        return self._get_simulation()

    @property
    def commands(self) -> CommandStack:
        return self._get_command_stack()

    @property
    def requires_async_close(self) -> bool:
        return bool(self.loaded_plugins)

    def discover(self) -> None:
        """Discover installed plugin declarations without importing modules."""
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

    async def load(self, name: str) -> Result[str, str]:
        """Load a discovered plugin by name."""
        async with self._lock:
            if self._state is not _ManagerState.OPEN:
                return Err("Plugin manager is closed")
            plugin = self.plugins.get(name.upper())
            if plugin is None:
                return Err(f"Error loading plugin: plugin {name} not found.")
            if plugin.loaded:
                return Err(f"Plugin {plugin.plugin_name} already loaded")
            return await self._load(plugin)

    async def _load(self, plugin: _PluginRecord) -> Result[str, str]:
        prepared: _PreparedPlugin | None = None
        plugin_runtime: PluginRuntime | None = None
        lifespan: AbstractAsyncContextManager[None] | None = None
        entered = False
        try:
            declaration = plugin.entry_point.load()
            if not isinstance(declaration, Plugin):
                raise PluginError(
                    f"plugin {plugin.plugin_name.lower()} entry point must export Plugin"
                )
            key = plugin.plugin_name.lower()
            spec = self._build(key, declaration)
            prepared = self._prepare(key, spec)
            if spec.state is not None:
                self.variables.validate_data_parent(key)

            plugin_runtime = self._plugin_runtime()
            lifespan = spec.lifespan(plugin_runtime)
            await lifespan.__aenter__()
            entered = True
            for entity in prepared.entities:
                entity._prepare(self.runtime.traffic)
            plugin_runtime._activate()
            self._publish(key, prepared)

            plugin.loaded = True
            plugin.spec = spec
            plugin.commands = prepared.commands
            plugin.hooks = prepared.hooks
            plugin.entities = prepared.entities
            plugin.replacements = prepared.replacements
            plugin.lifespan = lifespan
            plugin.runtime = plugin_runtime
            self.loaded_plugins[plugin.plugin_name] = plugin
            return Ok(f"Successfully loaded plugin {plugin.plugin_name}")
        except BaseException as exc:
            if prepared is not None:
                prepared.abort()
            if plugin_runtime is not None:
                plugin_runtime._revoke()
            if entered and lifespan is not None:
                try:
                    await lifespan.__aexit__(type(exc), exc, exc.__traceback__)
                except BaseException as cleanup_error:  # ruff: ignore[BLE001] lifespan cleanup is arbitrary
                    traceback.print_exception(cleanup_error)
            if not isinstance(exc, Exception):
                raise
            traceback.print_exception(exc)
            return Err(f"Error loading {plugin.plugin_name}: {exc}")

    def _build(self, key: str, declaration: Plugin) -> PluginSpec:
        raw = deepcopy(self.config.plugins.get(key, {}))
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

    def _prepare(self, key: str, spec: PluginSpec) -> _PreparedPlugin:
        commands: list[PreparedCommand] = []
        for component in spec.components:
            try:
                commands.extend(self.commands.prepare_component(component))
            except (TypeError, ValueError) as exc:
                raise PluginError(str(exc)) from exc
        try:
            command_tuple = tuple(commands)
            self.commands.validate_commands(command_tuple)
        except ValueError as exc:
            raise PluginError(str(exc)) from exc

        hooks: list[_Hook] = []
        for component in spec.components:
            try:
                for bound in declared_hooks(component):
                    hooks.append(
                        self._prepare_hook(
                            key,
                            bound.callback,
                            bound.hook,
                            bound.declaration.interval,
                            bound.name,
                        )
                    )
            except (TypeError, ValueError) as exc:
                raise PluginError(str(exc)) from exc

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

        return _PreparedPlugin(
            spec,
            command_tuple,
            tuple(hooks),
            entities,
            replacement_tuple,
        )

    @staticmethod
    def _prepare_hook(
        key: str,
        callback: Callable[..., Any],
        phase: HookName,
        interval: float,
        name: str,
    ) -> _Hook:
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
        return _Hook(callback, phase, interval, name, accepts_dt)

    def _plugin_runtime(self) -> PluginRuntime:
        runtime = self.runtime
        return PluginRuntime(
            status=lambda: PluginStatus(
                float(runtime.simulation.simt),
                float(runtime.simulation.simdt),
                runtime.simulation.utc,
                float(runtime.runner.speed),
                int(runtime.traffic.ntraf),
                int(runtime.simulation.state),
                runtime.commands.get_scenname(),
            ),
            snapshot=lambda: build_snapshot(
                runtime.simulation,
                runtime.traffic,
                runtime.runner,
                runtime.commands,
            ),
            echo=self.console.echo,
            subscribe_console=self.console.subscribe,
            stack_command=self.commands.stack,
        )

    def _publish(self, key: str, prepared: _PreparedPlugin) -> None:
        try:
            for entity in prepared.entities:
                entity._publish()
            self.commands.install_commands(prepared.commands)
            if prepared.spec.state is not None:
                self.variables.register_data_parent(prepared.spec.state, key)
            self.runtime.replaceables.install(prepared.replacements)
        except BaseException:
            self.commands.remove_commands(prepared.commands)
            if prepared.spec.state is not None:
                self.variables.unregister_data_parent(key, expected=prepared.spec.state)
            self.runtime.replaceables.remove(prepared.replacements)
            raise

    async def load_configured(self) -> tuple[str, ...]:
        """Attempt every configured plugin and return those loaded successfully."""
        loaded: list[str] = []
        for plugin_name in self.config.plugins:
            match await self.load(plugin_name):
                case Ok(message):
                    self.console.echo(message)
                    loaded.append(plugin_name.upper())
                case Err(message):
                    self.console.echo(message)
        return tuple(loaded)

    def listing(self) -> Result[str, str]:
        running = set(self.loaded_plugins)
        available = set(self.plugins) - running
        text = f"\nLoaded plugins: {', '.join(sorted(running)) if running else '(none)'}"
        if available:
            text += f"\nAvailable plugins: {', '.join(sorted(available))}"
        else:
            text += "\nNo additional plugins available."
        return Ok(text)

    def manage(
        self, command: str = "LIST", plugin_name: str = ""
    ) -> CommandReply | Awaitable[CommandReply]:
        """List available plugins or load a plugin through the command stack."""
        operation = command.strip().upper()
        if operation in ("", "LIST"):
            return self.listing()
        if operation == "LOAD":
            if not plugin_name.strip():
                return Err("plugin name is required")
            return self.load(plugin_name)
        if not plugin_name:
            return self.load(command)
        return Err(f"Unknown command: {command}")

    def preupdate(self) -> None:
        self._run_hooks("preupdate")

    def update(self) -> None:
        self._run_hooks("update")

    def reset(self) -> None:
        for plugin in self.loaded_plugins.values():
            for hook in plugin.hooks:
                hook.elapsed = 0.0
        self._run_hooks("reset")

    def hold(self) -> None:
        self._run_hooks("hold")

    def _run_hooks(self, phase: HookName) -> None:
        if self._state is not _ManagerState.OPEN:
            return
        simdt = self.simulation.simdt
        for plugin in tuple(self.loaded_plugins.values()):
            for hook in plugin.hooks:
                if not hook.enabled or hook.phase != phase:
                    continue
                due, elapsed = hook.due(simdt)
                if not due:
                    continue
                try:
                    if hook.accepts_dt:
                        hook.callback(dt=elapsed)
                    else:
                        hook.callback()
                except Exception as exc:  # ruff: ignore[BLE001] plugin hooks are arbitrary
                    hook.enabled = False
                    traceback.print_exception(exc)
                    self.console.echo(
                        f"Plugin {plugin.plugin_name} disabled failing {phase} hook "
                        f"{hook.name}: {exc}"
                    )

    async def aclose(self) -> None:
        """Remove registrations and exit active lifespans in reverse order."""
        async with self._lock:
            if self._state is _ManagerState.CLOSED:
                return
            self._state = _ManagerState.CLOSING
            errors: list[Exception] = []
            for plugin in reversed(tuple(self.loaded_plugins.values())):
                if plugin.runtime is not None:
                    plugin.runtime._revoke()
                try:
                    self._remove(plugin)
                except Exception as exc:  # ruff: ignore[BLE001] aggregate removal failures
                    errors.append(exc)

                if plugin.lifespan is not None:
                    try:
                        await plugin.lifespan.__aexit__(None, None, None)
                    except Exception as exc:  # ruff: ignore[BLE001] plugin lifespan is arbitrary
                        errors.append(exc)
                self._clear(plugin)

            self.loaded_plugins.clear()
            self._state = _ManagerState.CLOSED
            if errors:
                raise ExceptionGroup("Plugin shutdown failed", errors)

    def _remove(self, plugin: _PluginRecord) -> None:
        errors: list[Exception] = []
        cleanups: list[Callable[[], None]] = [
            lambda: self.commands.remove_commands(plugin.commands),
        ]
        state = plugin.spec.state if plugin.spec is not None else None
        if state is not None:
            cleanups.append(
                lambda: self.variables.unregister_data_parent(
                    plugin.plugin_name.lower(), expected=state
                )
            )
        cleanups.append(lambda: self.runtime.replaceables.remove(plugin.replacements))
        cleanups.extend(entity._retire for entity in reversed(plugin.entities))
        for cleanup in cleanups:
            try:
                cleanup()
            except Exception as exc:  # ruff: ignore[BLE001] aggregate cleanup failures
                errors.append(exc)
        if errors:
            raise ExceptionGroup(f"Plugin {plugin.plugin_name} removal failed", errors)

    @staticmethod
    def _clear(plugin: _PluginRecord) -> None:
        plugin.loaded = False
        plugin.spec = None
        plugin.commands = ()
        plugin.hooks = ()
        plugin.entities = ()
        plugin.replacements = ()
        plugin.lifespan = None
        plugin.runtime = None

    def close(self) -> None:
        """Close a manager with no active plugin lifespans."""
        if self.loaded_plugins:
            raise RuntimeError("active plugins require async close")
        self._state = _ManagerState.CLOSED
