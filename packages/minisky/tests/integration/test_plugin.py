"""Integration tests for runtime-owned plugin loading and lifecycle."""

from __future__ import annotations

import asyncio
import importlib
import warnings
from collections.abc import AsyncGenerator, Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import cast

import numpy as np
import pytest
from minisky import Err, MiniSky, MiniSkyConfig, Ok, Result
from minisky import plugin as plugin_api
from minisky import quantities as q
from minisky.simulation import Simulation
from minisky.traffic import Traffic
from minisky.traffic.autopilot import Autopilot
from minisky.values import CasMps, StdPressureAltM
from pydantic import BaseModel


@pytest.fixture
def anyio_backend() -> str:
    return "asyncio"


class TestDiscovery:
    def test_discovery_does_not_import(self, monkeypatch: pytest.MonkeyPatch) -> None:
        class LazyEntryPoint:
            name = "lazy"

            def load(self) -> object:
                pytest.fail("discovery imported the plugin")

        module = importlib.import_module("minisky.plugin.plugin")
        monkeypatch.setattr(module.metadata, "entry_points", lambda *, group: (LazyEntryPoint(),))
        runtime = MiniSky(MiniSkyConfig())
        try:
            assert "LAZY" in runtime.plugins.plugins
        finally:
            runtime.close()

    def test_listing(self, runtime: MiniSky) -> None:
        result = runtime.plugins.listing()
        assert result.is_ok()
        assert "EXAMPLE" in result.unwrap()

    @pytest.mark.anyio
    async def test_unknown_plugin_load_fails(self, runtime: MiniSky) -> None:
        result = await runtime.plugins.load("NOSUCHPLUGIN")
        assert result.is_err()
        assert "not found" in result.unwrap_err().lower()

    def test_discovery_emits_no_deprecation_warning(self, runtime: MiniSky) -> None:
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            runtime.plugins.discover()
        assert "EXAMPLE" in runtime.plugins.plugins


@dataclass
class FakeEntryPoint:
    name: str
    declaration: object

    def load(self) -> object:
        return self.declaration


def install(monkeypatch: pytest.MonkeyPatch, *entries: FakeEntryPoint) -> None:
    module = importlib.import_module("minisky.plugin.plugin")
    monkeypatch.setattr(module.metadata, "entry_points", lambda *, group: entries)


@pytest.mark.anyio
async def test_entity_backfill_follows_lifespan_startup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    entered = asyncio.Event()
    release = asyncio.Event()

    class Callsigns(plugin_api.Entity):
        def __init__(self) -> None:
            super().__init__()
            with self.settrafarrays():
                self.names = np.array([], dtype=object)

        def create(self, n: int = 1) -> None:
            super().create(n)
            self.names[-n:] = self.traffic.callsign[-n:]

    entity = Callsigns()

    @asynccontextmanager
    async def lifespan(_runtime: plugin_api.PluginRuntime) -> AsyncGenerator[None]:
        entered.set()
        await release.wait()
        yield

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(entity)
        return context.finish(lifespan=lifespan)

    install(monkeypatch, FakeEntryPoint("callsigns", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    load_task = asyncio.create_task(runtime.plugins.load("CALLSIGNS"))
    try:
        await entered.wait()
        runtime.traffic.cre("KL001", alt=StdPressureAltM(3000.0), airspeed=CasMps(150.0))
        release.set()
        result = await load_task
        assert result.is_ok(), result.err()
        assert entity.names.tolist() == ["KL001"]
    finally:
        release.set()
        if not load_task.done():
            await load_task
        await runtime.aclose()


@pytest.mark.anyio
async def test_typed_declaration_builds_validated_runtime_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Config(BaseModel):
        value: int

    @dataclass(frozen=True)
    class State:
        value: int
        random: object

    def build(context: plugin_api.PluginContext[Config]) -> plugin_api.PluginSpec:
        context.mount(State(context.config.value, context.python_random))
        return context.finish()

    install(
        monkeypatch, FakeEntryPoint("typed", plugin_api.Plugin(build=build, config_class=Config))
    )
    runtime = MiniSky(MiniSkyConfig(plugins={"typed": {"value": 7}}))
    try:
        result = await runtime.plugins.load("TYPED")
        assert result.is_ok(), result.err()
        state = State(7, runtime.python_random)
        assert runtime.variables.varlist["typed"] == (state, ["value", "random"])
        assert runtime.plugins.plugins["TYPED"].spec == plugin_api.PluginSpec((state,), state)
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_mount_binds_command_to_exact_instance_and_respects_annotation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # plugin parameter names are arbitrary. a parameter named `idx` must keep
    # its declared integer parser not become a callsign.
    class Component:
        def __init__(self) -> None:
            self.values: list[int] = []

        @plugin_api.command(name="SAVE", aliases=("RECORD",))
        def record(self, idx: int) -> Result[str, str]:
            """Record an integer."""
            self.values.append(idx)
            return Ok(f"recorded {idx}")

    component = Component()

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(component)
        return context.finish()

    install(monkeypatch, FakeEntryPoint("mounted", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    try:
        assert "SAVE" not in runtime.commands.cmddict
        result = await runtime.plugins.load("MOUNTED")
        assert result.is_ok(), result.err()
        command = runtime.commands.cmddict["SAVE"]
        assert runtime.commands.cmddict["RECORD"] is command
        assert getattr(command.forms[0].callback, "__self__", None) is component
        assert command.name == "SAVE"
        assert command.aliases == ("RECORD",)
        assert len(command.forms) == 1
        assert command.forms[0].parameters[0].name == "idx"
        assert command.forms[0].help == "Record an integer."

        runtime.commands.stack("RECORD 7")
        runtime.simulation.step()
        assert component.values == [7]
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_multiple_hook_declarations_keep_independent_timing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[tuple[str, float]] = []

    class Component:
        @plugin_api.hook("preupdate", name="before")
        @plugin_api.hook("update", interval=2.0, name="after")
        def pulse(self, dt: float) -> None:
            events.append(("pulse", dt))

    component = Component()

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(component, expose=False)
        return context.finish()

    install(monkeypatch, FakeEntryPoint("hooks", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    try:
        result = await runtime.plugins.load("HOOKS")
        assert result.is_ok(), result.err()
        runtime.plugins.preupdate()
        runtime.plugins.update()
        runtime.plugins.preupdate()
        runtime.plugins.update()
        assert events == [("pulse", 1.0), ("pulse", 1.0), ("pulse", 2.0)]
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_failing_hook_is_disabled_without_disabling_plugin(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = {"broken": 0, "healthy": 0}

    class Component:
        @plugin_api.hook
        def update(self) -> None:
            calls["broken"] += 1
            raise RuntimeError("hook failed")

        @plugin_api.hook("update", name="healthy")
        def healthy(self) -> None:
            calls["healthy"] += 1

    component = Component()

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(component, expose=False)
        return context.finish()

    install(monkeypatch, FakeEntryPoint("hooks", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    try:
        result = await runtime.plugins.load("HOOKS")
        assert result.is_ok(), result.err()
        runtime.plugins.update()
        runtime.plugins.update()
        assert calls == {"broken": 1, "healthy": 2}
        assert tuple(runtime.plugins.loaded_plugins) == ("HOOKS",)
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_replacement_arrays_size_existing_traffic(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    @plugin_api.replacement
    class ArrayAutopilot(Autopilot):
        def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
            super().__init__(traffic, get_simulation)
            self.alt_commands = 0
            with self.settrafarrays():
                self.plugin_value = np.array([])

        def selaltcmd(
            self,
            idx: np.ndarray,
            alt: StdPressureAltM,
            vspd: q.VerticalRateMps[float] | None = None,
        ):
            self.alt_commands += 1
            return super().selaltcmd(idx, alt, vspd)

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        return context.finish(replacements=(ArrayAutopilot,))

    install(monkeypatch, FakeEntryPoint("arrays", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    try:
        runtime.traffic.cre("KL001", alt=StdPressureAltM(3000.0), airspeed=CasMps(150.0))
        result = await runtime.plugins.load("ARRAYS")
        assert result.is_ok(), result.err()
        alt_callback = runtime.commands.cmddict["ALT"].forms[0].callback
        assert runtime.replaceables.select("AUTOPILOT", "ARRAYAUTOPILOT").is_ok()
        selected = cast(ArrayAutopilot, runtime.traffic.ap)
        runtime.commands.stack("ALT KL001 FL100")
        runtime.simulation.step()
        assert type(selected) is ArrayAutopilot
        assert selected.plugin_value.tolist() == [0.0]
        assert selected.alt_commands == 1
        assert runtime.commands.cmddict["ALT"].forms[0].callback is alt_callback
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_lifespan_wraps_publication_and_runtime_is_revoked(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[tuple[str, bool]] = []
    capability: plugin_api.PluginRuntime | None = None

    class Component:
        @plugin_api.command(name="LIFECYCLE")
        def command(self) -> None:
            pass

    @asynccontextmanager
    async def lifespan(runtime_api: plugin_api.PluginRuntime) -> AsyncGenerator[None]:
        nonlocal capability
        capability = runtime_api
        events.append(("enter", "LIFECYCLE" in runtime.commands.cmddict))
        with pytest.raises(RuntimeError, match="not published"):
            runtime_api.stack_command("LIFECYCLE")
        try:
            yield
        finally:
            with pytest.raises(RuntimeError, match="revoked"):
                runtime_api.status()
            events.append(("exit", "LIFECYCLE" in runtime.commands.cmddict))

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(Component(), expose=False)
        return context.finish(lifespan=lifespan)

    install(monkeypatch, FakeEntryPoint("lifecycle", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    result = await runtime.plugins.load("LIFECYCLE")
    assert result.is_ok(), result.err()
    assert events == [("enter", False)]
    assert "LIFECYCLE" in runtime.commands.cmddict
    assert capability is not None
    capability.stack_command("LIFECYCLE")
    assert runtime.simulation.step()

    await runtime.aclose()
    assert events == [("enter", False), ("exit", False)]


@pytest.mark.anyio
async def test_shutdown_cancels_pending_command_before_lifespan_exit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []

    class Component:
        @plugin_api.command(name="BLOCK")
        async def block(self) -> None:
            events.append("command started")
            try:
                await asyncio.Event().wait()
            finally:
                events.append("command cancelled")

    @asynccontextmanager
    async def lifespan(_runtime: plugin_api.PluginRuntime) -> AsyncGenerator[None]:
        try:
            yield
        finally:
            events.append("lifespan exited")

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(Component(), expose=False)
        return context.finish(lifespan=lifespan)

    install(monkeypatch, FakeEntryPoint("blocked", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    result = await runtime.plugins.load("BLOCKED")
    assert result.is_ok(), result.err()
    runtime.commands.stack("BLOCK")
    assert not runtime.simulation.step()
    await asyncio.sleep(0)

    await runtime.aclose()

    assert events == ["command started", "command cancelled", "lifespan exited"]
    assert not runtime.commands.command_pending


@pytest.mark.anyio
async def test_failed_lifespan_startup_is_atomic(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    capability: plugin_api.PluginRuntime | None = None
    console_messages: list[str] = []

    class Component:
        @plugin_api.command(name="FAILEDSTART")
        def command(self) -> None:
            pass

    @asynccontextmanager
    async def lifespan(runtime_api: plugin_api.PluginRuntime) -> AsyncGenerator[None]:
        nonlocal capability
        capability = runtime_api
        runtime_api.subscribe_console(console_messages.append)
        raise RuntimeError("startup failed")
        yield

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(Component())
        return context.finish(lifespan=lifespan)

    install(monkeypatch, FakeEntryPoint("failedstart", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkyConfig())
    result = await runtime.plugins.load("FAILEDSTART")

    assert result.is_err()
    assert "startup failed" in result.unwrap_err()
    assert "FAILEDSTART" not in runtime.commands.cmddict
    assert "failedstart" not in runtime.variables.varlist
    assert not runtime.plugins.plugins["FAILEDSTART"].loaded
    assert "FAILEDSTART" not in runtime.plugins.loaded_plugins
    assert capability is not None
    with pytest.raises(RuntimeError, match="revoked"):
        capability.status()

    runtime.console.echo("after failed startup")
    assert console_messages == []
    await runtime.aclose()


@pytest.mark.anyio
async def test_load_configured_continues_after_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        return context.finish()

    install(
        monkeypatch,
        FakeEntryPoint("first", plugin_api.Plugin(build=build)),
        FakeEntryPoint("broken", object()),
        FakeEntryPoint("last", plugin_api.Plugin(build=build)),
    )
    runtime = MiniSky(MiniSkyConfig(plugins={"first": {}, "broken": {}, "last": {}}))
    try:
        loaded = await runtime.plugins.load_configured()
        assert loaded == ("FIRST", "LAST")
        assert tuple(runtime.plugins.loaded_plugins) == ("FIRST", "LAST")
        assert not runtime.plugins.plugins["BROKEN"].loaded
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_shutdown_is_reverse_order_and_aggregates_failures(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []

    def declaration(name: str) -> plugin_api.Plugin:
        @asynccontextmanager
        async def lifespan(runtime_api: plugin_api.PluginRuntime) -> AsyncGenerator[None]:
            events.append(f"enter {name}")
            try:
                yield
            finally:
                with pytest.raises(RuntimeError, match="revoked"):
                    runtime_api.status()
                events.append(f"exit {name}")
                raise RuntimeError(f"{name} shutdown failed")

        def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
            return context.finish(lifespan=lifespan)

        return plugin_api.Plugin(build=build)

    install(
        monkeypatch,
        FakeEntryPoint("first", declaration("first")),
        FakeEntryPoint("second", declaration("second")),
    )
    runtime = MiniSky(MiniSkyConfig())
    assert (await runtime.plugins.load("FIRST")).is_ok()
    assert (await runtime.plugins.load("SECOND")).is_ok()

    with pytest.raises(ExceptionGroup) as exc_info:
        await runtime.aclose()

    assert events == ["enter first", "enter second", "exit second", "exit first"]
    assert [str(error) for error in exc_info.value.exceptions] == [
        "second shutdown failed",
        "first shutdown failed",
    ]
    assert runtime.plugins.loaded_plugins == {}


@pytest.mark.anyio
async def test_concurrent_duplicate_loads_are_serialized() -> None:
    runtime = MiniSky(MiniSkyConfig())
    try:
        results = await asyncio.gather(
            runtime.plugins.load("EXAMPLE"),
            runtime.plugins.load("EXAMPLE"),
        )
        assert set(results) == {
            Err("Plugin EXAMPLE already loaded"),
            Ok("Successfully loaded plugin EXAMPLE"),
        }
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_plugin_stack_load_uses_awaitable_command_boundary() -> None:
    runtime = MiniSky(MiniSkyConfig())
    try:
        runtime.commands.stack("PLUGINS LOAD EXAMPLE")
        assert runtime.simulation.step() is False
        assert runtime.commands.command_pending
        await runtime.commands.wait_for_pending()
        assert runtime.simulation.step() is True
        assert runtime.plugins.plugins["EXAMPLE"].loaded
    finally:
        await runtime.aclose()
