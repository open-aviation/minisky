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
from pydantic import BaseModel

from minisky import MiniSky, MiniSkySettings
from minisky import plugin as plugin_api
from minisky.core.trafficarrays import PreparedReplacement
from minisky.plugin.plugin import _Hook
from minisky.simulation import Simulation
from minisky.stack import Command, PreparedCommand
from minisky.traffic import Traffic
from minisky.traffic.autopilot import Autopilot


@pytest.fixture
def anyio_backend() -> str:
    return "asyncio"


class TestDiscovery:
    def test_discovers_installed_plugins(self, runtime: MiniSky) -> None:
        runtime.plugins.discover()
        assert {"CUSTOMAUTOPILOT", "EXAMPLE", "TANGRAM"} <= runtime.plugins.plugins.keys()

    def test_discovery_does_not_import(self, runtime: MiniSky) -> None:
        record = runtime.plugins.plugins["EXAMPLE"]
        assert not record.loaded
        assert record.spec is None

    def test_listing(self, runtime: MiniSky) -> None:
        ok, text = runtime.plugins.listing()
        assert ok
        assert "EXAMPLE" in text

    @pytest.mark.anyio
    async def test_unknown_plugin_load_fails(self, runtime: MiniSky) -> None:
        ok, message = await runtime.plugins.load("NOSUCHPLUGIN")
        assert not ok
        assert "not found" in message.lower()

    def test_discovery_emits_no_deprecation_warning(self, runtime: MiniSky) -> None:
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            runtime.plugins.discover()
        assert "EXAMPLE" in runtime.plugins.plugins


@dataclass
class FakeEntryPoint:
    name: str
    declaration: object
    module: str = "unused"

    def load(self) -> object:
        return self.declaration


def install(monkeypatch: pytest.MonkeyPatch, *entries: FakeEntryPoint) -> None:
    module = importlib.import_module("minisky.plugin.plugin")
    monkeypatch.setattr(module.metadata, "entry_points", lambda *, group: entries)


@dataclass(frozen=True, slots=True)
class CommandValue:
    callback: object
    name: str
    aliases: tuple[str, ...]
    brief: str
    help: str
    arguments: tuple[tuple[str, bool], ...]
    impl: str

    @classmethod
    def from_command(cls, command: Command) -> CommandValue:
        return cls(
            command.callback,
            command.name,
            command.aliases,
            command.brief,
            command.help,
            command.arguments,
            command.impl,
        )


@dataclass(frozen=True, slots=True)
class EntityValue:
    traffic: object | None
    prepared_traffic: object | None
    parent: object | None
    ownerless: bool
    retired: bool
    arrays: tuple[tuple[str, int], ...]

    @classmethod
    def from_entity(cls, entity: plugin_api.Entity) -> EntityValue:
        return cls(
            entity._traffic,
            entity._prepared_traffic,
            entity._parent,
            entity.ownerless,
            entity._retired,
            tuple((name, len(getattr(entity, name))) for name in entity._ArrVars),
        )


def run_command(runtime: MiniSky, command: str) -> str:
    runtime.commands.stack(command)
    runtime.simulation.step()
    return runtime.console.read_output_buffer()


@pytest.mark.anyio
async def test_example_commands_and_entity_are_runtime_owned() -> None:
    runtime = MiniSky(MiniSkySettings())
    try:
        ok, message = await runtime.plugins.load("EXAMPLE")
        assert ok, message
        record = runtime.plugins.plugins["EXAMPLE"]
        assert record.loaded
        assert tuple(runtime.plugins.loaded_plugins) == ("EXAMPLE",)

        run_command(runtime, "CRE KL001,A320,52,4,90,FL100,250")
        assert "150" in run_command(runtime, "PASSENGERS KL001 150")
        assert "150" in run_command(runtime, "PASSENGERS KL001")

        again = await runtime.plugins.load("EXAMPLE")
        assert again == (False, "Plugin EXAMPLE already loaded")
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_example_entity_sizes_existing_traffic_and_retires() -> None:
    runtime = MiniSky(MiniSkySettings())
    runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
    ok, message = await runtime.plugins.load("EXAMPLE")
    assert ok, message
    record = runtime.plugins.plugins["EXAMPLE"]
    entity = record.entities[0]
    assert record.entities == (entity,)
    assert record.spec == plugin_api.PluginSpec((entity,), entity)
    assert EntityValue.from_entity(entity) == EntityValue(
        runtime.traffic,
        None,
        runtime.traffic,
        False,
        False,
        (("npassengers", 1),),
    )

    await runtime.aclose()
    assert EntityValue.from_entity(entity) == EntityValue(
        None,
        None,
        None,
        False,
        True,
        (("npassengers", 1),),
    )


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
    runtime = MiniSky(MiniSkySettings(plugins={"typed": {"value": 7}}))
    try:
        ok, message = await runtime.plugins.load("TYPED")
        assert ok, message
        state = State(7, runtime.python_random)
        assert runtime.variables.varlist["typed"] == (state, ["value", "random"])
        assert runtime.plugins.plugins["TYPED"].spec == plugin_api.PluginSpec((state,), state)
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_mount_binds_command_to_exact_instance_and_infers_text(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Component:
        def __init__(self) -> None:
            self.values: list[int] = []

        @plugin_api.command(arguments="int")
        def record(self, value: int) -> tuple[bool, str]:
            """Record an integer."""
            self.values.append(value)
            return True, f"recorded {value}"

    component = Component()

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        context.mount(component)
        return context.finish()

    install(monkeypatch, FakeEntryPoint("mounted", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkySettings())
    try:
        assert "RECORD" not in runtime.commands.cmddict
        ok, message = await runtime.plugins.load("MOUNTED")
        assert ok, message
        command = runtime.commands.cmddict["RECORD"]
        assert runtime.plugins.plugins["MOUNTED"].commands == (
            PreparedCommand(command, ("RECORD",)),
        )
        assert CommandValue.from_command(command) == CommandValue(
            component.record,
            "RECORD",
            (),
            "RECORD value",
            "Record an integer.",
            (("int", False),),
            "Component",
        )

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
    runtime = MiniSky(MiniSkySettings())
    try:
        ok, message = await runtime.plugins.load("HOOKS")
        assert ok, message
        assert runtime.plugins.plugins["HOOKS"].hooks == (
            _Hook(component.pulse, "update", 2.0, "after", True),
            _Hook(component.pulse, "preupdate", 0.0, "before", True),
        )

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
    runtime = MiniSky(MiniSkySettings())
    try:
        ok, message = await runtime.plugins.load("HOOKS")
        assert ok, message
        runtime.plugins.update()
        runtime.plugins.update()
        assert calls == {"broken": 1, "healthy": 2}
        assert runtime.plugins.plugins["HOOKS"].hooks == (
            _Hook(component.update, "update", 0.0, "update", False, enabled=False),
            _Hook(component.healthy, "update", 0.0, "healthy", False),
        )
        assert tuple(runtime.plugins.loaded_plugins) == ("HOOKS",)
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_replacement_visibility_is_runtime_local_and_removed_on_shutdown() -> None:
    from minisky_example_customautopilot import CustomAutoPilot

    runtime_a = MiniSky(MiniSkySettings())
    runtime_b = MiniSky(MiniSkySettings())
    try:
        assert runtime_a.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT")[0] is False
        assert runtime_b.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT")[0] is False

        alt_callback = runtime_a.commands.cmddict["ALT"].callback
        ok, message = await runtime_a.plugins.load("CUSTOMAUTOPILOT")
        assert ok, message
        assert runtime_a.plugins.plugins["CUSTOMAUTOPILOT"].replacements == (
            PreparedReplacement(Autopilot, "CUSTOMAUTOPILOT", CustomAutoPilot),
        )
        assert runtime_a.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT")[0] is True
        assert type(runtime_a.traffic.ap) is CustomAutoPilot
        assert runtime_b.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT")[0] is False

        await runtime_a.plugins.aclose()
        assert type(runtime_a.traffic.ap) is Autopilot
        assert runtime_a.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT")[0] is False
        assert runtime_a.commands.cmddict["ALT"].callback is alt_callback
    finally:
        await runtime_a.aclose()
        await runtime_b.aclose()


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

        def selaltcmd(self, idx: int | np.ndarray, alt: float, vspd: float | None = None):
            self.alt_commands += 1
            return super().selaltcmd(idx, alt, vspd)

    def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
        return context.finish(replacements=(ArrayAutopilot,))

    install(monkeypatch, FakeEntryPoint("arrays", plugin_api.Plugin(build=build)))
    runtime = MiniSky(MiniSkySettings())
    try:
        runtime.traffic.cre("KL001", alt=3000.0, spd=150.0)
        ok, message = await runtime.plugins.load("ARRAYS")
        assert ok, message
        assert runtime.plugins.plugins["ARRAYS"].replacements == (
            PreparedReplacement(Autopilot, "ARRAYAUTOPILOT", ArrayAutopilot),
        )

        alt_callback = runtime.commands.cmddict["ALT"].callback
        assert runtime.replaceables.select("AUTOPILOT", "ARRAYAUTOPILOT")[0] is True
        selected = cast(ArrayAutopilot, runtime.traffic.ap)
        runtime.commands.stack("ALT KL001 FL100")
        runtime.simulation.step()
        assert type(selected) is ArrayAutopilot
        assert selected.plugin_value.tolist() == [0.0]
        assert selected.alt_commands == 1
        assert runtime.commands.cmddict["ALT"].callback is alt_callback
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
    runtime = MiniSky(MiniSkySettings())
    ok, message = await runtime.plugins.load("LIFECYCLE")
    assert ok, message
    assert events == [("enter", False)]
    assert "LIFECYCLE" in runtime.commands.cmddict

    await runtime.aclose()
    assert events == [("enter", False), ("exit", False)]
    assert capability is not None


@pytest.mark.anyio
async def test_concurrent_duplicate_loads_are_serialized() -> None:
    runtime = MiniSky(MiniSkySettings())
    try:
        results = await asyncio.gather(
            runtime.plugins.load("EXAMPLE"),
            runtime.plugins.load("EXAMPLE"),
        )
        assert sorted(results) == [
            (False, "Plugin EXAMPLE already loaded"),
            (True, "Successfully loaded plugin EXAMPLE"),
        ]
    finally:
        await runtime.aclose()


@pytest.mark.anyio
async def test_plugin_stack_load_uses_awaitable_command_boundary() -> None:
    runtime = MiniSky(MiniSkySettings())
    try:
        runtime.commands.stack("PLUGINS LOAD EXAMPLE")
        assert runtime.simulation.step() is False
        assert runtime.commands.command_pending
        await runtime.commands.wait_for_pending()
        assert runtime.simulation.step() is True
        assert runtime.plugins.plugins["EXAMPLE"].loaded
    finally:
        await runtime.aclose()
