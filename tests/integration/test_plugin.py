"""Integration tests for runtime-owned plugin discovery and loading."""

from __future__ import annotations

import importlib
import warnings
from dataclasses import dataclass
from typing import Any

import pytest
from pydantic import BaseModel

from minisky import MiniSky, MiniSkySettings
from minisky import plugin as plugin_api
from minisky.plugin.plugin import _PreparedHook
from minisky.simulation import Simulation
from minisky.stack import Command, PreparedCommand
from tests._types import RunCommand


class TestDiscovery:
    def test_discover_finds_example_plugins(self, runtime: MiniSky) -> None:
        runtime.plugins.discover()
        assert {"CUSTOMAUTOPILOT", "EXAMPLE", "TANGRAM"} <= runtime.plugins.plugins.keys()

    def test_discovery_does_not_import(self, runtime: MiniSky) -> None:
        plugin = runtime.plugins.plugins["EXAMPLE"]
        if not plugin.loaded:
            assert plugin.module is None

    def test_manage_plugins_list(self, runtime: MiniSky) -> None:
        ok, text = runtime.plugins.manage("LIST")
        assert ok
        assert "EXAMPLE" in text

    def test_unknown_plugin_load_fails(self, runtime: MiniSky) -> None:
        ok, message = runtime.plugins.load("NOSUCHPLUGIN")
        assert not ok
        assert "not found" in message.lower()

    def test_discovery_emits_no_deprecation_warning(self, runtime: MiniSky) -> None:
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            runtime.plugins.discover()
        assert "EXAMPLE" in runtime.plugins.plugins


@pytest.fixture
def loaded_example(runtime: MiniSky) -> Any:
    """Load the EXAMPLE plugin once into the session runtime."""
    plugin = runtime.plugins.plugins["EXAMPLE"]
    if not plugin.loaded:
        ok, message = runtime.plugins.load("EXAMPLE")
        assert ok, message
    return plugin


class TestLoading:
    def test_load_registers_plugin(self, runtime: MiniSky, loaded_example: Any) -> None:
        assert loaded_example.loaded
        assert "EXAMPLE" in runtime.plugins.loaded_plugins

    def test_double_load_rejected(self, runtime: MiniSky, loaded_example: Any) -> None:
        ok, message = runtime.plugins.load("EXAMPLE")
        assert not ok
        assert "already loaded" in message.lower()

    def test_plugin_stack_command_registered(
        self, sim: Simulation, loaded_example: Any, run_cmd: RunCommand
    ) -> None:
        run_cmd("CRE KL001,A320,52,4,90,FL100,250")
        output = run_cmd("PASSENGERS KL001 150")
        assert "150" in output

    def test_plugin_entity_tracks_aircraft(
        self, sim: Simulation, loaded_example: Any, run_cmd: RunCommand
    ) -> None:
        run_cmd("CRE KL001,A320,52,4,90,FL100,250")
        run_cmd("PASSENGERS KL001 42")
        output = run_cmd("PASSENGERS KL001")
        assert "42" in output


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


def test_example_entity_sizes_existing_traffic_and_retires() -> None:
    runtime = MiniSky(MiniSkySettings())
    runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
    try:
        ok, message = runtime.plugins.load("EXAMPLE")
        assert ok, message
        record = runtime.plugins.plugins["EXAMPLE"]
        entity = record.entities[0]
        assert (record.entities, record.spec, EntityValue.from_entity(entity)) == (
            (entity,),
            plugin_api.PluginSpec((entity,), entity),
            EntityValue(
                runtime.traffic,
                None,
                runtime.traffic,
                False,
                False,
                (("npassengers", 1),),
            ),
        )
    finally:
        runtime.close()

    assert EntityValue.from_entity(entity) == EntityValue(
        None,
        None,
        None,
        False,
        True,
        (("npassengers", 1),),
    )


def test_typed_declaration_builds_validated_runtime_state(
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

    entry = FakeEntryPoint("typed", plugin_api.Plugin(build=build, config_class=Config))
    install(monkeypatch, entry)

    runtime = MiniSky(MiniSkySettings(plugins={"typed": {"value": 7}}))
    try:
        ok, message = runtime.plugins.load("TYPED")
        assert ok, message
        state = State(7, runtime.python_random)
        assert runtime.variables.varlist["typed"] == (state, ["value", "random"])
        assert runtime.plugins.plugins["TYPED"].spec == plugin_api.PluginSpec((state,), state)
    finally:
        runtime.close()


def test_mount_binds_command_to_exact_instance_and_infers_text(
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
        ok, message = runtime.plugins.load("MOUNTED")
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
        runtime.close()


def test_multiple_hook_declarations_keep_independent_timing(
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
        ok, message = runtime.plugins.load("HOOKS")
        assert ok, message
        assert runtime.plugins.plugins["HOOKS"].hooks == (
            _PreparedHook(component.pulse, "update", 2.0, "hooks.update.after"),
            _PreparedHook(component.pulse, "preupdate", 0.0, "hooks.preupdate.before"),
        )

        runtime.plugins.preupdate()
        runtime.plugins.update()
        runtime.plugins.preupdate()
        runtime.plugins.update()
        assert events == [("pulse", 1.0), ("pulse", 1.0), ("pulse", 2.0)]
    finally:
        runtime.close()
