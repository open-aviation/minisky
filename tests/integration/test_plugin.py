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
from minisky.simulation import Simulation
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


def test_typed_declaration_builds_validated_runtime_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Config(BaseModel):
        value: int

    class State:
        def __init__(self, value: int, random: object) -> None:
            self.value = value
            self.random = random

    def build(context: plugin_api.PluginContext[Config]) -> plugin_api.PluginSpec:
        context.mount(State(context.config.value, context.python_random))
        return context.finish()

    entry = FakeEntryPoint("typed", plugin_api.Plugin(build=build, config_class=Config))
    module = importlib.import_module("minisky.plugin.plugin")
    monkeypatch.setattr(module.metadata, "entry_points", lambda *, group: (entry,))

    runtime = MiniSky(MiniSkySettings(plugins={"typed": {"value": 7}}))
    try:
        ok, message = runtime.plugins.load("TYPED")
        assert ok, message
        state = runtime.variables.varlist["typed"][0]
        assert isinstance(state, State)
        assert state.value == 7
        assert state.random is runtime.python_random
        assert runtime.plugins.plugins["TYPED"].spec is not None
    finally:
        runtime.close()
