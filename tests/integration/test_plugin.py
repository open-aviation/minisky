"""Integration tests for runtime-owned plugin discovery and loading."""

from __future__ import annotations

import warnings

import pytest


class TestDiscovery:
    def test_discover_finds_example_plugins(self, runtime):
        runtime.plugins.discover()
        assert "EXAMPLE" in runtime.plugins.plugins

    def test_discovery_does_not_import(self, runtime):
        plugin = runtime.plugins.plugins["EXAMPLE"]
        if not plugin.loaded:
            assert plugin.module is None

    def test_manage_plugins_list(self, runtime):
        ok, text = runtime.plugins.manage("LIST")
        assert ok
        assert "EXAMPLE" in text

    def test_unknown_plugin_load_fails(self, runtime):
        ok, message = runtime.plugins.load("NOSUCHPLUGIN")
        assert not ok
        assert "not found" in message.lower()

    def test_discovery_emits_no_deprecation_warning(self, runtime):
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            runtime.plugins.discover()
        assert "EXAMPLE" in runtime.plugins.plugins


@pytest.fixture
def loaded_example(runtime):
    """Load the EXAMPLE plugin once into the session runtime."""
    plugin = runtime.plugins.plugins["EXAMPLE"]
    if not plugin.loaded:
        ok, message = runtime.plugins.load("EXAMPLE")
        assert ok, message
    return plugin


class TestLoading:
    def test_load_registers_plugin(self, runtime, loaded_example):
        assert loaded_example.loaded
        assert "EXAMPLE" in runtime.plugins.loaded_plugins

    def test_double_load_rejected(self, runtime, loaded_example):
        ok, message = runtime.plugins.load("EXAMPLE")
        assert not ok
        assert "already loaded" in message.lower()

    def test_plugin_stack_command_registered(self, sim, loaded_example, run_cmd):
        run_cmd("CRE KL001,A320,52,4,90,FL100,250")
        output = run_cmd("PASSENGERS KL001 150")
        assert "150" in output

    def test_plugin_entity_tracks_aircraft(self, sim, loaded_example, run_cmd):
        run_cmd("CRE KL001,A320,52,4,90,FL100,250")
        run_cmd("PASSENGERS KL001 42")
        output = run_cmd("PASSENGERS KL001")
        assert "42" in output
