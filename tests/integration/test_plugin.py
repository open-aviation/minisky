"""Integration tests for the plugin system (AST discovery + loading).

Plugin loading imports modules and registers stack commands globally, so
these tests run against the session singletons like other integration tests.
"""

import warnings

import pytest

import minisky
from minisky.plugin import Plugin


class TestDiscovery:
    def test_discover_finds_example_plugins(self, bs):
        # discovery already ran during minisky.init(); it is idempotent
        minisky.plugin.discover()
        assert "EXAMPLE" in Plugin.plugins

    def test_discovery_does_not_import(self, bs):
        plug = Plugin.plugins["EXAMPLE"]
        if not plug.loaded:
            assert plug.imp is None  # AST parsing only, no module import

    def test_manage_plugins_list(self, bs):
        ok, text = minisky.plugin.manage_plugins("LIST")
        assert ok
        assert "EXAMPLE" in text

    def test_unknown_plugin_load_fails(self, bs):
        ok, msg = Plugin.load("NOSUCHPLUGIN")
        assert not ok
        assert "not found" in msg.lower()

    def test_discovery_emits_no_deprecation_warning(self, bs):
        # AST config parsing used the ast.Constant.s alias, deprecated
        # since Python 3.12 and removed in 3.14.
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            minisky.plugin.discover()
        assert "EXAMPLE" in Plugin.plugins


@pytest.fixture
def loaded_example(bs):
    """Load the EXAMPLE plugin (loading is not reversible, so load only once)."""
    if not Plugin.plugins["EXAMPLE"].loaded:
        ok, msg = Plugin.load("EXAMPLE")
        assert ok, msg
    return Plugin.plugins["EXAMPLE"]


class TestLoading:
    def test_load_registers_plugin(self, bs, loaded_example):
        assert loaded_example.loaded
        assert "EXAMPLE" in Plugin.loaded_plugins

    def test_double_load_rejected(self, bs, loaded_example):
        ok, msg = Plugin.load("EXAMPLE")
        assert not ok
        assert "already loaded" in msg.lower()

    def test_plugin_stack_command_registered(self, bs, sim, loaded_example, run_cmd):
        # example.py registers the PASSENGERS command via @stack.command
        run_cmd("CRE KL001,A320,52,4,90,FL100,250")
        output = run_cmd("PASSENGERS KL001 150")
        assert "150" in output

    def test_plugin_entity_tracks_aircraft(self, bs, sim, loaded_example, run_cmd):
        run_cmd("CRE KL001,A320,52,4,90,FL100,250")
        run_cmd("PASSENGERS KL001 42")
        output = run_cmd("PASSENGERS KL001")
        assert "42" in output
