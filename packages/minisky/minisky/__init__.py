"""MiniSky public API.

This is the curated public import surface for runtime users and plugin authors.
Prefer import runtime, plugin, command, and result contracts from here and avoid
reaching into the individual modules.
For physical quantities and semantic runtime values, use
[`minisky.quantities`][] and [`minisky.types`][].
"""

from minisky.core.config import (
    MiniSkyConfig,
    default_user_config_dir,
    default_user_config_toml_path,
)
from minisky.plugin.entity import Entity
from minisky.plugin.plugin import Plugin, PluginContext, PluginRuntime, PluginSpec, PluginStatus
from minisky.plugin.plugin_decorators import hook, replacement
from minisky.result import Err, Ok, Result, UnwrapError
from minisky.runtime import MiniSky
from minisky.simulation.simulation import SimulationState
from minisky.stack_command import (
    AcId,
    AcIdSelection,
    ArgumentIssue,
    CmdParser,
    CommandCursor,
    CommandParseContext,
    CoordinateWaypoint,
    DistanceM,
    FiniteFloat,
    HeadingDeg,
    Keyword,
    LatLonDeg,
    NamedWaypoint,
    NonNegativeFiniteFloat,
    OnOff,
    ParseResult,
    PositiveFiniteFloat,
    SimTimeS,
    Spanned,
    SpeedMps,
    Text,
    TimeS,
    Token,
    VspdMps,
    Wpt,
    command,
)
from minisky.streaming import Snapshot

__all__ = (  # noqa: RUF022 - public API and docs use this semantic order
    # runtime
    "MiniSky",
    "MiniSkyConfig",
    "SimulationState",
    "Snapshot",
    "default_user_config_dir",
    "default_user_config_toml_path",
    # plugin authoring
    "Plugin",
    "PluginContext",
    "PluginSpec",
    "PluginRuntime",
    "PluginStatus",
    "Entity",
    "hook",
    "replacement",
    # results
    "Result",
    "Ok",
    "Err",
    "UnwrapError",
    # commands
    "command",
    "AcId",
    "AcIdSelection",
    "Keyword",
    "Token",
    "Text",
    "OnOff",
    "HeadingDeg",
    "LatLonDeg",
    "DistanceM",
    "SpeedMps",
    "VspdMps",
    "TimeS",
    "SimTimeS",
    "Wpt",
    "NamedWaypoint",
    "CoordinateWaypoint",
    "FiniteFloat",
    "NonNegativeFiniteFloat",
    "PositiveFiniteFloat",
    # custom command parsers
    "CmdParser",
    "CommandParseContext",
    "CommandCursor",
    "ParseResult",
    "ArgumentIssue",
    "Spanned",
)
