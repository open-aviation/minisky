"""MiniSky public API.

This is the curated public import surface for runtime users and plugin authors.
Prefer import runtime, plugin, command, and result contracts from here and avoid
reaching into the individual modules.
For physical quantities and semantic runtime values, use
[`minisky.quantities`][] and [`minisky.types`][].
"""

from minisky import aero as aero
from minisky import geo as geo
from minisky._internal.active_waypoint import ActiveWaypoint
from minisky._internal.autopilot import Autopilot
from minisky._internal.command import (
    AcId,
    AcIdSelection,
    ArgumentIssue,
    CmdParser,
    CommandCursor,
    CommandField,
    CommandParseContext,
    Converter,
    CoordinateWaypoint,
    DistanceM,
    Keyword,
    LatitudeArg,
    LatLonDeg,
    LongitudeArg,
    NamedWaypoint,
    OnOff,
    ParseResult,
    SimTimeS,
    SourceSpan,
    Spanned,
    SpeedMps,
    Text,
    TimeS,
    Token,
    UseRunwayHeading,
    VspdMps,
    command,
)
from minisky._internal.conditions import Condition
from minisky._internal.config import (
    MiniSkyConfig,
    ServerConfig,
    default_user_config_dir,
    default_user_config_toml_path,
)
from minisky._internal.conflict.detection import ConflictDetection
from minisky._internal.conflict.resolution import ConflictResolution
from minisky._internal.console import ConsoleIO, ConsoleSubscription
from minisky._internal.geo_commands import GeoCommands
from minisky._internal.groups import TrafficGroups
from minisky._internal.guidance import APorASAS
from minisky._internal.kinematics import Kinematics
from minisky._internal.navigation import Navdatabase
from minisky._internal.performance.openap import OpenAP
from minisky._internal.plugin import (
    Plugin,
    PluginBuild,
    PluginContext,
    PluginLifespan,
    PluginManager,
    PluginRuntime,
    PluginSpec,
    PluginStatus,
)
from minisky._internal.plugin_decorators import HookName, hook, replacement
from minisky._internal.plugin_entity import Entity
from minisky._internal.result import Err, Ok, Result, UnwrapError
from minisky._internal.route import Route, RouteCommands, RunwayReference
from minisky._internal.runner import Runner
from minisky._internal.runtime import MiniSky
from minisky._internal.shapes import Shapes
from minisky._internal.simulation import Simulation, SimulationState
from minisky._internal.stack import CommandStack
from minisky._internal.streaming import AcData, SimInfo, Snapshot
from minisky._internal.traffic import Traffic
from minisky._internal.traffic_arrays import (
    OptionalArray,
    ReplaceableManager,
    TrafficArrays,
    VariantArray,
)
from minisky._internal.variables import VariableExplorer
from minisky._internal.wind import Wind, WindLevel
from minisky.geo import MagneticDeclination, MagneticDeclinationGrid

__all__ = (  # noqa: RUF022 - public API and docs use this semantic order
    # runtime
    "MiniSky",
    "MiniSkyConfig",
    "ServerConfig",
    "Simulation",
    "SimulationState",
    "Runner",
    "CommandStack",
    "ConsoleIO",
    "ConsoleSubscription",
    "Traffic",
    "Navdatabase",
    "Snapshot",
    "SimInfo",
    "AcData",
    "default_user_config_dir",
    "default_user_config_toml_path",
    "MagneticDeclination",
    "MagneticDeclinationGrid",
    # plugin authoring
    "Plugin",
    "PluginContext",
    "PluginSpec",
    "PluginRuntime",
    "PluginStatus",
    "PluginBuild",
    "PluginLifespan",
    "PluginManager",
    "HookName",
    "Entity",
    "hook",
    "replacement",
    # replacement authoring
    "ActiveWaypoint",
    "Autopilot",
    "APorASAS",
    "Kinematics",
    "OpenAP",
    "ConflictDetection",
    "ConflictResolution",
    "Condition",
    "GeoCommands",
    "RouteCommands",
    "Shapes",
    "TrafficGroups",
    "VariableExplorer",
    "Wind",
    "WindLevel",
    "TrafficArrays",
    "OptionalArray",
    "VariantArray",
    "ReplaceableManager",
    "Route",
    "RunwayReference",
    # public utility modules
    "aero",
    "geo",
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
    "LatitudeArg",
    "LongitudeArg",
    "LatLonDeg",
    "DistanceM",
    "SpeedMps",
    "VspdMps",
    "TimeS",
    "SimTimeS",
    "SourceSpan",
    "NamedWaypoint",
    "CoordinateWaypoint",
    "UseRunwayHeading",
    # custom command parsers
    "CmdParser",
    "CommandField",
    "Converter",
    "CommandParseContext",
    "CommandCursor",
    "ParseResult",
    "ArgumentIssue",
    "Spanned",
)
