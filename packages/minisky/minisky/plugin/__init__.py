"""Public contracts for runtime-local MiniSky plugins.

Plugin packages can expose a [Plugin][minisky.plugin.Plugin] value through
the `minisky.plugins` entry-point group.
"""

from __future__ import annotations

from minisky.command import (
    AcId,
    AcIdSelection,
    AltM,
    ArgumentIssue,
    CmdParser,
    CommandParseContext,
    FiniteFloat,
    HeadingDeg,
    Keyword,
    LatLonDeg,
    MagneticHeadingDeg,
    OnOff,
    Parsed,
    ParseResult,
    PositiveFiniteFloat,
    RunwayHeadingRequest,
    SpeedMpsOrMach,
    Text,
    TimeS,
    Token,
    TrueHeadingDeg,
    UseRunwayHeading,
    VspdMps,
    command,
)
from minisky.plugin.entity import Entity
from minisky.plugin.plugin import (
    Plugin,
    PluginContext,
    PluginError,
    PluginManager,
    PluginRuntime,
    PluginSpec,
    PluginStatus,
)
from minisky.plugin.plugin_decorators import HookName, hook, replacement

__all__ = (
    "AcId",
    "AcIdSelection",
    "AltM",
    "ArgumentIssue",
    "CmdParser",
    "CommandParseContext",
    "Entity",
    "FiniteFloat",
    "HeadingDeg",
    "HookName",
    "Keyword",
    "LatLonDeg",
    "MagneticHeadingDeg",
    "OnOff",
    "ParseResult",
    "Parsed",
    "Plugin",
    "PluginContext",
    "PluginError",
    "PluginManager",
    "PluginRuntime",
    "PluginSpec",
    "PluginStatus",
    "PositiveFiniteFloat",
    "RunwayHeadingRequest",
    "SpeedMpsOrMach",
    "Text",
    "TimeS",
    "Token",
    "TrueHeadingDeg",
    "UseRunwayHeading",
    "VspdMps",
    "command",
    "hook",
    "replacement",
)
