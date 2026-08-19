"""Hover-capable, track/heading-decoupled electric multicopters for MiniSky."""

from __future__ import annotations

from minisky import Plugin, PluginContext, PluginSpec

from minisky_multicopter.activewp import MulticopterActiveWaypoint
from minisky_multicopter.aporasas import MulticopterAPorASAS
from minisky_multicopter.autopilot import MulticopterAutopilot
from minisky_multicopter.config import (
    MulticopterConfig,
    MulticopterTypeSpec,
    RotorAirframeSpec,
    load_type_table,
)
from minisky_multicopter.entity import Multicopter, get_multicopter
from minisky_multicopter.kinematics import MulticopterKinematics
from minisky_multicopter.perf import MulticopterPerf

__all__ = (
    "Multicopter",
    "MulticopterAPorASAS",
    "MulticopterActiveWaypoint",
    "MulticopterAutopilot",
    "MulticopterConfig",
    "MulticopterKinematics",
    "MulticopterPerf",
    "MulticopterTypeSpec",
    "RotorAirframeSpec",
    "get_multicopter",
    "plugin",
)


def build(context: PluginContext[MulticopterConfig]) -> PluginSpec:
    typespecs = load_type_table(context.config.performance_path)
    context.mount(Multicopter(typespecs, context.config))
    return context.finish(
        replacements=(
            MulticopterKinematics,
            MulticopterAPorASAS,
            MulticopterAutopilot,
            MulticopterActiveWaypoint,
            MulticopterPerf,
        )
    )


plugin = Plugin(build=build, config_class=MulticopterConfig)
