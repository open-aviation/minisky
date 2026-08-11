"""MULTICOPTER — simulate small electric multirotors.

Makes MiniSky fly DJI MAVIC/M600/PHAN4-class and Amazon/Matternet-style
delivery drones with multicopter behaviour: hover and yaw at zero airspeed,
and a velocity vector decoupled from the body heading (the aircraft can
strafe — change course without rotating the nose).

Everything is implemented as replaceable subclasses of core entities, which
the plugin registers on load and keeps selected through its hooks (the first
simulation step after loading, and every reset):

- `KINEMATICS`      -> `MulticopterKinematics` — yaw-rate-limited
  heading, track-driven velocity vector.
- `APORASAS`        -> `MulticopterAPorASAS` — no track-to-heading
  coupling for multicopter rows.
- `AUTOPILOT`       -> `MulticopterAutopilot` — HOVER primitive,
  HDG-yaws-the-nose semantics, fly-over route defaults.
- `ACTIVEWAYPOINT`  -> `MulticopterActiveWaypoint` — fixed waypoint
  capture radius (the bank-angle turn distance degenerates at hover speeds).
- `OPENAP`          -> `MulticopterPerf` — electric performance:
  required thrust, momentum-theory power, battery state of charge with
  envelope feedback at low charge.

Fixed-wing aircraft in the same simulation are untouched: every override
calls `super()` and adjusts only the multicopter rows. Helicopters are out
of scope — membership is a typecode set, not `LiftType.ROTORCRAFT`.

Config (optional, under `[plugins.multicopter]` in the MiniSky user config
file, validated into `MulticopterConfig`):

- `capture_radius`: waypoint capture radius for multicopters [m]
  (default 10).
- `performance_path`: path of a performance TOML that extends or overrides
  the built-in per-typecode table (default: `multicopter.toml` in the
  platform cache directory, when it exists). See
  `minisky_multicopter.config`.
- `soc_low`, `lowbatt_spd_factor`, `lowbatt_vs_factor`: low-battery
  envelope feedback — the state-of-charge threshold (default 0.2) and the
  maximum-speed and climb-rate factors applied below it (defaults 0.6
  and 0.5).
- `gs_hover`, `alt_capture`: hover-hold thresholds — the ground speed
  below which an aircraft counts as stopped [m/s] (default 0.1) and the
  altitude tolerance [m] (default 0.5).
- `cruise_speed_fraction`: cruise speed as a fraction of the envelope
  maximum, for the range-derived pack-energy fallback (default 0.8).

Stack commands: MCOPT, YAW, YAWRATE, HOVER, BATT.
"""

from __future__ import annotations

from minisky import plugin as plugin_api

from minisky_multicopter.activewp import MulticopterActiveWaypoint
from minisky_multicopter.aporasas import MulticopterAPorASAS
from minisky_multicopter.autopilot import MulticopterAutopilot
from minisky_multicopter.config import (
    MulticopterConfig,
    MulticopterTypeSpec,
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
    "get_multicopter",
    "plugin",
)


def build(context: plugin_api.PluginContext[MulticopterConfig]) -> plugin_api.PluginSpec:
    """Build the multicopter components for one MiniSky runtime.

    Loads and validates the performance table (built-in data merged with
    the optional user TOML), mounts the per-aircraft state entity (which
    also carries the stack commands and the implementation-selection hooks)
    and registers the multicopter implementations as runtime-local
    replacements.
    """
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


plugin = plugin_api.Plugin(build=build, config_class=MulticopterConfig)
