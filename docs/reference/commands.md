---
render_macros: true
---

# Stack commands

Every text command understood by the simulator — usable in scenario files, the
[console](../guides/console.md), the REST [`stack/` endpoint](../guides/rest-api.md),
or [`runtime.commands.stack()`][minisky.stack.CommandStack.stack] from Python. Commands are
case-insensitive.

Argument conventions: optional arguments are enclosed in `[...]`; `callsign` is an
aircraft callsign; `alt` accepts flight levels (`FL100`), feet, or metres; `spd`
accepts CAS in knots or Mach (below 1); `latlon` accepts coordinates or any named
waypoint, navaid, airport, or aircraft.

{{ command_docs() }}
