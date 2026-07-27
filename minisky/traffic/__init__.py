"""Air-traffic state and dynamics.

[`Traffic`][minisky.traffic.traffic.Traffic] is owned as [`runtime.traffic`][minisky.traffic.traffic.Traffic] and
contains the per-aircraft arrays plus autopilot, routes, conflict detection and
resolution, performance, wind, turbulence, uncertainty, trails, and groups.
"""

from minisky.traffic.traffic import Traffic

__all__ = ("Traffic",)
