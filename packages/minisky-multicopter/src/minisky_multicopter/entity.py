"""Multicopter membership and per-aircraft state.

Holds the plugin-owned per-aircraft arrays that mark which aircraft are
multicopters and carry their decoupled body heading and yaw rate, plus the
stack commands that read and write them (`MCOPT`, `YAW`, `YAWRATE`,
`HOVER`, `BATT`) and the hooks that keep the multicopter implementations
selected (on the first simulation step after loading, and again after every
reset, which reverts all replaceables to their core defaults).

Membership is deliberately *not* `traf.perf.lifttype == LiftType.ROTORCRAFT`:
that
set also contains the EC35, a crewed helicopter, which this plugin does not
model. It is the typecode set of the loaded performance table instead (see
`minisky_multicopter.config`). `MCOPT` can disable and re-enable configured types.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from minisky import (
    AcId,
    Entity,
    Err,
    HeadingDeg,
    Ok,
    OnOff,
    PositiveFiniteFloat,
    Result,
    TimeS,
    command,
    geo,
    hook,
)
from minisky import quantities as q
from minisky.types import MagneticHeadingDeg, StdPressureAltM

from minisky_multicopter.config import (
    MulticopterConfig,
    MulticopterTypeSpec,
    load_type_table,
)

if TYPE_CHECKING:
    from collections.abc import Mapping

    from minisky import Traffic

DEFAULT_YAWRATE: q.YawRateDegPerS[float] = 90.0

# NOTE(abraham): SELECTIMPL replaces one implementation for the whole traffic
# component. it forces the multicopter replacements to run base behaviour for
# the full fleet and then patch only multicopter rows. a future per-archetype
# dispatcher should make these global swaps unnecessary
IMPLEMENTATIONS = (
    ("KINEMATICS", "MULTICOPTERKINEMATICS"),
    ("APORASAS", "MULTICOPTERAPORASAS"),
    ("AUTOPILOT", "MULTICOPTERAUTOPILOT"),
    ("ACTIVEWAYPOINT", "MULTICOPTERACTIVEWAYPOINT"),
    ("OPENAP", "MULTICOPTERPERF"),
)


def get_multicopter(traffic: Traffic) -> Multicopter | None:
    """Return the Multicopter entity attached to a traffic tree, if any.

    The entity is mounted by the plugin build as a child node of `traffic`.
    The replaceable subclasses use this lookup so that, when one of them is
    selected without the plugin loaded, they degrade to base behaviour
    instead of crashing.
    """
    return next(
        (child for child in traffic._children if isinstance(child, Multicopter)),
        None,
    )


class Multicopter(Entity):
    """Per-aircraft multicopter state."""

    def __init__(
        self,
        typespecs: Mapping[str, MulticopterTypeSpec] | None = None,
        config: MulticopterConfig | None = None,
    ) -> None:
        super().__init__()
        self.typespecs = dict(typespecs) if typespecs is not None else load_type_table()
        self.config = config if config is not None else MulticopterConfig()
        self._selected = False
        with self.settrafarrays():
            self.ismulticopter = np.array([], dtype=bool)
            """Whether each aircraft uses multicopter kinematics."""
            self.selhdg: q.TrueHeadingDegrees[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.swselhdg = np.array([], dtype=bool)
            """Whether body heading is explicitly selected rather than following track."""
            self.yawrate: q.YawRateDegPerS[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

    def create(self, n: int = 1) -> None:
        """Seed multicopter state for n newly created aircraft.

        Membership follows from the typecode; the body heading starts
        unconstrained (nose follows track) at the default yaw rate.
        """
        super().create(n)
        self.ismulticopter[-n:] = [
            typecode.upper() in self.typespecs for typecode in self.traffic.typecode[-n:]
        ]
        self.selhdg[-n:] = self.traffic.hdg[-n:]
        self.swselhdg[-n:] = False
        self.yawrate[-n:] = DEFAULT_YAWRATE

    def mask(self) -> np.ndarray:
        """Return the boolean row mask of aircraft flown as multicopters."""
        return self.ismulticopter

    def select_implementations(self) -> None:
        """Swap the multicopter implementations onto the owning traffic.

        Equivalent to issuing `SELECTIMPL <BASE> <IMPL>` for each entry of
        `IMPLEMENTATIONS`; replaces the live instance immediately.
        """
        # NOTE(abraham): plugin loading should eventually register behaviour,
        # not mutate which concrete implementation owns every aircraft.
        # importing multicopter should not have any side effects!
        for basename, implname in IMPLEMENTATIONS:
            result = self.traffic.select_implementation(basename, implname)
            if result.is_err():
                raise RuntimeError(f"MULTICOPTER: {result.err()}")
        self._selected = True

    @hook("preupdate")
    def ensure_implementations(self) -> None:
        """Select the multicopter implementations on the first step after load.

        Replacements are installed when the plugin loads but can only be
        selected once the plugin is published, so the initial selection
        happens here. A manual `SELECTIMPL` afterwards is respected until
        the next reset.
        """
        if not self._selected:
            self.select_implementations()

    @hook("reset")
    def reselect_implementations(self) -> None:
        """Re-select the multicopter implementations after a reset.

        A reset reverts every replaceable to its core default; this hook runs
        afterwards and restores the multicopter set.
        """
        self.select_implementations()

    @command(name="MCOPT")
    def mcopt_status(self, idx: AcId) -> Result[str, str]:
        """Report whether multicopter behavior is enabled for an aircraft."""
        callsign = self.traffic.callsign[idx]
        return Ok(f"MCOPT {callsign}: {'ON' if self.ismulticopter[idx] else 'OFF'}")

    @command(name="MCOPT")
    def set_mcopt(self, idx: AcId, flag: OnOff) -> Result[str, str]:
        """Enable or disable multicopter behavior for an aircraft."""
        callsign = self.traffic.callsign[idx]
        if flag and self.traffic.typecode[idx].upper() not in self.typespecs:
            return Err(f"MCOPT: {callsign} type is not configured as a multicopter")

        self.ismulticopter[idx] = flag
        # Multicopters fly point-to-point: newly added waypoints default to
        # fly-over (restored to fly-by when switched back off).
        self.traffic.ap.route[idx].swflyby = not flag
        if flag:
            # Start with the nose unconstrained, following the track.
            self.selhdg[idx] = self.traffic.hdg[idx]
            self.swselhdg[idx] = False
        return Ok(f"MCOPT {callsign}: {'ON' if flag else 'OFF'}")

    @command
    def yaw(self, idx: AcId, hdg: HeadingDeg) -> Result[str, str]:
        """Command the body heading (nose direction) of a multicopter.

        The velocity vector keeps following the track command from the FMS
        or conflict resolution, so this rotates the aircraft in place.
        """
        if not self.ismulticopter[idx]:
            callsign = self.traffic.callsign[idx]
            return Err(f"YAW: {callsign} is not a multicopter")

        resolved_hdg = hdg.value
        if isinstance(hdg, MagneticHeadingDeg):
            resolved_hdg += geo.magdec(float(self.traffic.lat[idx]), float(self.traffic.lon[idx]))
        resolved_hdg %= 360.0
        self.selhdg[idx] = resolved_hdg
        self.swselhdg[idx] = True
        return Ok(f"YAW {self.traffic.callsign[idx]}: nose to {resolved_hdg:.0f} deg")

    @command(name="YAWRATE")
    def yawrate_status(self, idx: AcId) -> Result[str, str]:
        """Report the maximum yaw rate of a multicopter."""
        callsign = self.traffic.callsign[idx]
        return Ok(f"YAWRATE {callsign}: {self.yawrate[idx]:.0f} deg/s")

    @command(name="YAWRATE")
    def set_yawrate(
        self,
        idx: AcId,
        yawrate: q.YawRateDegPerS[PositiveFiniteFloat],
    ) -> Result[str, str]:
        """Set the maximum yaw rate of a multicopter."""
        callsign = self.traffic.callsign[idx]
        self.yawrate[idx] = yawrate
        return Ok(f"YAWRATE {callsign}: {yawrate:.0f} deg/s")

    @command
    def hover(
        self,
        idx: AcId,
        duration: TimeS | None = None,
        alt: StdPressureAltM | None = None,
    ) -> Result[str, str]:
        """Hold position, optionally for a fixed time at a given altitude.

        Suspends LNAV/VNAV, commands zero ground speed, and holds the given
        altitude (the current one when omitted) — with an altitude the
        aircraft moves there vertically, at a fixed position. With a
        duration, the route resumes once position and altitude have been
        held that long; without one, the aircraft hovers until LNAV is
        re-engaged. Repeating the command while hovering updates the hold
        time and altitude, and a plain ALT command changes the hover
        altitude as well.
        """
        # Deferred import: the autopilot module imports this one.
        from minisky_multicopter.autopilot import MulticopterAutopilot

        ap = self.traffic.ap
        if not isinstance(ap, MulticopterAutopilot):
            return Err("HOVER: SELECTIMPL AUTOPILOT MULTICOPTERAUTOPILOT first")
        return ap.hover(idx, duration, alt)

    @command
    def batt(self, idx: AcId) -> Result[str, str]:
        """Report the battery state of charge, power draw and endurance."""
        # Deferred import: the perf module imports this one.
        from minisky_multicopter.perf import MulticopterPerf

        callsign = self.traffic.callsign[idx]
        if not self.ismulticopter[idx]:
            return Err(f"BATT: {callsign} is not a multicopter")
        perf = self.traffic.perf
        if not isinstance(perf, MulticopterPerf):
            return Err("BATT: SELECTIMPL OPENAP MULTICOPTERPERF first")
        return perf.batt(idx)
