"""Multicopter mission autopilot.

A thin subclass of the core
[`Autopilot`][minisky.Autopilot]: LNAV already emits a
*track* command, which is exactly what the decoupled multicopter kinematics
consumes, so no guidance rewrite is needed. What the stock FMS cannot
express is added here — the `HOVER` primitive, rerouted `HDG` semantics
(nose only), and fly-over route defaults. The fixed waypoint capture radius
lives in `MulticopterActiveWaypoint` (see `minisky_multicopter.activewp`).

`HOVER` is deliberately composable rather than a scripted manoeuvre: it
brakes to a stop and holds position, optionally at a commanded altitude, and
hands control back to the route after the optional hold time. Anything more
elaborate (a delivery profile, say) is written in the scenario from
`HOVER`, `ALT` and `LNAV` commands.

Selected with `SELECTIMPL AUTOPILOT MULTICOPTERAUTOPILOT`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from minisky import (
    AcIdSelection,
    Autopilot,
    Err,
    HeadingDeg,
    Ok,
    OptionalArray,
    Result,
    VariantArray,
    replacement,
)
from minisky import quantities as q
from minisky.types import AircraftIndex, AirspeedKind, StdPressureAltM

from minisky_multicopter.entity import get_multicopter

if TYPE_CHECKING:
    from collections.abc import Callable

    from minisky import Simulation, Traffic


@replacement
class MulticopterAutopilot(Autopilot):
    """Autopilot with a multicopter hover primitive."""

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic, get_simulation)
        with self.settrafarrays():
            self.swhover = np.array([], dtype=bool)
            """Whether each aircraft is in a commanded hover."""
            self.hovertimer: OptionalArray[q.DurationS[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Remaining hold time of an active hover."""
            self.resume_airspeed: VariantArray[np.ndarray] = VariantArray(
                np.array([]), np.array([], dtype=np.uint8)
            )
            """[CAS in m/s][minisky.types.CasMps] or [Mach][minisky.types.Mach] selection restored
            when hover ends."""
            self.resume_lnav = np.array([], dtype=bool)
            self.resume_vnav = np.array([], dtype=bool)
            self.resume_vnav_airspeed = np.array([], dtype=bool)

    def create(self, n: int = 1) -> None:
        """Seed the hover state of n newly created aircraft.

        New aircraft start with no hover active; new multicopters get
        fly-over waypoints by default (they fly point-to-point, without a
        turn-anticipation arc).
        """
        super().create(n)
        self.swhover[-n:] = False
        self.resume_airspeed.values[-n:] = 0.0
        self.resume_airspeed.kind[-n:] = AirspeedKind.CAS
        self.resume_lnav[-n:] = False
        self.resume_vnav[-n:] = False
        self.resume_vnav_airspeed[-n:] = False

        # Membership by typecode from the performance table: the Multicopter
        # entity may be created after this autopilot in the traffic tree, so
        # its arrays cannot be relied upon here.
        mc = get_multicopter(self.traffic)
        if mc is None:
            return
        for offset, typecode in enumerate(self.traffic.typecode[-n:], start=-n):
            if typecode.upper() in mc.typespecs:
                self.route[offset].swflyby = False

    def update(self) -> None:
        """Run the FMS, then advance any active hovers (vectorized).

        A timed hover counts its hold time down only while the position is
        actually held: stopped, at the selected altitude. A hover ends when
        that timer expires (the saved route state is restored; the selected
        altitude stays at the hover altitude) or when LNAV is re-engaged
        externally (LNAV is then left as commanded).
        """
        super().update()
        mc = get_multicopter(self.traffic)
        if not self.swhover.any() or mc is None:
            return

        traf = self.traffic
        # LNAV was re-engaged externally: cancel those hovers.
        cancel = self.swhover & traf.swlnav
        # Timed hovers holding position and altitude: count the timer down.
        timed = self.swhover & (self.hovertimer.present)
        holding = (
            timed
            & ~traf.swlnav
            & (traf.gs < mc.config.gs_hover)
            & (np.abs(traf.alt - traf.selalt) < mc.config.alt_capture)
        )
        self.hovertimer.values[holding] -= self.simulation.simdt
        expired = holding & (self.hovertimer.values <= 0.0)

        # Restore the saved route state; expiry also re-engages LNAV/VNAV.
        resume = cancel | expired
        traf.selected_airspeed.values[:] = np.where(
            resume, self.resume_airspeed.values, traf.selected_airspeed.values
        )
        traf.selected_airspeed.kind[:] = np.where(
            resume, self.resume_airspeed.kind, traf.selected_airspeed.kind
        ).astype(np.uint8)
        traf.swvnavairspeed = np.where(resume, self.resume_vnav_airspeed, traf.swvnavairspeed)
        traf.swvnav = np.where(cancel, self.resume_vnav, traf.swvnav)
        traf.swlnav = np.where(expired, self.resume_lnav, traf.swlnav)
        traf.swvnav = np.where(expired, self.resume_vnav & self.resume_lnav, traf.swvnav)
        self.swhover = self.swhover & ~resume
        self.hovertimer.clear(resume)

    def selhdgcmd(self, idx: AcIdSelection, hdg: HeadingDeg) -> Result[str, str]:
        """Select the autopilot heading; for multicopters, yaw the nose only.

        For multicopter rows the HDG stack command is an alias of `YAW`:
        it rotates the body without touching the track, and LNAV stays
        engaged — the velocity vector keeps following the FMS or conflict
        resolution. Other aircraft keep the stock behaviour.
        """
        mc = get_multicopter(self.traffic)
        if mc is None:
            return super().selhdgcmd(idx, hdg)

        is_multicopter = mc.ismulticopter[idx]
        if not is_multicopter.any():
            return super().selhdgcmd(idx, hdg)

        message = "heading set"
        for acidx in idx[is_multicopter]:
            result = mc.yaw(int(acidx), hdg)
            if isinstance(result, Err):
                return result
            message = result.ok()

        fixed_wing = idx[~is_multicopter]
        if fixed_wing.size:
            return super().selhdgcmd(fixed_wing, hdg)
        return Ok(message)

    def hover(
        self,
        idx: AircraftIndex,
        duration: q.DurationS[float] | None = None,
        alt: StdPressureAltM | None = None,
    ) -> Result[str, str]:
        """Hold position, optionally for a fixed time at a given altitude.

        Backs the `HOVER` stack command declared on the Multicopter entity,
        which delegates here at call time so the command survives the
        autopilot instance being swapped on reset.

        Args:
            duration: Hold time; `None` holds indefinitely.
            alt: Hover altitude; `None` holds the current altitude.
        """
        callsign = self.traffic.callsign[idx]
        mc = get_multicopter(self.traffic)
        if mc is None or not mc.ismulticopter[idx]:
            return Err(f"HOVER: {callsign} is not a multicopter")

        if not self.swhover[idx]:
            # Entering the hover: save the route state to resume later.
            self._suspend_route(idx)
            self.swhover[idx] = True
        if alt is not None:
            result = self.selaltcmd(np.asarray([idx], dtype=int), alt)
            if isinstance(result, Err):
                return result
        if duration is None:
            self.hovertimer.clear(idx)
        else:
            self.hovertimer.set(idx, duration)

        if duration is None:
            return Ok(f"HOVER {callsign}: holding position (resume with LNAV {callsign} ON)")
        return Ok(f"HOVER {callsign}: holding position for {duration:.0f} s")

    def _suspend_route(self, idx: AircraftIndex) -> None:
        """Save the route state of one aircraft and command a hover."""
        traf = self.traffic
        self.resume_lnav[idx] = traf.swlnav[idx]
        self.resume_vnav[idx] = traf.swvnav[idx]
        self.resume_vnav_airspeed[idx] = traf.swvnavairspeed[idx]
        self.resume_airspeed.values[idx] = traf.selected_airspeed.values[idx]
        self.resume_airspeed.kind[idx] = traf.selected_airspeed.kind[idx]
        traf.swlnav[idx] = False
        traf.swvnav[idx] = False
        traf.swvnavairspeed[idx] = False
        traf.selected_airspeed.values[idx] = 0.0
        traf.selected_airspeed.kind[idx] = AirspeedKind.CAS
        traf.selalt[idx] = traf.alt[idx]
