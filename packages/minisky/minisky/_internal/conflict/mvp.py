"""Conflict resolution based on the Modified Voltage Potential algorithm.

The Modified Voltage Potential (MVP) method treats each conflict as a
repulsive interaction: for every conflicting pair the predicted position at
the closest point of approach (CPA) is displaced just outside the (enlarged)
protected zone, and the velocity change required to achieve that displacement
within the remaining time to CPA is the resolution vector. Resolution vectors
from multiple simultaneous conflicts are summed per aircraft, making the
method implicitly cooperative and pairwise-symmetric (both aircraft manoeuvre
away from each other).

The resulting velocity change can be constrained to horizontal-only (heading
and/or speed) or vertical-only manoeuvres, and optional priority (right of
way) rules can assign the manoeuvre to only one aircraft of a pair.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, NamedTuple

import numpy as np

from minisky import quantities as q
from minisky._internal.config import MiniSkyConfig
from minisky._internal.conflict.resolution import (
    ConflictResolution,
    HorizontalResolutionMethod,
    PriorityCode,
    VerticalResolutionMethod,
)
from minisky._internal.result import Ok, Result
from minisky.types import AircraftIndex

_CRUISE_VERTICAL_RATE: q.VerticalRateMps[float] = 0.1
_HEAD_ON_CPA_FLOOR: q.DistanceM[float] = 10.0

if TYPE_CHECKING:
    from minisky._internal.traffic import Traffic

    from .detection import ConflictDetection


class MVP(ConflictResolution):
    """Conflict resolution using the Modified Voltage Potential Method.

    For each detected conflict pair, [`MVP`][.MVP] computes a repulsive
    velocity-change vector that pushes the closest point of approach out of
    the resolution zone (the protected zone scaled by `resofach` and
    `resofacv`). [`resolve`][.resolve] accumulates these vectors for all
    conflicts of each aircraft, adds them to the current velocity, and
    converts the result into track, ground speed, vertical speed, and
    altitude advisories, capped to the aircraft performance envelope.

    Selected via the stack command `RESO MVP`. Resolution manoeuvres can be
    restricted with RMETHH (horizontal: heading and/or speed) and RMETHV
    (vertical speed only).
    """

    def __init__(
        self,
        config: MiniSkyConfig,
        traffic: Traffic,
        select_implementation: Callable[[str, str], Result[str, str]],
    ) -> None:
        super().__init__(config, traffic, select_implementation)
        self.swresohoriz = True
        """Whether resolution is restricted to the horizontal plane."""
        self.swresospd = False
        """Whether horizontal resolution uses speed only."""
        self.swresohdg = False
        """Whether horizontal resolution uses heading only."""
        self.swresovert = False
        """Whether resolution is restricted to the vertical direction."""

    def priority_status(self) -> Result[str, str]:
        """Show MVP priority-rule options and state."""
        return Ok(
            "PRIORULES [ON/OFF] [PRIOCODE]"
            "\nAvailable priority codes: "
            "\n     FF1:  Free Flight Primary (No Prio) "
            "\n     FF2:  Free Flight Secondary (Cruising has priority)"
            "\n     FF3:  Free Flight Tertiary (Climbing/descending has priority)"
            "\n     LAY1: Layers Primary (Cruising has priority + horizontal resolutions)"
            "\n     LAY2: Layers Secondary (Climbing/descending has priority + horizontal resolutions)"
            f"\nPriority is currently {'ON' if self.priority_code is not None else 'OFF'}"
            f"\nPriority code is currently: "
            f"{self.priority_code.value if self.priority_code is not None else 'NONE'}"
        )

    def horizontal_method_status(self) -> Result[str, str]:
        """Show MVP horizontal resolution limitations."""
        return Ok(
            "RMETHH [ON / BOTH / OFF / NONE / SPD / HDG]"
            f"\nHorizontal resolution limitation is currently {'ON' if self.swresohoriz else 'OFF'}"
            f"\nSpeed resolution limitation is currently {'ON' if self.swresospd else 'OFF'}"
            f"\nHeading resolution limitation is currently {'ON' if self.swresohdg else 'OFF'}"
        )

    def configure_horizontal_method(self, value: HorizontalResolutionMethod) -> Result[str, str]:
        """Configure MVP horizontal resolution limitations."""
        if value in {"ON", "BOTH"}:
            self.swresohoriz = True
            self.swresospd = True
            self.swresohdg = True
            self.swresovert = False
        elif value in {"OFF", "OF", "NONE"}:
            self.swresohoriz = False
            self.swresospd = False
            self.swresohdg = False
        elif value == "SPD":
            self.swresohoriz = True
            self.swresospd = True
            self.swresohdg = False
            self.swresovert = False
        else:
            self.swresohoriz = True
            self.swresospd = False
            self.swresohdg = True
            self.swresovert = False
        return Ok(f"Horizontal resolution method set to {value}")

    def vertical_method_status(self) -> Result[str, str]:
        """Show MVP vertical resolution limitations."""
        return Ok(
            "RMETHV [ON / V/S / OFF / NONE]"
            f"\nVertical resolution limitation is currently {'ON' if self.swresovert else 'OFF'}"
        )

    def configure_vertical_method(self, value: VerticalResolutionMethod) -> Result[str, str]:
        """Configure MVP vertical resolution limitations."""
        if value in {"ON", "V/S"}:
            self.swresovert = True
            self.swresohoriz = False
            self.swresospd = False
            self.swresohdg = False
        else:
            # Do NOT swtich off self.swresohoriz if value == OFF
            self.swresovert = False
        return Ok(f"Vertical resolution method set to {value}")

    class PriorityResolution(NamedTuple):
        ownship: q.VelocityMps[np.ndarray]
        intruder: q.VelocityMps[np.ndarray]

    def applyprio(
        self,
        dv_mvp: q.VelocityMps[np.ndarray],
        dv1: q.VelocityMps[np.ndarray],
        dv2: q.VelocityMps[np.ndarray],
        vs1: q.VerticalRateMps[float],
        vs2: q.VerticalRateMps[float],
    ) -> PriorityResolution:
        """Apply the desired priority setting to the resolution.

        Distributes the pairwise MVP resolution vector (east, north, up) over
        the two aircraft of a conflict according to the selected priority code.
        The input vector may be modified in place. Cruising
        aircraft (|vs| < 0.1 m/s) or climbing/descending aircraft get right
        of way depending on the code; the "LAY" codes additionally force
        horizontal-only resolutions by zeroing the vertical component.

        Args:
            dv_mvp: Pairwise resolution velocity ordered east/north/up; may be modified in place.
            dv1: Accumulated resolution vector for aircraft 1.
            dv2: Accumulated resolution vector for aircraft 2.
            vs1: Vertical rate of aircraft 1, used to classify cruise versus climb/descent.
            vs2: Vertical rate of aircraft 2, used to classify cruise versus climb/descent.
        """

        # Primary Free Flight prio rules (no priority)
        if self.priority_code is PriorityCode.FF1:
            # since cooperative, the vertical resolution component can be halved, and then dv_mvp can be added
            dv_mvp[2] = dv_mvp[2] / 2.0
            dv1 = dv1 - dv_mvp
            dv2 = dv2 + dv_mvp

        # Secondary Free Flight (Cruising aircraft has priority, combined resolutions)
        if self.priority_code is PriorityCode.FF2:
            dv_mvp[2] = dv_mvp[2] / 2.0
            if abs(vs1) < _CRUISE_VERTICAL_RATE and abs(vs2) > _CRUISE_VERTICAL_RATE:
                dv2 = dv2 + dv_mvp
            elif abs(vs2) < _CRUISE_VERTICAL_RATE and abs(vs1) > _CRUISE_VERTICAL_RATE:
                dv1 = dv1 - dv_mvp
            else:
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        # Tertiary Free Flight (Climbing/descending aircraft have priority and crusing solves with horizontal resolutions)
        elif self.priority_code is PriorityCode.FF3:
            if abs(vs1) < _CRUISE_VERTICAL_RATE and abs(vs2) > _CRUISE_VERTICAL_RATE:
                dv_mvp[2] = 0.0
                dv1 = dv1 - dv_mvp
            elif abs(vs2) < _CRUISE_VERTICAL_RATE and abs(vs1) > _CRUISE_VERTICAL_RATE:
                dv_mvp[2] = 0.0
                dv2 = dv2 + dv_mvp
            else:
                dv_mvp[2] = dv_mvp[2] / 2.0
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        # Primary Layers (Cruising aircraft has priority and clmibing/descending solves. All conflicts solved horizontally)
        elif self.priority_code is PriorityCode.LAY1:
            dv_mvp[2] = 0.0
            if abs(vs1) < _CRUISE_VERTICAL_RATE and abs(vs2) > _CRUISE_VERTICAL_RATE:
                dv2 = dv2 + dv_mvp
            elif abs(vs2) < _CRUISE_VERTICAL_RATE and abs(vs1) > _CRUISE_VERTICAL_RATE:
                dv1 = dv1 - dv_mvp
            else:
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        # Secondary Layers (Climbing/descending aircraft has priority and cruising solves. All conflicts solved horizontally)
        elif self.priority_code is PriorityCode.LAY2:
            dv_mvp[2] = 0.0
            if abs(vs1) < _CRUISE_VERTICAL_RATE and abs(vs2) > _CRUISE_VERTICAL_RATE:
                dv1 = dv1 - dv_mvp
            elif abs(vs2) < _CRUISE_VERTICAL_RATE and abs(vs1) > _CRUISE_VERTICAL_RATE:
                dv2 = dv2 + dv_mvp
            else:
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        return self.PriorityResolution(dv1, dv2)

    def resolve(
        self, conf: ConflictDetection, ownship: Traffic, intruder: Traffic
    ) -> ConflictResolution.ResolutionAdvisories:
        """Resolve all current conflicts.

        Loops over all detected conflict pairs, computes the MVP resolution
        vector for each with [`MVP`][..MVP], and accumulates the vectors per
        aircraft (applying priority rules and the NORESO/RESOOFF opt-outs).
        The summed velocity change is added to the current velocity vector
        and converted back to advisories, honouring the horizontal/vertical
        manoeuvre limitations and the aircraft performance envelope. The
        altitude advisory is chosen such that the aircraft does not climb or
        descend longer than needed if the autopilot level-off altitude also
        resolves the conflict.
        """
        dv = np.zeros((ownship.ntraf, 3))

        # Time needed to resolve vertically exists only for aircraft that have
        # a current conflict resolution.
        timesolveV = np.zeros(ownship.ntraf)
        has_resolution_time = np.zeros(ownship.ntraf, dtype=bool)

        for conflict, qdr, dist, tcpa, tLOS in zip(
            conf.confpairs, conf.qdr, conf.dist, conf.tcpa, conf.tLOS, strict=False
        ):
            idx1 = ownship.callsign.index(conflict.ownship)
            idx2 = intruder.callsign.index(conflict.intruder)

            # Because ADSB is ON, this is done for each aircraft separately.
            pair_resolution = self.MVP(ownship, intruder, conf, qdr, dist, tcpa, tLOS, idx1, idx2)
            dv_mvp = pair_resolution.velocity_delta
            if has_resolution_time[idx1]:
                timesolveV[idx1] = min(timesolveV[idx1], pair_resolution.vertical_time)
            else:
                timesolveV[idx1] = pair_resolution.vertical_time
                has_resolution_time[idx1] = True

            if self.priority_code is not None:
                priority = self.applyprio(
                    dv_mvp, dv[idx1], dv[idx2], ownship.vs[idx1], intruder.vs[idx2]
                )
                dv[idx1] = priority.ownship
            else:
                dv_mvp[2] = 0.5 * dv_mvp[2]
                dv[idx1] = dv[idx1] - dv_mvp

            # Check the noreso aircraft. Nobody avoids noreso aircraft.
            # But noreso aircraft will avoid other aircraft
            if self.noresoac[idx2]:
                dv[idx1] = dv[idx1] + dv_mvp

            if self.resooffac[idx1]:
                dv[idx1] = 0.0

        dv = np.transpose(dv)

        v = np.array([ownship.gseast, ownship.gsnorth, ownship.vs])

        newv = v + dv

        if self.swresohoriz:  # horizontal resolutions
            if self.swresospd and not self.swresohdg:  # SPD only
                newtrack = ownship.trk
                newgs = np.sqrt(newv[0, :] ** 2 + newv[1, :] ** 2)
                newvs = ownship.vs
            elif self.swresohdg and not self.swresospd:  # HDG only
                newtrack = np.degrees(np.arctan2(newv[0, :], newv[1, :])) % 360
                newgs = ownship.gs
                newvs = ownship.vs
            else:  # SPD + HDG
                newtrack = np.degrees(np.arctan2(newv[0, :], newv[1, :])) % 360
                newgs = np.sqrt(newv[0, :] ** 2 + newv[1, :] ** 2)
                newvs = ownship.vs
        elif self.swresovert:  # vertical resolutions
            newtrack = ownship.trk
            newgs = ownship.gs
            newvs = newv[2, :]
        else:  # horizontal + vertical
            newtrack = np.degrees(np.arctan2(newv[0, :], newv[1, :])) % 360
            newgs = np.sqrt(newv[0, :] ** 2 + newv[1, :] ** 2)
            newvs = newv[2, :]

        newgscapped = np.maximum(ownship.perf.vmin, np.minimum(ownship.perf.vmax, newgs))

        vscapped = np.maximum(ownship.perf.vsmin, np.minimum(ownship.perf.vsmax, newvs))

        # Calculate if Autopilot selected altitude should be followed. This avoids ASAS from
        # climbing or descending longer than it needs to if the autopilot leveloff
        # altitude also resolves the conflict. Because asasalttemp is calculated using
        # the time to resolve, it may result in climbing or descending more than the selected
        # altitude.
        alt = np.array(ownship.selalt, copy=True)
        asasalttemp = np.array(ownship.selalt, copy=True)
        asasalttemp[has_resolution_time] = (
            vscapped[has_resolution_time] * timesolveV[has_resolution_time]
            + ownship.alt[has_resolution_time]
        )
        signdvs = np.sign(vscapped - ownship.ap.vs * np.sign(ownship.selalt - ownship.alt))
        signalt = np.sign(asasalttemp - ownship.selalt)
        use_resolution_altitude = has_resolution_time & np.logical_or(
            signdvs == 0, signdvs == signalt
        )
        alt[use_resolution_altitude] = asasalttemp[use_resolution_altitude]

        # Update the ASAS altitude only where a vertical resolution exists
        # within the conflict look-ahead time.
        altCondition = (
            has_resolution_time & (timesolveV < conf.dtlookahead) & (np.abs(dv[2, :]) > 0.0)
        )
        alt[altCondition] = asasalttemp[altCondition]

        # If resolutions are limited in the horizontal direction, then asasalt should
        # be equal to auto pilot alt (aalt). This is to prevent a new asasalt being computed
        # using the auto pilot vertical speed (ownship.avs) using the code in line 106 (asasalttemp) when only
        # horizontal resolutions are allowed.
        alt = alt * (1 - self.swresohoriz) + ownship.selalt * self.swresohoriz
        return self.ResolutionAdvisories(newtrack, newgscapped, vscapped, alt)

    class MvpResolution(NamedTuple):
        velocity_delta: q.VelocityMps[np.ndarray]
        """Resolution velocity change ordered east/north/up."""
        vertical_time: q.DurationS[float]
        """Time needed to resolve the conflict vertically."""

    def MVP(
        self,
        ownship: Traffic,
        intruder: Traffic,
        conf: ConflictDetection,
        qdr: q.BearingDeg[float],
        dist: q.DistanceM[float],
        tcpa: q.DurationS[float],
        tLOS: q.DurationS[float],
        idx1: AircraftIndex,
        idx2: AircraftIndex,
    ) -> MvpResolution:
        """Modified Voltage Potential (MVP) resolution method.

        Computes the velocity change that displaces the predicted closest
        point of approach (CPA) of a conflict pair to the edge of the
        resolution zone (protected zone scaled by `resofach`/`resofacv`).
        Horizontally, the intrusion at CPA is divided by the time to CPA to
        obtain the required speed change along the CPA displacement
        direction; a geometric correction is applied when the intruder is
        still outside the protected zone so the resolution does not graze
        the zone. Vertically, the intrusion is resolved within the time the
        pair needs to cross vertically (or by time of LoS for level pairs),
        reducing the climb/descent rate of the faster-climbing aircraft.
        Head-on encounters are given a small artificial CPA offset to avoid
        division by zero.

        Args:
            ownship: Traffic view containing the ownship state.
            intruder: Traffic view containing the intruder state.
            conf: Detection state providing protected-zone sizes and lookahead.
            qdr: Bearing from ownship to intruder.
            dist: Current horizontal separation of the pair.
            tcpa: Signed time to horizontal closest point of approach.
            tLOS: Signed time until the combined loss-of-separation interval begins.
            idx1: Ownship aircraft index.
            idx2: Intruder aircraft index.
        """
        rpz_m = np.max(conf.rpz[[idx1, idx2]] * self.resofach)
        hpz_m = np.max(conf.hpz[[idx1, idx2]] * self.resofacv)
        dtlook = conf.dtlookahead[idx1]
        qdr = np.radians(qdr)

        drel = np.array(
            [
                np.sin(qdr) * dist,
                np.cos(qdr) * dist,
                intruder.alt[idx2] - ownship.alt[idx1],
            ]
        )

        v1 = np.array([ownship.gseast[idx1], ownship.gsnorth[idx1], ownship.vs[idx1]])
        v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2], intruder.vs[idx2]])
        vrel = v2 - v1

        dcpa = drel + vrel * tcpa
        dabsH = np.sqrt(dcpa[0] * dcpa[0] + dcpa[1] * dcpa[1])

        iH = rpz_m - dabsH

        # Exception handlers for head-on conflicts
        # This is done to prevent division by zero in the next step
        if dabsH <= _HEAD_ON_CPA_FLOOR:
            dabsH = _HEAD_ON_CPA_FLOOR
            dcpa[0] = drel[1] / dist * dabsH
            dcpa[1] = -drel[0] / dist * dabsH

        # If intruder is outside the ownship PZ, then apply extra factor
        # to make sure that resolution does not graze IPZ
        if rpz_m < dist and dabsH < dist:
            # abs(tcpa) because it bcomes negative during intrusion.
            erratum = np.cos(np.arcsin(rpz_m / dist) - np.arcsin(dabsH / dist))
            dv1 = ((rpz_m / erratum - dabsH) * dcpa[0]) / (abs(tcpa) * dabsH)
            dv2 = ((rpz_m / erratum - dabsH) * dcpa[1]) / (abs(tcpa) * dabsH)
        else:
            dv1 = (iH * dcpa[0]) / (abs(tcpa) * dabsH)
            dv2 = (iH * dcpa[1]) / (abs(tcpa) * dabsH)

        # Amount of vertical intrusion dependent on vertical relative velocity
        iV = hpz_m if abs(vrel[2]) > 0.0 else hpz_m - abs(drel[2])

        tsolV = abs(drel[2] / vrel[2]) if abs(vrel[2]) > 0.0 else tLOS

        # If the time to solve the conflict vertically is longer than the look-ahead time,
        # because the the relative vertical speed is very small, then solve the intrusion
        # within tinconf
        if tsolV > dtlook:
            tsolV = tLOS
            iV = hpz_m

        # The direction of the vertical resolution is such that the aircraft with
        # higher climb/decent rate reduces their climb/decent rate
        dv3 = np.where(abs(vrel[2]) > 0.0, (iV / tsolV) * (-vrel[2] / abs(vrel[2])), (iV / tsolV))

        # It is necessary to cap dv3 to prevent that a vertical conflict
        # is solved in 1 timestep, leading to a vertical separation that is too
        # high (high vs assumed in traf). If vertical dynamics are included to
        # aircraft  model in traffic.py, the below three lines should be deleted.

        dv = np.array([dv1, dv2, dv3])

        return self.MvpResolution(dv, tsolV)
