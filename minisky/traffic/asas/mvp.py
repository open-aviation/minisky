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

from typing import Any

import numpy as np

from minisky.traffic.asas import ConflictResolution


class MVP(ConflictResolution):
    """Conflict resolution using the Modified Voltage Potential Method.

    For each detected conflict pair, :meth:`MVP` computes a repulsive
    velocity-change vector that pushes the closest point of approach out of
    the resolution zone (the protected zone scaled by ``resofach`` and
    ``resofacv``). :meth:`resolve` accumulates these vectors for all
    conflicts of each aircraft, adds them to the current velocity, and
    converts the result into track, ground speed, vertical speed, and
    altitude advisories, capped to the aircraft performance envelope.

    Selected via the stack command ``RESO MVP``. Resolution manoeuvres can be
    restricted with RMETHH (horizontal: heading and/or speed) and RMETHV
    (vertical speed only).

    Attributes:
        swresohoriz (bool): Limit resolutions to the horizontal plane.
        swresospd (bool): Use speed-only resolutions (with swresohoriz).
        swresohdg (bool): Use heading-only resolutions (with swresohoriz).
        swresovert (bool): Limit resolutions to the vertical direction.
    """

    def __init__(self) -> None:
        super().__init__()
        # [-] switch to limit resolution to the horizontal direction
        self.swresohoriz = True
        # [-] switch to use only speed resolutions (works with swresohoriz = True)
        self.swresospd = False
        # [-] switch to use only heading resolutions (works with swresohoriz = True)
        self.swresohdg = False
        # [-] switch to limit resolution to the vertical direction
        self.swresovert = False

    def setprio(self, flag=None, priocode="") -> "bool | tuple":
        """Set the prio switch and the type of prio.

        Implements the PRIORULES stack command for MVP. Validates the
        priority code against the codes supported by :meth:`applyprio`.

        Args:
            flag (bool): True to enable priority rules, False to disable.
                When None, the available priority codes are reported.
            priocode (str): One of "FF1", "FF2", "FF3", "LAY1", "LAY2".

        Returns:
            True on success, or (success (bool), message (str)) tuple.
        """
        if flag is None:
            return (
                True,
                "PRIORULES [ON/OFF] [PRIOCODE]"
                + "\nAvailable priority codes: "
                + "\n     FF1:  Free Flight Primary (No Prio) "
                + "\n     FF2:  Free Flight Secondary (Cruising has priority)"
                + "\n     FF3:  Free Flight Tertiary (Climbing/descending has priority)"
                + "\n     LAY1: Layers Primary (Cruising has priority + horizontal resolutions)"
                + "\n     LAY2: Layers Secondary (Climbing/descending has priority + horizontal resolutions)"
                + "\nPriority is currently "
                + ("ON" if self.swprio else "OFF")
                + "\nPriority code is currently: "
                + str(self.priocode),
            )
        options = ["FF1", "FF2", "FF3", "LAY1", "LAY2"]
        if priocode not in options:
            return False, "Priority code Not Understood. Available Options: " + str(options)
        return super().setprio(flag, priocode)

    def setresometh(self, value: "txt" = "") -> "tuple | None":
        """Processes the RMETHH command. Sets swresovert = False.

        Selects which horizontal degrees of freedom MVP may use for
        resolutions: both heading and speed (ON/BOTH), speed only (SPD),
        heading only (HDG), or none (OFF/NONE).

        Args:
            value (str): One of "BOTH", "SPD", "HDG", "NONE", "ON", "OFF",
                "OF". When empty, the current settings are reported.

        Returns:
            tuple or None: (success (bool), message (str)) when reporting or
                on invalid input; None after applying a valid setting.
        """
        # Acceptable arguments for this command
        options = ["BOTH", "SPD", "HDG", "NONE", "ON", "OFF", "OF"]
        if not value:
            return (
                True,
                "RMETHH [ON / BOTH / OFF / NONE / SPD / HDG]"
                + "\nHorizontal resolution limitation is currently "
                + ("ON" if self.swresohoriz else "OFF")
                + "\nSpeed resolution limitation is currently "
                + ("ON" if self.swresospd else "OFF")
                + "\nHeading resolution limitation is currently "
                + ("ON" if self.swresohdg else "OFF"),
            )
        if value not in options:
            return (
                False,
                "RMETH Not Understood" + "\nRMETHH [ON / BOTH / OFF / NONE / SPD / HDG]",
            )
        else:
            if value == "ON" or value == "BOTH":
                self.swresohoriz = True
                self.swresospd = True
                self.swresohdg = True
                self.swresovert = False
            elif value == "OFF" or value == "OF" or value == "NONE":
                # Do NOT swtich off self.swresovert if value == OFF
                self.swresohoriz = False
                self.swresospd = False
                self.swresohdg = False
            elif value == "SPD":
                self.swresohoriz = True
                self.swresospd = True
                self.swresohdg = False
                self.swresovert = False
            elif value == "HDG":
                self.swresohoriz = True
                self.swresospd = False
                self.swresohdg = True
                self.swresovert = False

    def setresometv(self, value: "txt" = "") -> "tuple | None":
        """Processes the RMETHV command. Sets swresohoriz = False.

        Enables (ON/"V/S") or disables (OFF/NONE) vertical-speed-only
        resolutions; enabling it switches off all horizontal limitations.

        Args:
            value (str): One of "ON", "V/S", "OFF", "OF", "NONE". When empty,
                the current setting is reported.

        Returns:
            tuple or None: (success (bool), message (str)) when reporting or
                on invalid input; None after applying a valid setting.
        """
        # Acceptable arguments for this command
        options = ["NONE", "ON", "OFF", "OF", "V/S"]
        if not value:
            return (
                True,
                "RMETHV [ON / V/S / OFF / NONE]"
                + "\nVertical resolution limitation is currently "
                + ("ON" if self.swresovert else "OFF"),
            )
        if value not in options:
            return (
                False,
                f"RMETHV '{value}' Not Understood\nRMETHV [ON / V/S / OFF / NONE]",
            )

        if value == "ON" or value == "V/S":
            self.swresovert = True
            self.swresohoriz = False
            self.swresospd = False
            self.swresohdg = False
        elif value == "OFF" or value == "OF" or value == "NONE":
            # Do NOT swtich off self.swresohoriz if value == OFF
            self.swresovert = False

    def applyprio(
        self,
        dv_mvp: np.ndarray,
        dv1: np.ndarray,
        dv2: np.ndarray,
        vs1: float,
        vs2: float,
    ) -> tuple:
        """Apply the desired priority setting to the resolution.

        Distributes the pairwise MVP resolution vector over the two aircraft
        of a conflict according to the selected priority code. Cruising
        aircraft (|vs| < 0.1 m/s) or climbing/descending aircraft get right
        of way depending on the code; the "LAY" codes additionally force
        horizontal-only resolutions by zeroing the vertical component.

        Args:
            dv_mvp (ndarray): Pairwise MVP resolution velocity vector
                (east, north, up) [m/s]; may be modified in place.
            dv1 (ndarray): Accumulated resolution vector of aircraft 1 [m/s].
            dv2 (ndarray): Accumulated resolution vector of aircraft 2 [m/s].
            vs1 (float): Vertical speed of aircraft 1 [m/s].
            vs2 (float): Vertical speed of aircraft 2 [m/s].

        Returns:
            tuple: Updated (dv1, dv2) resolution vectors [m/s].
        """

        # Primary Free Flight prio rules (no priority)
        if self.priocode == "FF1":
            # since cooperative, the vertical resolution component can be halved, and then dv_mvp can be added
            dv_mvp[2] = dv_mvp[2] / 2.0
            dv1 = dv1 - dv_mvp
            dv2 = dv2 + dv_mvp

        # Secondary Free Flight (Cruising aircraft has priority, combined resolutions)
        if self.priocode == "FF2":
            # since cooperative, the vertical resolution component can be halved, and then dv_mvp can be added
            dv_mvp[2] = dv_mvp[2] / 2.0
            # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 2 solves conflict
            if abs(vs1) < 0.1 and abs(vs2) > 0.1:
                dv2 = dv2 + dv_mvp
            # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 1 solves conflict
            elif abs(vs2) < 0.1 and abs(vs1) > 0.1:
                dv1 = dv1 - dv_mvp
            else:  # both are climbing/descending/cruising -> both aircraft solves the conflict
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        # Tertiary Free Flight (Climbing/descending aircraft have priority and crusing solves with horizontal resolutions)
        elif self.priocode == "FF3":
            # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 1 solves conflict horizontally
            if abs(vs1) < 0.1 and abs(vs2) > 0.1:
                dv_mvp[2] = 0.0
                dv1 = dv1 - dv_mvp
            # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 2 solves conflict horizontally
            elif abs(vs2) < 0.1 and abs(vs1) > 0.1:
                dv_mvp[2] = 0.0
                dv2 = dv2 + dv_mvp
            else:  # both are climbing/descending/cruising -> both aircraft solves the conflict, combined
                dv_mvp[2] = dv_mvp[2] / 2.0
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        # Primary Layers (Cruising aircraft has priority and clmibing/descending solves. All conflicts solved horizontally)
        elif self.priocode == "LAY1":
            dv_mvp[2] = 0.0
            # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 2 solves conflict horizontally
            if abs(vs1) < 0.1 and abs(vs2) > 0.1:
                dv2 = dv2 + dv_mvp
            # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 1 solves conflict horizontally
            elif abs(vs2) < 0.1 and abs(vs1) > 0.1:
                dv1 = dv1 - dv_mvp
            else:  # both are climbing/descending/cruising -> both aircraft solves the conflict horizontally
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        # Secondary Layers (Climbing/descending aircraft has priority and cruising solves. All conflicts solved horizontally)
        elif self.priocode == "LAY2":
            dv_mvp[2] = 0.0
            # If aircraft 1 is cruising, and aircraft 2 is climbing/descending -> aircraft 1 solves conflict horizontally
            if abs(vs1) < 0.1 and abs(vs2) > 0.1:
                dv1 = dv1 - dv_mvp
            # If aircraft 2 is cruising, and aircraft 1 is climbing -> aircraft 2 solves conflict horizontally
            elif abs(vs2) < 0.1 and abs(vs1) > 0.1:
                dv2 = dv2 + dv_mvp
            else:  # both are climbing/descending/cruising -> both aircraft solves the conflic horizontally
                dv1 = dv1 - dv_mvp
                dv2 = dv2 + dv_mvp

        return dv1, dv2

    def resolve(self, conf: Any, ownship: Any, intruder: Any) -> tuple:
        """Resolve all current conflicts.

        Loops over all detected conflict pairs, computes the MVP resolution
        vector for each with :meth:`MVP`, and accumulates the vectors per
        aircraft (applying priority rules and the NORESO/RESOOFF opt-outs).
        The summed velocity change is added to the current velocity vector
        and converted back to advisories, honouring the horizontal/vertical
        manoeuvre limitations and the aircraft performance envelope. The
        altitude advisory is chosen such that the aircraft does not climb or
        descend longer than needed if the autopilot level-off altitude also
        resolves the conflict.

        Args:
            conf: The ConflictDetection instance with the current conflicts.
            ownship: Traffic object with ownship states.
            intruder: Traffic object with intruder states.

        Returns:
            tuple: Per-aircraft advisories:
                - newtrack (ndarray): Resolution track [deg].
                - newgscapped (ndarray): Resolution ground speed, capped to
                  the performance envelope [m/s].
                - vscapped (ndarray): Resolution vertical speed, capped to
                  the performance envelope [m/s].
                - alt (ndarray): Resolution altitude [m].
        """
        # Initialize an array to store the resolution velocity vector for all A/C
        dv = np.zeros((ownship.ntraf, 3))

        # Initialize an array to store time needed to resolve vertically
        timesolveV = np.ones(ownship.ntraf) * 1e9

        # Call MVP function to resolve conflicts-----------------------------------
        for (ac1, ac2), qdr, dist, tcpa, tLOS in zip(
            conf.confpairs, conf.qdr, conf.dist, conf.tcpa, conf.tLOS, strict=False
        ):
            idx1 = ownship.callsign.index(ac1)
            idx2 = intruder.callsign.index(ac2)

            # If A/C indexes are found, then apply MVP on this conflict pair
            # Because ADSB is ON, this is done for each aircraft separately
            if idx1 > -1 and idx2 > -1:
                dv_mvp, tsolV = self.MVP(ownship, intruder, conf, qdr, dist, tcpa, tLOS, idx1, idx2)
                if tsolV < timesolveV[idx1]:
                    timesolveV[idx1] = tsolV

                # Use priority rules if activated
                if self.swprio:
                    dv[idx1], _ = self.applyprio(
                        dv_mvp, dv[idx1], dv[idx2], ownship.vs[idx1], intruder.vs[idx2]
                    )
                else:
                    # since cooperative, the vertical resolution component can be halved, and then dv_mvp can be added
                    dv_mvp[2] = 0.5 * dv_mvp[2]
                    dv[idx1] = dv[idx1] - dv_mvp

                # Check the noreso aircraft. Nobody avoids noreso aircraft.
                # But noreso aircraft will avoid other aircraft
                if self.noresoac[idx2]:
                    dv[idx1] = dv[idx1] + dv_mvp

                # Check the resooff aircraft. These aircraft will not do resolutions.
                if self.resooffac[idx1]:
                    dv[idx1] = 0.0

        # Determine new speed and limit resolution direction for all aicraft-------

        # Resolution vector for all aircraft, cartesian coordinates
        dv = np.transpose(dv)

        # The old speed vector, cartesian coordinates
        v = np.array([ownship.gseast, ownship.gsnorth, ownship.vs])

        # The new speed vector, cartesian coordinates
        newv = v + dv

        # Limit resolution direction if required-----------------------------------

        # Compute new speed vector in polar coordinates based on desired resolution
        if self.swresohoriz:  # horizontal resolutions
            if self.swresospd and not self.swresohdg:  # SPD only
                newtrack = ownship.trk
                newgs = np.sqrt(newv[0, :] ** 2 + newv[1, :] ** 2)
                newvs = ownship.vs
            elif self.swresohdg and not self.swresospd:  # HDG only
                newtrack = (np.arctan2(newv[0, :], newv[1, :]) * 180 / np.pi) % 360
                newgs = ownship.gs
                newvs = ownship.vs
            else:  # SPD + HDG
                newtrack = (np.arctan2(newv[0, :], newv[1, :]) * 180 / np.pi) % 360
                newgs = np.sqrt(newv[0, :] ** 2 + newv[1, :] ** 2)
                newvs = ownship.vs
        elif self.swresovert:  # vertical resolutions
            newtrack = ownship.trk
            newgs = ownship.gs
            newvs = newv[2, :]
        else:  # horizontal + vertical
            newtrack = (np.arctan2(newv[0, :], newv[1, :]) * 180 / np.pi) % 360
            newgs = np.sqrt(newv[0, :] ** 2 + newv[1, :] ** 2)
            newvs = newv[2, :]

        # Determine ASAS module commands for all aircraft--------------------------

        # Cap the velocity
        newgscapped = np.maximum(ownship.perf.vmin, np.minimum(ownship.perf.vmax, newgs))

        # Cap the vertical speed
        vscapped = np.maximum(ownship.perf.vsmin, np.minimum(ownship.perf.vsmax, newvs))

        # Calculate if Autopilot selected altitude should be followed. This avoids ASAS from
        # climbing or descending longer than it needs to if the autopilot leveloff
        # altitude also resolves the conflict. Because asasalttemp is calculated using
        # the time to resolve, it may result in climbing or descending more than the selected
        # altitude.
        asasalttemp = vscapped * timesolveV + ownship.alt
        signdvs = np.sign(vscapped - ownship.ap.vs * np.sign(ownship.selalt - ownship.alt))
        signalt = np.sign(asasalttemp - ownship.selalt)
        alt = np.where(np.logical_or(signdvs == 0, signdvs == signalt), asasalttemp, ownship.selalt)

        # To compute asas alt, timesolveV is used. timesolveV is a really big value (1e9)
        # when there is no conflict. Therefore asas alt is only updated when its
        # value is less than the look-ahead time, because for those aircraft are in conflict
        altCondition = np.logical_and(timesolveV < conf.dtlookahead, np.abs(dv[2, :]) > 0.0)
        alt[altCondition] = asasalttemp[altCondition]

        # If resolutions are limited in the horizontal direction, then asasalt should
        # be equal to auto pilot alt (aalt). This is to prevent a new asasalt being computed
        # using the auto pilot vertical speed (ownship.avs) using the code in line 106 (asasalttemp) when only
        # horizontal resolutions are allowed.
        alt = alt * (1 - self.swresohoriz) + ownship.selalt * self.swresohoriz
        return newtrack, newgscapped, vscapped, alt

    def MVP(
        self,
        ownship: Any,
        intruder: Any,
        conf: Any,
        qdr,
        dist: float,
        tcpa: float,
        tLOS: float,
        idx1: int,
        idx2: int,
    ) -> tuple:
        """Modified Voltage Potential (MVP) resolution method.

        Computes the velocity change that displaces the predicted closest
        point of approach (CPA) of one conflict pair to the edge of the
        resolution zone (protected zone scaled by ``resofach``/``resofacv``).
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
            ownship: Traffic object with ownship states.
            intruder: Traffic object with intruder states.
            conf: The ConflictDetection instance (for rpz, hpz, dtlookahead).
            qdr (float): Bearing from ownship to intruder [deg].
            dist (float): Current horizontal distance between the pair [m].
            tcpa (float): Time to closest point of approach [s].
            tLOS (float): Time until loss of separation starts [s].
            idx1 (int): Index of the ownship aircraft.
            idx2 (int): Index of the intruder aircraft.

        Returns:
            tuple: (dv, tsolV) where dv is the resolution velocity change
                (east, north, up) [m/s] and tsolV the time needed to resolve
                the conflict vertically [s].
        """
        # Preliminary calculations-------------------------------------------------
        # Determine largest RPZ and HPZ of the conflict pair, use lookahead of ownship
        rpz_m = np.max(conf.rpz[[idx1, idx2]] * self.resofach)
        hpz_m = np.max(conf.hpz[[idx1, idx2]] * self.resofacv)
        dtlook = conf.dtlookahead[idx1]
        # Convert qdr from degrees to radians
        qdr = np.radians(qdr)

        # Relative position vector between id1 and id2
        drel = np.array(
            [
                np.sin(qdr) * dist,
                np.cos(qdr) * dist,
                intruder.alt[idx2] - ownship.alt[idx1],
            ]
        )

        # Write velocities as vectors and find relative velocity vector
        v1 = np.array([ownship.gseast[idx1], ownship.gsnorth[idx1], ownship.vs[idx1]])
        v2 = np.array([intruder.gseast[idx2], intruder.gsnorth[idx2], intruder.vs[idx2]])
        vrel = v2 - v1

        # Horizontal resolution----------------------------------------------------

        # Find horizontal distance at the tcpa (min horizontal distance)
        dcpa = drel + vrel * tcpa
        dabsH = np.sqrt(dcpa[0] * dcpa[0] + dcpa[1] * dcpa[1])

        # Compute horizontal intrusion
        iH = rpz_m - dabsH

        # Exception handlers for head-on conflicts
        # This is done to prevent division by zero in the next step
        if dabsH <= 10.0:
            dabsH = 10.0
            dcpa[0] = drel[1] / dist * dabsH
            dcpa[1] = -drel[0] / dist * dabsH

        # If intruder is outside the ownship PZ, then apply extra factor
        # to make sure that resolution does not graze IPZ
        if rpz_m < dist and dabsH < dist:
            # Compute the resolution velocity vector in horizontal direction.
            # abs(tcpa) because it bcomes negative during intrusion.
            erratum = np.cos(np.arcsin(rpz_m / dist) - np.arcsin(dabsH / dist))
            dv1 = ((rpz_m / erratum - dabsH) * dcpa[0]) / (abs(tcpa) * dabsH)
            dv2 = ((rpz_m / erratum - dabsH) * dcpa[1]) / (abs(tcpa) * dabsH)
        else:
            dv1 = (iH * dcpa[0]) / (abs(tcpa) * dabsH)
            dv2 = (iH * dcpa[1]) / (abs(tcpa) * dabsH)

        # Vertical resolution------------------------------------------------------

        # Compute the  vertical intrusion
        # Amount of vertical intrusion dependent on vertical relative velocity
        iV = hpz_m if abs(vrel[2]) > 0.0 else hpz_m - abs(drel[2])

        # Get the time to solve the conflict vertically - tsolveV
        tsolV = abs(drel[2] / vrel[2]) if abs(vrel[2]) > 0.0 else tLOS

        # If the time to solve the conflict vertically is longer than the look-ahead time,
        # because the the relative vertical speed is very small, then solve the intrusion
        # within tinconf
        if tsolV > dtlook:
            tsolV = tLOS
            iV = hpz_m

        # Compute the resolution velocity vector in the vertical direction
        # The direction of the vertical resolution is such that the aircraft with
        # higher climb/decent rate reduces their climb/decent rate
        dv3 = np.where(abs(vrel[2]) > 0.0, (iV / tsolV) * (-vrel[2] / abs(vrel[2])), (iV / tsolV))

        # It is necessary to cap dv3 to prevent that a vertical conflict
        # is solved in 1 timestep, leading to a vertical separation that is too
        # high (high vs assumed in traf). If vertical dynamics are included to
        # aircraft  model in traffic.py, the below three lines should be deleted.
        #    mindv3 = -400*fpm# ~ 2.016 [m/s]
        #    maxdv3 = 400*fpm
        #    dv3 = np.maximum(mindv3,np.minimum(maxdv3,dv3))

        # Combine resolutions------------------------------------------------------

        # combine the dv components
        dv = np.array([dv1, dv2, dv3])

        return dv, tsolV
