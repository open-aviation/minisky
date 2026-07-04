"""This module provides the Conflict Detection base class.

Conflict detection in MiniSky is pairwise and state-based: at every update the
current position and velocity of each aircraft (the ownship) is linearly
extrapolated and compared against every other aircraft (the intruder). A
conflict is flagged when the extrapolated trajectories penetrate each other's
cylindrical protected zone (radius ``rpz``, half-height ``hpz``) within the
lookahead time ``dtlookahead``. A loss of separation (LoS) is flagged when the
protected zone is already penetrated at the current time.

All geometry is computed with vectorised numpy matrix operations over all
aircraft pairs simultaneously. Internally SI units are used (m, m/s, s);
user-facing (stack command) arguments are in aviation units (NM, ft).
"""

from typing import Any

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools import geo
from minisky.tools.aero import ft, nm


class ConflictDetection(TrafficArrays):
    """Base class for Conflict Detection implementations.

    Implements state-based conflict detection: for each aircraft pair the
    closest point of approach (CPA) is computed from relative position and
    ground velocity. A pair is in conflict when the horizontal distance at CPA
    is smaller than the protected zone radius, the vertical crossing of the
    protected zone disk overlaps in time with the horizontal intrusion, and
    the conflict starts within the lookahead time.

    The result of each update is stored both as pairwise lists (``confpairs``,
    ``lospairs`` and the per-conflict geometry arrays) and as per-aircraft
    arrays (``inconf``, ``tcpamax``). Separation minima and lookahead time can
    be set globally or per aircraft.

    Attributes:
        rpz_def (float): Default horizontal separation minimum (PZ radius) [m].
        hpz_def (float): Default vertical separation minimum (half PZ height) [m].
        dtlookahead_def (float): Default conflict detection lookahead time [s].
        dtnolook_def (float): Default detection hold-off interval [s].
        activate (bool): Whether conflict detection is switched on.
        confpairs (list): Callsign pairs in conflict this timestep; contains
            both (a, b) and (b, a).
        lospairs (list): Callsign pairs in loss of separation this timestep.
        confpairs_unique (set): Unique (frozenset) conflict pairs this timestep.
        lospairs_unique (set): Unique (frozenset) LoS pairs this timestep.
        confpairs_all (list): All unique conflict pairs since simulation start.
        lospairs_all (list): All unique LoS pairs since simulation start.
        qdr (ndarray): Bearing from ownship to intruder per conflict [deg].
        dist (ndarray): Current horizontal distance per conflict [m].
        dcpa (ndarray): Predicted horizontal distance at CPA per conflict [m].
        tcpa (ndarray): Time to closest point of approach per conflict [s].
        tLOS (ndarray): Time until loss of separation starts per conflict [s].
        dalt (ndarray): Current altitude difference per conflict [m].
        inconf (ndarray): Per-aircraft flag, True when in at least one conflict [-].
        tcpamax (ndarray): Per-aircraft maximum time to CPA over its conflicts [s].
        rpz (ndarray): Per-aircraft horizontal separation minimum [m].
        hpz (ndarray): Per-aircraft vertical separation minimum [m].
        dtlookahead (ndarray): Per-aircraft lookahead time [s].
        dtnolook (ndarray): Per-aircraft detection hold-off interval [s].
    """

    def __init__(self) -> None:
        super().__init__()
        ## Default values
        # [m] Horizontal separation minimum for detection
        self.rpz_def = minisky.core.settings.asas_pzr * nm
        self.global_rpz = True
        # [m] Vertical separation minimum for detection
        self.hpz_def = minisky.core.settings.asas_pzh * ft
        self.global_hpz = True
        # [s] lookahead time
        self.dtlookahead_def = minisky.core.settings.asas_dtlookahead
        self.global_dtlook = True
        self.dtnolook_def = 0.0
        self.global_dtnolook = True
        self.activate = True

        # Conflicts and LoS detected in the current timestep (used for resolving)
        self.confpairs = []
        self.lospairs = []
        self.qdr = np.array([])
        self.dist = np.array([])
        self.dcpa = np.array([])
        self.tcpa = np.array([])
        self.tLOS = np.array([])
        self.dalt = np.array([])
        # Unique conflicts and LoS in the current timestep (a, b) = (b, a)
        self.confpairs_unique = set()
        self.lospairs_unique = set()

        # All conflicts and LoS since simt=0
        self.confpairs_all = []
        self.lospairs_all = []

        # Per-aircraft conflict data
        with self.settrafarrays():
            self.inconf = np.array([], dtype=bool)  # In-conflict flag
            self.tcpamax = np.array([])  # Maximum time to CPA for aircraft in conflict
            # [m] Horizontal separation minimum for detection
            self.rpz = np.array([])
            # [m] Vertical separation minimum for detection
            self.hpz = np.array([])
            # [s] lookahead time
            self.dtlookahead = np.array([])
            self.dtnolook = np.array([])

    def clearconfdb(self) -> None:
        """Clear the conflict database.

        Empties the pairwise conflict/LoS lists and geometry arrays of the
        current timestep and resets the per-aircraft conflict flags. The
        historic lists (``confpairs_all``, ``lospairs_all``) are kept.
        """
        self.confpairs_unique.clear()
        self.lospairs_unique.clear()
        self.confpairs.clear()
        self.lospairs.clear()
        self.qdr = np.array([])
        self.dist = np.array([])
        self.dcpa = np.array([])
        self.tcpa = np.array([])
        self.tLOS = np.array([])
        self.dalt = np.array([])
        self.inconf = np.zeros(minisky.traf.ntraf)
        self.tcpamax = np.zeros(minisky.traf.ntraf)

    def create(self, n: int) -> None:
        """Initialise per-aircraft detection parameters for new aircraft.

        Called by the traffic object when aircraft are created. Extends all
        per-aircraft arrays and fills the last ``n`` elements with the current
        default separation minima and lookahead times.

        Args:
            n (int): Number of newly created aircraft.
        """
        super().create(n)
        # Initialise values of own states
        self.rpz[-n:] = self.rpz_def
        self.hpz[-n:] = self.hpz_def
        self.dtlookahead[-n:] = self.dtlookahead_def
        self.dtnolook[-n:] = self.dtnolook_def

    def reset(self) -> None:
        """Reset the conflict detection state to defaults.

        Called on simulation reset: clears the conflict database and the
        historic conflict/LoS lists, and restores the default separation
        minima and lookahead times from the simulation settings.
        """
        super().reset()
        self.clearconfdb()
        self.confpairs_all.clear()
        self.lospairs_all.clear()
        self.rpz_def = minisky.core.settings.asas_pzr * nm
        self.hpz_def = (minisky.core.settings.asas_pzh - 1) * ft  # -1 for rounding margins
        self.dtlookahead_def = minisky.core.settings.asas_dtlookahead
        self.dtnolook_def = 0.0
        self.global_rpz = self.global_hpz = True
        self.global_dtlook = self.global_dtnolook = True

    def switch(self, name: "txt" = "ON") -> "tuple | None":
        """Turn Conflict Detection (CD) ON / OFF.

        Switching off also clears the current conflict database.

        Args:
            name (str): Either "ON" or "OFF".

        Returns:
            tuple: (success (bool), message (str)) for the command stack.

        Raises:
            AssertionError: If ``name`` is not "ON" or "OFF".
        """
        assert name in ["ON", "OFF"], f"Invalid CD method: {name}"

        if name == "OFF":
            self.clearconfdb()
            self.activate = False
            return True, "Conflict Detection turned off."

        if name == "ON":
            self.activate = True
            return True, "Conflict Detection is on."

    def setrpz(self, radius: float = -1.0, *acidx) -> tuple:
        """Set the horizontal separation distance (i.e., the radius of the
        protected zone) in nautical miles.

        Implements the ZONER stack command. When an absolute resolution zone
        radius was previously set (RSZONER), the resolution factor is rescaled
        so that the absolute resolution zone size is preserved.

        Args:
            radius (float): The protected zone radius [NM]. When negative
                (default), the current default radius is reported instead.
            *acidx: Aircraft index/indices or group. When not provided, the
                default PZ radius is changed. Otherwise the PZ radius for the
                passed aircraft is changed.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if radius < 0.0:
            return (
                True,
                f"ZONER [radius(nm), acid(s)/ac group]\nCurrent default PZ radius: {self.rpz_def / nm:.2f} NM",
            )
        if len(acidx) > 0:
            if isinstance(acidx[0], np.ndarray):
                acidx = acidx[0]
            self.rpz[acidx] = radius * nm
            self.global_rpz = False
            return True, f"Setting PZ radius to {radius} NM for {len(acidx)} aircraft"
        oldradius = self.rpz_def
        self.rpz_def = radius * nm
        if self.global_rpz:
            self.rpz[:] = self.rpz_def
        # Adjust factors for reso zone if those were set with an absolute value
        if not minisky.traf.cr.resorrelative:
            minisky.stack.stack(f"RSZONER {minisky.traf.cr.resofach * oldradius / nm}")
        return True, f"Setting default PZ radius to {radius} NM"

    def sethpz(self, height: float = -1.0, *acidx) -> tuple:
        """Set the vertical separation distance (i.e., half of the protected
        zone height) in feet.

        Implements the ZONEDH stack command. When an absolute resolution zone
        height was previously set (RSZONEDH), the resolution factor is
        rescaled so that the absolute resolution zone size is preserved.

        Args:
            height (float): The vertical separation height [ft]. When negative
                (default), the current default height is reported instead.
            *acidx: Aircraft index/indices or group. When not provided, the
                default PZ height is changed. Otherwise the PZ height for the
                passed aircraft is changed.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if height < 0.0:
            return (
                True,
                f"ZONEDH [height (ft), acid(s)/ac group]\nCurrent default PZ height: {self.hpz / ft:.2f} ft",
            )
        if len(acidx) > 0:
            if isinstance(acidx[0], np.ndarray):
                acidx = acidx[0]
            self.hpz[acidx] = height * ft
            self.global_hpz = False
            return True, f"Setting PZ height to {height} ft for {len(acidx)} aircraft"
        oldhpz = self.hpz_def
        self.hpz_def = height * ft
        if self.global_hpz:
            self.hpz[:] = self.hpz_def
        # Adjust factors for reso zone if those were set with an absolute value
        if not minisky.traf.cr.resodhrelative:
            minisky.stack.stack(f"RSZONEDH {minisky.traf.cr.resofacv * oldhpz / ft}")
        return True, f"Setting default PZ height to {height} ft"

    def setdtlook(self, time: "time" = -1.0, *acidx) -> tuple:
        """Set the lookahead time (in [hh:mm:]sec) for conflict detection.

        Implements the DTLOOK stack command.

        Args:
            time (float): Lookahead time [s]. When negative (default), the
                current default lookahead time is reported instead.
            *acidx: Aircraft index/indices or group. When not provided, the
                default lookahead time is changed. Otherwise the lookahead
                time for the passed aircraft is changed.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if time < 0.0:
            return True, f"DTLOOK[time]\nCurrent value: {self.dtlookahead_def: .1f} sec"
        if len(acidx) > 0:
            if isinstance(acidx[0], np.ndarray):
                acidx = acidx[0]
            self.dtlookahead[acidx] = time
            self.global_dtlook = False
            return True, f"Setting CD lookahead to {time} sec for {len(acidx)} aircraft"
        self.dtlookahead_def = time
        if self.global_dtlook:
            self.dtlookahead[:] = time
        return True, f"Setting default CD lookahead to {time} sec"

    def setdtnolook(self, time: "time" = -1.0, *acidx) -> tuple:
        """Set the interval (in [hh:mm:]sec) in which conflict detection
        is skipped after a conflict resolution.

        Implements the DTNOLOOK stack command.

        Args:
            time (float): No-look interval [s]. When negative (default), the
                current default no-look interval is reported instead.
            *acidx: Aircraft index/indices or group. When not provided, the
                default interval is changed. Otherwise the interval for the
                passed aircraft is changed.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if time < 0.0:
            return True, f"DTNOLOOK[time]\nCurrent value: {self.dtnolook_def: .1f} sec"
        if len(acidx) > 0:
            if isinstance(acidx[0], np.ndarray):
                acidx = acidx[0]
            self.dtnolook[acidx] = time
            self.global_dtnolook = False
            return True, f"Setting CD no-look to {time} sec for {len(acidx)} aircraft"
        self.dtnolook_def = time
        if self.global_dtnolook:
            self.dtnolook[:] = time
        return True, f"Setting default CD no-look to {time} sec"

    def update(self, ownship: Any, intruder: Any) -> None:
        """Perform an update step of the Conflict Detection implementation.

        Runs :meth:`detect` on the current traffic states and stores its
        results. Also maintains the sets of unique conflict/LoS pairs (where
        (a, b) and (b, a) count as one pair) and appends newly appearing
        pairs to the cumulative ``confpairs_all``/``lospairs_all`` lists.

        Args:
            ownship: Traffic object with the states of the ownship aircraft.
            intruder: Traffic object with the states of the intruder aircraft
                (usually the same traffic object, or an ADS-B derived copy).
        """
        if not self.activate:
            return

        (
            self.confpairs,
            self.lospairs,
            self.inconf,
            self.tcpamax,
            self.qdr,
            self.dist,
            self.dcpa,
            self.tcpa,
            self.tLOS,
            self.dalt,
        ) = self.detect(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)

        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique

    def detect(
        self,
        ownship: Any,
        intruder: Any,
        rpz: np.ndarray,
        hpz: np.ndarray,
        dtlookahead: np.ndarray,
    ) -> tuple:
        """Conflict detection between ownship (traf) and intruder (traf/adsb).

        State-based detection over all aircraft pairs at once. For every pair,
        the time to the horizontal closest point of approach (tCPA) and the
        distance at CPA are computed from the relative position and relative
        ground velocity, assuming straight-line (constant velocity)
        extrapolation. Horizontal conflict entry/exit times follow from the
        chord the relative track cuts through the protected zone circle;
        vertical entry/exit times follow from the relative vertical speed
        crossing the +/-hpz altitude band. A conflict requires the combined
        horizontal and vertical conflict intervals to overlap, end in the
        future, and start within the lookahead time. When separation minima
        differ per aircraft, the largest value of each pair is used.

        Args:
            ownship: Traffic object with ownship states (lat [deg], lon [deg],
                trk [deg], gs [m/s], alt [m], vs [m/s]).
            intruder: Traffic object with intruder states (same units).
            rpz (ndarray): Per-aircraft horizontal separation minimum [m].
            hpz (ndarray): Per-aircraft vertical separation minimum [m].
            dtlookahead (ndarray): Per-aircraft lookahead time [s].

        Returns:
            tuple: The detection results:
                - confpairs (list): Conflicting callsign pairs, both directions.
                - lospairs (list): Callsign pairs in loss of separation.
                - inconf (ndarray): Per-aircraft in-conflict flag [-].
                - tcpamax (ndarray): Per-aircraft maximum tCPA [s].
                - qdr (ndarray): Bearing ownship to intruder per conflict [deg].
                - dist (ndarray): Current horizontal distance per conflict [m].
                - dcpa (ndarray): Horizontal distance at CPA per conflict [m].
                - tcpa (ndarray): Time to CPA per conflict [s].
                - tinconf (ndarray): Time to start of LoS per conflict [s].
                - dalt (ndarray): Current altitude difference per conflict [m].
        """
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        eye = np.eye(ownship.ntraf)

        # Horizontal conflict ------------------------------------------------------

        # qdrlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates
        qdr, dist = geo.kwikqdrdist_matrix(
            np.asmatrix(ownship.lat),
            np.asmatrix(ownship.lon),
            np.asmatrix(intruder.lat),
            np.asmatrix(intruder.lon),
        )

        # Convert back to array to allow element-wise array multiplications later on
        # Convert to meters and add large value to own/own pairs
        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * eye

        # Calculate horizontal closest point of approach (CPA)
        qdrrad = np.radians(qdr)
        dx = dist * np.sin(qdrrad)  # is pos j rel to i
        dy = dist * np.cos(qdrrad)  # is pos j rel to i

        # Ownship track angle and speed
        owntrkrad = np.radians(ownship.trk)
        ownu = ownship.gs * np.sin(owntrkrad).reshape((1, ownship.ntraf))  # m/s
        ownv = ownship.gs * np.cos(owntrkrad).reshape((1, ownship.ntraf))  # m/s

        # Intruder track angle and speed
        inttrkrad = np.radians(intruder.trk)
        intu = intruder.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))  # m/s
        intv = intruder.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))  # m/s

        du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
        dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

        dv2 = du * du + dv * dv
        dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
        vrel = np.sqrt(dv2)

        tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * eye

        # Calculate distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        # RPZ can differ per aircraft, get the largest value per aircraft pair
        rpz = np.asarray(np.maximum(np.asmatrix(rpz), np.asmatrix(rpz).transpose()))
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Calculate times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(np.maximum(0.0, R2 - dcpa2))  # half the distance travelled inzide zone
        dtinhor = dxinhor / vrel

        tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
        touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = (
            ownship.alt.reshape((1, ownship.ntraf))
            - intruder.alt.reshape((1, ownship.ntraf)).T
            + 1e9 * eye
        )

        dvs = ownship.vs.reshape(1, ownship.ntraf) - intruder.vs.reshape(1, ownship.ntraf).T
        dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

        # Check for passing through each others zone
        # hPZ can differ per aircraft, get the largest value per aircraft pair
        hpz = np.asarray(np.maximum(np.asmatrix(hpz), np.asmatrix(hpz).transpose()))
        tcrosshi = (dalt + hpz) / -dvs
        tcrosslo = (dalt - hpz) / -dvs
        tinver = np.minimum(tcrosshi, tcrosslo)
        toutver = np.maximum(tcrosshi, tcrosslo)

        # Combine vertical and horizontal conflict----------------------------------
        tinconf = np.maximum(tinver, tinhor)
        toutconf = np.minimum(toutver, touthor)

        swconfl = np.array(
            swhorconf
            * (tinconf <= toutconf)
            * (toutconf > 0.0)
            * np.asarray(tinconf < np.asmatrix(dtlookahead).T)
            * (1.0 - eye),
            dtype=bool,
        )

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Ownship conflict flag and max tCPA
        inconf = np.any(swconfl, 1)
        tcpamax = np.max(tcpa * swconfl, 1)

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [
            (ownship.callsign[i], ownship.callsign[j])
            for i, j in zip(*np.where(swconfl), strict=False)
        ]
        swlos = (dist < rpz) * (np.abs(dalt) < hpz)
        lospairs = [
            (ownship.callsign[i], ownship.callsign[j])
            for i, j in zip(*np.where(swlos), strict=False)
        ]

        return (
            confpairs,
            lospairs,
            inconf,
            tcpamax,
            qdr[swconfl],
            dist[swconfl],
            np.sqrt(dcpa2[swconfl]),
            tcpa[swconfl],
            tinconf[swconfl],
            dalt[swconfl],
        )
