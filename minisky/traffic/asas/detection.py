"""This module provides the Conflict Detection base class."""

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools import geo
from minisky.tools.aero import ft, nm


class ConflictDetection(TrafficArrays):
    """Base class for Conflict Detection implementations."""

    def __init__(self):
        super().__init__()
        ## Default values
        # [m] Horizontal separation minimum for detection
        self.rpz_def = minisky.settings.asas_pzr * nm
        self.global_rpz = True
        # [m] Vertical separation minimum for detection
        self.hpz_def = minisky.settings.asas_pzh * ft
        self.global_hpz = True
        # [s] lookahead time
        self.dtlookahead_def = minisky.settings.asas_dtlookahead
        self.global_dtlook = True
        self.dtnolook_def = 0.0
        self.global_dtnolook = True

        # Conflicts and LoS detected in the current timestep (used for resolving)
        self.confpairs = list()
        self.lospairs = list()
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
        self.confpairs_all = list()
        self.lospairs_all = list()

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

    def clearconfdb(self):
        """Clear conflict database."""
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

    def create(self, n):
        super().create(n)
        # Initialise values of own states
        self.rpz[-n:] = self.rpz_def
        self.hpz[-n:] = self.hpz_def
        self.dtlookahead[-n:] = self.dtlookahead_def
        self.dtnolook[-n:] = self.dtnolook_def

    def reset(self):
        super().reset()
        self.clearconfdb()
        self.confpairs_all.clear()
        self.lospairs_all.clear()
        self.rpz_def = minisky.settings.asas_pzr * nm
        self.hpz_def = (minisky.settings.asas_pzh - 1) * ft  # -1 for rounding margins
        self.dtlookahead_def = minisky.settings.asas_dtlookahead
        self.dtnolook_def = 0.0
        self.global_rpz = self.global_hpz = True
        self.global_dtlook = self.global_dtnolook = True

    def switch(self, name: "txt" = "ON"):
        """Turn Conflict Detection (CD) ON / OFF."""
        assert name in ["ON", "OFF"], f"Invalid CD method: {name}"

        if name == "OFF":
            self.clearconfdb()
            self.active = False
            return True, "Conflict Detection turned off."

        if name == "ON":
            self.active = True
            return True, "Conflict Detection is on."

    def setrpz(self, radius: float = -1.0, *acidx: "acid"):
        """Set the horizontal separation distance (i.e., the radius of the
        protected zone) in nautical miles.

        Arguments:
        - radius: The protected zone radius in nautical miles
        - acidx: Aircraft id(s) or group. When this argument is not provided the default PZ radius is changed.
          Otherwise the PZ radius for the passed aircraft is changed."""
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

    def sethpz(self, height: float = -1.0, *acidx: "acid"):
        """Set the vertical separation distance (i.e., half of the protected
        zone height) in feet.

        Arguments:
        - height: The vertical separation height in feet
        - acidx: Aircraft id(s) or group. When this argument is not provided the default PZ height is changed.
          Otherwise the PZ height for the passed aircraft is changed."""
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

    def setdtlook(self, time: "time" = -1.0, *acidx: "acid"):
        """Set the lookahead time (in [hh:mm:]sec) for conflict detection."""
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

    def setdtnolook(self, time: "time" = -1.0, *acidx: "acid"):
        """Set the interval (in [hh:mm:]sec) in which conflict detection
        is skipped after a conflict resolution."""
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

    def update(self, ownship, intruder):
        """Perform an update step of the Conflict Detection implementation."""
        if not self.active:
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

    def detect(self, ownship, intruder, rpz, hpz, dtlookahead):
        """Conflict detection between ownship (traf) and intruder (traf/adsb)."""
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        I = np.eye(ownship.ntraf)

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
        dist = np.asarray(dist) * nm + 1e9 * I

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

        tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

        # Calculate distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        # RPZ can differ per aircraft, get the largest value per aircraft pair
        rpz = np.asarray(np.maximum(np.asmatrix(rpz), np.asmatrix(rpz).transpose()))
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Calculate times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(
            np.maximum(0.0, R2 - dcpa2)
        )  # half the distance travelled inzide zone
        dtinhor = dxinhor / vrel

        tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
        touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = (
            ownship.alt.reshape((1, ownship.ntraf))
            - intruder.alt.reshape((1, ownship.ntraf)).T
            + 1e9 * I
        )

        dvs = (
            ownship.vs.reshape(1, ownship.ntraf)
            - intruder.vs.reshape(1, ownship.ntraf).T
        )
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
            * (1.0 - I),
            dtype=bool,
        )

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Ownship conflict flag and max tCPA
        inconf = np.any(swconfl, 1)
        tcpamax = np.max(tcpa * swconfl, 1)

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swconfl))]
        swlos = (dist < rpz) * (np.abs(dalt) < hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

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
