"""This module provides the Conflict Detection base class.

Conflict detection in MiniSky is pairwise and state-based: at every update the
current position and velocity of each aircraft (the ownship) is linearly
extrapolated and compared against every other aircraft (the intruder). A
conflict is flagged when the extrapolated trajectories penetrate each other's
cylindrical protected zone (radius `rpz`, half-height `hpz`) within the
lookahead time `dtlookahead`. A loss of separation (LoS) is flagged when the
protected zone is already penetrated at the current time.

Rather than evaluating all N^2 aircraft pairs, detection first selects
candidate pairs with a KD-tree on flat-earth-projected positions (only pairs
close enough to possibly conflict within the lookahead time) and drops
candidates that are vertically out of reach; the CPA geometry is then
computed for the remaining pairs as flat vectorised numpy arrays. Internally
SI units are used (m, m/s, s); user-facing (stack command) arguments are in
aviation units (NM, ft).
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Annotated, NamedTuple

import numpy as np
from annotated_types import Ge
from scipy.spatial import KDTree

from minisky import quantities as q
from minisky.command import (
    AcIdSelection,
    DistanceM,
    OnOff,
    TimeS,
    aircraft_indices,
    command,
)
from minisky.core.config import MiniSkyConfig
from minisky.core.trafficarrays import TrafficArrays
from minisky.result import Ok, Result

if TYPE_CHECKING:
    from minisky.traffic import Traffic

# Mean earth radius [m], same value as the geo module's flat-earth helpers
RE: q.LengthM[float] = 6371000.0

NonNegativeTime = Annotated[TimeS, Ge(0)]
ProtectedRadiusM = Annotated[DistanceM, Ge(0)]
ProtectedHeightM = Annotated[DistanceM, Ge(0)]


# TODO(abraham): model callsign pairs as a named ConflictPair record.
class ConflictDetectionResult(NamedTuple):
    confpairs: list[tuple[str, str]]
    """Conflicting callsign pairs, in both directions."""
    lospairs: list[tuple[str, str]]
    """Callsign pairs in loss of separation."""
    inconf: np.ndarray
    """Per-aircraft in-conflict flags [-]."""
    tcpamax: q.DurationS[np.ndarray]
    """Per-aircraft maximum time to closest point of approach [s]."""
    qdr: q.BearingDeg[np.ndarray]
    """Bearing from ownship to intruder per conflict [deg]."""
    dist: q.DistanceM[np.ndarray]
    """Current horizontal distance per conflict [m]."""
    dcpa: q.DistanceM[np.ndarray]
    """Horizontal distance at closest point of approach per conflict [m]."""
    tcpa: q.DurationS[np.ndarray]
    """Time to closest point of approach per conflict [s]."""
    tLOS: q.DurationS[np.ndarray]
    """Time until loss of separation per conflict [s]."""
    dalt: q.VerticalDistanceM[np.ndarray]
    """Current altitude difference per conflict [m]."""


def _noconflicts(ntraf: int) -> ConflictDetectionResult:
    """Detection result for a timestep without any conflicts or LoS."""
    empty = np.array([])
    return ConflictDetectionResult(
        [],
        [],
        np.zeros(ntraf, dtype=bool),
        np.zeros(ntraf),
        empty,
        empty,
        empty,
        empty,
        empty,
        empty,
    )


class ConflictDetection(TrafficArrays):
    """Base class for Conflict Detection implementations.

    Implements state-based conflict detection: for each aircraft pair the
    closest point of approach (CPA) is computed from relative position and
    ground velocity. A pair is in conflict when the horizontal distance at CPA
    is smaller than the protected zone radius, the vertical crossing of the
    protected zone disk overlaps in time with the horizontal intrusion, and
    the conflict starts within the lookahead time.

    The result of each update is stored both as pairwise lists (`confpairs`,
    `lospairs` and the per-conflict geometry arrays) and as per-aircraft
    arrays (`inconf`, `tcpamax`). Separation minima and lookahead time can
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

    rpz_def: q.DistanceM[float]
    hpz_def: q.VerticalDistanceM[float]
    dtlookahead_def: q.DurationS[float]
    dtnolook_def: q.DurationS[float]
    qdr: q.BearingDeg[np.ndarray]
    dist: q.DistanceM[np.ndarray]
    dcpa: q.DistanceM[np.ndarray]
    tcpa: q.DurationS[np.ndarray]
    tLOS: q.DurationS[np.ndarray]
    dalt: q.VerticalDistanceM[np.ndarray]
    tcpamax: q.DurationS[np.ndarray]
    rpz: q.DistanceM[np.ndarray]
    hpz: q.VerticalDistanceM[np.ndarray]
    dtlookahead: q.DurationS[np.ndarray]
    dtnolook: q.DurationS[np.ndarray]

    def __init__(
        self, config: MiniSkyConfig, traffic: Traffic, stack_command: Callable[..., None]
    ) -> None:
        super().__init__()
        self.config = config
        self.traffic = traffic
        self.stack_command = stack_command
        ## Default values
        # [m] Horizontal separation minimum for detection
        self.rpz_def = q.nmi_to_m(self.config.asas_pzr)
        self.global_rpz = True
        # [m] Vertical separation minimum for detection
        self.hpz_def = q.ft_to_m(self.config.asas_pzh)
        self.global_hpz = True
        # [s] lookahead time
        self.dtlookahead_def = self.config.asas_dtlookahead
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

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and command stack."""
        return implementation(self.config, self.traffic, self.stack_command)

    def clearconfdb(self) -> None:
        """Clear the conflict database.

        Empties the pairwise conflict/LoS lists and geometry arrays of the
        current timestep and resets the per-aircraft conflict flags. The
        historic lists (`confpairs_all`, `lospairs_all`) are kept.
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
        self.inconf = np.zeros(self.traffic.ntraf)
        self.tcpamax = np.zeros(self.traffic.ntraf)

    def create(self, n: int = 1) -> None:
        """Initialise per-aircraft detection parameters for new aircraft.

        Called by the traffic object when aircraft are created. Extends all
        per-aircraft arrays and fills the last `n` elements with the current
        default separation minima and lookahead times.

        Args:
            n: Number of newly created aircraft.
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
        minima and lookahead times from the simulation config.
        """
        super().reset()
        self.clearconfdb()
        self.confpairs_all.clear()
        self.lospairs_all.clear()
        self.rpz_def = q.nmi_to_m(self.config.asas_pzr)
        self.hpz_def = q.ft_to_m(self.config.asas_pzh)
        self.dtlookahead_def = self.config.asas_dtlookahead
        self.dtnolook_def = 0.0
        self.global_rpz = self.global_hpz = True
        self.global_dtlook = self.global_dtnolook = True

    def _set_detection(self, enabled: bool) -> Result[str, str]:
        if not enabled:
            self.clearconfdb()
            self.activate = False
            return Ok("Conflict Detection turned off.")
        self.activate = True
        return Ok("Conflict Detection is on.")

    @command(name="ASAS", aliases=("CD", "CDMETHOD"))
    def enable_detection(self) -> Result[str, str]:
        """Enable conflict detection."""
        return self._set_detection(True)

    @command(name="ASAS")
    def set_detection(self, enabled: OnOff) -> Result[str, str]:
        """Enable or disable conflict detection."""
        return self._set_detection(enabled)

    @command(name="ZONER", aliases=("PZR", "RPZ", "PZRADIUS"))
    def protected_zone_radius(self) -> Result[str, str]:
        """Report the default protected-zone radius."""
        return Ok(
            f"ZONER [radius, acid(s)/ac group], e.g. ZONER 5NM\nCurrent default PZ radius: {q.m_to_nmi(self.rpz_def):.2f} NM"
        )

    @command(name="ZONER")
    def set_protected_zone_radius(self, radius: ProtectedRadiusM) -> Result[str, str]:
        """Set the default protected-zone radius."""
        oldradius = self.rpz_def
        self.rpz_def = radius
        if self.global_rpz:
            self.rpz[:] = self.rpz_def
        # Preserve an absolute resolution zone if it was configured before the detection zone changed.
        if not self.traffic.cr.resorrelative:
            self.stack_command(f"RSZONER {q.m_to_nmi(self.traffic.cr.resofach * oldradius)}NM")
        return Ok(f"Setting default PZ radius to {q.m_to_nmi(radius)} NM")

    @command(name="ZONER")
    def set_aircraft_protected_zone_radius(
        self, radius: ProtectedRadiusM, first: AcIdSelection, *additional: AcIdSelection
    ) -> Result[str, str]:
        """Set the protected-zone radius for selected aircraft."""
        idx = aircraft_indices((first, *additional))
        self.rpz[idx] = radius
        self.global_rpz = False
        return Ok(f"Setting PZ radius to {q.m_to_nmi(radius)} NM for {len(idx)} aircraft")

    @command(name="ZONEDH", aliases=("PZDH", "DHPZ", "PZHEIGHT"))
    def protected_zone_height(self) -> Result[str, str]:
        """Report the default protected-zone half-height."""
        return Ok(
            f"ZONEDH [height, acid(s)/ac group]\nCurrent default PZ height: {q.m_to_ft(self.hpz_def):.2f} ft"
        )

    @command(name="ZONEDH")
    def set_protected_zone_height(self, height: ProtectedHeightM) -> Result[str, str]:
        """Set the default protected-zone half-height."""
        oldhpz = self.hpz_def
        self.hpz_def = height
        if self.global_hpz:
            self.hpz[:] = self.hpz_def
        # Adjust factors for reso zone if those were set with an absolute value
        if not self.traffic.cr.resodhrelative:
            self.stack_command(f"RSZONEDH {q.m_to_ft(self.traffic.cr.resofacv * oldhpz)}FT")
        return Ok(f"Setting default PZ height to {q.m_to_ft(height)} ft")

    @command(name="ZONEDH")
    def set_aircraft_protected_zone_height(
        self, height: ProtectedHeightM, first: AcIdSelection, *additional: AcIdSelection
    ) -> Result[str, str]:
        """Set the protected-zone half-height for selected aircraft."""
        idx = aircraft_indices((first, *additional))
        self.hpz[idx] = height
        self.global_hpz = False
        return Ok(f"Setting PZ height to {q.m_to_ft(height)} ft for {len(idx)} aircraft")

    @command(name="DTLOOK")
    def detection_lookahead(self) -> Result[str, str]:
        """Report the default conflict-detection lookahead."""
        return Ok(f"DTLOOK[time]\nCurrent value: {self.dtlookahead_def: .1f} sec")

    @command(name="DTLOOK")
    def set_detection_lookahead(self, time: NonNegativeTime) -> Result[str, str]:
        """Set the default conflict-detection lookahead."""
        self.dtlookahead_def = time
        if self.global_dtlook:
            self.dtlookahead[:] = time
        return Ok(f"Setting default CD lookahead to {time} sec")

    @command(name="DTLOOK")
    def set_aircraft_detection_lookahead(
        self, time: NonNegativeTime, first: AcIdSelection, *additional: AcIdSelection
    ) -> Result[str, str]:
        """Set conflict-detection lookahead for selected aircraft."""
        idx = aircraft_indices((first, *additional))
        self.dtlookahead[idx] = time
        self.global_dtlook = False
        return Ok(f"Setting CD lookahead to {time} sec for {len(idx)} aircraft")

    @command(name="DTNOLOOK")
    def detection_no_look(self) -> Result[str, str]:
        """Report the default post-resolution no-look interval."""
        return Ok(f"DTNOLOOK[time]\nCurrent value: {self.dtnolook_def: .1f} sec")

    @command(name="DTNOLOOK")
    def set_detection_no_look(self, time: NonNegativeTime) -> Result[str, str]:
        """Set the default post-resolution no-look interval."""
        self.dtnolook_def = time
        if self.global_dtnolook:
            self.dtnolook[:] = time
        return Ok(f"Setting default CD no-look to {time} sec")

    @command(name="DTNOLOOK")
    def set_aircraft_detection_no_look(
        self, time: NonNegativeTime, first: AcIdSelection, *additional: AcIdSelection
    ) -> Result[str, str]:
        """Set the post-resolution no-look interval for selected aircraft."""
        idx = aircraft_indices((first, *additional))
        self.dtnolook[idx] = time
        self.global_dtnolook = False
        return Ok(f"Setting CD no-look to {time} sec for {len(idx)} aircraft")

    def update(self, ownship: Traffic, intruder: Traffic) -> None:
        """Perform an update step of the Conflict Detection implementation.

        Runs [`ConflictDetection.detect`][minisky.traffic.asas.detection.ConflictDetection.detect] on the current traffic states and stores its
        results. Also maintains the sets of unique conflict/LoS pairs (where
        (a, b) and (b, a) count as one pair) and appends newly appearing
        pairs to the cumulative `confpairs_all`/`lospairs_all` lists.

        Args:
            ownship: Traffic state used as the ownship side of each candidate pair.
            intruder: Intruder traffic state; normally the same object, but may be an ADS-B-derived copy.
        """
        if not self.activate:
            return

        result = self.detect(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)
        # TODO(abraham): consider storing the entire result
        self.confpairs = result.confpairs
        self.lospairs = result.lospairs
        self.inconf = result.inconf
        self.tcpamax = result.tcpamax
        self.qdr = result.qdr
        self.dist = result.dist
        self.dcpa = result.dcpa
        self.tcpa = result.tcpa
        self.tLOS = result.tLOS
        self.dalt = result.dalt

        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique

    class VerticalInterval(NamedTuple):
        entry: q.DurationS[np.ndarray]
        exit: q.DurationS[np.ndarray]

    def detect(
        self,
        ownship: Traffic,
        intruder: Traffic,
        rpz: q.DistanceM[np.ndarray],
        hpz: q.VerticalDistanceM[np.ndarray],
        dtlookahead: q.DurationS[np.ndarray],
    ) -> ConflictDetectionResult:
        """Conflict detection between ownship (traf) and intruder (traf/adsb).

        State-based detection with spatial candidate pruning: a KD-tree on
        flat-earth-projected positions selects the pairs within horizontal
        reach (`max(rpz) + 2 * max(gs) * max(dtlookahead)`), pairs that are
        vertically out of reach within the lookahead are dropped, and the CPA
        geometry is evaluated only for the remaining candidates. For every
        candidate pair, the time to the horizontal closest point of approach
        (tCPA) and the distance at CPA are computed from the relative
        position and relative ground velocity, assuming straight-line
        (constant velocity) extrapolation. Horizontal conflict entry/exit times follow from the
        chord the relative track cuts through the protected zone circle;
        vertical entry/exit times follow from the relative vertical speed
        crossing the +/-hpz altitude band. A conflict requires the combined
        horizontal and vertical conflict intervals to overlap, end in the
        future, and start within the lookahead time. When separation minima
        differ per aircraft, the largest value of each pair is used.

        Args:
            ownship: Ownship traffic state.
            intruder: Intruder traffic state, which may come from surveillance.
            rpz: Per-aircraft horizontal separation minimum.
            hpz: Per-aircraft vertical separation minimum.
            dtlookahead: Per-aircraft conflict lookahead time.
        """
        ntraf = ownship.ntraf
        if ntraf < 2:
            return _noconflicts(ntraf)

        # Candidate selection ------------------------------------------------------

        # Flat-earth projection for the spatial index (same mean earth radius
        # as geo.kwikqdrdist_matrix). A single reference cos(lat) keeps the
        # projection consistent; the query radius is inflated by the
        # worst-case cos(lat) ratio over the traffic extent, so the candidate
        # set is a superset of all pairs that can possibly conflict. Assumes
        # traffic does not straddle the antimeridian or sit at the poles.
        lat = ownship.lat
        lon = ownship.lon
        coslat = np.cos(np.radians(lat))
        x = RE * np.radians(lon) * np.mean(coslat)
        y = RE * np.radians(lat)

        # Farthest horizontal distance at which a conflict (or LoS) within
        # the lookahead is geometrically possible
        rmax = np.max(rpz) + 2.0 * np.max(ownship.gs) * np.max(dtlookahead)
        rquery = rmax * (np.max(coslat) / max(np.min(coslat), 1e-9))

        tree = KDTree(np.column_stack((x, y)))
        pairs = tree.query_pairs(rquery, output_type="ndarray")
        if len(pairs) == 0:
            return _noconflicts(ntraf)
        ii, jj = pairs[:, 0], pairs[:, 1]  # candidate pairs, ii < jj

        # Vertical pre-filter: a pair separated by more than
        # hpz + |dvs| * dtlookahead can neither conflict nor lose separation
        # within the lookahead
        dalt = intruder.alt[jj] - ownship.alt[ii]
        dvs = np.abs(intruder.vs[jj] - ownship.vs[ii])
        hpz = np.maximum(hpz[ii], hpz[jj])
        dtl = np.maximum(dtlookahead[ii], dtlookahead[jj])
        keep = np.abs(dalt) <= hpz + np.maximum(dvs, 1e-6) * dtl
        if not np.all(keep):
            ii, jj = ii[keep], jj[keep]
            dalt, hpz = dalt[keep], hpz[keep]
            if len(ii) == 0:
                return _noconflicts(ntraf)

        # Horizontal conflict ------------------------------------------------------

        # Flat-earth offsets per candidate pair; d* is the state of j
        # relative to i (same formulation as geo.kwikqdrdist_matrix)
        dlatrad = np.radians(intruder.lat[jj] - ownship.lat[ii])
        dlonrad = np.radians(((intruder.lon[jj] - ownship.lon[ii]) + 180.0) % 360.0 - 180.0)
        cavelat = np.cos(np.radians(intruder.lat[jj] + ownship.lat[ii]) * 0.5)
        dx = RE * dlonrad * cavelat
        dy = RE * dlatrad
        dist = np.sqrt(dx * dx + dy * dy)

        # Ground velocity components; du/dv is the velocity of j relative to i
        owntrkrad = np.radians(ownship.trk)
        inttrkrad = np.radians(intruder.trk)
        du = intruder.gs[jj] * np.sin(inttrkrad[jj]) - ownship.gs[ii] * np.sin(owntrkrad[ii])
        dv = intruder.gs[jj] * np.cos(inttrkrad[jj]) - ownship.gs[ii] * np.cos(owntrkrad[ii])

        dv2 = du * du + dv * dv
        dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
        vrel = np.sqrt(dv2)

        # Horizontal closest point of approach (CPA)
        tcpa = -(du * dx + dv * dy) / dv2

        # Distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        # RPZ can differ per aircraft, get the largest value per aircraft pair
        rpz = np.maximum(rpz[ii], rpz[jj])
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(np.maximum(0.0, R2 - dcpa2))  # half the distance travelled inside zone
        dtinhor = dxinhor / vrel

        tinhor = tcpa - dtinhor
        touthor = tcpa + dtinhor

        # Vertical conflict --------------------------------------------------------

        # The vertical test is evaluated for both directions of each pair:
        # |dvs| is floored to +1e-6 irrespective of its sign, which makes the
        # crossing times of the +/-hpz band direction-asymmetric for level
        # pairs at exactly |dalt| == hpz (only the direction looking "down"
        # flags a conflict). The horizontal geometry is fully symmetric.
        dvs = intruder.vs[jj] - ownship.vs[ii]

        def vertical_interval(da: np.ndarray, dw: np.ndarray) -> ConflictDetection.VerticalInterval:
            """Vertical crossing interval of the disk (-hpz, +hpz)."""
            dw = np.where(np.abs(dw) < 1e-6, 1e-6, dw)  # prevent division by zero
            tcrosshi = (da + hpz) / -dw
            tcrosslo = (da - hpz) / -dw
            return self.VerticalInterval(
                np.minimum(tcrosshi, tcrosslo), np.maximum(tcrosshi, tcrosslo)
            )

        tinver_ij, toutver_ij = vertical_interval(dalt, dvs)
        tinver_ji, toutver_ji = vertical_interval(-dalt, -dvs)

        # Combine vertical and horizontal conflict----------------------------------
        tinconf_ij = np.maximum(tinver_ij, tinhor)
        toutconf_ij = np.minimum(toutver_ij, touthor)
        tinconf_ji = np.maximum(tinver_ji, tinhor)
        toutconf_ji = np.minimum(toutver_ji, touthor)

        # The lookahead time is the ownship's, so it also differs per direction
        sw_ij = (
            swhorconf
            & (tinconf_ij <= toutconf_ij)
            & (toutconf_ij > 0.0)
            & (tinconf_ij < dtlookahead[ii])
        )
        sw_ji = (
            swhorconf
            & (tinconf_ji <= toutconf_ji)
            & (toutconf_ji > 0.0)
            & (tinconf_ji < dtlookahead[jj])
        )

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Assemble both directions of each conflict and sort them into
        # row-major (ownship, intruder) index order
        iown = np.concatenate((ii[sw_ij], jj[sw_ji]))
        jint = np.concatenate((jj[sw_ij], ii[sw_ji]))
        qdr_ij = np.degrees(np.arctan2(dx, dy)) % 360.0
        qdr_ji = np.degrees(np.arctan2(-dx, -dy)) % 360.0
        qdrconf = np.concatenate((qdr_ij[sw_ij], qdr_ji[sw_ji]))
        distconf = np.concatenate((dist[sw_ij], dist[sw_ji]))
        dcpaconf = np.sqrt(np.concatenate((dcpa2[sw_ij], dcpa2[sw_ji])))
        tcpaconf = np.concatenate((tcpa[sw_ij], tcpa[sw_ji]))
        tinconfconf = np.concatenate((tinconf_ij[sw_ij], tinconf_ji[sw_ji]))
        daltconf = np.concatenate((dalt[sw_ij], -dalt[sw_ji]))

        order = np.lexsort((jint, iown))
        iown, jint = iown[order], jint[order]

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [
            (ownship.callsign[i], ownship.callsign[j]) for i, j in zip(iown, jint, strict=False)
        ]

        # Ownship conflict flag and max tCPA
        inconf = np.zeros(ntraf, dtype=bool)
        inconf[iown] = True
        tcpamax = np.zeros(ntraf)
        np.maximum.at(tcpamax, iown, tcpaconf[order])

        swlos = (dist < rpz) & (np.abs(dalt) < hpz)
        ilos = np.concatenate((ii[swlos], jj[swlos]))
        jlos = np.concatenate((jj[swlos], ii[swlos]))
        losorder = np.lexsort((jlos, ilos))
        lospairs = [
            (ownship.callsign[i], ownship.callsign[j])
            for i, j in zip(ilos[losorder], jlos[losorder], strict=False)
        ]

        return ConflictDetectionResult(
            confpairs,
            lospairs,
            inconf,
            tcpamax,
            qdrconf[order],
            distconf[order],
            dcpaconf[order],
            tcpaconf[order],
            tinconfconf[order],
            daltconf[order],
        )
